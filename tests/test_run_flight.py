"""Launcher starts the recorder first and drains it after other processes."""
from importlib.util import spec_from_file_location, module_from_spec
from io import StringIO
from pathlib import Path
from types import SimpleNamespace
import json

import pytest


def test_recorded_launcher_order_logs_and_shutdown(tmp_path,monkeypatch):
    spec=spec_from_file_location('run_flight',Path(__file__).resolve().parents[1]/'tools/run_flight.py')
    module=module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module.YamlConfigLoader, "load", lambda self, path: SimpleNamespace(control=SimpleNamespace(backend="internal")))
    monkeypatch.setattr(module,'ROOT',tmp_path/'bullet_render')
    app_config=tmp_path/'app.json'
    app_config.write_text(json.dumps({'recording':{'output_dir':'mcaps'}}))
    override=tmp_path/'custom MCAP output'
    mission_config=tmp_path/'betaflight.yaml'
    monkeypatch.setattr(module.sys,'argv',['run_flight','--app-config',str(app_config),'--mcap-output-dir',str(override),
                                           '--mission-config',str(mission_config)])
    monkeypatch.setattr(module.signal,'signal',lambda *args:None)
    monkeypatch.setattr(module.time,'sleep',lambda *args:None)
    launched=[]
    killed=[]
    class Process:
        def __init__(self,command,**kwargs):
            self.pid=100+len(launched)
            self.stdout=StringIO('startup diagnostic\n')
            self.returncode=1 if 'examples.external_rc.main' in command else None
            launched.append((command,kwargs))
            if '--ready-file' in command:
                Path(command[command.index('--ready-file')+1]).write_text('ready')
        def poll(self): return self.returncode
        def wait(self,timeout=None): self.returncode=0
    monkeypatch.setattr(module.subprocess,'Popen',Process)
    monkeypatch.setattr(module.os,'killpg',lambda pid,sig:killed.append(pid))
    assert module.main()==1
    assert 'mcap_bagger.cli' in launched[0][0]
    command=launched[0][0]
    assert command[command.index('--output-dir')+1]==str(override)
    app_command=next(command for command,_ in launched if '--debug' in command)
    assert app_command[app_command.index('--mission-config')+1]==str(mission_config)
    session=next((tmp_path/'bullet_render/recordings').iterdir())
    assert all((session/f'{name}.log').read_text()=='startup diagnostic\n' for name in ('recorder','godot','physics','app'))
    assert launched[0][1]['env']['BULLET_LAUNCH_ID']==session.name
    assert killed[-1]==100


@pytest.mark.parametrize('failure',['directory','config'])
def test_recorder_or_config_failure_never_launches_flight_services(tmp_path,monkeypatch,failure):
    spec=spec_from_file_location('run_flight',Path(__file__).resolve().parents[1]/'tools/run_flight.py')
    module=module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module.YamlConfigLoader, "load", lambda self, path: SimpleNamespace(control=SimpleNamespace(backend="internal")))
    monkeypatch.setattr(module,'ROOT',tmp_path/'bullet_render')
    config=tmp_path/'app.json'
    config.write_text('{bad json' if failure=='config' else json.dumps({'recording':{'output_dir':'blocked'}}))
    monkeypatch.setattr(module.sys,'argv',['run_flight','--app-config',str(config)])
    monkeypatch.setattr(module.signal,'signal',lambda *args:None)
    monkeypatch.setattr(module.time,'sleep',lambda *args:None)
    calls=[]
    class FailedRecorder:
        pid=100
        returncode=1
        stdout=StringIO('Cannot write MCAP output directory: denied\n')
        def __init__(self,command,**kwargs): calls.append(command)
        def poll(self): return self.returncode
        def wait(self,timeout=None): return self.returncode
    monkeypatch.setattr(module.subprocess,'Popen',FailedRecorder)
    assert module.main()==1
    assert len(calls)==(1 if failure=='directory' else 0)
    if calls:
        assert 'mcap_bagger.cli' in calls[0]
        assert calls[0][calls[0].index('--output-dir')+1]==str(tmp_path/'blocked')
    assert not any('examples.external_rc.main' in c or '--debug' in c or '--path' in c for c in calls)
