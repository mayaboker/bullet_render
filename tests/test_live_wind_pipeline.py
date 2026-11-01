"""Native Godot request -> actual external-RC physics -> authoritative native readback."""
import json
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]


def test_native_gui_request_updates_real_physics(tmp_path):
    sockets=[socket.socket() for _ in range(4)]
    try:
        for item in sockets:
            item.bind(('127.0.0.1',0))
        ports=[item.getsockname()[1] for item in sockets]
    finally:
        for item in sockets:
            item.close()
    transport=dict(zip(['telemetry_endpoint','image_endpoint','control_endpoint','flight_command_endpoint'],
                       [f'tcp://127.0.0.1:{p}' for p in ports]))
    profile=tmp_path/'transport.json'
    profile.write_text(json.dumps(transport))
    render=tmp_path/'render.json'
    render.write_text(json.dumps(dict(transport_config='transport.json',fps=30,camera=dict(width_px=2,height_px=2,fov_x_deg=90))))
    processes=[]
    native_log=tmp_path/'native.log'
    with (tmp_path/'physics.log').open('w') as plog,native_log.open('w') as glog:
        try:
            processes.append(subprocess.Popen([sys.executable,'-m','examples.external_rc.main','--render-config',str(render)],cwd=ROOT.parent/'bullet_drone',stdout=plog,stderr=subprocess.STDOUT))
            processes.append(subprocess.Popen(['godot','--headless','--path',str(ROOT/'godot'),'--script','adapter_probe.gd','--',str(profile),'unused','0','0','wind_edit'],stdout=glog,stderr=subprocess.STDOUT))
            applied=None
            deadline=time.monotonic()+12
            while time.monotonic()<deadline and applied is None:
                for line in native_log.read_text().splitlines():
                    if line.startswith('PROBE '):
                        wind=json.loads(line[6:]).get('wind_state',{})
                        if wind.get('result')=='applied':
                            applied=wind
                if applied is None:
                    assert all(p.poll() is None for p in processes), native_log.read_text()+(tmp_path/'physics.log').read_text()
                    time.sleep(.02)
            assert applied is not None, native_log.read_text()+(tmp_path/'physics.log').read_text()
            assert applied['settings']['velocity_world_m_s']==[3.,-1.,0.]
            assert applied['velocity_world_m_s']==[3.,-1.,0.]
            assert applied['simulation_time_s']==0. # A stopped Apply must not step physics.
        finally:
            for process in processes:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
            for session in re.findall(r'SHM publisher session: ([0-9a-f]{32})',native_log.read_text()):
                for index in range(3):
                    (Path('/dev/shm')/f'bullet_msgs_{session}_s{index}').unlink(missing_ok=True)
