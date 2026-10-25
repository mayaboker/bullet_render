"""Launch the flight services with an episode recorder and durable process logs."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from bullet_app.recording_config import DEFAULT_APP_CONFIG, load_recording_output

ROOT = Path(__file__).resolve().parents[1]
READY_TIMEOUT_S = 15
SHUTDOWN_TIMEOUT_S = 10
LOG_LOCK = threading.Lock()


def copy_log(pipe, path, name):
    with path.open('w', buffering=1) as log:
        for line in pipe:
            log.write(line)
            with LOG_LOCK:
                print(f'[{name}] {line}', end='', flush=True)
    pipe.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--app-config', type=Path, default=DEFAULT_APP_CONFIG, help='App JSON configuration, including recording.output_dir')
    parser.add_argument('--mcap-output-dir', help='Override MCAP directory for this launch; relative to the working directory')
    args = parser.parse_args()
    session = ROOT / 'recordings' / ('launch_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    session.mkdir(parents=True)
    environment = dict(os.environ, PYTHONUNBUFFERED='1', BULLET_LAUNCH_ID=session.name)
    processes, threads = [], []
    def launch(name, command, cwd):
        process = subprocess.Popen(command, cwd=cwd, env=environment, start_new_session=True,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors='replace')
        processes.append((name, process))
        thread = threading.Thread(target=copy_log, args=(process.stdout, session / (name + '.log'), name))
        threads.append(thread)
        thread.start()
        return process
    def stop_signal(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop_signal)
    signal.signal(signal.SIGINT, stop_signal)
    result = 0
    try:
        print(f'Flight process logs: {session}', flush=True)
        output_dir = load_recording_output(args.app_config, output_dir=args.mcap_output_dir)
        print(f'MCAP output directory: {output_dir}', flush=True)
        recorder = launch('recorder', [sys.executable, '-m', 'mcap_bagger.cli', '--config', str(ROOT / 'configs/recorder.yaml'), '--output-dir', str(output_dir), '--ready-file', str(session / 'recorder.ready')], ROOT.parent / 'mcap_bagger')
        deadline = time.monotonic() + READY_TIMEOUT_S
        while not (session / 'recorder.ready').exists():
            if recorder.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError('Recorder did not become ready; inspect recorder.log')
            time.sleep(.05)
        launch('godot', [str(ROOT / 'tools/run_godot.sh'), '--path', str(ROOT / 'godot')], ROOT)
        launch('physics', [sys.executable, '-m', 'examples.external_rc.main', '--config', str(ROOT.parent / 'bullet_drone/configs/basic.yaml'), '--render-config', str(ROOT / 'configs/physics.json')], ROOT.parent / 'bullet_drone')
        launch('app', [str(Path(sys.executable).parent / 'bullet-app'), '--transport-config', str(ROOT / 'configs/transport.json'), '--mission-config', str(ROOT.parent / 'los_tracking/examples/bullet_profile.json'), '--debug'], ROOT.parent / 'bullet_app')
        (session / 'processes.json').write_text(json.dumps({name: process.pid for name, process in processes}, indent=2))
        while all(process.poll() is None for _, process in processes):
            time.sleep(.1)
        result = next((p.returncode for _, p in processes if p.returncode), 0)
    except KeyboardInterrupt:
        result = 130
    except Exception as error:
        print(str(error), file=sys.stderr, flush=True)
        result = 1
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        # Keep the recorder alive while flight workers emit their final diagnostics.
        for name, process in reversed(processes):
            if name != 'recorder' and process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
        for name, process in reversed(processes):
            if name == 'recorder':
                time.sleep(.3)
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(SHUTDOWN_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for thread in threads:
            thread.join()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
