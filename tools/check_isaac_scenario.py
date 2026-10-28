"""Displayed Isaac contact/spawn/MCAP checks on isolated ports.

Motion is deterministic test data; geometry, lifecycle, app and recording are real.
"""
import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from mcap.reader import make_reader
import yaml
import zmq

ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT.parent / '.venv/bin'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report-dir', type=Path, default=Path('/tmp/bullet-isaac-scenario'))
    parser.add_argument('--reject-collisions', action='store_true')
    args = parser.parse_args()
    args.report_dir.mkdir(parents=True, exist_ok=True)
    processes = []
    report = dict(passed=False)
    try:
        with tempfile.TemporaryDirectory(prefix='isaac-scenario-') as directory, ExitStack() as stack, zmq.Context() as context:
            temp = Path(directory)
            reservations = [context.socket(zmq.PUB) for _ in range(5)]
            endpoints = [f'tcp://127.0.0.1:{s.bind_to_random_port("tcp://127.0.0.1")}' for s in reservations]
            for socket in reservations:
                socket.close(0)
            transport = dict(zip(('telemetry_endpoint', 'image_endpoint', 'control_endpoint', 'flight_command_endpoint'), endpoints))
            (temp / 'transport.json').write_text(json.dumps(transport))
            profile = json.loads((ROOT / 'configs/physics.json').read_text())
            profile['transport_config'] = str(temp / 'transport.json')
            (temp / 'physics.json').write_text(json.dumps(profile))
            (temp / 'trajectory.json').write_text(json.dumps([[35, 0, 2], [45, 0, 2]]))
            recorder = yaml.safe_load((ROOT / 'configs/recorder.yaml').read_text())
            recorder.update(transport_config=str(temp / 'transport.json'), output_dir=str(temp / 'recordings'))
            for topic in recorder['topics']:
                if topic['key'] == 'render/drone_1/collision':
                    topic['endpoint'] = endpoints[2]
                elif topic['key'] == 'flight/drone_1/rc':
                    topic['endpoint'] = endpoints[3]
                elif 'endpoint' in topic:
                    topic['endpoint'] = endpoints[4]
            (temp / 'recorder.yaml').write_text(yaml.safe_dump(recorder))
            command_path = temp / 'command.json'
            state_path = temp / 'isaac_state.json'
            sequence = 0
            latest = {}
            sub = stack.enter_context(context.socket(zmq.SUB))
            sub.setsockopt(zmq.LINGER, 0)
            sub.setsockopt(zmq.SUBSCRIBE, b'')
            sub.connect(endpoints[0])
            sub.connect(endpoints[4])

            def launch(name, command, cwd=ROOT):
                log = stack.enter_context((args.report_dir / f'{name}.log').open('w'))
                p = subprocess.Popen(command, cwd=cwd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                processes.append(p)
                return p

            def snapshot():
                return json.loads(state_path.read_text()) if state_path.exists() else {}

            def pump():
                for _ in range(512):
                    try:
                        parts = sub.recv_multipart(zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    if len(parts) == 2:
                        latest[parts[0]] = json.loads(parts[1])

            def wait(predicate, description, timeout=20):
                deadline = time.monotonic() + timeout
                while True:
                    pump()
                    if predicate():
                        return
                    assert all(p.poll() is None for p in processes), f'component exited during {description}; see {args.report_dir}'
                    assert time.monotonic() < deadline, f'timeout: {description}; snapshot={snapshot()}; mission={latest.get(b"app/drone_1/mission")}'
                    time.sleep(.02)

            def command(action, **fields):
                nonlocal sequence
                sequence += 1
                pending = command_path.with_suffix('.tmp')
                pending.write_text(json.dumps(dict(sequence=sequence, action=action, **fields)))
                pending.replace(command_path)
                wait(lambda: snapshot().get('command_sequence', 0) >= sequence, f'probe accepted {action}')

            launch('recorder', [str(BIN / 'mcap-bagger'), '--config', str(temp / 'recorder.yaml')])
            launch('physics', [str(BIN / 'python'), str(ROOT / 'tools/collision_physics_probe.py'),
                str(temp / 'physics.json'), str(temp / 'trajectory.json'), '--respect-spawn',
                *(['--reject-collisions'] if args.reject_collisions else [])], ROOT.parent / 'bullet_drone')
            app = launch('app', [str(BIN / 'bullet-app'), '--transport-config', str(temp / 'transport.json'),
                '--bbox-endpoint', endpoints[4], '--mission-config', str(ROOT.parent / 'los_tracking/examples/bullet_profile.json'), '--debug'])
            launch('isaac', ['/home/user/isaacsim5.0/python.sh', str(ROOT / 'tools/isaac_control_probe.py'),
                str(command_path), '--transport-config', str(temp / 'transport.json')])
            wait(lambda: snapshot().get('editable') and latest.get(b'app/drone_1/mission', {}).get('phase') == 'READY', 'initial ready', 180)
            assert snapshot()['marker']
            command('apply_spawn', position_m=[35., 0., 2.], takeoff_height_m=6.)
            wait(lambda: (snapshot().get('spawn') or {}).get('result') == 'applied' and not snapshot().get('pending'), 'spawn/height confirmed')
            assert snapshot()['spawn']['position_m'] == [35., 0., 2.]
            assert snapshot()['spawn']['takeoff_height_m'] == 6
            wait(lambda: snapshot().get('position') == [35., 0., 2.], 'actual pose follows applied spawn')
            wait(lambda: latest.get(b'flight/drone_1/feedback', {}).get('takeoff_height_m') == 6, 'height reaches feedback')
            command('focus')
            screenshot = args.report_dir / 'scenario-controls.png'
            command('screenshot', path=str(screenshot))
            wait(screenshot.exists, 'controls/marker screenshot')
            command('start')
            if args.reject_collisions:
                wait(lambda: 'acknowledgement failed' in snapshot().get('contact', '').lower(), 'contact timeout')
                wait(lambda: latest.get(b'flight/drone_1/feedback', {}).get('state') == 'stopped', 'ordinary Stop after timeout')
                assert latest[b'flight/drone_1/feedback']['reason'] == 'stop'
                assert latest[b'app/drone_1/mission']['phase'] != 'COMPLETED'
                assert b'scenario/ended' not in latest
                report.update(passed=True, acknowledgement_timeout=True, false_success=False, screenshot=str(screenshot))
            else:
                event_ids = []
                for episode in range(2):
                    wait(lambda: latest.get(b'app/drone_1/mission', {}).get('phase') == 'COMPLETED', 'confirmed app completion')
                    wait(lambda: snapshot().get('contact') == 'COMPLETED / Target hit' and b'scenario/ended' in latest, 'renderer confirmation')
                    result = latest[b'scenario/ended']
                    assert result['event_id'] not in event_ids
                    event_ids.append(result['event_id'])
                    frozen = result['final_source']['step_index']
                    time.sleep(.3)
                    pump()
                    assert latest[b'flight/drone_1/feedback']['source']['step_index'] == frozen
                    wait(lambda: len(list((temp / 'recordings').glob('*.mcap'))) == episode + 1 and not list((temp / 'recordings').glob('*.partial')), 'recording finalized')
                    if episode == 0:
                        command('screenshot', path=str(args.report_dir / 'completed.png'))
                        wait((args.report_dir / 'completed.png').exists, 'completed screenshot')
                        os.killpg(app.pid, signal.SIGINT)
                        app.wait(10)
                        processes.remove(app)
                        old_id = latest[b'app/drone_1/mission']['app_id']
                        app = launch('app-restarted', [str(BIN / 'bullet-app'), '--transport-config', str(temp / 'transport.json'),
                            '--bbox-endpoint', endpoints[4], '--mission-config', str(ROOT.parent / 'los_tracking/examples/bullet_profile.json')])
                        wait(lambda: latest.get(b'app/drone_1/mission', {}).get('phase') == 'COMPLETED' and latest[b'app/drone_1/mission']['app_id'] != old_id, 'late app completion recovery')
                        command('reset')
                        wait(lambda: latest[b'app/drone_1/mission']['phase'] == 'READY' and (snapshot().get('state') or {}).get('reason') == 'reset', 'reset clears completion')
                        command('start')
                counts, ids, completed = {}, set(), 0
                for bag in (temp / 'recordings').glob('*.mcap'):
                    with bag.open('rb') as stream:
                        for _, channel, message in make_reader(stream).iter_messages():
                            counts[channel.topic] = counts.get(channel.topic, 0) + 1
                            if channel.topic == 'scenario/ended':
                                ids.add(json.loads(message.data)['event_id'])
                            if channel.topic == 'app/drone_1/mission' and json.loads(message.data)['phase'] == 'COMPLETED':
                                completed += 1
                assert len(ids) == counts.get('scenario/ended') == 2
                assert completed >= 2
                assert counts.get('render/drone_1/collision', 0) >= 4
                assert counts.get('camera/front/image', 0) > 0
                report.update(passed=True, event_ids=event_ids, counts=counts, late_app_completed=True,
                    reset_verified=True, dropped_first_report_recovered=True, selected_takeoff_height_m=6,
                    screenshot=str(screenshot))
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in processes:
            try:
                p.wait(15)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
        (args.report_dir / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
