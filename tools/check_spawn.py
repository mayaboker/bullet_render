"""Real-display scene-pick -> stopped spawn -> app takeoff -> MCAP readback check."""
import argparse
from contextlib import ExitStack
import json
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
    parser.add_argument('--report-dir', type=Path, default=Path('/tmp/bullet-spawn'))
    args = parser.parse_args()
    args.report_dir.mkdir(parents=True, exist_ok=True)
    processes = []
    (args.report_dir / "spawn-applied.png.done").unlink(missing_ok=True)
    with tempfile.TemporaryDirectory(prefix='bullet-spawn-') as directory, ExitStack() as stack, zmq.Context() as context:
        temp = Path(directory)
        reservations = [context.socket(zmq.PUB) for _ in range(5)]
        ports = [s.bind_to_random_port('tcp://127.0.0.1') for s in reservations]
        for sock in reservations:
            sock.close(0)
        transport = dict(zip(('telemetry_endpoint', 'image_endpoint', 'control_endpoint', 'flight_command_endpoint'),
                             [f'tcp://127.0.0.1:{p}' for p in ports[:4]]))
        transport_path = temp / 'transport.json'
        transport_path.write_text(json.dumps(transport))
        profile = json.loads((ROOT / 'configs/physics.json').read_text())
        profile['transport_config'] = str(transport_path)
        physics_path = temp / 'physics.json'
        physics_path.write_text(json.dumps(profile))
        app_endpoint = f'tcp://127.0.0.1:{ports[4]}'
        recorder = yaml.safe_load((ROOT / 'configs/recorder.yaml').read_text())
        recorder.update(transport_config=str(transport_path), output_dir=str(temp / 'recordings'))
        for topic in recorder['topics']:
            if topic['key'] == 'render/drone_1/collision':
                topic['endpoint'] = transport['control_endpoint']
            elif topic['key'] == 'flight/drone_1/rc':
                topic['endpoint'] = transport['flight_command_endpoint']
            elif topic['key'] == 'analysis/render/health':
                topic['endpoint'] = transport['image_endpoint']
            elif 'endpoint' in topic:
                topic['endpoint'] = app_endpoint
        recorder_path = temp / 'recorder.yaml'
        recorder_path.write_text(yaml.safe_dump(recorder))
        command_path, report_path = temp / 'command.json', temp / 'state.json'
        sequence = 0
        latest = {}
        sub = context.socket(zmq.SUB)
        sub.setsockopt(zmq.LINGER, 0)
        sub.setsockopt(zmq.SUBSCRIBE, b'')
        sub.connect(transport['telemetry_endpoint'])
        sub.connect(app_endpoint)
        def launch(name, command, cwd):
            log = stack.enter_context((args.report_dir / f'{name}.log').open('w'))
            process = subprocess.Popen(command, cwd=cwd, stdout=log, stderr=subprocess.STDOUT)
            processes.append(process)
            return process
        def snapshot():
            try:
                return json.loads(report_path.read_text())
            except (OSError, ValueError):
                return {}
        def pump():
            for _ in range(1024):
                try:
                    parts = sub.recv_multipart(zmq.NOBLOCK)
                except zmq.Again:
                    break
                if len(parts) == 2:
                    latest[parts[0].decode()] = json.loads(parts[1])
        def wait(predicate, description, timeout=15):
            deadline = time.monotonic() + timeout
            while True:
                pump()
                if predicate():
                    return
                assert all(p.poll() is None for p in processes), f'Process exited: {description}; {args.report_dir}'
                assert time.monotonic() < deadline, f'Timeout: {description}; {snapshot()}'
                time.sleep(.01)
        def command(action, **kwargs):
            nonlocal sequence
            sequence += 1
            pending = command_path.with_suffix('.tmp')
            pending.write_text(json.dumps(dict(sequence=sequence, action=action, **kwargs)))
            pending.replace(command_path)
        def current():
            return snapshot().get('spawn', {})
        try:
            launch('recorder', [str(BIN / 'mcap-bagger'), '--config', str(recorder_path)], ROOT.parent / 'mcap_bagger')
            launch('physics', [str(BIN / 'python'), '-m', 'examples.external_rc.main', '--render-config', str(physics_path)], ROOT.parent / 'bullet_drone')
            launch('app', [str(BIN / 'bullet-app'), '--transport-config', str(transport_path), '--bbox-endpoint', app_endpoint,
                           '--mission-config', str(ROOT.parent / 'los_tracking/examples/bullet_profile.yaml')], ROOT.parent / 'bullet_app')
            godot = launch('godot', [str(ROOT / 'tools/run_godot.sh'), '--path', str(ROOT / 'godot'), '--script', 'control_probe.gd', '--',
                                    str(command_path), str(report_path), str(transport_path)], ROOT)
            wait(lambda: bool(current()) and not snapshot().get('disabled', {}).get('start', True), 'spawn capability and app ready')
            initial_episode = current()['episode_id']
            command('spawn_pick', position_m=[10, 5, 0], takeoff_height_m=6.0)
            wait(lambda: abs(snapshot().get('draft', [0])[0]-10) < .01, 'real screen click selects non-origin point')
            draft = snapshot()['draft']
            assert abs(draft[1]-5) < .01 and abs(draft[2] - (snapshot()['terrain_height'] + .015)) <= .0011, draft
            assert snapshot()['position'] != [-draft[1], draft[2], -draft[0]], 'draft moved drone'
            assert current()['episode_id'] == initial_episode
            command('spawn_apply')
            wait(lambda: current().get('result') == 'applied' and current()['episode_id'] != initial_episode, 'Apply confirmed fresh stopped episode')
            wait(lambda: len(snapshot().get('position', [])) == 3 and all(abs(a-b) < 1e-5 for a, b in zip(snapshot()['position'], [-draft[1], draft[2], -draft[0]])), 'authoritative render pose')
            applied = current()
            assert applied['position_m'] == draft and applied['takeoff_height_m'] == 6
            assert snapshot()['state']['state'] == 'stopped'
            time.sleep(.3)
            pump()
            assert current()['episode_id'] == applied['episode_id'], 'retries created duplicate episode'
            assert not list((temp / 'recordings').glob('*.mcap')), 'stopped preview opened recorder'
            command('spawn_screenshot', path=str(args.report_dir / 'spawn-applied.png'))
            wait(lambda: (args.report_dir / 'spawn-applied.png.done').exists(), 'scenario panel screenshot')
            # Restart renderer while physics remains frozen: the retained status restores applied configuration.
            godot.send_signal(signal.SIGINT)
            godot.wait(5)
            processes.remove(godot)
            report_path.unlink(missing_ok=True)
            command('')
            launch('godot-restarted', [str(ROOT / 'tools/run_godot.sh'), '--path', str(ROOT / 'godot'), '--script', 'control_probe.gd', '--',
                                      str(command_path), str(report_path), str(transport_path)], ROOT)
            wait(lambda: current().get('position_m') == draft and not snapshot().get('disabled', {}).get('start', True), 'renderer restart recovery')
            assert current()['episode_id'] == applied['episode_id']
            command('reset')
            wait(lambda: current().get('episode_id') != applied['episode_id'], 'Reset fresh episode reuses spawn')
            assert current()['position_m'] == draft
            command('start')
            wait(lambda: latest.get('app/drone_1/mission', {}).get('phase') == 'HOVER', 'selected six metre climb', timeout=60)
            feedback = latest['flight/drone_1/feedback']
            assert feedback['takeoff_height_m'] == 6 and abs(feedback['barometer_altitude_m'] - (draft[2] + 6)) < .5, feedback
            assert snapshot()['spawn_apply_disabled']
            command('stop')
            wait(lambda: latest.get('flight/drone_1/feedback', {}).get('state') == 'stopped', 'Stop')
            wait(lambda: bool(list((temp / 'recordings').glob('*.mcap'))) and not list((temp / 'recordings').glob('*.partial')), 'MCAP finalized')
            packets = []
            for path in (temp / 'recordings').glob('*.mcap'):
                with path.open('rb') as stream:
                    for _, channel, message in make_reader(stream).iter_messages():
                        if channel.topic == 'simulation/spawn_state':
                            packets.append(json.loads(message.data))
            assert len(packets) >= 10 and all(p['position_m'] == draft and p['takeoff_height_m'] == 6 for p in packets), packets
            report = dict(selected_position_m=draft, takeoff_height_m=6, applied_episode=applied['episode_id'], renderer_restart_recovered=True,
                          reset_reused_spawn=True, hover_altitude_m=feedback['barometer_altitude_m'], mcap_spawn_messages=len(packets))
            (args.report_dir / 'report.json').write_text(json.dumps(report, indent=2))
            print(json.dumps(report, indent=2))
        finally:
            sub.close(0)
            for process in reversed(processes):
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(5)


if __name__ == '__main__':
    main()
