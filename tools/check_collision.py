"""Real-display collision/confirmation/COMPLETED/MCAP lifecycle check.

Uses isolated ports and temporary recordings. Run from the shared environment.
"""
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
from bullet_msgs.flight import MISSION_TOPIC, FEEDBACK_TOPIC
from bullet_msgs.scenario import END_TOPIC, COLLISION_TOPIC
from bullet_msgs.los import LOS_TOPIC, validate_los


ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT.parent / '.venv/bin'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report-dir', type=Path, default=Path('/tmp/bullet-collision'))
    parser.add_argument('--reject-collisions', action='store_true', help='Exercise the one-second acknowledgement timeout and Stop fallback')
    args = parser.parse_args()
    args.report_dir.mkdir(parents=True, exist_ok=True)
    processes = []
    with tempfile.TemporaryDirectory(prefix='bullet-flight-') as temp, ExitStack() as stack, zmq.Context() as context:
        temp = Path(temp)
        # Reserve separate test ports; default workspace services are left untouched.
        reservations = [context.socket(zmq.PUB) for _ in range(5)]
        ports = [s.bind_to_random_port('tcp://127.0.0.1') for s in reservations]
        for s in reservations:
            s.close(0)
        transport = dict(zip(('telemetry_endpoint', 'image_endpoint', 'control_endpoint', 'flight_command_endpoint'),
                             [f'tcp://127.0.0.1:{p}' for p in ports[:4]]))
        (temp / 'transport.json').write_text(json.dumps(transport))
        profile = json.loads((ROOT / 'configs/physics.json').read_text())
        profile['transport_config'] = str(temp / 'transport.json')
        (temp / 'physics.json').write_text(json.dumps(profile))
        app_endpoint = f'tcp://127.0.0.1:{ports[4]}'
        recorder = yaml.safe_load((ROOT / 'configs/recorder.yaml').read_text())
        recorder.update(transport_config=str(temp / 'transport.json'), output_dir=str(temp / 'recordings'))
        for topic in recorder['topics']:
            if topic['key'] == 'render/drone_1/collision':
                topic['endpoint'] = transport['control_endpoint']
            elif topic['key'] == 'flight/drone_1/rc':
                topic['endpoint'] = transport['flight_command_endpoint']
            elif topic['key'] == 'analysis/render/health':
                topic['endpoint'] = transport['image_endpoint']
            elif 'endpoint' in topic:
                topic['endpoint'] = app_endpoint
        (temp / 'recorder.yaml').write_text(yaml.safe_dump(recorder))
        command_path, state_path = temp / 'command.json', temp / 'state.json'
        trajectory_path = temp / 'trajectory.json'
        screenshot = args.report_dir / 'completed.png'
        sequence = 0
        def launch(name, cmd, cwd):
            log = stack.enter_context((args.report_dir / f'{name}.log').open('w'))
            p = subprocess.Popen(cmd, cwd=cwd, stdout=log, stderr=subprocess.STDOUT)
            processes.append(p)
            return p
        def snapshot():
            try:
                return json.loads(state_path.read_text())
            except (ValueError, OSError):
                return {}
        sub = context.socket(zmq.SUB)
        sub.setsockopt(zmq.LINGER, 0)
        sub.setsockopt(zmq.SUBSCRIBE, b'')
        sub.connect(app_endpoint)
        sub.connect(transport['telemetry_endpoint'])
        latest = {}
        phases = []
        def pump():
            for _ in range(512):
                try:
                    parts = sub.recv_multipart(zmq.NOBLOCK)
                except zmq.Again:
                    break
                if len(parts) == 2:
                    latest[parts[0]] = json.loads(parts[1])
                    if parts[0] == MISSION_TOPIC:
                        phase = latest[MISSION_TOPIC]['phase']
                        if not phases or phases[-1] != phase:
                            phases.append(phase)
        def wait(predicate, description, timeout=15):
            deadline = time.monotonic() + timeout
            while True:
                pump()
                if predicate():
                    return
                assert all(p.poll() is None for p in processes), f'component exited: {description}; inspect {args.report_dir}'
                assert time.monotonic() < deadline, f'timed out: {description}; latest={latest.get(MISSION_TOPIC)}; inspect {args.report_dir}'
                time.sleep(.01)
        def command(action):
            nonlocal sequence
            sequence += 1
            pending = command_path.with_suffix('.tmp')
            pending.write_text(json.dumps(dict(sequence=sequence, action=action)))
            pending.replace(command_path)
        try:
            launch('recorder', [str(BIN / 'mcap-bagger'), '--config', str(temp / 'recorder.yaml')], ROOT.parent / 'mcap_bagger')
            launch('godot', [str(ROOT / 'tools/run_godot.sh'), '--path', str(ROOT / 'godot'), '--script', 'collision_probe.gd', '--',
                             str(command_path), str(state_path), str(temp / 'transport.json'), str(trajectory_path), str(screenshot)], ROOT)
            wait(lambda: trajectory_path.exists(), 'Godot trajectory ready')
            physics = launch('physics', [str(BIN / 'python'), str(ROOT / 'tools/collision_physics_probe.py'), str(temp / 'physics.json'), str(trajectory_path), *(['--reject-collisions'] if args.reject_collisions else [])], ROOT.parent / 'bullet_drone')
            app = launch('app', [str(BIN / 'bullet-app'), '--transport-config', str(temp / 'transport.json'),
                                '--bbox-endpoint', app_endpoint, '--mission-config', str(ROOT.parent / 'los_tracking/examples/bullet_profile.json')], ROOT.parent / 'bullet_app')
            wait(lambda: latest.get(MISSION_TOPIC, {}).get('phase') == 'READY' and snapshot().get('state', {}).get('state') == 'stopped' and not snapshot().get('disabled', {}).get('start', True), 'ready')
            time.sleep(.2)
            command('start')
            if args.reject_collisions:
                wait(lambda: snapshot().get('collision_failed', False) and latest.get(FEEDBACK_TOPIC, {}).get('state') == 'stopped', 'collision timeout freezes through ordinary Stop')
                assert latest[FEEDBACK_TOPIC]['reason'] == 'stop'
                assert latest[MISSION_TOPIC]['phase'] != 'COMPLETED' and END_TOPIC not in latest
                wait(lambda: len(list((temp / 'recordings').glob('*.mcap'))) == 1 and not list((temp / 'recordings').glob('*.partial')), 'timeout recording finalized')
                messages = []
                for path in (temp / 'recordings').glob('*.mcap'):
                    with path.open('rb') as stream:
                        messages.extend((c.topic, m.data) for _, c, m in make_reader(stream).iter_messages())
                assert not any(t == 'scenario/ended' for t, _ in messages)
                assert sum(t == 'render/drone_1/collision' for t, _ in messages) >= 2
                report = dict(acknowledgement_timeout=True, stopped_reason='stop', false_success=False)
                (args.report_dir / 'report.json').write_text(json.dumps(report, indent=2))
                print(json.dumps(report, indent=2))
                return
            wait(lambda: latest.get(MISSION_TOPIC, {}).get('phase') == 'COMPLETED', 'collision confirmed in app', timeout=15)
            wait(lambda: END_TOPIC in latest and screenshot.exists(), 'terminal result and completed screenshot')
            result = latest[END_TOPIC]
            frozen = result['final_source']['step_index']
            wait(lambda: latest.get(FEEDBACK_TOPIC, {}).get('reason') == 'target_hit' and
                 (snapshot().get('state', {}).get('terminal_result') or {}).get('event_id') == result['event_id'], 'all consumers see terminal state')
            time.sleep(.5)
            pump()
            assert latest[FEEDBACK_TOPIC]['source']['step_index'] == frozen
            assert latest[MISSION_TOPIC]['phase'] == 'COMPLETED'
            wait(lambda: len(list((temp / 'recordings').glob('*.mcap'))) == 1 and not list((temp / 'recordings').glob('*.partial')), 'completed recording finalized')
            # Late subscriber / process restart recovers COMPLETED, not READY.
            app.send_signal(signal.SIGINT)
            app.wait(5)
            processes.remove(app)
            prior_app_id = latest[MISSION_TOPIC]['app_id']
            time.sleep(.2)  # Expire old readiness before attempting a rejected Start.
            command('start')
            time.sleep(.2)
            pump()
            assert all(p.poll() is None for p in processes)
            assert latest[FEEDBACK_TOPIC]['reason'] == 'target_hit'
            assert latest[FEEDBACK_TOPIC]['source']['step_index'] == frozen
            launch('app-restarted', [str(BIN / 'bullet-app'), '--transport-config', str(temp / 'transport.json'),
                                    '--bbox-endpoint', app_endpoint, '--mission-config', str(ROOT.parent / 'los_tracking/examples/bullet_profile.json')], ROOT.parent / 'bullet_app')
            wait(lambda: latest.get(MISSION_TOPIC, {}).get('phase') == 'COMPLETED' and latest[MISSION_TOPIC]['app_id'] != prior_app_id, 'late app recovers completion')
            assert latest[FEEDBACK_TOPIC]['source']['step_index'] == frozen
            command('reset')
            wait(lambda: latest.get(MISSION_TOPIC, {}).get('phase') == 'READY' and snapshot().get('state', {}).get('reason') == 'reset', 'reset clears completion')
            time.sleep(.2)
            command('start')
            wait(lambda: END_TOPIC in latest and latest[END_TOPIC]['event_id'] != result['event_id'], 'second fresh collision')
            wait(lambda: len(list((temp / 'recordings').glob('*.mcap'))) == 2 and not list((temp / 'recordings').glob('*.partial')), 'second completed recording finalized')
            counts = {}
            completed_in_bags = 0
            completed_los_episodes = set()
            terminal_ids = set()
            for path in (temp / 'recordings').glob('*.mcap'):
                with path.open('rb') as stream:
                    reader = make_reader(stream)
                    health = next(item for item in reader.iter_metadata() if item.name == 'recording_health')
                    assert json.loads(health.metadata['complete'])
                    for _, channel, message in reader.iter_messages():
                        counts[channel.topic] = counts.get(channel.topic, 0) + 1
                        if channel.topic == LOS_TOPIC.decode():
                            packet = validate_los(json.loads(message.data))
                            if packet['mission_phase'] == 'COMPLETED':
                                assert not packet['active'] and packet['optical_ttc_s'] is None
                                completed_los_episodes.add(packet['source']['episode_id'])
                        if channel.topic == 'scenario/ended':
                            terminal_ids.add(json.loads(message.data)['event_id'])
                        elif channel.topic == 'app/drone_1/mission' and json.loads(message.data)['phase'] == 'COMPLETED':
                            completed_in_bags += 1
            assert all(counts.get(t, 0) > 0 for t in ('flight/drone_1/rc', 'flight/drone_1/feedback', 'app/drone_1/mission', 'camera/front/image', 'render/drone_1/pose')), counts
            assert all(counts.get(t, 0) > 0 for t in ('analysis/physics/config', 'analysis/app/config', 'analysis/physics/ground_truth', 'analysis/physics/sensors', 'simulation/wind_state', 'analysis/render/health')), counts
            assert counts.get('camera/front/image/metadata') == counts['camera/front/image'], counts
            assert counts.get('scenario/ended') == 2 and len(terminal_ids) == 2, counts
            assert completed_in_bags >= 2, 'missing final app status'
            assert len(completed_los_episodes) == 2, 'missing final inactive LOS packet'
            assert counts.get('render/drone_1/collision', 0) >= 4, counts
            report = dict(phases=phases, counts=counts, frozen_step=frozen, result=result,
                          late_app_completed=True, reset_verified=True, dropped_first_report_recovered=True, final_los_markers=len(completed_los_episodes), screenshot=str(screenshot))
            (args.report_dir / 'report.json').write_text(json.dumps(report, indent=2))
            print(json.dumps(report, indent=2))
        finally:
            sub.close(0)
            for process in reversed(processes):
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(5)


if __name__ == '__main__':
    main()
