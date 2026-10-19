"""Real-display app/physics/renderer/MCAP mission and process-loss smoke check.

Uses isolated ports and temporary recordings. Run from the shared environment.
"""
import argparse
from contextlib import ExitStack
import json
from math import degrees, isclose
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from mcap.reader import make_reader
import yaml
import zmq
from bullet_msgs.flight import MISSION_TOPIC, FEEDBACK_TOPIC
from bullet_msgs.los import LOS_TOPIC, validate_los


ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT.parent / '.venv/bin'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report-dir', type=Path, default=Path('/tmp/bullet-app-flight'))
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
            elif 'endpoint' in topic:
                topic['endpoint'] = app_endpoint
        (temp / 'recorder.yaml').write_text(yaml.safe_dump(recorder))
        command_path, state_path = temp / 'command.json', temp / 'state.json'
        sequence = 0
        def launch(name, cmd, cwd):
            log = stack.enter_context((args.report_dir / f'{name}.log').open('w'))
            env = dict(os.environ, __NV_PRIME_RENDER_OFFLOAD='1', __GLX_VENDOR_LIBRARY_NAME='nvidia')
            p = subprocess.Popen(cmd, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT)
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
            physics = launch('physics', [str(BIN / 'python'), '-m', 'examples.external_rc.main', '--render-config', str(temp / 'physics.json')], ROOT.parent / 'bullet_drone')
            app = launch('app', [str(BIN / 'bullet-app'), '--transport-config', str(temp / 'transport.json'),
                                '--bbox-endpoint', app_endpoint, '--mission-config', str(ROOT.parent / 'los_tracking/examples/bullet_profile.json')], ROOT.parent / 'bullet_app')
            # Probe supports a transport override after the command/state paths.
            launch('godot', ['godot', '--path', str(ROOT / 'godot'), '--script', 'control_probe.gd', '--',
                             str(command_path), str(state_path), str(temp / 'transport.json')], ROOT)
            wait(lambda: latest.get(MISSION_TOPIC, {}).get('phase') == 'READY' and snapshot().get('state', {}).get('state') == 'stopped' and not snapshot().get('disabled', {}).get('start', True), 'ready')
            time.sleep(.2)
            command('start')
            wait(lambda: latest.get(MISSION_TOPIC, {}).get('phase') == 'LOS_APPROACH', 'takeoff, hover, post-hover detection and LOS', timeout=65)
            assert all(p in phases for p in ('SENSOR_WARMUP', 'ARMING', 'TAKEOFF', 'HOVER', 'WAIT_TARGET', 'LOS_ACQUIRE', 'LOS_APPROACH')), phases
            until = time.monotonic() + 1
            while time.monotonic() < until:
                pump()
                assert latest[MISSION_TOPIC]['phase'] != 'FAULT', latest[MISSION_TOPIC]
                time.sleep(.01)
            diagnostics = latest[MISSION_TOPIC]['diagnostics']
            assert diagnostics['bearing_valid'] and diagnostics['rate_observable'], diagnostics
            assert abs(diagnostics['reference_pitch_rad']) > 0
            # A hard process loss cannot emit a graceful withdrawal; watchdog must freeze.
            killed_at = time.monotonic()
            prior_app_id = latest[MISSION_TOPIC]['app_id']
            app.kill()
            app.wait(5)
            processes.remove(app)
            wait(lambda: latest.get(FEEDBACK_TOPIC, {}).get('state') == 'stopped', 'app loss watchdog')
            watchdog_latency_s = time.monotonic() - killed_at
            assert watchdog_latency_s < .3, watchdog_latency_s
            frozen = latest[FEEDBACK_TOPIC]['source']['step_index']
            assert latest[FEEDBACK_TOPIC]['reason'] == 'rc_expired', latest[FEEDBACK_TOPIC]
            time.sleep(.3)
            pump()
            assert latest[FEEDBACK_TOPIC]['source']['step_index'] == frozen
            wait(lambda: bool(list((temp / 'recordings').glob('*.mcap'))) and not list((temp / 'recordings').glob('*.partial')), 'recording finalized')
            # No app can silently resume a frozen episode, and Start needs readiness.
            command('start')
            wait(lambda: latest.get(FEEDBACK_TOPIC, {}).get('reason') == 'app_not_ready', 'Start rejected without app')
            old_episode = latest[FEEDBACK_TOPIC]['source']['episode_id']
            launch('app-restarted', [str(BIN / 'bullet-app'), '--transport-config', str(temp / 'transport.json'),
                                    '--bbox-endpoint', app_endpoint, '--mission-config', str(ROOT.parent / 'los_tracking/examples/bullet_profile.json')], ROOT.parent / 'bullet_app')
            wait(lambda: latest.get(MISSION_TOPIC, {}).get('phase') == 'READY' and latest[MISSION_TOPIC]['app_id'] != prior_app_id, 'restarted app ready')
            time.sleep(.2)
            pump()
            assert latest[FEEDBACK_TOPIC]['state'] == 'stopped'
            assert latest[FEEDBACK_TOPIC]['source']['step_index'] == frozen
            command('reset')
            wait(lambda: latest.get(FEEDBACK_TOPIC, {}).get('reason') == 'reset' and latest[FEEDBACK_TOPIC]['source']['episode_id'] != old_episode, 'reset fresh stopped episode')
            time.sleep(.2)
            command('start')
            wait(lambda: latest.get(MISSION_TOPIC, {}).get('phase') == 'TAKEOFF', 'fresh Start after app restart')
            command('stop')
            wait(lambda: latest.get(FEEDBACK_TOPIC, {}).get('reason') == 'stop', 'explicit Stop')
            wait(lambda: len(list((temp / 'recordings').glob('*.mcap'))) == 2 and not list((temp / 'recordings').glob('*.partial')), 'second recording finalized')
            counts, poses, boxes = {}, {}, []
            los_packets = []
            mission_diagnostics = {}
            for path in (temp / 'recordings').glob('*.mcap'):
                with path.open('rb') as stream:
                    for _, channel, message in make_reader(stream).iter_messages():
                        counts[channel.topic] = counts.get(channel.topic, 0) + 1
                        if channel.topic == LOS_TOPIC.decode():
                            los_packets.append(validate_los(json.loads(message.data)))
                        elif channel.topic == MISSION_TOPIC.decode():
                            packet = json.loads(message.data)
                            mission_diagnostics[(packet['episode_id'], packet['measurement_us'])] = packet['diagnostics']
                        if channel.topic == 'render/drone_1/pose':
                            p = json.loads(message.data)
                            source = p['source']
                            poses[(source['episode_id'], source['step_index'])] = source['measurement_us']
                        elif channel.topic == 'tracking/front/bbox':
                            boxes.append(json.loads(message.data))
            assert all(counts.get(t, 0) > 0 for t in ('flight/drone_1/rc', 'flight/drone_1/feedback', 'app/drone_1/mission', 'tracking/front/bbox', 'camera/front/image', 'render/drone_1/pose')), counts
            active_los = [p for p in los_packets if p['active'] and p['bearing_valid']]
            assert len(active_los) >= 10, 'no active bearing telemetry in MCAP'
            converted = 0
            for packet in active_los:
                episode = packet['source']['episode_id']
                diagnostic = next((mission_diagnostics[(episode, packet['generated_us'] + offset)]
                                   for offset in (0, -1, 1) if (episode, packet['generated_us'] + offset) in mission_diagnostics), None)
                if diagnostic is not None:
                    assert isclose(packet['elevation_deg'], degrees(diagnostic['elevation_rad']), abs_tol=1e-8)
                    assert isclose(packet['azimuth_deg'], degrees(diagnostic['azimuth_rad']), abs_tol=1e-8)
                    converted += 1
            assert converted >= 10, 'could not associate degree/radian samples'
            matched = 0
            for box in boxes:
                assert box['schema_version'] == 2
                source = box['source']
                assert box['capture_ns'] == source['measurement_us'] * 1000
                key = (source['episode_id'], source['step_index'])
                if key in poses:
                    assert poses[key] == source['measurement_us']
                    matched += 1
            assert matched > 10, 'no exact image/bbox/pose source associations'
            report = dict(los_degree_samples_checked=converted, exact_source_associations=matched, phases=phases, counts=counts, watchdog_reason='rc_expired', watchdog_latency_s=watchdog_latency_s, frozen_step=frozen, restart_requires_start=True, reset_verified=True)
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
