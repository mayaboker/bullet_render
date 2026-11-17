"""Real-display app/physics/renderer/MCAP mission and process-loss smoke check.

Uses isolated ports and temporary recordings. Run from the shared environment.
"""
import argparse
from contextlib import ExitStack
import json
from datetime import datetime, timezone
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
from bullet_drone.config import YamlConfigLoader
from sitl_supervisor import SitlSupervisor


ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT.parent / '.venv/bin'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report-dir', type=Path, default=Path('/tmp/bullet-app-flight'))
    parser.add_argument('--physics-config', type=Path)
    parser.add_argument('--mission-config', type=Path, help='Navigator profile for the normal mission check')
    parser.add_argument('--yaw-search', choices=('found', 'empty'), help='Search with the target behind the spawn, or hide it for a full circle')
    parser.add_argument('--hover-check', action='store_true', help='Verify a stable Betaflight 20-second hover')
    args = parser.parse_args()
    if args.hover_check and args.yaw_search:
        parser.error('--hover-check cannot be combined with --yaw-search')
    args.report_dir.mkdir(parents=True, exist_ok=True)
    recording_dir = (args.report_dir / ('recordings-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
                     if args.hover_check else None)
    processes = []
    supervisor = None
    environment = dict(os.environ)
    with tempfile.TemporaryDirectory(prefix='bullet-flight-') as temp, ExitStack() as stack, zmq.Context() as context:
        temp = Path(temp)
        if recording_dir is None:
            recording_dir = temp / 'recordings'
        # Reserve separate test ports; default workspace services are left untouched.
        reservations = [context.socket(zmq.PUB) for _ in range(5)]
        ports = [s.bind_to_random_port('tcp://127.0.0.1') for s in reservations]
        for s in reservations:
            s.close(0)
        transport = dict(zip(('telemetry_endpoint', 'image_endpoint', 'control_endpoint', 'flight_command_endpoint'),
                             [f'tcp://127.0.0.1:{p}' for p in ports[:4]]))
        (temp / 'transport.json').write_text(json.dumps(transport))
        profile = json.loads((ROOT / 'configs/physics.json').read_text())
        if args.hover_check:
            profile['fps'] = 10
        profile['transport_config'] = str(temp / 'transport.json')
        (temp / 'physics.json').write_text(json.dumps(profile))
        app_endpoint = f'tcp://127.0.0.1:{ports[4]}'
        recorder = yaml.safe_load((ROOT / 'configs/recorder.yaml').read_text())
        recorder.update(transport_config=str(temp / 'transport.json'), output_dir=str(recording_dir))
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
        sequence = 0
        def launch(name, cmd, cwd):
            log = stack.enter_context((args.report_dir / f'{name}.log').open('w'))
            p = subprocess.Popen(cmd, cwd=cwd, env=environment, stdout=log, stderr=subprocess.STDOUT)
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
                if (latest.get(MISSION_TOPIC, {}).get('phase') == 'FAULT'
                        and description != 'one measured full circle without detection'):
                    raise AssertionError(f'unexpected mission fault: {latest[MISSION_TOPIC]}; MSP={latest.get(b"analysis/app/msp")}; inspect {args.report_dir}')
                assert all(p.poll() is None for p in processes), f'component exited: {description}; statuses={[(p.args, p.poll()) for p in processes]}; inspect {args.report_dir}'
                assert time.monotonic() < deadline, f'timed out: {description}; latest={latest.get(MISSION_TOPIC)}; snapshot={snapshot()}; inspect {args.report_dir}'
                time.sleep(.01)
        def command(action, **kwargs):
            nonlocal sequence
            sequence += 1
            pending = command_path.with_suffix('.tmp')
            pending.write_text(json.dumps(dict(sequence=sequence, action=action, **kwargs)))
            pending.replace(command_path)
        try:
            if args.physics_config is not None and YamlConfigLoader().load(args.physics_config).control.backend == 'betaflight':
                supervisor = SitlSupervisor(ROOT.parent / 'bullet_drone/.sitl/betaflight-source/obj/main/betaflight_SITL.elf',
                                            ROOT.parent / 'bullet_drone/configs/betaflight.config', args.report_dir / 'sitl-session')
                supervisor.start()
                environment['BULLET_SITL_CONTROL'] = supervisor.endpoint
            launch('recorder', [str(BIN / 'mcap-bagger'), '--config', str(temp / 'recorder.yaml')], ROOT.parent / 'mcap_bagger')
            physics_command = [str(BIN / 'python'), '-m', 'examples.external_rc.main', '--render-config', str(temp / 'physics.json')]
            if args.physics_config is not None:
                physics_command += ['--config', str(args.physics_config.resolve())]
            physics = launch('physics', physics_command, ROOT.parent / 'bullet_drone')
            mission_config = (args.mission_config.resolve() if args.mission_config else
                              ROOT.parent / ('bullet_app/configs/betaflight_hover_profile.yaml' if args.hover_check
                                             else 'los_tracking/examples/bullet_profile.yaml'))
            app_command = [str(BIN / 'bullet-app'), '--transport-config', str(temp / 'transport.json'),
                           '--bbox-endpoint', app_endpoint, '--mission-config', str(mission_config)]
            if args.hover_check:
                app_command.append('--hover-check')
            app = launch('app', app_command, ROOT.parent / 'bullet_app')
            # Probe supports a transport override after the command/state paths.
            launch('godot', [str(ROOT / 'tools/run_godot.sh'), '--path', str(ROOT / 'godot'), '--script', 'control_probe.gd', '--',
                             str(command_path), str(state_path), str(temp / 'transport.json')], ROOT)
            wait(lambda: latest.get(MISSION_TOPIC, {}).get('phase') == 'READY' and snapshot().get('state', {}).get('state') == 'stopped' and not snapshot().get('disabled', {}).get('start', True), 'ready')
            if args.yaw_search:
                command('search_setup', target_visible=args.yaw_search == 'found')
                wait(lambda: snapshot().get('target_visible') == (args.yaw_search == 'found'), 'search scene setup')
                wait(lambda: snapshot().get('draft', [0])[0] == 100 and not snapshot().get('spawn_apply_disabled', True), 'search draft and terrain clearance')
                old_episode = snapshot()['spawn']['episode_id']
                command('spawn_apply')
                wait(lambda: snapshot().get('spawn', {}).get('result') == 'applied' and snapshot()['spawn']['episode_id'] != old_episode, 'search spawn confirmed')
                wait(lambda: latest.get(MISSION_TOPIC, {}).get('episode_id') == snapshot()['spawn']['episode_id'], 'app sees selected episode')
            time.sleep(.2)
            command('start')
            if args.hover_check:
                wait(lambda: latest.get(MISSION_TOPIC, {}).get('phase') == 'COMPLETED',
                     'stable 20-second hover completes', timeout=100)
                status = latest[MISSION_TOPIC]
                assert status['reason'] == 'hover_complete', status
                assert all(phase not in phases for phase in ('WAIT_TARGET', 'YAW_SEARCH', 'LOS_APPROACH')), phases
                wait(lambda: latest.get(FEEDBACK_TOPIC, {}).get('state') == 'stopped', 'hover completion freezes physics')
                frozen = latest[FEEDBACK_TOPIC]['source']['step_index']
                time.sleep(.3)
                pump()
                assert latest[FEEDBACK_TOPIC]['source']['step_index'] == frozen
                wait(lambda: bool(list(recording_dir.glob('*.mcap')))
                     and not list(recording_dir.glob('*.partial')), 'hover MCAP finalized')
                truth, mission_rows = [], []
                for path in recording_dir.glob('*.mcap'):
                    with path.open('rb') as stream:
                        for _, channel, message in make_reader(stream).iter_messages(
                                topics=['analysis/physics/ground_truth', 'app/drone_1/mission']):
                            packet = json.loads(message.data)
                            if channel.topic == 'analysis/physics/ground_truth':
                                truth.append(packet)
                            else:
                                mission_rows.append(packet)
                completed = [p for p in mission_rows if p['phase'] == 'COMPLETED' and p['reason'] == 'hover_complete']
                held = [p for p in mission_rows if p['phase'] == 'HOVER']
                takeoff = [p for p in mission_rows if p['phase'] == 'TAKEOFF']
                assert completed and held and takeoff and len(truth) > 1000
                end_s = completed[0]['simulation_time_s']
                episode = completed[0]['episode_id']
                takeoff_times = [p['simulation_time_s'] for p in takeoff
                                 if p['episode_id'] == episode and p['simulation_time_s'] < end_s]
                takeoff_s = min(takeoff_times)
                last_takeoff_s = max(takeoff_times)
                start_s = min(p['simulation_time_s'] for p in held
                              if p['episode_id'] == episode and last_takeoff_s <= p['simulation_time_s'] < end_s)
                initial = next(p['ground_truth']['position_m'][2] for p in sorted(
                    truth, key=lambda item: item['source']['simulation_time_s'])
                    if p['source']['episode_id'] == episode and p['source']['simulation_time_s'] >= takeoff_s)
                altitudes = [p['ground_truth']['position_m'][2] - initial for p in truth
                             if p['source']['episode_id'] == episode
                             and start_s <= p['source']['simulation_time_s'] <= end_s]
                assert end_s - start_s >= 20.0, (start_s, end_s)
                errors = [altitude - 20.0 for altitude in altitudes]
                peak_error = max(abs(error) for error in errors)
                peak_to_peak = max(altitudes) - min(altitudes)
                rms_error = (sum(error * error for error in errors) / len(errors)) ** .5
                assert peak_error <= .5, dict(peak_error_m=peak_error, peak_to_peak_m=peak_to_peak,
                                               rms_error_m=rms_error, samples=len(altitudes))
                report = dict(phases=phases, relative_target_m=20., hold_duration_s=end_s - start_s,
                              takeoff_start_altitude_m=initial,
                              peak_error_m=peak_error, peak_to_peak_m=peak_to_peak,
                              rms_error_m=rms_error, samples=len(altitudes), final_state=latest[FEEDBACK_TOPIC]['state'],
                              final_reason=latest[FEEDBACK_TOPIC]['reason'], frozen_step=frozen)
                (args.report_dir / 'report.json').write_text(json.dumps(report, indent=2))
                print(json.dumps(report, indent=2))
                return
            if args.yaw_search:
                wait(lambda: latest.get(MISSION_TOPIC, {}).get('phase') == 'YAW_SEARCH', 'yaw search begins after hover', timeout=60)
                assert 'LOS_ACQUIRE' not in phases and 'LOS_APPROACH' not in phases, phases
                (args.report_dir / 'yaw-search.png.done').unlink(missing_ok=True)
                command('spawn_screenshot', path=str(args.report_dir / 'yaw-search.png'))
                wait(lambda: (args.report_dir / 'yaw-search.png.done').exists(), 'search screenshot')
            if args.yaw_search == 'empty':
                wait(lambda: latest.get(MISSION_TOPIC, {}).get('phase') == 'FAULT', 'one measured full circle without detection', timeout=40)
                status = latest[MISSION_TOPIC]
                assert status['reason'] == 'search_exhausted', status
                assert status['diagnostics']['search_swept_deg'] >= 360 - 1e-6, status
                assert 'LOS_ACQUIRE' not in phases and 'LOS_APPROACH' not in phases, phases
                wait(lambda: latest.get(FEEDBACK_TOPIC, {}).get('state') == 'stopped', 'exhausted search freezes physics')
                frozen = latest[FEEDBACK_TOPIC]['source']['step_index']
                time.sleep(.3)
                pump()
                assert latest[FEEDBACK_TOPIC]['source']['step_index'] == frozen
                wait(lambda: bool(list((temp / 'recordings').glob('*.mcap'))) and not list((temp / 'recordings').glob('*.partial')), 'search MCAP finalized')
                recorded_search = []
                for path in (temp / 'recordings').glob('*.mcap'):
                    with path.open('rb') as stream:
                        for _, channel, message in make_reader(stream).iter_messages():
                            if channel.topic == MISSION_TOPIC.decode():
                                packet = json.loads(message.data)
                                if packet['phase'] == 'YAW_SEARCH':
                                    recorded_search.append(packet)
                            elif channel.topic == LOS_TOPIC.decode():
                                assert not validate_los(json.loads(message.data))['active']
                assert len(recorded_search) >= 100
                assert any(p['diagnostics']['search_swept_deg'] > 350 for p in recorded_search)
                report = dict(phases=phases, search_status_samples=len(recorded_search),
                              final_diagnostics=status['diagnostics'], frozen_step=frozen, reason=status['reason'])
                (args.report_dir / 'report.json').write_text(json.dumps(report, indent=2))
                print(json.dumps(report, indent=2))
                return
            wait(lambda: latest.get(MISSION_TOPIC, {}).get('phase') == 'LOS_APPROACH', 'takeoff, hover, post-hover detection and LOS', timeout=65)
            assert all(p in phases for p in ('SENSOR_WARMUP', 'ARMING', 'TAKEOFF', 'HOVER', 'WAIT_TARGET', 'LOS_ACQUIRE', 'LOS_APPROACH')), phases
            if args.yaw_search == 'found':
                assert 'YAW_SETTLE' in phases, phases
                assert latest[MISSION_TOPIC]['diagnostics']['bearing_valid']
                from los_tracking.geometry import euler
                acquired_heading_deg = degrees(euler(latest[FEEDBACK_TOPIC]['orientation_xyzw'])[2])
                assert abs(abs(acquired_heading_deg) - 180) < 5, acquired_heading_deg
            until = time.monotonic() + 1
            while time.monotonic() < until:
                pump()
                assert latest[MISSION_TOPIC]['phase'] != 'FAULT', latest[MISSION_TOPIC]
                assert latest[FEEDBACK_TOPIC]['state'] == 'running', latest[FEEDBACK_TOPIC]
                time.sleep(.01)
            # A short image gap may hold in recovery, then must reacquire without stopping physics.
            wait(lambda: latest[MISSION_TOPIC]['phase'] == 'LOS_APPROACH'
                 and latest[MISSION_TOPIC]['diagnostics']['bearing_valid']
                 and latest[MISSION_TOPIC]['diagnostics']['rate_observable'], 'fresh LOS after image gap', timeout=5)
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
                                    '--bbox-endpoint', app_endpoint, '--mission-config', str(mission_config)], ROOT.parent / 'bullet_app')
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
            search_packets = []
            centered_handoffs = []
            for path in (temp / 'recordings').glob('*.mcap'):
                with path.open('rb') as stream:
                    for _, channel, message in make_reader(stream).iter_messages():
                        counts[channel.topic] = counts.get(channel.topic, 0) + 1
                        if channel.topic == LOS_TOPIC.decode():
                            los_packets.append(validate_los(json.loads(message.data)))
                        elif channel.topic == MISSION_TOPIC.decode():
                            packet = json.loads(message.data)
                            mission_diagnostics[(packet['episode_id'], packet['measurement_us'])] = packet['diagnostics']
                            if packet['phase'] == 'LOS_ACQUIRE' and 'target_centered' in packet['diagnostics']:
                                centered_handoffs.append(packet)
                            if packet['phase'] in ('YAW_SEARCH', 'YAW_CENTER', 'YAW_SETTLE'):
                                search_packets.append(packet)
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
            assert centered_handoffs, 'missing centered-only handoff in MCAP'
            for handoff in centered_handoffs:
                d = handoff['diagnostics']
                assert d['target_centered'] and abs(d['target_error_x_px']) <= 10, d
                assert d['search_commanded_yaw_rate_deg_s'] == 0, d
            if args.yaw_search == 'found':
                assert any(p['phase'] == 'YAW_CENTER' for p in search_packets)
                assert any(p['phase'] == 'YAW_SEARCH' and p['diagnostics']['search_swept_deg'] > 90 for p in search_packets)
                assert any(p['phase'] == 'YAW_SETTLE' for p in search_packets)
            report = dict(centered_handoff_errors_px=[p['diagnostics']['target_error_x_px'] for p in centered_handoffs], acquired_heading_deg=acquired_heading_deg if args.yaw_search == 'found' else None, search_status_samples=len(search_packets), los_degree_samples_checked=converted, exact_source_associations=matched, phases=phases, counts=counts, watchdog_reason='rc_expired', watchdog_latency_s=watchdog_latency_s, frozen_step=frozen, restart_requires_start=True, reset_verified=True)
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
            if supervisor is not None:
                supervisor.close()


if __name__ == '__main__':
    main()
