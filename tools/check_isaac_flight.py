"""Isolated real physics/app/Isaac acquisition probe; no production controls sent."""
import argparse
from collections import Counter
from contextlib import ExitStack
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import zmq
from bullet_msgs.shm import FrameReader, topic_prefix

ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT.parent / '.venv/bin'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report-dir', type=Path, default=Path('/tmp/bullet-isaac-flight'))
    parser.add_argument('--gui', action='store_true', help='Use the displayed renderer, matching interactive flight')
    parser.add_argument('--app-debug', action='store_true', help='Include the actual app camera/debug window')
    parser.add_argument('--approach-seconds', type=float, default=3, help='Simulation seconds to verify after acquisition')
    parser.add_argument('--takeoff-height', type=float, help='Verify a physics-confirmed custom relative climb')
    parser.add_argument('--world', type=Path, help='USD world for the renderer; omit for the starter slab')
    parser.add_argument('--second-viewport', action='store_true', help='Open a second GUI viewport on the front camera before Start to measure its render cost')
    parser.add_argument('--spawn', type=float, nargs=3, metavar=('X', 'Y', 'Z'), help='Starting position applied through stopped physics')
    parser.add_argument('--target', type=float, nargs=3, metavar=('X', 'Y', 'Z'), help='Red truck origin in Bullet metres')
    parser.add_argument('--mission-config', type=Path, default=ROOT.parent / 'los_tracking/examples/bullet_profile.json',
                        help='Navigator profile for the app; isaac/mission_profile.json allows Isaac image latency')
    args = parser.parse_args()
    args.report_dir.mkdir(parents=True, exist_ok=True)
    processes = []
    reader = FrameReader('camera/front/image')
    report = dict(passed=False)
    try:
        with ExitStack() as stack, zmq.Context() as context:
            sockets = [context.socket(zmq.PUB) for _ in range(5)]
            endpoints = [f'tcp://127.0.0.1:{s.bind_to_random_port("tcp://127.0.0.1")}' for s in sockets]
            for socket in sockets:
                socket.close(0)
            profile = dict(zip(('telemetry_endpoint', 'image_endpoint', 'control_endpoint', 'flight_command_endpoint'), endpoints))
            transport_path = args.report_dir / 'transport.json'
            transport_path.write_text(json.dumps(profile))
            physics = json.loads((ROOT / 'configs/physics.json').read_text())
            physics['transport_config'] = str(transport_path)
            physics_path = args.report_dir / 'physics.json'
            physics_path.write_text(json.dumps(physics))
            command_path = args.report_dir / 'command.json'
            command_path.unlink(missing_ok=True)
            telemetry = stack.enter_context(context.socket(zmq.SUB))
            telemetry.setsockopt(zmq.LINGER, 0)
            telemetry.setsockopt(zmq.SUBSCRIBE, b'')
            telemetry.connect(endpoints[0])
            telemetry.connect(endpoints[4])
            telemetry.connect(endpoints[3])
            images = stack.enter_context(context.socket(zmq.SUB))
            images.setsockopt(zmq.LINGER, 0)
            images.setsockopt(zmq.SUBSCRIBE, topic_prefix('camera/front/image'))
            images.connect(endpoints[1])

            def launch(name, command, cwd):
                log = stack.enter_context((args.report_dir / f'{name}.log').open('w'))
                processes.append(subprocess.Popen(command, cwd=cwd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))

            launch('physics', [str(BIN / 'python'), '-m', 'examples.external_rc.main', '--render-config', str(physics_path)], ROOT.parent / 'bullet_drone')
            launch('app', [str(BIN / 'bullet-app'), '--transport-config', str(transport_path), '--bbox-endpoint', endpoints[4],
                           '--mission-config', str(args.mission_config.resolve()),
                           *(['--debug'] if args.app_debug else [])], ROOT.parent / 'bullet_app')
            launch('isaac', ['/home/user/isaacsim5.0/python.sh', str(ROOT / 'tools/isaac_control_probe.py'), str(command_path),
                             *([] if args.gui else ['--headless']), '--transport-config', str(transport_path),
                             *(['--takeoff-height', str(args.takeoff_height)] if args.takeoff_height is not None else []),
                             *(['--world', str(args.world)] if args.world is not None else []),
                             *(['--spawn', *map(str, args.spawn)] if args.spawn is not None else []),
                             *(['--target', *map(str, args.target)] if args.target is not None else [])], ROOT)
            latest, phases, reasons, ages, search_ages = {}, [], Counter(), [], []
            recent_frames, saved_frames, bbox_log = {}, 0, Counter()
            MAX_SAVED_FRAMES = 12
            started = False
            second_viewport_sent = False
            approach_started = None
            command_timings = []
            ready_at = None
            deadline = time.monotonic() + 240
            trace = stack.enter_context((args.report_dir / 'trace.jsonl').open('w'))
            while time.monotonic() < deadline:
                assert all(p.poll() is None for p in processes), f'component exited; see {args.report_dir}'
                for _ in range(512):
                    try:
                        parts = telemetry.recv_multipart(zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    if len(parts) != 2:
                        continue
                    value = json.loads(parts[1])
                    latest[parts[0]] = value
                    if parts[0] == b'flight/drone_1/rc':
                        stamp = time.monotonic_ns() // 1000
                        command_timings.append(dict(received_us=stamp, issued_us=value['issued_us'],
                            valid_until_us=value['valid_until_us'], active=value['active'], sequence=value['sequence']))
                        command_timings = command_timings[-100:]
                    if parts[0] == b'tracking/front/bbox' and value.get('bbox') is not None and latest.get(b'app/drone_1/mission', {}).get('phase', '').startswith('LOS'):
                        # Record every LOS-phase detection; keep the frames behind edge-touching boxes.
                        x, y, w, h = (value['bbox'][k] for k in ('x', 'y', 'width', 'height'))
                        clipped = x <= 0 or y <= 0 or x + w >= 640 or y + h >= 480
                        bbox_log['clipped' if clipped else 'inside'] += 1
                        trace.write(json.dumps(dict(bbox=[x, y, w, h], clipped=clipped, source=value.get('source'))) + '\n')
                        key = (value.get('session'), value.get('sequence'))
                        if clipped and saved_frames < MAX_SAVED_FRAMES and key in recent_frames:
                            import cv2
                            import numpy as np
                            frame = cv2.cvtColor(np.frombuffer(recent_frames[key], dtype=np.uint8).reshape(480, 640, 3), cv2.COLOR_RGB2BGR)
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                            cv2.imwrite(str(args.report_dir / f'clipped_{saved_frames:02d}.png'), frame)
                            saved_frames += 1
                    if parts[0] == b'app/drone_1/mission':
                        phase = value['phase']
                        if not phases or phases[-1] != phase:
                            phases.append(phase)
                            print('Mission', phase, value['reason'], flush=True)
                        if phase in ('WAIT_TARGET', 'YAW_SEARCH', 'YAW_SETTLE', 'LOS_ACQUIRE', 'LOS_APPROACH'):
                            reason = value['diagnostics'].get('vision_reason')
                            reasons[str(reason)] += 1
                            trace.write(json.dumps(value) + '\n')
                mission = latest.get(b'app/drone_1/mission', {})
                state = latest.get(b'simulation/state', {})
                if images.poll(5):
                    frame = reader.read(images.recv())
                    if frame:
                        metadata, pixels = frame
                        recent_frames[(metadata['session'], metadata['sequence'])] = pixels
                        if len(recent_frames) > 30:
                            del recent_frames[next(iter(recent_frames))]
                        age = (time.monotonic_ns() - metadata['capture_ns']) / 1e6
                        ages.append(age)
                        if mission.get('phase') in ('WAIT_TARGET', 'YAW_SEARCH', 'YAW_SETTLE', 'LOS_ACQUIRE'):
                            search_ages.append(age)
                            trace.write(json.dumps(dict(image_age_ms=age, source=metadata.get('source'))) + '\n')
                settings_ready = True
                if args.takeoff_height is not None or args.spawn is not None:
                    snapshot_path = args.report_dir / 'isaac_state.json'
                    snapshot = json.loads(snapshot_path.read_text()) if snapshot_path.exists() else {}
                    applied = snapshot.get('spawn') or {}
                    settings_ready = (not snapshot.get('pending', True) and bool(applied)
                                      and (args.takeoff_height is None or applied.get('takeoff_height_m') == args.takeoff_height)
                                      and (args.spawn is None or applied.get('position_m') == list(args.spawn)))
                if mission.get('phase') == 'READY' and state.get('controller_id') and settings_ready and not started:
                    ready_at = ready_at or time.monotonic()
                    if time.monotonic() - ready_at >= 1:
                        pending = command_path.with_suffix('.tmp')
                        if args.second_viewport and args.gui and not second_viewport_sent:
                            pending.write_text(json.dumps(dict(sequence=1, action='second_viewport')))
                            pending.replace(command_path)
                            second_viewport_sent = True
                            ready_at = time.monotonic() + 2
                            continue
                        pending.write_text(json.dumps(dict(sequence=2, action='start')))
                        pending.replace(command_path)
                        started = True
                        deadline = time.monotonic() + 100
                if mission.get('phase') == 'LOS_APPROACH' and approach_started is None:
                    approach_started = mission['simulation_time_s']
                approach_verified = (mission.get('phase') == 'LOS_APPROACH' and approach_started is not None
                                     and mission['simulation_time_s'] - approach_started >= args.approach_seconds)
                rc_expired = started and state.get('reason') == 'rc_expired'
                completed = mission.get('phase') == 'COMPLETED' and state.get('reason') == 'target_hit' and state.get('terminal_result') is not None
                if approach_verified or completed or mission.get('phase') == 'FAULT' or rc_expired:
                    import statistics
                    report.update(passed=approach_verified or completed, phases=phases, vision_reasons=dict(reasons),
                                  image_age_ms=dict(median=statistics.median(ages), p90=sorted(ages)[int(len(ages) * .9)], max=max(ages), n=len(ages)) if ages else None,
                                  world=str(args.world) if args.world else None, spawn=args.spawn, target=args.target, mission_config=str(args.mission_config),
                                  los_bboxes=dict(bbox_log), clipped_frames_saved=saved_frames,
                                  last_mission=mission, search_frame_count=len(search_ages),
                                  search_image_age_ms=search_ages[-100:],
                                  approach_verified_s=mission['simulation_time_s'] - approach_started if approach_started is not None else 0,
                                  physics_state=state, last_commands=command_timings, completion_confirmed=completed,
                                  selected_takeoff_height_m=args.takeoff_height)
                    assert report['passed'], f'mission did not acquire visible target: {mission["reason"]}; see {args.report_dir}'
                    break
            else:
                report.update(phases=phases, latest_mission=mission)
                raise TimeoutError(f'mission did not reach LOS_APPROACH; see {args.report_dir}')
    finally:
        reader.close()
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        (args.report_dir / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
