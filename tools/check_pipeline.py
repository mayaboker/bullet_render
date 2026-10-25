"""Run real native Godot + Bullet + ZeroMQ TCP + SHM + MCAP and independently inspect the recording.

uv run --locked --all-packages --all-extras --all-groups python tools/check_pipeline.py --seconds 60
Temporary MCAP files are deleted; logs/report/preview remain in --report-dir.
"""
import argparse
from contextlib import ExitStack
from dataclasses import replace
import json
import math
import os
import re
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time

from foxglove_schemas_protobuf.RawImage_pb2 import RawImage
from mcap.reader import make_reader
from mcap_bagger.config import Config
from mcap_bagger.recorder import Recording
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
PHYSICS = ROOT.parent / 'bullet_drone'
SHARED_BIN = ROOT.parent / '.venv/bin'


def stop_process(process):
    if process.poll() is None:
        process.send_signal(signal.SIGCONT)
        process.send_signal(signal.SIGINT)
        try:
            process.wait(5)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=5)
    parser.add_argument('--report-dir', type=Path, default=Path('/tmp/bullet-render-check'))
    parser.add_argument('--fault-checks', action='store_true', help='Also test late recorder startup, pauses and renderer restart.')
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    args.report_dir.mkdir(parents=True, exist_ok=True)
    processes = []
    godot_logs = []
    run = None
    with tempfile.TemporaryDirectory(prefix='bullet-render-mcap-') as temporary, ExitStack() as stack:
        config = replace(Config.load(ROOT / 'configs/recorder.yaml'), output_dir=Path(temporary))
        run = Recording(config, on_error=lambda message: print(message, flush=True))
        try:
            if not args.fault_checks:
                run.start()
            def launch(name, command, cwd):
                log_path = args.report_dir / (name + '.log')
                log = stack.enter_context(log_path.open('w'))
                if name.startswith('godot'):
                    godot_logs.append(log_path)
                process = subprocess.Popen(command, cwd=cwd, stdout=log, stderr=subprocess.STDOUT)
                processes.append(process)
                return process
            command_path = Path(temporary) / 'godot-command.json'
            state_path = Path(temporary) / 'godot-state.json'
            godot_command = [str(ROOT / 'tools/run_godot.sh'), '--path', str(ROOT / 'godot'), '--script', 'control_probe.gd',
                             '--', str(command_path), str(state_path)]
            command_sequence = 0
            def snapshot():
                try:
                    return json.loads(state_path.read_text())
                except (OSError, ValueError):
                    return {}
            def wait_state(expected):
                deadline = time.monotonic() + 15
                while True:
                    current = snapshot()
                    if current.get('state', {}).get('state') == expected and not current.get('disabled', {}).get('reset', True):
                        return current['state']
                    assert time.monotonic() < deadline, f'physics did not reach {expected}; inspect logs'
                    assert godot.poll() is None, 'Godot exited; inspect logs'
                    time.sleep(0.05)
            def command(action, expected):
                nonlocal command_sequence
                command_sequence += 1
                pending = command_path.with_suffix('.tmp')
                pending.write_text(json.dumps({'sequence': command_sequence, 'action': action}))
                pending.replace(command_path)
                if action == 'reset':
                    deadline = time.monotonic() + 15
                    while snapshot().get('state', {}).get('reason') != 'reset':
                        assert time.monotonic() < deadline, 'Reset was not observed'
                        time.sleep(0.05)
                return wait_state(expected)
            godot = launch('godot', godot_command, ROOT)
            viewers = [launch(f'viewer-{i}', [str(SHARED_BIN / 'bullet-app'),
                '--transport-config', str(ROOT / 'configs/transport.json'), '--debug',
                '--bbox-endpoint', f'tcp://127.0.0.1:{5557 + i}'], ROOT.parent / 'bullet_app')
                for i in (1, 2)]
            physics = launch('physics', [str(SHARED_BIN / 'python'), '-m',
                'examples.position_square.main', '--headless', '--render-config',
                str(ROOT / 'configs/physics.json')], PHYSICS)
            if args.fault_checks:
                time.sleep(1)
                run.start()
            initial = wait_state('stopped')
            time.sleep(0.3)
            assert snapshot()['state']['simulation_time_s'] == initial['simulation_time_s'] == 0
            command('start', 'running')
            deadline = time.monotonic() + 25
            while run.counts['camera/front/image']['written'] == 0:
                assert run.error is None, run.error
                assert all(p.poll() is None for p in processes), 'component exited; inspect logs'
                assert time.monotonic() < deadline, 'no images; inspect logs'
                time.sleep(0.05)
            with socket.socket() as unused_capture_port:
                unused_capture_port.bind(('127.0.0.1', 9871))
            start = time.monotonic()
            def ticks(pid):
                try:
                    fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
                    return int(fields[11]) + int(fields[12])
                except FileNotFoundError:
                    return None
            previous_cpu = {p.pid: ticks(p.pid) for p in processes}
            previous_cpu[os.getpid()] = ticks(os.getpid())
            cpu_totals = {'godot': 0, 'physics': 0, 'recorder': 0}
            first_count = run.counts['camera/front/image']['written']
            while time.monotonic() - start < args.seconds:
                assert run.error is None, run.error
                assert godot.poll() is None, 'renderer exited'
                if physics.poll() is not None:
                    assert physics.returncode == 0, 'physics failed'
                    physics = launch('physics-restarted', [str(SHARED_BIN / 'python'), '-m',
                        'examples.position_square.main', '--headless', '--render-config',
                        str(ROOT/'configs/physics.json')], PHYSICS)
                if snapshot().get('state', {}).get('state') == 'stopped':
                    command('start', 'running')
                for pid, name in [(godot.pid, 'godot'), (physics.pid, 'physics'), (os.getpid(), 'recorder')]:
                    current = ticks(pid)
                    if current is not None:
                        cpu_totals[name] += current - previous_cpu.get(pid, 0)
                        previous_cpu[pid] = current
                time.sleep(0.05)
            cpu_seconds = {name: ticks / os.sysconf("SC_CLK_TCK") for name, ticks in cpu_totals.items()}
            assert all(viewer.poll() is None for viewer in viewers), 'viewer exited; inspect logs'
            elapsed = time.monotonic() - start
            measured = (run.counts['camera/front/image']['written'] - first_count) / elapsed
            if args.fault_checks:
                def wait_images(previous):
                    deadline = time.monotonic() + 15
                    while run.counts['camera/front/image']['written'] <= previous:
                        assert run.error is None, run.error
                        assert time.monotonic() < deadline, 'image stream did not recover'
                        time.sleep(0.05)
                physics.send_signal(signal.SIGSTOP)
                time.sleep(0.5)
                paused = run.counts['camera/front/image']['written']
                time.sleep(0.5)
                assert run.counts['camera/front/image']['written'] == paused
                physics.send_signal(signal.SIGCONT)
                wait_images(paused)
                previous_poses = run.counts['render/drone_1/pose']['written']
                godot.send_signal(signal.SIGSTOP)
                time.sleep(1.5)
                assert run.counts['render/drone_1/pose']['written'] > previous_poses + 20, 'slow rendering blocked physics'
                paused = run.counts['camera/front/image']['written']
                godot.send_signal(signal.SIGCONT)
                wait_images(paused)
                stop_process(godot)
                paused = run.counts['camera/front/image']['written']
                time.sleep(3.2)  # The absent Godot heartbeat must end the current episode.
                state_path.unlink(missing_ok=True)
                godot = launch('godot-restarted', godot_command, ROOT)
                stopped = wait_state('stopped')
                assert stopped['reason'] == 'controller_timeout'
                command('start', 'running')
                wait_images(paused)
            stopped = command('stop', 'stopped')
            time.sleep(0.3)
            assert snapshot()['state']['simulation_time_s'] == stopped['simulation_time_s']
            reset = command('reset', 'stopped')
            assert reset['simulation_time_s'] == 0 and reset['episode_id'] != stopped['episode_id']
            restarted = command('start', 'running')
            assert restarted['episode_id'] != reset['episode_id']
            stop_process(physics)
            # Draining one in-flight capture is allowed; duplicates after that are not.
            time.sleep(0.5)
            paused_count = run.counts['camera/front/image']['written']
            time.sleep(0.5)
            assert run.counts['camera/front/image']['written'] == paused_count, 'images repeated after physics stopped'
        finally:
            for process in reversed(processes):
                stop_process(process)
            run.close()
            # SIGINT/kill can leave SHM names, reap only these stopped publishers' logged sessions.
            orphan_slots_removed = 0
            for log_path in godot_logs:
                for session in re.findall(r'^SHM publisher session: ([0-9a-f]{32})$', log_path.read_text(), re.MULTILINE):
                    for index in range(3):
                        path = Path('/dev/shm') / f'bullet_msgs_{session}_s{index}'
                        if path.exists():
                            path.unlink()
                            orphan_slots_removed += 1
        assert run.error is None, run.error
        counts = run.counts
        assert counts['camera/front/image']['written'] > 0
        images, poses = 0, 0
        first_image = None
        with run.path.open('rb') as stream:
            reader = make_reader(stream)
            assert reader.get_summary().chunk_indexes
            for schema, channel, message in reader.iter_messages():
                assert message.log_time == message.publish_time
                if channel.topic == 'camera/front/image':
                    image = RawImage.FromString(message.data)
                    assert (image.width, image.height, image.step, image.encoding) == (640,480,1920,'rgb8')
                    assert len(image.data) == 640*480*3
                    if first_image is None:
                        first_image = Image.frombytes('RGB', (640,480), image.data)
                        assert len(set(image.data)) > 32, 'blank camera frame'
                    images += 1
                elif channel.topic == 'render/drone_1/pose':
                    pose = json.loads(message.data)
                    assert len(pose['position_m']) == 3 and len(pose['orientation_xyzw']) == 4
                    assert pose['schema_version'] == 2 and len(pose['linear_velocity_m_s']) == 3
                    assert all(math.isfinite(v) for v in pose['linear_velocity_m_s'])
                    poses += 1
        assert poses > 0
        first_image.save(args.report_dir/'camera.png')
        native_stats = []
        for path in godot_logs:
            snapshots = [json.loads(line.split(': ', 1)[1]) for line in path.read_text().splitlines()
                         if line.startswith(('Native camera final: ', 'Native camera stats: '))]
            if snapshots:
                native_stats.append({'log': path.name, 'last_observed': snapshots[-1]})
        sessions = {item['last_observed'].get('shm_session') for item in native_stats} - {None, ''}
        assert all(not (Path('/dev/shm') / f'bullet_msgs_{session}_s{i}').exists()
                   for session in sessions for i in range(3)), 'owned SHM slots still exist'
        report = {'transport': 'zmq-tcp-shm', 'cpu_percent': {name: seconds / elapsed * 100 for name, seconds in cpu_seconds.items()}, 'native_stats': native_stats, 'controls_verified': True, 'fault_checks' : args.fault_checks, 'seconds': elapsed, 'measured_fps': measured, 'images_read_back': images,
                  'poses_read_back': poses, 'counts': counts, 'mcap_bytes': run.path.stat().st_size,
                  'peak_queued_bytes': run.peak_queued_bytes, 'skipped_images': run.skipped_images, 'orphan_slots_removed': orphan_slots_removed}
        (args.report_dir/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
