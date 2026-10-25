"""Check command-driven MCAP with real Godot rendering and physics.

Run with the shared workspace environment and a graphics display. Temporary recordings
are deleted; logs and a readback report remain in --report-dir.
"""

import argparse
from contextlib import ExitStack
import json
import math
from pathlib import Path
import re
import subprocess
import tempfile
import time

from foxglove_schemas_protobuf.RawImage_pb2 import RawImage
from mcap.reader import make_reader
import yaml

from check_pipeline import ROOT, PHYSICS, SHARED_BIN, stop_process


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report-dir', type=Path, default=Path('/tmp/bullet-recording-controls'))
    args = parser.parse_args()
    args.report_dir.mkdir(parents=True, exist_ok=True)
    processes = []
    with tempfile.TemporaryDirectory(prefix='bullet-controlled-mcap-') as temporary, ExitStack() as stack:
        directory = Path(temporary)
        output = directory / 'recordings'
        profile = yaml.safe_load((ROOT / 'configs/recorder.yaml').read_text())
        profile['output_dir'] = str(output)
        profile['transport_config'] = str(ROOT / 'configs/transport.json')
        config_path = directory / 'recorder.yaml'
        config_path.write_text(yaml.safe_dump(profile))
        command_path, state_path = directory / 'command.json', directory / 'state.json'
        sequence = 0

        def launch(name, command, cwd):
            log = stack.enter_context((args.report_dir / f'{name}.log').open('w'))
            process = subprocess.Popen(command, cwd=cwd, stdin=subprocess.DEVNULL,
                                       stdout=log, stderr=subprocess.STDOUT)
            processes.append(process)
            return process

        def snapshot():
            try:
                return json.loads(state_path.read_text())
            except (OSError, ValueError):
                return {}

        def wait(predicate, description, timeout=20):
            deadline = time.monotonic() + timeout
            while not predicate():
                assert time.monotonic() < deadline, f'timed out: {description}; inspect {args.report_dir}'
                assert all(process.poll() is None for process in processes), 'component exited; inspect logs'
                time.sleep(0.05)

        def command(action, expected):
            nonlocal sequence
            sequence += 1
            pending = command_path.with_suffix('.tmp')
            pending.write_text(json.dumps({'sequence': sequence, 'action': action}))
            pending.replace(command_path)
            wait(lambda: snapshot().get('state', {}).get('state') == expected and
                 (action != 'reset' or snapshot().get('state', {}).get('reason') == 'reset'), action)

        def closed(count):
            return len(list(output.glob('*.mcap'))) == count and not list(output.glob('*.partial'))

        try:
            launch('recorder', [str(SHARED_BIN / 'mcap-bagger'),
                               '--config', str(config_path)], ROOT.parent / 'mcap_bagger')
            launch('godot', [str(ROOT / 'tools/run_godot.sh'), '--path', str(ROOT / 'godot'), '--script', 'control_probe.gd',
                             '--', str(command_path), str(state_path)], ROOT)
            physics = launch('physics', [str(SHARED_BIN / 'python'), '-m',
                'examples.position_square.main', '--headless', '--render-config',
                str(ROOT / 'configs/physics.json')], PHYSICS)
            wait(lambda: snapshot().get('state', {}).get('state') == 'stopped' and
                 not snapshot().get('disabled', {}).get('start', True), 'initial stopped state')
            assert not list(output.glob('*')), 'recorder created a file before Start'
            for ending, expected_files in [('stop', 1), ('reset', 2), ('timeout', 3)]:
                command('start', 'running')
                wait(lambda: bool(list(output.glob('*.partial'))), 'recording opened')
                time.sleep(2)  # Allow real camera samples before finalizing each episode.
                if ending == 'timeout':
                    stop_process(physics)
                    processes.remove(physics)
                else:
                    command(ending, 'stopped')
                wait(lambda: closed(expected_files), f'{ending} finalized recording')
                time.sleep(0.3)
                assert closed(expected_files), 'a closed episode reopened'
            report = []
            for path in sorted(output.glob('*.mcap')):
                counts = {'images': 0, 'poses': 0}
                with path.open('rb') as stream:
                    reader = make_reader(stream)
                    assert reader.get_summary().chunk_indexes
                    for _, channel, message in reader.iter_messages():
                        if channel.topic == 'camera/front/image':
                            image = RawImage.FromString(message.data)
                            assert (image.width, image.height, image.step, image.encoding) == (640, 480, 1920, 'rgb8')
                            assert len(image.data) == 640 * 480 * 3 and len(set(image.data)) > 32
                            counts['images'] += 1
                        elif channel.topic == 'render/drone_1/pose':
                            pose = json.loads(message.data)
                            assert len(pose['position_m']) == 3
                            assert pose['schema_version'] == 2 and len(pose['linear_velocity_m_s']) == 3
                            assert all(math.isfinite(v) for v in pose['linear_velocity_m_s'])
                            counts['poses'] += 1
                assert counts['images'] > 0 and counts['poses'] > 0, counts
                report.append({'file': path.name, **counts})
            (args.report_dir / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
            print(json.dumps(report, indent=2))
        finally:
            for process in reversed(processes):
                stop_process(process)
            # Reap only slots belonging to this stopped renderer, as in check_pipeline.
            log_path = args.report_dir / 'godot.log'
            if log_path.exists():
                for session in re.findall(r'^SHM publisher session: ([0-9a-f]{32})$', log_path.read_text(), re.MULTILINE):
                    for index in range(3):
                        (Path('/dev/shm') / f'bullet_msgs_{session}_s{index}').unlink(missing_ok=True)


if __name__ == '__main__':
    main()
