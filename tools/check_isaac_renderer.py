"""GPU smoke check with isolated TCP ports and synthetic stationary physics poses."""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

import cv2
import numpy as np
import zmq

from bullet_app.detector import detect_red_box
from bullet_msgs.shm import FrameReader, topic_prefix


ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--isaac', type=Path, default=Path('/home/user/isaacsim5.0'))
    parser.add_argument('--report-dir', type=Path, default=Path('/tmp/bullet-isaac-render'))
    parser.add_argument('--gui', action='store_true', help='Also exercise the displayed viewport and control panel')
    parser.add_argument('--moving-camera', action='store_true', help='Alternate camera position to check exact image/source correspondence')
    args = parser.parse_args()
    args.report_dir.mkdir(parents=True, exist_ok=True)
    profile = {}
    reservations = []
    for key in ('telemetry_endpoint', 'image_endpoint', 'control_endpoint'):
        reserved = socket.socket()
        reserved.bind(('127.0.0.1', 0))
        reservations.append(reserved)
        profile[key] = f'tcp://127.0.0.1:{reserved.getsockname()[1]}'
    for reserved in reservations:
        reserved.close()
    path = args.report_dir / 'transport.json'
    path.write_text(json.dumps(profile))
    process = None
    reader = FrameReader('camera/front/image')
    result = {'passed': False}
    try:
        with zmq.Context() as context, context.socket(zmq.PUB) as telemetry, context.socket(zmq.SUB) as images, (args.report_dir / 'isaac.log').open('w') as log:
            telemetry.setsockopt(zmq.LINGER, 0)
            images.setsockopt(zmq.LINGER, 0)
            images.setsockopt(zmq.SUBSCRIBE, topic_prefix('camera/front/image'))
            telemetry.bind(profile['telemetry_endpoint'])
            images.connect(profile['image_endpoint'])
            process = subprocess.Popen([str(args.isaac / 'python.sh'), str(ROOT / 'isaac/main.py'),
                                        *([] if args.gui else ['--headless']),
                                        '--seconds', '60', '--transport-config', str(path)],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 180
            sequence = 0
            sent = deque(maxlen=4096)
            arrivals, ages = [], []
            camera_positions_seen = set()
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(f'Isaac exited with {process.returncode}; see {args.report_dir / "isaac.log"}')
                sequence += 1
                source = dict(physics_id='probe', episode_id='probe', stream_id='probe', step_index=sequence,
                              simulation_time_s=sequence / 30, measurement_us=time.monotonic_ns() // 1000)
                sent.append(source)
                config = dict(schema_version=1, stream_id='probe', camera=dict(width_px=640, height_px=480, fov_x_deg=90), fps=30)
                camera_y = (5 if sequence % 2 else -5) if args.moving_camera else 0
                pose = dict(schema_version=3, stream_id='probe', drone_id='drone_1', episode_id='probe',
                            sequence=sequence, publisher_drops=0, step_index=sequence, simulation_time_s=sequence / 30,
                            position_m=[0, camera_y, 3], orientation_xyzw=[0, 0, 0, 1], linear_velocity_m_s=[0, 0, 0], source=source)
                telemetry.send_multipart([b'render/drone_1/config', json.dumps(config).encode()])
                telemetry.send_multipart([b'render/drone_1/pose', json.dumps(pose).encode()])
                if images.poll(33):
                    frame = reader.read(images.recv())
                    if frame is None:
                        continue
                    metadata, pixels = frame
                    assert metadata['version'] == 2 and metadata['source'] in sent
                    assert metadata['capture_ns'] == metadata['source']['measurement_us'] * 1000
                    rgb = np.frombuffer(pixels, dtype=np.uint8).reshape(480, 640, 3)
                    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    bbox = detect_red_box(bgr)
                    assert bbox is not None, 'rendered red truck not detected'
                    # At 90 degrees HFOV, a 2.4 m rear at X=37.9 m spans about 20 px.
                    # This catches camera unit/FOV errors that detection alone would miss.
                    assert 16 <= bbox[2] <= 26, f'incorrect camera FOV: bbox={bbox}'
                    rendered_y = (5 if metadata['source']['step_index'] % 2 else -5) if args.moving_camera else 0
                    camera_positions_seen.add(rendered_y)
                    expected_center = 320 + rendered_y * 320 / 37.9
                    assert abs(bbox[0] + bbox[2] / 2 - expected_center) <= 3, f'image/pose mismatch: bbox={bbox}, expected center={expected_center}'
                    assert cv2.imwrite(str(args.report_dir / 'camera.png'), bgr)
                    arrivals.append(time.monotonic())
                    ages.append(arrivals[-1] - metadata['source']['measurement_us'] / 1e6)
                    if len(arrivals) >= 20:
                        assert not args.moving_camera or camera_positions_seen == {-5, 5}, 'probe must exercise both camera positions'
                        result = dict(passed=True, source=metadata['source'], bbox=list(bbox), image_bytes=len(pixels),
                                      frames=len(arrivals), fps=(len(arrivals) - 1) / (arrivals[-1] - arrivals[0]),
                                      max_source_age_ms=max(ages) * 1000, camera_y_values=sorted(camera_positions_seen))
                        break
            else:
                raise TimeoutError('No usable camera frame within 180 seconds')
    finally:
        reader.close()
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        (args.report_dir / 'report.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
