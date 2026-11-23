"""Real TCP and SHM checks for the Isaac renderer transport boundary."""
from contextlib import ExitStack
import json
import time

import pytest
import zmq

from bullet_image_transport.shm import FrameReader, decode_descriptor, topic_prefix
from isaac.transport import Receiver, Publisher, camera_pose, CONFIG, POSE


def config_message():
    return dict(schema_version=1, stream_id='stream', capture_fps_hz=30,
                camera_info=dict(width=2, height=2,
                    k=(10.0, 0., 1., 0., 10., 1., 0., 0., 1.),
                    p=(10.0, 0., 1., 0., 0., 10., 1., 0., 0., 0., 1., 0.),
                    header=dict(stamp=dict(sec=0, nanosec=0), frame_id='camera_front_optical'),
                    distortion_model='plumb_bob', d=(0.,)*5,
                    r=(1., 0., 0., 0., 1., 0., 0., 0., 1.),
                    binning_x=0, binning_y=0,
                    roi=dict(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False)))


def pose(fid=1):
    return dict(header=dict(fid=fid, time_stamp=dict(sec=0, nanosec=fid * 33_333_333)),
                drone_id='drone_1',
                position_m=[1, 2, 3],
                orientation_xyzw=[0, 0, 0, 1])


def bound(context, kind):
    socket = context.socket(kind)
    socket.setsockopt(zmq.LINGER, 0)
    port = socket.bind_to_random_port('tcp://127.0.0.1')
    return socket, f'tcp://127.0.0.1:{port}'


def test_camera_axes_and_bad_quaternion():
    value = pose()
    assert camera_pose(value) == ([1, 2, 3], [1, 0, 0, 0])
    value['orientation_xyzw'] = [0, 0, 0, 0]
    with pytest.raises(ValueError, match='quaternion'):
        camera_pose(value)


def test_real_tcp_pose_matching_duplicates_and_stale():
    with ExitStack() as stack:
        context = stack.enter_context(zmq.Context())
        telemetry, endpoint = bound(context, zmq.PUB)
        stack.callback(telemetry.close)
        receiver = Receiver(dict(telemetry_endpoint=endpoint))
        stack.callback(receiver.close)
        deadline = time.monotonic() + 3
        result = None
        while result is None and time.monotonic() < deadline:
            telemetry.send_multipart([CONFIG, json.dumps(config_message()).encode()])
            telemetry.send_multipart([POSE, json.dumps(pose()).encode()])
            time.sleep(0.02)
            result = receiver.poll()
        assert result is not None
        assert result[0]['header']['fid'] == 1
        # Duplicate fid is rejected.
        telemetry.send_multipart([POSE, json.dumps(pose()).encode()])
        time.sleep(0.04)
        assert receiver.poll() is None
        # Wrong topic is ignored.
        telemetry.send_multipart([POSE + b'/wrong', json.dumps(pose(99)).encode()])
        time.sleep(0.04)
        assert receiver.poll() is None


def test_rgb_round_trip_and_cleanup():
    with ExitStack() as stack:
        context = stack.enter_context(zmq.Context())
        reserve, endpoint = bound(context, zmq.PUB)
        reserve.close()
        publisher = Publisher(endpoint, dict(width_px=2, height_px=2))
        stack.callback(publisher.close)
        subscriber = stack.enter_context(context.socket(zmq.SUB))
        subscriber.setsockopt(zmq.SUBSCRIBE, topic_prefix('camera/front/image'))
        subscriber.connect(endpoint)
        reader = FrameReader('camera/front/image')
        stack.callback(reader.close)
        pixels = bytes(range(12))
        source_pose = pose()
        deadline = time.monotonic() + 3
        while not subscriber.poll(20) and time.monotonic() < deadline:
            assert publisher.publish(pixels, source_pose, time.monotonic())
        assert subscriber.poll(100)
        message = subscriber.recv()
        actual, copied = reader.read(message)
        assert copied == pixels
        assert actual['drone_id'] == 'drone_1'
        assert actual['header'] == source_pose['header']
        assert reader.read(message) is None
        assert not publisher.publish(pixels, source_pose, time.monotonic() - 1)


def test_config_stream_change_preserves_camera():
    with ExitStack() as stack:
        context = stack.enter_context(zmq.Context())
        telemetry, endpoint = bound(context, zmq.PUB)
        stack.callback(telemetry.close)
        receiver = Receiver(dict(telemetry_endpoint=endpoint))
        stack.callback(receiver.close)
        deadline = time.monotonic() + 3
        while receiver.config is None and time.monotonic() < deadline:
            telemetry.send_multipart([CONFIG, json.dumps(config_message()).encode()])
            time.sleep(0.02)
            receiver.poll()
        assert receiver.config is not None
        new_config = config_message()
        new_config['stream_id'] = 'stream2'
        telemetry.send_multipart([CONFIG, json.dumps(new_config).encode()])
        time.sleep(0.04)
        receiver.poll()
        assert receiver.config['stream_id'] == 'stream2'
