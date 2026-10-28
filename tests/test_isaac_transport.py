"""Real TCP and SHM checks for the renderer-independent Isaac boundary."""
from contextlib import ExitStack
import importlib.util
import json
from pathlib import Path
import time

import pytest
import zmq

from bullet_msgs.shm import FrameReader, decode_descriptor, topic_prefix

spec = importlib.util.spec_from_file_location('isaac_transport', Path(__file__).resolve().parents[1] / 'isaac/transport.py')
transport = importlib.util.module_from_spec(spec)
spec.loader.exec_module(transport)


def config():
    return dict(schema_version=1, stream_id='stream', fps=30,
                camera=dict(width_px=2, height_px=2, fov_x_deg=90))


def pose(sequence=1):
    source = dict(physics_id='physics', episode_id='episode', stream_id='stream',
                  step_index=sequence, simulation_time_s=sequence / 30,
                  measurement_us=time.monotonic_ns() // 1000)
    return dict(schema_version=3, stream_id='stream', drone_id='drone_1', episode_id='episode',
                sequence=sequence, publisher_drops=0, step_index=sequence,
                simulation_time_s=sequence / 30, position_m=[1, 2, 3],
                orientation_xyzw=[0, 0, 0, 1], linear_velocity_m_s=[0, 0, 0], source=source)


def bound(context, kind):
    socket = context.socket(kind)
    socket.setsockopt(zmq.LINGER, 0)
    port = socket.bind_to_random_port('tcp://127.0.0.1')
    return socket, f'tcp://127.0.0.1:{port}'


def test_camera_axes_and_bad_quaternion():
    value = pose()
    assert transport.camera_pose(value) == ([1, 2, 3], [1, 0, 0, 0])
    value['orientation_xyzw'] = [0, 0, 0, 0]
    with pytest.raises(ValueError, match='quaternion'):
        transport.camera_pose(value)


def test_real_tcp_pose_matching_duplicates_stale_and_heartbeat():
    with ExitStack() as stack:
        context = stack.enter_context(zmq.Context())
        telemetry, endpoint = bound(context, zmq.PUB)
        stack.callback(telemetry.close)
        reserve, control_endpoint = bound(context, zmq.PUB)
        reserve.close()
        receiver = transport.Receiver(dict(telemetry_endpoint=endpoint, control_endpoint=control_endpoint))
        stack.callback(receiver.close)
        control = stack.enter_context(context.socket(zmq.SUB))
        control.setsockopt(zmq.SUBSCRIBE, b'simulation/')
        control.connect(control_endpoint)
        deadline = time.monotonic() + 3
        result = None
        while result is None and time.monotonic() < deadline:
            telemetry.send_multipart([transport.CONFIG, json.dumps(config()).encode()])
            telemetry.send_multipart([transport.POSE, json.dumps(pose()).encode()])
            time.sleep(0.02)
            result = receiver.poll()
        assert result is not None
        assert result[0]['sequence'] == 1
        assert control.poll(1500)
        topic, payload = control.recv_multipart()
        assert topic == b'simulation/heartbeat'
        assert json.loads(payload)['action'] == 'heartbeat'

        telemetry.send_multipart([transport.POSE + b'/wrong', json.dumps(pose(99)).encode()])
        time.sleep(0.04)
        assert receiver.poll() is None
        telemetry.send_multipart([transport.POSE, json.dumps(pose()).encode()])
        time.sleep(0.04)
        assert receiver.poll() is None
        with receiver.lock:
            receiver.slots[transport.POSE] = (json.dumps(pose(2)).encode(), time.monotonic() - 1)
        assert receiver.poll() is None
        bad = pose(3)
        bad['source']['step_index'] = 99
        with receiver.lock:
            receiver.slots[transport.POSE] = (json.dumps(bad).encode(), time.monotonic())
        with pytest.raises(ValueError, match='identity mismatch'):
            receiver.poll()


def test_rgb_round_trip_source_timestamp_expiry_and_cleanup():
    with ExitStack() as stack:
        context = stack.enter_context(zmq.Context())
        reserve, endpoint = bound(context, zmq.PUB)
        reserve.close()
        publisher = transport.Publisher(endpoint, config()['camera'])
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
        metadata = decode_descriptor('camera/front/image', message)
        actual, copied = reader.read(message)
        assert copied == pixels
        assert actual['source'] == source_pose['source']
        assert actual['capture_ns'] == source_pose['source']['measurement_us'] * 1000
        assert reader.read(message) is None
        assert not publisher.publish(pixels, source_pose, time.monotonic() - 1)
        path = Path('/dev/shm' + metadata['shm_name'])
        assert path.exists()
    assert not path.exists()


def test_spawn_apply_waits_for_matching_physics_confirmation():
    with ExitStack() as stack:
        context = stack.enter_context(zmq.Context())
        telemetry, endpoint = bound(context, zmq.PUB)
        stack.callback(telemetry.close)
        reserve, control_endpoint = bound(context, zmq.PUB)
        reserve.close()
        receiver = transport.Receiver(dict(telemetry_endpoint=endpoint, control_endpoint=control_endpoint))
        stack.callback(receiver.close)
        subscriber = stack.enter_context(context.socket(zmq.SUB))
        subscriber.setsockopt(zmq.SUBSCRIBE, b'simulation/')
        subscriber.connect(control_endpoint)
        state = dict(schema_version=1, physics_id='physics', controller_id=receiver.controller_id,
                     episode_id='episode', state='stopped', reason='startup', simulation_time_s=0.)
        deadline = time.monotonic() + 3
        while receiver.state is None and time.monotonic() < deadline:
            telemetry.send_multipart([transport.STATE, json.dumps(state).encode()])
            time.sleep(.02)
            receiver.poll()
        assert receiver.state is not None
        request_id = receiver.apply_spawn([3, 4, 5], 6)
        assert receiver.pending_spawn and receiver.spawn_state is None
        with pytest.raises(ValueError, match='wait for physics'):
            receiver.command('start')
        requests = []
        deadline = time.monotonic() + 3
        while not requests and time.monotonic() < deadline:
            if subscriber.poll(100):
                topic, payload = subscriber.recv_multipart()
                if topic == transport.SPAWN_REQUEST:
                    requests.append(json.loads(payload))
        assert requests and requests[0]['request_id'] == request_id
        assert requests[0]['takeoff_height_m'] == 6
        ack = dict(schema_version=2, physics_id='physics', controller_id=receiver.controller_id,
                   episode_id='new_episode', scenario_name='Custom', position_m=[3., 4., 5.],
                   takeoff_height_m=6., request_id='wrong_request', result='applied', reason='spawn_applied')
        telemetry.send_multipart([transport.SPAWN_STATE, json.dumps(ack).encode()])
        time.sleep(.04)
        receiver.poll()
        assert receiver.pending_spawn
        ack['request_id'] = request_id
        deadline = time.monotonic() + 3
        while receiver.pending_spawn and time.monotonic() < deadline:
            telemetry.send_multipart([transport.SPAWN_STATE, json.dumps(ack).encode()])
            time.sleep(.02)
            receiver.poll()
        assert receiver.pending_spawn is None
        assert receiver.spawn_state['takeoff_height_m'] == 6


@pytest.mark.parametrize('payload', [b'{', b'{"schema_version": NaN}', b'{"fps": 1e999}', b'x' * 16385])
def test_malformed_and_nonfinite_json(payload):
    from jsonschema import Draft202012Validator
    with pytest.raises((ValueError, UnicodeError)):
        transport.decode(payload, Draft202012Validator({}))
