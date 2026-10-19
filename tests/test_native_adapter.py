"""Exercise the compiled adapter through real Godot and ZeroMQ sessions."""
from collections import deque
from contextlib import ExitStack
import json
from pathlib import Path
from queue import Empty, Queue
import signal
import subprocess
from threading import Thread
import time

import pytest
import zmq
from uuid import uuid4
from bullet_msgs.shm import FrameReader, decode_descriptor, topic_prefix

ROOT = Path(__file__).resolve().parents[1]

def tcp_profile():
    import socket
    with socket.socket() as telemetry, socket.socket() as images, socket.socket() as control:
        telemetry.bind(('127.0.0.1', 0))
        images.bind(('127.0.0.1', 0))
        control.bind(('127.0.0.1', 0))
        return {'telemetry_endpoint': f'tcp://127.0.0.1:{telemetry.getsockname()[1]}',
                'image_endpoint': f'tcp://127.0.0.1:{images.getsockname()[1]}',
                'control_endpoint': f'tcp://127.0.0.1:{control.getsockname()[1]}'}



def config(stream='a', width=2, height=2):
    return {'schema_version': 1, 'stream_id': stream, 'camera': {
        'width_px': width, 'height_px': height, 'fov_x_deg': 90}, 'fps': 30}


def pose(sequence=0, stream='a', episode='first'):
    return {'schema_version': 2, 'stream_id': stream, 'drone_id': 'drone_1',
            'episode_id': episode, 'sequence': sequence, 'publisher_drops': 0,
            'step_index': sequence * 8, 'simulation_time_s': sequence / 30,
            'position_m': [0, 0, 3], 'orientation_xyzw': [0, 0, 0, 1],
            'linear_velocity_m_s': [1.25, -2.5, 0.75]}


def wait_for(predicate, timeout=8):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, 'ZeroMQ event timed out'
        time.sleep(0.01)


class Probe:
    def __init__(self, process, settings, poses, samples):
        self.process, self.settings, self.poses, self.image_socket = process, settings, poses, samples
        self.images = []
        self.reader = FrameReader("camera/front/image")
        self.events = Queue()
        self.log = deque(maxlen=50)
        self.sessions = set()
        def read():
            for line in process.stdout:
                self.log.append(line)
                if line.startswith('SHM publisher session: '):
                    self.sessions.add(line.strip().split(': ', 1)[1])
                if line.startswith('PROBE '):
                    self.events.put(json.loads(line[6:]))
        self.log_reader = Thread(target=read, daemon=True)
        self.log_reader.start()

    @property
    def samples(self):
        while self.image_socket.poll(0):
            message = self.image_socket.recv()
            assert not self.image_socket.getsockopt(zmq.RCVMORE)
            result = self.reader.read(message)
            if result is not None:
                self.images.append(result[1])
        return self.images

    def event(self, predicate):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            try:
                value = self.events.get(timeout=0.1)
            except Empty:
                if self.process.poll() is not None:
                    break
                continue
            if predicate(value):
                return value
        raise AssertionError('Native adapter event missing:\n' + ''.join(self.log))

    def capture(self, sequence=0, stream='a', episode='first'):
        self.poses.put(json.dumps(pose(sequence, stream, episode)))
        return self.event(lambda e: e.get('sequence') == sequence and e.get('stream_id') == stream)


class Publisher:
    def __init__(self, socket, topic):
        self.socket, self.topic = socket, topic.encode()

    def put(self, payload):
        if isinstance(payload, str): payload = payload.encode()
        self.socket.send_multipart([self.topic, payload])


@pytest.fixture
def probe(tmp_path):
    with ExitStack() as stack:
        def start(*, invalid_profile=False, delay=0, close_after=0, image_hwm=8, image_endpoint=None, control_action=''):
            identity = uuid4().hex
            transport = tcp_profile()
            observer_endpoint = transport["image_endpoint"]
            if image_endpoint is not None:
                transport["image_endpoint"] = image_endpoint
            profile = tmp_path/(identity+'.json')
            profile.write_text(json.dumps({} if invalid_profile else transport))
            context = stack.enter_context(zmq.Context())
            publisher = stack.enter_context(context.socket(zmq.XPUB))
            publisher.setsockopt(zmq.LINGER, 0)
            publisher.bind(transport['telemetry_endpoint'])
            observer = stack.enter_context(context.socket(zmq.SUB))
            observer.setsockopt(zmq.LINGER, 0)
            observer.setsockopt(zmq.RCVHWM, image_hwm)
            observer.setsockopt(zmq.SUBSCRIBE, topic_prefix('camera/front/image'))
            observer.connect(observer_endpoint)
            process = subprocess.Popen(['godot','--headless','--path',str(ROOT/'godot'),
                '--script','adapter_probe.gd','--',str(profile),'unused',str(delay),str(close_after),control_action],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            def close():
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try: process.wait(5)
                    except subprocess.TimeoutExpired:
                        process.kill(); process.wait()
                result.log_reader.join(timeout=2)
                process.stdout.close()
                # SIGINT is abrupt in this Godot build, reap only this stopped probe's sessions.
                for session in result.sessions:
                    if len(session) == 32 and all(c in '0123456789abcdef' for c in session):
                        for index in range(3):
                            (Path('/dev/shm') / f'bullet_msgs_{session}_s{index}').unlink(missing_ok=True)
            stack.callback(close)
            result = Probe(process, Publisher(publisher, 'render/drone_1/config'),
                           Publisher(publisher, 'render/drone_1/pose'), observer)
            stack.callback(result.reader.close)
            if not invalid_profile and image_endpoint is None:
                result.event(lambda e: e.get('ready'))
                assert publisher.poll(5000)
                assert publisher.recv() == b'\x01render/drone_1/'
                time.sleep(0.3)  # Image PUB connection has no subscription acknowledgement.
            return result
        yield start


def test_direct_tcp_pose_and_images_with_stream_restart(probe):
    native = probe()
    native.settings.put(json.dumps(config()))
    captured = native.capture()
    assert captured['stats']['frames'] == 1
    assert captured['linear_velocity_m_s'] == [1.25, -2.5, 0.75]
    wait_for(lambda: len(native.samples) == 1)
    assert native.samples[0] == bytes([42])*12
    native.poses.put(json.dumps(pose()))  # Duplicate must not publish again.
    time.sleep(0.1)
    assert len(native.samples) == 1
    native.settings.put(json.dumps(config()))  # Periodic config is idempotent.
    assert native.capture(1, episode='second')['stats']['frames'] == 2
    native.settings.put(json.dumps(config('b')))
    assert native.capture(0, 'b')['stats']['frames'] == 3
    native.settings.put(json.dumps(config('a')))  # Retired stream must stay retired.
    assert native.capture(1, 'b')['stats']['frames'] == 4


def test_invalid_data_and_immutable_configuration(probe):
    native = probe()
    native.settings.put(json.dumps(config()))
    assert native.capture()['success']
    for bad in [dict(pose(1), orientation_xyzw=[0,0,0,0]),
                dict(pose(1), sequence=-1), dict(pose(1), surprise=True),
                dict(pose(1), position_m=[0,1e999,0]), dict(pose(1), schema_version=True),
                dict(pose(1), position_m=[0,0]), {'missing':'fields'}]:
        native.poses.put(json.dumps(bad))
        native.event(lambda e: e.get('invalid',0) > 0)
        time.sleep(0.03)
    for raw in [b'', b'not-json', b'x' * 17000]:
        native.poses.put(raw)
        time.sleep(0.03)
    native.settings.put(json.dumps(dict(config(), fps=60)))
    assert native.event(lambda e: e.get('invalid',0) >= 11)['frames'] == 1
    assert native.capture(2)['stats']['frames'] == 2
    native.settings.put(json.dumps(config('b', width=4)))
    assert 'Camera settings changed' in native.event(lambda e: bool(e.get('error')))['error']


def test_stale_capture_is_discarded(probe):
    native = probe(delay=1.2)
    native.settings.put(json.dumps(config()))
    result = native.capture()
    assert result['success'] and result['stats']['stale'] == 1
    assert result['stats']['frames'] == 0 and not native.samples


def test_invalid_transport_profile_fails_startup(probe):
    native = probe(invalid_profile=True)
    assert 'invalid transport profile' in native.event(lambda e: bool(e.get('error')))['error']
    assert native.process.wait(5) == 1


def test_slow_consumer_does_not_block_rendering(probe):
    native = probe(image_hwm=1)
    native.settings.put(json.dumps(config(width=640, height=480)))
    for sequence in range(30):
        assert native.capture(sequence)['success']
    assert not native.process.poll()
    native.samples  # Drains descriptors that may now refer to overwritten slots.
    assert native.capture(30)['stats']['frames'] == 31
    wait_for(lambda: bool(native.samples))


def test_topic_prefix_is_not_an_exact_match_and_malformed_multipart(probe):
    native = probe()
    native.settings.put(json.dumps(config()))
    native.poses.socket.send_multipart([b'render/drone_1/pose/extra', json.dumps(pose()).encode()])
    time.sleep(0.05)
    assert not native.samples
    native.poses.socket.send_multipart([b'render/drone_1/pose', b'{}', b'extra'])
    native.event(lambda e: e.get('invalid', 0) > 0)
    assert native.capture()['success']


def test_endpoint_collision_fails_without_displacing_publisher(probe, tmp_path):
    native = probe()
    # Starting a second adapter with the first adapter's endpoint must fail.
    profile = next(tmp_path.glob('*.json'))
    result = subprocess.run(['godot','--headless','--path',str(ROOT/'godot'),
        '--script','adapter_probe.gd','--',str(profile)],capture_output=True,text=True,timeout=10)
    assert result.returncode == 1 and 'Address already in use' in result.stdout
    native.settings.put(json.dumps(config()))
    assert native.capture()['success']


def test_only_latest_pose_is_kept_while_capture_is_busy(probe):
    native = probe(delay=0.2)
    native.settings.put(json.dumps(config()))
    native.poses.put(json.dumps(pose()))
    native.event(lambda e: e.get('capturing') == 0)
    for sequence in range(1, 101):
        native.poses.put(json.dumps(pose(sequence)))
    result = native.event(lambda e: e.get('sequence') == 100)
    assert result['success']
    assert result['stats']['superseded'] >= 1
    assert result['stats']['sequence_gaps'] == 99
    assert result['stats']['frames'] == 2


def test_cold_editor_import(tmp_path):
    (tmp_path/'project.godot').write_text('config_version=5\n[application]\nconfig/name="Native import test"\n')
    library = ROOT/'godot/bin/libzmq_camera.so'
    (tmp_path/'camera.gdextension').write_text(
        '[configuration]\nentry_symbol="zmq_camera_library_init"\ncompatibility_minimum="4.5"\n'
        '[libraries]\nlinux.x86_64="'+str(library)+'"\n')
    result = subprocess.run(['godot','--headless','--path',str(tmp_path),'--editor',
        '--import','--frame-delay','1000','--quit'], stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, timeout=30)
    assert result.returncode == 0, result.stdout


def test_graceful_close_is_idempotent(probe):
    native = probe(close_after=1)
    native.settings.put(json.dumps(config()))
    assert native.capture()['success']
    closed = native.event(lambda e: e.get('closed'))
    assert not closed['stats']['running'] and not closed['stats']['error']
    assert native.process.wait(5) == 0


def test_multiple_image_consumers_receive_the_same_frame(probe):
    native = probe()
    with native.image_socket.context.socket(zmq.SUB) as second:
        second.setsockopt(zmq.LINGER, 0)
        second.setsockopt(zmq.SUBSCRIBE, topic_prefix('camera/front/image'))
        second.connect(native.image_socket.getsockopt_string(zmq.LAST_ENDPOINT))
        time.sleep(0.3)
        native.settings.put(json.dumps(config()))
        assert native.capture()['success']
        assert second.poll(5000)
        reader = FrameReader('camera/front/image')
        try:
            message = second.recv()
            assert len(message) < 1024
            assert reader.read(message)[1] == bytes([42])*12
        finally:
            reader.close()
        wait_for(lambda: len(native.samples) == 1)
        assert native.samples[0] == bytes([42])*12


def test_native_busy_slots_are_skipped_and_unlinked(probe):
    import fcntl
    import os
    native = probe(close_after=2)
    native.settings.put(json.dumps(config()))
    first = native.capture()
    session = first['stats']['shm_session']
    paths = [Path('/dev/shm') / f'bullet_msgs_{session}_s{i}' for i in range(3)]
    fds = [os.open(path, os.O_RDONLY) for path in paths]
    try:
        for fd in fds:
            fcntl.flock(fd, fcntl.LOCK_SH)
        result = native.capture(1)
        assert result['success'] and result['stats']['busy_slots'] == 1
        assert result['stats']['frames'] == 1
        fcntl.flock(fds[1], fcntl.LOCK_UN)
        assert native.capture(2)['stats']['frames'] == 2
    finally:
        for fd in fds:
            os.close(fd)
    native.event(lambda e: e.get('closed'))
    assert native.process.wait(5) == 0
    assert not any(path.exists() for path in paths)


@pytest.mark.parametrize('endpoint', ['ipc://@old', 'tcp://0.0.0.0:5556',
    'tcp://127.0.0.1:65536', 'tcp://127.0.0.1:5556\0extra'])
def test_native_rejects_invalid_tcp_endpoint(probe, endpoint):
    native = probe(image_endpoint=endpoint)
    assert 'loopback TCP endpoint' in native.event(lambda e: bool(e.get('error')))['error']
    assert native.process.wait(5) == 1


def test_native_control_fanout_and_authoritative_state(probe, tmp_path):
    from bullet_msgs.control import validate_command, HEARTBEAT_TOPIC, COMMAND_TOPIC
    native = probe(control_action='start')
    profile = json.loads(next(tmp_path.glob('*.json')).read_text())
    with native.image_socket.context.socket(zmq.SUB) as first, native.image_socket.context.socket(zmq.SUB) as second:
        for socket in (first, second):
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.SUBSCRIBE, b'simulation/')
            socket.connect(profile['control_endpoint'])
        assert first.poll(5000)
        topic, payload = first.recv_multipart()
        assert topic == HEARTBEAT_TOPIC
        heartbeat = validate_command(json.loads(payload))
        state = dict(schema_version=1, physics_id='physics', controller_id=heartbeat['controller_id'],
                     episode_id='episode', state='stopped', reason='startup', simulation_time_s=0)
        Publisher(native.settings.socket, 'simulation/state').put(json.dumps(state))
        assert native.event(lambda e: e.get('control_state', {}).get('state') == 'stopped')['control_state'] == state
        def command(socket):
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                assert socket.poll(5000)
                topic, payload = socket.recv_multipart()
                value = validate_command(json.loads(payload))
                if topic == COMMAND_TOPIC:
                    return value
            raise AssertionError('native command was not broadcast')
        sent = command(first)
        assert command(second) == sent
        assert sent['action'] == 'start' and sent['sequence'] > heartbeat['sequence']
        bad = dict(state, state='optimistic')
        Publisher(native.settings.socket, 'simulation/state').put(json.dumps(bad))
        native.event(lambda e: e.get('control_error'))
        native.event(lambda e: e.get('control_state') == {})


def test_pose_velocity_required_and_validated(probe):
    native = probe()
    native.settings.put(json.dumps(config()))
    assert native.capture()['success']
    missing = pose(1)
    missing.pop('linear_velocity_m_s')
    bad_messages = [missing, dict(pose(1), schema_version=1)]
    bad_messages += [dict(pose(1), linear_velocity_m_s=value) for value in
                     [[0, 0], [0, 0, 0, 0], [True, 0, 0], ['1', 0, 0],
                      [1e10, 0, 0], [float('inf'), 0, 0]]]
    for invalid_count, message in enumerate(bad_messages, start=1):
        native.poses.put(json.dumps(message))
        native.event(lambda e: e.get('invalid', 0) >= invalid_count)
    result = native.capture(1)
    assert result['success'] and result['stats']['frames'] == 2
    assert result['linear_velocity_m_s'] == [1.25, -2.5, 0.75]
