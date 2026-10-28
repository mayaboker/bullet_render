"""Bounded transport for the Isaac renderer; no Isaac imports or scene access."""
import json
import math
import queue
import threading
import time
from uuid import uuid4

import zmq
from jsonschema import Draft202012Validator

from bullet_msgs import schema_bytes
from bullet_msgs.control import ControlPublisher, validate_state
from bullet_msgs.flight import validate_source
from bullet_msgs.shm import FrameWriter, MAX_IMAGE_BYTES
from bullet_msgs.scenario import COLLISION_TOPIC
from bullet_msgs.spawn import STATE_TOPIC as SPAWN_STATE, REQUEST_TOPIC as SPAWN_REQUEST, validate_request, validate_state as validate_spawn
from isaac.scenario import ContactSession, DEFAULT_TARGET_M, vector


CONFIG = b'render/drone_1/config'
POSE = b'render/drone_1/pose'
STATE = b'simulation/state'


def decode(payload, schema):
    if not 0 < len(payload) <= 16384:
        raise ValueError('oversized or empty telemetry')
    def invalid(value):
        raise ValueError(f'nonfinite JSON: {value}')
    def number(value):
        result = float(value)
        if not math.isfinite(result):
            raise ValueError('nonfinite JSON number')
        return result
    value = json.loads(payload, parse_constant=invalid, parse_float=number)
    schema.validate(value)
    return value


def camera_pose(pose):
    """Bullet FLU/Z-up to Isaac world camera axes; XYZW becomes WXYZ."""
    x, y, z, w = pose['orientation_xyzw']
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or abs(norm - 1) > 0.001:
        raise ValueError('pose quaternion must be unit length')
    return pose['position_m'], [w/norm, x/norm, y/norm, z/norm]


class Receiver:
    """One socket-owning worker, latest-only raw slots, bounded command queue."""
    def __init__(self, profile, target_position=DEFAULT_TARGET_M):
        self.profile = profile
        self.commands = queue.Queue(maxsize=8)
        self.lock = threading.Lock()
        self.slots = {}
        self.error = None
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.config = None
        self.state = None
        self.state_received = 0
        self.controller_id = None
        self.spawn_state = None
        self.spawn_received = 0
        self.pending_spawn = None
        self.spawn_notice = ''
        self.contact_status = ''
        self.target_position = vector(target_position)
        self.retired = set()
        self.sequence = -1
        self.validators = {topic: Draft202012Validator(json.loads(schema_bytes(name)))
                           for topic, name in ((CONFIG, 'config.json'), (POSE, 'pose.json'), (STATE, 'state.json'), (SPAWN_STATE, 'spawn_state.json'))}
        self.thread = threading.Thread(target=self._run, name='isaac-transport', daemon=True)
        self.thread.start()
        if not self.ready.wait(5):
            self.close()
            raise RuntimeError('transport startup timed out')
        self.check()

    def check(self):
        if self.error is not None:
            raise RuntimeError('Isaac transport failed') from self.error

    def command(self, action):
        if action in ('start', 'reset') and self.pending_spawn:
            raise ValueError('wait for physics to confirm scenario settings')
        self.commands.put_nowait(action)

    def set_target(self, position):
        self.commands.put_nowait(('target', vector(position)))

    def apply_spawn(self, position, takeoff_height, name='Custom'):
        if (not self.state or self.state['state'] != 'stopped' or time.monotonic() - self.state_received >= 3
                or self.state['controller_id'] != self.controller_id or self.pending_spawn):
            raise ValueError('scenario settings require connected, stopped physics and no pending Apply')
        packet = dict(schema_version=2, controller_id=self.controller_id, physics_id=self.state['physics_id'],
            episode_id=self.state['episode_id'], request_id=uuid4().hex, issued_ms=int(time.monotonic() * 1000),
            scenario_name=name, position_m=list(position), takeoff_height_m=takeoff_height)
        validate_request(packet)
        self.commands.put_nowait(('spawn', packet))
        self.pending_spawn = packet
        self.spawn_notice = 'Waiting for physics to confirm settings'
        return packet['request_id']

    def _run(self):
        try:
            with zmq.Context() as context, context.socket(zmq.SUB) as subscriber:
                subscriber.setsockopt(zmq.LINGER, 0)
                subscriber.setsockopt(zmq.RCVHWM, 64)
                subscriber.setsockopt(zmq.MAXMSGSIZE, 65536)
                for topic in (CONFIG, POSE, STATE, SPAWN_STATE):
                    subscriber.setsockopt(zmq.SUBSCRIBE, topic)
                subscriber.connect(self.profile['telemetry_endpoint'])
                with ControlPublisher(self.profile['control_endpoint']) as control:
                    self.controller_id = control.controller_id
                    contact = ContactSession(control.controller_id, self.target_position)
                    state = None
                    spawn_outgoing = None
                    spawn_started = spawn_retry = 0.0
                    self.ready.set()
                    heartbeat = 0
                    while not self.stop.is_set():
                        now = time.monotonic()
                        if now >= heartbeat:
                            control.heartbeat()
                            heartbeat = now + 0.5
                        try:
                            action = self.commands.get_nowait()
                            if isinstance(action, str):
                                if not (action in ('start', 'reset') and self.pending_spawn):
                                    control.send(action)
                            elif action[0] == 'target':
                                contact.set_target(action[1])
                            elif action[0] == 'spawn':
                                spawn_outgoing = action[1]
                                spawn_started, spawn_retry = now, 0.0
                        except queue.Empty:
                            pass
                        if spawn_outgoing and self.pending_spawn:
                            if now - spawn_started >= 4.5:
                                self.spawn_notice = 'Apply unconfirmed; inspect applied settings before retrying'
                                self.pending_spawn = spawn_outgoing = None
                            elif now - spawn_started < 2 and now >= spawn_retry:
                                packet = dict(spawn_outgoing, issued_ms=int(now * 1000))
                                control.socket.send_multipart([SPAWN_REQUEST, json.dumps(packet).encode()], zmq.NOBLOCK)
                                spawn_retry = now + 0.1
                        report, stop_requested = contact.tick(now)
                        if report is not None:
                            control.socket.send_multipart([COLLISION_TOPIC, json.dumps(report).encode()], zmq.NOBLOCK)
                        if stop_requested:
                            control.send('stop')
                        self.contact_status = contact.status
                        if subscriber.poll(10):
                            # Bound each drain so telemetry cannot starve heartbeats.
                            for _ in range(64):
                                try:
                                    parts = subscriber.recv_multipart(zmq.NOBLOCK)
                                except zmq.Again:
                                    break
                                if len(parts) == 2 and parts[0] in (CONFIG, POSE, STATE, SPAWN_STATE) and len(parts[1]) <= 16384:
                                    topic, raw = parts
                                    if topic == STATE:
                                        state = validate_state(decode(raw, self.validators[STATE]))
                                        contact.state(state)
                                    elif topic == POSE and state is not None and state['state'] == 'running':
                                        pose = decode(raw, self.validators[POSE])
                                        camera_pose(pose)
                                        if pose.get('schema_version') == 3:
                                            source = validate_source(pose['source'])
                                            if any(source[k] != pose[k] for k in ('episode_id', 'stream_id', 'step_index', 'simulation_time_s')):
                                                raise ValueError('pose/source identity mismatch')
                                        contact.observe(pose, state, time.monotonic())
                                    elif topic == SPAWN_STATE:
                                        spawn = validate_spawn(decode(raw, self.validators[SPAWN_STATE]))
                                        if (self.pending_spawn and spawn['request_id'] == self.pending_spawn['request_id']
                                                and spawn['physics_id'] == self.pending_spawn['physics_id']
                                                and spawn['controller_id'] == control.controller_id):
                                            self.spawn_notice = f"{spawn['result']}: {spawn['reason']}"
                                            self.pending_spawn = spawn_outgoing = None
                                    with self.lock:
                                        self.slots[parts[0]] = (parts[1], time.monotonic())
                    control.send('stop')
        except Exception as error:
            self.error = error
            self.ready.set()

    def poll(self):
        self.check()
        with self.lock:
            slots, self.slots = self.slots, {}
        if STATE in slots:
            raw, received = slots[STATE]
            self.state = validate_state(decode(raw, self.validators[STATE]))
            self.state_received = received
        if SPAWN_STATE in slots:
            raw, received = slots[SPAWN_STATE]
            self.spawn_state = validate_spawn(decode(raw, self.validators[SPAWN_STATE]))
            self.spawn_received = received
        if CONFIG in slots:
            config = decode(slots[CONFIG][0], self.validators[CONFIG])
            if config['stream_id'] not in self.retired:
                if self.config and config['stream_id'] == self.config['stream_id']:
                    if config != self.config:
                        raise ValueError('camera config changed within a stream')
                else:
                    if self.config:
                        if config['camera'] != self.config['camera']:
                            raise ValueError('camera layout changed; restart renderer and recorder')
                        self.retired.add(self.config['stream_id'])
                        if len(self.retired) > 64:
                            raise RuntimeError('stream retirement limit reached; restart renderer')
                    camera = config['camera']
                    if camera['width_px'] * camera['height_px'] * 3 > MAX_IMAGE_BYTES:
                        raise ValueError('camera image exceeds SHM limit')
                    self.config, self.sequence = config, -1
        if POSE not in slots or self.config is None:
            return None
        raw, received = slots[POSE]
        pose = decode(raw, self.validators[POSE])
        if pose['stream_id'] != self.config['stream_id'] or pose['sequence'] <= self.sequence:
            return None
        camera_pose(pose)
        if pose['schema_version'] == 3:
            source = validate_source(pose['source'])
            if any(source[key] != pose[key] for key in ('episode_id', 'stream_id', 'step_index', 'simulation_time_s')):
                raise ValueError('pose/source identity mismatch')
        self.sequence = pose['sequence']
        if time.monotonic() - received > 0.1:
            return None
        if self.state and self.state['episode_id'] != pose['episode_id']:
            return None
        return pose, received

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)


class Publisher:
    def __init__(self, endpoint, camera):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.writer = None
        try:
            self.socket.setsockopt(zmq.LINGER, 0)
            self.socket.setsockopt(zmq.SNDHWM, 8)
            self.socket.bind(endpoint)
            self.writer = FrameWriter(camera['width_px'], camera['height_px'])
        except BaseException:
            self.close()
            raise

    def publish(self, pixels, pose, received):
        if time.monotonic() - received > 0.1:
            return False
        source = pose.get('source')
        capture_ns = source['measurement_us'] * 1000 if source else int(received * 1e9)
        message = self.writer.write('camera/front/image', pixels, capture_ns, source=source)
        if message is None:
            return False
        try:
            self.socket.send(message, zmq.NOBLOCK)
        except zmq.Again:
            return False
        return True

    def close(self):
        if self.writer is not None:
            self.writer.close()
        self.socket.close(linger=0)
        self.context.term()
