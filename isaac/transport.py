"""Bounded transport for the Isaac renderer; no Isaac imports or scene access.

Subscribes to pose and config telemetry from the launcher's RenderPublisher,
and publishes camera images back via shared memory. The launcher owns physics
lifecycle — this transport is a passive consumer.
"""
import json
import math
import time

import zmq

from bullet_image_transport.shm import FrameWriter, MAX_IMAGE_BYTES


CONFIG = b'camera/front/camera_info'
POSE = b'render/drone_1/pose'
PROFILE = b'render/drone_1/profile'
STATE = b'simulation/state'


def decode(payload):
    if not 0 < len(payload) <= 16384:
        raise ValueError('oversized or empty telemetry')
    def invalid(value):
        raise ValueError(f'nonfinite JSON: {value}')
    def number(value):
        result = float(value)
        if not math.isfinite(result):
            raise ValueError('nonfinite JSON number')
        return result
    return json.loads(payload, parse_constant=invalid, parse_float=number)


def camera_pose(pose):
    """Bullet FLU/Z-up to Isaac world camera axes; XYZW becomes WXYZ."""
    x, y, z, w = pose['orientation_xyzw']
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or abs(norm - 1) > 0.001:
        raise ValueError('pose quaternion must be unit length')
    return pose['position_m'], [w/norm, x/norm, y/norm, z/norm]


class Receiver:
    """Single-threaded subscriber; latest-only slots for each telemetry topic."""
    def __init__(self, profile):
        self.profile = profile
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.setsockopt(zmq.LINGER, 0)
        self.subscriber.setsockopt(zmq.RCVHWM, 64)
        self.subscriber.setsockopt(zmq.MAXMSGSIZE, 65536)
        for topic in (CONFIG, POSE):
            self.subscriber.setsockopt(zmq.SUBSCRIBE, topic)
        self.subscriber.connect(profile['telemetry_endpoint'])
        self.config = None
        self.retired = set()
        self.fid = -1

    def drain(self):
        """Read all pending messages into latest-only slots; returns slots dict."""
        slots = {}
        for _ in range(256):
            try:
                parts = self.subscriber.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                break
            if len(parts) == 2 and parts[0] in (CONFIG, POSE) and len(parts[1]) <= 16384:
                slots[parts[0]] = (parts[1], time.monotonic())
        return slots

    def poll(self):
        """Return (pose, received_at) for the latest renderable frame, or None."""
        slots = self.drain()
        if CONFIG in slots:
            config = decode(slots[CONFIG][0])
            if config['stream_id'] not in self.retired:
                if self.config and config['stream_id'] == self.config['stream_id']:
                    if config != self.config:
                        raise ValueError('camera config changed within a stream')
                else:
                    if self.config:
                        if config['camera_info'] != self.config['camera_info']:
                            raise ValueError('camera layout changed; restart renderer')
                        self.retired.add(self.config['stream_id'])
                        if len(self.retired) > 64:
                            raise RuntimeError('stream retirement limit reached; restart renderer')
                    camera = config['camera_info']
                    if camera['width'] * camera['height'] * 3 > MAX_IMAGE_BYTES:
                        raise ValueError('camera image exceeds SHM limit')
                    self.config, self.fid = config, -1
        if POSE not in slots or self.config is None:
            return None
        raw, received = slots[POSE]
        pose = decode(raw)
        fid = pose['header']['fid']
        if fid <= self.fid:
            return None
        camera_pose(pose)
        self.fid = fid
        if time.monotonic() - received > 0.1:
            return None
        return pose, received

    def flush(self):
        """Drain all buffered messages so the next poll() gets a fresh pose."""
        self.drain()
        self.fid = -1

    def close(self):
        self.subscriber.close(linger=0)
        self.context.term()


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
        header = pose.get('header', {})
        ts = header.get('time_stamp', {})
        capture_ns = ts.get('sec', 0) * 10**9 + ts.get('nanosec', 0)
        if capture_ns == 0:
            capture_ns = int(received * 1e9)
        message = self.writer.write('camera/front/image', pixels, capture_ns,
                                    header=header, drone_id=pose.get('drone_id', 'drone_1'),
                                    host_capture_ns=time.monotonic_ns())
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
