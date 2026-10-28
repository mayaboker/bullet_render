"""Truck geometry and acknowledged contact lifecycle, independent of USD/GPU."""
import math
from uuid import uuid4

from bullet_msgs.scenario import validate_collision

DRONE_RADIUS_M = 0.42
MAX_SWEEP_GAP_US = 1_000_000
CONTACT_RETRY_S = 0.1
CONTACT_TIMEOUT_S = 1.0
DEFAULT_TARGET_M = (40.0, 0.0, 0.0)
# Name, local centre (metres), full XYZ size (metres), display colour.
TRUCK_PARTS = (
    ('Cargo', (0, 0, 2.1), (4.2, 2.4, 2.2), (0.8, 0.015, 0.01)),
    ('Cab', (3.05, 0, 1.9), (1.8, 2.3, 2.0), (0.8, 0.015, 0.01)),
    ('Chassis', (1, 0, 0.85), (6, 2.2, 0.35), (0.06, 0.06, 0.06)),
)


def vector(value):
    if len(value) != 3 or any(type(x) not in (int, float) or not math.isfinite(x) for x in value):
        raise ValueError('expected three finite coordinates in metres')
    return tuple(value)


def truck_bounds(position):
    position = vector(position)
    return [(tuple(position[i] + centre[i] - size[i] / 2 for i in range(3)),
             tuple(position[i] + centre[i] + size[i] / 2 for i in range(3)))
            for _, centre, size, _ in TRUCK_PARTS]


def sphere_sweep(start, finish, bounds, radius=DRONE_RADIUS_M):
    """Earliest exact sphere/AABB contact; no expanded-box corner false hits."""
    delta = tuple(b - a for a, b in zip(start, finish))
    lower, upper = bounds
    cuts = {0.0, 1.0}
    for i in range(3):
        if delta[i]:
            for edge in (lower[i], upper[i]):
                t = (edge - start[i]) / delta[i]
                if 0 < t < 1:
                    cuts.add(t)
    cuts = sorted(cuts)
    for lo, hi in zip(cuts, cuts[1:]):
        midpoint = (lo + hi) / 2
        a = b = 0.0
        c = -radius * radius
        for i in range(3):
            p = start[i] + delta[i] * midpoint
            edge = lower[i] if p < lower[i] else upper[i] if p > upper[i] else None
            if edge is not None:
                offset = start[i] - edge
                a += delta[i] ** 2
                b += 2 * delta[i] * offset
                c += offset ** 2
        if a * lo * lo + b * lo + c <= 1e-12:
            return lo
        discriminant = b*b - 4*a*c
        if a and discriminant >= 0:
            root = (-b - math.sqrt(discriminant)) / (2*a)
            if lo <= root <= hi:
                return root
    return None


class ContactSession:
    """Pure geometry plus retry/confirmation state for one renderer controller."""
    def __init__(self, controller_id, position=DEFAULT_TARGET_M):
        self.controller_id = controller_id
        self.set_target(position)

    def set_target(self, position):
        self.bounds = truck_bounds(position)
        self.identity = None
        self.previous = self.pending = None
        self.started = self.retry = 0.0
        self.status = ''
        self.latched = False
        self.stopping = False

    def state(self, state):
        identity = (state['physics_id'], state['episode_id'])
        if identity != self.identity:
            self.previous = self.pending = None
            self.latched = False
            self.stopping = False
            self.status = ''
            self.identity = identity
        result = state.get('terminal_result')
        if state['state'] != 'running':
            self.stopping = False
        if result is not None:
            self.pending = None
            self.latched = True
            self.status = 'COMPLETED / Target hit'
        elif state['state'] != 'running' and self.pending:
            self.pending = None
            self.status = 'Contact not confirmed: physics stopped'

    def observe(self, pose, state, now):
        if (state is None or state['state'] != 'running' or self.latched
                or state['controller_id'] != self.controller_id or pose.get('schema_version') != 3):
            return
        source = pose['source']
        if ((source['physics_id'], source['episode_id']) != self.identity
                or not 0 <= now - source['measurement_us'] / 1e6 <= 1):
            return
        old = self.previous
        if old and old['source']['stream_id'] == source['stream_id'] and source['step_index'] <= old['source']['step_index']:
            return
        finish = vector(pose['position_m'])
        start = finish
        if (old and old['source']['stream_id'] == source['stream_id']
                and 0 < source['measurement_us'] - old['source']['measurement_us'] <= MAX_SWEEP_GAP_US):
            start = vector(old['position_m'])
        self.previous = pose
        hits = [t for bounds in self.bounds if (t := sphere_sweep(start, finish, bounds)) is not None]
        if not hits:
            return
        fraction = min(hits)
        self.pending = dict(schema_version=1, event_id=uuid4().hex, controller_id=self.controller_id,
            issued_us=int(now * 1e6), source=dict(source), target_id='red_truck',
            drone_position_m=[a + fraction * (b - a) for a, b in zip(start, finish)])
        validate_collision(self.pending)
        self.started, self.retry = now, 0.0
        self.latched = True
        self.status = 'Contact detected: waiting for physics confirmation'

    def tick(self, now):
        """Return (report, stop_requested); success only comes through state()."""
        if self.stopping:
            if now >= self.retry:
                self.retry = now + CONTACT_RETRY_S
                return None, True
            return None, False
        if self.pending is None or now < self.retry:
            return None, False
        if now - self.started >= CONTACT_TIMEOUT_S:
            self.pending = None
            self.stopping = True
            self.retry = now + CONTACT_RETRY_S
            self.status = 'Contact acknowledgement failed; stopping'
            return None, True
        self.retry = now + CONTACT_RETRY_S
        report = dict(self.pending, issued_us=int(now * 1e6))
        return validate_collision(report), False
