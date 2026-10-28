"""Deterministic mission/physics lease replay at measured camera latencies.

Checks that fresh images permit tracking and delayed images retain the expiry
watchdog. It does not render or change any running process.
"""
from collections import deque
import json
from pathlib import Path
import sys

from bullet_app.mission import Mission
from los_tracking import NavigatorConfig

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent / 'bullet_drone'))
from tools.zmq_render.flight import FlightLease


def replay(delay_s):
    config = NavigatorConfig.from_json(ROOT.parent / 'los_tracking/examples/bullet_profile.json')
    mission = Mission(config, app_id='app')
    vehicle, camera = config.vehicle, config.camera
    mission.configure(dict(schema_version=1, physics_id='physics', stream_id='stream',
        mass_kg=vehicle.mass_kg, gravity_m_s2=vehicle.gravity_m_s2, max_thrust_n=vehicle.max_thrust_n,
        rc_angle_scale_rad=vehicle.rc_angle_scale_rad, rc_yaw_rate_scale_rad_s=vehicle.rc_yaw_rate_scale_rad_s,
        aetr_channels=[1, 2, 3, 4], arm_channel=5, mode_channel=6, attitude_model='ideal',
        physics_hz=240, feedback_hz=120, command_hz=100,
        camera=dict(width_px=camera.width_px, height_px=camera.height_px, fov_x_deg=90)))
    now = 1000.0
    lease = FlightLease('physics', clock=lambda: now)
    lease.running, lease.owner = True, 'app'
    pending = deque()
    detection = command = None
    next_tick = now + 0.001  # Reproduce a nonaligned 100 Hz / 240 Hz phase.
    acquired = False
    minimum_us = 100_000
    for step in range(2400):
        now = 1000 + step / 240
        source = dict(physics_id='physics', episode_id='episode', stream_id='stream', step_index=step,
                      simulation_time_s=step / 240, measurement_us=int(now * 1e6))
        mission.observe(dict(schema_version=1, source=source, barometer_altitude_m=0 if step < 50 else 20,
            orientation_xyzw=[0, 0, 0, 1], armed=command is not None and command['channels'][4] >= 1500,
            mode='angle', last_applied_sequence=command['sequence'] if command else 0,
            owner_id='app', state='running', reason='test'))
        if step % 8 == 0:
            # Acquire normally, then apply the latency measured during approach.
            arrival = now + (delay_s if acquired else 0.03)
            pending.append((arrival, (dict(sequence=step + 1, source=source), (310, 365, 20, 35))))
        while pending and pending[0][0] <= now:
            detection = pending.popleft()[1]
        if now + 1e-9 >= next_tick:
            command = mission.tick(now_s=now, detection=detection)
            next_tick += 0.01
            lease.accept(command, 'episode')
            acquired |= mission.phase == 'LOS_APPROACH'
            if acquired:
                minimum_us = min(minimum_us, command['valid_until_us'] - command['issued_us'])
        if lease.latest is not None and lease.command() is None:
            return dict(camera_delay_ms=delay_s * 1000, acquired=acquired,
                        stop_reason=lease.reason, minimum_command_ms=minimum_us / 1000)
    return dict(camera_delay_ms=delay_s * 1000, acquired=acquired,
                stop_reason=None, minimum_command_ms=minimum_us / 1000)


if __name__ == '__main__':
    fresh, delayed = replay(0.03), replay(0.07)
    assert fresh['acquired'] and fresh['stop_reason'] is None, fresh
    assert delayed['acquired'] and delayed['stop_reason'] == 'rc_expired', delayed
    print(json.dumps(dict(fresh=fresh, delayed=delayed), indent=2))
