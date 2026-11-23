"""Contact geometry and confirmation must not invent a successful episode."""
import pytest

from isaac.scenario import ContactSession, sphere_sweep


def state(episode='episode', **updates):
    return dict(physics_id='physics', episode_id=episode, controller_id='controller',
                state='running', terminal_result=None, **updates)


def pose(position, step=1, episode='episode', now=10):
    return dict(schema_version=3, position_m=position,
                source=dict(physics_id='physics', episode_id=episode, stream_id='stream',
                            step_index=step, simulation_time_s=step / 30, measurement_us=int(now * 1e6)))


def test_sphere_overlap_sweep_and_corner_near_miss():
    bounds = ((0, 0, 0), (1, 1, 1))
    assert sphere_sweep((0.5, 0.5, 0.5), (0.5, 0.5, 0.5), bounds) == 0
    assert sphere_sweep((-10, 0.5, 0.5), (10, 0.5, 0.5), bounds) == pytest.approx((10 - .42) / 20)
    assert sphere_sweep((-.4, -.4, -.4), (-.4, -.4, -.4), bounds) is None
    assert sphere_sweep((-2, -.4, -.4), (2, -.4, -.4), bounds) is None


def test_collision_retry_and_confirm_only_after_physics_result():
    contact = ContactSession('controller')
    current = state()
    contact.state(current)
    contact.observe(pose([35, 0, 2]), current, 10)
    contact.observe(pose([45, 0, 2], step=2, now=10.03), current, 10.04)
    first, stop = contact.tick(10.04)
    assert first and not stop and first['source']['step_index'] == 2
    assert 'waiting' in contact.status and 'COMPLETED' not in contact.status
    assert contact.tick(10.05) == (None, False)
    retry, stop = contact.tick(10.15)
    assert retry['event_id'] == first['event_id'] and not stop
    contact.state(dict(current, state='stopped', terminal_result={'event_id': first['event_id']}))
    assert contact.status == 'COMPLETED / Target hit'
    assert contact.tick(11.2) == (None, False)


def test_ack_timeout_stops_without_claiming_success_and_reset_clears_latch():
    contact = ContactSession('controller')
    current = state()
    contact.state(current)
    contact.observe(pose([40, 0, 2]), current, 10)
    assert contact.tick(10)[0]
    assert contact.tick(11.01) == (None, True)
    assert 'failed' in contact.status and 'COMPLETED' not in contact.status
    assert contact.tick(11.05) == (None, False)
    assert contact.tick(11.15) == (None, True)
    contact.state(dict(current, state='stopped'))
    assert contact.tick(11.3) == (None, False)
    contact.state(state('new'))
    contact.observe(pose([40, 0, 2], episode='new', now=12), state('new'), 12)
    assert contact.tick(12)[0]


def test_stale_foreign_and_long_gap_do_not_create_false_sweeps():
    contact = ContactSession('controller')
    current = state()
    contact.state(current)
    contact.observe(pose([40, 0, 2], episode='foreign'), current, 10)
    contact.observe(pose([40, 0, 2]), current, 12)
    assert contact.tick(12) == (None, False)
    contact.observe(pose([35, 0, 2], now=13), current, 13)
    contact.observe(pose([45, 0, 2], step=2, now=15), current, 15)
    assert contact.tick(15) == (None, False)
