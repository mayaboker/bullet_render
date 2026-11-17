from pathlib import Path
import sys


TOOLS = Path(__file__).parents[1] / 'tools'
sys.path.insert(0, str(TOOLS))

from tune_betaflight_takeoff import candidate_profile, update_pwm


def test_candidate_profile_replaces_only_calibrated_values(tmp_path):
    source = '# keep comments\nbetaflight_takeoff_pwm: 1700\nbetaflight_hover_pwm: 1550\n'
    path = tmp_path / 'candidate.yaml'

    candidate_profile(source, 1725, 1575, path)

    assert path.read_text() == '# keep comments\nbetaflight_takeoff_pwm: 1725\nbetaflight_hover_pwm: 1575\n'


def test_update_pwm_requires_exactly_one_profile_field():
    try:
        update_pwm('betaflight_takeoff_pwm: 1700\nbetaflight_takeoff_pwm: 1725\n', 'betaflight_takeoff_pwm', 1750)
    except ValueError as error:
        assert 'Expected one' in str(error)
    else:
        raise AssertionError('duplicate profile field was accepted')
