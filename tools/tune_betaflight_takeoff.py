"""Calibrate simulated Betaflight takeoff and hover PWM with hover-check runs."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = ROOT.parent / 'bullet_app/configs/betaflight_hover_profile.yaml'
DEFAULT_PHYSICS_CONFIG = ROOT.parent / 'bullet_drone/configs/basic.yaml'
CHECK_SCRIPT = ROOT / 'tools/check_app_flight.py'
TAKEOFF_CANDIDATES = tuple(range(1650, 1901, 25))
HOVER_CANDIDATES = tuple(range(1500, 1701, 25))
FINAL_RUNS = 3


def update_pwm(profile_text, name, pwm):
    pattern = rf'^(\s*{name}:\s*)\d+[ \t]*$'
    updated, count = re.subn(pattern, rf'\g<1>{pwm}', profile_text, flags=re.MULTILINE)
    if count != 1:
        raise ValueError(f'Expected one {name} field in profile')
    return updated


def candidate_profile(profile_text, takeoff_pwm, hover_pwm, path):
    path.write_text(update_pwm(update_pwm(profile_text, 'betaflight_takeoff_pwm', takeoff_pwm),
                               'betaflight_hover_pwm', hover_pwm))


def run_hover_check(profile, physics_config, report_dir):
    command = [sys.executable, str(CHECK_SCRIPT), '--hover-check', '--mission-config', str(profile),
               '--physics-config', str(physics_config), '--report-dir', str(report_dir)]
    result = subprocess.run(command, cwd=ROOT, text=True, capture_output=True)
    report = report_dir / 'report.json'
    return dict(passed=result.returncode == 0 and report.exists(), returncode=result.returncode,
                output=(result.stdout + result.stderr)[-4000:],
                report=None if not report.exists() else json.loads(report.read_text()))


def first_passing(candidates, profile_text, *, takeoff_pwm, hover_pwm, profile_path, physics_config, report_root, results, label):
    for candidate in candidates:
        pair = (candidate, hover_pwm) if label == 'takeoff' else (takeoff_pwm, candidate)
        candidate_profile(profile_text, *pair, profile_path)
        outcome = run_hover_check(profile_path, physics_config, report_root / f'{label}-{candidate}')
        results.append(dict(stage=label, takeoff_pwm=pair[0], hover_pwm=pair[1], **outcome))
        if outcome['passed']:
            return candidate
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument('--physics-config', type=Path, default=DEFAULT_PHYSICS_CONFIG)
    parser.add_argument('--report-dir', type=Path, default=Path('/tmp/betaflight-tune'))
    parser.add_argument('--takeoff-pwm', type=int, help='Use a previously qualified takeoff PWM')
    args = parser.parse_args()
    profile, physics = args.profile.resolve(), args.physics_config.resolve()
    report_root = args.report_dir.resolve()
    report_root.mkdir(parents=True, exist_ok=True)
    source = profile.read_text()
    candidate = report_root / 'candidate.yaml'
    results = []
    initial_hover = int(re.search(r'^\s*betaflight_hover_pwm:\s*(\d+)\s*$', source, re.MULTILINE).group(1))
    takeoff_pwm = args.takeoff_pwm or first_passing(
        TAKEOFF_CANDIDATES, source, takeoff_pwm=0, hover_pwm=initial_hover,
        profile_path=candidate, physics_config=physics, report_root=report_root, results=results, label='takeoff')
    if takeoff_pwm is None:
        raise SystemExit('No safe takeoff PWM candidate passed hover-check')
    hover_pwm, final = None, []
    for hover_candidate in HOVER_CANDIDATES:
        attempts = []
        for index in range(FINAL_RUNS):
            candidate_profile(source, takeoff_pwm, hover_candidate, candidate)
            outcome = run_hover_check(candidate, physics, report_root / f'hover-{hover_candidate}-final-{index + 1}')
            attempts.append(outcome)
            results.append(dict(stage='hover', takeoff_pwm=takeoff_pwm, hover_pwm=hover_candidate,
                                attempt=index + 1, **outcome))
            if not outcome['passed']:
                break
        if len(attempts) == FINAL_RUNS and all(outcome['passed'] for outcome in attempts):
            hover_pwm, final = hover_candidate, attempts
            break
    report = dict(takeoff_pwm=takeoff_pwm, hover_pwm=hover_pwm, trials=results, final=final)
    (report_root / 'tuning-report.json').write_text(json.dumps(report, indent=2))
    if hover_pwm is None:
        raise SystemExit('No hover PWM candidate passed three hover-check runs')
    profile.write_text(update_pwm(update_pwm(source, 'betaflight_takeoff_pwm', takeoff_pwm),
                                  'betaflight_hover_pwm', hover_pwm))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
