"""Repeat station physics trials; retain failures and measured JSON."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace

from .station import run


def assess(report, status=0):
    return dict(completed=status == 0 and report['completed'],
        no_fall=not report['fallen'], no_contacts=not report['contact_constraint_samples'],
        moved=report['robot_displacement_m'] > 5.,
        following_distance=report['distance_mean_m'] <= 2.5 and report['distance_final_m'] <= 3.5,
        target_clearance=report['distance_min_m'] >= .85,
        pedestrian_clearance=report['pedestrian_clearance_min_m'] is None or report['pedestrian_clearance_min_m'] >= 0,
        fixture_clearance=report['fixture_clearance_min_m'] >= 0,
        target_visible=report['target_visible_ratio'] >= .7,
        loss_stops=report['lost_command_abs_max'] == 0,
        proxy_updates=report['robot_crowd_proxy_update_failures'] == 0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime', type=Path, default=Path('~/g1-official-demo/official-runtime'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=60.)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    trials = []
    for count, pattern, seed, crowd_speed in [(20, 'bidirectional', 1, .8),
            (20, 'bidirectional', 42, .8), (30, 'arrival-wave', 1, .8),
            (30, 'arrival-wave', 42, .8), (50, 'bidirectional', 42, 1.2),
            (50, 'arrival-wave', 42, 1.2)]:
        name = f'{pattern}-{count}-seed{seed}'
        output = args.output / name
        status = run(SimpleNamespace(runtime=args.runtime, output=output,
            pedestrians=count, crowd_pattern=pattern, seed=seed,
            crowd_speed=crowd_speed, speed=.4, distance=1.5,
            seconds=args.seconds, fov=140., sensor_range=8.,
            target_stop_at=20., target_stop_for=3., viewer=False,
            camera='overview', snapshot=None))
        report = json.loads((output/'report.json').read_text())
        checks = assess(report, status)
        report['acceptance'] = checks
        report['accepted'] = all(checks.values())
        (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        trials.append(dict(name=name, **report))
        print(f'{name}: {"PASS" if report["accepted"] else "FAIL"} {checks}', flush=True)
    (args.output/'summary.json').write_text(json.dumps(dict(trials=trials,
        all_passed=all(t['accepted'] for t in trials)), indent=2)+'\n')
    raise SystemExit(0 if all(t['accepted'] for t in trials) else 1)


if __name__ == '__main__':
    main()
