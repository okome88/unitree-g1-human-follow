"""Run our existing planner in pinned upstream Follow-Bench (2D only)."""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import random
import subprocess
import time

import numpy as np
import yaml

from .planner import Follower, Person

UPSTREAM = "202d1645015f44b965a474211d5ef2bc67a00fe3"


def verify_files(root, paths):
    # Read tree objects directly: git diff can refresh all 15k scenario files
    # on a WSL /mnt/c checkout even when pathspecs select only 122 source files.
    entries = subprocess.check_output(['git', '-C', str(root), 'ls-tree', '-rz',
                                       'HEAD', '--', *paths])
    names = []
    for entry in entries.split(b'\0'):
        if not entry:
            continue
        metadata, name = entry.split(b'\t', 1)
        names.append(name.decode())
        expected = metadata.split()[2].decode()
        content = (root / name.decode()).read_bytes()
        candidates = [content, content.replace(b'\r\n', b'\n')]
        if not any(hashlib.sha1(b'blob ' + str(len(c)).encode() + b'\0' + c).hexdigest()
                   == expected for c in candidates):
            raise ValueError(f'Upstream file differs from pinned version: {name.decode()}')
    for path in paths:
        if not any(n == path or n.startswith(path.rstrip('/') + '/') for n in names):
            raise ValueError(f'Path is not in the pinned upstream tree: {path}')


def prepare_config(source, speed):
    """Explicit G1-limited differential-drive proxy, not the paper profile."""
    config = yaml.safe_load(source.read_text())
    if not np.isclose(config['world']['step_time'], .1):
        raise ValueError('Planner requires a 0.1 s world step')
    robot = config['robot'][0]
    robot['kinematics'] = {'name': 'diff'}
    robot['shape'] = {'name': 'circle', 'radius': .35}
    robot['vel_min'], robot['vel_max'] = [0., -.7], [.8, .7]
    robot['acce'] = [.8, 1.2]
    for target in config['target']:
        target['vel_min'], target['vel_max'] = [-speed, -speed], [speed, speed]
        target.setdefault('behavior', {}).update(vxmax=speed, vymax=speed)
    return config


def scan_obstacles(scan, pose, visible):
    """World-frame lidar point discs plus visible moving-person circles.

    No hidden target position is used to mask scans. Visible person footprints
    are removed from scan points because their moving circles replace them.
    """
    moving = [o for o in visible if o.role in ('human', 'target')]
    ranges = np.asarray(scan['ranges'])
    angles = np.linspace(scan['angle_min'], scan['angle_max'], len(ranges)) + pose[2]
    valid = np.isfinite(ranges) & (ranges > scan.get('range_min', 0))
    valid &= ranges < scan['range_max'] - .01
    points = pose[:2] + ranges[valid, None] * np.column_stack((np.cos(angles[valid]), np.sin(angles[valid])))
    for obj in moving:
        points = points[np.linalg.norm(points - obj.state[:2, 0], axis=1) > obj.radius + .2]
    obstacles = [[*p, 0., 0., .05] for p in points]
    obstacles += [[*o.state[:2, 0], *o.velocity_xy[:, 0], float(o.radius)]
                  for o in moving if o.role == 'human']
    return obstacles


def observed_target(visible, heading):
    for obj in visible:
        if obj.role == 'target':
            velocity = obj.velocity_xy[:, 0].copy()
            if np.linalg.norm(velocity) > .01:
                heading = float(np.arctan2(velocity[1], velocity[0]))
            return Person(obj.state[:2, 0].copy(), velocity, heading), heading
    # Hidden ground truth never enters the controller.
    return Person(np.zeros(2), np.zeros(2), heading, visible=False), heading


def run_trial(irsim, source, output, seed, steps, search_steps, distance, speed):
    output.mkdir(parents=True, exist_ok=False)
    random.seed(seed)
    np.random.seed(seed)
    config = prepare_config(source, speed)
    effective = output / 'effective.yaml'
    effective.write_text(yaml.safe_dump(config, sort_keys=False))
    env = irsim.make(str(effective), display=False, disable_all_plot=True,
                     eval_dir=str(output), log_level='ERROR')
    planner = Follower(seed=seed, distance=distance)
    # Constrain candidates BEFORE prediction, rather than dropping lateral
    # velocity after selecting a checked rollout.
    planner.low[1] = planner.high[1] = 0.
    reasons, costs, commands = Counter(), [], []
    heading = 0.
    completed = 0
    try:
        for _ in range(steps):
            pose = env.robot.state[:, 0].copy()
            visible = env.visible_object_list
            target, heading = observed_target(visible, heading)
            obstacles = scan_obstacles(env.get_lidar_scan(), pose, visible)
            started = time.perf_counter()
            command = planner.command(pose, target, obstacles)
            elapsed = time.perf_counter() - started
            costs.append(elapsed)
            commands.append(command.tolist())
            reasons[planner.reason] += 1
            # List dispatch steps every object exactly once. Upstream scalar
            # dispatch mixes list indices and persistent IDs across trials.
            env.step([command[[0, 2], None]])
            env.record(elapsed)
            completed += 1
            if env.done():
                break
        env.eval(max_steps=steps, max_search_steps=search_steps)
    finally:
        import matplotlib.pyplot as plt
        plt.close('all')
    metrics = json.loads((output / 'eval_result.json').read_text())
    report = dict(profile='g1_limited_diff_2d', upstream_commit=UPSTREAM,
                  scenario=source.name, source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  seed=seed, steps_requested=steps, steps_completed=completed,
                  completed=completed == steps, distance=distance, target_speed_limit=speed,
                  search_steps=search_steps, planner_reasons=dict(reasons),
                  command_abs_max=np.max(np.abs(commands), axis=0).tolist(),
                  planning_time_mean_s=float(np.mean(costs)),
                  metrics=metrics, physics='upstream 2D differential proxy; no ONNX or biped physics')
    (output / 'trial.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bench-root', required=True, type=Path)
    parser.add_argument('--scenario', required=True, type=Path, nargs='+',
                        help='YAML paths relative to bench-root')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--seeds', type=int, nargs='+', default=[0, 1, 2])
    parser.add_argument('--steps', type=int, default=500)
    parser.add_argument('--search-steps', type=int, default=125)
    parser.add_argument('--distance', type=float, default=1.5)
    parser.add_argument('--target-speed', type=float, default=.4)
    args = parser.parse_args()
    if args.steps < 3 or args.search_steps < 1:
        parser.error('steps must be >= 3 and search-steps positive')
    if not np.isfinite(args.distance) or args.distance < 1.2:
        parser.error('distance must be finite and >= 1.2')
    if not np.isfinite(args.target_speed) or not 0 < args.target_speed <= .8:
        parser.error('target-speed must be in (0, 0.8]')
    if len(set(args.seeds)) != len(args.seeds) or any(s < 0 for s in args.seeds):
        parser.error('seeds must be distinct nonnegative integers')
    root = args.bench_root.resolve()
    revision = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    if revision != UPSTREAM:
        parser.error(f'Use pinned Follow-Bench commit {UPSTREAM}')
    sources = [(root / p).resolve() for p in args.scenario]
    if any(not p.is_file() or not p.is_relative_to(root) for p in sources):
        parser.error('scenario must be an existing file inside bench-root')
    if len({p.stem for p in sources}) != len(sources):
        parser.error('scenario names must be distinct')
    paths = ['ir-sim/irsim', *[p.relative_to(root).as_posix() for p in sources]]
    verify_files(root, paths)
    # Prevent partial overwrites and mixed old/new results.
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['MPLBACKEND'] = 'Agg'
    import irsim
    if not Path(irsim.__file__).resolve().is_relative_to(root / 'ir-sim'):
        parser.error('Install ir-sim from the specified bench-root')
    results = []
    for source in sources:
        for seed in args.seeds:
            output = args.output.resolve() / source.stem / f'seed_{seed}'
            result = run_trial(irsim, source, output, seed, args.steps,
                               args.search_steps, args.distance, args.target_speed)
            results.append(result)
            print(f'{source.stem} seed={seed}: {result["metrics"]}', flush=True)
    summaries = {}
    for source in sources:
        trials = [r for r in results if r['scenario'] == source.name]
        summaries[source.stem] = {
            'trials': len(trials),
            'completion_rate': float(np.mean([r['completed'] for r in trials])),
            'collision_free_rate': float(np.mean([r['metrics']['obstacle_avoidance_success'] for r in trials])),
            'search_success_rate': float(np.mean([r['metrics']['search_success'] for r in trials])),
            'mean_metrics': {k: float(np.mean([r['metrics'][k] for r in trials]))
                             for k, v in trials[0]['metrics'].items() if type(v) in (int, float)},
        }
    (args.output / 'summary.json').write_text(json.dumps(
        dict(upstream_commit=UPSTREAM, profile='g1_limited_diff_2d',
             paper_comparable=False, scenarios=summaries, trials=results), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
