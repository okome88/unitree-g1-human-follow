"""Isaac Lab 2.3 / Isaac Sim 5.1 entry point. Use its Python 3.11 environment."""
import argparse
from pathlib import Path
import sys

# Allow source checkout execution without installing the MuJoCo dependency group.
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def main():
    from isaac_carry.preflight import check_runtime
    import json
    report = check_runtime()
    if '--check' in sys.argv:
        print(json.dumps(report,indent=2))
        return 0 if report['ready'] else 2
    if not report['ready']:
        print(json.dumps(report,indent=2))
        print('See docs/ISAAC_LAB.md. The MuJoCo .venv is not an Isaac environment.')
        return 2
    from isaaclab.app import AppLauncher
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default=str(ROOT/'configs/carry_g1.toml'))
    parser.add_argument('--duration',type=float)
    parser.add_argument('--g1-usd',help='Optional locally converted 29-joint USD; default converts pinned MJCF')
    parser.add_argument('--output',default=None)
    parser.add_argument('--require-goal',action='store_true')
    parser.add_argument('--require-stable',action='store_true')
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    from cooperative_carry.config import load_config
    config = load_config(args.config)
    if args.duration is not None:
        config.duration = args.duration
    config.validate()
    if config.robot != 'g1' or config.human.kind == 'keyboard':
        parser.error('Isaac backend currently supports G1 with scripted human drivers only')
    for attr in ('asset_dir','policy_dir'):
        value = Path(getattr(config,attr))
        if not value.is_absolute():
            setattr(config,attr,str(ROOT/value))
    launcher = AppLauncher(args)
    try:
        return run(args,config,launcher.app,report)
    finally:
        launcher.app.close()


def run(args,config,app,report):
    # Isaac imports MUST follow AppLauncher initialization.
    import csv
    from dataclasses import asdict
    from datetime import datetime
    import hashlib
    import json
    import isaaclab.sim as sim_utils
    from isaaclab.scene import InteractiveScene
    from isaac_carry.policy import Policy
    from isaac_carry.scene import scene_config
    from isaac_carry.backend import IsaacCarryBackend
    from cooperative_carry.cli import summarize
    output = Path(args.output) if args.output else ROOT/'results'/'isaaclab'/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    output.mkdir(parents=True,exist_ok=False)
    policy = Policy(config.policy_dir,config.control_dt)
    metadata = dict(backend='isaaclab',validation_status='experimental',runtime=report,
        config=asdict(config),isaaclab_source='3c6e67bb5c7ada942a6d1884ab69338f57596f77',
        policy_sha256=hashlib.sha256((Path(config.policy_dir)/'policy.onnx').read_bytes()).hexdigest())
    (output/'metadata.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
    rows,reason = [],'startup_error'
    try:
        sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=config.timestep,device=args.device))
        sim.set_camera_view([3,-3,2],[.7,0,.7])
        cfg,usd = scene_config(config,policy,args.g1_usd)
        metadata['g1_usd'] = usd
        (output/'metadata.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
        scene = InteractiveScene(cfg)
        sim.reset()
        backend = IsaacCarryBackend(config,sim,scene,policy)
        reason = 'user_closed'
        with (output/'trajectory.csv').open('w',newline='',encoding='utf-8') as stream:
            writer = None
            while app.is_running() and not backend.done:
                row = backend.step()
                if writer is None:
                    writer = csv.DictWriter(stream,fieldnames=list(row))
                    writer.writeheader()
                writer.writerow(row)
                rows.append(row)
            if backend.done:
                reason = backend.reason
    except Exception:
        import traceback
        reason = 'runtime_error'
        (output/'error.txt').write_text(traceback.format_exc(),encoding='utf-8')
        raise
    finally:
        summary = summarize(rows,reason,'g1')
        summary['backend'] = 'isaaclab'
        (output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
        print(json.dumps(summary,indent=2))
    if args.require_goal and reason != 'goal_reached':
        return 1
    if args.require_stable and reason not in ('time_limit','goal_reached'):
        return 1
    return 0 if reason in ('time_limit','goal_reached','user_closed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
