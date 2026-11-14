"""Headless evaluation, interactive viewer and reproducible experiment records."""
import argparse
from contextlib import nullcontext
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import subprocess
import time
import numpy as np
from .config import load_config


def summarize(rows, reason, robot):
    forces = np.array([r["human_horizontal_force"] for r in rows])
    return dict(robot=robot, status=reason, success=reason == "goal_reached", samples=len(rows),
        simulated_seconds=rows[-1]["time"] if rows else 0,
        progress_m=rows[-1]["progress"] if rows else 0,
        max_tilt_deg=max((r["payload_tilt_deg"] for r in rows), default=0),
        human_horizontal_force_rms_N=float(np.sqrt(np.mean(forces**2))) if len(forces) else 0,
        max_grasp_error_m=max((r["grasp_error"] for r in rows), default=0))


def run_episode(config, output, viewer=False, video=False, snapshot=False):
    from .environment import CarryEnvironment
    import mujoco
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    env = CarryEnvironment(config)
    metadata = {"config": asdict(config), "versions": {p: version(p) for p in ("mujoco", "numpy", "onnxruntime")},
                "created_utc": datetime.now(timezone.utc).isoformat(),
                "notes": "Simulation only. Surrogate results do not establish G1 carrying capability."}
    try:
        metadata["git_commit"] = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip()
        metadata["git_dirty"] = bool(subprocess.check_output(["git", "status", "--porcelain"], text=True).strip())
    except (OSError, subprocess.CalledProcessError):
        metadata["git_commit"] = None
    if config.robot == "g1":
        metadata["policy_sha256"] = hashlib.sha256((Path(config.policy_dir)/"policy.onnx").read_bytes()).hexdigest()
    (output / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    display = nullcontext(None)
    if viewer:
        import mujoco.viewer
        callback = getattr(env.human, "key", None)
        display = mujoco.viewer.launch_passive(env.model, env.data, key_callback=callback)
    renderer = writer = None
    if video or snapshot:
        import imageio.v2 as imageio
        renderer = mujoco.Renderer(env.model, height=480, width=640)
        camera = mujoco.MjvCamera()
        camera.distance, camera.azimuth, camera.elevation = 3.8, 135, -20
        if video:
            writer = imageio.get_writer(output / "rollout.mp4", fps=25)
    rows, reason = [], "user_closed"
    next_frame = 0.0
    try:
        with display as handle, (output / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
            csv_writer = None
            if handle:
                handle.cam.distance, handle.cam.azimuth, handle.cam.elevation = 3.8, 135, -20
            while True:
                started = time.perf_counter()
                if handle and not handle.is_running():
                    break
                result = env.step()
                row = result.metrics
                if csv_writer is None:
                    csv_writer = csv.DictWriter(stream, fieldnames=list(row))
                    csv_writer.writeheader()
                csv_writer.writerow(row)
                rows.append(row)
                if handle:
                    handle.cam.lookat[:] = env.data.body("pelvis").xpos + [.5, 0, 0]
                    handle.sync()
                if renderer and env.data.time >= next_frame:
                    camera.lookat[:] = env.data.body("pelvis").xpos + [.5, 0, 0]
                    renderer.update_scene(env.data, camera=camera)
                    frame = renderer.render()
                    if writer:
                        writer.append_data(frame)
                    if snapshot:
                        imageio.imwrite(output / "preview.png", frame)
                        snapshot = False
                    next_frame += .04
                if result.terminated or result.truncated:
                    reason = result.reason
                    break
                if handle:
                    time.sleep(max(0, config.control_dt - (time.perf_counter() - started)))
    finally:
        if writer:
            writer.close()
        if renderer:
            renderer.close()
    summary = summarize(rows, reason, config.robot)
    (output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    download = sub.add_parser("fetch-assets", help="Download fixed-revision Unitree assets and verify hashes")
    download.add_argument("--destination", default=".cache/robots/unitree_g1")
    run = sub.add_parser("run")
    run.add_argument("--config", default="configs/carry_g1.toml")
    run.add_argument("--viewer", action="store_true")
    run.add_argument("--video", action="store_true")
    run.add_argument("--snapshot", action="store_true")
    run.add_argument("--keyboard", action="store_true")
    run.add_argument("--duration", type=float)
    run.add_argument("--seed", type=int)
    run.add_argument("--output")
    args = parser.parse_args()
    if args.action == "fetch-assets":
        from .assets import fetch_assets
        print(fetch_assets(args.destination))
        return
    config = load_config(args.config)
    if args.duration is not None:
        config.duration = args.duration
    if args.seed is not None:
        config.seed = args.seed
    if args.keyboard:
        if not args.viewer:
            parser.error("--keyboard requires --viewer")
        config.human.kind = "keyboard"
    config.validate()
    output = args.output or "results/carry/" + datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    print(json.dumps(run_episode(config, output, args.viewer, args.video, args.snapshot), indent=2))


if __name__ == "__main__":
    main()
