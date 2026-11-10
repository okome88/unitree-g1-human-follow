"""Run a gamepad-free G1 simulation; never connects to DDS or a robot."""
import argparse
import hashlib
import json
import time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from .planner import Follower, Person
from .policy import OfficialPolicy
from .conditions import conditions, pedestrians_at


def scene(runtime, routes=()):
    path = runtime / "src/assets/robots/unitree_g1/xmls/scene_g1.xml"
    normalized = path.read_bytes().replace(b"\r\n", b"\n")
    if hashlib.sha256(normalized).hexdigest() != "f96a137240a2ed248e115fb6e5253959a54cc23216188ac538456d2ac179b6fd":
        raise ValueError("Scene differs from pinned official commit 1425b15; use the documented runtime")
    root = ET.parse(path).getroot()
    compiler = root.find("compiler")
    compiler.set("meshdir", str((path.parent / compiler.get("meshdir", ".")).resolve()))
    world = root.find("worldbody")
    people = [("follow_target", "0.1 0.8 0.3 1", .25)]
    people += [(f"pedestrian_{i}", "0.9 0.3 0.2 1", route[4]) for i, route in enumerate(routes)]
    for name, color, radius in people:
        body = ET.SubElement(world, "body", name=name, mocap="true", pos="2 0 0")
        ET.SubElement(body, "geom", type="capsule", fromto="0 0 0.3 0 0 1.4",
                      size=str(radius), rgba=color, contype="0", conaffinity="0")
        ET.SubElement(body, "geom", type="sphere", pos="0 0 1.65", size="0.15",
                      rgba=color, contype="0", conaffinity="0")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    model.opt.timestep = .002
    if model.nu != 29 or model.nq != 36:
        raise ValueError("Expected official 29-DoF G1 scene")
    # Official lowstate sensor order defines motor order; check each actuator.
    names = [e.get("joint") for e in root.find("sensor") if e.tag == "jointpos"]
    if len(names) != 29:
        raise ValueError("Unexpected motor sensor order")
    joints = np.array([model.joint(n).id for n in names])
    if not np.array_equal(model.actuator_trnid[:, 0], joints):
        raise ValueError("Actuator/sensor joint order mismatch")
    return model, model.jnt_qposadr[joints], model.jnt_dofadr[joints]


def target_at(t, scenario, speed, turn_rate=.1, start_x=2.):
    t = max(0., t - 2.)  # two seconds for the standing policy to settle
    if scenario == "stop":
        moving_time = min(t, 8.)
        return Person(np.array([start_x + speed * moving_time, 0.]), np.array([speed if t < 8 else 0., 0.]))
    if scenario == "turn" and abs(turn_rate) > 1e-8:
        omega = turn_rate
        return Person(np.array([start_x + speed / omega * np.sin(omega * t),
                                speed / omega * (1 - np.cos(omega * t))]),
                      speed * np.array([np.cos(omega * t), np.sin(omega * t)]), omega * t)
    return Person(np.array([start_x + speed * t, 0.]), np.array([speed, 0.]),
                  visible=not (scenario == "loss" and 8 <= t < 11))


def run(args):
    if not np.isfinite(args.seconds) or args.seconds <= 0:
        raise ValueError("seconds must be finite and positive")
    if not np.isfinite(args.speed) or not 0 <= args.speed <= .8:
        raise ValueError("speed must be in [0, 0.8] m/s")
    distance, turn_rate, routes = conditions(args)
    policy = OfficialPolicy()
    planner = Follower(args.seed, distance=distance)
    model, qids, vids = scene(args.runtime.expanduser(), routes)
    data = mujoco.MjData(model)
    data.qpos[qids] = policy.default
    mujoco.mj_forward(model, data)
    viewer = None
    if args.viewer:
        from mujoco import viewer as mjviewer
        viewer = mjviewer.launch_passive(model, data)
    distances, heights, commands, reasons, clearances = [], [], [], [], []
    command = np.zeros(3)
    fallen = False
    started = time.monotonic()
    initial_xy = data.qpos[:2].copy()
    frames = int(np.ceil(args.seconds / policy.dt))
    try:
        for step in range(frames):
            target = target_at(data.time, args.scenario, args.speed, turn_rate, distance + .5)
            obstacles = pedestrians_at(data.time, routes)
            data.mocap_pos[model.body("follow_target").mocapid[0]] = [*target.position, 0.]
            for i, obstacle in enumerate(obstacles):
                data.mocap_pos[model.body(f"pedestrian_{i}").mocapid[0]] = [*obstacle[:2], 0.]
            rotation = data.xmat[model.body("pelvis").id].reshape(3, 3).copy()
            yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
            if step % 5 == 0:
                command = planner.command(np.array([*data.qpos[:2], yaw]), target, obstacles)
            if data.time < 2:
                command = planner.stop("settling")
            gravity = rotation.T @ [0., 0., -1.]
            desired = policy.act(data.sensor("imu_gyro").data.copy(), gravity, command,
                                 data.qpos[qids], data.qvel[vids])
            for _ in range(10):
                torque = policy.kp * (desired - data.qpos[qids]) - policy.kd * data.qvel[vids]
                data.ctrl[:] = np.clip(torque, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
                mujoco.mj_step(model, data)
            if not np.isfinite(data.qpos).all() or data.qpos[2] < .45 or data.xmat[model.body("pelvis").id, 8] < .5:
                fallen = True
                break
            distances.append(float(np.linalg.norm(data.qpos[:2] - target.position)))
            heights.append(float(data.qpos[2]))
            commands.append(command.tolist())
            reasons.append(planner.reason)
            if obstacles:
                clearances.append(min(float(np.linalg.norm(data.qpos[:2] - o[:2]) - o[4] - .35) for o in obstacles))
            if viewer is not None:
                if not viewer.is_running():
                    break
                viewer.cam.lookat[:] = [*data.qpos[:2], .8]
                viewer.cam.distance = 5
                viewer.sync()
                time.sleep(max(0., data.time - (time.monotonic() - started)))
    finally:
        if viewer is not None:
            viewer.close()
    report = dict(scenario=args.scenario, seed=args.seed, target_speed=args.speed,
                  following_method="adaptive_sampling_mpc", desired_distance_m=distance,
                  target_turn_rate_rad_s=turn_rate, pedestrian_routes=routes,
                  requested_seconds=args.seconds, simulated_seconds=float(data.time),
                  fallen=fallen, samples=len(distances),
                  displacement_m=float(np.linalg.norm(data.qpos[:2] - initial_xy)),
                  distance_mean_m=float(np.mean(distances)) if distances else None,
                  distance_final_m=distances[-1] if distances else None,
                  distance_min_m=min(distances) if distances else None,
                  height_min_m=min(heights) if heights else None,
                  obstacle_clearance_min_m=min(clearances) if clearances else None,
                  command_abs_max=np.max(np.abs(commands), axis=0).tolist() if commands else None,
                  lost_command_abs_max=max((max(abs(v) for v in command) for command, reason
                                            in zip(commands, reasons) if reason == "target_lost"), default=0.),
                  stop_reasons={reason: reasons.count(reason) for reason in sorted(set(reasons))},
                  wall_seconds=time.monotonic() - started,
                  perception="simulation ground truth; visual proxy people; no camera detector",
                  transport="none; in-process MuJoCo only")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 1 if fallen else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", type=Path, default=Path("~/g1-official-demo/official-runtime"))
    parser.add_argument("--scenario", choices=["straight", "turn", "stop", "loss", "crossing"], default="straight")
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--speed", type=float, default=.4)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--distance", type=float, default=1.5, help="Following distance, 1.2–3 m")
    parser.add_argument("--turn-rate", type=float, default=.1, help="Target yaw rate for turn scenario, ±0.3 rad/s; positive=left")
    parser.add_argument("--pedestrians", type=int, default=None, help="0–20; default 1 for crossing, otherwise 0")
    parser.add_argument("--pedestrian-path", choices=["crossing", "parallel", "oncoming", "stationary"], default="crossing")
    parser.add_argument("--pedestrian-speed", type=float, default=.25, help="0–1.5 m/s")
    parser.add_argument("--pedestrian-routes", type=Path, help="JSON list of [x,y,vx,vy,radius]; overrides preset routes")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--output", type=Path)
    raise SystemExit(run(parser.parse_args()))


if __name__ == "__main__":
    main()
