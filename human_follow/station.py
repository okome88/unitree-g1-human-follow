"""G1 walking in a fictional station with JuPedSim crowd dynamics; no DDS."""
import argparse
from collections import Counter
from functools import partial
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from .planner import Follower, Person
from .policy import OfficialPolicy
from .sim import scene
from .station_world import StationLayout, StationCrowd, animate_person, decorate_station


def observation(layout, pose, target, people, fov, sensor_range):
    """Only idealized visible target information enters the controller."""
    position, velocity, heading = target
    visible = layout.visible(pose, position, people, fov, sensor_range)
    return (Person(position.copy(), velocity.copy(), heading) if visible else
            Person(np.zeros(2), np.zeros(2), visible=False))


def blend_people(before, after, alpha):
    result = []
    for a, b in zip(before, after):
        if b is None:
            result.append(None)
        elif a is None:
            result.append(list(b))
        else:
            result.append([*((1-alpha)*np.array(a[:2]) + alpha*np.array(b[:2])), *b[2:]])
    return result


def camera(model, data, mode):
    cam = mujoco.MjvCamera()
    if mode == 'overview':
        cam.lookat[:] = [11, 0, .4]
        cam.distance, cam.azimuth, cam.elevation = 27, 110, -55
    else:
        cam.lookat[:] = [*data.qpos[:2], .9]
        cam.distance, cam.azimuth, cam.elevation = 8, 135, -28
    return cam


def validate_args(args):
    for name, low, high in [('speed', .05, .8), ('crowd_speed', .2, 1.5),
                            ('distance', 1.2, 3.), ('fov', 30., 360.),
                            ('sensor_range', 1., 30.), ('seconds', .1, 3600.)]:
        value = getattr(args, name)
        if not np.isfinite(value) or not low <= value <= high:
            raise ValueError(f'{name} must be finite and in [{low}, {high}]')
    if not 0 <= args.pedestrians <= 50 or args.seed < 0:
        raise ValueError('pedestrians must be in [0, 50] and seed nonnegative')
    if (not np.isfinite([args.target_stop_at, args.target_stop_for]).all() or
            min(args.target_stop_at, args.target_stop_for) < 0):
        raise ValueError('target stop times must be finite and nonnegative')


def run(args):
    validate_args(args)
    args.output.mkdir(parents=True, exist_ok=False)
    layout = StationLayout()
    crowd = StationCrowd(layout, args.pedestrians, args.seed, args.speed,
                         args.crowd_speed, args.crowd_pattern)
    policy, planner = OfficialPolicy(), Follower(args.seed, args.distance)
    model, qids, vids = scene(args.runtime.expanduser(), decorate=partial(
        decorate_station, count=args.pedestrians, seed=args.seed))
    data = mujoco.MjData(model)
    data.qpos[qids] = policy.default
    # Semantic collision groups are derived from body ancestry, not guessed IDs.
    pelvis = model.body('pelvis').id
    robot_bodies = {pelvis}
    for bid in range(pelvis+1, model.nbody):
        if model.body_parentid[bid] in robot_bodies:
            robot_bodies.add(bid)
    robot_geoms = {i for i in range(model.ngeom) if model.geom_bodyid[i] in robot_bodies}
    contacts = Counter()
    def collision_type(gid):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ''
        if name.startswith('station_collision_'):
            return 'fixture'
        if name.startswith('person_') and name.endswith('_collision'):
            return 'target' if name == 'person_0_collision' else 'pedestrian'
        return None
    root_ids = [model.body(f'person_{i}').mocapid[0] for i in range(args.pedestrians+1)]
    previous_target = next_target = crowd.target()
    previous_people = next_people = crowd.people()
    interval_start = 2.
    phase = np.zeros(args.pedestrians+1)
    visible = True
    command = np.zeros(3)
    viewer = None
    started = time.monotonic()
    reasons, distances, clearances, static_clearances = Counter(), [], [], []
    visible_count = lost_count = 0
    lost_max_command = 0.
    fallen = False
    frames = int(np.ceil(args.seconds/policy.dt))
    initial_target = next_target[0].copy()
    target_traveled = 0.
    last_target = initial_target.copy()
    planning_times = []
    trace_path = args.output / 'trace.jsonl'
    try:
        if args.viewer:
            from mujoco import viewer as mjviewer
            viewer = mjviewer.launch_passive(model, data)
        with trace_path.open('w') as trace:
            for step in range(frames):
                mujoco.mj_forward(model, data)
                rotation = data.xmat[pelvis].reshape(3, 3).copy()
                yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
                pose = np.array([*data.qpos[:2], yaw])
                if step % 5 == 0 and data.time >= 2.-1e-8:
                    previous_target, previous_people = crowd.target(), crowd.people()
                    active = [p for p in previous_people if p is not None]
                    obs = observation(layout, pose, previous_target, active, args.fov, args.sensor_range)
                    visible = obs.visible
                    planning_start = time.perf_counter()
                    # Static map and pedestrian states are ideal ground truth;
                    # visibility is applied to the designated target only.
                    command = planner.command(pose, obs, layout.obstacles(pose[:2], active))
                    planning_times.append(time.perf_counter()-planning_start)
                    if visible:
                        visible_count += 1
                    else:
                        lost_count += 1
                        lost_max_command = max(lost_max_command, float(np.max(np.abs(command))))
                    reasons[planner.reason] += 1
                    previous_ids = list(crowd.ids)
                    crowd.step(data.qpos[:2], args.target_stop_at, args.target_stop_for)
                    next_target, next_people = crowd.target(), crowd.people()
                    # A respawned slot has a new identity. Never interpolate
                    # from the departed agent's exit to the new entry point.
                    previous_people = [p if old == new else None for p, old, new
                                       in zip(previous_people, previous_ids, crowd.ids)]
                    interval_start = data.time
                if data.time < 2.-1e-8:
                    command = planner.stop('settling')
                def actors_at(t):
                    alpha = float(np.clip((t-interval_start)/.1, 0, 1))
                    xy = (1-alpha)*previous_target[0]+alpha*next_target[0]
                    people = blend_people(previous_people, next_people, alpha)
                    return [[*xy, *next_target[1], .25], *people]
                actors = actors_at(data.time)
                for i, actor in enumerate(actors):
                    if actor is None:
                        animate_person(model, data, i, None, [0, 0], 0, 0)
                        continue
                    vel = np.array(actor[2:4])
                    heading = next_target[2] if i == 0 else float(np.arctan2(vel[1], vel[0]))
                    phase[i] += float(np.linalg.norm(vel))*policy.dt*2*np.pi/1.1
                    animate_person(model, data, i, np.array(actor[:2]), vel, heading, phase[i])
                desired = policy.act(data.sensor('imu_gyro').data.copy(),
                                     rotation.T @ [0, 0, -1], command,
                                     data.qpos[qids], data.qvel[vids])
                for _ in range(10):
                    # Interpolate mocap collision bodies at the physics rate.
                    for mid, actor in zip(root_ids, actors_at(data.time+model.opt.timestep)):
                        data.mocap_pos[mid] = [*actor[:2], 0.] if actor is not None else [0, 0, -10]
                    torque = policy.kp*(desired-data.qpos[qids])-policy.kd*data.qvel[vids]
                    data.ctrl[:] = np.clip(torque, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
                    mujoco.mj_step(model, data)
                    for contact in data.contact:
                        g1, g2 = int(contact.geom1), int(contact.geom2)
                        kind = collision_type(g2) if g1 in robot_geoms else (
                            collision_type(g1) if g2 in robot_geoms else None)
                        if kind and contact.dist <= 0:
                            contacts[kind] += 1
                    if contacts:
                        break
                actors = actors_at(data.time)
                true_position = np.array(actors[0][:2])
                active = [p for p in actors[1:] if p is not None]
                target_traveled += float(np.linalg.norm(true_position-last_target))
                last_target = true_position
                distances.append(float(np.linalg.norm(data.qpos[:2]-true_position)))
                static_clearances.append(layout.clearance(data.qpos[:2]))
                if active:
                    clearances.append(min(float(np.linalg.norm(data.qpos[:2]-p[:2])-.35-p[4]) for p in active))
                fallen = (not np.isfinite(data.qpos).all() or data.qpos[2] < .45 or data.xmat[pelvis, 8] < .5)
                if step % 5 == 0:
                    trace.write(json.dumps(dict(time=float(data.time), robot=[*data.qpos[:2], yaw],
                        target=true_position.tolist(), target_visible=visible, command=command.tolist(),
                        reason=planner.reason, pedestrians=actors[1:], height=float(data.qpos[2])))+'\n')
                if viewer is not None:
                    if not viewer.is_running():
                        break
                    cam = camera(model, data, args.camera)
                    viewer.cam.lookat[:] = cam.lookat
                    viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = cam.distance, cam.azimuth, cam.elevation
                    viewer.sync()
                    time.sleep(max(0, data.time-(time.monotonic()-started)))
                if fallen or contacts:
                    break
    finally:
        if viewer is not None:
            viewer.close()
    completed = data.time >= args.seconds-.001
    report = dict(environment='fictional_station_concourse', seed=args.seed,
        pedestrian_slots=args.pedestrians, crowd_pattern=args.crowd_pattern,
        crowd_engine='JuPedSim 1.4.2 CollisionFreeSpeedModel',
        pedestrian_desired_speed_range_m_s=[.75*args.crowd_speed, 1.2*args.crowd_speed],
        target_desired_speed_m_s=args.speed, desired_distance_m=args.distance,
        target_stop_at_crowd_time_s=args.target_stop_at, target_stop_duration_s=args.target_stop_for,
        requested_seconds=args.seconds, simulated_seconds=float(data.time), completed=completed,
        fallen=bool(fallen), contact_constraint_samples=dict(contacts),
        distance_mean_m=float(np.mean(distances)), distance_min_m=min(distances), distance_final_m=distances[-1],
        robot_displacement_m=float(np.linalg.norm(data.qpos[:2])), target_path_m=target_traveled,
        target_final_stage=int(crowd.sim.agent(crowd.target_id).stage_id),
        target_reached_final_stage=crowd.sim.agent(crowd.target_id).stage_id == crowd.target_stages[-1],
        target_goal_distance_m=float(np.linalg.norm(last_target-crowd.target_points[-1])),
        target_reached_goal=bool(np.linalg.norm(last_target-crowd.target_points[-1]) <= .45),
        pedestrian_clearance_min_m=min(clearances) if clearances else None,
        fixture_clearance_min_m=min(static_clearances),
        target_visible_ratio=visible_count/max(1, visible_count+lost_count),
        target_lost_control_steps=lost_count, lost_command_abs_max=lost_max_command,
        planning_time_mean_s=float(np.mean(planning_times)) if planning_times else None,
        stop_reasons=dict(reasons), spawned=crowd.spawned, exited=crowd.exited,
        robot_crowd_proxy_radius_m=.45, robot_crowd_proxy_update_failures=crowd.shadow_update_failures,
        fov_degrees=args.fov, sensor_range_m=args.sensor_range,
        wall_seconds=time.monotonic()-started,
        perception='ideal map and pedestrian states; geometric target visibility; no image detector',
        limitations='planar pedestrians; kinematic colliders; G1 proxy refreshed at 10 Hz with .02 s handoff gap; not a real-station calibration',
        transport='none; MuJoCo in process; no DDS')
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    (args.output/'settings.json').write_text(json.dumps(
        {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}, indent=2)+'\n')
    if args.snapshot:
        import imageio.v3 as iio
        args.snapshot.parent.mkdir(parents=True, exist_ok=True)
        mujoco.mj_forward(model, data)
        with mujoco.Renderer(model, height=720, width=1280) as renderer:
            renderer.update_scene(data, camera=camera(model, data, args.camera))
            iio.imwrite(args.snapshot, renderer.render())
    print(json.dumps(report, indent=2))
    return 0 if completed and not fallen and not contacts else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime', type=Path, default=Path('~/g1-official-demo/official-runtime'))
    parser.add_argument('--output', type=Path, required=True, help='New result directory')
    parser.add_argument('--pedestrians', type=int, default=20)
    parser.add_argument('--crowd-pattern', choices=['bidirectional', 'arrival-wave'], default='bidirectional')
    parser.add_argument('--crowd-speed', type=float, default=.8)
    parser.add_argument('--speed', type=float, default=.4)
    parser.add_argument('--distance', type=float, default=1.5)
    parser.add_argument('--seconds', type=float, default=60.)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--target-stop-at', type=float, default=20.)
    parser.add_argument('--target-stop-for', type=float, default=3.)
    parser.add_argument('--fov', type=float, default=140.)
    parser.add_argument('--sensor-range', type=float, default=8.)
    parser.add_argument('--viewer', action='store_true')
    parser.add_argument('--camera', choices=['overview', 'follow'], default='overview')
    parser.add_argument('--snapshot', type=Path)
    raise SystemExit(run(parser.parse_args()))


if __name__ == '__main__':
    main()
