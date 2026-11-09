"""Train and evaluate G1 locomotion with MJLab 1.6.0."""

import argparse
from dataclasses import asdict
from pathlib import Path
import re


def positive_int(value):
    number = int(value)
    if number < 1:
        raise argparse.ArgumentTypeError("must be at least 1")
    return number


def forward_only(cfg, speed):
    # The default speed curriculum would overwrite the fixed command ranges.
    cfg.curriculum.pop("command_vel", None)
    twist = cfg.commands["twist"]
    twist.heading_command = False
    twist.ranges.heading = None
    twist.rel_standing_envs = 0.0
    twist.rel_world_envs = 0.0
    twist.rel_forward_envs = 1.0
    twist.ranges.lin_vel_x = (speed, speed)
    twist.ranges.lin_vel_y = (0.0, 0.0)
    twist.ranges.ang_vel_z = (0.0, 0.0)


def train(args, task):
    from mjlab.scripts.train import TrainConfig, launch_training

    config = TrainConfig.from_task(task)
    config.agent.experiment_name = "g1_terrain_walk"
    config.agent.logger = "tensorboard"
    config.agent.max_iterations = args.iterations
    config.agent.save_interval = 100
    config.env.scene.num_envs = args.envs
    forward_only(config.env, 0.4)
    config.env.rewards["track_linear_velocity"].weight = 4.0
    terrain = config.env.scene.terrain
    terrain.max_init_terrain_level = 0
    terrain.terrain_generator.difficulty_range = (0.0, args.difficulty)

    if args.resume:
        checkpoint = args.resume.resolve()
        expected_root = (Path(config.log_root) / config.agent.experiment_name).resolve()
        if not checkpoint.is_file() or checkpoint.parent.parent != expected_root:
            raise SystemExit(f"Choose a checkpoint from {expected_root}/<run>/")
        config.agent.resume = True
        config.agent.load_run = f"^{re.escape(checkpoint.parent.name)}$"
        config.agent.load_checkpoint = f"^{re.escape(checkpoint.name)}$"

    print(f"Training: {args.envs} envs, difficulty 0–{args.difficulty}, target 0.4 m/s")
    launch_training(task, config)


def evaluate(args, task):
    import torch
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
    from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls
    from mjlab.utils.torch import configure_torch_backends

    if not args.checkpoint.is_file():
        raise SystemExit(f"Checkpoint not found: {args.checkpoint}")
    configure_torch_backends()
    cfg = load_env_cfg(task, play=True)
    cfg.scene.num_envs = args.envs
    cfg.seed = args.seed
    forward_only(cfg, 0.4)
    if args.terrain != "flat":
        generator = cfg.scene.terrain.terrain_generator
        if args.terrain != "mixed":
            terrain_key = "flat" if args.terrain == "rough-flat" else args.terrain
            if terrain_key not in generator.sub_terrains:
                choices = ", ".join(generator.sub_terrains)
                raise SystemExit(f"Unknown terrain {args.terrain}. Available: {choices}")
            generator.sub_terrains = {terrain_key: generator.sub_terrains[terrain_key]}
        generator.difficulty_range = (args.difficulty, args.difficulty)
        generator.seed = args.seed

    agent = load_rl_cfg(task)
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    env = ManagerBasedRlEnv(cfg=cfg, device=device)
    try:
        wrapper = RslRlVecEnvWrapper(env, clip_actions=agent.clip_actions)
        runner_cls = load_runner_cls(task) or MjlabOnPolicyRunner
        runner = runner_cls(wrapper, asdict(agent), device=device)
        runner.load(str(args.checkpoint), load_cfg={"actor": True}, strict=True, map_location=device)
        policy = runner.get_inference_policy(device=device)
        with torch.inference_mode():
            observations, _ = wrapper.reset()
            robot = env.scene["robot"]
            active = torch.ones(args.envs, device=device, dtype=torch.bool)
            frames = torch.zeros(args.envs, device=device)
            speed_sum = torch.zeros_like(frames)
            error_sum = torch.zeros_like(frames)
            target_sum = torch.zeros_like(frames)
            feet = env.scene["feet_ground_contact"]
            previous_air_time = feet.data.current_air_time.clone()
            landings = torch.zeros_like(previous_air_time)
            steps = max(1, round(args.seconds / env.step_dt))
            for _ in range(steps):
                target = env.command_manager.get_command("twist")[:, 0].clone()
                observations, _, done, _ = wrapper.step(policy(observations))
                active &= ~done.bool()
                speed = robot.data.root_link_lin_vel_b[:, 0]
                frames += active
                speed_sum += torch.where(active, speed, 0.0)
                error_sum += torch.where(active, (target - speed).abs(), 0.0)
                target_sum += torch.where(active, target, 0.0)
                air_time = feet.data.current_air_time
                landed = (previous_air_time >= 0.05) & (air_time == 0.0)
                landings += landed & active[:, None]
                previous_air_time.copy_(air_time)
            count = int(frames.sum().item())
            print(f"Checkpoint: {args.checkpoint} | terrain: {args.terrain}")
            print(f"Completed {args.seconds}s without reset: {int(active.sum().item())}/{args.envs}")
            if count:
                print(f"Measured target: {target_sum.sum().item() / count:.3f} m/s")
                print(f"Actual forward speed: {speed_sum.sum().item() / count:.3f} m/s")
                print(f"Mean absolute forward error: {error_sum.sum().item() / count:.3f} m/s")
                print(f"Mean landings per foot: {landings.mean(dim=0).tolist()}")
            else:
                print("No samples before the first reset.")
    finally:
        env.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    training = commands.add_parser("train")
    training.add_argument("--envs", type=positive_int, default=128)
    training.add_argument("--iterations", type=positive_int, default=1000)
    training.add_argument("--difficulty", type=float, default=0.15)
    training.add_argument("--resume", type=Path)
    evaluation = commands.add_parser("eval")
    evaluation.add_argument("checkpoint", type=Path)
    evaluation.add_argument("--terrain", default="mixed", help="flat for legacy flat models; otherwise mixed or a rough terrain key")
    evaluation.add_argument("--envs", type=positive_int, default=16)
    evaluation.add_argument("--seconds", type=positive_int, default=10)
    evaluation.add_argument("--difficulty", type=float, default=0.15)
    evaluation.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    if not 0.0 <= args.difficulty <= 1.0:
        parser.error("--difficulty must be in [0, 1]")

    import mjlab.tasks  # Registers built-in tasks.

    rough_task = "Mjlab-Velocity-Rough-Unitree-G1"
    if args.command == "train":
        train(args, rough_task)
    else:
        task = "Mjlab-Velocity-Flat-Unitree-G1" if args.terrain == "flat" else rough_task
        evaluate(args, task)


if __name__ == "__main__":
    main()
