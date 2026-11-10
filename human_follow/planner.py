"""Research-inspired sampling planner; not a reproduction of either paper."""
from dataclasses import dataclass
import numpy as np


@dataclass
class Person:
    position: np.ndarray
    velocity: np.ndarray
    heading: float = 0.0
    visible: bool = True
    age: float = 0.0


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class Follower:
    """Rear semi-annular goals + constant-velocity prediction + sampled MPC.

    Commands are body-frame [forward m/s, lateral m/s, yaw rad/s].
    Clearance checks are planar approximations, not a biped safety guarantee.
    """
    def __init__(self, seed=42, distance=1.5):
        if not np.isfinite(distance) or distance < 1.2:
            raise ValueError("distance must be finite and >= 1.2 m")
        self.distance = distance
        self.rng = np.random.default_rng(seed)
        self.previous = np.zeros(3)
        self.offset = None
        self.reason = "initializing"
        self.dt, self.horizon = 0.1, 15
        self.low = np.array([0., -.25, -.7])
        self.high = np.array([.8, .25, .7])

    def stop(self, reason):
        self.previous[:] = 0
        self.reason = reason
        return self.previous.copy()

    def command(self, pose, target, obstacles=()):
        pose = np.asarray(pose, dtype=float)
        values = [pose, target.position, target.velocity, [target.heading, target.age]]
        values.extend(np.asarray(o) for o in obstacles)
        if not all(np.isfinite(v).all() for v in values):
            return self.stop("invalid_observation")
        if not target.visible or not 0 <= target.age <= .3:
            return self.stop("target_lost")
        if np.linalg.norm(target.position - pose[:2]) < .85:
            return self.stop("target_too_close")
        # Each obstacle is [x, y, vx, vy, radius].
        obs = np.asarray(obstacles, dtype=float).reshape(-1, 5)
        if len(obs) and np.any(np.linalg.norm(obs[:, :2] - pose[:2], axis=1) < obs[:, 4] + .45):
            return self.stop("obstacle_too_close")
        times = np.arange(1, self.horizon + 1) * self.dt
        future = target.position + times[:, None] * target.velocity
        heading = target.heading
        angles = heading + np.linspace(np.pi / 2, 3 * np.pi / 2, 25)
        offsets = np.concatenate([r * np.column_stack([np.cos(angles), np.sin(angles)])
                                  for r in (self.distance, self.distance + .3)])
        goals = future[None, :, :] + offsets[:, None, :]
        cost = 2 * np.abs(np.linalg.norm(offsets, axis=1) - self.distance)
        cost += .5 * np.linalg.norm(goals[:, 0] - pose[:2], axis=1)
        cost += .4 * np.abs(wrap(np.arctan2(offsets[:, 1], offsets[:, 0]) - heading - np.pi))
        if self.offset is not None:
            cost += .8 * np.linalg.norm(offsets - self.offset, axis=1)
        predicted_obstacles = []
        for x, y, vx, vy, radius in obs:
            prediction = np.array([x, y]) + times[:, None] * [vx, vy]
            predicted_obstacles.append((prediction, radius))
            clearance = np.linalg.norm(goals - prediction, axis=2)
            cost[np.any(clearance < radius + .5, axis=1)] = np.inf
            # Line-segment visibility proxy (not the paper's camera IoU).
            sight = future[None] - goals
            fraction = np.clip(np.sum((prediction - goals) * sight, axis=2) /
                               np.sum(sight * sight, axis=2), 0, 1)
            occlusion = np.linalg.norm(goals + fraction[..., None] * sight - prediction, axis=2)
            cost += 3 * np.mean(occlusion < radius + .15, axis=1)
        if not np.isfinite(cost).any():
            return self.stop("no_goal")
        self.offset = offsets[np.argmin(cost)]
        reference = future + self.offset
        # Feed-forward plus position correction; noise explores alternative paths.
        world = target.velocity + .9 * (target.position + self.offset - pose[:2])
        c, s = np.cos(pose[2]), np.sin(pose[2])
        nominal = np.array([c * world[0] + s * world[1],
                            -s * world[0] + c * world[1],
                            1.5 * wrap(np.arctan2(target.position[1] - pose[1],
                                                 target.position[0] - pose[0]) - pose[2])])
        controls = nominal + self.rng.normal(size=(192, 1, 3)) * [.25, .16, .35]
        controls = np.broadcast_to(controls, (192, self.horizon, 3)).copy()
        controls[0] = nominal
        controls[1] = 0
        controls = np.clip(controls, self.low, self.high)
        # Acceleration limits at every predicted control step, including the first.
        prev = np.broadcast_to(self.previous, (192, 3)).copy()
        for k in range(self.horizon):
            controls[:, k] = prev + np.clip(controls[:, k] - prev, [-.08, -.06, -.12], [.08, .06, .12])
            prev = controls[:, k]
        states = np.broadcast_to(pose, (192, 3)).copy()
        scores = np.zeros(192)
        feasible = np.ones(192, dtype=bool)
        for k in range(self.horizon):
            u = controls[:, k]
            c, s = np.cos(states[:, 2]), np.sin(states[:, 2])
            states[:, 0] += self.dt * (c * u[:, 0] - s * u[:, 1])
            states[:, 1] += self.dt * (s * u[:, 0] + c * u[:, 1])
            states[:, 2] += self.dt * u[:, 2]
            scores += np.sum((states[:, :2] - reference[k]) ** 2, axis=1)
            scores += .15 * wrap(states[:, 2] - heading) ** 2
            feasible &= np.linalg.norm(states[:, :2] - future[k], axis=1) >= .85
            for prediction, radius in predicted_obstacles:
                distance = np.linalg.norm(states[:, :2] - prediction[k], axis=1)
                feasible &= distance >= radius + .45
                scores += .15 / np.maximum(distance - radius, .05) ** 2
        scores += .2 * np.sum((controls[:, 0] - self.previous) ** 2, axis=1)
        scores[~feasible] = np.inf
        if not feasible.any():
            return self.stop("no_safe_rollout")
        # Select a checked rollout. Averaging trajectories can cross an obstacle.
        self.previous = controls[np.argmin(scores), 0].copy()
        self.reason = "following"
        return self.previous.copy()
