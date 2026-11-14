import numpy as np
from .interfaces import register_controller


def limit_norm(value, maximum):
    value = np.asarray(value, dtype=float)
    return value * min(1.0, maximum / max(float(np.linalg.norm(value)), 1e-12))


class Admittance:
    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.velocity = np.zeros(2)
        self.force = np.zeros(2)

    def command(self, obs, dt):
        c = self.config
        self.force += (1 - np.exp(-dt / c.force_filter_tau)) * (
            obs.robot_force_world[:2] - self.force)
        acceleration = limit_norm((self.force - c.damping * self.velocity)
                                  / c.virtual_mass, c.max_acceleration)
        self.velocity = limit_norm(self.velocity + dt * acceleration, c.max_speed)
        world = np.array([*self.velocity, 0.0])
        body = obs.base_rotation.T @ world
        return np.array([body[0], body[1], 0.0])


class FixedVelocity:
    """Diagnostic baseline: no human-force feedback."""
    def __init__(self, config):
        self.config = config

    def reset(self):
        pass

    def command(self, obs, dt):
        return np.array([min(self.config.max_speed, 0.15), 0.0, 0.0])


register_controller("admittance", Admittance)
register_controller("fixed", FixedVelocity)
