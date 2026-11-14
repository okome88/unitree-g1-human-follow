"""Replace human intent and high-level control independently from MuJoCo."""
from dataclasses import dataclass
from typing import Protocol
import numpy as np


@dataclass(frozen=True)
class Observation:
    time: float
    base_position: np.ndarray
    base_rotation: np.ndarray
    robot_force_world: np.ndarray
    # Payload truth belongs to evaluation, intentionally not this policy input.


class Controller(Protocol):
    def reset(self) -> None: ...
    def command(self, obs: Observation, dt: float) -> np.ndarray:
        """Return body-frame vx, vy, yaw-rate (m/s, m/s, rad/s)."""
        ...


class HumanDriver(Protocol):
    def reset(self) -> None: ...
    def target(self, time: float) -> tuple[np.ndarray, np.ndarray]:
        """World displacement and velocity of human hands (m, m/s)."""
        ...


CONTROLLERS = {}
HUMANS = {}


def register_controller(name, factory):
    if name in CONTROLLERS:
        raise ValueError(f"Controller already registered: {name}")
    CONTROLLERS[name] = factory


def register_human(name, factory):
    if name in HUMANS:
        raise ValueError(f"Human driver already registered: {name}")
    HUMANS[name] = factory
