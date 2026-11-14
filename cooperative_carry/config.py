"""Strict, serializable experiment configuration; all distances SI units."""
from dataclasses import asdict, dataclass, field
from pathlib import Path
import math
import tomllib


@dataclass
class HumanConfig:
    kind: str = "straight"
    speed: float = 0.15
    start_time: float = 2.0
    stop_time: float = 16.0
    stiffness: float = 180.0
    damping: float = 18.0
    max_force: float = 45.0


@dataclass
class PayloadConfig:
    mass: float = 0.5
    length: float = 1.0
    thickness: float = 0.04
    handle_length: float = 0.2
    stiffness: float = 300.0
    damping: float = 12.0
    max_force: float = 50.0
    break_distance: float = 0.35


@dataclass
class ControllerConfig:
    kind: str = "admittance"
    virtual_mass: float = 8.0
    damping: float = 25.0
    max_speed: float = 0.35
    max_acceleration: float = 0.5
    force_filter_tau: float = 0.15


@dataclass
class Config:
    robot: str = "g1"
    carry: bool = True
    timestep: float = 0.002
    control_dt: float = 0.02
    duration: float = 20.0
    seed: int = 0
    goal_distance: float = 1.5
    max_tilt_deg: float = 35.0
    asset_dir: str = ".cache/robots/unitree_g1"
    policy_dir: str = "models/unitree_official/g1_velocity"
    human: HumanConfig = field(default_factory=HumanConfig)
    payload: PayloadConfig = field(default_factory=PayloadConfig)
    controller: ControllerConfig = field(default_factory=ControllerConfig)

    def validate(self):
        # Reject NaN/Inf, unknown TOML fields (dataclass constructors) and invalid rates.
        def finite(values):
            for value in values.values():
                if isinstance(value, dict):
                    finite(value)
                elif isinstance(value, (int, float)) and not math.isfinite(value):
                    raise ValueError("Configuration contains a non-finite number")
        finite(asdict(self))
        positive = [self.timestep, self.control_dt, self.duration, self.goal_distance,
                    self.max_tilt_deg, self.payload.mass, self.payload.length,
                    self.payload.thickness, self.payload.stiffness, self.payload.damping,
                    self.payload.max_force, self.payload.break_distance,
                    self.human.stiffness, self.human.damping, self.human.max_force,
                    self.controller.virtual_mass, self.controller.damping,
                    self.controller.max_speed, self.controller.max_acceleration,
                    self.controller.force_filter_tau]
        if min(positive) <= 0:
            raise ValueError("Physical parameters and periods must be positive")
        if self.payload.handle_length < 0:
            raise ValueError("handle_length must be nonnegative")
        if self.human.start_time < 0 or self.human.stop_time <= self.human.start_time:
            raise ValueError("Human stop_time must follow nonnegative start_time")
        if abs(self.human.speed) > 0.5 or self.controller.max_speed > 0.5:
            raise ValueError("Initial experiment supports speeds up to 0.5 m/s")
        ratio = self.control_dt / self.timestep
        if ratio < 1 or not math.isclose(ratio, round(ratio)):
            raise ValueError("control_dt must be an integer multiple of timestep")
        if not isinstance(self.carry, bool) or self.robot not in ("g1", "surrogate"):
            raise ValueError("robot must be g1 or surrogate; carry must be boolean")
        return self


def load_config(path: str | Path) -> Config:
    with Path(path).open("rb") as stream:
        values = tomllib.load(stream)
    for key, cls in (("human", HumanConfig), ("payload", PayloadConfig),
                     ("controller", ControllerConfig)):
        if key in values:
            values[key] = cls(**values[key])
    return Config(**values).validate()
