"""Force-coupled MuJoCo backend with deterministic reset and explicit failure states."""
from dataclasses import dataclass
import numpy as np
import mujoco
from . import controllers, humans  # registers builtins
from .controllers import limit_norm
from .interfaces import CONTROLLERS, HUMANS, Observation
from .policy import OfficialG1
from .scene import build_scene


@dataclass(frozen=True)
class StepResult:
    observation: Observation
    metrics: dict
    terminated: bool
    truncated: bool
    reason: str


def spring_force(target, target_velocity, position, velocity, stiffness, damping, maximum):
    return limit_norm(stiffness * (target - position) + damping * (target_velocity - velocity), maximum)


class CarryEnvironment:
    def __init__(self, config, controller=None, human=None, scene_builder=build_scene,
                 policy_factory=OfficialG1):
        self.config = config.validate()
        self.model = scene_builder(config)
        self.data = mujoco.MjData(self.model)
        if controller is None and config.controller.kind not in CONTROLLERS:
            raise ValueError(f"Unknown controller: {config.controller.kind}")
        if human is None and config.human.kind not in HUMANS:
            raise ValueError(f"Unknown human: {config.human.kind}")
        self.controller = controller or CONTROLLERS[config.controller.kind](config.controller)
        self.human = human or HUMANS[config.human.kind](config.human)
        self.policy = policy_factory(self.model, config.policy_dir, config.control_dt) if config.robot == "g1" else None
        self.base_id = self.model.body("pelvis").id
        self.avatar_id = self.model.body_mocapid[self.model.body("human_avatar").id]
        self.robot_sites = [self.model.site(f"robot_{side}").id for side in ("left", "right")]
        self.jacobian = np.zeros((3, self.model.nv))
        self.jacobian_rot = np.zeros_like(self.jacobian)
        self.reset()

    def reset(self, seed=None):
        self.rng = np.random.default_rng(self.config.seed if seed is None else seed)
        mujoco.mj_resetData(self.model, self.data)
        self.controller.reset()
        self.human.reset()
        if self.policy:
            self.policy.reset()
            self.data.qpos[self.policy.q] = self.policy.default
        mujoco.mj_forward(self.model, self.data)
        hands = self.data.site_xpos[self.robot_sites].copy()
        if self.config.carry:
            joint = self.model.joint("payload_free")
            start = joint.qposadr[0]
            self.data.qpos[start:start + 3] = hands.mean(axis=0) + [self.config.payload.length / 2 + self.config.payload.handle_length, 0, 0]
            self.data.qpos[start + 3:start + 7] = [1, 0, 0, 0]
            # Align the two grasp sites with actual robot hands in the initial pose.
            for i, side in enumerate(("left", "right")):
                self.model.site(f"payload_robot_{side}").pos[:] = hands[i] - self.data.qpos[start:start+3]
                self.model.site(f"payload_human_{side}").pos[:] = hands[i] - self.data.qpos[start:start+3] + [self.config.payload.length + 2*self.config.payload.handle_length, 0, 0]
            mujoco.mj_forward(self.model, self.data)
            self.human_origins = np.array([self.data.site(f"payload_human_{s}").xpos.copy() for s in ("left", "right")])
            self.payload_start = self.data.body("payload").xpos.copy()
        else:
            self.human_origins = hands + [self.config.payload.length, 0, 0]
            self.payload_start = np.zeros(3)
        self.data.mocap_pos[self.avatar_id] = self.human_origins.mean(axis=0) + [.25, 0, 0]
        self.robot_force = np.zeros(3)
        self.human_force = np.zeros(3)
        self.command = np.zeros(3)
        self.max_grasp_error = 0.0
        self.done = False
        self.reason = "running"
        self.steps = 0
        mujoco.mj_forward(self.model, self.data)
        return self.observe()

    def observe(self):
        return Observation(float(self.data.time), self.data.body("pelvis").xpos.copy(),
            self.data.body("pelvis").xmat.reshape(3, 3).copy(), self.robot_force.copy())

    def site_velocity(self, site):
        mujoco.mj_jacSite(self.model, self.data, self.jacobian, self.jacobian_rot, site)
        return self.jacobian @ self.data.qvel

    def apply_force(self, force, point, body):
        mujoco.mj_applyFT(self.model, self.data, force, np.zeros(3), point, int(body), self.data.qfrc_applied)

    def apply_interaction(self):
        self.robot_force[:] = 0
        self.human_force[:] = 0
        self.max_grasp_error = 0.0
        displacement, velocity = self.human.target(float(self.data.time))
        self.data.mocap_pos[self.avatar_id] = self.human_origins.mean(axis=0) + displacement + [.25, 0, 0]
        if not self.config.carry:
            return
        p = self.config.payload
        h = self.config.human
        payload_id = self.model.body("payload").id
        for i, side in enumerate(("left", "right")):
            robot_site = self.robot_sites[i]
            payload_site = self.model.site(f"payload_robot_{side}").id
            a, b = self.data.site_xpos[robot_site], self.data.site_xpos[payload_site]
            error = float(np.linalg.norm(a - b))
            self.max_grasp_error = max(self.max_grasp_error, error)
            force_on_payload = spring_force(a, self.site_velocity(robot_site), b,
                self.site_velocity(payload_site), p.stiffness, p.damping, p.max_force)
            # Equal and opposite forces at each attachment; no kinematic payload teleport.
            self.apply_force(force_on_payload, b, payload_id)
            self.apply_force(-force_on_payload, a, self.model.site_bodyid[robot_site])
            self.robot_force -= force_on_payload
            human_site = self.model.site(f"payload_human_{side}").id
            point = self.data.site_xpos[human_site]
            force = spring_force(self.human_origins[i] + displacement, velocity, point,
                self.site_velocity(human_site), h.stiffness, h.damping, h.max_force)
            self.apply_force(force, point, payload_id)
            self.human_force += force

    def metrics(self):
        position = self.data.body("payload").xpos if self.config.carry else self.data.body("pelvis").xpos
        tilt = 0.0
        if self.config.carry:
            tilt = float(np.degrees(np.arccos(np.clip(self.data.body("payload").xmat.reshape(3,3)[2,2], -1, 1))))
        return dict(time=float(self.data.time), robot_x=float(self.data.body("pelvis").xpos[0]),
            robot_z=float(self.data.body("pelvis").xpos[2]), payload_x=float(position[0]),
            payload_z=float(position[2]), payload_tilt_deg=tilt,
            progress=float(position[0] - self.payload_start[0]),
            human_horizontal_force=float(np.linalg.norm(self.human_force[:2])),
            human_vertical_force=float(self.human_force[2]), robot_vertical_force=float(-self.robot_force[2]),
            robot_force_x=float(self.robot_force[0]), robot_force_y=float(self.robot_force[1]),
            grasp_error=self.max_grasp_error, command_vx=float(self.command[0]), command_vy=float(self.command[1]))

    def failure(self):
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            return "nonfinite"
        if any(self.data.warning[i].number for i in range(len(self.data.warning))):
            return "physics_warning"
        if self.config.robot == "g1" and (self.data.body("pelvis").xpos[2] < .45 or self.data.body("pelvis").xmat.reshape(3,3)[2,2] < .5):
            return "robot_fall"
        if self.config.carry:
            if self.data.body("payload").xpos[2] < .2:
                return "payload_drop"
            if self.max_grasp_error > self.config.payload.break_distance:
                return "grasp_separation"
            if self.metrics()["payload_tilt_deg"] > self.config.max_tilt_deg:
                return "payload_tilt"
        return ""

    def step(self):
        if self.done:
            raise RuntimeError("Episode ended; call reset()")
        self.command = np.asarray(self.controller.command(self.observe(), self.config.control_dt), dtype=float)
        if self.command.shape != (3,) or not np.isfinite(self.command).all():
            raise ValueError("Controller must return finite [vx, vy, yaw_rate]")
        if self.policy:
            self.policy.update(self.model, self.data, self.command)
        for _ in range(round(self.config.control_dt / self.config.timestep)):
            self.data.qfrc_applied[:] = 0
            self.apply_interaction()
            if self.policy:
                self.data.ctrl[:] = self.policy.torques(self.model, self.data)
            else:
                # Explicitly a planar supported surrogate, never claimed as biped locomotion.
                self.data.ctrl[:] = np.clip(100 * (self.command[:2] - self.data.qvel[:2]), -100, 100)
            mujoco.mj_step(self.model, self.data)
            mujoco.mj_forward(self.model, self.data)
            reason = self.failure()
            if reason:
                self.done, self.reason = True, reason
                break
        self.steps += 1
        metrics = self.metrics()
        if not self.done and self.config.carry and metrics["progress"] >= self.config.goal_distance:
            self.done, self.reason = True, "goal_reached"
        timeout = self.data.time >= self.config.duration - 1e-9
        if not self.done and timeout:
            self.done, self.reason = True, "time_limit"
        return StepResult(self.observe(), metrics, self.done and self.reason != "time_limit",
                          self.done and self.reason == "time_limit", self.reason)
