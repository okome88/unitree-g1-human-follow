"""Adapter for the repository's pinned official 98-input, 29-action policy."""
import hashlib
import json
from pathlib import Path
import numpy as np
import onnxruntime as ort
import yaml

MODEL = Path(__file__).resolve().parents[1] / "models/unitree_official/g1_velocity"


class OfficialPolicy:
    def __init__(self):
        manifest = json.loads((MODEL / "manifest.json").read_text())
        for item in manifest["files"]:
            content = (MODEL / item["file"]).read_bytes()
            if item["file"] != "policy.onnx":
                content = content.replace(b"\r\n", b"\n")
            if hashlib.sha256(content).hexdigest() != item["sha256"]:
                raise ValueError(f"Official file checksum mismatch: {item['file']}")
        self.cfg = yaml.safe_load((MODEL / "deploy.yaml").read_text())
        self.default = np.array(self.cfg["default_joint_pos"])
        self.kp = np.array(self.cfg["stiffness"])
        self.kd = np.array(self.cfg["damping"])
        self.action_cfg = self.cfg["actions"]["JointPositionAction"]
        self.dt = self.cfg["step_dt"]
        self.last = np.zeros(29)
        self.phase = 0.
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(MODEL / "policy.onnx"), options,
                                           providers=["CPUExecutionProvider"])
        inp = self.session.get_inputs()[0]
        if inp.shape != [1, 98] or self.session.get_outputs()[0].shape != [1, 29]:
            raise ValueError("Unexpected official ONNX dimensions")
        self.input_name = inp.name

    def observation(self, gyro, gravity, command, q, dq):
        ranges = self.cfg["commands"]["base_velocity"]["ranges"]
        bounds = np.array([ranges[k] for k in ("lin_vel_x", "lin_vel_y", "ang_vel_z")])
        command = np.clip(command, bounds[:, 0], bounds[:, 1])
        period = self.cfg["observations"]["gait_phase"]["params"]["period"]
        self.phase = (self.phase + self.dt / period) % 1
        phase = [np.sin(2 * np.pi * self.phase), np.cos(2 * np.pi * self.phase)]
        if np.linalg.norm(command) < .1:
            phase = [0., 0.]
        terms = dict(base_ang_vel=gyro, projected_gravity=gravity,
                     velocity_commands=command, gait_phase=phase,
                     joint_pos_rel=q - self.default, joint_vel_rel=dq, last_action=self.last)
        parts = []
        for name, config in self.cfg["observations"].items():
            if config["history_length"] != 1:
                raise ValueError("Only the pinned single-frame policy is supported")
            value = np.array(terms[name])
            if config["clip"] is not None:
                value = np.clip(value, *config["clip"])
            parts.extend(value * np.array(config["scale"]))
        observation = np.asarray(parts, dtype=np.float32)[None]
        if observation.shape != (1, 98) or not np.isfinite(observation).all():
            raise ValueError("Invalid ONNX observation")
        return observation

    def act(self, gyro, gravity, command, q, dq):
        observation = self.observation(gyro, gravity, command, q, dq)
        action = self.session.run(None, {self.input_name: observation})[0][0]
        if not np.isfinite(action).all():
            raise ValueError("Non-finite policy action")
        self.last = action.copy()
        return action * np.array(self.action_cfg["scale"]) + np.array(self.action_cfg["offset"])
