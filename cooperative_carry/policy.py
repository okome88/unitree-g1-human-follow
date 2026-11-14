"""Exact 98-input official G1 ONNX adapter; no COLA input-contract guessing."""
from pathlib import Path
import hashlib
import json
import numpy as np
import onnxruntime as ort
import yaml


class OfficialG1:
    def __init__(self, model, directory, control_dt):
        directory = Path(directory)
        manifest = json.loads((directory / "manifest.json").read_text())
        for entry in manifest["files"]:
            path = directory / entry["file"]
            if hashlib.sha256(path.read_bytes()).hexdigest() != entry["sha256"]:
                raise ValueError(f"Modified policy artifact: {path}")
        cfg = yaml.safe_load((directory / "deploy.yaml").read_text())
        if not np.isclose(control_dt, cfg["step_dt"]):
            raise ValueError("Official policy requires control_dt=0.02")
        self.dt = control_dt
        self.default = np.array(cfg["default_joint_pos"])
        self.kp = np.array(cfg["stiffness"])
        self.kd = np.array(cfg["damping"])
        self.scale = np.array(cfg["actions"]["JointPositionAction"]["scale"])
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        self.session = ort.InferenceSession(str(directory / "policy.onnx"),
            sess_options=options, providers=["CPUExecutionProvider"])
        if self.session.get_inputs()[0].shape != [1, 98]:
            raise ValueError("Unsupported official policy input")
        self.joints = model.actuator_trnid[:, 0].copy()
        if len(self.joints) != 29:
            raise ValueError("Expected 29 G1 actuators")
        expected = [f"{side}_{joint}_joint" for side in ("left", "right")
                    for joint in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")]
        expected += ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
        expected += [f"{side}_{joint}_joint" for side in ("left", "right") for joint in
                     ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")]
        if [model.joint(int(j)).name for j in self.joints] != expected:
            raise ValueError("G1 actuator joint order does not match policy")
        self.q = model.jnt_qposadr[self.joints]
        self.v = model.jnt_dofadr[self.joints]
        self.reset()

    def reset(self):
        self.previous = np.zeros(29)
        self.target = self.default.copy()
        self.phase = 0.0

    def update(self, model, data, command):
        rotation = data.body("pelvis").xmat.reshape(3, 3)
        command = np.clip(command, [-0.5, -0.5, -1.0], [1.0, 0.5, 1.0])
        self.phase = (self.phase + self.dt / 0.6) % 1.0
        gait = np.array([np.sin(2 * np.pi * self.phase), np.cos(2 * np.pi * self.phase)])
        if np.linalg.norm(command) < 0.1:
            gait[:] = 0
        obs = np.concatenate([data.sensor("imu_gyro").data,
            rotation.T @ [0.0, 0.0, -1.0], command, gait,
            data.qpos[self.q] - self.default, data.qvel[self.v], self.previous]).astype(np.float32)
        if obs.shape != (98,) or not np.isfinite(obs).all():
            raise ValueError("Invalid G1 observation")
        action = self.session.run(None, {self.session.get_inputs()[0].name: obs[None]})[0][0]
        if action.shape != (29,) or not np.isfinite(action).all():
            raise ValueError("Invalid G1 action")
        self.previous = action.copy()
        self.target = self.default + self.scale * action

    def torques(self, model, data):
        torque = self.kp * (self.target - data.qpos[self.q]) - self.kd * data.qvel[self.v]
        return np.clip(torque, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
