"""Official single-frame 98 -> 29 ONNX contract, independent of either simulator."""
from pathlib import Path
import hashlib
import json
import numpy as np
import onnxruntime as ort
import yaml


JOINT_NAMES = [f'{side}_{name}_joint' for side in ('left', 'right') for name in
               ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle_pitch', 'ankle_roll')]
JOINT_NAMES += ['waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint']
JOINT_NAMES += [f'{side}_{name}_joint' for side in ('left', 'right') for name in
                ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')]
TORQUE_LIMITS = np.array([88,139,88,139,50,50]*2 + [88,50,50] + [25,25,25,25,25,5,5]*2, dtype=float)


class Policy:
    def __init__(self, directory, dt=.02):
        directory = Path(directory)
        manifest = json.loads((directory/'manifest.json').read_text())
        for entry in manifest['files']:
            if hashlib.sha256((directory/entry['file']).read_bytes()).hexdigest() != entry['sha256']:
                raise ValueError('Policy/config checksum mismatch')
        cfg = yaml.safe_load((directory/'deploy.yaml').read_text())
        if not np.isclose(dt, cfg['step_dt']):
            raise ValueError('Official G1 policy requires a 0.02 second control period')
        self.dt = dt
        self.default = np.array(cfg['default_joint_pos'])
        self.kp, self.kd = np.array(cfg['stiffness']), np.array(cfg['damping'])
        self.scale = np.array(cfg['actions']['JointPositionAction']['scale'])
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        self.session = ort.InferenceSession(str(directory/'policy.onnx'), sess_options=options,
                                           providers=['CPUExecutionProvider'])
        if self.session.get_inputs()[0].shape != [1,98] or self.session.get_outputs()[0].shape != [1,29]:
            raise ValueError('Unsupported policy shape')
        self.reset()

    def reset(self):
        self.phase = 0.0
        self.previous = np.zeros(29)
        self.target = self.default.copy()

    def update(self, gyro_body, gravity_body, command, joint_position, joint_velocity):
        command = np.clip(command, [-.5,-.5,-1], [1,.5,1])
        self.phase = (self.phase+self.dt/.6) % 1
        gait = [np.sin(2*np.pi*self.phase), np.cos(2*np.pi*self.phase)] if np.linalg.norm(command) >= .1 else [0.,0.]
        obs = np.concatenate([gyro_body, gravity_body, command, gait,
                              joint_position-self.default, joint_velocity, self.previous]).astype(np.float32)
        if obs.shape != (98,) or not np.isfinite(obs).all():
            raise ValueError('Invalid observation')
        action = self.session.run(None, {self.session.get_inputs()[0].name: obs[None]})[0][0]
        if action.shape != (29,) or not np.isfinite(action).all():
            raise ValueError('Invalid policy action')
        self.previous = action.copy()
        self.target = self.default+self.scale*action

    def torques(self, joint_position, joint_velocity):
        return np.clip(self.kp*(self.target-joint_position)-self.kd*joint_velocity, -TORQUE_LIMITS, TORQUE_LIMITS)
