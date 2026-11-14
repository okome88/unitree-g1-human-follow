"""Single-environment Isaac Lab backend; no MuJoCo imports or hidden root support.

Must be imported after AppLauncher. PhysX behavior is pending a supported-GPU run.
"""
import numpy as np
import torch
import isaaclab.sim as sim_utils
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from cooperative_carry import controllers, humans
from cooperative_carry.interfaces import CONTROLLERS, HUMANS, Observation
from .math_utils import rotation_matrix, point_state, spring, wrench_at_com
from .policy import JOINT_NAMES


def numpy(value):
    return value.detach().cpu().numpy().copy()


class IsaacCarryBackend:
    def __init__(self, config, sim, scene, policy, controller=None, human=None):
        self.config, self.sim, self.scene, self.policy = config, sim, scene, policy
        self.robot = scene['robot']
        self.payload = scene['payload'] if config.carry else None
        self.device = self.robot.device
        if len(self.robot.joint_names) != 29 or set(self.robot.joint_names) != set(JOINT_NAMES):
            raise ValueError('Converted USD must contain exactly the 29 official G1 joints; no hand variants.')
        self.joints = [self.robot.joint_names.index(name) for name in JOINT_NAMES]
        self.wrists = [self.robot.body_names.index(f'{s}_wrist_yaw_link') for s in ('left','right')]
        self.base = self.robot.body_names.index('pelvis')
        self.controller = controller or CONTROLLERS[config.controller.kind](config.controller)
        self.human = human or HUMANS[config.human.kind](config.human)
        if config.human.kind == 'keyboard':
            raise ValueError('Isaac backend currently supports scripted human drivers only')
        self.markers = VisualizationMarkers(VisualizationMarkersCfg(prim_path='/Visuals/Human', markers={
            'head':sim_utils.SphereCfg(radius=.1, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.9,.5,.15))),
            'torso':sim_utils.CapsuleCfg(radius=.12,height=.4, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.9,.5,.15))),
            'hand':sim_utils.SphereCfg(radius=.035, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.2,.9,.3)))}))
        self.reset()

    def tensor(self, value):
        return torch.as_tensor(value, dtype=torch.float32, device=self.device)

    def reset(self):
        self.time = 0.0
        self.done, self.reason = False, 'running'
        self.robot_force = np.zeros(3)
        self.human_force = np.zeros(3)
        self.command = np.zeros(3)
        self.max_error = 0.0
        self.controller.reset()
        self.human.reset()
        self.policy.reset()
        root = self.robot.data.default_root_state.clone()
        root[:,:3] += self.scene.env_origins
        root[:,3:7] = self.tensor([[1,0,0,0]])
        root[:,7:] = 0
        self.robot.write_root_pose_to_sim(root[:,:7])
        self.robot.write_root_velocity_to_sim(root[:,7:])
        q = self.robot.data.default_joint_pos.clone()
        q[:,self.joints] = self.tensor(self.policy.default)[None]
        self.robot.write_joint_state_to_sim(q, torch.zeros_like(q))
        self.scene.reset()
        self.sim.forward()
        self.scene.update(self.config.timestep)
        hands = np.array([self.hand_state(i)[0] for i in range(2)])
        self.human_origins = hands + [self.config.payload.length+2*self.config.payload.handle_length,0,0]
        if self.payload:
            center = hands.mean(axis=0)+[self.config.payload.length/2+self.config.payload.handle_length,0,0]
            pose = np.r_[center, [1.,0,0,0]][None]
            self.payload.write_root_pose_to_sim(self.tensor(pose))
            self.payload.write_root_velocity_to_sim(torch.zeros((1,6),device=self.device))
            self.robot_attachments = hands-center
            self.human_attachments = self.human_origins-center
            self.start_x = float(center[0])
        else:
            self.start_x = float(root[0,0])
        self.sim.forward()
        self.scene.update(self.config.timestep)
        self.update_markers()

    def hand_state(self, index):
        d, b = self.robot.data, self.wrists[index]
        return point_state(numpy(d.body_link_pos_w[0,b]), rotation_matrix(numpy(d.body_link_quat_w[0,b])),
            np.array([.08,0,0]), numpy(d.body_com_pos_w[0,b]), numpy(d.body_com_lin_vel_w[0,b]),
            numpy(d.body_com_ang_vel_w[0,b]))

    def payload_point(self, offset):
        d = self.payload.data
        return point_state(numpy(d.root_link_pos_w[0]), rotation_matrix(numpy(d.root_link_quat_w[0])),
            offset, numpy(d.root_com_pos_w[0]), numpy(d.root_com_lin_vel_w[0]), numpy(d.root_com_ang_vel_w[0]))

    def observe(self):
        d = self.robot.data
        return Observation(self.time, numpy(d.body_link_pos_w[0,self.base]),
            rotation_matrix(numpy(d.body_link_quat_w[0,self.base])), self.robot_force.copy())

    def update_markers(self):
        displacement, _ = self.human.target(self.time)
        hands = self.human_origins+displacement
        torso = hands.mean(axis=0)+[.25,0,.2]
        self.markers.visualize(translations=np.array([torso+[0,0,.4],torso,*hands]), marker_indices=[0,1,2,2])

    def apply_interaction(self):
        self.robot_force[:] = 0
        self.human_force[:] = 0
        self.max_error = 0.0
        if not self.payload:
            return
        p, h = self.config.payload, self.config.human
        shift, human_velocity = self.human.target(self.time)
        payload_com = numpy(self.payload.data.root_com_pos_w[0])
        payload_force, payload_torque = np.zeros(3), np.zeros(3)
        robot_forces, robot_torques = [], []
        for i in range(2):
            hand, hand_v = self.hand_state(i)
            point, point_v = self.payload_point(self.robot_attachments[i])
            self.max_error = max(self.max_error, float(np.linalg.norm(hand-point)))
            force = spring(hand,hand_v,point,point_v,p.stiffness,p.damping,p.max_force)
            f,t = wrench_at_com(force,point,payload_com)
            payload_force += f
            payload_torque += t
            f,t = wrench_at_com(-force,hand,numpy(self.robot.data.body_com_pos_w[0,self.wrists[i]]))
            robot_forces.append(f)
            robot_torques.append(t)
            self.robot_force += f
            point, point_v = self.payload_point(self.human_attachments[i])
            force = spring(self.human_origins[i]+shift,human_velocity,point,point_v,
                           h.stiffness,h.damping,h.max_force)
            f,t = wrench_at_com(force,point,payload_com)
            payload_force += f
            payload_torque += t
            self.human_force += force
        # World-frame wrench around each body's COM. Never apply a world force as local.
        self.robot.set_external_force_and_torque(self.tensor([robot_forces]),self.tensor([robot_torques]),
                                                body_ids=self.wrists,is_global=True)
        self.payload.set_external_force_and_torque(self.tensor([[payload_force]]),self.tensor([[payload_torque]]),is_global=True)

    def metrics(self):
        base = self.observe()
        position = numpy(self.payload.data.root_link_pos_w[0]) if self.payload else base.base_position
        rotation = rotation_matrix(numpy(self.payload.data.root_link_quat_w[0])) if self.payload else np.eye(3)
        return dict(time=self.time,robot_x=float(base.base_position[0]),robot_z=float(base.base_position[2]),
            payload_x=float(position[0]),payload_z=float(position[2]),progress=float(position[0]-self.start_x),
            payload_tilt_deg=float(np.degrees(np.arccos(np.clip(rotation[2,2],-1,1)))),
            human_horizontal_force=float(np.linalg.norm(self.human_force[:2])),
            human_vertical_force=float(self.human_force[2]),robot_vertical_force=float(-self.robot_force[2]),
            grasp_error=self.max_error,command_vx=float(self.command[0]),command_vy=float(self.command[1]))

    def step(self):
        if self.done:
            raise RuntimeError('Episode ended; call reset()')
        obs = self.observe()
        self.command = np.asarray(self.controller.command(obs,self.config.control_dt))
        if self.command.shape != (3,) or not np.isfinite(self.command).all():
            raise ValueError('Controller must return finite body-frame [vx,vy,yaw_rate]')
        q = numpy(self.robot.data.joint_pos[0,self.joints])
        dq = numpy(self.robot.data.joint_vel[0,self.joints])
        gyro = obs.base_rotation.T @ numpy(self.robot.data.body_link_ang_vel_w[0,self.base])
        self.policy.update(gyro,obs.base_rotation.T @ [0,0,-1],self.command,q,dq)
        for _ in range(round(self.config.control_dt/self.config.timestep)):
            self.apply_interaction()
            q = numpy(self.robot.data.joint_pos[0,self.joints])
            dq = numpy(self.robot.data.joint_vel[0,self.joints])
            # Passive friction is an approximation, not a claim of identical MuJoCo/PhysX dynamics.
            torque = self.policy.torques(q,dq)-.05*dq-.2*np.tanh(dq/.01)
            from .policy import TORQUE_LIMITS
            torque = np.clip(torque,-TORQUE_LIMITS,TORQUE_LIMITS)
            self.robot.set_joint_effort_target(self.tensor(torque)[None],joint_ids=self.joints)
            self.scene.write_data_to_sim()
            self.sim.step()
            self.scene.update(self.config.timestep)
            self.time += self.config.timestep
            row = self.metrics()
            up = self.observe().base_rotation[2,2]
            if not all(np.isfinite(list(row.values()))) or not torch.isfinite(self.robot.data.joint_pos).all():
                self.reason = 'nonfinite'
            elif row['robot_z'] < .45 or up < .5:
                self.reason = 'robot_fall'
            elif self.payload and row['payload_z'] < .2:
                self.reason = 'payload_drop'
            elif self.payload and row['grasp_error'] > self.config.payload.break_distance:
                self.reason = 'grasp_separation'
            elif self.payload and row['payload_tilt_deg'] > self.config.max_tilt_deg:
                self.reason = 'payload_tilt'
            if self.reason != 'running':
                self.done = True
                break
        self.update_markers()
        if not self.done and self.payload and row['progress'] >= self.config.goal_distance:
            self.done,self.reason = True,'goal_reached'
        if not self.done and self.time >= self.config.duration-1e-9:
            self.done,self.reason = True,'time_limit'
        return row
