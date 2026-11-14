"""Import only after AppLauncher. Convert the same pinned 29-joint G1 MJCF."""
from pathlib import Path
import numpy as np
import isaaclab.sim as sim_utils
from isaaclab.actuators import IdealPDActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.converters import MjcfConverter, MjcfConverterCfg
from isaaclab.utils import configclass
from .policy import JOINT_NAMES, TORQUE_LIMITS


def scene_config(config, policy, usd_path=None):
    if usd_path is None:
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension('isaacsim.asset.importer.mjcf')
        source = Path(config.asset_dir).resolve()/'g1.xml'
        if not source.is_file():
            raise FileNotFoundError('Run the pinned asset downloader before launching Isaac Lab.')
        converter = MjcfConverter(MjcfConverterCfg(asset_path=str(source),
            usd_dir=str(source.parent/'isaaclab-2.3'), usd_file_name='g1.usd',
            fix_base=False, import_inertia_tensor=True, make_instanceable=False,
            self_collision=True))
        usd_path = converter.usd_path
    else:
        usd_path = str(Path(usd_path).resolve())
        if not Path(usd_path).is_file():
            raise FileNotFoundError(usd_path)
    joint_defaults = dict(zip(JOINT_NAMES, policy.default.tolist()))
    limits = dict(zip(JOINT_NAMES, TORQUE_LIMITS.tolist()))

    @configclass
    class CarrySceneCfg(InteractiveSceneCfg):
        ground = AssetBaseCfg(prim_path='/World/Ground', spawn=sim_utils.GroundPlaneCfg(
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0)))
        light = AssetBaseCfg(prim_path='/World/Light', spawn=sim_utils.DomeLightCfg(intensity=2500))
        robot = ArticulationCfg(prim_path='{ENV_REGEX_NS}/Robot',
            spawn=sim_utils.UsdFileCfg(usd_path=usd_path,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(fix_root_link=False,
                    enabled_self_collisions=True, solver_position_iteration_count=8,
                    solver_velocity_iteration_count=4)),
            init_state=ArticulationCfg.InitialStateCfg(pos=(0,0,.793), rot=(1,0,0,0),
                joint_pos=joint_defaults, joint_vel={'.*':0.0}),
            actuators={'joints':IdealPDActuatorCfg(joint_names_expr=JOINT_NAMES,
                stiffness=0.0, damping=0.0, armature=.01, friction=0.0,
                effort_limit=limits, effort_limit_sim=limits)})
        payload = RigidObjectCfg(prim_path='{ENV_REGEX_NS}/Payload',
            spawn=sim_utils.CuboidCfg(size=(config.payload.length,.44,config.payload.thickness),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False),
                mass_props=sim_utils.MassPropertiesCfg(mass=config.payload.mass),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.6,.3,.1))),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(1,0,.7)))
    cfg = CarrySceneCfg(num_envs=1, env_spacing=4.0)
    if not config.carry:
        cfg.payload = None
    return cfg, usd_path
