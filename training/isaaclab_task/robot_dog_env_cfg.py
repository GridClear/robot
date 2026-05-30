"""Velocity-tracking locomotion env for the robot dog.

Built on Isaac Lab's ManagerBasedRLEnv. Obs/action layout intentionally matches
training/mujoco_fallback/gym_env.py and policy_meta.json:
  obs  = base_lin_vel(3) + base_ang_vel(3) + projected_gravity(3) + cmd(3)
         + joint_pos_rel(12) + joint_vel(12) + prev_action(12)  = 48
  act  = 12 joint position targets (scaled deltas around default pose)
"""
from dataclasses import MISSING

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from .robot_dog_articulation_cfg import ROBOT_DOG_CFG


@configclass
class RobotDogSceneCfg(InteractiveSceneCfg):
    terrain = TerrainImporterCfg(prim_path="/World/ground", terrain_type="plane",
                                 collision_group=-1)
    robot = ROBOT_DOG_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    light = AssetBaseCfg(prim_path="/World/light",
                         spawn=sim_utils.DomeLightCfg(intensity=1000.0,
                                                      color=(0.9, 0.9, 0.9)))


@configclass
class CommandsCfg:
    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot", resampling_time_range=(10.0, 10.0),
        rel_standing_envs=0.02, rel_heading_envs=1.0, heading_command=True,
        debug_vis=False,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-0.6, 0.8), lin_vel_y=(-0.4, 0.4),
            ang_vel_z=(-1.0, 1.0), heading=(-3.14, 3.14)),
    )


@configclass
class ActionsCfg:
    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot", joint_names=[".*"], scale=0.4, use_default_offset=True)


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(n_min=-0.1, n_max=0.1))
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-1.5, n_max=1.5))
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    # domain randomization — essential for sim-to-real on hobby servos
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", body_names=".*"),
                "static_friction_range": (0.6, 1.2), "dynamic_friction_range": (0.4, 1.0),
                "restitution_range": (0.0, 0.1), "num_buckets": 64})
    add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
                "mass_distribution_params": (-0.2, 0.3), "operation": "add"})
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity, mode="interval", interval_range_s=(8.0, 12.0),
        params={"velocity_range": {"x": (-0.4, 0.4), "y": (-0.4, 0.4)}})
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform, mode="reset",
        params={"pose_range": {"x": (-0.3, 0.3), "y": (-0.3, 0.3), "yaw": (-3.14, 3.14)},
                "velocity_range": {"x": (-0.3, 0.3), "y": (-0.3, 0.3)},
                "asset_cfg": SceneEntityCfg("robot")})
    reset_joints = EventTerm(
        func=mdp.reset_joints_by_scale, mode="reset",
        params={"position_range": (0.8, 1.2), "velocity_range": (0.0, 0.0)})


@configclass
class RewardsCfg:
    track_lin_vel = RewTerm(func=mdp.track_lin_vel_xy_exp, weight=1.5,
                            params={"command_name": "base_velocity", "std": 0.5})
    track_ang_vel = RewTerm(func=mdp.track_ang_vel_z_exp, weight=0.75,
                            params={"command_name": "base_velocity", "std": 0.5})
    lin_vel_z = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)
    ang_vel_xy = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)
    dof_torques = RewTerm(func=mdp.joint_torques_l2, weight=-2.0e-4)
    dof_acc = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    flat_orientation = RewTerm(func=mdp.flat_orientation_l2, weight=-2.5)
    alive = RewTerm(func=mdp.is_alive, weight=0.25)


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    base_contact = DoneTerm(func=mdp.bad_orientation, params={"limit_angle": 1.0})


@configclass
class RobotDogFlatEnvCfg(ManagerBasedRLEnvCfg):
    scene: RobotDogSceneCfg = RobotDogSceneCfg(num_envs=4096, env_spacing=2.5)
    commands: CommandsCfg = CommandsCfg()
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        self.decimation = 4
        self.episode_length_s = 20.0
        self.sim.dt = 0.005           # 200 Hz physics, 50 Hz control
        self.sim.render_interval = self.decimation


@configclass
class RobotDogRoughEnvCfg(RobotDogFlatEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        # swap to a rough-terrain generator once flat locomotion is solid
        self.scene.terrain.terrain_type = "generator"
