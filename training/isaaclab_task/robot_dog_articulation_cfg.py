"""Articulation config for the robot dog in Isaac Lab.

Imports model/robot_dog.urdf via Isaac's URDF->USD converter at first run (or
point usd_path at a pre-converted .usd). Joint names match model/params.yaml and
the firmware servo map.
"""
import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

_URDF = os.path.join(os.path.dirname(__file__), "..", "..", "model", "robot_dog.urdf")

DEFAULT_JOINT_POS = {
    ".*_abd": 0.45,    # sprawled splay (abduction axis mirrored per side in the URDF)
    ".*_knee": 0.7,    # bent to plant the foot
}

ROBOT_DOG_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=_URDF,
        fix_base=False,
        merge_fixed_joints=True,
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=8.0, damping=0.3),
        ),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            max_depenetration_velocity=1.0, disable_gravity=False,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.18),
        joint_pos=DEFAULT_JOINT_POS,
        joint_vel={".*": 0.0},
    ),
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_abd", ".*_knee"],
            # MG90S-class micro servos: ~0.2 N*m stall, ~6 rad/s loaded
            effort_limit=0.20, velocity_limit=6.0,
            stiffness=8.0, damping=0.3,
        ),
    },
    soft_joint_pos_limit_factor=0.95,
)
