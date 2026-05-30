"""Isaac Lab task package for the robot dog (velocity-tracking locomotion).

Install as an Isaac Lab extension or place on PYTHONPATH, then:

  import robot_dog_task   # registers the gym ids below

  ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
      --task Isaac-Velocity-Flat-RobotDog-v0 --headless

Gym ids:
  Isaac-Velocity-Flat-RobotDog-v0    flat ground (start here)
  Isaac-Velocity-Rough-RobotDog-v0   rough terrain curriculum (after flat works)
"""
import gymnasium as gym

from . import agents
from .robot_dog_env_cfg import RobotDogFlatEnvCfg, RobotDogRoughEnvCfg

gym.register(
    id="Isaac-Velocity-Flat-RobotDog-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": RobotDogFlatEnvCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RobotDogPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Velocity-Rough-RobotDog-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": RobotDogRoughEnvCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RobotDogPPORunnerCfg",
    },
)
