"""Gymnasium environment wrapping the generated MuJoCo quadruped.

Observation/action layout is kept IDENTICAL to the Isaac Lab task so a policy
trained in either can be exported and run by control/runtime/policy_runner.py:

  obs (45) = [base_lin_vel(3), base_ang_vel(3), projected_gravity(3),
              cmd(3), q - q_default(12), qdot(12), prev_action(12)]
            (note: full layout incl. all terms is 48 in Isaac; here base vel is
             read from sim so we keep the same 45+cmd ordering — see OBS_DIM.)
  action (12) = position-target deltas around the default pose, scaled.

Reward mirrors the Isaac reward terms (velocity tracking + stability penalties).
"""
from __future__ import annotations
import os

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:  # allow import without gym for static checks
    gym = None

import mujoco

XML = os.path.join(os.path.dirname(__file__), "..", "..", "model", "robot_dog.xml")
DEFAULT_POSE = np.array([0.0, 0.7, -1.4] * 4, dtype=np.float32)
ACTION_SCALE = 0.4
OBS_DIM = 3 + 3 + 3 + 3 + 12 + 12 + 12   # 48
ACT_DIM = 12


class QuadrupedEnv(gym.Env if gym else object):
    metadata = {"render_modes": []}

    def __init__(self, cmd=(0.6, 0.0, 0.0), control_hz=50, render_mode=None):
        self.model = mujoco.MjModel.from_xml_path(XML)
        self.data = mujoco.MjData(self.model)
        self.cmd = np.array(cmd, dtype=np.float32)
        self.control_dt = 1.0 / control_hz
        self.sim_steps = max(1, int(round(self.control_dt / self.model.opt.timestep)))
        self.prev_action = np.zeros(ACT_DIM, dtype=np.float32)
        self.trunk_bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "trunk")
        if gym:
            self.observation_space = spaces.Box(-np.inf, np.inf, (OBS_DIM,), np.float32)
            self.action_space = spaces.Box(-1.0, 1.0, (ACT_DIM,), np.float32)
        self._step = 0
        self.max_steps = 1000

    # --- helpers ---
    def _proj_gravity(self):
        R = self.data.xmat[self.trunk_bid].reshape(3, 3)
        return R.T @ np.array([0, 0, -1.0])

    def _obs(self):
        d = self.data
        lin = d.qvel[0:3].astype(np.float32)
        ang = d.qvel[3:6].astype(np.float32)
        grav = self._proj_gravity().astype(np.float32)
        q = d.qpos[7:7 + 12].astype(np.float32)
        qd = d.qvel[6:6 + 12].astype(np.float32)
        return np.concatenate([lin, ang, grav, self.cmd, q - DEFAULT_POSE, qd, self.prev_action])

    def reset(self, *, seed=None, options=None):
        if gym:
            super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[7:7 + 12] = DEFAULT_POSE
        self.data.ctrl[:] = DEFAULT_POSE
        mujoco.mj_forward(self.model, self.data)
        self.prev_action[:] = 0
        self._step = 0
        return self._obs(), {}

    def step(self, action):
        action = np.clip(action, -1.0, 1.0).astype(np.float32)
        target = DEFAULT_POSE + ACTION_SCALE * action
        # respect joint ranges
        lo = self.model.jnt_range[1:13, 0]
        hi = self.model.jnt_range[1:13, 1]
        self.data.ctrl[:] = np.clip(target, lo, hi)
        for _ in range(self.sim_steps):
            mujoco.mj_step(self.model, self.data)

        d = self.data
        vx = float(d.qvel[0])
        # reward: track forward command, stay upright, penalize effort/wobble
        track = np.exp(-4.0 * (vx - self.cmd[0]) ** 2)
        upright = self.data.xmat[self.trunk_bid].reshape(3, 3)[2, 2]
        height = float(d.qpos[2])
        r = (1.5 * track
             - 0.5 * abs(float(d.qvel[1]))            # lateral drift
             - 0.05 * float(np.sum(np.square(d.qvel[3:6])))   # body rotation
             - 0.01 * float(np.sum(np.square(action - self.prev_action)))  # action rate
             + 0.2)                                    # alive bonus
        self.prev_action[:] = action
        self._step += 1

        fell = upright < 0.5 or height < 0.06
        terminated = bool(fell)
        truncated = self._step >= self.max_steps
        if fell:
            r -= 2.0
        return self._obs(), float(r), terminated, truncated, {"vx": vx, "upright": upright}


def make_env(cmd=(0.6, 0.0, 0.0)):
    def _f():
        return QuadrupedEnv(cmd=cmd)
    return _f
