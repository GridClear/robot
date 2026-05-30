#!/usr/bin/env python3
"""PPO training on the MuJoCo quadruped (CPU/GPU via stable-baselines3).

This is the runnable fallback to the Isaac Lab GPU pipeline (same obs/action
layout, so the exported policy is interchangeable). On the GB10's 20 cores use
many parallel envs:

  python3 training/mujoco_fallback/train_ppo.py --envs 16 --steps 3_000_000

Outputs a SB3 .zip checkpoint; convert to ONNX with training/export_policy.py.
For a real walking gait expect millions of steps — this is the insurance path;
the Isaac Lab task (training/isaaclab_task) is the primary, GPU-parallel route.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from gym_env import QuadrupedEnv  # noqa: E402

OUT = os.path.join(os.path.dirname(__file__), "..", "..", "policy")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--envs", type=int, default=8)
    ap.add_argument("--steps", type=int, default=200_000)
    ap.add_argument("--out", default=os.path.join(OUT, "ppo_mujoco"))
    args = ap.parse_args()

    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize

    venv = SubprocVecEnv([lambda: QuadrupedEnv() for _ in range(args.envs)])
    venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0)

    model = PPO("MlpPolicy", venv, n_steps=512, batch_size=2048, gae_lambda=0.95,
                gamma=0.99, n_epochs=5, ent_coef=0.0, learning_rate=3e-4,
                policy_kwargs=dict(net_arch=[256, 128, 64]), verbose=1,
                device="auto")
    model.learn(total_timesteps=args.steps, progress_bar=False)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    model.save(args.out)
    venv.save(args.out + "_vecnorm.pkl")
    print(f"saved {args.out}.zip")


if __name__ == "__main__":
    main()
