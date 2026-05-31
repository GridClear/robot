#!/usr/bin/env python3
"""Export a trained policy to ONNX + policy_meta.json for deployment.

Supports SB3 (.zip, MuJoCo fallback) now; the Isaac Lab RSL-RL exporter writes
the same artifacts (see training/isaaclab_task/README). The meta file pins the
obs order / action scale / default pose so control/runtime/policy_runner.py
reproduces the training-time contract EXACTLY.

  python3 training/export_policy.py --sb3 policy/ppo_mujoco.zip --out policy/policy.onnx
"""
import argparse
import json
import os

import numpy as np

# 8-DOF, deploy/per-leg order: [abd, knee] x (FL, FR, RL, RR)
DEFAULT_POSE = [0.45, 0.70] * 4
ACTION_SCALE = 0.4
OBS_DIM = 36   # 3+3+3+3 + 8+8+8
ACT_DIM = 8
JOINT_NAMES = [
    "FL_abd", "FL_knee", "FR_abd", "FR_knee",
    "RL_abd", "RL_knee", "RR_abd", "RR_knee",
]
OBS_LAYOUT = [
    ["base_lin_vel", 3], ["base_ang_vel", 3], ["projected_gravity", 3],
    ["velocity_cmd", 3], ["joint_pos_rel_default", 8], ["joint_vel", 8],
    ["prev_action", 8],
]


def write_meta(path):
    meta = {
        "obs_dim": OBS_DIM, "act_dim": ACT_DIM,
        "obs_layout": OBS_LAYOUT, "joint_names": JOINT_NAMES,
        "default_joint_pos": DEFAULT_POSE, "action_scale": ACTION_SCALE,
        "action_is_position_delta": True, "control_hz": 50,
    }
    with open(path, "w") as f:
        json.dump(meta, f, indent=2)
    print("wrote", path)


def _load_vecnorm_stats(pkl_path):
    """Pull obs running mean/var + clip from a saved VecNormalize (pickle)."""
    import pickle
    with open(pkl_path, "rb") as f:
        vn = pickle.load(f)
    return (np.asarray(vn.obs_rms.mean, dtype=np.float32),
            np.asarray(vn.obs_rms.var, dtype=np.float32),
            float(vn.clip_obs), float(vn.epsilon))


def export_sb3(zip_path, onnx_path, vecnorm_path=None):
    import torch
    from stable_baselines3 import PPO

    model = PPO.load(zip_path, device="cpu")
    policy = model.policy
    policy.eval()

    # bake observation normalization into the graph so the ONNX takes RAW obs
    if vecnorm_path and os.path.exists(vecnorm_path):
        mean, var, clip, eps = _load_vecnorm_stats(vecnorm_path)
        mean_t = torch.tensor(mean); std_t = torch.tensor(np.sqrt(var + eps)); clip_t = float(clip)
        print(f"baking VecNormalize obs stats from {os.path.basename(vecnorm_path)}")
    else:
        mean_t = torch.zeros(OBS_DIM); std_t = torch.ones(OBS_DIM); clip_t = 1e9

    class Wrapper(torch.nn.Module):
        def __init__(self, p, mean, std, clip):
            super().__init__()
            self.p = p
            self.register_buffer("mean", mean)
            self.register_buffer("std", std)
            self.clip = clip

        def forward(self, obs):
            obs = torch.clamp((obs - self.mean) / self.std, -self.clip, self.clip)
            feats = self.p.extract_features(obs)
            latent_pi = self.p.mlp_extractor.forward_actor(feats)
            return self.p.action_net(latent_pi)   # deterministic mean action

    wrapper = Wrapper(policy, mean_t, std_t, clip_t)
    dummy = torch.zeros(1, OBS_DIM)
    torch.onnx.export(wrapper, dummy, onnx_path, input_names=["obs"],
                      output_names=["action"], opset_version=17,
                      dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}})
    print("wrote", onnx_path)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--sb3", help="path to SB3 .zip checkpoint")
    ap.add_argument("--vecnorm", help="path to VecNormalize .pkl (bakes obs norm into ONNX)")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "policy", "policy.onnx"))
    a = ap.parse_args()
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    if a.sb3:
        export_sb3(a.sb3, a.out, a.vecnorm)
    write_meta(os.path.join(os.path.dirname(a.out), "policy_meta.json"))
