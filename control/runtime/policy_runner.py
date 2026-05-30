"""Run a trained ONNX locomotion policy and stream joint targets to the robot.

The policy does NOT run on the ESP32. It runs here (companion computer on the
robot, or a host during bring-up): build the observation -> ONNX inference ->
12 joint targets -> `set_joints` over WiFi WebSocket at control_hz.

Two backends:
  --sim    drive the MuJoCo model (closed-loop validation on this machine)
  --robot HOST   stream to a real/mock ESP32 over WebSocket

Observation order MUST match policy_meta.json (written by export_policy.py).
On a real robot, base_lin_vel/base_ang_vel/projected_gravity come from an IMU;
in --sim they are read from the simulator.
"""
from __future__ import annotations
import argparse
import asyncio
import json
import os

import numpy as np

HERE = os.path.dirname(__file__)
POLICY_DIR = os.path.join(HERE, "..", "..", "policy")


def load_meta(d=POLICY_DIR):
    with open(os.path.join(d, "policy_meta.json")) as f:
        return json.load(f)


class Policy:
    def __init__(self, onnx_path):
        import onnxruntime as ort
        self.sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
        self.iname = self.sess.get_inputs()[0].name

    def act(self, obs: np.ndarray) -> np.ndarray:
        out = self.sess.run(None, {self.iname: obs[None, :].astype(np.float32)})[0]
        return out[0]


def run_sim(meta, policy, steps=1000):
    """Closed-loop in MuJoCo — proves the export + obs construction are correct."""
    import sys
    sys.path.insert(0, os.path.join(HERE, "..", "..", "training", "mujoco_fallback"))
    from gym_env import QuadrupedEnv
    env = QuadrupedEnv()
    obs, _ = env.reset()
    total = 0.0
    x0 = float(env.data.qpos[0])
    for _ in range(steps):
        a = policy.act(obs)
        obs, r, term, trunc, info = env.step(a)
        total += r
        if term or trunc:
            break
    dist = float(env.data.qpos[0]) - x0
    print(f"[sim] return={total:.1f}  forward={dist*100:+.1f} cm  "
          f"upright={info.get('upright', 0):.2f}")


async def run_robot(meta, policy, host, port, hz):
    """Stream set_joints to a (mock or real) ESP32 over WebSocket.

    Base-velocity / gravity observations need an IMU on the robot; here they are
    zeroed as a placeholder (replace with a real IMU read). This path validates
    the transport + framing, not gait quality.
    """
    import sys
    sys.path.insert(0, HERE)
    from transport_wifi import WifiWs
    import protocol as P

    default = np.array(meta["default_joint_pos"], dtype=np.float32)
    scale = meta["action_scale"]
    prev = np.zeros(12, dtype=np.float32)
    cmd = np.array([0.6, 0.0, 0.0], dtype=np.float32)
    dt = 1.0 / hz

    async with WifiWs(host, port) as ws:
        for _ in range(int(5 * hz)):  # 5 seconds
            # placeholder obs: IMU terms zeroed; joints assumed at last target
            obs = np.concatenate([
                np.zeros(3), np.zeros(3), np.array([0, 0, -1.0]), cmd,
                np.zeros(12), np.zeros(12), prev,
            ]).astype(np.float32)
            a = np.clip(policy.act(obs), -1, 1)
            q = default + scale * a
            await ws.send_nowait(P.set_joints(q.tolist()))
            prev = a
            await asyncio.sleep(dt)
    print("[robot] streamed 5 s of set_joints")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx", default=os.path.join(POLICY_DIR, "policy.onnx"))
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--robot", metavar="HOST")
    ap.add_argument("--port", type=int, default=80)
    ap.add_argument("--hz", type=int, default=50)
    a = ap.parse_args()

    meta = load_meta()
    policy = Policy(a.onnx)
    if a.robot:
        asyncio.run(run_robot(meta, policy, a.robot, a.port, a.hz))
    else:
        run_sim(meta, policy)
