"""Run a trained ONNX locomotion policy and stream joint targets to the robot.

The policy does NOT run on the ESP32. It runs here (companion computer on the
robot, or a host during bring-up): build the observation -> ONNX inference ->
8 joint targets -> firmware at control_hz.

Backends:
  --sim                 drive the MuJoCo model (closed-loop validation here)
  --robot [HOST]        stream to a real/mock ESP32 (bare --robot -> robotdog.local)
     --transport wifi     WiFi WS JSON set_joints (DEFAULT; firmware/ PlatformIO build
                          or mock_esp32). No USB — commands go over the network.
     --transport serial   legacy robotdog8.ino over /dev/ttyUSB0 (line proto)

Observation order MUST match the policy metadata (policy_isaac_meta.json for the
8-DOF Isaac policy; policy_meta.json for the older fallback). The Isaac ONNX
consumes obs joint sub-vectors and emits actions in the type-grouped "isaac"
joint order; the firmware/servos use the per-leg "deploy" order, so we remap with
isaac_to_deploy_index / deploy_to_isaac_index from the metadata.

On a real robot, base_lin_vel/base_ang_vel/projected_gravity come from an IMU; in
--sim they are read from the simulator. With no IMU those terms are zeroed (the
--robot path then validates transport + framing, not gait quality).
"""
from __future__ import annotations
import argparse
import asyncio
import json
import math
import os

import numpy as np

HERE = os.path.dirname(__file__)
POLICY_DIR = os.path.join(HERE, "..", "..", "policy")

# Firmware deploy channel order ch0..7 = FL/FR/RL/RR (abd, knee). Right-side legs
# (FR ch2,3 ; RR ch6,7) are mirror-mounted -> dir = -1, matching robotdog8.ino.
DEPLOY_DIR = np.array([1, 1, -1, -1, 1, 1, -1, -1], dtype=np.float32)
DEPLOY_LO = np.array([15, 15, 15, 15, 15, 15, 15, 15], dtype=np.float32)
DEPLOY_HI = np.array([165, 165, 165, 165, 165, 165, 165, 180], dtype=np.float32)


def load_meta(name=None, d=POLICY_DIR):
    """Load policy metadata. Default: prefer the Isaac meta if present."""
    if name is None:
        name = ("policy_isaac_meta.json"
                if os.path.exists(os.path.join(d, "policy_isaac_meta.json"))
                else "policy_meta.json")
    # accept an absolute/cwd-relative path as-is, else resolve under POLICY_DIR
    path = name if os.path.exists(name) else os.path.join(d, os.path.basename(name))
    with open(path) as f:
        return json.load(f)


class MetaFields:
    """Normalized view over either the Isaac meta or the older 8-DOF meta."""

    def __init__(self, meta):
        self.act_dim = int(meta.get("act_dim", 8))
        self.action_scale = float(meta.get("action_scale", 0.4))
        self.obs_layout = meta.get("obs_layout", [
            ["base_lin_vel", 3], ["base_ang_vel", 3], ["projected_gravity", 3],
            ["velocity_cmd", 3], ["joint_pos_rel_default", self.act_dim],
            ["joint_vel", self.act_dim], ["prev_action", self.act_dim],
        ])
        n = self.act_dim
        # isaac<->deploy joint remap (identity for the older meta with no keys)
        self.i2d = np.array(meta.get("isaac_to_deploy_index", list(range(n))))
        self.d2i = np.array(meta.get("deploy_to_isaac_index", list(range(n))))
        # default joint pose, in each order
        self.default_isaac = np.array(
            meta.get("default_joint_pos_isaac_order")
            or meta.get("default_joint_pos", [0.0] * n), dtype=np.float32)
        self.default_deploy = np.array(
            meta.get("default_joint_pos_deploy_order")
            or meta.get("default_joint_pos", [0.0] * n), dtype=np.float32)
        # byte offsets of the three joint sub-vectors within the obs vector
        self.joint_slices = []
        off = 0
        joint_names = {"joint_pos_rel_default", "joint_pos_rel", "joint_vel",
                       "prev_action", "actions"}
        for name, dim in self.obs_layout:
            if name in joint_names and dim == n:
                self.joint_slices.append(slice(off, off + dim))
            off += dim
        self.obs_dim = off


def reorder_obs_joint_blocks(obs, mf: MetaFields, index):
    """Return a copy of obs with each joint sub-vector reordered by `index`."""
    out = obs.copy()
    for sl in mf.joint_slices:
        out[sl] = obs[sl][index]
    return out


class Policy:
    def __init__(self, onnx_path):
        import onnxruntime as ort
        self.sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
        self.iname = self.sess.get_inputs()[0].name

    def act(self, obs: np.ndarray) -> np.ndarray:
        out = self.sess.run(None, {self.iname: obs[None, :].astype(np.float32)})[0]
        return out[0]


def isaac_to_servo_degrees(target_rad_isaac, mf: MetaFields):
    """Map Isaac-order joint targets (radians) to deploy-order servo degrees.

    Joint zero -> 90 deg; mirror handled by DEPLOY_DIR. Clamped to servo limits.
    """
    target_rad_deploy = target_rad_isaac[mf.i2d]
    deg = 90.0 + DEPLOY_DIR * np.degrees(target_rad_deploy)
    return np.clip(deg, DEPLOY_LO, DEPLOY_HI)


def run_sim(mf: MetaFields, policy, steps=1000, remap=True, cmd=(0.6, 0.0, 0.0)):
    """Closed-loop in MuJoCo — proves export + obs construction + joint remap.

    gym_env returns obs joint sub-vectors and consumes actions in DEPLOY (MJCF)
    order. The Isaac ONNX speaks ISAAC order, so we reorder obs deploy->isaac
    before inference and the action isaac->deploy before stepping. `remap=False`
    is the negative control (should fall fast).
    """
    import sys
    sys.path.insert(0, os.path.join(HERE, "..", "..", "training", "mujoco_fallback"))
    from gym_env import QuadrupedEnv
    env = QuadrupedEnv(cmd=tuple(cmd))
    obs, _ = env.reset()
    total = 0.0
    x0 = float(env.data.qpos[0])
    vxs, ups = [], []
    term = trunc = False
    info = {}
    steps_done = 0
    for _ in range(steps):
        obs_isaac = reorder_obs_joint_blocks(obs, mf, mf.d2i) if remap else obs
        a_isaac = np.clip(policy.act(obs_isaac), -1, 1)
        a_deploy = a_isaac[mf.i2d] if remap else a_isaac
        obs, r, term, trunc, info = env.step(a_deploy)
        total += r
        vxs.append(info.get("vx", 0.0))
        ups.append(info.get("upright", 0.0))
        steps_done += 1
        if term or trunc:
            break
    dist = float(env.data.qpos[0]) - x0
    print(f"[sim] remap={remap} steps={steps_done} terminated={term} "
          f"return={total:.1f} forward={dist*100:+.1f}cm "
          f"mean_vx={np.mean(vxs):+.3f} mean_upright={np.mean(ups):.2f}")


def _build_obs_isaac(mf: MetaFields, cmd, prev_isaac, lin=None, ang=None, grav=None,
                     qpos_rel=None, qvel=None):
    """Assemble a 36-d obs in ISAAC order per obs_layout (zeros where no sensor)."""
    n = mf.act_dim
    parts = {
        "base_lin_vel": np.zeros(3) if lin is None else lin,
        "base_ang_vel": np.zeros(3) if ang is None else ang,
        "projected_gravity": np.array([0, 0, -1.0]) if grav is None else grav,
        "velocity_cmd": cmd,
        "joint_pos_rel_default": np.zeros(n) if qpos_rel is None else qpos_rel,
        "joint_pos_rel": np.zeros(n) if qpos_rel is None else qpos_rel,
        "joint_vel": np.zeros(n) if qvel is None else qvel,
        "prev_action": prev_isaac,
        "actions": prev_isaac,
    }
    return np.concatenate([parts[name] for name, _ in mf.obs_layout]).astype(np.float32)


def _start_capture(capture, cmd_vec, max_speed_mps_default=None):
    """Build a CaptureRun + DistanceTrigger from a capture-config dict, or None.

    Returns (run, trigger, speed_mps). The robot has no odometry, so distance is
    dead-reckoned: speed_mps = |commanded vx| * max_speed_mps (see capture_coordinator).
    """
    if not capture:
        return None, None, 0.0
    from capture_coordinator import CaptureRun, DistanceTrigger
    step_m = capture["step_m"]
    max_speed_mps = capture["max_speed_mps"]
    cmd_list = [float(x) for x in cmd_vec]
    run = CaptureRun(capture["phone_url"], capture["root"], run_meta={
        "step_m": step_m, "max_speed_mps": max_speed_mps, "mode": "policy_runner",
        "velocity_cmd": cmd_list})
    trig = DistanceTrigger(step_m, lambda total: run.submit(total, cmd_list))
    speed_mps = abs(cmd_list[0]) * max_speed_mps
    return run, trig, speed_mps


def run_robot_serial(mf: MetaFields, policy, port, hz, secs, cmd_vec, dry_run,
                     on_exit="x", capture=None):
    """Stream the policy to the real robotdog8.ino over serial (line protocol)."""
    import sys
    import time
    sys.path.insert(0, HERE)
    from transport_serial import SerialLink, set8

    n = mf.act_dim
    prev = np.zeros(n, dtype=np.float32)
    cmd = np.array(cmd_vec, dtype=np.float32)
    dt = 1.0 / hz
    nsteps = int(secs * hz)

    def one_frame():
        nonlocal prev
        obs = _build_obs_isaac(mf, cmd, prev)            # IMU/joint terms zeroed
        a = np.clip(policy.act(obs), -1, 1)
        target_rad = mf.default_isaac + mf.action_scale * a
        deg = isaac_to_servo_degrees(target_rad, mf)
        prev = a
        return deg

    if dry_run:
        print(f"[dry-run] serial port={port} hz={hz} secs={secs} cmd={cmd.tolist()}")
        neutral = isaac_to_servo_degrees(mf.default_isaac, mf)
        print(f"[dry-run] neutral (action=0) degrees: {set8(neutral)}")
        for i in range(min(5, nsteps)):
            print(f"[dry-run] frame {i}: {set8(one_frame())}")
        return

    run, trig, speed_mps = _start_capture(capture, cmd_vec)
    with SerialLink(port) as link:
        link.send_line("x")                               # stop any gait, hold
        time.sleep(0.2)
        t0 = time.time()
        for _ in range(nsteps):
            link.send_line(set8(one_frame()))
            if trig:
                trig.update(speed_mps * dt)               # dead-reckon -> capture @ ~1cm
            time.sleep(dt)
        link.send_line(on_exit)                           # "x" hold (safe) or "r" relax
        print(f"[robot] streamed {secs:.1f}s of policy targets "
              f"({nsteps} frames @ {hz}Hz); sent '{on_exit}' on exit")
    if run:
        run.close()


async def run_robot_wifi(mf: MetaFields, policy, host, port, hz, secs, cmd_vec, capture=None):
    """Stream set_joints to a mock/WiFi ESP32 over WebSocket (JSON)."""
    import sys
    sys.path.insert(0, HERE)
    from transport_wifi import WifiWs
    import protocol as P

    n = mf.act_dim
    prev = np.zeros(n, dtype=np.float32)
    cmd = np.array(cmd_vec, dtype=np.float32)
    dt = 1.0 / hz
    run, trig, speed_mps = _start_capture(capture, cmd_vec)
    try:
        async with WifiWs(host, port) as ws:
            for _ in range(int(secs * hz)):
                obs = _build_obs_isaac(mf, cmd, prev)
                a = np.clip(policy.act(obs), -1, 1)
                q = mf.default_isaac + mf.action_scale * a    # radians, isaac order
                await ws.send_nowait(P.set_joints(q[mf.i2d].tolist()))  # deploy order
                if trig:
                    trig.update(speed_mps * dt)               # dead-reckon -> capture @ ~1cm
                prev = a
                await asyncio.sleep(dt)
        print(f"[robot] streamed {secs:.1f}s of set_joints over WiFi")
    finally:
        if run:
            run.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx", default=os.path.join(POLICY_DIR, "policy_isaac.onnx"))
    ap.add_argument("--meta", default=None, help="metadata file (default: auto)")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--no-remap", action="store_true",
                    help="sim negative control: skip the isaac<->deploy remap")
    _robot_host = os.environ.get("ROBOTDOG_HOST", "robotdog.local")
    ap.add_argument("--robot", metavar="HOST", nargs="?", const=_robot_host,
                    help=f"stream to robot; bare --robot uses {_robot_host} (or $ROBOTDOG_HOST). "
                         "Omit entirely to run --sim.")
    ap.add_argument("--transport", choices=["serial", "wifi"], default="wifi",
                    help="wifi (default, over the network) or serial (legacy robotdog8.ino USB)")
    ap.add_argument("--port", default=None,
                    help="WS port for wifi (default 80), or serial device for serial "
                         "(default /dev/ttyUSB0)")
    ap.add_argument("--hz", type=int, default=50)
    ap.add_argument("--secs", type=float, default=5.0)
    ap.add_argument("--cmd", default="0.6,0,0", help="velocity command vx,vy,wz")
    ap.add_argument("--dry-run", action="store_true", help="print serial lines, no port")
    ap.add_argument("--on-exit", choices=["x", "r", "c"], default="x")
    # --- gaussian-splatting photo capture (defaults shared with capture_coordinator) ---
    import sys as _sys
    _sys.path.insert(0, HERE)
    from capture_coordinator import (DEFAULT_PHONE_URL, DEFAULT_ROOT,
                                      DEFAULT_STEP_M, DEFAULT_MAX_SPEED_MPS)
    ap.add_argument("--capture", action="store_true",
                    help="pull a phone photo every --step-m of dead-reckoned travel")
    ap.add_argument("--phone-url", default=DEFAULT_PHONE_URL,
                    help="phone still URL (default: %(default)s; or set $PHONE_URL)")
    ap.add_argument("--step-m", type=float, default=DEFAULT_STEP_M,
                    help="capture interval (m); default 1cm")
    ap.add_argument("--max-speed-mps", type=float, default=DEFAULT_MAX_SPEED_MPS,
                    help="m/s at commanded vx=1.0 — CALIBRATE (open-loop estimate)")
    ap.add_argument("--gs-root", default=DEFAULT_ROOT,
                    help="photo root (default $GS_PHOTOS_ROOT or ~/robotdog/gaussian_splatting_photos)")
    a = ap.parse_args()

    mf = MetaFields(load_meta(a.meta))
    policy = Policy(a.onnx)
    cmd_vec = [float(x) for x in a.cmd.split(",")]

    capture = None
    if a.capture:
        capture = {"phone_url": a.phone_url, "step_m": a.step_m,
                   "max_speed_mps": a.max_speed_mps, "root": a.gs_root}

    if a.robot:
        if a.transport == "serial":
            run_robot_serial(mf, policy, a.port or "/dev/ttyUSB0", a.hz, a.secs,
                             cmd_vec, a.dry_run, a.on_exit, capture=capture)
        else:
            asyncio.run(run_robot_wifi(mf, policy, a.robot, int(a.port or 80), a.hz,
                                       a.secs, cmd_vec, capture=capture))
    else:
        run_sim(mf, policy, remap=not a.no_remap, cmd=cmd_vec)
