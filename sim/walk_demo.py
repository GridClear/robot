#!/usr/bin/env python3
"""Open-loop trot-gait walk demo in MuJoCo — proves the generated model can
locomote (independent of any learned policy). Mirrors the CPG trot in the ESP32
firmware (servos.cpp gaitStep). Reports net forward displacement.

  python3 sim/walk_demo.py            # headless, prints distance traveled
  python3 sim/walk_demo.py --render out.png   # snapshot mid-stride
"""
import argparse
import os
import sys

import mujoco
import numpy as np

XML = os.path.join(os.path.dirname(__file__), "..", "model", "robot_dog.xml")
DEFAULT = np.array([0.45, 0.70] * 4, dtype=np.float32)   # 8-DOF: [abd, knee] x4
LEG_PHASE = np.array([0.0, np.pi, np.pi, 0.0])   # FL, FR, RL, RR (trot diagonals)


def trot_targets(t, speed=0.6):
    step_freq = 1.5 + 1.5 * speed
    amp = 0.35 * speed
    q = DEFAULT.copy()
    for leg in range(4):
        ph = 2 * np.pi * step_freq * t + LEG_PHASE[leg]
        lift = max(0.0, np.sin(ph))
        swing = np.cos(ph)
        b = leg * 2
        q[b + 0] = DEFAULT[b + 0] + amp * lift            # abduction lifts the leg for clearance
        q[b + 1] = DEFAULT[b + 1] + amp * swing * 1.4     # knee swings the foot fore-aft
    return q


def run(seconds=6.0, speed=0.6, render_path=None):
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    d.qpos[7:7 + len(DEFAULT)] = DEFAULT
    d.ctrl[:] = DEFAULT
    mujoco.mj_forward(m, d)
    lo, hi = m.jnt_range[1:13, 0], m.jnt_range[1:13, 1]

    x0 = float(d.qpos[0])
    n = int(seconds / m.opt.timestep)
    snap_at = n // 2
    img = None
    for i in range(n):
        t = i * m.opt.timestep
        d.ctrl[:] = np.clip(trot_targets(t, speed), lo, hi)
        mujoco.mj_step(m, d)
        if render_path and i == snap_at:
            with mujoco.Renderer(m, 480, 640) as r:
                r.update_scene(d, camera=-1)
                img = r.render()

    dist = float(d.qpos[0]) - x0
    upright = m and d.xmat[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "trunk")].reshape(3, 3)[2, 2]
    print(f"trot {seconds:.1f}s @ speed {speed}: forward {dist*100:+.1f} cm, "
          f"final height {d.qpos[2]:.3f} m, upright {upright:.2f}, "
          f"{'UPRIGHT+MOVED' if dist > 0.05 and upright > 0.6 else 'check tuning'}")
    if render_path and img is not None:
        from PIL import Image
        Image.fromarray(img).save(render_path)
        print("wrote", render_path)
    return dist


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--speed", type=float, default=0.6)
    ap.add_argument("--render", metavar="PNG")
    a = ap.parse_args()
    d = run(a.seconds, a.speed, a.render)
    sys.exit(0 if d > 0.02 else 1)
