"""Tune the scripted trot in the corrected-physics MuJoCo model.

The firmware (robotdog8.ino) runs a 2-DOF diagonal trot: per leg, abduction and
knee follow phase-offset sinusoids around a stance pose. This sweeps the same
scalar knobs the firmware exposes (freq, abd amplitude, knee amplitude, phase)
on the realistic 0.7 kg / 0.2 N*m model and ranks them by forward distance while
upright. Mirror handling lives in the MJCF abduction axis, so we command all legs
symmetrically (radians); results convert back to firmware degrees via deg=90+57.3*rad.

Run: .venv/bin/python training/gait_sweep.py
"""
from __future__ import annotations
import math
import os
import numpy as np
import mujoco

XML = os.path.join(os.path.dirname(__file__), "..", "model", "robot_dog.xml")
# deploy joint order = MJCF ctrl order: [FL,FR,RL,RR] x [abd,knee]
TROT_PHASE = [0.0, math.pi, math.pi, 0.0]   # FL, FR, RL, RR (diagonal pairs)
ABD_STANCE, KNEE_STANCE = 0.45, 0.70        # sprawled default pose (radians)


def rollout(model, data, freq, A, K, phase, secs=4.0, abd0=ABD_STANCE, knee0=KNEE_STANCE):
    mujoco.mj_resetData(model, data)
    data.qpos[2] = 0.09
    data.qpos[7:15] = [abd0, knee0] * 4
    data.ctrl[:] = data.qpos[7:15]
    mujoco.mj_forward(model, data)
    trunk = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "trunk")
    x0 = float(data.qpos[0])
    dt = model.opt.timestep
    n = int(secs / dt)
    w = 2 * math.pi * freq
    min_h, min_up, fell = 1.0, 1.0, False
    for k in range(n):
        t = k * dt
        for leg in range(4):
            ph = w * t + TROT_PHASE[leg]
            data.ctrl[leg*2]   = abd0  + A * math.sin(ph)
            data.ctrl[leg*2+1] = knee0 + K * math.sin(ph + phase)
        mujoco.mj_step(model, data)
        up = float(data.xmat[trunk].reshape(3, 3)[2, 2])
        h = float(data.qpos[2])
        min_h, min_up = min(min_h, h), min(min_up, up)
        if up < 0.4 or h < 0.025:
            fell = True; break
    fwd = float(data.qpos[0]) - x0
    lateral = abs(float(data.qpos[1]))
    return {"fwd_cm": fwd*100, "lateral_cm": lateral*100, "min_h": min_h,
            "min_up": min_up, "fell": fell, "speed_cms": fwd*100/secs}


def main():
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)

    # 1) reproduce the CURRENT firmware gait (amp 28deg, kamp 30deg, freq 0.6, phase 90)
    cur = rollout(model, data, freq=0.6, A=math.radians(28), K=math.radians(30), phase=math.pi/2)
    print(f"[current firmware gait] fwd={cur['fwd_cm']:+.1f}cm  speed={cur['speed_cms']:+.1f}cm/s  "
          f"lateral={cur['lateral_cm']:.1f}cm  min_h={cur['min_h']:.3f} min_up={cur['min_up']:.2f} fell={cur['fell']}")

    # 2) sweep
    freqs  = [0.8, 1.2, 1.6, 2.0, 2.5]
    Adeg   = [10, 18, 26, 34]
    Kdeg   = [15, 25, 35, 45]
    phases = [45, 90, 135]
    results = []
    for f in freqs:
        for a in Adeg:
            for kk in Kdeg:
                for ph in phases:
                    r = rollout(model, data, f, math.radians(a), math.radians(kk), math.radians(ph))
                    r.update(freq=f, amp_deg=a, kamp_deg=kk, phase_deg=ph)
                    results.append(r)
    # rank: must stay upright, then max forward speed, prefer low lateral drift
    ok = [r for r in results if not r["fell"] and r["fwd_cm"] > 0]
    ok.sort(key=lambda r: (r["speed_cms"] - 0.3*r["lateral_cm"]), reverse=True)
    print(f"\n{len(ok)}/{len(results)} gaits walked forward without falling. Top 8:")
    print(f"{'freq':>5} {'amp':>4} {'kamp':>5} {'phase':>6} {'fwd_cm':>7} {'cm/s':>6} {'lat_cm':>7} {'min_h':>6}")
    for r in ok[:8]:
        print(f"{r['freq']:>5} {r['amp_deg']:>4} {r['kamp_deg']:>5} {r['phase_deg']:>6} "
              f"{r['fwd_cm']:>7.1f} {r['speed_cms']:>6.1f} {r['lateral_cm']:>7.1f} {r['min_h']:>6.3f}")


if __name__ == "__main__":
    main()
