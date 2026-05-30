#!/usr/bin/env python3
"""Local MuJoCo sanity check for the generated quadruped.

Modes:
  python3 sim/view_mujoco.py            # interactive viewer (needs a display)
  python3 sim/view_mujoco.py --check    # headless: drive each joint, assert the
                                         # correct foot moves; prints a report.
  python3 sim/view_mujoco.py --render out.png   # headless offscreen snapshot

The --check mode is what runs in CI / on this iGPU box (no display needed).
It verifies the kinematics are wired correctly: actuating FL_* must move the
FL foot and (mostly) leave the RR foot alone, etc.
"""
import argparse
import os
import sys

import mujoco
import numpy as np

XML = os.path.join(os.path.dirname(__file__), "..", "model", "robot_dog.xml")
LEGS = ["FL", "FR", "RL", "RR"]


def _foot_geom_ids(m):
    """Map leg -> geom id of its foot sphere (the sphere geom inside each shank body)."""
    ids = {}
    for leg in LEGS:
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{leg}_shank")
        for g in range(m.ngeom):
            if m.geom_bodyid[g] == bid and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE:
                ids[leg] = g
    return ids


def check():
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    foot_geom = _foot_geom_ids(m)

    def _foot_world(leg):
        return d.geom_xpos[foot_geom[leg]].copy()
    # hold the body in the air so legs swing freely (disable gravity for the test)
    m.opt.gravity[:] = 0
    mujoco.mj_forward(m, d)

    act_name = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(m.nu)}
    ok = True
    print("Per-leg kinematic isolation (actuate one leg's knee, measure foot motion):")
    for leg in LEGS:
        mujoco.mj_resetData(m, d)
        mujoco.mj_forward(m, d)
        base = {l: _foot_world(l) for l in LEGS}
        # drive this leg's knee to mid-flex
        d.ctrl[:] = 0
        d.ctrl[act_name[f"{leg}_knee_act"]] = -1.4
        pin_q = d.qpos[:7].copy()   # freejoint pose: pin the trunk so legs move in isolation
        for _ in range(400):
            mujoco.mj_step(m, d)
            d.qpos[:7] = pin_q
            d.qvel[:6] = 0
        moved = {l: float(np.linalg.norm(_foot_world(l) - base[l])) for l in LEGS}
        target = moved[leg]
        others = max(v for l, v in moved.items() if l != leg)
        verdict = "OK" if target > 0.02 and target > 3 * max(others, 1e-9) else "FAIL"
        ok = ok and verdict == "OK"
        print(f"  {leg}: moved {target*1000:5.1f} mm  (max other leg {others*1000:4.1f} mm)  {verdict}")

    # default standing pose is stable under gravity
    m.opt.gravity[:] = [0, 0, -9.81]
    mujoco.mj_resetData(m, d)
    for i in range(m.nu):
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        # set ctrl to the default pose for each joint type
        for jt, val in (("hip", 0.0), ("thigh", 0.7), ("knee", -1.4)):
            if nm.endswith(f"_{jt}_act"):
                d.ctrl[i] = val
    for _ in range(800):
        mujoco.mj_step(m, d)
    z = float(d.qpos[2])
    upright = d.xmat[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "trunk")].reshape(3, 3)[2, 2]
    stable = z > 0.05 and upright > 0.8 and not np.isnan(d.qpos).any()
    print(f"\nStanding pose under gravity: trunk z={z:.3f} m, upright={upright:.2f}  "
          f"{'OK' if stable else 'FAIL'}")
    ok = ok and stable
    print("\nRESULT:", "ALL OK" if ok else "FAILURES PRESENT")
    return 0 if ok else 1


def render(path):
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    for i in range(m.nu):
        nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        for jt, val in (("thigh", 0.7), ("knee", -1.4)):
            if nm.endswith(f"_{jt}_act"):
                d.ctrl[i] = val
    for _ in range(500):
        mujoco.mj_step(m, d)
    with mujoco.Renderer(m, height=480, width=640) as r:
        r.update_scene(d, camera=-1)
        img = r.render()
    try:
        from PIL import Image
        Image.fromarray(img).save(path)
    except ImportError:
        import numpy as _np
        _np.save(path + ".npy", img)
        path = path + ".npy"
    print("wrote", path)
    return 0


def view():
    import mujoco.viewer
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    t = 0.0
    with mujoco.viewer.launch_passive(m, d) as v:
        while v.is_running():
            t += m.opt.timestep
            # gentle sinusoidal jiggle so you can see all joints move
            for i in range(m.nu):
                nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                if nm.endswith("_thigh_act"):
                    d.ctrl[i] = 0.7 + 0.3 * np.sin(2 * t)
                elif nm.endswith("_knee_act"):
                    d.ctrl[i] = -1.4 + 0.3 * np.sin(2 * t)
            mujoco.mj_step(m, d)
            v.sync()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="headless kinematic checks")
    ap.add_argument("--render", metavar="PNG", help="headless offscreen snapshot")
    args = ap.parse_args()
    if args.check:
        sys.exit(check())
    elif args.render:
        sys.exit(render(args.render))
    else:
        view()
