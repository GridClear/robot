#!/usr/bin/env python3
"""Generate robot_dog.urdf, robot_dog.xml (MJCF) and servo_map.json from params.yaml.

One leg is described once (make_leg_*) and replicated across all four corners,
so the "configure one leg, copy to the rest" requirement is enforced structurally:
there is exactly one place that defines a leg.

Usage:  python3 model/generate.py
Outputs are written next to params.yaml.
"""
import json
import os
import xml.etree.ElementTree as ET
from xml.dom import minidom

HERE = os.path.dirname(os.path.abspath(__file__))


def load_params():
    import yaml
    with open(os.path.join(HERE, "params.yaml")) as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------
# inertia helpers (simple, physically valid, tune later)
# ---------------------------------------------------------------------------
def cyl_inertia(mass, r, length, axis):
    """Solid-cylinder inertia, principal axis along 'x'|'y'|'z'."""
    i_long = 0.5 * mass * r * r
    i_tran = mass * (3 * r * r + length * length) / 12.0
    if axis == "x":
        return i_long, i_tran, i_tran
    if axis == "y":
        return i_tran, i_long, i_tran
    return i_tran, i_tran, i_long  # z


def box_inertia(mass, sx, sy, sz):
    ixx = mass * (sy * sy + sz * sz) / 12.0
    iyy = mass * (sx * sx + sz * sz) / 12.0
    izz = mass * (sx * sx + sy * sy) / 12.0
    return ixx, iyy, izz


def sphere_inertia(mass, r):
    i = 0.4 * mass * r * r
    return i, i, i


# ---------------------------------------------------------------------------
# URDF
# ---------------------------------------------------------------------------
def _inertial(link, mass, com, I):
    el = ET.SubElement(link, "inertial")
    ET.SubElement(el, "origin", xyz=f"{com[0]} {com[1]} {com[2]}", rpy="0 0 0")
    ET.SubElement(el, "mass", value=f"{mass}")
    ixx, iyy, izz = I
    ET.SubElement(el, "inertia", ixx=f"{ixx}", ixy="0", ixz="0",
                  iyy=f"{iyy}", iyz="0", izz=f"{izz}")


def _visual_collision(link, geom_tag, geom_attrs, origin_xyz, origin_rpy, color):
    for kind in ("visual", "collision"):
        v = ET.SubElement(link, kind)
        ET.SubElement(v, "origin", xyz=origin_xyz, rpy=origin_rpy)
        g = ET.SubElement(v, "geometry")
        ET.SubElement(g, geom_tag, **geom_attrs)
        if kind == "visual":
            mat = ET.SubElement(v, "material", name=color)
            rgba = {"grey": "0.4 0.4 0.45 1", "blue": "0.2 0.4 0.8 1"}[color]
            ET.SubElement(mat, "color", rgba=rgba)


def build_urdf(p):
    leg = p["leg"]
    robot = ET.Element("robot", name=p["robot_name"])

    # --- trunk / base_link ---
    sx, sy, sz = p["trunk"]["size"]
    base = ET.SubElement(robot, "link", name="base_link")
    _inertial(base, p["trunk"]["mass"], (0, 0, 0),
              box_inertia(p["trunk"]["mass"], sx, sy, sz))
    _visual_collision(base, "box", {"size": f"{sx} {sy} {sz}"}, "0 0 0", "0 0 0", "blue")

    jdefs = p["joints"]
    for prefix, c in p["corners"].items():
        outb = -1.0 if c["mirror"] else 1.0   # +y outboard for left, -y for right
        cx, cy = c["xy"]
        hip_len = leg["hip_link"]["length"]
        th_len = leg["thigh"]["length"]
        sh_len = leg["shank"]["length"]

        # hip_link (cylinder along y)
        hl = ET.SubElement(robot, "link", name=f"{prefix}_hip_link")
        _inertial(hl, leg["hip_link"]["mass"], (0, outb * hip_len / 2, 0),
                  cyl_inertia(leg["hip_link"]["mass"], leg["hip_link"]["radius"], hip_len, "y"))
        _visual_collision(hl, "cylinder",
                          {"radius": f"{leg['hip_link']['radius']}", "length": f"{hip_len}"},
                          f"0 {outb*hip_len/2} 0", "1.5708 0 0", "grey")

        j = ET.SubElement(robot, "joint", name=f"{prefix}_hip", type="revolute")
        ET.SubElement(j, "parent", link="base_link")
        ET.SubElement(j, "child", link=f"{prefix}_hip_link")
        ET.SubElement(j, "origin", xyz=f"{cx} {cy} 0", rpy="0 0 0")
        a = jdefs["hip"]["axis"]
        ET.SubElement(j, "axis", xyz=f"{a[0]} {a[1]} {a[2]}")
        ET.SubElement(j, "limit", lower=f"{jdefs['hip']['lower']}", upper=f"{jdefs['hip']['upper']}",
                      effort=f"{jdefs['hip']['effort']}", velocity=f"{jdefs['hip']['velocity']}")

        # thigh (cylinder along -z)
        th = ET.SubElement(robot, "link", name=f"{prefix}_thigh")
        _inertial(th, leg["thigh"]["mass"], (0, 0, -th_len / 2),
                  cyl_inertia(leg["thigh"]["mass"], leg["thigh"]["radius"], th_len, "z"))
        _visual_collision(th, "cylinder",
                          {"radius": f"{leg['thigh']['radius']}", "length": f"{th_len}"},
                          f"0 0 {-th_len/2}", "0 0 0", "grey")

        j = ET.SubElement(robot, "joint", name=f"{prefix}_thigh", type="revolute")
        ET.SubElement(j, "parent", link=f"{prefix}_hip_link")
        ET.SubElement(j, "child", link=f"{prefix}_thigh")
        ET.SubElement(j, "origin", xyz=f"0 {outb*hip_len} 0", rpy="0 0 0")
        a = jdefs["thigh"]["axis"]
        ET.SubElement(j, "axis", xyz=f"{a[0]} {a[1]} {a[2]}")
        ET.SubElement(j, "limit", lower=f"{jdefs['thigh']['lower']}", upper=f"{jdefs['thigh']['upper']}",
                      effort=f"{jdefs['thigh']['effort']}", velocity=f"{jdefs['thigh']['velocity']}")

        # shank (cylinder along -z)
        sh = ET.SubElement(robot, "link", name=f"{prefix}_shank")
        _inertial(sh, leg["shank"]["mass"], (0, 0, -sh_len / 2),
                  cyl_inertia(leg["shank"]["mass"], leg["shank"]["radius"], sh_len, "z"))
        _visual_collision(sh, "cylinder",
                          {"radius": f"{leg['shank']['radius']}", "length": f"{sh_len}"},
                          f"0 0 {-sh_len/2}", "0 0 0", "grey")

        j = ET.SubElement(robot, "joint", name=f"{prefix}_knee", type="revolute")
        ET.SubElement(j, "parent", link=f"{prefix}_thigh")
        ET.SubElement(j, "child", link=f"{prefix}_shank")
        ET.SubElement(j, "origin", xyz=f"0 0 {-th_len}", rpy="0 0 0")
        a = jdefs["knee"]["axis"]
        ET.SubElement(j, "axis", xyz=f"{a[0]} {a[1]} {a[2]}")
        ET.SubElement(j, "limit", lower=f"{jdefs['knee']['lower']}", upper=f"{jdefs['knee']['upper']}",
                      effort=f"{jdefs['knee']['effort']}", velocity=f"{jdefs['knee']['velocity']}")

        # foot (sphere, fixed)
        ft = ET.SubElement(robot, "link", name=f"{prefix}_foot")
        _inertial(ft, 0.02, (0, 0, 0), sphere_inertia(0.02, leg["foot_radius"]))
        _visual_collision(ft, "sphere", {"radius": f"{leg['foot_radius']}"}, "0 0 0", "0 0 0", "grey")
        j = ET.SubElement(robot, "joint", name=f"{prefix}_foot_fixed", type="fixed")
        ET.SubElement(j, "parent", link=f"{prefix}_shank")
        ET.SubElement(j, "child", link=f"{prefix}_foot")
        ET.SubElement(j, "origin", xyz=f"0 0 {-sh_len}", rpy="0 0 0")

    return robot


# ---------------------------------------------------------------------------
# MJCF (MuJoCo) — geoms carry mass; compiler infers inertia.
# ---------------------------------------------------------------------------
def build_mjcf(p):
    leg = p["leg"]
    jdefs = p["joints"]
    sx, sy, sz = p["trunk"]["size"]
    th_len = leg["thigh"]["length"]
    sh_len = leg["shank"]["length"]
    hip_len = leg["hip_link"]["length"]
    spawn_h = round(th_len + sh_len + 0.04, 3)

    m = ET.Element("mujoco", model=p["robot_name"])
    ET.SubElement(m, "compiler", angle="radian", autolimits="true")
    ET.SubElement(m, "option", timestep="0.004", gravity="0 0 -9.81", integrator="implicitfast")

    default = ET.SubElement(m, "default")
    ET.SubElement(default, "joint", armature="0.01", damping="0.3")
    ET.SubElement(default, "geom", rgba="0.4 0.4 0.45 1", friction="1 0.1 0.1")
    pos_def = ET.SubElement(default, "default", attrib={"class": "srv"})
    ET.SubElement(pos_def, "position", kp="8", forcerange="-2.5 2.5")

    asset = ET.SubElement(m, "asset")
    ET.SubElement(asset, "texture", name="grid", type="2d", builtin="checker",
                  rgb1="0.2 0.2 0.2", rgb2="0.3 0.3 0.3", width="256", height="256")
    ET.SubElement(asset, "material", name="grid", texture="grid", texrepeat="8 8", reflectance="0.1")

    world = ET.SubElement(m, "worldbody")
    ET.SubElement(world, "light", pos="0 0 2", dir="0 0 -1", diffuse="0.8 0.8 0.8")
    ET.SubElement(world, "geom", name="floor", type="plane", size="5 5 0.1",
                  material="grid", condim="3")

    trunk = ET.SubElement(world, "body", name="trunk", pos=f"0 0 {spawn_h}")
    ET.SubElement(trunk, "freejoint", name="root")
    ET.SubElement(trunk, "geom", type="box",
                  size=f"{sx/2} {sy/2} {sz/2}", mass=f"{p['trunk']['mass']}",
                  rgba="0.2 0.4 0.8 1")

    actuators = []
    for prefix, c in p["corners"].items():
        outb = -1.0 if c["mirror"] else 1.0
        cx, cy = c["xy"]

        hipb = ET.SubElement(trunk, "body", name=f"{prefix}_hip_link", pos=f"{cx} {cy} 0")
        ET.SubElement(hipb, "joint", name=f"{prefix}_hip", axis="1 0 0",
                      range=f"{jdefs['hip']['lower']} {jdefs['hip']['upper']}")
        ET.SubElement(hipb, "geom", type="cylinder",
                      fromto=f"0 0 0 0 {outb*hip_len} 0",
                      size=f"{leg['hip_link']['radius']}", mass=f"{leg['hip_link']['mass']}")

        thb = ET.SubElement(hipb, "body", name=f"{prefix}_thigh", pos=f"0 {outb*hip_len} 0")
        ET.SubElement(thb, "joint", name=f"{prefix}_thigh", axis="0 1 0",
                      range=f"{jdefs['thigh']['lower']} {jdefs['thigh']['upper']}")
        ET.SubElement(thb, "geom", type="cylinder",
                      fromto=f"0 0 0 0 0 {-th_len}",
                      size=f"{leg['thigh']['radius']}", mass=f"{leg['thigh']['mass']}")

        shb = ET.SubElement(thb, "body", name=f"{prefix}_shank", pos=f"0 0 {-th_len}")
        ET.SubElement(shb, "joint", name=f"{prefix}_knee", axis="0 1 0",
                      range=f"{jdefs['knee']['lower']} {jdefs['knee']['upper']}")
        ET.SubElement(shb, "geom", type="cylinder",
                      fromto=f"0 0 0 0 0 {-sh_len}",
                      size=f"{leg['shank']['radius']}", mass=f"{leg['shank']['mass']}")
        ET.SubElement(shb, "geom", type="sphere", pos=f"0 0 {-sh_len}",
                      size=f"{leg['foot_radius']}", mass="0.02", rgba="0.1 0.1 0.1 1")

        for jt in p["joint_order"]:
            jn = f"{prefix}_{jt if jt != 'thigh' else 'thigh'}"
            jn = f"{prefix}_hip" if jt == "hip" else (f"{prefix}_thigh" if jt == "thigh" else f"{prefix}_knee")
            lo = jdefs[jt]["lower"]
            hi = jdefs[jt]["upper"]
            actuators.append((jn, lo, hi))

    act = ET.SubElement(m, "actuator")
    for jn, lo, hi in actuators:
        ET.SubElement(act, "position", attrib={"class": "srv"}, name=f"{jn}_act",
                      joint=jn, ctrlrange=f"{lo} {hi}")
    return m


# ---------------------------------------------------------------------------
# servo_map.json — joint <-> PCA9685 channel <-> PWM contract
# ---------------------------------------------------------------------------
def build_servo_map(p):
    sd = p["servo_defaults"]
    overrides = p.get("servo_overrides", {})
    jdefs = p["joints"]
    entries = []
    ch = 0
    for prefix in p["channel_order"]:
        for jt in p["joint_order"]:
            jn = f"{prefix}_hip" if jt == "hip" else (f"{prefix}_thigh" if jt == "thigh" else f"{prefix}_knee")
            ov = overrides.get(jt, {})
            entries.append({
                "name": jn,
                "channel": ch,
                "leg": prefix,
                "joint_type": jt,
                "lower": jdefs[jt]["lower"],
                "upper": jdefs[jt]["upper"],
                "default": p["default_pose"][jt],
                "pwm_min": sd["pwm_min"],
                "pwm_max": sd["pwm_max"],
                "angle_min": sd["angle_min"],
                "angle_max": sd["angle_max"],
                "direction": ov.get("direction", sd["direction"]),
                "zero_offset": ov.get("zero_offset", sd["zero_offset"]),
            })
            ch += 1
    return {"freq_hz": sd["freq_hz"], "n_joints": len(entries), "joints": entries}


def build_servo_config_h(smap):
    """Emit firmware/src/servo_config.h from the same servo map (single source of truth)."""
    lines = [
        "// AUTO-GENERATED by model/generate.py from params.yaml — DO NOT EDIT BY HAND.",
        "#pragma once",
        "#include <stdint.h>",
        "",
        f"#define N_JOINTS {smap['n_joints']}",
        f"#define SERVO_FREQ_HZ {smap['freq_hz']}",
        "",
        "struct ServoCal {",
        "  const char* name;",
        "  uint8_t channel;",
        "  float lower, upper, default_q;",
        "  float pwm_min, pwm_max, angle_min, angle_max;",
        "  int8_t direction;",
        "  float zero_offset;",
        "};",
        "",
        "static const ServoCal SERVOS[N_JOINTS] = {",
    ]
    def fl(x):  # emit a valid C++ float literal (e.g. 500 -> "500.0f")
        return f"{float(x)}f"
    for j in smap["joints"]:
        lines.append(
            f'  {{"{j["name"]}", {j["channel"]}, {fl(j["lower"])}, {fl(j["upper"])}, {fl(j["default"])}, '
            f'{fl(j["pwm_min"])}, {fl(j["pwm_max"])}, {fl(j["angle_min"])}, {fl(j["angle_max"])}, '
            f'{j["direction"]}, {fl(j["zero_offset"])}}},'
        )
    lines.append("};")
    return "\n".join(lines) + "\n"


def pretty(elem):
    raw = ET.tostring(elem, "utf-8")
    return minidom.parseString(raw).toprettyxml(indent="  ")


def main():
    p = load_params()
    urdf = pretty(build_urdf(p))
    mjcf = pretty(build_mjcf(p))
    smap = build_servo_map(p)

    with open(os.path.join(HERE, "robot_dog.urdf"), "w") as f:
        f.write(urdf)
    with open(os.path.join(HERE, "robot_dog.xml"), "w") as f:
        f.write(mjcf)
    with open(os.path.join(HERE, "servo_map.json"), "w") as f:
        json.dump(smap, f, indent=2)

    fw = os.path.join(HERE, "..", "firmware", "src", "servo_config.h")
    if os.path.isdir(os.path.dirname(fw)):
        with open(fw, "w") as f:
            f.write(build_servo_config_h(smap))

    print(f"wrote robot_dog.urdf ({urdf.count('<link') } links, {urdf.count('type=\"revolute\"')} revolute joints)")
    print(f"wrote robot_dog.xml  ({mjcf.count('<body')} bodies, {mjcf.count('<position')-1} actuators)")
    print(f"wrote servo_map.json ({smap['n_joints']} joints, channels 0-{smap['n_joints']-1})")
    print(f"wrote firmware/src/servo_config.h")


if __name__ == "__main__":
    main()
