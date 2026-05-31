#!/usr/bin/env python3
"""Validate the joint<->PWM servo map: round-trip accuracy + limit clamping.

Run:  python3 sim/test_servo_map.py     (exit 0 = all pass)
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "control", "runtime"))
from servo_math import load_servo_map  # noqa: E402

TOL = 1e-6


def main() -> int:
    cals = load_servo_map()
    assert len(cals) == 8, f"expected 8 joints, got {len(cals)}"
    channels = sorted(c.channel for c in cals)
    assert channels == list(range(8)), f"channels must be 0..7, got {channels}"

    failures = 0
    for c in cals:
        # 1) round-trip across the joint range
        for k in range(11):
            q = c.lower + (c.upper - c.lower) * k / 10.0
            pwm = c.angle_to_pwm(q)
            q2 = c.pwm_to_angle(pwm)
            if abs(q2 - q) > TOL:
                print(f"  FAIL {c.name}: round-trip q={q:.4f} -> pwm={pwm:.1f} -> q={q2:.4f}")
                failures += 1
            if not (c.pwm_min - TOL <= pwm <= c.pwm_max + TOL):
                print(f"  FAIL {c.name}: pwm {pwm:.1f} out of [{c.pwm_min},{c.pwm_max}]")
                failures += 1

        # 2) clamping: commands beyond limits must saturate, never exceed pulse band
        for q_bad in (c.lower - 5.0, c.upper + 5.0):
            pwm = c.angle_to_pwm(q_bad)
            assert c.pwm_min - TOL <= pwm <= c.pwm_max + TOL, \
                f"{c.name}: clamp leaked pwm={pwm}"

    if failures:
        print(f"\n{failures} FAILURES")
        return 1
    print(f"OK: 8 joints, round-trip < {TOL}, clamping holds, channels 0-7 unique")
    return 0


if __name__ == "__main__":
    sys.exit(main())
