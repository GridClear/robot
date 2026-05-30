"""Canonical joint-angle <-> PCA9685 PWM conversion.

This Python implementation MUST stay numerically identical to the C version in
firmware/src/servos.cpp. Both consume model/servo_map.json. Angles in radians,
PWM in microseconds.
"""
from __future__ import annotations
import json
import os
from dataclasses import dataclass

DEFAULT_MAP = os.path.join(os.path.dirname(__file__), "..", "..", "model", "servo_map.json")


@dataclass
class ServoCal:
    name: str
    channel: int
    lower: float
    upper: float
    default: float
    pwm_min: float
    pwm_max: float
    angle_min: float
    angle_max: float
    direction: int
    zero_offset: float

    def clamp(self, q: float) -> float:
        return max(self.lower, min(self.upper, q))

    def angle_to_pwm(self, q: float) -> float:
        """Joint angle (rad) -> pulse width (us). Clamps to joint limits first."""
        q = self.clamp(q)
        servo_angle = self.direction * (q + self.zero_offset)
        frac = (servo_angle - self.angle_min) / (self.angle_max - self.angle_min)
        frac = max(0.0, min(1.0, frac))
        return self.pwm_min + (self.pwm_max - self.pwm_min) * frac

    def pwm_to_angle(self, pwm: float) -> float:
        """Inverse of angle_to_pwm (within unclamped range)."""
        frac = (pwm - self.pwm_min) / (self.pwm_max - self.pwm_min)
        servo_angle = self.angle_min + frac * (self.angle_max - self.angle_min)
        return servo_angle / self.direction - self.zero_offset


def load_servo_map(path: str = DEFAULT_MAP) -> list[ServoCal]:
    with open(path) as f:
        data = json.load(f)
    return [ServoCal(**{k: j[k] for k in ServoCal.__dataclass_fields__}) for j in data["joints"]]
