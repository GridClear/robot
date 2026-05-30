"""Command protocol — the Python side of the contract implemented in
firmware/src/protocol.cpp. Build command envelopes and parse replies.

Envelope: {"t": <ms>, "id": <n>, "cmd": <name>, "args": {...}}
"""
from __future__ import annotations
import itertools
import json
import time

_ids = itertools.count(1)

# Canonical joint order (matches model/servo_map.json channel order)
JOINT_NAMES = [
    "FL_hip", "FL_thigh", "FL_knee",
    "FR_hip", "FR_thigh", "FR_knee",
    "RL_hip", "RL_thigh", "RL_knee",
    "RR_hip", "RR_thigh", "RR_knee",
]


def _envelope(cmd: str, args: dict | None = None) -> dict:
    return {"t": int(time.time() * 1000), "id": next(_ids), "cmd": cmd, "args": args or {}}


def estop() -> dict:        return _envelope("estop")
def stop() -> dict:         return _envelope("stop")
def stand() -> dict:        return _envelope("stand")
def state() -> dict:        return _envelope("state?")
def gait(name: str, speed: float = 0.5, dir=(1.0, 0.0, 0.0)) -> dict:
    return _envelope("gait", {"name": name, "speed": float(speed), "dir": [float(x) for x in dir]})
def set_joints(q) -> dict:
    q = list(q)
    assert len(q) == 12, "set_joints needs 12 angles"
    return _envelope("set_joints", {"q": [float(x) for x in q]})
def set_joint(name: str, angle: float) -> dict:
    return _envelope("set_joint", {"name": name, "angle": float(angle)})
def photo(webhook: str = "") -> dict:
    return _envelope("photo", {"webhook": webhook})


def dumps(cmd: dict) -> str:
    return json.dumps(cmd, separators=(",", ":"))
