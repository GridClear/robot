#!/usr/bin/env python3
"""robotctl — command-line control for the robot dog.

Examples:
  robotctl stand
  robotctl walk --speed 0.6 --dir 1 0 0
  robotctl gait trot --speed 0.4
  robotctl set-joint FL_knee -1.2
  robotctl joints 0 0.7 -1.4 0 0.7 -1.4 0 0.7 -1.4 0 0.7 -1.4
  robotctl photo --webhook http://host:8090/capture
  robotctl state
  robotctl --host 192.168.4.1 stop
  robotctl --ble stand              # use Bluetooth LE instead of WiFi/HTTP

Transports:
  default        WiFi HTTP  (POST /cmd)
  --ws           WiFi WebSocket
  --ble          Bluetooth LE
"""
import argparse
import asyncio
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "runtime"))
import protocol as P                      # noqa: E402
from transport_wifi import WifiHttp, WifiWs  # noqa: E402


def build_cmd(a) -> dict:
    if a.action == "stand":     return P.stand()
    if a.action == "stop":      return P.stop()
    if a.action == "estop":     return P.estop()
    if a.action == "state":     return P.state()
    if a.action == "photo":     return P.photo(a.webhook)
    if a.action == "walk":      return P.gait("trot", a.speed, a.dir)
    if a.action == "gait":      return P.gait(a.name, a.speed, a.dir)
    if a.action == "set-joint": return P.set_joint(a.name, a.angle)
    if a.action == "joints":    return P.set_joints(a.values)
    raise SystemExit(f"unknown action {a.action}")


async def run_ws(a, cmd):
    async with WifiWs(a.host, a.port) as ws:
        return await ws.send(cmd)


async def run_ble(a, cmd):
    from transport_ble import BleClient
    async with BleClient() as c:
        return await c.send(cmd)


def main():
    ap = argparse.ArgumentParser(description="Robot dog CLI")
    ap.add_argument("--host", default=os.environ.get("ROBOTDOG_HOST", "192.168.4.1"))
    ap.add_argument("--port", type=int, default=int(os.environ.get("ROBOTDOG_PORT", "80")))
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--ws", action="store_true", help="use WiFi WebSocket")
    g.add_argument("--ble", action="store_true", help="use Bluetooth LE")
    sub = ap.add_subparsers(dest="action", required=True)

    for name in ("stand", "stop", "estop", "state"):
        sub.add_parser(name)
    for name in ("walk", "gait"):
        sp = sub.add_parser(name)
        if name == "gait":
            sp.add_argument("name", choices=["idle", "stand", "walk", "trot"])
        sp.add_argument("--speed", type=float, default=0.5)
        sp.add_argument("--dir", type=float, nargs=3, default=[1.0, 0.0, 0.0],
                        metavar=("VX", "VY", "WZ"))
    sj = sub.add_parser("set-joint"); sj.add_argument("name"); sj.add_argument("angle", type=float)
    jj = sub.add_parser("joints"); jj.add_argument("values", type=float, nargs=12)
    ph = sub.add_parser("photo"); ph.add_argument("--webhook", default="")

    a = ap.parse_args()
    cmd = build_cmd(a)

    try:
        if a.ble:
            reply = asyncio.run(run_ble(a, cmd))
        elif a.ws:
            reply = asyncio.run(run_ws(a, cmd))
        else:
            reply = WifiHttp(a.host, a.port).send(cmd)
    except Exception as e:
        print(f"error: {e}", file=sys.stderr)
        sys.exit(1)

    print(json.dumps(reply, indent=2) if reply else "(no reply)")


if __name__ == "__main__":
    main()
