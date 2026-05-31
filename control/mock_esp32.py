#!/usr/bin/env python3
"""Mock ESP32 robot server — speaks the SAME protocol as the firmware so the CLI,
WebSocket streaming, and web app can be developed and tested without hardware.

Implements: POST /cmd, GET /state, WS /ws. Holds 12 joint angles + mode in memory,
applies the same clamping as servo_math, and echoes a telemetry reply.

Run:  python3 control/mock_esp32.py            # listens on 0.0.0.0:8080
Then: robotctl --host 127.0.0.1 --port 8080 stand
"""
import json
import os
import sys

from aiohttp import web

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "runtime"))
from servo_math import load_servo_map  # noqa: E402

CALS = load_servo_map()
STATE = {
    "q": [c.default for c in CALS],
    "mode": "stand",
    "estop": False,
}


def apply(cmd: dict) -> dict:
    args = cmd.get("args", {})
    c = cmd.get("cmd")
    if c == "estop":
        STATE["estop"] = True; STATE["mode"] = "estop"
    elif c == "stop":
        STATE["mode"] = "hold"
    elif c == "stand":
        STATE["estop"] = False; STATE["mode"] = "stand"
        STATE["q"] = [cal.default for cal in CALS]
    elif c == "gait":
        if not STATE["estop"]:
            STATE["mode"] = f"gait:{args.get('name', 'idle')}"
    elif c == "set_joints" and not STATE["estop"]:
        q = args.get("q", [])
        if len(q) == 8:
            STATE["q"] = [cal.clamp(v) for cal, v in zip(CALS, q)]
            STATE["mode"] = "joints"
    elif c == "set_joint" and not STATE["estop"]:
        name = args.get("name")
        for i, cal in enumerate(CALS):
            if cal.name == name:
                STATE["q"][i] = cal.clamp(args.get("angle", 0.0))
                STATE["mode"] = "joint"
    elif c == "photo":
        STATE["mode"] = STATE["mode"]  # no-op; real firmware fires the webhook
    return reply(cmd.get("id", 0), c == "photo" or True)


def reply(cid: int, ok: bool) -> dict:
    return {"id": cid, "ok": ok, "mode": STATE["mode"], "q": [round(x, 4) for x in STATE["q"]],
            "batt": 7.4, "err": "estop" if STATE["estop"] else None}


async def h_cmd(req):
    try:
        cmd = await req.json()
    except Exception:
        return web.json_response({"ok": False, "err": "parse"}, status=400)
    return web.json_response(apply(cmd))


async def h_state(req):
    return web.json_response(reply(0, True))


async def h_ws(req):
    ws = web.WebSocketResponse()
    await ws.prepare(req)
    async for msg in ws:
        if msg.type == web.WSMsgType.TEXT:
            try:
                await ws.send_str(json.dumps(apply(json.loads(msg.data))))
            except Exception:
                await ws.send_str(json.dumps({"ok": False, "err": "parse"}))
    return ws


def make_app():
    app = web.Application()
    app.add_routes([web.post("/cmd", h_cmd), web.get("/state", h_state),
                    web.get("/ws", h_ws),
                    web.get("/", lambda r: web.Response(text="mock robotdog"))])
    return app


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    print(f"mock ESP32 robotdog on http://0.0.0.0:{port}  (POST /cmd, GET /state, WS /ws)")
    web.run_app(make_app(), host="0.0.0.0", port=port, print=None)
