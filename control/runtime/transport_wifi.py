"""WiFi transports to the ESP32: HTTP (request/reply) and WebSocket (streaming).

HTTP is synchronous and good for one-shot CLI commands.
WebSocket is for high-rate streaming (e.g. policy_runner pushing set_joints @ 50 Hz).
"""
from __future__ import annotations
import json

import requests  # HTTP client


class WifiHttp:
    def __init__(self, host: str, port: int = 80, timeout: float = 2.0):
        self.base = f"http://{host}:{port}"
        self.timeout = timeout

    def send(self, cmd: dict) -> dict:
        r = requests.post(f"{self.base}/cmd", data=json.dumps(cmd),
                          headers={"Content-Type": "application/json"}, timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def state(self) -> dict:
        r = requests.get(f"{self.base}/state", timeout=self.timeout)
        r.raise_for_status()
        return r.json()


class WifiWs:
    """Async WebSocket client. Use for streaming control loops."""
    def __init__(self, host: str, port: int = 80):
        self.url = f"ws://{host}:{port}/ws"
        self._ws = None

    async def __aenter__(self):
        import websockets
        self._ws = await websockets.connect(self.url, max_queue=8)
        return self

    async def __aexit__(self, *exc):
        if self._ws:
            await self._ws.close()

    async def send(self, cmd: dict) -> dict:
        await self._ws.send(json.dumps(cmd, separators=(",", ":")))
        return json.loads(await self._ws.recv())

    async def send_nowait(self, cmd: dict):
        """Fire a command without awaiting the reply (for high-rate streaming)."""
        await self._ws.send(json.dumps(cmd, separators=(",", ":")))
