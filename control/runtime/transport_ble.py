"""BLE transport to the ESP32 using bleak. Reserved for low-rate, high-value
commands (stop/estop/stand/gait/photo) — not 50 Hz joint streaming.

UUIDs must match firmware/src/config.h.
"""
from __future__ import annotations
import asyncio
import json

BLE_DEVICE_NAME = "robotdog"
SVC_UUID   = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
CMD_UUID   = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
STATE_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"


class BleClient:
    def __init__(self, name: str = BLE_DEVICE_NAME):
        self.name = name
        self._client = None
        self._last_state = None

    async def __aenter__(self):
        from bleak import BleakClient, BleakScanner
        dev = await BleakScanner.find_device_by_name(self.name, timeout=10.0)
        if dev is None:
            raise RuntimeError(f"BLE device '{self.name}' not found")
        self._client = BleakClient(dev)
        await self._client.connect()

        def _on_state(_handle, data: bytearray):
            try:
                self._last_state = json.loads(data.decode())
            except Exception:
                pass
        await self._client.start_notify(STATE_UUID, _on_state)
        return self

    async def __aexit__(self, *exc):
        if self._client and self._client.is_connected:
            await self._client.disconnect()

    async def send(self, cmd: dict) -> dict | None:
        payload = json.dumps(cmd, separators=(",", ":")).encode()
        await self._client.write_gatt_char(CMD_UUID, payload, response=True)
        await asyncio.sleep(0.05)   # let a notify land
        return self._last_state
