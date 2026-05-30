"""Camera trigger — send a simple signal to an external capture service.

The robot does NOT control a camera directly. It fires a lightweight HTTP request
to a separate service (which owns the camera / DJI drone / phone) telling it to
capture a photo on the robot's behalf. Two paths:

  1. Host-side (this module): the controller calls trigger() directly.
  2. Onboard: the ESP32 fires the same webhook itself when it gets a `photo`
     command (firmware firePhoto()) — useful when no host is in the loop.

Configure CAPTURE_WEBHOOK to your service's endpoint.
"""
from __future__ import annotations
import requests

CAPTURE_WEBHOOK = "http://192.168.4.50:8090/capture"   # <-- set to your capture service


def trigger(webhook: str = CAPTURE_WEBHOOK, meta: dict | None = None, timeout: float = 2.0) -> bool:
    """Fire the capture signal. Returns True on HTTP 2xx. Fire-and-forget friendly."""
    try:
        r = requests.post(webhook, json=meta or {"action": "capture"}, timeout=timeout)
        return r.ok
    except requests.RequestException:
        return False
