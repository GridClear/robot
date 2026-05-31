"""Serial transport to the real robotdog8.ino firmware (USB, 115200 baud).

The active 8-DOF firmware (firmware_arduino/robotdog8/robotdog8.ino) is
line-based serial, NOT a WiFi/WebSocket JSON server. This transport speaks that
line protocol so policy_runner can stream the locomotion policy to the real
ESP32 over /dev/ttyUSB0. Mirrors the WifiWs context-manager API so run_robot can
use either backend.

Joint targets are sent with the firmware `q` command — 8 absolute servo degrees
(channel order ch0..7 = FL/FR/RL/RR abd+knee), mirror already applied host-side.
"""
from __future__ import annotations


def set8(deg_list) -> str:
    """Build the firmware batch-joint line: `q d0 d1 ... d7` (8 degrees)."""
    d = list(deg_list)
    assert len(d) == 8, "set8 needs 8 joint degrees"
    return "q " + " ".join(f"{float(x):.1f}" for x in d)


class SerialLink:
    """Open the ESP32 serial port and send newline-terminated commands.

    Use as a context manager:
        with SerialLink("/dev/ttyUSB0") as link:
            link.send_line("s")
            link.send_line(set8(degs))
    """

    def __init__(self, port="/dev/ttyUSB0", baud=115200, settle=2.0):
        self.port = port
        self.baud = baud
        self.settle = settle      # ESP32 resets on port open; wait before sending
        self.ser = None

    def __enter__(self):
        import time
        import serial
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        time.sleep(self.settle)
        self.ser.reset_input_buffer()
        return self

    def __exit__(self, *exc):
        if self.ser is not None:
            self.ser.close()
            self.ser = None
        return False

    def send_line(self, s: str) -> None:
        self.ser.write((s + "\n").encode())

    def drain(self) -> str:
        """Non-blocking read of any pending reply bytes (telemetry/acks)."""
        if self.ser is None or self.ser.in_waiting == 0:
            return ""
        return self.ser.read(self.ser.in_waiting).decode("utf-8", "replace")
