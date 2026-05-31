"""Distance-triggered photo capture for gaussian-splatting scan runs.

The robot has no IMU/encoders (see vision_eval.py / policy_runner.py), so we cannot
*measure* how far it has travelled. Instead the DGX dead-reckons distance from the
gait velocity it is already commanding (velocity x dt), and every ~1 cm of estimated
travel it pulls a still from the phone mounted on the robot and stores it on the DGX.

Phone setup (same as vision_eval.py): install Android "IP Webcam", Start server, use
the still URL, e.g.  http://<phone-ip>:8080/photo.jpg  (full-res, autofocus). The DGX
must be on the same WiFi LAN as the phone (and the ESP32).

Photos land under  <root>/run_N/NNNN.jpg  with a per-run manifest.json. `root` defaults
to $GS_PHOTOS_ROOT or ~/robotdog/gaussian_splatting_photos (on the DGX).

Two ways to drive it:
  1. From policy_runner.py --capture (the primary scan driver): it calls
     DistanceTrigger.update(...) every control tick with commanded velocity x dt.
  2. Standalone, timing-based, for a manual gait you started with robotctl:
       python capture_coordinator.py --phone-url http://IP:8080/photo.jpg \
         --speed 0.5 --max-speed-mps 0.08 --step-m 0.01 --duration 60
"""
from __future__ import annotations
import argparse
import json
import os
import queue
import re
import threading
import time

import requests

DEFAULT_ROOT = os.environ.get(
    "GS_PHOTOS_ROOT", os.path.expanduser("~/robotdog/gaussian_splatting_photos"))
DEFAULT_STEP_M = 0.01            # capture every ~1 cm of estimated travel
DEFAULT_MAX_SPEED_MPS = 0.08     # m/s at commanded vx=1.0 — CALIBRATE (see module docs)
# Phone IP Webcam snapshot URL. On the POCO M2 Pro hotspot the phone is the gateway at
# 10.197.152.200. /shot.jpg = fast video frame (keeps up with the ~1 cm cadence); switch
# to /photo.jpg for full-res stills (slower — raise --step-m to match).
DEFAULT_PHONE_URL = os.environ.get("PHONE_URL", "http://10.197.152.200:8080/shot.jpg")


def fetch_bytes(url: str, timeout: float = 4.0) -> bytes:
    """Fetch the raw JPEG bytes of a phone snapshot (IP Webcam /photo.jpg).

    We keep the original bytes rather than re-encoding (cf. vision_eval.grab(), which
    decodes to a PIL image for motion analysis) so the photogrammetry dataset stays at
    full fidelity and preserves any EXIF the phone embeds.
    """
    r = requests.get(url, timeout=timeout)
    r.raise_for_status()
    return r.content


class DistanceTrigger:
    """Accumulate estimated travel; fire `on_step` once per `step_m` crossed.

    Cheap and allocation-free — safe to call from a 50 Hz control loop.
    """

    def __init__(self, step_m: float, on_step):
        self.step_m = float(step_m)
        self.on_step = on_step
        self.acc = 0.0      # distance since last trigger
        self.total = 0.0    # distance since start

    def update(self, d_meters: float) -> None:
        if d_meters <= 0.0:
            return
        self.acc += d_meters
        self.total += d_meters
        while self.acc >= self.step_m:
            self.acc -= self.step_m
            self.on_step(self.total)


def _next_run_dir(root: str) -> str:
    """Return <root>/run_{N} where N is one past the highest existing run."""
    os.makedirs(root, exist_ok=True)
    n = 0
    for name in os.listdir(root):
        m = re.fullmatch(r"run_(\d+)", name)
        if m and os.path.isdir(os.path.join(root, name)):
            n = max(n, int(m.group(1)))
    run_dir = os.path.join(root, f"run_{n + 1}")
    os.makedirs(run_dir)
    return run_dir


class CaptureRun:
    """A single scan run: owns run_N/, a background fetch worker, and the manifest.

    submit() is non-blocking (enqueue only) so it never stalls the control loop; a
    worker thread does the HTTP fetch + disk write. Call close() to drain and flush.
    """

    def __init__(self, phone_url: str, root: str = DEFAULT_ROOT, run_meta: dict | None = None,
                 timeout: float = 4.0):
        self.phone_url = phone_url
        self.timeout = timeout
        self.run_dir = _next_run_dir(root)
        self._seq = 0
        self._manifest: list[dict] = []
        self._q: queue.Queue = queue.Queue()
        self._lock = threading.Lock()
        meta = {"phone_url": phone_url, "step_m": None, "max_speed_mps": None,
                "started_iso": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "started_unix": time.time()}
        meta.update(run_meta or {})
        self._write_json("run_meta.json", meta)
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()
        print(f"[capture] run dir: {self.run_dir}")

    def submit(self, est_distance_m: float, velocity_cmd=None) -> None:
        """Queue a capture. Seq is assigned here so ordering is deterministic."""
        seq = self._seq
        self._seq += 1
        self._q.put({"seq": seq, "t_unix": time.time(),
                     "t_iso": time.strftime("%Y-%m-%dT%H:%M:%S"),
                     "est_distance_m": round(float(est_distance_m), 4),
                     "velocity_cmd": list(velocity_cmd) if velocity_cmd is not None else None})

    def _run(self) -> None:
        while True:
            job = self._q.get()
            if job is None:
                break
            fname = f"{job['seq']:04d}.jpg"
            entry = {"seq": job["seq"], "file": fname, "t_unix": round(job["t_unix"], 3),
                     "t_iso": job["t_iso"], "est_distance_m": job["est_distance_m"],
                     "velocity_cmd": job["velocity_cmd"], "ok": True, "err": None}
            try:
                data = fetch_bytes(self.phone_url, self.timeout)
                with open(os.path.join(self.run_dir, fname), "wb") as f:
                    f.write(data)
                entry["bytes"] = len(data)
            except Exception as e:               # don't let a dropped frame kill the run
                entry["ok"] = False
                entry["err"] = str(e)
                print(f"[capture] seq {job['seq']} FAILED: {e}")
            with self._lock:
                self._manifest.append(entry)
            self._flush_manifest()

    def _flush_manifest(self) -> None:
        with self._lock:
            self._write_json("manifest.json", self._manifest)

    def _write_json(self, name: str, obj) -> None:
        path = os.path.join(self.run_dir, name)
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(obj, f, indent=2)
        os.replace(tmp, path)                     # atomic; manifest is always valid

    def close(self) -> None:
        """Drain queued captures, stop the worker, write the final manifest."""
        self._q.put(None)
        self._worker.join()
        self._flush_manifest()
        ok = sum(1 for e in self._manifest if e["ok"])
        print(f"[capture] done: {ok}/{len(self._manifest)} photos saved to {self.run_dir}")


def run_standalone(phone_url: str, speed: float, max_speed_mps: float, step_m: float,
                   duration: float, root: str, tick: float = 0.05) -> str:
    """Timing-based capture for a manual gait already running (no policy_runner).

    Assumes the robot is walking at a steady commanded `speed`; integrates
    speed*max_speed_mps over wall-clock and captures every `step_m`.
    """
    speed_mps = abs(speed) * max_speed_mps
    run = CaptureRun(phone_url, root, run_meta={
        "step_m": step_m, "max_speed_mps": max_speed_mps, "mode": "standalone",
        "commanded_speed": speed, "velocity_cmd": [speed, 0.0, 0.0]})
    trig = DistanceTrigger(step_m, lambda total: run.submit(total, [speed, 0.0, 0.0]))
    t0 = time.time()
    try:
        while time.time() - t0 < duration:
            time.sleep(tick)
            trig.update(speed_mps * tick)
    except KeyboardInterrupt:
        print("[capture] interrupted")
    finally:
        run.close()
    print(f"[capture] est. travel {trig.total*100:.1f} cm over {time.time()-t0:.1f}s")
    return run.run_dir


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--phone-url", default=DEFAULT_PHONE_URL,
                    help="phone still URL (default: %(default)s; or set $PHONE_URL)")
    ap.add_argument("--speed", type=float, default=0.5, help="commanded gait speed (vx, 0..1)")
    ap.add_argument("--max-speed-mps", type=float, default=DEFAULT_MAX_SPEED_MPS,
                    help="m/s at vx=1.0 (calibrate!)")
    ap.add_argument("--step-m", type=float, default=DEFAULT_STEP_M,
                    help="capture interval in meters (default 0.01 = 1 cm)")
    ap.add_argument("--duration", type=float, default=60.0, help="seconds to run")
    ap.add_argument("--root", default=DEFAULT_ROOT)
    a = ap.parse_args()
    run_standalone(a.phone_url, a.speed, a.max_speed_mps, a.step_m, a.duration, a.root)
