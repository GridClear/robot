"""Video feedback for the sensorless robot — phone camera as an evaluation tool.

The robot has no IMU/encoders, so we can't feed the policy real state. Instead we
use a phone's live stream as a *human/AI-in-the-loop evaluator*: grab frames
before/during/after a gait run, save them for visual review, and quantify gross
motion by frame differencing. This judges whether the robot actually moved /
drifted / fell — it is NOT a real-time control sensor.

Phone setup (Android example): install "IP Webcam", Start server, note the URL.
  snapshot:  http://<phone-ip>:8080/shot.jpg
The laptop must be on the same network and able to reach <phone-ip>.

Usage:
  python vision_eval.py --url http://192.168.1.50:8080/shot.jpg --frames 12 --interval 0.5
  # grabs 12 frames 0.5s apart into /tmp/robotcam, prints per-step + total motion
"""
from __future__ import annotations
import argparse
import io
import os
import time

import numpy as np
import requests
from PIL import Image

OUT_DIR = "/tmp/robotcam"


def grab(url: str, timeout: float = 4.0) -> Image.Image:
    """Fetch a single JPEG snapshot from the phone stream URL."""
    r = requests.get(url, timeout=timeout)
    r.raise_for_status()
    return Image.open(io.BytesIO(r.content)).convert("RGB")


def _gray_small(img: Image.Image, w: int = 160) -> np.ndarray:
    h = int(img.height * w / img.width)
    return np.asarray(img.resize((w, h)).convert("L"), dtype=np.float32) / 255.0


def motion_score(a: Image.Image, b: Image.Image) -> float:
    """Mean absolute pixel difference (0..1) between two frames, downscaled+gray.

    ~0.00-0.01 = essentially static; higher = more gross movement in view.
    """
    ga, gb = _gray_small(a), _gray_small(b)
    n = min(ga.shape[0], gb.shape[0])
    return float(np.mean(np.abs(ga[:n] - gb[:n])))


def capture_series(url: str, frames: int, interval: float, out_dir: str = OUT_DIR):
    os.makedirs(out_dir, exist_ok=True)
    paths, imgs = [], []
    for i in range(frames):
        img = grab(url)
        p = os.path.join(out_dir, f"frame_{i:03d}.jpg")
        img.save(p, quality=85)
        paths.append(p); imgs.append(img)
        if i < frames - 1:
            time.sleep(interval)
    diffs = [motion_score(imgs[i], imgs[i + 1]) for i in range(len(imgs) - 1)]
    print(f"[vision] saved {len(paths)} frames to {out_dir}")
    for i, d in enumerate(diffs):
        print(f"  step {i:02d}->{i+1:02d}: motion={d:.4f}")
    if diffs:
        print(f"[vision] mean={np.mean(diffs):.4f} max={np.max(diffs):.4f} "
              f"first-vs-last={motion_score(imgs[0], imgs[-1]):.4f}")
    return paths


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", required=True, help="phone snapshot URL, e.g. http://IP:8080/shot.jpg")
    ap.add_argument("--frames", type=int, default=12)
    ap.add_argument("--interval", type=float, default=0.5)
    ap.add_argument("--out", default=OUT_DIR)
    a = ap.parse_args()
    capture_series(a.url, a.frames, a.interval, a.out)
