# Architecture

```
                         ┌─────────────────────────────────────────────┐
                         │            model/ (source of truth)          │
                         │  params.yaml ──generate.py──┬─ robot_dog.urdf │
                         │                              ├─ robot_dog.xml  │
                         │                              ├─ servo_map.json │
                         │                              └─ firmware/src/  │
                         │                                  servo_config.h│
                         └───────┬──────────────┬───────────────┬────────┘
                                 │              │               │
                  ┌──────────────▼──┐   ┌───────▼───────┐  ┌────▼───────────────┐
                  │ training/        │   │ sim/          │  │ firmware/ (ESP32)  │
                  │  isaaclab_task   │   │  view_mujoco  │  │  servos.cpp (PCA)  │
                  │  mujoco_fallback │   │  walk_demo    │  │  net_wifi / net_ble│
                  │  export_policy ──┼─► │  test_servo   │  │  protocol.cpp      │
                  └──────┬───────────┘   └───────────────┘  └────▲───────────────┘
                         │ policy.onnx + meta                    │ set_joints / cmds
                  ┌──────▼───────────────────────────────┐       │ (WiFi WS/HTTP, BLE)
                  │ control/                              │       │
                  │  runtime/policy_runner ───────────────┼───────┘
                  │  cli/robotctl  runtime/transport_*    │
                  │  runtime/camera_trigger ──► capture service (HTTP webhook)
                  │  mock_esp32 (hardware-free testing)   │
                  └───────────────────────────────────────┘
```

## Key decisions

- **One source of truth.** All geometry, joint limits, and servo calibration live
  in `model/params.yaml`. `generate.py` emits the URDF (Isaac), MJCF (MuJoCo),
  `servo_map.json` (runtime), and the firmware's `servo_config.h`. Change params,
  regenerate, and every consumer stays in sync. `sim/test_servo_map.py` guards the
  angle↔PWM contract.
- **One leg, replicated ×4.** `generate.py` defines a single leg and instantiates it
  for FL/FR/RL/RR with corner offsets + L/R mirroring. The "configure one leg, copy
  to the rest" requirement is structural — there is exactly one leg definition.
- **One protocol, three transports.** `docs/command_protocol.md` is implemented once
  in C++ (firmware) and once in Python (control), spoken over WiFi HTTP, WiFi
  WebSocket, and BLE.
- **Policy off the ESP32.** The NN runs on a companion computer or host; the ESP32
  executes joint targets and owns safety (clamp, slew, watchdog, E-STOP). See
  `docs/sim2real.md`.
- **Two training paths, interchangeable artifacts.** Isaac Lab on the GB10 (primary,
  GPU-parallel) and MuJoCo PPO (fallback, runs anywhere) share the obs/action layout,
  so a `policy.onnx` from either drops into `policy_runner.py`.

## Hardware (real robot)

ESP32 ─ I²C ─ PCA9685 ─ 12× servo. Dedicated 5–6 V BEC for servos (not the ESP32
rail), common star ground, bulk cap across the servo rail. Companion computer +
IMU for autonomous walking. Full wiring: `firmware/README_WIRING.md`.

## What the Fusion file is

`robot/RobotDogUpdt.f3d` is an Autodesk Fusion 360 archive — proprietary binary
B-Rep geometry, not parseable on Linux and not editable as text. The model here was
re-derived parametrically; dimensions in `params.yaml` are placeholders to replace
with measured values during bring-up.
