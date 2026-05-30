# Robot Dog — Wiring & Bring-up Guide

12 servos (4 legs × 3 joints) driven by a **PCA9685** 16-channel PWM board over
I²C from an **ESP32**. The ESP32 never powers servos directly — it only speaks
I²C to the PCA9685 and runs WiFi + BLE control.

## Bill of materials

| Qty | Part | Notes |
|----:|------|-------|
| 1 | ESP32 dev board (e.g. ESP32-DevKitC / WROOM-32) | 3.3 V logic |
| 1 | PCA9685 16-ch 12-bit PWM driver | I²C addr `0x40` (default) |
| 12 | Servos | hobby digital servos; pick torque for your link masses |
| 1 | 5–6 V BEC / UBEC, ≥ 6 A (≥ 10 A recommended) | dedicated servo supply |
| 1 | 2S LiPo (7.4 V) or equiv. + power switch + fuse | robot battery |
| — | 470–1000 µF capacitor across PCA9685 V+ / GND | absorbs servo inrush |
| 1 | IMU (MPU-6050 / BNO055) — **for the walking policy** | see Sim-to-real note |

## Power topology (critical — do NOT power servos from the ESP32)

```
 2S LiPo ──┬── switch ── fuse ──┬── BEC 5-6V ──► PCA9685  V+   (servo power rail)
           │                    │                └─► all 12 servo V+ (via PCA headers)
           └────────────────────┴── ESP32 5V/VIN (or separate BEC)
 COMMON GROUND: LiPo GND ── BEC GND ── PCA9685 GND ── ESP32 GND   (single star ground)
```

- **Servo current** is the #1 gotcha: 12 servos can pull > 6 A under load/stall.
  Size the BEC accordingly and add the bulk cap across the PCA9685 V+/GND screw
  terminals.
- The PCA9685 logic (VCC) runs from ESP32 **3.3 V**; the servo rail (V+) runs
  from the **BEC**. These are separate — never bridge them.

## ESP32 ↔ PCA9685 (I²C)  — pins set in `src/config.h`

| ESP32 pin | PCA9685 | Purpose |
|-----------|---------|---------|
| GPIO 21 (`I2C_SDA`) | SDA | I²C data |
| GPIO 22 (`I2C_SCL`) | SCL | I²C clock |
| 3.3 V | VCC | logic power |
| GND | GND | common ground |
| (opt) GPIO `PCA9685_OE_PIN` | OE | active-low output enable → hardware E-STOP. Tie to GND if unused. |

## Servo → leg → PCA9685 channel map

Channels are assigned **leg-by-leg** (`FL, FR, RL, RR`), joints in order
**hip (abduction), thigh (hip-flexion), knee**. This table is the contract and is
auto-generated into `src/servo_config.h` from `model/params.yaml`:

| Channel | Joint | Leg | Joint type |
|--------:|-------|-----|-----------|
| 0 | FL_hip   | Front-Left  | hip abduction |
| 1 | FL_thigh | Front-Left  | hip flexion |
| 2 | FL_knee  | Front-Left  | knee |
| 3 | FR_hip   | Front-Right | hip abduction |
| 4 | FR_thigh | Front-Right | hip flexion |
| 5 | FR_knee  | Front-Right | knee |
| 6 | RL_hip   | Rear-Left   | hip abduction |
| 7 | RL_thigh | Rear-Left   | hip flexion |
| 8 | RL_knee  | Rear-Left   | knee |
| 9 | RR_hip   | Rear-Right  | hip abduction |
| 10 | RR_thigh | Rear-Right  | hip flexion |
| 11 | RR_knee  | Rear-Right  | knee |
| 12–15 | — | spare | — |

## Servo calibration (do this on a bench BEFORE assembly)

Each joint's pulse band and zero are defined in `model/params.yaml`
(`servo_defaults` + `servo_overrides`). Defaults assume a ~270° digital servo
(500–2500 µs). The **knee** uses `zero_offset: 1.4` so its working band
`[-2.6, -0.2] rad` maps into the servo's centered travel — confirm your knee
geometry/horn matches, or it will hit a mechanical stop.

Procedure per joint:
1. Disconnect the leg linkage (servo free).
2. Send `set_joint {name, angle:0}` → servo should sit at its mechanical mid.
3. Command `lower` and `upper` limits → verify no buzzing/stall at the ends.
4. If a joint moves the wrong way, flip `direction` for that joint type in
   `servo_overrides`, re-run `python model/generate.py`, re-flash.
5. `python3 sim/test_servo_map.py` must stay green after any change.

## Flash & first run

```bash
cd firmware
pio run                # build
pio run -t upload      # flash (ESP32 on USB)
pio device monitor     # 115200 baud
```
On boot the robot joins `WIFI_SSID`; if that fails within 8 s it raises its own
AP `robotdog` / `walkies123`. It also advertises BLE as `robotdog`. See
`docs/command_protocol.md` and `control/` for how to send commands.

## Safety

- **Watchdog**: if no command arrives for `WATCHDOG_MS` (500 ms) while walking,
  the robot freezes to a hold pose.
- **E-STOP**: `estop` latches outputs to limp (if `/OE` wired) and ignores motion
  until an explicit `stand`.
- First power-on: prop the body up so feet are off the ground, and keep a hand on
  the battery switch.
