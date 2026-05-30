# Components & Wiring — 8-servo Quadruped (ESP32 + PCA9685)

Physical hardware reference for the real robot: an **ESP32** drives **8 servos**
through a **PCA9685** 16-channel PWM board over I²C. The ESP32 connects to the
**DGX (GB10)** by USB (`/dev/ttyUSB0`) and is controlled over serial. Firmware:
[`firmware_arduino/robotdog8/robotdog8.ino`](firmware_arduino/robotdog8/robotdog8.ino).

> This robot is **8-DOF** (4 legs × 2 joints: hip-abduction + knee). It is a
> simpler build than the 12-DOF model used for the URDF/MJCF + Isaac Lab policy —
> hardware walking uses the **scripted gait** in the sketch, not the RL policy.

## Bill of materials

| Qty | Part | Notes |
|----:|------|-------|
| 1 | ESP32 dev board (CP210x USB-UART) | shows up as `/dev/ttyUSB0` on the DGX |
| 1 | PCA9685 16-ch PWM driver | I²C addr `0x40` (default) |
| 8 | 180° servos | calibrated to PCA9685 ticks 125–575 (per your test sketch) |
| 1 | 5–6 V BEC / UBEC ≥ 5 A | **dedicated servo power** (not from the ESP32) |
| 1 | battery + switch + fuse | servo supply |
| — | 470–1000 µF cap across PCA9685 V+/GND | absorbs servo inrush |

## Wiring

**ESP32 ↔ PCA9685 (I²C + logic power):**

| ESP32 | PCA9685 | purpose |
|-------|---------|---------|
| GPIO 21 | SDA | I²C data |
| GPIO 22 | SCL | I²C clock |
| 3V3 | VCC | logic power |
| GND | GND | common ground |

**Servo power (critical — do NOT power servos from the ESP32):**

```
 battery ──switch──fuse──► BEC 5-6V ──► PCA9685 V+ (screw terminal) ──► all 8 servo V+
                                          └─ 470-1000µF cap across V+/GND
 COMMON GROUND: battery GND ─ BEC GND ─ PCA9685 GND ─ ESP32 GND  (single star ground)
```

- 8 servos can momentarily pull several amps; size the BEC accordingly and fit the
  bulk cap, or you'll see brown-outs / jitter / ESP32 resets.
- PCA9685 **VCC** (logic) = ESP32 3V3; PCA9685 **V+** (servo rail) = BEC. Keep separate.

## Channel → joint → leg map (wire each servo to its channel)

Leg order **FL, FR, RL, RR**; each leg = abduction then knee.

| Channel | Leg | Joint | | Channel | Leg | Joint |
|--------:|-----|-------|---|--------:|-----|-------|
| **0** | Front-Left  | abduction | | **4** | Rear-Left  | abduction |
| **1** | Front-Left  | knee      | | **5** | Rear-Left  | knee      |
| **2** | Front-Right | abduction | | **6** | Rear-Right | abduction |
| **3** | Front-Right | knee      | | **7** | Rear-Right | knee      |
| 8–15 | — | spare | | | | |

- **abduction** = hip joint that swings the leg sideways (in/out from the body).
- **knee** = lower-leg joint that extends/tucks the foot.

## Servo calibration

- 180° servos, **PCA9685 ticks 125 → 575** map to **0° → 180°** at 50 Hz
  (`SERVOMIN=125`, `SERVOMAX=575` — from your validated test). 125 ticks ≈ 610 µs,
  575 ticks ≈ 2810 µs.
- Per-channel in firmware (live-tunable over serial, no reflash):
  `center` (default **90°**), `lo`/`hi` clamp (default **15–165°**), `dir` (±1 to
  fix a reversed-mounted horn). Set with `ctr <ch> <deg>`, `lim <ch> <lo> <hi>`,
  `dir <ch> <±1>`; view with `dump`.

## Connect to the DGX & control

The ESP32 is on the DGX at **`/dev/ttyUSB0`** (115200 baud). Flash + drive it with
arduino-cli on the DGX:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 robotdog8
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/ttyUSB0 robotdog8
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200   # or a pyserial helper
```

### Serial command reference

| cmd | action |
|-----|--------|
| `c` | center all servos (safe pose), stop gait |
| `s` | stand pose (= centers) |
| `r` | relax — cut PWM to all servos (limp) |
| `x` | stop gait, hold current pose |
| `w` | start scripted trot gait |
| `t <ch> <deg>` | drive one channel to a raw angle 0–180 (also `j`) |
| `amp <deg>` / `kamp <deg>` | abduction / knee swing amplitude |
| `freq <hz>` | gait frequency |
| `phase <deg>` | abduction→knee phase offset |
| `dir <ch> <±1>` / `ctr <ch> <deg>` / `lim <ch> <lo> <hi>` | per-servo calibration |
| `slew <deg/s>` | max slew rate |
| `dump` | print calibration + gait params + current angles |

## Gait

A **2-DOF trot**: diagonal pairs (FL+RR) and (FR+RL) move in antiphase. Per leg the
abduction and knee servos follow phase-offset sinusoids (`amp`, `kamp`, `phase`,
`freq`) so the gait can be tuned empirically to produce forward motion — abduction
lifts/places the leg while the knee provides reach/propulsion. Start low
(`amp 12`, `kamp 18`, `freq 1.0`) and increase once motion looks coordinated.

## Safety / bring-up order

1. **Prop the robot up** (feet off the ground) and keep a hand near the servo-power
   switch for the first power-on — servos snap to center on boot.
2. `dump` → `c` (center) → confirm nothing binds.
3. `t 0 70` / `t 0 110` … step through **each** channel 0–7 to confirm the right
   joint moves the right way; flip `dir`/`ctr`/`lim` as needed.
4. Only then `s` → `w` at low amplitude. Use `x` (hold) or `r` (relax) to stop.
