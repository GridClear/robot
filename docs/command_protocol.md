# Command Protocol

One JSON schema, three transports (WiFi HTTP, WiFi WebSocket, BLE). Implemented
identically in `firmware/src/protocol.cpp` and `control/runtime/protocol.py`.

## Envelope

Every command:
```json
{ "t": 1717000000000, "id": 42, "cmd": "<name>", "args": { ... } }
```
- `t` — client ms timestamp (used by the firmware command watchdog).
- `id` — monotonic id for ack correlation.
- `cmd` / `args` — see below.

Every reply (telemetry/ack):
```json
{ "id": 42, "ok": true, "mode": "gait:trot", "q": [12 floats], "batt": 7.4, "err": null }
```
- `mode` — current controller mode (`stand`, `hold`, `joints`, `gait:<name>`, `estop`, `hold(wd)`).
- `q` — 12 current joint angles (radians), in canonical order.
- `err` — null, or `"estop"` while latched.

## Commands

| cmd | args | effect |
|-----|------|--------|
| `estop` | `{}` | **latching**: cut servo output / go limp. Ignores motion until `stand`. |
| `stop` | `{}` | freeze at current pose (hold). |
| `stand` | `{}` | go to nominal standing pose; also clears a latched `estop`. |
| `gait` | `{"name":"idle\|stand\|walk\|trot", "speed":0..1, "dir":[vx,vy,wz]}` | select onboard gait; `dir` is the velocity command. |
| `set_joints` | `{"q":[12 floats radians]}` | direct joint targets (the policy runner uses this @ 50 Hz). |
| `set_joint` | `{"name":"FL_knee", "angle":-1.2}` | single joint (calibration/sliders). |
| `photo` | `{"webhook":"http://host/capture"}` | fire the camera-capture signal (firmware-side). |
| `state?` | `{}` | request a telemetry reply. |

## Canonical joint order (index 0–11 = PCA9685 channel)

```
FL_hip FL_thigh FL_knee  FR_hip FR_thigh FR_knee
RL_hip RL_thigh RL_knee  RR_hip RR_thigh RR_knee
```
`hip` = abduction (axis x), `thigh` = hip flexion (axis y), `knee` (axis y).

## Transport bindings

- **HTTP** — `POST /cmd` with the envelope as the body; `GET /state`. Best for one-shot CLI commands.
- **WebSocket** `/ws` — send the envelope as a text frame, receive the reply frame. Use for high-rate streaming (50 Hz `set_joints`).
- **BLE** — service `6e400001-…`; write the envelope to char `6e400002-…` (CMD), subscribe to `6e400003-…` (STATE) for notifications. Reserved for low-rate, high-value commands (`stop/estop/stand/gait/photo`) — not 50 Hz streaming.

## Safety (enforced in firmware regardless of transport)

- **Watchdog**: while a gait is active, no command for `WATCHDOG_MS` (500 ms) → auto hold.
- **Clamping**: every joint target is clamped to its limit before it reaches a servo.
- **E-STOP latch**: `estop` holds until an explicit `stand`.
