# Sim-to-Real: where the policy runs and how joint targets reach the servos

## The policy does NOT run on the ESP32

A 48→12 MLP locomotion policy at 50 Hz alongside WiFi + BLE + I²C is the wrong job
for the ESP32. Instead the ESP32 is a **pure servo executor + safety layer**: it
receives 12 joint angles (`set_joints`) plus discrete commands, clamps them, slews
to them, and runs the watchdog/E-STOP. It never sees observations or network weights.

```
 [policy host] ── set_joints @50Hz (WiFi WS) ──► [ESP32] ── I²C ──► [PCA9685] ──► 12 servos
   onnxruntime                                     clamp + slew + watchdog
   + IMU read
```

## Two deployment modes (same firmware)

1. **Companion compute on the robot (recommended for real walking).** A Raspberry
   Pi 5 / Jetson Orin Nano runs `control/runtime/policy_runner.py`: reads an **IMU**
   (base linear/angular velocity + gravity direction), builds the observation,
   runs ONNX inference at 50 Hz, and streams `set_joints` to the ESP32 over the
   WiFi WebSocket (or USB-serial). **An IMU is required** — the locomotion
   observations need base velocity + gravity; without it the policy has no
   proprioceptive base state.
2. **Host-in-the-loop (bring-up / demos).** A laptop or the GB10 runs the policy
   runner and streams to the ESP32. Simpler, but WiFi jitter degrades a 50 Hz
   loop — fine for standing/scripted gaits and validation, not robust field walking.

During bring-up you don't need a policy at all: the firmware has a built-in trot
gait (`gait trot`), and `sim/walk_demo.py` shows that same CPG walking the model.

## The obs/action contract (must match exactly)

`training/export_policy.py` writes `policy/policy_meta.json` with the **exact**
observation order, `action_scale`, and `default_joint_pos`. Both the Isaac Lab env
and the MuJoCo env produce this layout, and `policy_runner.py` reconstructs it.
Never hand-transcribe the order — drift here makes the policy silently misbehave.

```
obs (48) = base_lin_vel(3) base_ang_vel(3) projected_gravity(3) velocity_cmd(3)
           joint_pos_rel_default(12) joint_vel(12) prev_action(12)
action (12) = position-target deltas: q_target = default_joint_pos + action_scale * a
```

## Validation path (no hardware)

```bash
python training/export_policy.py --sb3 policy/ppo_mujoco.zip --out policy/policy.onnx
python control/runtime/policy_runner.py --sim          # closed loop in MuJoCo
python control/mock_esp32.py 8770 &                    # then stream to a fake ESP32:
python control/runtime/policy_runner.py --robot 127.0.0.1 --port 8770
```

## Reality of transfer on hobby servos

PCA9685 + cheap servos have backlash, latency, and no torque feedback. Mitigations
already in the pipeline: domain randomization in the Isaac task (friction, mass,
PD gains, pushes, observation noise), conservative `action_scale` (0.4), firmware
slew limiting, and per-joint clamps. Bench-test a single leg's step response before
trusting all 12.
