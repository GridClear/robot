// PCA9685 servo driver + joint-angle->PWM mapping (mirrors control/runtime/servo_math.py)
// plus slew limiting, a built-in trot gait generator, and E-STOP power cut.
#pragma once
#include <Arduino.h>
#include "servo_config.h"

namespace servos {

void begin();                       // I2C + PCA9685 init, move to safe stand pose
void setTarget(int joint, float q); // set one joint target (rad), clamped
void setTargets(const float q[N_JOINTS]);
void standPose();                   // targets = per-joint default
void estop(bool latched);           // cut servo power (true) / re-enable (false)
void stopHold();                    // freeze targets at current position
void update(float dt);              // slew toward targets, write PWM (call at CONTROL_HZ)
void getCurrent(float out[N_JOINTS]);

// Gait: name in {idle,stand,walk,trot}; speed 0..1; dir = [vx,vy,wz].
// Produces joint targets internally; call update() to drive servos.
void setGait(const char* name, float speed, const float dir[3]);
void gaitStep(float dt);            // advance gait clock -> targets (no-op if idle/stand)
bool gaitActive();

}  // namespace servos
