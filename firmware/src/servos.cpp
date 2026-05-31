#include "servos.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

namespace servos {

static Adafruit_PWMServoDriver pca(PCA9685_ADDR);
static float target[N_JOINTS];
static float current[N_JOINTS];
static bool estopped = false;

// gait state
static char gaitName[12] = "idle";
static float gaitSpeed = 0.0f;
static float gaitDir[3] = {0, 0, 0};
static float gaitClock = 0.0f;

static inline float clampq(int j, float q) {
  if (q < SERVOS[j].lower) return SERVOS[j].lower;
  if (q > SERVOS[j].upper) return SERVOS[j].upper;
  return q;
}

// joint angle (rad) -> PCA9685 "tick" (0..4095) at SERVO_FREQ_HZ.
static uint16_t angleToTick(int j, float q) {
  const ServoCal& s = SERVOS[j];
  float sa = s.direction * (q + s.zero_offset);
  float frac = (sa - s.angle_min) / (s.angle_max - s.angle_min);
  if (frac < 0) frac = 0; else if (frac > 1) frac = 1;
  float us = s.pwm_min + (s.pwm_max - s.pwm_min) * frac;
  float period_us = 1000000.0f / SERVO_FREQ_HZ;
  return (uint16_t)lroundf(us / period_us * 4096.0f);
}

static void writeJoint(int j, float q) {
  if (estopped) return;
  pca.setPWM(SERVOS[j].channel, 0, angleToTick(j, q));
}

void begin() {
  Wire.begin(I2C_SDA, I2C_SCL);
  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(SERVO_FREQ_HZ);
#if PCA9685_OE_PIN >= 0
  pinMode(PCA9685_OE_PIN, OUTPUT);
  digitalWrite(PCA9685_OE_PIN, LOW);  // active-low: LOW = outputs enabled
#endif
  for (int j = 0; j < N_JOINTS; j++) {
    current[j] = target[j] = SERVOS[j].default_q;
    writeJoint(j, current[j]);
  }
}

void setTarget(int j, float q) { if (j >= 0 && j < N_JOINTS) target[j] = clampq(j, q); }
void setTargets(const float q[N_JOINTS]) { for (int j = 0; j < N_JOINTS; j++) target[j] = clampq(j, q[j]); }
void standPose() { strcpy(gaitName, "stand"); for (int j = 0; j < N_JOINTS; j++) target[j] = SERVOS[j].default_q; }
void stopHold() { strcpy(gaitName, "idle"); for (int j = 0; j < N_JOINTS; j++) target[j] = current[j]; }

void estop(bool latched) {
  estopped = latched;
#if PCA9685_OE_PIN >= 0
  digitalWrite(PCA9685_OE_PIN, latched ? HIGH : LOW);  // HIGH = outputs hi-Z (limp)
#endif
  if (!latched) { for (int j = 0; j < N_JOINTS; j++) writeJoint(j, current[j]); }
}

void update(float dt) {
  float maxstep = SLEW_RAD_PER_S * dt;
  for (int j = 0; j < N_JOINTS; j++) {
    float e = target[j] - current[j];
    if (e > maxstep) e = maxstep; else if (e < -maxstep) e = -maxstep;
    current[j] += e;
    writeJoint(j, current[j]);
  }
}

void getCurrent(float out[N_JOINTS]) { for (int j = 0; j < N_JOINTS; j++) out[j] = current[j]; }

// ---- gait ----
void setGait(const char* name, float speed, const float dir[3]) {
  strlcpy(gaitName, name, sizeof(gaitName));
  gaitSpeed = speed < 0 ? 0 : (speed > 1 ? 1 : speed);
  for (int i = 0; i < 3; i++) gaitDir[i] = dir[i];
  if (!strcmp(name, "stand")) standPose();
}
bool gaitActive() { return strcmp(gaitName, "walk") == 0 || strcmp(gaitName, "trot") == 0; }

// This robot is 8-DOF: 2 joints per leg (abduction + knee), NOT 3. SERVOS order:
//   0:FL_abd 1:FL_knee  2:FR_abd 3:FR_knee  4:RL_abd 5:RL_knee  6:RR_abd 7:RR_knee
// Gait ported from the tuned robotdog8.ino: sinusoidal abduction + a 90deg
// phase-shifted sinusoidal knee, trot diagonal phasing. Right-side mirroring is
// applied downstream in angleToTick (s.direction), so offsets here are symmetric
// across legs (same convention as the policy set_joints stream). gaitSpeed only
// gates walking; cadence/amplitude come from the tuned config.h values.
void gaitStep(float dt) {
  if (!gaitActive() || gaitSpeed <= 0.0f) return;
  gaitClock += dt;
  const float legPhase[4] = {0.0f, PI, PI, 0.0f};     // FL, FR, RL, RR (trot)
  float fwd = gaitDir[0] >= 0 ? 1.0f : -1.0f;
  for (int leg = 0; leg < 4; leg++) {
    float ph = legPhase[leg] + 2.0f * PI * GAIT_FREQ_HZ * gaitClock;
    float abdOff  = GAIT_AMP_ABD  * sinf(ph) * fwd;
    float kneeOff = GAIT_AMP_KNEE * sinf(ph + GAIT_PHASE_RAD);
    int a = leg * 2, k = leg * 2 + 1;                 // abduction, knee
    target[a] = clampq(a, SERVOS[a].default_q + abdOff);
    target[k] = clampq(k, SERVOS[k].default_q + kneeOff);
  }
}

}  // namespace servos
