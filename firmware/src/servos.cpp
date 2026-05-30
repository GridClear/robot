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

// joint type index within a leg: SERVOS are ordered FL{hip,thigh,knee} FR... etc.
void gaitStep(float dt) {
  if (!gaitActive() || gaitSpeed <= 0.0f) return;
  const float stepFreq = 1.5f + 1.5f * gaitSpeed;     // Hz
  const float amp = 0.35f * gaitSpeed;                // rad swing
  gaitClock += dt * stepFreq;
  // trot: diagonal pairs (FL+RR) and (FR+RL) in antiphase.
  // leg order in SERVOS: 0:FL 1:FR 2:RL 3:RR (each *3 joints)
  const float legPhase[4] = {0.0f, PI, PI, 0.0f};     // FL, FR, RL, RR
  float fwd = gaitDir[0] >= 0 ? 1.0f : -1.0f;
  for (int leg = 0; leg < 4; leg++) {
    float ph = 2 * PI * gaitClock + legPhase[leg];
    float lift = max(0.0f, sinf(ph));                 // swing = lift foot
    float swing = cosf(ph) * fwd;                     // fore/aft
    int base = leg * 3;                               // hip, thigh, knee
    target[base + 0] = clampq(base + 0, 0.0f);                              // hip (abduction)
    target[base + 1] = clampq(base + 1, SERVOS[base + 1].default_q + amp * swing);     // thigh
    target[base + 2] = clampq(base + 2, SERVOS[base + 2].default_q - amp * lift * 1.4f); // knee tuck on swing
  }
}

}  // namespace servos
