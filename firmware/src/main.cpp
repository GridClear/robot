// Robot Dog ESP32 controller.
//   transports (WiFi HTTP+WS, BLE) -> app_handle() -> servos (PCA9685)
//   control loop runs at CONTROL_HZ with a command watchdog + E-STOP latch.
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "config.h"
#include "protocol.h"
#include "app.h"
#include "servos.h"
#include "net_wifi.h"
#include "net_ble.h"

static char curMode[24] = "boot";
static uint32_t lastCmdMs = 0;
static bool estopLatched = false;

static void firePhoto(const char* webhook) {
  if (WiFi.status() != WL_CONNECTED || strlen(webhook) == 0) return;
  HTTPClient http;
  http.begin(webhook);          // simple signal to an external capture service
  http.setTimeout(1500);
  http.GET();                   // fire-and-forget; capture service does the rest
  http.end();
}

size_t app_state(char* reply, size_t cap, uint32_t id) {
  float q[N_JOINTS];
  servos::getCurrent(q);
  // battery read placeholder: wire an ADC divider and replace 7.4f.
  return buildState(reply, cap, id, true, curMode, q, 7.4f,
                    estopLatched ? "estop" : nullptr);
}

size_t app_handle(const Command& c, char* reply, size_t cap) {
  lastCmdMs = millis();
  switch (c.type) {
    case CMD_ESTOP:
      estopLatched = true; servos::estop(true); strcpy(curMode, "estop"); break;
    case CMD_STOP:
      servos::stopHold(); strcpy(curMode, "hold"); break;
    case CMD_STAND:
      if (estopLatched) { estopLatched = false; servos::estop(false); }
      servos::standPose(); strcpy(curMode, "stand"); break;
    case CMD_GAIT:
      if (!estopLatched) { servos::setGait(c.gait, c.speed, c.dir);
        snprintf(curMode, sizeof(curMode), "gait:%s", c.gait); }
      break;
    case CMD_SET_JOINTS:
      if (!estopLatched && c.q_valid) { servos::setTargets(c.q); strcpy(curMode, "joints"); }
      break;
    case CMD_SET_JOINT:
      if (!estopLatched) { servos::setTarget(c.joint_index, c.angle); strcpy(curMode, "joint"); }
      break;
    case CMD_PHOTO:
      firePhoto(c.webhook); break;
    case CMD_STATE:
    default: break;
  }
  return app_state(reply, cap, c.id);
}

void setup() {
  Serial.begin(115200);
  servos::begin();
  wifi_begin();
  ble_begin();
  servos::standPose();
  strcpy(curMode, "stand");
  lastCmdMs = millis();
  Serial.println("robotdog ready");
}

void loop() {
  static uint32_t last = 0;
  const uint32_t period = 1000 / CONTROL_HZ;
  uint32_t now = millis();
  if (now - last < period) { wifi_loop(); return; }
  float dt = (now - last) / 1000.0f;
  last = now;

  // watchdog: lost comms -> hold safe pose (not estop, so it can recover)
  if (!estopLatched && now - lastCmdMs > WATCHDOG_MS && servos::gaitActive()) {
    servos::stopHold();
    strcpy(curMode, "hold(wd)");
  }

  servos::gaitStep(dt);
  servos::update(dt);
  wifi_loop();

  // ~5 Hz telemetry push
  static uint32_t lastTel = 0;
  if (now - lastTel > 200) { lastTel = now; wifi_broadcastState(); ble_notifyState(); }
}
