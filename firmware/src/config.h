// User-editable runtime configuration. Servo calibration lives in the
// AUTO-GENERATED servo_config.h (from model/params.yaml) — edit params, not here.
#pragma once

// ---- WiFi ----
// STA mode: join an existing network. If JOIN fails, falls back to its own AP.
#define WIFI_SSID      "your-wifi-ssid"
#define WIFI_PASS      "your-wifi-password"
#define AP_SSID        "robotdog"      // fallback access point
#define AP_PASS        "walkies123"    // >= 8 chars
#define HTTP_PORT      80
#define MDNS_HOST      "robotdog"      // reachable at http://robotdog.local/ (mDNS)

// ---- BLE ----
#define BLE_DEVICE_NAME   "robotdog"
// 128-bit UUIDs for the control service (must match control/runtime/transport_ble.py)
#define BLE_SVC_UUID      "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_CMD_UUID      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // write  (JSON command)
#define BLE_STATE_UUID    "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // notify (JSON telemetry)

// ---- Safety / control loop ----
#define CONTROL_HZ          50      // servo command update rate
#define WATCHDOG_MS         500     // no command in this window -> hold safe pose
#define SLEW_RAD_PER_S      6.0f    // max joint slew (limits inrush + protects gears)

// ---- Scripted trot gait (ported from the tuned robotdog8.ino: amp 28/30 deg,
//      freq 0.6 Hz, 90 deg phase). Amplitudes are radian swings about default_q. ----
#define GAIT_FREQ_HZ        0.6f      // cadence (slow, steady)
#define GAIT_AMP_ABD        0.45f     // abduction swing (rad, ~26 deg)
#define GAIT_AMP_KNEE       0.50f     // knee swing (rad, ~29 deg)
#define GAIT_PHASE_RAD      1.5708f   // abduction->knee phase offset (90 deg)

// ---- Hardware ----
#define I2C_SDA   21
#define I2C_SCL   22
#define PCA9685_ADDR  0x40
// Optional active-low enable on PCA9685 /OE to cut all servo power on E-STOP.
// Set to -1 if /OE is tied to GND (always enabled).
#define PCA9685_OE_PIN  -1
