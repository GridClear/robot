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

// ---- Hardware ----
#define I2C_SDA   21
#define I2C_SCL   22
#define PCA9685_ADDR  0x40
// Optional active-low enable on PCA9685 /OE to cut all servo power on E-STOP.
// Set to -1 if /OE is tied to GND (always enabled).
#define PCA9685_OE_PIN  -1
