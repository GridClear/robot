// Bridge between transports (WiFi/BLE) and the control core in main.cpp.
#pragma once
#include "protocol.h"

// Handle a parsed command. Writes a JSON reply (telemetry/ack) into reply/cap.
// Returns reply length. Thread-safe enough for the single control task model:
// transports queue the raw command; main applies it. Here we apply directly and
// it's cheap, so we keep it synchronous.
size_t app_handle(const Command& c, char* reply, size_t cap);

// Fill a state/telemetry JSON (used for unsolicited notify pushes).
size_t app_state(char* reply, size_t cap, uint32_t id);
