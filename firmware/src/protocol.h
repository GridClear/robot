// Shared command protocol — MUST match control/runtime/protocol.py and
// docs/command_protocol.md. Envelope: {"t":ms,"id":n,"cmd":name,"args":{...}}
#pragma once
#include <Arduino.h>
#include "servo_config.h"

enum CmdType {
  CMD_NONE, CMD_ESTOP, CMD_STOP, CMD_STAND, CMD_GAIT,
  CMD_SET_JOINTS, CMD_SET_JOINT, CMD_PHOTO, CMD_STATE
};

struct Command {
  CmdType type = CMD_NONE;
  uint32_t id = 0;
  // gait
  char gait[12] = "idle";
  float speed = 0.0f;
  float dir[3] = {0, 0, 0};   // vx, vy, wz
  // set_joints
  float q[N_JOINTS] = {0};
  bool q_valid = false;
  // set_joint
  int joint_index = -1;       // resolved from name
  float angle = 0.0f;
  // photo
  char webhook[160] = "";
};

// Parse a JSON command string. Returns false on malformed JSON / unknown cmd.
bool parseCommand(const char* json, size_t len, Command& out);

// Serialize a telemetry/ack reply into buf. Returns bytes written.
size_t buildState(char* buf, size_t cap, uint32_t id, bool ok,
                  const char* mode, const float* q, float batt, const char* err);
