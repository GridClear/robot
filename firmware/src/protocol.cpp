#include "protocol.h"
#include <ArduinoJson.h>

static int jointIndexByName(const char* name) {
  for (int i = 0; i < N_JOINTS; i++)
    if (strcmp(SERVOS[i].name, name) == 0) return i;
  return -1;
}

bool parseCommand(const char* json, size_t len, Command& out) {
  JsonDocument doc;
  if (deserializeJson(doc, json, len)) return false;

  const char* cmd = doc["cmd"] | "";
  out = Command();
  out.id = doc["id"] | 0;
  JsonObjectConst a = doc["args"];

  if      (!strcmp(cmd, "estop"))  out.type = CMD_ESTOP;
  else if (!strcmp(cmd, "stop"))   out.type = CMD_STOP;
  else if (!strcmp(cmd, "stand"))  out.type = CMD_STAND;
  else if (!strcmp(cmd, "state?")) out.type = CMD_STATE;
  else if (!strcmp(cmd, "gait")) {
    out.type = CMD_GAIT;
    strlcpy(out.gait, a["name"] | "idle", sizeof(out.gait));
    out.speed = a["speed"] | 0.0f;
    JsonArrayConst d = a["dir"];
    for (int i = 0; i < 3 && i < (int)d.size(); i++) out.dir[i] = d[i];
  } else if (!strcmp(cmd, "set_joints")) {
    out.type = CMD_SET_JOINTS;
    JsonArrayConst qa = a["q"];
    if (qa.size() == N_JOINTS) {
      for (int i = 0; i < N_JOINTS; i++) out.q[i] = qa[i];
      out.q_valid = true;
    } else return false;
  } else if (!strcmp(cmd, "set_joint")) {
    out.type = CMD_SET_JOINT;
    out.joint_index = jointIndexByName(a["name"] | "");
    out.angle = a["angle"] | 0.0f;
    if (out.joint_index < 0) return false;
  } else if (!strcmp(cmd, "photo")) {
    out.type = CMD_PHOTO;
    strlcpy(out.webhook, a["webhook"] | "", sizeof(out.webhook));
  } else {
    return false;
  }
  return true;
}

size_t buildState(char* buf, size_t cap, uint32_t id, bool ok,
                  const char* mode, const float* q, float batt, const char* err) {
  JsonDocument doc;
  doc["id"] = id;
  doc["ok"] = ok;
  doc["mode"] = mode;
  JsonArray qa = doc["q"].to<JsonArray>();
  for (int i = 0; i < N_JOINTS; i++) qa.add(q[i]);
  doc["batt"] = batt;
  if (err) doc["err"] = err; else doc["err"] = nullptr;
  return serializeJson(doc, buf, cap);
}
