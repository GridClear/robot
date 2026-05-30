#include "net_wifi.h"
#include "config.h"
#include "protocol.h"
#include "app.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

static AsyncWebServer server(HTTP_PORT);
static AsyncWebSocket ws("/ws");

static void onWsEvent(AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType type,
                      void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      Command cmd;
      char reply[512];
      if (parseCommand((const char*)data, len, cmd)) {
        size_t n = app_handle(cmd, reply, sizeof(reply));
        c->text(reply, n);
      } else {
        c->text("{\"ok\":false,\"err\":\"parse\"}");
      }
    }
  }
}

void wifi_begin() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) delay(200);
  // Always raise the fallback AP too, so the robot is reachable even off-network.
  WiFi.softAP(AP_SSID, AP_PASS);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // REST: POST /cmd  (JSON body = command envelope)
  server.on("/cmd", HTTP_POST,
    [](AsyncWebServerRequest* req) {},
    nullptr,
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      Command cmd;
      char reply[512];
      if (parseCommand((const char*)data, len, cmd)) {
        size_t n = app_handle(cmd, reply, sizeof(reply));
        req->send(200, "application/json", String(reply).substring(0, n));
      } else {
        req->send(400, "application/json", "{\"ok\":false,\"err\":\"parse\"}");
      }
    });

  // REST: GET /state
  server.on("/state", HTTP_GET, [](AsyncWebServerRequest* req) {
    char reply[512];
    size_t n = app_state(reply, sizeof(reply), 0);
    req->send(200, "application/json", String(reply).substring(0, n));
  });

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/plain", "robotdog esp32: POST /cmd, GET /state, WS /ws");
  });

  server.begin();
}

void wifi_loop() { ws.cleanupClients(); }

void wifi_broadcastState() {
  if (ws.count() == 0) return;
  char reply[512];
  size_t n = app_state(reply, sizeof(reply), 0);
  ws.textAll(reply, n);
}
