#pragma once
void wifi_begin();
void wifi_loop();          // housekeeping (WS cleanup)
void wifi_broadcastState(); // push telemetry to all WS clients
