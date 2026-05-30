#include "net_ble.h"
#include "config.h"
#include "protocol.h"
#include "app.h"
#include <NimBLEDevice.h>

static NimBLECharacteristic* cmdChar = nullptr;
static NimBLECharacteristic* stateChar = nullptr;

class CmdCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c) {
    std::string v = c->getValue();
    Command cmd;
    char reply[512];
    if (parseCommand(v.c_str(), v.size(), cmd)) {
      size_t n = app_handle(cmd, reply, sizeof(reply));
      if (stateChar) { stateChar->setValue((uint8_t*)reply, n); stateChar->notify(); }
    }
  }
};

void ble_begin() {
  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setMTU(247);   // allow larger JSON frames
  NimBLEServer* server = NimBLEDevice::createServer();
  NimBLEService* svc = server->createService(BLE_SVC_UUID);

  cmdChar = svc->createCharacteristic(BLE_CMD_UUID, NIMBLE_PROPERTY::WRITE);
  cmdChar->setCallbacks(new CmdCallbacks());

  stateChar = svc->createCharacteristic(
      BLE_STATE_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  svc->start();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SVC_UUID);
  adv->setName(BLE_DEVICE_NAME);
  adv->start();
}

void ble_notifyState() {
  if (!stateChar) return;
  char reply[512];
  size_t n = app_state(reply, sizeof(reply), 0);
  stateChar->setValue((uint8_t*)reply, n);
  stateChar->notify();
}
