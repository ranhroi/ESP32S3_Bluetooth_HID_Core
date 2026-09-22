/**
 * Tools > USB Mode > USB-OTG (TinyUSB)
 * Install library NimBLEDevice
 */

#include <Arduino.h>

// ---- BLE (Peripheral) via NimBLE ----
#include <NimBLEDevice.h>

// ---- USB HID Keyboard & Mouse (Arduino-ESP32 core) ----
#include "USB.h"
#include "USBHIDKeyboard.h"
#include "USBHIDMouse.h"

// =====================
// UUIDs (MUST MATCH iOS)
// =====================
static const char* kServiceUUID = "2D2A0001-8A5A-4E76-A2E3-1E57D9A1B001";
static const char* kWriteCharUUID = "2D2A0002-8A5A-4E76-A2E3-1E57D9A1B001";

// =====================
// Protocol Commands
// =====================

// ---- v1 (legacy): 3 bytes per frame ----
static const uint8_t CMD_KEY = 0x01;
static const uint8_t CMD_MOUSE_MOVE = 0x02;
static const uint8_t CMD_MOUSE_CLICK = 0x03;
static const uint8_t CMD_MOUSE_SCROLL = 0x04;

// ---- v2 (future-ready): [0xAA, 0x01] header + TLV frames ----
// Frame format: [cmd][len][payload...]
static const uint8_t V2_MAGIC = 0xAA;
static const uint8_t V2_VERSION = 0x01;

// Keyboard
static const uint8_t V2_SET_MODIFIERS = 0x01;  // payload: [mask]
static const uint8_t V2_KEY_DOWN = 0x02;       // payload: [keycode]
static const uint8_t V2_KEY_UP = 0x03;         // payload: [keycode]
static const uint8_t V2_KEY_TAP = 0x04;        // payload: [mask, keycode]

// Mouse
static const uint8_t V2_MOUSE_MOVE = 0x10;    // payload: [dx, dy]
static const uint8_t V2_MOUSE_SCROLL = 0x11;  // payload: [dx, dy]
static const uint8_t V2_MOUSE_CLICK = 0x12;   // payload: [button]
static const uint8_t V2_MOUSE_DOWN = 0x13;    // payload: [button]
static const uint8_t V2_MOUSE_UP = 0x14;      // payload: [button]

// =====================
// USB HID instances
// =====================
USBHIDKeyboard Keyboard;
USBHIDMouse Mouse;

static uint8_t gModifiersMask = 0x00;
static bool gKeysDown[256] = { false };

static void setModifiers(uint8_t newMask) {
  uint8_t diff = gModifiersMask ^ newMask;
  if (!diff) return;

  // Raw modifier codes: 0xE0..0xE7
  if (diff & 0x01) {
    if (newMask & 0x01) Keyboard.pressRaw(0xE0);
    else Keyboard.releaseRaw(0xE0);
  }  // LCtrl
  if (diff & 0x02) {
    if (newMask & 0x02) Keyboard.pressRaw(0xE1);
    else Keyboard.releaseRaw(0xE1);
  }  // LShift
  if (diff & 0x04) {
    if (newMask & 0x04) Keyboard.pressRaw(0xE2);
    else Keyboard.releaseRaw(0xE2);
  }  // LAlt
  if (diff & 0x08) {
    if (newMask & 0x08) Keyboard.pressRaw(0xE3);
    else Keyboard.releaseRaw(0xE3);
  }  // LGUI
  if (diff & 0x10) {
    if (newMask & 0x10) Keyboard.pressRaw(0xE4);
    else Keyboard.releaseRaw(0xE4);
  }  // RCtrl
  if (diff & 0x20) {
    if (newMask & 0x20) Keyboard.pressRaw(0xE5);
    else Keyboard.releaseRaw(0xE5);
  }  // RShift
  if (diff & 0x40) {
    if (newMask & 0x40) Keyboard.pressRaw(0xE6);
    else Keyboard.releaseRaw(0xE6);
  }  // RAlt
  if (diff & 0x80) {
    if (newMask & 0x80) Keyboard.pressRaw(0xE7);
    else Keyboard.releaseRaw(0xE7);
  }  // RGUI

  gModifiersMask = newMask;
}

static void keyDown(uint8_t keycode) {
  if (keycode == 0x00) return;
  if (gKeysDown[keycode]) return;
  Keyboard.pressRaw(keycode);
  gKeysDown[keycode] = true;
}

static void keyUp(uint8_t keycode) {
  if (keycode == 0x00) return;
  if (!gKeysDown[keycode]) return;
  Keyboard.releaseRaw(keycode);
  gKeysDown[keycode] = false;
}

static void keyTap(uint8_t modifiersMask, uint8_t keycode) {
  if (keycode == 0x00) return;

  bool wasDown = gKeysDown[keycode];
  uint8_t savedMods = gModifiersMask;

  setModifiers(modifiersMask);

  if (!wasDown) {
    keyDown(keycode);
    delay(5);
    keyUp(keycode);
    delay(1);
  }

  setModifiers(savedMods);
}

// v1 compatibility shim
static void sendKeyPressAndRelease(uint8_t modifiers, uint8_t keycode) {
  keyTap(modifiers, keycode);
}

static void sendMouseMove(int8_t dx, int8_t dy) {
  Mouse.move(dx, dy);
}

static void sendMouseClick(uint8_t button) {
  // button: 1=left, 2=right, 4=middle
  Mouse.click(button);
}

static void sendMouseButtonDown(uint8_t button) {
  Mouse.press(button);
}

static void sendMouseButtonUp(uint8_t button) {
  Mouse.release(button);
}

static void sendMouseScroll(int8_t dx, int8_t dy) {
  // Scroll: positive Y = scroll up, negative Y = scroll down
  Mouse.move(0, 0, dy, dx);
}

// =====================
// BLE GATT server
// =====================
NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pWriteChar = nullptr;

class WriteCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
    std::string v = pCharacteristic->getValue();
    if (v.size() < 3) return;

    // ---- v2: [0xAA, 0x01] + TLV frames ----
    if ((uint8_t)v[0] == V2_MAGIC && (uint8_t)v[1] == V2_VERSION) {
      size_t idx = 2;

      while (idx + 1 < v.size()) {
        uint8_t cmd = (uint8_t)v[idx + 0];
        uint8_t len = (uint8_t)v[idx + 1];
        idx += 2;

        if (idx + len > v.size()) break;

        const uint8_t* payload = (const uint8_t*)&v[idx];

        switch (cmd) {
          case V2_SET_MODIFIERS:
            if (len == 1) setModifiers(payload[0]);
            break;

          case V2_KEY_DOWN:
            if (len == 1) keyDown(payload[0]);
            break;

          case V2_KEY_UP:
            if (len == 1) keyUp(payload[0]);
            break;

          case V2_KEY_TAP:
            if (len == 2) keyTap(payload[0], payload[1]);
            break;

          case V2_MOUSE_MOVE:
            if (len == 2) sendMouseMove((int8_t)payload[0], (int8_t)payload[1]);
            break;

          case V2_MOUSE_SCROLL:
            if (len == 2) sendMouseScroll((int8_t)payload[0], (int8_t)payload[1]);
            break;

          case V2_MOUSE_CLICK:
            if (len == 1) sendMouseClick(payload[0]);
            break;

          case V2_MOUSE_DOWN:
            if (len == 1) sendMouseButtonDown(payload[0]);
            break;

          case V2_MOUSE_UP:
            if (len == 1) sendMouseButtonUp(payload[0]);
            break;

          default:
            break;
        }

        idx += len;
      }

      return;
    }

    // ---- v1: 3-byte frames (supports batching) ----
    for (size_t i = 0; i + 2 < v.size(); i += 3) {
      uint8_t type = (uint8_t)v[i + 0];
      uint8_t byte1 = (uint8_t)v[i + 1];
      uint8_t byte2 = (uint8_t)v[i + 2];

      switch (type) {
        case CMD_KEY:
          sendKeyPressAndRelease(byte1, byte2);
          break;

        case CMD_MOUSE_MOVE:
          sendMouseMove((int8_t)byte1, (int8_t)byte2);
          break;

        case CMD_MOUSE_CLICK:
          sendMouseClick(byte1);
          break;

        case CMD_MOUSE_SCROLL:
          sendMouseScroll((int8_t)byte1, (int8_t)byte2);
          break;

        default:
          break;
      }
    }
  }
};

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    Serial.print("BLE connected: ");
    Serial.println(connInfo.getAddress().toString().c_str());
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    Serial.print("BLE disconnected, reason=");
    Serial.println(reason);

    NimBLEDevice::startAdvertising();
  }
};

static void setupBle() {
  NimBLEDevice::init("MouseKeyboard-ESP32S3");   
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService* svc = pServer->createService(kServiceUUID);
  pWriteChar = svc->createCharacteristic(
    kWriteCharUUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pWriteChar->setCallbacks(new WriteCallbacks());
  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();

  // Bật scan response TRƯỚC
  adv->enableScanResponse(true);

  // Đưa UUID vào advertising packet chính
  adv->addServiceUUID(kServiceUUID);

  // Đưa tên dài vào scan response
  adv->setName("MouseKeyboard-ESP32S3");

  adv->start();

  Serial.println("BLE advertising started.");
}

static void setupUsbHid() {
  Keyboard.begin();
  Mouse.begin();
  USB.begin();

  Serial.println("USB HID Keyboard & Mouse started.");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("Starting ESP32-S3 BLE -> USB HID keyboard & mouse bridge...");

  setupUsbHid();
  setupBle();
}

void loop() {
  delay(20);
}
