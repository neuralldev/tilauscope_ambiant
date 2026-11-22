#include "door.h"
#ifndef TILAU_AUDIO_ONLY
#include "common.h"

// Frame, 12 bytes (tilaucore/drop_door.py parses it):
//   header 0x5555 | open u8 | open_count u16 | since_open_ms u32 (0xFFFFFFFF = none) | checksum u8 | footer 0xAAAA
typedef struct __attribute__((packed))
{
  uint16_t header = 0x5555;
  uint8_t open;
  uint16_t open_count;
  uint32_t since_open_ms;
  uint8_t checksum;
  uint16_t footer = 0xAAAA;
} DoorData;

// Written by loop(), read by the NimBLE task.
static portMUX_TYPE doorMux = portMUX_INITIALIZER_UNLOCKED;
static bool doorOpen = true;         // debounced state
static uint16_t doorOpenCount = 0;   // openings since boot
static uint32_t doorOpenAt = 0;      // millis() when the last opening began
static bool doorEverOpened = false;

// loop() only.
static bool rawLast = true;
static uint32_t rawSince = 0;

void doorBegin()
{
  pinMode(DOOR_PIN, INPUT_PULLUP);   // the external 10 k sets the level; this one is a spare
  doorOpen = rawLast = digitalRead(DOOR_PIN) == HIGH;   // open at boot is no opening
  rawSince = millis();
  Serial.printf("Drop door on GPIO %d: %s\n", DOOR_PIN, doorOpen ? "open or not wired" : "closed");
}

void doorService()
{
  bool raw = digitalRead(DOOR_PIN) == HIGH;
  uint32_t now = millis();
  if (raw != rawLast)
  {
    rawLast = raw;
    rawSince = now;   // a bounce restarts the wait; the edge is dated from here
    return;
  }
  if (raw == doorOpen || now - rawSince < DOOR_DEBOUNCE_MS)
    return;
  portENTER_CRITICAL(&doorMux);
  doorOpen = raw;
  if (raw)
  {
    doorOpenCount++;
    doorOpenAt = rawSince;
    doorEverOpened = true;
  }
  portEXIT_CRITICAL(&doorMux);
  Serial.printf("Drop door %s (openings: %u)\n", raw ? "open" : "closed", doorOpenCount);
}

class DoorCallbacks : public NimBLECharacteristicCallbacks
{
  void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
  {
    DoorData data;
    portENTER_CRITICAL(&doorMux);
    data.open = doorOpen ? 1 : 0;
    data.open_count = doorOpenCount;
    data.since_open_ms = doorEverOpened ? millis() - doorOpenAt : 0xFFFFFFFFu;
    portEXIT_CRITICAL(&doorMux);
    const uint8_t *payload = reinterpret_cast<const uint8_t *>(&data) + sizeof(data.header);
    data.checksum = calculateChecksum(payload, sizeof(data.open) + sizeof(data.open_count) + sizeof(data.since_open_ms));
    pCharacteristic->setValue(reinterpret_cast<uint8_t *>(&data), sizeof(DoorData));
  }
};

void doorAttach(NimBLEService *service)
{
  NimBLECharacteristic *doorChar = service->createCharacteristic(DOOR_CHAR_UUID, NIMBLE_PROPERTY::READ);
  doorChar->setCallbacks(new DoorCallbacks());
}
#endif
