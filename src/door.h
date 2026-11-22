// Drop door contact (wiki/Drop-Door-Contact-Spec.md in TilauScope).
// Reed form A (closed when the magnet on the door is near) between DOOR_PIN and GND,
// 10 k pull-up to 3V3: LOW = door closed, HIGH = door open or nothing wired.
// The probe counts the openings and says how long ago the last one was, so the app
// never misses one between two reads and dates DROP on the opening itself.
#pragma once
#ifndef TILAU_AUDIO_ONLY   // GPIO 10 is a flash pin on the WROOM-32D
#include <Arduino.h>
#include <NimBLEDevice.h>

#define DOOR_CHAR_UUID   "f3b6e2a0-8c4e-4e1f-9c2d-1a7f5b9a1c06"
#define DOOR_PIN         10
#define DOOR_DEBOUNCE_MS 30

void doorBegin();                          // setup(): pin and first reading
void doorService();                        // loop(): debounce, count the openings
void doorAttach(NimBLEService *service);   // setup(): the read-only characteristic
#endif
