#ifndef TILAUONAUDIO_H
#define TILAUONAUDIO_H

#include <driver/i2s.h>
#include <arduinoFFT.h>

#pragma once
#include <NimBLEDevice.h>

# if defined(TILAUONAUDIO_H)

#define BUFFER_SIZE       512
#define REFRACTORY_MS     100
#define CALIB_TIME_MS     30000
#define I2S_SAMPLE_RATE   16000
#define I2S_BCK_PIN       14
#define I2S_WS_PIN        13
#define I2S_SD_PIN        34
#define I2S_PORT          I2S_NUM_0

// BLE Characteristic UUID for the audio crack count data 
#define ENV_AUDIO_CHAR_UUID  "f3b6e2a0-8c4e-4e1f-9c2d-1a7f5b9a1c04" 

extern NimBLECharacteristic* envAudioChar; // Pointer to the BLE Characteristic for audio
extern TaskHandle_t monitorTaskHandle;

//void monitorAudio();        // sampling routine
void monitorAudioTask(void *pvParameters);

class AudioDataCallbacks: public NimBLECharacteristicCallbacks { 
    void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override;
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override;
    void onStatus(NimBLECharacteristic* pCharacteristic, int code) override;
    void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override;  
};

extern bool crackCounterStatus; // true = running, false = not running
extern bool isCalibrated;    // true = calibration has been done and finished ok, false=calibraton not done, skip counting
extern bool audioStarted;       // true = audio correctly initialized, false = not initialized, therefore no feature working

# endif

#endif
