#ifndef TILAUONAUDIO_H
#define TILAUONAUDIO_H

#include <driver/i2s.h>
#include <arduinoFFT.h>

#pragma once
#include <NimBLEDevice.h>

# if defined(TILAUONAUDIO_H)

#define BUFFER_SIZE       1024
#define REFRACTORY_MS     150
#define CALIB_TIME_MS     30000
#define I2S_SAMPLE_RATE   16000
#define I2S_WS_PIN    25 // L/R Clock (Word Select)
#define I2S_SD_PIN    32 // Data Out (SD/DOUT)
#define I2S_SCK_PIN   33 // Bit Clock
#define I2S_PORT      I2S_NUM_0 // Utilisation du port I2S 0

#define COMMAND_RUNCALIBRATION 0x0000
#define COMMAND_START_SAMPLING 0x0001
#define COMMAND_STOP_SAMPLING 0x0002
#define COMMAND_CALIBRATIONSTATE 0x0003
#define COMMAND_SAMPLINGSTATUS 0x0004
#define COMMAND_GETCRACKCOUNTER 0x0005

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

int TestAudio(int m);

extern bool crackCounterStatus; // true = running, false = not running
extern bool isCalibrated;    // true = calibration has been done and finished ok, false=calibraton not done, skip counting
extern bool audioStarted;       // true = audio correctly initialized, false = not initialized, therefore no feature working

# endif

#endif
