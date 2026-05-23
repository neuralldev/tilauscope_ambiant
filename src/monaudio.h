#ifndef TILAUONAUDIO_H
#define TILAUONAUDIO_H

#include <driver/i2s.h>

#include <FS.h>
#include <LittleFS.h>

#define FILE_SYSTEM LittleFS

#include <ArduinoJson.h>

#pragma once
#include <NimBLEDevice.h>

#if defined(TILAUONAUDIO_H)

#define CALIBRATION_FILE "/audio_calib.json"

// ************** Audio / DSP parameters ******************
#include "esp_dsp.h"
#include "freertos/semphr.h"

// Temps mort anti-rebond pour le tambour métallique (ms)
#define DSP_DEAD_TIME_MS    200

struct AudioStats {
    uint32_t crack_count;       // compteur cumulatif
    float peak_amplitude;       // pic max observé
    float noise_floor_rms;      // RMS moyen du bruit de fond
    float threshold;            // seuil de détection calculé à la calibration
    float noise_p999;           // percentile 99.9% des amplitudes de bruit
    float noise_p99;            // percentile 99%
    float snr_db;               // SNR estimé en dB
};

extern AudioStats stats;

// Coefficients et état du filtre biquad (passe-bande crack)
extern float coeffs[5];
extern float filter_state[2];

// ---- Paramètres audio ----
#define BUFFER_SIZE         1024
#define FFT_SIZE            256         // FFT réduite (était 1024) — suffisant pour résolution ~62 Hz/bin à 16kHz
#define REFRACTORY_MS       150
#define CALIB_TIME_MS       30000
#define I2S_SAMPLE_RATE     16000

// Filtre passe-bande centré sur la zone fréquentielle des cracks (1.5–4 kHz)
// Fréquence centrale : ~2500 Hz, Q = 1.2 (bande d'environ 2 kHz)
#define BPF_CENTER_FREQ     2500.0f
#define BPF_Q               1.2f

// Pins I2S INMP441
#define I2S_WS_PIN    25    // L/R Clock (Word Select)
#define I2S_SD_PIN    32    // Data Out (SD/DOUT)
#define I2S_SCK_PIN   33    // Bit Clock
#define I2S_PORT      I2S_NUM_0

// ---- Commandes BLE audio ----
#define COMMAND_RUNCALIBRATION      0x0000
#define COMMAND_START_SAMPLING      0x0001
#define COMMAND_STOP_SAMPLING       0x0002
#define COMMAND_CALIBRATIONSTATE    0x0003
#define COMMAND_SAMPLINGSTATUS      0x0004
#define COMMAND_GETCRACKCOUNTER     0x0005
#define COMMAND_RAISERATIO          0x0006
#define COMMAND_DECREASERATIO       0x0007
#define COMMAND_RAISERATIO5         0x0506
#define COMMAND_DECREASERATIO5      0x0507
#define COMMAND_DEBUG_ON            0x0010  // Active la verbosité série étendue
#define COMMAND_DEBUG_OFF           0x0011  // Désactive la verbosité série étendue

// UUID de la caractéristique BLE audio
#define ENV_AUDIO_CHAR_UUID  "f3b6e2a0-8c4e-4e1f-9c2d-1a7f5b9a1c04"

extern NimBLECharacteristic* envAudioChar;
extern TaskHandle_t monitorTaskHandle;

// Flag debug série (activable/désactivable via BLE)
extern volatile bool audioDebugEnabled;

void monitorAudioTask(void *pvParameters);

class AudioDataCallbacks: public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override;
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override;
    void onStatus(NimBLECharacteristic* pCharacteristic, int code) override;
    void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override;
};

int TestAudio(int m);

// ---- Flags globaux — volatile pour visibilité cross-core ----
extern volatile bool crackCounterStatus; // true = comptage actif
extern volatile bool isCalibrated;       // true = calibration valide disponible
extern volatile bool audioStarted;       // true = I2S initialisé correctement
extern volatile bool calibrating;        // true = calibration en cours

bool saveCalibrationToFile();
bool loadCalibrationFromFile();

#endif  // defined(TILAUONAUDIO_H)

#endif  // TILAUONAUDIO_H
