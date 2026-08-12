#ifndef TILAUONAUDIO_H
#define TILAUONAUDIO_H

// API I2S legacy (Arduino-ESP32 2.x / IDF 4.4) — fonctionne aussi sur l'ESP32-S3
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

// Pins I2S INMP441 — board-dependent.
//  - ESP32-S3 (N8R8) : GPIO 4/5/6 (les 25/32/33 du WROOM-32 sont absents ou
//    réservés flash/PSRAM octale sur le S3).
//  - ESP32-WROOM-32D (classique) : GPIO 25/33/32.
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #define I2S_WS_PIN    5     // L/R Clock (Word Select) -> GPIO5
  #define I2S_SD_PIN    4     // Data In du micro (SD/DOUT) -> GPIO4
  #define I2S_SCK_PIN   6     // Bit Clock -> GPIO6
#else  // ESP32-WROOM-32D
  #define I2S_WS_PIN    25    // L/R Clock (Word Select) -> GPIO25
  #define I2S_SD_PIN    32    // Data In du micro (SD/DOUT) -> GPIO32
  #define I2S_SCK_PIN   33    // Bit Clock -> GPIO33
#endif
#define I2S_PORT      I2S_NUM_0

// ---- Commandes BLE audio ----
#define COMMAND_RUNCALIBRATION      0x0000
#define COMMAND_START_SAMPLING      0x0001
#define COMMAND_STOP_SAMPLING       0x0002
#define COMMAND_CALIBRATIONSTATE    0x0003
#define COMMAND_SAMPLINGSTATUS      0x0004
#define COMMAND_GETCRACKCOUNTER     0x0005
#define COMMAND_DEBUG_ON            0x0010  // Active la verbosité série étendue
#define COMMAND_DEBUG_OFF           0x0011  // Désactive la verbosité série étendue

// UUID de la caractéristique BLE audio
#define ENV_AUDIO_CHAR_UUID  "f3b6e2a0-8c4e-4e1f-9c2d-1a7f5b9a1c04"

#define HOP            (FFT_SIZE/2)   // 128 samples = 8 ms @16k (50% overlap)
#define FLUX_F_LO      1000.0f        // bande de flux (Hz)
#define FLUX_F_HI      6000.0f
#define FLUX_ALPHA     0.02f          // EMA du plancher de flux (τ≈0.4 s)
#define CREST_MIN      4.0f           // gate d'impulsivité (~12 dB)
#define WARMUP_FRAMES 60           // (alternative basée sur le nombre de frames, pour plus de cohérence lors de l'ajout du BLE ratio adjust)
// K (multiplicateur du seuil) devient runtime pour les commandes BLE ratio :

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
