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
#define CALIBRATION_TMP_FILE "/audio_calib.tmp"   // ecriture atomique (cf. saveCalibrationToFile)

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
    // Distribution des mêmes features que celles utilisées en détection.
    float flux_median;
    float flux_mad;
    float flux_p99;
    float flux_p999;
    float crest_p99;
    float crest_p999;
    float rms_median;
    float rms_p99;
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
// v4 : bande de flux passée à 4000-7900 Hz. Un profil v3 (stats de flux calculées
// sur 1000-4500 Hz) est rejeté au boot au lieu d'être appliqué à la mauvaise bande.
#define CALIBRATION_FORMAT_VERSION 4
#define I2S_SAMPLE_RATE     16000

// Filtre passe-bande (facteur de crête) centré là où les cracks se détachent du
// souffle du Skywalker. Mesuré sur capture réelle (Monsooned Malabar 2026-09-12,
// micro à 30-40 cm) : contraste crack/fond maximal en 4-8 kHz, alors que
// 1.5-4 kHz est dominé par la turbine. Centre 5500 Hz, Q = 1.2 (~3.5-8 kHz).
#define BPF_CENTER_FREQ     5500.0f
#define BPF_Q               1.2f

// Pins I2S ICS-43434 — board-dependent.
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
#define COMMAND_RUN_MAILLARD_CALIBRATION 0x0006
// Auto-test d'acquisition (~2 s) : lit l'I2S, streame les hops sur le port
// serie et publie #TILAU_SELFTEST frames/rms/peak. Ne calibre rien, ne compte
// rien : sert a prouver la chaine micro -> DSP -> UART AVANT d'engager un roast.
#define COMMAND_AUDIO_SELFTEST      0x0007
#define COMMAND_DEBUG_ON            0x0010  // Active la verbosité série étendue
#define COMMAND_DEBUG_OFF           0x0011  // Désactive la verbosité série étendue
#define COMMAND_GET_CONFIDENCE      0x0021  // Imprime confidencePct au port série

// Réglage utilisateur de la confiance (score composite), en %. Commande absolue et
// idempotente : 0x0100 + pct(0-100). Ex: 0x0100+80=0x0150 -> confidencePct=80.
// Volontairement pas de commande relative (cf. ancien réglage fluxK par BLE,
// retiré car ingérable côté Artisan).
#define COMMAND_SET_CONFIDENCE_BASE 0x0100
#define CONFIDENCE_DEFAULT          60.0f

extern volatile float confidencePct;   // 0-100 : seuil requis sur le score composite

// UUID de la caractéristique BLE audio
#define ENV_AUDIO_CHAR_UUID  "f3b6e2a0-8c4e-4e1f-9c2d-1a7f5b9a1c04"

#define HOP            (FFT_SIZE/2)   // 128 samples = 8 ms @16k (50% overlap)
// Bande de flux (Hz), choisie par rejeu du pipeline sur la capture du 2026-09-12 :
// 1000-4500 captait 6 cracks sur 20, 4000-7900 en capte 19 sur 20 sans onset
// supplémentaire avant le FC. Contrepartie : 4-8 kHz recouvre les sifflantes de
// la voix. 7900 reste sous Nyquist (8000).
#define FLUX_F_LO      4000.0f
#define FLUX_F_HI      7900.0f
#define FLUX_ALPHA     0.02f          // EMA du plancher de flux (τ≈0.4 s)
// Le seuil de detection est thr = flux_base + fluxK * flux_dev, ou flux_dev est
// entretenu en RUNTIME par un EMA de l'ecart absolu MOYEN. La calibration, elle,
// mesure une MAD (median absolute deviation) : pour du bruit gaussien
// MAD = 0.6745σ contre 0.7979σ pour l'ecart absolu moyen. Amorcer flux_dev avec
// la MAD brute donnerait un seuil ~18 % trop bas pendant les ~0.4 s de
// convergence de l'EMA, avec un fluxK regle empiriquement sur la valeur
// convergee. On convertit donc la MAD dans l'unite du runtime.
#define MAD_TO_MEANDEV 1.1829f        // 0.7979 / 0.6745
// Duree max de gel du plancher de flux. Le gel protege le detecteur pendant les
// salves de cracks, mais un gel INCONDITIONNEL bloque aussi l'adaptation quand
// le bruit de fond monte reellement (tambour qui chauffe, ventilation) : le
// plancher resterait sous le nouveau niveau ambiant et tout serait candidat.
// Entre deux cracks le flux repasse sous le seuil (les cracks durent quelques
// dizaines de ms), donc un depassement CONTINU de plus de 10 s ne peut pas etre
// une salve : c'est un changement de niveau, et l'EMA doit le suivre.
#define FLUX_FREEZE_MAX_MS  10000u
// Plancher absolu du seuil : FLUX_FLOOR_P99_FACTOR x P99 du flux de la calibration
// MAILLARD (cf. fluxFloor()). Remplace le P99.9, soit la 4e plus grande valeur sur
// ~3700 frames : un seul pic de plus ou de moins pendant la calibration le déplaçait
// de 5 à 10 % et changeait le nombre d'onsets d'environ 10 %. Le P99 (~37e valeur)
// est 3 à 5 fois plus stable (bootstrap sur les captures du 2026-09-12 et du
// 2026-09-20). Le facteur 1.35 conserve la sensibilité validée en direct le 2026-09-20.
#define FLUX_FLOOR_P99_FACTOR  1.35f
// La bande de flux doit rester sous Nyquist : au-dela, les bins n'existent pas
// et l'index deborderait de prevMag[FFT_SIZE/2] (borne aussi a l'execution).
static_assert(FLUX_F_LO < FLUX_F_HI, "FLUX_F_LO doit etre < FLUX_F_HI");
static_assert(FLUX_F_HI < I2S_SAMPLE_RATE / 2, "FLUX_F_HI doit rester sous Nyquist");
// Garde numérique seulement : la vraie référence du sous-score de crête est le P99
// mesuré pendant la calibration. Un crest factor est par définition >= 1.
#define CREST_REFERENCE_FLOOR  1.0f
#define WARMUP_FRAMES 60           // (alternative basée sur le nombre de frames, pour plus de cohérence lors de l'ajout du BLE ratio adjust)
// K (multiplicateur du seuil) devient runtime pour les commandes BLE ratio :

// ---- Score composite (remplace le AND booléen flux/crête) ----
// Poids favorisant flux+crête (établis sur de nombreux essais réels) ; le centroïde
// spectral est un discriminant plus récent, moins éprouvé -> poids plus faible.
#define SCORE_W_FLUX      0.45f
#define SCORE_W_CREST     0.30f
#define SCORE_W_CENTROID  0.25f   // W_FLUX + W_CREST + W_CENTROID doit valoir 1.0
// Tolérance (Hz) autour de BPF_CENTER_FREQ pour score_centroid. Avec la bande
// 4000-7900, ce sous-score agit presque comme une porte : une tolérance étroite
// rend le rappel très sensible à la confiance (à 65 % : 10/20 cracks avec 1500 Hz,
// 16/20 avec 2000 Hz) et à une erreur sur le centre du filtre.
#define CENTROID_TOLERANCE_HZ  2000.0f

// ---- FSM burst detection (cf. ALGORITHM_NOTES.md) ----
typedef enum {
    FC_STATE_IDLE = 0,
    FC_STATE_MONITORING,
    FC_STATE_CANDIDATE,
    FC_STATE_CONFIRMED
} FCState;

// NOTE : fcState n'est PAS remonte en BLE aujourd'hui — le client ne lit que
// crack_count, la FSM ne sort que sur l'UART (#TILAU_FC). L'exposer demande une
// caracteristique NOTIFY (piste 6 du CLAUDE.md) et une evolution cote TilauScope.
extern volatile FCState    fcState;
extern volatile uint32_t   fcConfirmedTimestamp;  // millis() de l'entrée en candidature confirmée

#define FC_START_SHORT_COUNT         2
#define FC_START_SHORT_WINDOW_MS     5000
#define FC_START_LONG_COUNT          5
#define FC_START_LONG_WINDOW_MS      10000

// Confirmation assouplie (était 8 en 10 s, silence 5 s, timeout 15 s) : sur un FC
// clairsemé (~15-20 cracks/min, Monsooned Malabar 2026-09-12), 8 en 10 s n'était
// jamais atteint avant le DROP. 6 en 20 s confirme à FC+16 s en rejeu, sans fausse
// confirmation avant le FC dans les réglages testés (5 en 15 s confirmait plus tôt
// mais s'est déclenché 31 s avant le FC à confiance 50 %).
#define FC_CONFIRM_COUNT             6
#define FC_CONFIRM_WINDOW_MS         20000
#define FC_CONFIRM_RECENT_COUNT      2
#define FC_CONFIRM_RECENT_WINDOW_MS  5000

#define FC_CANDIDATE_SILENCE_MS      8000
#define FC_CANDIDATE_TIMEOUT_MS      25000
#define FC_BURST_HISTORY             40   // marge au-delà du max plausible sur les fenêtres FC

extern NimBLECharacteristic* envAudioChar;
extern TaskHandle_t monitorTaskHandle;

// Flag debug série (activable/désactivable via BLE)
extern volatile bool audioDebugEnabled;

void monitorAudioTask(void *pvParameters);

// Acquisition I2S (ICS-43434)
esp_err_t audioHwInit();
esp_err_t audioReadHop(int32_t *buf, size_t n_samples, size_t *n_read); // *n_read en octets
float sampleToFloat(int32_t raw);

class AudioDataCallbacks: public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override;
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override;
    void onStatus(NimBLECharacteristic* pCharacteristic, int code) override;
    void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override;
};

#if defined(TESTMODE)
int TestAudio(int m);   // definie uniquement sous TESTMODE (cf. monaudio.cpp)
#endif

// ---- Flags globaux — volatile pour visibilité cross-core ----
extern volatile bool crackCounterStatus; // true = comptage actif
extern volatile bool isCalibrated;       // true = calibration valide disponible
extern volatile bool emptyCalibrated;    // profil machine a vide valide
extern volatile bool maillardCalibrated; // profil grains apres DE valide
extern volatile bool audioStarted;       // true = I2S initialisé correctement
extern volatile bool calibrating;        // true = calibration en cours
extern volatile bool selftestRequested;  // true = auto-test d'acquisition demande

bool saveCalibrationToFile();
bool loadCalibrationFromFile();

// Sauvegarde differee de la calibration : les callbacks BLE (pile NimBLE de
// 4 ko) posent la demande, loop() execute l'ecriture LittleFS.
extern volatile bool saveCalibrationRequested;
void audioPendingSaveService();

// Lancement differe de la calibration : le callback BLE pose la demande,
// loop() execute la barriere d'attente du bus I2S (jusqu'a 3 s) puis cree
// calibrateTask. Jamais sur la tache hote NimBLE, qui doit continuer a
// servir le GATT pendant ce temps.
void audioPendingCalibrationService();

// Echec irrecuperable d'initialisation de monitorAudioTask (FFT ou buffer DMA) :
// loop() ne doit plus tenter de la recreer.
extern volatile bool monitorAudioFatal;

// Abandon d'un auto-test demande mais qui ne pourra jamais tourner (tache
// monitor morte ou impossible a creer) : publie #TILAU_SELFTEST error=<reason>
// et libere selftestRequested, sinon toute calibration resterait refusee.
void audioSelftestAbort(const char *reason);

#endif  // defined(TILAUONAUDIO_H)

#endif  // TILAUONAUDIO_H
