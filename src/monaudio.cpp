// audio processing part
#include "monaudio.h"
#include <Wire.h>
#include "common.h"
#include "esp_timer.h"
#include <algorithm>

//#define TESTMODE

/*
Brochage ICS-43434 ↔ ESP32-S3 (DevKitC-1 N8R8)
ICS-43434   ESP32-S3    Fonction
VDD         3.3 V       Alimentation
GND         GND         Masse
WS (LRCL)   GPIO 5      Word Select
SCK (BCLK)  GPIO 6      Bit Clock
SD (DOUT)   GPIO 4      Données (entrée du S3)
L/R         GND         Canal gauche (Left)
*/

// ---------------------------------------------------------------------------
// Mutex pour crack_counter (partagé entre task audio et callbacks BLE)
// ---------------------------------------------------------------------------
static portMUX_TYPE crack_mux = portMUX_INITIALIZER_UNLOCKED;

// ---------------------------------------------------------------------------
// Variables globales — volatile pour garantir la visibilité cross-core
// (Core 0 : BLE/WiFi stack  |  Core 1 : tâches audio)
// ---------------------------------------------------------------------------
volatile bool crackCounterStatus = false;
volatile bool isCalibrated       = false;
volatile bool emptyCalibrated    = false;
volatile bool maillardCalibrated = false;
volatile bool audioStarted       = false;
volatile bool calibrating        = false;
volatile bool audioDebugEnabled  = false;  // activé via COMMAND_DEBUG_ON
volatile float fluxK = 6.0f;          // thr = base + fluxK * dev (8.0 a trop filtré : effet multiplicatif sur flux_dev en ambiance bruyante/variable)
volatile float confidencePct = CONFIDENCE_DEFAULT;   // seuil utilisateur sur le score composite (0-100%)

// Handshake I2S : true quand monitorAudioTask NE touche PAS le bus I2S.
// calibrate() attend ce flag avant de lancer calibrateTask, pour éviter deux
// i2s_read() concurrents sur le même port (samples scindés -> calibration faussée).
volatile bool monitorAudioIdle = true;
volatile bool selftestRequested = false;
// true tant qu'audioSelfTest() lit le bus : il le detient ~2 s, bien plus que
// l'ancienne barriere de 500 ms de calibrate().
static volatile bool selftestRunning = false;

NimBLECharacteristic *envAudioChar = nullptr;

// ---------------------------------------------------------------------------
// Variables audio
// ---------------------------------------------------------------------------
// FIX #3 : crack_counter passe en int32_t pour ne pas déborder en 1C
// (int16_t débordait à 32767 cracks)
static int32_t crack_counter = 0;

// lastCrackTime globale supprimée — déclarée static dans monitorAudioTask
// (variable orpheline supprimée : samples[], lastCrackTime, noise_std_dev,
//  signal_to_noise_estimated)

// Handles de tâches
TaskHandle_t calibrateTaskHandle = NULL;
TaskHandle_t monitorTaskHandle   = NULL;

// ---------------------------------------------------------------------------
// Structures BLE (packed)
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed))
{
    uint16_t header     = 0x5555;
    int32_t  crack_count;           // FIX #3 : int32_t, cohérent avec crack_counter
    uint8_t  checksum;
    uint16_t footer     = 0xAAAA;
} AudioData;

typedef struct __attribute__((packed))
{
    uint16_t header     = 0x5555;
    int16_t  command;
    uint8_t  checksum;
    uint16_t footer     = 0xAAAA;
} AudioCommand;

// ---------------------------------------------------------------------------
// Filtre biquad
// ---------------------------------------------------------------------------
float filter_state[2] = {0.0f, 0.0f};
float coeffs[5];

// ---------------------------------------------------------------------------
// Buffers FFT — FIX #10 : FFT_SIZE réduit à 256 (défini dans .h)
// static pour éviter l'allocation sur la stack task
// ---------------------------------------------------------------------------
static float fft_input[FFT_SIZE * 2];   // complexe : réel + imaginaire entrelacés
static float fft_window[FFT_SIZE];

esp_err_t initFFT();

// ---------------------------------------------------------------------------
// Stats de calibration
// ---------------------------------------------------------------------------
AudioStats stats = {0, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f};
static AudioStats emptyCalibration = {};
static AudioStats maillardCalibration = {};

enum CalibrationTarget {
    CALIBRATION_EMPTY,
    CALIBRATION_MAILLARD
};

static volatile CalibrationTarget calibrationTarget = CALIBRATION_EMPTY;
volatile bool monitorAudioFatal = false;   // init monitorAudioTask irrecuperable

// Une impulsion largement hors du bruit de machine ne doit pas contaminer le
// profil grains. Les ~96 ms suivantes sont aussi exclues (queue/reverberation).
#define MAILLARD_GUARD_FACTOR 1.5f
#define MAILLARD_GUARD_MAD_K  12.0f
#define MAILLARD_GUARD_FRAMES 12

struct AudioFeatures {
    float flux;
    float crest;
    float rms;
    float centroid;
};

static AudioFeatures computeAudioFeatures(const float *frameRaw,
                                          const float *frameBp,
                                          float *prevMag)
{
    float peak = 0.0f;
    float sumsq = 0.0f;
    for (int i = 0; i < FFT_SIZE; i++) {
        const float amplitude = fabsf(frameBp[i]);
        peak = fmaxf(peak, amplitude);
        sumsq += frameBp[i] * frameBp[i];
        fft_input[2 * i] = frameRaw[i] * fft_window[i];
        fft_input[2 * i + 1] = 0.0f;
    }

    AudioFeatures features = {};
    features.rms = sqrtf(sumsq / FFT_SIZE);
    features.crest = peak / (features.rms + 1e-9f);

    dsps_fft2r_fc32(fft_input, FFT_SIZE);
    dsps_bit_rev_fc32(fft_input, FFT_SIZE);

    const float binHz = (float)I2S_SAMPLE_RATE / FFT_SIZE;
    // prevMag[] ne compte que FFT_SIZE/2 entrees : borner kHi, sinon un
    // retuning de la bande de flux (second crack) ecrit hors tableau.
    const int kLo = (int)(FLUX_F_LO / binHz);
    const int kHi = std::min((int)(FLUX_F_HI / binHz), FFT_SIZE / 2 - 1);
    float magSum = 0.0f;
    float magFreqSum = 0.0f;
    for (int k = kLo; k <= kHi; k++) {
        const float re = fft_input[2 * k];
        const float im = fft_input[2 * k + 1];
        const float mag = sqrtf(re * re + im * im) / FFT_SIZE;
        const float delta = mag - prevMag[k];
        if (delta > 0.0f) features.flux += delta;
        prevMag[k] = mag;
        magSum += mag;
        magFreqSum += mag * ((float)k * binHz);
    }
    features.centroid = (magSum > 1e-9f)
        ? magFreqSum / magSum
        : ((float)(kLo + kHi) * 0.5f * binHz);
    return features;
}

// Plancher absolu du seuil de détection : FLUX_FLOOR_P99_FACTOR x P99 du flux de
// calibration (cf. monaudio.h). Même formule dans tools/replay_detector.py.
static float fluxFloor(const AudioStats &p)
{
    return FLUX_FLOOR_P99_FACTOR * p.flux_p99;
}

// Criteres de validite d'un profil de calibration. Partages entre la fin de
// calibrateTask et la relecture du fichier : un profil accepte en RAM doit etre
// exactement celui que le boot suivant acceptera. Sinon deux calibrations de
// 30 s sont annoncees "validees", sauvees, puis rejetees silencieusement au
// reboot. Un micro muet (samples tout a zero) echoue ici : threshold, flux_p99,
// flux_p999 et rms_median sont nuls.
static bool calibrationProfileUsable(const AudioStats &p)
{
    return isfinite(p.threshold) && p.threshold > 0.0f &&
           isfinite(p.flux_median) && isfinite(p.flux_mad) &&
           isfinite(p.flux_p99) && p.flux_p99 > 0.0f &&     // base du plancher du seuil
           isfinite(p.flux_p999) && p.flux_p999 > 0.0f &&
           isfinite(p.crest_p99) && p.crest_p99 >= CREST_REFERENCE_FLOOR &&
           isfinite(p.rms_median) && p.rms_median > 0.0f;
}

// Les tableaux de features de calibration (maxFrames floats chacun) vont en
// PSRAM quand la carte en a (S3 N8R8) : ils ne sont jamais DMA. Repli sur la
// heap interne sinon (WROOM-32D).
static void *calibAlloc(size_t bytes)
{
#if defined(BOARD_HAS_PSRAM)
    void *psram = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
    if (psram) return psram;
#endif
    return heap_caps_malloc(bytes, MALLOC_CAP_DEFAULT);
}

// Sauvegarde differee : saveCalibrationToFile() (ArduinoJson + effacement/
// ecriture LittleFS) ne doit jamais tourner sur la pile de 4 ko de la tache
// hote NimBLE. Les callbacks BLE posent ce drapeau, loop() fait le travail.
volatile bool saveCalibrationRequested = false;

static void requestCalibrationSave() { saveCalibrationRequested = true; }

void audioPendingSaveService()
{
    if (!saveCalibrationRequested) return;
    saveCalibrationRequested = false;   // avant l'ecriture : une demande
    saveCalibrationToFile();            // concurrente re-arme le drapeau
}

// Mutex des profils de calibration. emptyCalibration/maillardCalibration sont
// ecrits par calibrateTask (coeur 1, priorite 1) et relus par loopTask (meme
// coeur, meme priorite -> preemption possible en plein milieu d'une struct de
// 60 octets) au moment de serialiser le JSON. Sans ce verrou, le fichier peut
// contenir un profil moitie ancien / moitie neuf, et passer quand meme
// calibrationProfileUsable() au boot suivant.
static StaticSemaphore_t calibProfileMutexBuf;
static SemaphoreHandle_t calibProfileMutex = NULL;

// Cree inconditionnellement en tete de loadCalibrationFromFile(), appelee par
// setup() avant toute creation de tache -> pas de course a la creation, quel que
// soit l'etat du fichier (absent, illisible, format obsolete).
static void calibProfileInit()
{
    if (calibProfileMutex == NULL) {
        calibProfileMutex = xSemaphoreCreateMutexStatic(&calibProfileMutexBuf);
    }
}

static void calibProfileLock()
{
    if (calibProfileMutex == NULL) return;   // avant setup() : aucune tache concurrente
    xSemaphoreTake(calibProfileMutex, portMAX_DELAY);
}

static void calibProfileUnlock()
{
    if (calibProfileMutex != NULL) xSemaphoreGive(calibProfileMutex);
}

// Calibration differee : calibrate() est appelee depuis onWrite, donc sur la
// tache hote NimBLE. La barriere d'attente du bus I2S (jusqu'a 3 s) y gelerait
// le traitement GATT -- lectures ENV en attente, et avec deux connexions un
// client peut atteindre le supervision timeout. Le callback pose la demande,
// loop() execute la barriere puis cree la tache.
static volatile bool             calibrationRequestPending = false;
static volatile CalibrationTarget calibrationRequestTarget  = CALIBRATION_EMPTY;
// Detection a reprendre si la calibration demandee echoue avant de demarrer.
// Un STOP recu pendant la barriere I2S le remet a false : sinon la reprise
// annulerait le STOP de l'utilisateur.
static volatile bool             resumeSamplingOnAbort     = false;
// Numero de session de detection, incremente par START uniquement. La tache
// monitor ne repart de zero (FSM FC, plancher EMA, compteur de frames) que si
// le numero a change : une pause suivie d'une reprise (barriere de calibration
// avortee) continue la meme session au lieu d'effacer un FC deja confirme.
static volatile uint32_t         detectionSession          = 0;

static float sortedPercentile(const float *sorted, size_t count, float percentile)
{
    if (count == 0) return 0.0f;
    size_t index = (size_t)ceilf(percentile * count) - 1;
    if (index >= count) index = count - 1;
    return sorted[index];
}

// ---------------------------------------------------------------------------
// Acquisition I2S (ICS-43434)
// ---------------------------------------------------------------------------

float sampleToFloat(int32_t raw)
{
    return (float)(raw >> 8) / 8388608.0f;
}

esp_err_t audioHwInit()
{
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0};

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD_PIN};

    esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("Erreur i2s_driver_install: %d\n", err);
        return err;
    }
    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("Erreur i2s_set_pin: %d\n", err);
        return err;
    }
    err = dsps_biquad_gen_bpf_f32(coeffs, BPF_CENTER_FREQ / float(I2S_SAMPLE_RATE), BPF_Q);
    if (err != ESP_OK) {
        Serial.printf("Erreur dsps_biquad_gen_bpf_f32: %d\n", err);
        return err;
    }
    Serial.println("I2S started, listening to ICS-43434");
    return ESP_OK;
}

esp_err_t audioReadHop(int32_t *buf, size_t n_samples, size_t *n_read)
{
    return i2s_read(I2S_PORT, buf, n_samples * sizeof(int32_t), n_read, portMAX_DELAY);
}


// ---------------------------------------------------------------------------
// Streaming audio brut sur UART (banc de test uniquement, cf. platformio.ini
// env esp32s3_devtest). Frame binaire : magic(4) pcm16(HOP*2, taille FIXE) chk(1).
// Signal streamé AVANT le biquad (xhop) pour ne pas figer le filtre actuel
// dans l'archive — un futur mode replay doit pouvoir retester d'autres BPF.
//
// Magic sur 4 octets et taille fixe (pas de champ longueur) : un champ longueur
// lu sur un faux-positif de magic (collision avec des octets PCM aléatoires)
// ferait attendre le parseur hôte sur une taille arbitraire avant de resynchroniser.
// cf. tools/bench_capture.py (FRAME_MAGIC/PAYLOAD_LEN — à garder synchronisés).
// ---------------------------------------------------------------------------
#if defined(TILAU_AUDIO_STREAM)
// Frame entiere en UN SEUL Serial.write : HardwareSerial prend le verrou UART
// par appel, donc un printf d'un callback BLE (coeur 0) ne peut plus s'inserer
// entre magic, PCM et checksum (frame rejetee -> trou de 8 ms dans la capture).
#define STREAM_MAGIC_LEN   4
#define STREAM_PCM_LEN     (HOP * sizeof(int16_t))
#define STREAM_FRAME_LEN   (STREAM_MAGIC_LEN + STREAM_PCM_LEN + 1)
static void streamAudioHop(const float *xhop, int n)
{
    static uint8_t frame[STREAM_FRAME_LEN] = { 0xFA, 0xCE, 0x17, 0x42 };
    int16_t pcm[HOP];
    for (int i = 0; i < n; i++) {
        float v = xhop[i] * 32767.0f;
        v = fmaxf(-32768.0f, fminf(32767.0f, v));
        pcm[i] = (int16_t)v;
    }
    for (int i = n; i < HOP; i++) pcm[i] = 0;   // pad si lecture courte

    memcpy(frame + STREAM_MAGIC_LEN, pcm, STREAM_PCM_LEN);
    frame[STREAM_FRAME_LEN - 1] =
        calculateChecksum(frame + STREAM_MAGIC_LEN, STREAM_PCM_LEN);
    Serial.write(frame, STREAM_FRAME_LEN);
}
#endif

// ---------------------------------------------------------------------------
// FSM burst detection (cf. ALGORITHM_NOTES.md) — distingue un vrai FC (montée
// statistique de densité de cracks) d'un choc mécanique isolé du tambour.
// ---------------------------------------------------------------------------
volatile FCState  fcState               = FC_STATE_IDLE;
volatile uint32_t fcConfirmedTimestamp  = 0;

static uint32_t fcCrackTimes[FC_BURST_HISTORY];
static uint8_t  fcCrackHead = 0, fcCrackLen = 0;
static uint32_t fcCandidateEnteredAt = 0;

static void fcPushCrackTime(uint32_t t)
{
    fcCrackTimes[fcCrackHead] = t;
    fcCrackHead = (fcCrackHead + 1) % FC_BURST_HISTORY;
    if (fcCrackLen < FC_BURST_HISTORY) fcCrackLen++;
}

// timestamps poussés en ordre croissant -> on peut casser dès qu'on sort de la fenêtre
static int fcCountCracksInWindow(uint32_t now, uint32_t windowMs)
{
    int count = 0;
    for (int i = 0; i < fcCrackLen; i++) {
        uint8_t idx = (fcCrackHead + FC_BURST_HISTORY - 1 - i) % FC_BURST_HISTORY;
        if (now - fcCrackTimes[idx] <= windowMs) count++;
        else break;
    }
    return count;
}

static void fcResetSession()
{
    fcState = FC_STATE_MONITORING;
    fcCrackHead = fcCrackLen = 0;
    fcConfirmedTimestamp = 0;
}

// Appelée à chaque hop ; onset==true déclenche aussi l'enregistrement du timestamp.
static void fcUpdate(uint32_t now, bool onset, uint32_t last_onset_ms)
{
    if (onset) fcPushCrackTime(now);

    switch (fcState) {
    case FC_STATE_IDLE:
        break;  // rien à faire hors session (reset par fcResetSession au démarrage)

    case FC_STATE_MONITORING: {
        const int cracks5s  = fcCountCracksInWindow(now, FC_START_SHORT_WINDOW_MS);
        const int cracks10s = fcCountCracksInWindow(now, FC_START_LONG_WINDOW_MS);
        const bool shortBurst  = cracks5s  >= FC_START_SHORT_COUNT;
        const bool growingMass = cracks10s >= FC_START_LONG_COUNT;

        if (onset && (shortBurst || growingMass)) {
            fcState = FC_STATE_CANDIDATE;
            fcCandidateEnteredAt = now;
            Serial.printf("#TILAU_FC candidate ms=%lu count5s=%d count10s=%d trigger=%s\n",
                          now, cracks5s, cracks10s, shortBurst ? "short" : "mass");
        }
        break;
    }

    case FC_STATE_CANDIDATE: {
        const int cracks5s  = fcCountCracksInWindow(now, FC_CONFIRM_RECENT_WINDOW_MS);
        const int cracks10s = fcCountCracksInWindow(now, FC_CONFIRM_WINDOW_MS);
        const bool enoughMass = cracks10s >= FC_CONFIRM_COUNT;
        const bool stillActive = cracks5s >= FC_CONFIRM_RECENT_COUNT;
        const bool silentTooLong = (now - last_onset_ms) > FC_CANDIDATE_SILENCE_MS;
        const bool candidateTimedOut =
            (now - fcCandidateEnteredAt) > FC_CANDIDATE_TIMEOUT_MS;

        if (enoughMass && stillActive) {
            fcState = FC_STATE_CONFIRMED;
            fcConfirmedTimestamp = fcCandidateEnteredAt;
            Serial.printf("#TILAU_FC confirmed detected_ms=%lu confirmed_ms=%lu count5s=%d count10s=%d\n",
                          fcConfirmedTimestamp, now, cracks5s, cracks10s);
        } else if (silentTooLong || candidateTimedOut) {
            fcState = FC_STATE_MONITORING;
            Serial.printf("#TILAU_FC revert ms=%lu reason=%s count5s=%d count10s=%d\n",
                          now, silentTooLong ? "silence" : "timeout", cracks5s, cracks10s);
        }
        break;
    }

    case FC_STATE_CONFIRMED:
    default:
        break;  // FC déjà déclaré pour cette session ; reste latché jusqu'au stop
    }
}

// ---------------------------------------------------------------------------
// calibrateTask
// ---------------------------------------------------------------------------
void calibrateTask(void *pvParameters)
{
    calibrating = true;
    const CalibrationTarget target = calibrationTarget;
    const char *targetName = target == CALIBRATION_EMPTY ? "EMPTY" : "MAILLARD";
    Serial.printf("\n>>> Calibration DSP %s - maintenir le bruit de fond cible\n", targetName);

    if (initFFT() != ESP_OK) {
        Serial.println("Calibration: FFT init failed, abort.");
        isCalibrated = false; calibrating = false; calibrateTaskHandle = NULL;
        vTaskDelete(NULL); return;
    }

    // 3 tableaux de features seulement (etait 4 x ~15 ko) : le scratch de la MAD
    // reutilise crestValues une fois ses percentiles extraits, et la garde
    // MAILLARD alloue le sien le temps de son calcul.
    const size_t maxFrames = ((size_t)CALIB_TIME_MS * I2S_SAMPLE_RATE) / (1000 * HOP) + 16;
    int32_t *raw = (int32_t *)heap_caps_malloc(HOP * sizeof(int32_t), MALLOC_CAP_DMA);
    float *fluxValues = (float *)calibAlloc(maxFrames * sizeof(float));
    float *crestValues = (float *)calibAlloc(maxFrames * sizeof(float));
    float *rmsValues = (float *)calibAlloc(maxFrames * sizeof(float));
    if (!raw || !fluxValues || !crestValues || !rmsValues) {
        Serial.printf("Calibration: heap alloc failed (%u frames, %u o/tableau), abort.\n",
                      (unsigned)maxFrames, (unsigned)(maxFrames * sizeof(float)));
        heap_caps_free(raw); heap_caps_free(fluxValues); heap_caps_free(crestValues);
        heap_caps_free(rmsValues);
        isCalibrated = false; calibrating = false; calibrateTaskHandle = NULL;
        vTaskDelete(NULL); return;
    }

    float frameRaw[FFT_SIZE] = {};
    float frameBp[FFT_SIZE] = {};
    float prevMag[FFT_SIZE / 2] = {};
    float calibFilterState[2] = {};
    float xhop[HOP], bhop[HOP];
    size_t featureCount = 0;
    size_t rejectedTransientCount = 0;
    uint32_t framesSeen = 0;
    float peakDuringCalib = 0.0f;
    const uint32_t start = millis();
    uint32_t lastUiUpdate = 0;

    while ((millis() - start) < CALIB_TIME_MS && featureCount < maxFrames) {
        size_t bytesRead = 0;
        if (audioReadHop(raw, HOP, &bytesRead) != ESP_OK || bytesRead == 0) {
            vTaskDelay(1);   // sinon le chemin d'erreur tourne en busy-spin 30 s
            continue;
        }
        const int n = bytesRead / sizeof(int32_t);

        memmove(frameRaw, frameRaw + HOP, (FFT_SIZE - HOP) * sizeof(float));
        memmove(frameBp, frameBp + HOP, (FFT_SIZE - HOP) * sizeof(float));
        for (int i = 0; i < n; i++) xhop[i] = sampleToFloat(raw[i]);
#if defined(TILAU_AUDIO_STREAM)
        // Banc de test : le flux brut sort aussi pendant la calibration. Deux
        // raisons : (1) archiver l'audio du profil de bruit, (2) permettre a
        // bench_capture.py de valider toute la chaine d'acquisition des les
        // premieres secondes du profil EMPTY -- avant qu'une torrefaction ne
        // soit engagee pour rien.
        streamAudioHop(xhop, n);
#endif
        dsps_biquad_f32_ansi(xhop, bhop, n, coeffs, calibFilterState);
        for (int i = 0; i < HOP; i++) {
            frameRaw[FFT_SIZE - HOP + i] = (i < n) ? xhop[i] : 0.0f;
            frameBp[FFT_SIZE - HOP + i] = (i < n) ? bhop[i] : 0.0f;
            if (i < n) peakDuringCalib = fmaxf(peakDuringCalib, fabsf(bhop[i]));
        }

        AudioFeatures features = computeAudioFeatures(frameRaw, frameBp, prevMag);
        if (framesSeen++ >= WARMUP_FRAMES) {
            fluxValues[featureCount] = features.flux;
            crestValues[featureCount] = features.crest;
            rmsValues[featureCount] = features.rms;
            featureCount++;
        }

        const uint32_t now = millis();
        if (now - lastUiUpdate >= 500) {
            lastUiUpdate = now;
            Serial.printf("\rCalibration DSP %s: %lu%% frames=%u rejected=%u flux=%.5f crest=%.2f rms=%.5f",
                          targetName,
                          (unsigned long)((now - start) * 100 / CALIB_TIME_MS),
                          (unsigned)featureCount, (unsigned)rejectedTransientCount,
                          features.flux, features.crest, features.rms);
        }
    }
    Serial.println();
    heap_caps_free(raw);

    // Le bruit des grains peut être naturellement bien supérieur au tambour vide.
    // On estime donc d'abord le centre robuste du profil chargé, puis on élimine
    // uniquement les excursions dépassant à la fois ce fond chargé et la garde EMPTY.
    // Le compactage conserve l'alignement flux/crest/RMS avant les tris par feature.
    if (target == CALIBRATION_MAILLARD && featureCount >= 100) {
        // Scratch temporaire (libere aussitot) : la garde a besoin d'une copie
        // triable du flux alors que fluxValues doit rester dans l'ordre temporel.
        float *scratch = (float *)calibAlloc(featureCount * sizeof(float));
        float loadedMedian = 0.0f, loadedMad = 0.0f;
        bool  haveLoadedStats = false;
        if (scratch) {
            for (size_t i = 0; i < featureCount; i++) scratch[i] = fluxValues[i];
            std::sort(scratch, scratch + featureCount);
            loadedMedian = sortedPercentile(scratch, featureCount, 0.5f);
            for (size_t i = 0; i < featureCount; i++)
                scratch[i] = fabsf(fluxValues[i] - loadedMedian);
            std::sort(scratch, scratch + featureCount);
            loadedMad = sortedPercentile(scratch, featureCount, 0.5f);
            heap_caps_free(scratch);
            haveLoadedStats = true;
        } else {
            Serial.println("Calibration MAILLARD guard: scratch alloc failed -> garde EMPTY seule");
        }
        // Meme formule que le seuil runtime (flux_base + fluxK * flux_dev), donc
        // meme unite de dispersion : MAD convertie en ecart absolu moyen.
        const float emptyAdaptive = emptyCalibration.flux_median
                                  + fluxK * emptyCalibration.flux_mad * MAD_TO_MEANDEV;
        const float emptyLimit = MAILLARD_GUARD_FACTOR
                               * fmaxf(emptyCalibration.flux_p999, emptyAdaptive);
        // Sans scratch, pas de borne "fond charge" : on retombe sur la garde EMPTY.
        const float loadedLimit = haveLoadedStats
                                ? (loadedMedian + MAILLARD_GUARD_MAD_K * loadedMad)
                                : 0.0f;
        const float transientLimit = fmaxf(emptyLimit, loadedLimit);

        size_t accepted = 0;
        uint16_t guardFramesRemaining = 0;
        for (size_t i = 0; i < featureCount; i++) {
            if (fluxValues[i] > transientLimit)
                guardFramesRemaining = MAILLARD_GUARD_FRAMES;
            if (guardFramesRemaining > 0) {
                guardFramesRemaining--;
                rejectedTransientCount++;
                continue;
            }
            fluxValues[accepted] = fluxValues[i];
            crestValues[accepted] = crestValues[i];
            rmsValues[accepted] = rmsValues[i];
            accepted++;
        }
        Serial.printf("Calibration MAILLARD guard: loaded_med=%.6f loaded_mad=%.6f limit=%.6f rejected=%u/%u\n",
                      loadedMedian, loadedMad, transientLimit,
                      (unsigned)rejectedTransientCount, (unsigned)featureCount);
        featureCount = accepted;
    }

    if (featureCount < 100) {
        // Meme traitement que le profil rejete plus bas : dire POURQUOI (zero
        // frame lue = panne d'acquisition, pas un probleme de niveau) et
        // persister l'invalidation, sinon un reboot restaurerait l'ancien
        // profil comme encore valide.
        Serial.printf("Calibration %s ECHOUEE: %u frames exploitables (<100) "
                      "sur %u hops lus, %u rejetees par la garde transitoire\n",
                      targetName, (unsigned)featureCount, (unsigned)framesSeen,
                      (unsigned)rejectedTransientCount);
        if (framesSeen == 0)
            Serial.println("  -> aucun hop lu : chaine d'acquisition muette, "
                           "lancer COMMAND_AUDIO_SELFTEST (0x0007).");
        else
            Serial.println("  -> verifier la chaine micro avec COMMAND_AUDIO_SELFTEST (0x0007).");
        heap_caps_free(fluxValues); heap_caps_free(crestValues);
        heap_caps_free(rmsValues);
        calibProfileLock();
        if (target == CALIBRATION_EMPTY) emptyCalibration = AudioStats{};
        else                             maillardCalibration = AudioStats{};
        calibProfileUnlock();
        if (target == CALIBRATION_EMPTY) emptyCalibrated = false;
        maillardCalibrated = false;
        isCalibrated = false;
        requestCalibrationSave();
        calibrating = false; calibrateTaskHandle = NULL;
        vTaskDelete(NULL); return;
    }

    std::sort(fluxValues, fluxValues + featureCount);
    std::sort(crestValues, crestValues + featureCount);
    std::sort(rmsValues, rmsValues + featureCount);
    AudioStats result = {};
    result.flux_median = sortedPercentile(fluxValues, featureCount, 0.5f);
    result.flux_p99 = sortedPercentile(fluxValues, featureCount, 0.99f);
    result.flux_p999 = sortedPercentile(fluxValues, featureCount, 0.999f);
    result.crest_p99 = sortedPercentile(crestValues, featureCount, 0.99f);
    result.crest_p999 = sortedPercentile(crestValues, featureCount, 0.999f);
    result.rms_median = sortedPercentile(rmsValues, featureCount, 0.5f);
    result.rms_p99 = sortedPercentile(rmsValues, featureCount, 0.99f);
    // crestValues a livre ses percentiles : reutilise comme scratch pour la MAD.
    float *madScratch = crestValues;
    for (size_t i = 0; i < featureCount; i++)
        madScratch[i] = fabsf(fluxValues[i] - result.flux_median);
    std::sort(madScratch, madScratch + featureCount);
    result.flux_mad = sortedPercentile(madScratch, featureCount, 0.5f);

    // Champs historiques conservés pour compatibilité de rapport/protocole interne.
    result.peak_amplitude = peakDuringCalib;
    result.noise_floor_rms = result.rms_median;
    result.noise_p99 = result.rms_p99;
    result.noise_p999 = sortedPercentile(rmsValues, featureCount, 0.999f);
    result.threshold = fluxFloor(result);   // plancher réellement appliqué en détection
    result.snr_db = 20.0f * log10f(0.2f / fmaxf(result.rms_median, 1e-9f));

    heap_caps_free(fluxValues); heap_caps_free(crestValues);
    heap_caps_free(rmsValues);

    Serial.printf("========== CALIBRATION DSP %s REPORT ==========\n", targetName);
    Serial.printf("Feature frames      : %u\n", (unsigned)featureCount);
    Serial.printf("Frames transitoires : %u\n", (unsigned)rejectedTransientCount);
    Serial.printf("Flux median / MAD   : %.6f / %.6f\n", result.flux_median, result.flux_mad);
    Serial.printf("Flux P99 / P99.9    : %.6f / %.6f\n", result.flux_p99, result.flux_p999);
    Serial.printf("Crest P99 / P99.9   : %.2f / %.2f\n", result.crest_p99, result.crest_p999);
    Serial.printf("RMS median / P99    : %.6f / %.6f\n", result.rms_median, result.rms_p99);
    Serial.printf("Pic filtre max      : %.6f\n", result.peak_amplitude);
    Serial.printf("Seuil flux calibre  : %.6f\n", result.threshold);
    Serial.println("============================================\n");

    // Ne jamais annoncer "validee" un profil que readCalibrationProfile() rejettera
    // au prochain boot (micro muet, saturation, division degeneree).
    if (!calibrationProfileUsable(result)) {
        Serial.printf("Calibration %s REJETEE: profil inexploitable "
                      "(threshold=%.6f flux_p999=%.6f crest_p99=%.2f rms_median=%.6f)\n",
                      targetName, result.threshold, result.flux_p999,
                      result.crest_p99, result.rms_median);
        Serial.println("  -> verifier la chaine micro avec COMMAND_AUDIO_SELFTEST (0x0007).");
        calibProfileLock();
        if (target == CALIBRATION_EMPTY) emptyCalibration = AudioStats{};
        else                             maillardCalibration = AudioStats{};
        calibProfileUnlock();
        if (target == CALIBRATION_EMPTY) emptyCalibrated = false;
        maillardCalibrated = false;
        isCalibrated = false;
        requestCalibrationSave();   // une seule tache ecrit le fichier : loop()
        calibrating = false; calibrateTaskHandle = NULL;
        vTaskDelete(NULL); return;
    }

    portENTER_CRITICAL(&crack_mux);
    crack_counter = 0;
    portEXIT_CRITICAL(&crack_mux);
    calibProfileLock();
    if (target == CALIBRATION_EMPTY) {
        emptyCalibration = result;
        calibProfileUnlock();
        emptyCalibrated = true;
        maillardCalibrated = false;
        isCalibrated = false;
        Serial.println("Calibration EMPTY validée; calibration MAILLARD requise avant START.");
    } else {
        maillardCalibration = result;
        stats = maillardCalibration;
        calibProfileUnlock();
        maillardCalibrated = true;
        stats.crack_count = 0;
        isCalibrated = emptyCalibrated && maillardCalibrated;
        Serial.println("Calibration MAILLARD validée; détecteur prêt.");
    }
    requestCalibrationSave();   // ecriture LittleFS serialisee dans loop()

    calibrating         = false;
    calibrateTaskHandle = NULL;
    vTaskDelete(NULL);
}

static void startCalibrationNow(CalibrationTarget target);

// ---------------------------------------------------------------------------
// calibrate() — point d'entrée, appelé depuis callback BLE (onWrite)
// FIX #5 : on utilise un simple flag volatile plutôt que portENTER_CRITICAL
// depuis un contexte potentiellement protégé du stack NimBLE
// ---------------------------------------------------------------------------
// Les refus evidents sont traites ICI, avant toute modification d'etat : une
// demande refusee ne doit ni arreter la detection ni toucher au compteur.
void calibrate(CalibrationTarget target)
{
    if (calibrationRequestPending) {
        Serial.println("Audio process - Calibration request already pending!");
        return;
    }
    if (calibrating) {
        Serial.println("Audio process - Calibration already running!");
        return;
    }
    if (selftestRequested || selftestRunning) {
        Serial.println("Audio process - Calibration refused: audio selftest in progress");
        return;
    }
    if (target == CALIBRATION_MAILLARD && !emptyCalibrated) {
        Serial.println("Audio process - MAILLARD calibration refused: EMPTY profile missing");
        return;
    }
    calibrationRequestTarget  = target;
    calibrationRequestPending = true;
}

// Execute par loop() : contient la barriere bloquante sur le bus I2S, qui ne
// doit jamais tourner sur la tache hote NimBLE.
// calibrationRequestPending reste vrai PENDANT startCalibrationNow() : START et
// SELFTEST sont refuses sur ce drapeau, donc rien ne peut relancer la lecture
// du bus pendant la barriere d'attente.
void audioPendingCalibrationService()
{
    if (!calibrationRequestPending) return;
    startCalibrationNow(calibrationRequestTarget);
    calibrationRequestPending = false;
}

static void startCalibrationNow(CalibrationTarget target)
{
    if (calibrating) {
        Serial.println("Audio process - Calibration already running!");
        return;
    }
    if (target == CALIBRATION_MAILLARD && !emptyCalibrated) {
        Serial.println("Audio process - MAILLARD calibration refused: EMPTY profile missing");
        return;
    }
    if (selftestRequested || selftestRunning) {
        Serial.println("Audio process - Calibration refused: audio selftest in progress");
        return;
    }

    // Stopper la détection et ATTENDRE que monitorAudioTask relâche le bus I2S
    // avant d'en lancer un second lecteur (calibrateTask). Sans cette barrière, les
    // deux i2s_read() se partagent les samples -> stats de calibration corrompues.
    // Timeout > duree d'un auto-test (2 s) : sinon la barriere expirait pendant
    // un selftest et deux i2s_read() concurrents corrompaient la calibration.
    resumeSamplingOnAbort = crackCounterStatus;
    crackCounterStatus = false;
    if (monitorTaskHandle != NULL) {
        const uint32_t t0 = millis();
        while (!monitorAudioIdle && (millis() - t0) < 3000) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        if (!monitorAudioIdle) {
            // Refus plutot qu'avertissement : une calibration lue a deux tetes
            // produit des stats fausses, donc un detecteur faux tout le roast.
            // La detection en cours reprend telle quelle (compteur intact) :
            // une calibration refusee ne doit pas couper le comptage en silence.
            const bool resume = resumeSamplingOnAbort;
            crackCounterStatus = resume;
            Serial.printf("Audio process - Calibration ABORTED: monitor still holds the audio bus%s\n",
                          resume ? " (detection resumed)" : "");
            return;
        }
    }

    // Etat d'avant, restaure si la tache ne peut pas etre creee : un echec
    // d'allocation ne doit ni invalider des profils valides ni couper la detection.
    const bool prevEmpty    = emptyCalibrated;
    const bool prevMaillard = maillardCalibrated;
    portENTER_CRITICAL(&crack_mux);
    const int32_t prevCounter = crack_counter;
    portEXIT_CRITICAL(&crack_mux);
    const uint32_t prevStatsCount = stats.crack_count;

    calibrating = true;
    calibrationTarget = target;
    if (target == CALIBRATION_EMPTY) {
        emptyCalibrated = false;
        maillardCalibrated = false;
    } else {
        maillardCalibrated = false;
    }
    isCalibrated = false;   // sinon un START_SAMPLING prématuré passe la garde `if (isCalibrated)`
                             // avec la valeur d'une calibration PRÉCÉDENTE, et monitorAudioTask
                             // se remet à lire le bus audio en même temps que calibrateTask.
    // Persister immédiatement l'invalidation : une coupure pendant la nouvelle
    // mesure ne doit jamais restaurer au boot l'ancienne paire comme encore valide.
    requestCalibrationSave();

    // Réinitialiser le compteur de façon thread-safe
    portENTER_CRITICAL(&crack_mux);
    crack_counter = -1;
    portEXIT_CRITICAL(&crack_mux);
    stats.crack_count = 0;   // sinon STATUS affiche encore le compteur de la session précédente

    Serial.printf("Audio process - Starting %s Calibration Task...\n",
                  target == CALIBRATION_EMPTY ? "EMPTY" : "MAILLARD");

    BaseType_t res = xTaskCreatePinnedToCore(
        calibrateTask,
        "CalibrateTask",
        12288,
        NULL,
        1,
        &calibrateTaskHandle,
        1
    );
    if (res != pdPASS) {
        emptyCalibrated    = prevEmpty;
        maillardCalibrated = prevMaillard;
        isCalibrated       = prevEmpty && prevMaillard;
        requestCalibrationSave();   // annule l'invalidation persistee plus haut
        portENTER_CRITICAL(&crack_mux);
        crack_counter = prevCounter;
        portEXIT_CRITICAL(&crack_mux);
        stats.crack_count = prevStatsCount;
        calibrating = false;
        const bool resume = resumeSamplingOnAbort && isCalibrated;
        crackCounterStatus = resume;
        Serial.printf("Failed to create calibrate task! Previous calibration kept%s\n",
                      resume ? ", detection resumed" : "");
    }
}

// ---------------------------------------------------------------------------
// initFFT — FIX #3 : utilise FFT_SIZE (256) au lieu de BUFFER_SIZE (1024)
// ---------------------------------------------------------------------------
esp_err_t initFFT()
{
    static bool initialized = false;
    if (initialized) return ESP_OK;
    esp_err_t err = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (err != ESP_OK) {
        Serial.printf("initFFT: dsps_fft2r_init_fc32 failed: %d\n", err);
        return err;
    }
    dsps_wind_hann_f32(fft_window, FFT_SIZE);
    initialized = true;
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// audioSelfTest — auto-test d'acquisition (COMMAND_AUDIO_SELFTEST).
//
// Lit ~2 s d'I2S sans rien calibrer ni compter, streame les hops (build banc)
// et publie une ligne machine #TILAU_SELFTEST. Objectif : permettre a l'hote
// de prouver toute la chaine micro -> DSP -> UART -> WAV avant qu'une
// torrefaction ne soit engagee. Un micro muet donne rms=0/peak=0 et se voit
// donc immediatement, ce qu'aucun compteur BLE ne revele.
// Appelee depuis monitorAudioTask (branche idle) : c'est la seule tache
// autorisee a lire le bus I2S hors calibration.
// ---------------------------------------------------------------------------
void audioSelftestAbort(const char *reason)
{
    if (!selftestRequested) return;
    selftestRequested = false;
    Serial.printf("#TILAU_SELFTEST error=%s\n", reason);
}

static void audioSelfTest()
{
    const uint32_t SELFTEST_MS = 2000;
    int32_t *raw = (int32_t *)heap_caps_malloc(HOP * sizeof(int32_t), MALLOC_CAP_DMA);
    if (!raw) {
        Serial.println("#TILAU_SELFTEST error=alloc");
        return;
    }

    i2s_zero_dma_buffer(I2S_PORT);   // repartir sur de l'audio live, pas un backlog

    float    peak = 0.0f, sumsq = 0.0f;
    uint32_t frames = 0, samples = 0;
    const uint32_t start = millis();

    while ((millis() - start) < SELFTEST_MS) {
        size_t bytesRead = 0;
        if (audioReadHop(raw, HOP, &bytesRead) != ESP_OK || bytesRead == 0) {
            vTaskDelay(1);
            continue;
        }
        const int n = bytesRead / sizeof(int32_t);
        float xhop[HOP];
        for (int i = 0; i < n; i++) {
            xhop[i] = sampleToFloat(raw[i]);
            peak = fmaxf(peak, fabsf(xhop[i]));
            sumsq += xhop[i] * xhop[i];
        }
        samples += n;
#if defined(TILAU_AUDIO_STREAM)
        streamAudioHop(xhop, n);
#endif
        frames++;
    }

    heap_caps_free(raw);
    const float rms = samples ? sqrtf(sumsq / samples) : 0.0f;
    Serial.printf("#TILAU_SELFTEST frames=%u samples=%u rms=%.6f peak=%.6f ms=%lu\n",
                  (unsigned)frames, (unsigned)samples, rms, peak,
                  (unsigned long)(millis() - start));
}

// ---------------------------------------------------------------------------
// monitorAudioTask
// ---------------------------------------------------------------------------
void monitorAudioTask(void *pvParameters)
{
    static float frame_raw[FFT_SIZE];        // sliding window (unfiltered) for FFT
    static float frame_bp [FFT_SIZE];        // sliding window (band-passed) for crest
    static float prev_mag [FFT_SIZE/2];      // previous magnitude spectrum (flux)

    // monitorAudioFatal : sans lui, loop() recree immediatement la tache (ses
    // conditions n'ont pas change) -> creation/mort d'une pile de 12 ko en
    // boucle, UART sature et heap thrashee jusqu'au watchdog.
    if (initFFT() != ESP_OK) {
        Serial.println("monitorAudioTask: FFT init failed, task aborted.");
        monitorAudioFatal = true;
        audioSelftestAbort("monitor_fatal");
        monitorTaskHandle = NULL;
        vTaskDelete(NULL);
        return;
    }

    int32_t *raw = (int32_t *)heap_caps_malloc(HOP * sizeof(int32_t), MALLOC_CAP_DMA);
    if (!raw) {
        Serial.println("monitorAudioTask: heap alloc failed, task aborted.");
        monitorAudioFatal = true;
        audioSelftestAbort("monitor_fatal");
        monitorTaskHandle = NULL;
        vTaskDelete(NULL);
        return;
    }

    float    flux_base = 0.0f, flux_dev = 0.0f, prev_flux = 0.0f;
    uint32_t above_thr_since = 0;      // debut du depassement continu du seuil
    bool     above_thr_run   = false;  // depassement en cours (garde-fou du gel)
    uint32_t last_onset_ms = 0, t_start = 0, last_status = 0;
    bool     was_counting = false;
    uint32_t frames_seen = 0;
    uint32_t session = 0;              // detectionSession de la session en cours
    FCState  paused_fc_state = FC_STATE_IDLE;

    // Infinite task: idle when not counting, never exit (else loop() respawns it).
    while (1)
    {
        // ---- OFF state ----
        if (!crackCounterStatus) {
            if (selftestRequested) {
                if (calibrating) {
                    // calibrateTask detient le bus I2S : on ne le lui dispute pas.
                    Serial.println("#TILAU_SELFTEST error=calibrating");
                } else {
                    monitorAudioIdle = false;   // reserve le bus le temps du test
                    selftestRunning  = true;
                    audioSelfTest();
                    selftestRunning  = false;
                }
                selftestRequested = false;
            }
            monitorAudioIdle = true;   // I2S bus released : calibrate() may proceed
            if (was_counting) {
                Serial.printf("#TILAU_AUDIO stop ms=%lu cracks=%u\n", millis(), stats.crack_count);
                was_counting = false;
                paused_fc_state = fcState;   // repris si la meme session redemarre
                fcState = FC_STATE_IDLE;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // Active : on s'apprête à lire le bus I2S — interdit la calibration concurrente
        monitorAudioIdle = false;

        // ---- ON edge: start a fresh detection session ----
        if (!was_counting && session == detectionSession) {
            // Reprise de la MEME session (pause sans STOP/START) : fenetres et
            // filtre repartent de zero (trou dans le signal), warmup refait,
            // mais FSM FC, plancher EMA et dernier onset sont conserves.
            memset(frame_raw, 0, sizeof(frame_raw));
            memset(frame_bp , 0, sizeof(frame_bp));
            memset(prev_mag , 0, sizeof(prev_mag));
            filter_state[0] = filter_state[1] = 0.0f;
            above_thr_since = 0;
            above_thr_run   = false;
            prev_flux   = 0.0f;
            frames_seen = 0;
            fcState     = paused_fc_state;
            i2s_zero_dma_buffer(I2S_PORT);
            Serial.printf("#TILAU_AUDIO resume ms=%lu cracks=%u fc=%d\n",
                          millis(), stats.crack_count, (int)fcState);
            was_counting = true;
        }
        if (!was_counting) {
            session = detectionSession;
            memset(frame_raw, 0, sizeof(frame_raw));
            memset(frame_bp , 0, sizeof(frame_bp));
            memset(prev_mag , 0, sizeof(prev_mag));
            filter_state[0] = filter_state[1] = 0.0f;
            flux_base = stats.flux_median;
            flux_dev  = fmaxf(stats.flux_mad * MAD_TO_MEANDEV, 1e-9f);
            above_thr_since = 0;
            above_thr_run   = false;
            prev_flux = 0.0f;
            last_onset_ms = 0;
            frames_seen   = 0;
            fcResetSession();
            i2s_zero_dma_buffer(I2S_PORT);   // drop stale backlog, start on live audio
            t_start = last_status = millis();
            Serial.printf("#TILAU_AUDIO start ms=%lu fs=%d fft=%d hop=%d band=%.0f-%.0f alpha=%.3f K=%.1f cal_flux=%.5f cal_crest=%.2f refr=%d\n",
                          t_start, I2S_SAMPLE_RATE, FFT_SIZE, HOP, FLUX_F_LO, FLUX_F_HI,
                          FLUX_ALPHA, fluxK, fluxFloor(stats),
                          fmaxf(stats.crest_p99, CREST_REFERENCE_FLOOR), REFRACTORY_MS);
            was_counting = true;
        }

        // ---- one hop (128 samples ≈ 8 ms) ----
        size_t bytesRead = 0;
        if (audioReadHop(raw, HOP, &bytesRead) != ESP_OK || bytesRead == 0) {
            vTaskDelay(1); continue;
        }
        int n = bytesRead / sizeof(int32_t);

        // shift sliding windows left by HOP
        memmove(frame_raw, frame_raw + HOP, (FFT_SIZE - HOP) * sizeof(float));
        memmove(frame_bp , frame_bp  + HOP, (FFT_SIZE - HOP) * sizeof(float));

        // new hop: convert + CONTINUOUS band-pass (state preserved)
        float xhop[HOP], bhop[HOP];
        for (int i = 0; i < n; i++) xhop[i] = sampleToFloat(raw[i]);
#if defined(TILAU_AUDIO_STREAM)
        streamAudioHop(xhop, n);
#endif
        dsps_biquad_f32_ansi(xhop, bhop, n, coeffs, filter_state);
        for (int i = 0; i < HOP; i++) {
            frame_raw[FFT_SIZE - HOP + i] = (i < n) ? xhop[i] : 0.0f;
            frame_bp [FFT_SIZE - HOP + i] = (i < n) ? bhop[i] : 0.0f;
        }

        uint32_t now = millis();

        {
            static uint32_t last_raw_log = 0;
            if (audioDebugEnabled && (now - last_raw_log) >= 1000) {
                last_raw_log = now;
                int32_t rmin = INT32_MAX, rmax = INT32_MIN;
                for (int i = 0; i < n; i++) { if (raw[i] < rmin) rmin = raw[i]; if (raw[i] > rmax) rmax = raw[i]; }
                Serial.printf("RAW min=%ld max=%ld\n", (long)rmin, (long)rmax);
            }
        }

        AudioFeatures features = computeAudioFeatures(frame_raw, frame_bp, prev_mag);

        // Plancher absolu issu de la calibration MAILLARD (fluxFloor) ; l'EMA suit
        // les changements de bruit pendant la torréfaction. À 30-40 cm, c'est le
        // plancher qui fixe le seuil presque tout le temps.
        float adaptive_thr = flux_base + fluxK * flux_dev;
        float thr = fmaxf(adaptive_thr, fluxFloor(stats));
        float crest_reference = fmaxf(stats.crest_p99, CREST_REFERENCE_FLOOR);
        // Chaque sous-score sature en douceur vers 1.0 ; il vaut 0.5 exactement au
        // seuil de flux et à la référence de crête mesurée pendant la calibration.
        // Rapports au seuil calibre, satures en r^2/(r^2+1) : 0.5 exactement au
        // seuil, ~0.05 au niveau du bruit de fond. Sans ce durcissement le score
        // composite gardait un plancher de ~0.35 sur du silence (la crete et le
        // centroide ne descendent jamais a 0 dans la bande de flux) : toute
        // confiance sous ~40% validait alors chaque hop.
        const float r_flux   = features.flux / (thr + 1e-9f);
        const float r_crest  = features.crest / (crest_reference + 1e-9f);
        float score_flux     = (r_flux * r_flux) / (r_flux * r_flux + 1.0f);
        float score_crest    = (r_crest * r_crest) / (r_crest * r_crest + 1.0f);
        float centroid_dist  = fabsf(features.centroid - BPF_CENTER_FREQ);
        float score_centroid = fmaxf(0.0f, 1.0f - centroid_dist / CENTROID_TOLERANCE_HZ);
        float score = SCORE_W_FLUX*score_flux + SCORE_W_CREST*score_crest + SCORE_W_CENTROID*score_centroid;
        float score_threshold = confidencePct / 100.0f;

        bool warming = (frames_seen < WARMUP_FRAMES);
        // Garde absolue CONSERVEE en plus du score : le score seul, avec une
        // confiance basse, ne peut pas distinguer un hop de bruit de fond d'un
        // crack (plancher du sous-score centroide). Le score ne fait que trier
        // ce qui a deja franchi le seuil de flux calibre.
        bool onset = (!warming) && (features.flux > thr) && (score > score_threshold)
                     && (features.flux > prev_flux)
                     && ((now - last_onset_ms) > REFRACTORY_MS);
        if (onset) {
            portENTER_CRITICAL(&crack_mux);     // BLE GETCRACKCOUNTER reads crack_counter
            crack_counter++;
            int32_t val = crack_counter;
            portEXIT_CRITICAL(&crack_mux);
            stats.crack_count = (uint32_t)val;  // mirror for logs
            last_onset_ms = now;
        }
        fcUpdate(now, onset, last_onset_ms);
        // Geler le plancher des que le flux depasse le seuil, sans attendre la
        // decision finale : en plein FC la plupart des hops energiques sont
        // rejetes (score juste sous le seuil, ou refractaire de 150 ms actif)
        // et alimenteraient l'EMA (tau ~0.4 s) -> le detecteur se desensibilise
        // pendant l'evenement meme qu'il doit compter.
        // Garde-fou : un depassement continu de plus de FLUX_FREEZE_MAX_MS est
        // une montee du bruit de fond, pas une salve -> degeler (cf. monaudio.h).
        const bool below_thr = (features.flux <= thr);
        if (below_thr) {
            above_thr_since = 0;
            above_thr_run   = false;
        } else if (!above_thr_run) {
            above_thr_since = now;
            above_thr_run   = true;
        }
        if (below_thr || (now - above_thr_since) > FLUX_FREEZE_MAX_MS) {
            flux_base += FLUX_ALPHA * (features.flux - flux_base);
            flux_dev  += FLUX_ALPHA * (fabsf(features.flux - flux_base) - flux_dev);
        }
        prev_flux = features.flux;
        frames_seen++;

        // per-frame machine log (debug gated) -- ajout score/centroid vs avant
        if (audioDebugEnabled) {
            Serial.printf("A %lu %.5f %.5f %.5f %.2f %.5f %.0f %.2f %d\n",
                          now - t_start, features.flux, flux_base, thr, features.crest,
                          features.rms, features.centroid, score, onset ? 1 : 0);
        }
        // lightweight human STATUS every 2 s (not debug-gated)
        if (now - last_status >= 2000) {
            last_status = now;
            Serial.printf("STATUS t=%lu base=%.4f thr=%.4f crest=%.2f score=%.2f conf=%.0f cracks=%u fc=%d\n",
                          now - t_start, flux_base, thr, features.crest, score,
                          confidencePct, stats.crack_count, (int)fcState);
        }
    }
    // unreachable
}
// ---------------------------------------------------------------------------
// Callbacks BLE
// ---------------------------------------------------------------------------
void AudioDataCallbacks::onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo)
{
    AudioData data;

    portENTER_CRITICAL(&crack_mux);
    int32_t val = crack_counter;
    portEXIT_CRITICAL(&crack_mux);

    data.crack_count = (!isCalibrated || !audioStarted) ? -1 : val;

    const uint8_t *payloadPtr = reinterpret_cast<const uint8_t *>(&data) + sizeof(data.header);
    size_t         payloadLen = sizeof(data.crack_count);
    data.checksum = calculateChecksum(payloadPtr, payloadLen);

    pCharacteristic->setValue(reinterpret_cast<uint8_t *>(&data), sizeof(AudioData));
    Serial.printf("on read received - crack counter: %d crack(s)\n", (int)data.crack_count);
}

void AudioDataCallbacks::onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo)
{
    std::string rxData = pCharacteristic->getValue();
    if (rxData.length() != sizeof(AudioCommand)) {
        Serial.printf("WRITE Error: size mismatch recv=%d expected=%d\n",
                      (int)rxData.length(), (int)sizeof(AudioCommand));
        return;
    }

    AudioCommand cmd;
    memcpy(&cmd, rxData.data(), sizeof(AudioCommand));

    const uint8_t *payloadPtr = reinterpret_cast<const uint8_t *>(&cmd) + sizeof(cmd.header);
    size_t         payloadLen = sizeof(cmd.command);
    uint8_t        calcChk   = calculateChecksum(payloadPtr, payloadLen);

    if (cmd.header != 0x5555 || cmd.footer != 0xAAAA) {
        Serial.printf("WRITE Error: bad header 0x%04X / footer 0x%04X\n",
                      cmd.header, cmd.footer);
        return;
    }
    if (cmd.checksum != calcChk) {
        Serial.printf("WRITE Error: checksum mismatch recv=0x%02X calc=0x%02X\n",
                      cmd.checksum, calcChk);
        return;
    }

    // Réglage absolu (idempotent) de la confiance utilisateur : 0x0100 + pct(0-100).
    // Volontairement pas de commande "relative" (×1.1, ×1.5) — c'est ce qui avait
    // rendu l'ancien réglage ingérable côté Artisan (impossible de connaître l'état
    // courant sans le retenir côté client). Ici une seule valeur fixe le seuil.
    if (cmd.command >= COMMAND_SET_CONFIDENCE_BASE &&
        cmd.command <= COMMAND_SET_CONFIDENCE_BASE + 100) {
        confidencePct = (float)(cmd.command - COMMAND_SET_CONFIDENCE_BASE);
        requestCalibrationSave();   // ecriture LittleFS hors pile NimBLE
        Serial.printf("Audio process - confidence seuil = %.0f%%\n", confidencePct);
        return;
    }

    // COMMAND_AUDIO_SELFTEST doit passer meme sans acquisition initialisee :
    // c'est precisement la panne qu'il sert a diagnostiquer (l'ancien retour
    // anticipe rendait le message error=i2s_not_started inatteignable).
    if (!audioStarted && cmd.command != COMMAND_AUDIO_SELFTEST) {
        Serial.printf("Audio process - audio hardware not started, command 0x%04X ignored\n",
                      cmd.command);
        return;
    }

    switch (cmd.command)
    {
    case COMMAND_RUNCALIBRATION:
        // Arret de la detection et remise a -1 du compteur faits par
        // startCalibrationNow(), seulement une fois la calibration engagee.
        calibrate(CALIBRATION_EMPTY);
        break;

    case COMMAND_RUN_MAILLARD_CALIBRATION:
        calibrate(CALIBRATION_MAILLARD);
        break;

    case COMMAND_START_SAMPLING:
        if (calibrationRequestPending || calibrating) {
            // isCalibrated peut encore valoir true pendant la barriere I2S de
            // startCalibrationNow() : relancer le comptage ferait echouer ou
            // corromprait la calibration demandee.
            Serial.println("Audio process - START refused: calibration pending or running");
        } else if (isCalibrated) {
            if (!crackCounterStatus) {
                detectionSession++;      // avant crackCounterStatus : nouvelle session
                crackCounterStatus = true;
                portENTER_CRITICAL(&crack_mux);
                crack_counter = 0;
                portEXIT_CRITICAL(&crack_mux);
                stats.crack_count = 0;   // sinon STATUS affiche le compteur de la session précédente jusqu'au 1er onset
                Serial.println("Audio process - start Sampling/Counting");
            }
        } else {
            Serial.printf("Audio process - START ignored, missing calibration:%s%s\n",
                          emptyCalibrated ? "" : " EMPTY",
                          maillardCalibrated ? "" : " MAILLARD");
        }
        break;

    case COMMAND_STOP_SAMPLING:
        resumeSamplingOnAbort = false;   // STOP pendant une barriere I2S : ne pas reprendre
        crackCounterStatus = false;
        Serial.println("Audio process - stop Sampling/Counting");
        break;

    case COMMAND_CALIBRATIONSTATE:
        if (calibrating)
            Serial.println("Audio process - calibration running, please wait");
        else
            Serial.printf("Audio process - calibration EMPTY=%s MAILLARD=%s ready=%s\n",
                          emptyCalibrated ? "OK" : "MISSING",
                          maillardCalibrated ? "OK" : "MISSING",
                          isCalibrated ? "YES" : "NO");
        break;

    case COMMAND_AUDIO_SELFTEST:
        if (!audioStarted) {
            Serial.println("#TILAU_SELFTEST error=i2s_not_started");
        } else if (monitorAudioFatal) {
            // La tache qui execute l'auto-test ne sera jamais (re)creee.
            Serial.println("#TILAU_SELFTEST error=monitor_fatal");
        } else if (calibrating || calibrationRequestPending) {
            Serial.println("#TILAU_SELFTEST error=calibrating");
        } else if (crackCounterStatus) {
            Serial.println("#TILAU_SELFTEST error=sampling");
        } else {
            selftestRequested = true;   // execute par monitorAudioTask (branche idle)
        }
        break;

    case COMMAND_SAMPLINGSTATUS:
        Serial.printf("Audio process - sampling %s | calibrated %s\n",
                      crackCounterStatus ? "ON" : "OFF",
                      isCalibrated       ? "YES" : "NO");
        // Ligne d'identification machine : c'est le seul aller-retour
        // BLE -> serie garanti sans prerequis (ni calibration, ni debug ON).
        // bench_capture.py s'en sert en preflight pour prouver que le port
        // serie ouvert est bien celui de CE board ET que le firmware est
        // bien le build banc de test (stream=1).
        Serial.printf("#TILAU_BUILD stream=%d fs=%d hop=%d magic=FACE1742\n",
#if defined(TILAU_AUDIO_STREAM)
                      1,
#else
                      0,
#endif
                      I2S_SAMPLE_RATE, HOP);
        break;

    case COMMAND_GETCRACKCOUNTER:
    {
        portENTER_CRITICAL(&crack_mux);
        int32_t v = crack_counter;
        portEXIT_CRITICAL(&crack_mux);
        Serial.printf("Audio process - crack counter: %d\n", v);
        break;
    }

    // FIX #5 : commandes de debug série activables/désactivables via BLE
    case COMMAND_DEBUG_ON:
        audioDebugEnabled = true;
        Serial.println("Audio process - serial debug ENABLED");
        break;

    case COMMAND_DEBUG_OFF:
        audioDebugEnabled = false;
        Serial.println("Audio process - serial debug DISABLED");
        break;

    case COMMAND_GET_CONFIDENCE:
        Serial.printf("Audio process - confidence seuil = %.0f%%\n", confidencePct);
        break;

    default:
        Serial.printf("Audio process - Unknown command: 0x%04X\n", cmd.command);
        break;
    }
}

void AudioDataCallbacks::onStatus(NimBLECharacteristic *pCharacteristic, int code)
{
    Serial.printf("Notification/Indication return code on audio : %d, %s\n",
                  code, NimBLEUtils::returnCodeToString(code));
}

void AudioDataCallbacks::onSubscribe(NimBLECharacteristic *pCharacteristic,
                                     NimBLEConnInfo &connInfo, uint16_t subValue)
{
    std::string str = "Client ID: ";
    str += connInfo.getConnHandle();
    str += " Address: ";
    str += connInfo.getAddress().toString();
    if      (subValue == 0) str += " Unsubscribed to ";
    else if (subValue == 1) str += " Subscribed to notifications for ";
    else if (subValue == 2) str += " Subscribed to indications for ";
    else                    str += " Subscribed to notifications and indications for ";
    str += std::string(pCharacteristic->getUUID());
    Serial.printf("%s\n", str.c_str());
}

// ---------------------------------------------------------------------------
// Persistance calibration (LittleFS / JSON)
// ---------------------------------------------------------------------------
// Cles construites dans un buffer pile : 28 concatenations de String par
// sauvegarde, c'est trop de churn heap pour un chemin appele depuis un contexte
// contraint (et inutile ici).
static const char *calibKey(char *buf, size_t len, const char *prefix, const char *field)
{
    snprintf(buf, len, "%s_%s", prefix, field);
    return buf;
}
#define CALIB_KEY(field) calibKey(key, sizeof(key), prefix, field)

static void writeCalibrationProfile(JsonDocument &doc, const char *prefix,
                                    const AudioStats &profile)
{
    char key[40];
    doc[CALIB_KEY("peak_amplitude")]  = profile.peak_amplitude;
    doc[CALIB_KEY("noise_floor_rms")]= profile.noise_floor_rms;
    doc[CALIB_KEY("threshold")]= profile.threshold;
    doc[CALIB_KEY("noise_p99")]= profile.noise_p99;
    doc[CALIB_KEY("noise_p999")]= profile.noise_p999;
    doc[CALIB_KEY("snr_db")]= profile.snr_db;
    doc[CALIB_KEY("flux_median")]= profile.flux_median;
    doc[CALIB_KEY("flux_mad")]= profile.flux_mad;
    doc[CALIB_KEY("flux_p99")]= profile.flux_p99;
    doc[CALIB_KEY("flux_p999")]= profile.flux_p999;
    doc[CALIB_KEY("crest_p99")]= profile.crest_p99;
    doc[CALIB_KEY("crest_p999")]= profile.crest_p999;
    doc[CALIB_KEY("rms_median")]= profile.rms_median;
    doc[CALIB_KEY("rms_p99")]= profile.rms_p99;
}

static bool readCalibrationProfile(const JsonDocument &doc, const char *prefix,
                                   AudioStats &profile)
{
    char key[40];
    if (!doc[CALIB_KEY("peak_amplitude")].is<float>() ||
        !doc[CALIB_KEY("noise_floor_rms")].is<float>() ||
        !doc[CALIB_KEY("threshold")].is<float>() ||
        !doc[CALIB_KEY("noise_p99")].is<float>() ||
        !doc[CALIB_KEY("noise_p999")].is<float>() ||
        !doc[CALIB_KEY("snr_db")].is<float>() ||
        !doc[CALIB_KEY("flux_median")].is<float>() ||
        !doc[CALIB_KEY("flux_mad")].is<float>() ||
        !doc[CALIB_KEY("flux_p99")].is<float>() ||
        !doc[CALIB_KEY("flux_p999")].is<float>() ||
        !doc[CALIB_KEY("crest_p99")].is<float>() ||
        !doc[CALIB_KEY("crest_p999")].is<float>() ||
        !doc[CALIB_KEY("rms_median")].is<float>() ||
        !doc[CALIB_KEY("rms_p99")].is<float>()) {
        return false;
    }

    profile = {};
    profile.peak_amplitude  = doc[CALIB_KEY("peak_amplitude")].as<float>();
    profile.noise_floor_rms = doc[CALIB_KEY("noise_floor_rms")].as<float>();
    profile.threshold       = doc[CALIB_KEY("threshold")].as<float>();
    profile.noise_p99       = doc[CALIB_KEY("noise_p99")].as<float>();
    profile.noise_p999      = doc[CALIB_KEY("noise_p999")].as<float>();
    profile.snr_db          = doc[CALIB_KEY("snr_db")].as<float>();
    profile.flux_median     = doc[CALIB_KEY("flux_median")].as<float>();
    profile.flux_mad        = doc[CALIB_KEY("flux_mad")].as<float>();
    profile.flux_p99        = doc[CALIB_KEY("flux_p99")].as<float>();
    profile.flux_p999       = doc[CALIB_KEY("flux_p999")].as<float>();
    profile.crest_p99       = doc[CALIB_KEY("crest_p99")].as<float>();
    profile.crest_p999      = doc[CALIB_KEY("crest_p999")].as<float>();
    profile.rms_median      = doc[CALIB_KEY("rms_median")].as<float>();
    profile.rms_p99         = doc[CALIB_KEY("rms_p99")].as<float>();

    // Meme predicat que celui applique en fin de calibrateTask : ce qui est
    // accepte en RAM est exactement ce qui sera accepte au boot suivant.
    return calibrationProfileUsable(profile);
}

bool saveCalibrationToFile()
{
    JsonDocument doc;
    doc["format_version"]  = CALIBRATION_FORMAT_VERSION;
    doc["empty_valid"]     = emptyCalibrated;
    doc["maillard_valid"]  = maillardCalibrated;
    doc["confidence_pct"]  = confidencePct;
    // Snapshot sous verrou : aucune E/S LittleFS ici, le fichier n'est ouvert
    // qu'apres, une fois le document JSON entierement construit.
    calibProfileLock();
    writeCalibrationProfile(doc, "empty", emptyCalibration);
    writeCalibrationProfile(doc, "maillard", maillardCalibration);
    calibProfileUnlock();

    Serial.printf("Audio process - Saving calibration to %s...\n", CALIBRATION_FILE);

    // Ecriture atomique : FILE_WRITE tronque le fichier, une coupure pendant
    // l'ecriture directe laissait un JSON corrompu et les deux profils perdus.
    // On ecrit a cote puis on renomme ; le fichier final est toujours soit
    // l'ancien soit le nouveau, complet.
    File file = FILE_SYSTEM.open(CALIBRATION_TMP_FILE, FILE_WRITE);
    if (!file) {
        Serial.println("Audio process - Failed to open file for writing!");
        return false;
    }
    const size_t expected = measureJson(doc);
    const size_t written  = serializeJson(doc, file);
    file.close();
    if (written == 0 || written != expected) {
        Serial.printf("Audio process - Failed to write to file (%u/%u bytes)!\n",
                      (unsigned)written, (unsigned)expected);
        FILE_SYSTEM.remove(CALIBRATION_TMP_FILE);
        return false;
    }
    if (!FILE_SYSTEM.rename(CALIBRATION_TMP_FILE, CALIBRATION_FILE)) {
        // Selon la couche VFS, rename() peut refuser d'ecraser la cible. Entre
        // le remove et le rename seul le .tmp (complet) existe : le boot sait
        // le relire (cf. loadCalibrationFromFile).
        FILE_SYSTEM.remove(CALIBRATION_FILE);
        if (!FILE_SYSTEM.rename(CALIBRATION_TMP_FILE, CALIBRATION_FILE)) {
            Serial.println("Audio process - Failed to rename calibration file!");
            return false;
        }
    }
    Serial.println("Audio process - Calibration saved successfully.");
    return true;
}

static bool readCalibrationJson(const char *path, JsonDocument &doc)
{
    if (!FILE_SYSTEM.exists(path)) return false;
    File file = FILE_SYSTEM.open(path, FILE_READ);
    if (!file) {
        Serial.printf("Audio process - Failed to open %s for reading.\n", path);
        return false;
    }
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    if (error) {
        Serial.printf("Audio process - JSON parse error in %s: %s\n", path, error.c_str());
        return false;
    }
    return true;
}

bool loadCalibrationFromFile()
{
    calibProfileInit();
    JsonDocument doc;
    // Le .tmp ne sert que si le fichier final manque ou est illisible : coupure
    // entre le remove et le rename de saveCalibrationToFile().
    if (!readCalibrationJson(CALIBRATION_FILE, doc)) {
        doc.clear();
        if (!readCalibrationJson(CALIBRATION_TMP_FILE, doc)) {
            Serial.println("Audio process - Calibration file not found or unreadable.");
            return false;
        }
        Serial.printf("Audio process - Calibration restored from %s.\n", CALIBRATION_TMP_FILE);
    }
    if (doc["format_version"].as<int>() != CALIBRATION_FORMAT_VERSION) {
        Serial.printf("Audio process - Calibration format obsolete (found=%d expected=%d).\n",
                      doc["format_version"].as<int>(), CALIBRATION_FORMAT_VERSION);
        return false;
    }
    if (!doc["empty_valid"].is<bool>() || !doc["maillard_valid"].is<bool>()) {
        Serial.println("Audio process - Dual calibration file incomplete.");
        return false;
    }

    const bool storedEmpty = doc["empty_valid"].as<bool>();
    const bool storedMaillard = doc["maillard_valid"].as<bool>();
    calibProfileLock();   // boot uniquement, mais meme discipline que les autres ecritures
    emptyCalibrated = storedEmpty &&
                      readCalibrationProfile(doc, "empty", emptyCalibration);
    maillardCalibrated = emptyCalibrated && storedMaillard &&
                         readCalibrationProfile(doc, "maillard", maillardCalibration);
    calibProfileUnlock();
    confidencePct = doc["confidence_pct"].is<float>() ? doc["confidence_pct"].as<float>()
                                                       : CONFIDENCE_DEFAULT;
    if (!isfinite(confidencePct) || confidencePct < 0.0f || confidencePct > 100.0f) {
        confidencePct = CONFIDENCE_DEFAULT;
    }

    isCalibrated = emptyCalibrated && maillardCalibrated;
    if (isCalibrated) {
        stats = maillardCalibration;
        stats.crack_count = 0;
    }

    Serial.printf("Audio process - Dual calibration loaded: EMPTY=%s MAILLARD=%s ready=%s confidence=%.0f%%\n",
                  emptyCalibrated ? "OK" : "INVALID",
                  maillardCalibrated ? "OK" : "INVALID",
                  isCalibrated ? "YES" : "NO", confidencePct);
    return isCalibrated;
}

// ---------------------------------------------------------------------------
// TestAudio (mode TESTMODE uniquement)
// ---------------------------------------------------------------------------
#if defined(TESTMODE)
int TestAudio(int m)
{
    switch (m)
    {
    case COMMAND_RUNCALIBRATION:
        calibrate(CALIBRATION_EMPTY);
        return 1;
    case COMMAND_RUN_MAILLARD_CALIBRATION:
        calibrate(CALIBRATION_MAILLARD);
        return 1;
    case COMMAND_START_SAMPLING:
        if (isCalibrated && !calibrationRequestPending && !calibrating) {
            detectionSession++;
            crackCounterStatus = true;
            return 1;
        }
        break;
    case COMMAND_STOP_SAMPLING:
        if (isCalibrated) { crackCounterStatus = false; return 1; }
        break;
    case COMMAND_CALIBRATIONSTATE:
        if (calibrating)   return 0;
        if (isCalibrated)  return 1;
        return 0;
    case COMMAND_SAMPLINGSTATUS:
        if (calibrating || !isCalibrated) return 0;
        return crackCounterStatus ? 1 : 0;
    case COMMAND_GETCRACKCOUNTER:
        if (calibrating || !isCalibrated || !crackCounterStatus) return -1;
        {
            portENTER_CRITICAL(&crack_mux);
            int32_t v = crack_counter;
            portEXIT_CRITICAL(&crack_mux);
            return (int)v;
        }
    default:
        Serial.printf("TestAudio - Unknown command: 0x%04X\n", m);
        break;
    }
    return 0;
}
#endif
