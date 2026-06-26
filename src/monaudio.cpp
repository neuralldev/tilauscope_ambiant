// audio processing part
#include "monaudio.h"
#include <Wire.h>
#include "common.h"

//#define TESTMODE

/*
Brochage INMP441 ↔ ESP32-S3 (DevKitC-1 N16R8)
INMP441     ESP32-S3    Fonction
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
volatile bool audioStarted       = false;
volatile bool calibrating        = false;
volatile bool audioDebugEnabled  = false;  // activé via COMMAND_DEBUG_ON
volatile float fluxK = 5.0f;          // thr = base + fluxK * dev

// Handshake I2S : true quand monitorAudioTask NE touche PAS le bus I2S.
// calibrate() attend ce flag avant de lancer calibrateTask, pour éviter deux
// i2s_read() concurrents sur le même port (samples scindés -> calibration faussée).
volatile bool monitorAudioIdle = true;

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

// ---------------------------------------------------------------------------
// Stats de calibration
// ---------------------------------------------------------------------------
AudioStats stats = {0, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f};

#define HISTO_BINS    200
#define HISTO_MAX_AMP 0.5f

// ---------------------------------------------------------------------------
// calibrateTask
// ---------------------------------------------------------------------------
void calibrateTask(void *pvParameters)
{
    float    sum_rms          = 0.0f;
    float    sum_sq_rms       = 0.0f;
    int      count            = 0;
    float    peak_during_calib = 0.0f;

    uint32_t histo[HISTO_BINS] = {0};
    uint32_t total_samples     = 0;

    // FIX #2 : calibrating est déjà mis à true dans calibrate() avant la création
    // de la tâche ; pas besoin de le remettre ici — on laisse quand même pour
    // robustesse en cas d'appel direct futur.
    calibrating = true;
    Serial.println("\n>>> Calibration - Stay quiet!");

    unsigned long start         = millis();
    unsigned long last_ui_update = 0;

    // FIX #10 : raw alloué en heap DMA (NE PAS supprimer — utilisé par i2s_read)
    int32_t *raw = (int32_t *)heap_caps_malloc(BUFFER_SIZE * sizeof(int32_t), MALLOC_CAP_DMA);
    if (!raw) {
        Serial.println("Calibration: heap alloc failed, abort.");
        calibrating         = false;
        calibrateTaskHandle = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Same band-pass as detection, so calibration stats live on the FILTERED scale
    float *cin  = (float *)heap_caps_malloc(BUFFER_SIZE * sizeof(float), MALLOC_CAP_DEFAULT);
    float *cout = (float *)heap_caps_malloc(BUFFER_SIZE * sizeof(float), MALLOC_CAP_DEFAULT);
    if (!cin || !cout) {
        Serial.println("Calibration: filter buffer alloc failed, abort.");
        heap_caps_free(raw);
        heap_caps_free(cin);
        heap_caps_free(cout);
        calibrating = false; calibrateTaskHandle = NULL;
        vTaskDelete(NULL); return;
    }
    float calib_fstate[2] = {0.0f, 0.0f};   // local biquad state (do not touch the detection one)

    while (millis() - start < CALIB_TIME_MS) {
        size_t    bytesRead = 0;
        esp_err_t result    = i2s_read(I2S_PORT, raw, BUFFER_SIZE * sizeof(int32_t),
                                       &bytesRead, portMAX_DELAY);

        if (result == ESP_OK && bytesRead > 0) {
            int   n               = bytesRead / sizeof(int32_t);
            float current_sum_sq  = 0.0f;
            float local_peak      = 0.0f;

            for (int i = 0; i < n; i++)
                cin[i] = (float)(raw[i] >> 8) / 8388608.0f;

            // identical filter to monitorAudioTask — threshold must match detection scale
            dsps_biquad_f32_ansi(cin, cout, n, coeffs, calib_fstate);

            for (int i = 0; i < n; i++) {
                float abs_s = fabsf(cout[i]);

                int bin = (int)(abs_s / HISTO_MAX_AMP * HISTO_BINS);
                if (bin >= HISTO_BINS) bin = HISTO_BINS - 1;
                histo[bin]++;
                total_samples++;

                current_sum_sq += cout[i] * cout[i];
                if (abs_s > local_peak) local_peak = abs_s;
            }

            if (local_peak > peak_during_calib) peak_during_calib = local_peak;
            float current_rms  = sqrtf(current_sum_sq / n);
            sum_rms    += current_rms;
            sum_sq_rms += current_rms * current_rms;
            count++;

            // Affichage progression toutes les 200 ms
            if (millis() - last_ui_update > 200) {
                last_ui_update = millis();
                int progress = (int)((millis() - start) * 20 / CALIB_TIME_MS);
                int vu_len   = (int)(local_peak * 40);
                if (vu_len > 20) vu_len = 20;
                Serial.print("\rProgress: [");
                for (int i = 0; i < 20; i++) Serial.print(i < progress ? "#" : "-");
                Serial.print("] Peak: ");
                for (int i = 0; i < vu_len;  i++) Serial.print(">");
                for (int i = vu_len; i < 20; i++) Serial.print(" ");
                Serial.printf(" (%.4f)", local_peak);
                Serial.flush();
            }
        }
        vTaskDelay(1);
    }
    Serial.println();

    heap_caps_free(raw);
    heap_caps_free(cin);
    heap_caps_free(cout);

    if (count == 0 || total_samples == 0) {
        Serial.println("Calibration failed - no data");
        isCalibrated        = false;
        calibrating         = false;
        calibrateTaskHandle = NULL;
        vTaskDelete(NULL);
        return;
    }

    // --- Calculs statistiques ---
    stats.noise_floor_rms = sum_rms / count;
    float variance = (sum_sq_rms / count) - (stats.noise_floor_rms * stats.noise_floor_rms);
    float std_dev  = sqrtf(fmaxf(0.0f, variance));

    // --- Percentiles depuis histogramme ---
    uint32_t target_p99  = (uint32_t)(total_samples * 0.990f);
    uint32_t target_p999 = (uint32_t)(total_samples * 0.999f);
    float    p99         = HISTO_MAX_AMP;
    float    p999        = HISTO_MAX_AMP;
    uint32_t cumul       = 0;
    bool     found_p99   = false;
    bool     found_p999  = false;

    for (int b = 0; b < HISTO_BINS; b++) {
        cumul += histo[b];
        float bin_amp = ((float)b / HISTO_BINS) * HISTO_MAX_AMP;
        if (!found_p99 && cumul >= target_p99) {
            p99       = bin_amp;
            found_p99 = true;
        }
        if (!found_p999 && cumul >= target_p999) {
            p999       = bin_amp;
            found_p999 = true;
            break;
        }
    }

    // LOG HISTO pour analyse externe (Python analysecrack.py)
    if (audioDebugEnabled) {
        Serial.print("HISTO_DATA [");
        for (int b = 0; b < HISTO_BINS; b++) {
            Serial.print(histo[b]);
            if (b < HISTO_BINS - 1) Serial.print(",");
        }
        Serial.println("]");
    }

    stats.noise_p99      = p99;
    stats.noise_p999     = p999;
    stats.peak_amplitude = peak_during_calib;

    // Seuil adaptatif : max(P99.9 × 1.5 , RMS × 8)
    float threshold_from_percentile = p999 * 1.5f;
    float threshold_from_rms        = stats.noise_floor_rms * 8.0f;
    stats.threshold = fmaxf(threshold_from_percentile, threshold_from_rms);

    // SNR estimé (crack typique à 0.2 conservateur)
    float crack_ref_amp = 0.2f;
    stats.snr_db = 20.0f * log10f(crack_ref_amp / fmaxf(stats.noise_floor_rms, 1e-9f));

    // --- Rapport de calibration ---
    Serial.println("========== CALIBRATION REPORT ==========");
    Serial.printf("Samples collectes   : %u\n",   total_samples);
    Serial.printf("Bruit RMS moyen     : %.6f\n", stats.noise_floor_rms);
    Serial.printf("Ecart-type RMS      : %.6f\n", std_dev);
    Serial.printf("Pic max capture     : %.6f\n", peak_during_calib);
    Serial.printf("Percentile P99      : %.6f\n", p99);
    Serial.printf("Percentile P99.9    : %.6f\n", p999);
    Serial.printf("Seuil final         : %.6f\n", stats.threshold);
    Serial.printf("  (from P99.9x1.5)  : %.6f\n", threshold_from_percentile);
    Serial.printf("  (from RMSx8)      : %.6f\n", threshold_from_rms);
    Serial.printf("SNR estime          : %.1f dB\n", stats.snr_db);
    Serial.println("----------------------------------------");
    if (stats.snr_db < 15.0f) {
        Serial.println("! SNR faible (<15dB) : detection peu fiable.");
        Serial.println("  -> Rapprocher le micro du grain, ou isoler des vibrations.");
    } else if (stats.snr_db < 25.0f) {
        Serial.println("~ SNR acceptable (15-25dB) : detection correcte.");
        Serial.println("  -> Re-calibrer si faux positifs.");
    } else {
        Serial.println("OK SNR bon (>25dB) : conditions optimales.");
    }
    if (p999 > 0.05f) {
        Serial.println("! Queue de distribution large : bruit impulsionnel present.");
        Serial.println("  -> Vibrations mecaniques ? Seuil automatiquement releve.");
    }
    Serial.println("========================================\n");

    portENTER_CRITICAL(&crack_mux);
    crack_counter = 0;
    portEXIT_CRITICAL(&crack_mux);

    isCalibrated = true;
    saveCalibrationToFile();

    calibrating         = false;
    calibrateTaskHandle = NULL;
    vTaskDelete(NULL);
}

// ---------------------------------------------------------------------------
// calibrate() — point d'entrée, appelé depuis callback BLE (onWrite)
// FIX #5 : on utilise un simple flag volatile plutôt que portENTER_CRITICAL
// depuis un contexte potentiellement protégé du stack NimBLE
// ---------------------------------------------------------------------------
void calibrate()
{
    if (calibrating) {
        Serial.println("Audio process - Calibration already running!");
        return;
    }

    // Stopper la détection et ATTENDRE que monitorAudioTask relâche le bus I2S
    // avant d'en lancer un second lecteur (calibrateTask). Sans cette barrière, les
    // deux i2s_read() se partagent les samples -> stats de calibration corrompues.
    crackCounterStatus = false;
    if (monitorTaskHandle != NULL) {
        const uint32_t t0 = millis();
        while (!monitorAudioIdle && (millis() - t0) < 500) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        if (!monitorAudioIdle) {
            Serial.println("Audio process - WARN: monitor still active, calibration may be noisy");
        }
    }

    calibrating = true;

    // Réinitialiser le compteur de façon thread-safe
    portENTER_CRITICAL(&crack_mux);
    crack_counter = -1;
    portEXIT_CRITICAL(&crack_mux);

    Serial.println("Audio process - Starting Calibration Task...");

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
        Serial.println("Failed to create calibrate task!");
        calibrating = false;
        portENTER_CRITICAL(&crack_mux);
        crack_counter = 0;
        portEXIT_CRITICAL(&crack_mux);
    }
}

// ---------------------------------------------------------------------------
// initFFT — FIX #3 : utilise FFT_SIZE (256) au lieu de BUFFER_SIZE (1024)
// ---------------------------------------------------------------------------
esp_err_t initFFT()
{
    esp_err_t err = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (err != ESP_OK) {
        Serial.printf("initFFT: dsps_fft2r_init_fc32 failed: %d\n", err);
        return err;
    }
    dsps_wind_hann_f32(fft_window, FFT_SIZE);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// monitorAudioTask
// ---------------------------------------------------------------------------
void monitorAudioTask(void *pvParameters)
{
    static float frame_raw[FFT_SIZE];        // sliding window (unfiltered) for FFT
    static float frame_bp [FFT_SIZE];        // sliding window (band-passed) for crest
    static float prev_mag [FFT_SIZE/2];      // previous magnitude spectrum (flux)

    if (initFFT() != ESP_OK) {
        Serial.println("monitorAudioTask: FFT init failed, task aborted.");
        monitorTaskHandle = NULL;
        vTaskDelete(NULL);
        return;
    }

    int32_t *raw = (int32_t *)heap_caps_malloc(HOP * sizeof(int32_t), MALLOC_CAP_DMA);
    if (!raw) {
        Serial.println("monitorAudioTask: heap alloc failed, task aborted.");
        monitorTaskHandle = NULL;
        vTaskDelete(NULL);
        return;
    }

    const float bin_hz = (float)I2S_SAMPLE_RATE / FFT_SIZE;   // 62.5 Hz
    const int   k_lo   = (int)(FLUX_F_LO / bin_hz);
    const int   k_hi   = (int)(FLUX_F_HI / bin_hz);

    float    flux_base = 0.0f, flux_dev = 0.0f, prev_flux = 0.0f;
    uint32_t last_onset_ms = 0, t_start = 0, last_status = 0;
    bool     was_counting = false;
    uint32_t frames_seen = 0; 

    // Infinite task: idle when not counting, never exit (else loop() respawns it).
    while (1)
    {
        // ---- OFF state ----
        if (!crackCounterStatus) {
            monitorAudioIdle = true;   // I2S bus released : calibrate() may proceed
            if (was_counting) {
                Serial.printf("#TILAU_AUDIO stop ms=%lu cracks=%u\n", millis(), stats.crack_count);
                was_counting = false;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // Active : on s'apprête à lire le bus I2S — interdit la calibration concurrente
        monitorAudioIdle = false;

        // ---- ON edge: start a fresh detection session ----
        if (!was_counting) {
            memset(frame_raw, 0, sizeof(frame_raw));
            memset(frame_bp , 0, sizeof(frame_bp));
            memset(prev_mag , 0, sizeof(prev_mag));
            filter_state[0] = filter_state[1] = 0.0f;
            flux_base = flux_dev = prev_flux = 0.0f;
            last_onset_ms = 0;
            frames_seen   = 0;
            i2s_zero_dma_buffer(I2S_PORT);   // drop stale backlog, start on live audio
            t_start = last_status = millis();
            Serial.printf("#TILAU_AUDIO start ms=%lu fs=%d fft=%d hop=%d band=%.0f-%.0f alpha=%.3f K=%.1f crest=%.1f refr=%d\n",
                          t_start, I2S_SAMPLE_RATE, FFT_SIZE, HOP, FLUX_F_LO, FLUX_F_HI, FLUX_ALPHA, fluxK, CREST_MIN, REFRACTORY_MS);
            was_counting = true;
        }

        // ---- one hop (128 samples ≈ 8 ms) ----
        size_t bytesRead = 0;
        if (i2s_read(I2S_PORT, raw, HOP * sizeof(int32_t), &bytesRead, portMAX_DELAY) != ESP_OK || bytesRead == 0) {
            vTaskDelay(1); continue;
        }
        int n = bytesRead / sizeof(int32_t);

        // shift sliding windows left by HOP
        memmove(frame_raw, frame_raw + HOP, (FFT_SIZE - HOP) * sizeof(float));
        memmove(frame_bp , frame_bp  + HOP, (FFT_SIZE - HOP) * sizeof(float));

        // new hop: convert (24-in-32 >>8) + CONTINUOUS band-pass (state preserved)
        float xhop[HOP], bhop[HOP];
        for (int i = 0; i < n; i++) xhop[i] = (float)(raw[i] >> 8) / 8388608.0f;
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

        // crest factor on band-passed window (impulsivity)
        float peak = 0.0f, sumsq = 0.0f;
        for (int i = 0; i < FFT_SIZE; i++) {
            float a = fabsf(frame_bp[i]);
            if (a > peak) peak = a;
            sumsq += frame_bp[i] * frame_bp[i];
        }
        float rms   = sqrtf(sumsq / FFT_SIZE);
        float crest = peak / (rms + 1e-9f);

        // spectral flux on raw window (broadband onset)
        for (int i = 0; i < FFT_SIZE; i++) {
            fft_input[2*i]   = frame_raw[i] * fft_window[i];
            fft_input[2*i+1] = 0.0f;
        }
        dsps_fft2r_fc32(fft_input, FFT_SIZE);
        dsps_bit_rev_fc32(fft_input, FFT_SIZE);
        float flux = 0.0f;
        for (int k = k_lo; k <= k_hi; k++) {
            float re = fft_input[2*k], im = fft_input[2*k+1];
            float mag = sqrtf(re*re + im*im) / FFT_SIZE;
            float d = mag - prev_mag[k];
            if (d > 0.0f) flux += d;
            prev_mag[k] = mag;
        }

        // adaptive threshold + onset decision
        float thr = flux_base + fluxK * flux_dev;
        bool warming = (frames_seen < WARMUP_FRAMES);
        bool onset = (!warming) && (flux > thr) && (flux > prev_flux)
                     && (crest > CREST_MIN) && ((now - last_onset_ms) > REFRACTORY_MS);
        if (onset) {
            portENTER_CRITICAL(&crack_mux);     // BLE GETCRACKCOUNTER reads crack_counter
            crack_counter++;
            int32_t val = crack_counter;
            portEXIT_CRITICAL(&crack_mux);
            stats.crack_count = (uint32_t)val;  // mirror for logs
            last_onset_ms = now;
        }
        // update the floor ONLY when NOT an onset (cracks must not raise it)
        if (!onset) {
            flux_base += FLUX_ALPHA * (flux - flux_base);
            flux_dev  += FLUX_ALPHA * (fabsf(flux - flux_base) - flux_dev);
        }
        prev_flux = flux;
        frames_seen++;      

        // per-frame machine log (debug gated)
        if (audioDebugEnabled) {
            Serial.printf("A %lu %.5f %.5f %.5f %.2f %.5f %d\n",
                          now - t_start, flux, flux_base, thr, crest, rms, onset ? 1 : 0);
        }
        // lightweight human STATUS every 2 s (not debug-gated)
        if (now - last_status >= 2000) {
            last_status = now;
            Serial.printf("STATUS t=%lu base=%.4f thr=%.4f crest=%.2f cracks=%u\n",
                          now - t_start, flux_base, thr, crest, stats.crack_count);
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
    if (!audioStarted) return;

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

    switch (cmd.command)
    {
    case COMMAND_RUNCALIBRATION:
        crackCounterStatus = false;
        portENTER_CRITICAL(&crack_mux);
        crack_counter = -1;
        portEXIT_CRITICAL(&crack_mux);
        calibrate();
        break;

    case COMMAND_START_SAMPLING:
        if (isCalibrated) {
            if (!crackCounterStatus) {
                crackCounterStatus = true;
                portENTER_CRITICAL(&crack_mux);
                crack_counter = 0;
                portEXIT_CRITICAL(&crack_mux);
                Serial.println("Audio process - start Sampling/Counting");
            }
        } else {
            Serial.println("Audio process - START ignored, not calibrated");
        }
        break;

    case COMMAND_STOP_SAMPLING:
        crackCounterStatus = false;
        Serial.println("Audio process - stop Sampling/Counting");
        break;

    case COMMAND_CALIBRATIONSTATE:
        if (calibrating)
            Serial.println("Audio process - calibration running, please wait");
        else if (isCalibrated)
            Serial.println("Audio process - calibration OK");
        else
            Serial.println("Audio process - calibration not done or KO");
        break;

    case COMMAND_SAMPLINGSTATUS:
        Serial.printf("Audio process - sampling %s | calibrated %s\n",
                      crackCounterStatus ? "ON" : "OFF",
                      isCalibrated       ? "YES" : "NO");
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
bool saveCalibrationToFile()
{
    JsonDocument doc;
    doc["peak_amplitude"]  = stats.peak_amplitude;
    doc["noise_floor_rms"] = stats.noise_floor_rms;
    doc["threshold"]       = stats.threshold;
    doc["isCalibrated"]    = true;
    doc["noise_p99"]       = stats.noise_p99;
    doc["noise_p999"]      = stats.noise_p999;
    doc["snr_db"]          = stats.snr_db;

    Serial.printf("Audio process - Saving calibration to %s...\n", CALIBRATION_FILE);

    File file = FILE_SYSTEM.open(CALIBRATION_FILE, FILE_WRITE);
    if (!file) {
        Serial.println("Audio process - Failed to open file for writing!");
        return false;
    }
    if (serializeJson(doc, file) == 0) {
        Serial.println("Audio process - Failed to write to file!");
        file.close();
        return false;
    }
    file.close();
    Serial.println("Audio process - Calibration saved successfully.");
    return true;
}

bool loadCalibrationFromFile()
{
    if (!FILE_SYSTEM.exists(CALIBRATION_FILE)) {
        Serial.println("Audio process - Calibration file not found.");
        return false;
    }

    File file = FILE_SYSTEM.open(CALIBRATION_FILE, FILE_READ);
    if (!file) {
        Serial.println("Audio process - Failed to open calibration file for reading.");
        return false;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
        Serial.printf("Audio process - JSON parse error: %s\n", error.c_str());
        return false;
    }
    if (!doc["threshold"].is<float>() || !doc["peak_amplitude"].is<float>() ||
        !doc["noise_floor_rms"].is<float>() || !doc["isCalibrated"].is<bool>()) {
        Serial.println("Audio process - Calibration file incomplete.");
        return false;
    }

    float t = doc["threshold"].as<float>();
    if (t <= 0.0f || isnan(t) || isinf(t)) {
        Serial.println("Audio process - Invalid threshold in calibration file.");
        return false;
    }

    stats.noise_floor_rms = doc["noise_floor_rms"].as<float>();
    stats.threshold       = t;
    stats.peak_amplitude  = doc["peak_amplitude"].as<float>();
    isCalibrated          = doc["isCalibrated"].as<bool>();
    stats.noise_p99       = doc["noise_p99"].as<float>();
    stats.noise_p999      = doc["noise_p999"].as<float>();
    stats.snr_db          = doc["snr_db"].as<float>();

    Serial.printf("Audio process - Calibration loaded: floor=%.4f threshold=%.4f peak=%.4f SNR=%.1fdB\n",
                  stats.noise_floor_rms, stats.threshold,
                  stats.peak_amplitude,  stats.snr_db);
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
        crackCounterStatus = false;
        portENTER_CRITICAL(&crack_mux);
        crack_counter = 0;
        portEXIT_CRITICAL(&crack_mux);
        calibrate();
        return 1;
    case COMMAND_START_SAMPLING:
        if (isCalibrated) { crackCounterStatus = true;  return 1; }
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
