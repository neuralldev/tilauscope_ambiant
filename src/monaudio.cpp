// audio processing part
#include "monaudio.h"
#include <Wire.h>
#include "common.h"

//#define TESTMODE

/*
Brochage typique INMP441 ↔ ESP32
INMP441     ESP32       Fonction
VDD         3.3 V       Alimentation
GND         GND         Masse
WS (LRCL)   GPIO 25     Word Select
SCK (BCLK)  GPIO 33     Bit Clock
SD (DOUT)   GPIO 32     Données
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
static float fft_output[FFT_SIZE];

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

    // FIX #10 : raw alloué en heap DMA pour éviter de consommer 4 Ko de stack
    int32_t *raw = (int32_t *)heap_caps_malloc(BUFFER_SIZE * sizeof(int32_t), MALLOC_CAP_DMA);
    if (!raw) {
        Serial.println("Calibration: heap alloc failed, abort.");
        calibrating         = false;
        calibrateTaskHandle = NULL;
        vTaskDelete(NULL);
        return;
    }

    while (millis() - start < CALIB_TIME_MS) {
        size_t    bytesRead = 0;
        esp_err_t result    = i2s_read(I2S_PORT, raw, BUFFER_SIZE * sizeof(int32_t),
                                       &bytesRead, portMAX_DELAY);

        if (result == ESP_OK && bytesRead > 0) {
            int   n               = bytesRead / sizeof(int32_t);
            float current_sum_sq  = 0.0f;
            float local_peak      = 0.0f;

            for (int i = 0; i < n; i++) {
                float s     = (float)(raw[i] >> 8) / 8388608.0f;
                float abs_s = fabsf(s);

                int bin = (int)(abs_s / HISTO_MAX_AMP * HISTO_BINS);
                if (bin >= HISTO_BINS) bin = HISTO_BINS - 1;
                histo[bin]++;
                total_samples++;

                current_sum_sq += s * s;
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

    stats.noise_p99      = p99;
    stats.noise_p999     = p999;
    stats.peak_amplitude = peak_during_calib;

    // Seuil adaptatif : max(P99.9 × 2.5 , RMS × 8)
    float threshold_from_percentile = p999 * 2.5f;
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
    Serial.printf("  (from P99.9x2.5)  : %.6f\n", threshold_from_percentile);
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
void initFFT()
{
    dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    dsps_wind_hann_f32(fft_window, FFT_SIZE);
}

// ---------------------------------------------------------------------------
// analyzeFrequency — FIX #3 : travaille sur FFT_SIZE points
// FIX #1 : appelée une seule fois par buffer crack depuis monitorAudioTask
// ---------------------------------------------------------------------------
void analyzeFrequency(float *data, int n)
{
    // Si n > FFT_SIZE, on ne traite que les FFT_SIZE premiers samples
    int fft_n = (n < FFT_SIZE) ? n : FFT_SIZE;

    for (int i = 0; i < fft_n; i++) {
        fft_input[i * 2 + 0] = data[i] * fft_window[i];
        fft_input[i * 2 + 1] = 0.0f;
    }
    dsps_fft2r_fc32(fft_input, fft_n);
    dsps_bit_rev_fc32(fft_input, fft_n);

    for (int i = 0; i < fft_n / 2; i++) {
        float re       = fft_input[i * 2 + 0];
        float im       = fft_input[i * 2 + 1];
        fft_output[i]  = sqrtf(re * re + im * im) / fft_n;
    }

    float bin_width       = (float)I2S_SAMPLE_RATE / fft_n;
    float max_crack_energy = 0.0f;
    float total_energy    = 0.0f;

    // Spectre global (0 – 5 kHz)
    Serial.print("\nGlobal : ");
    for (int i = 0; i < fft_n / 2; i += 3) {
        float freq = i * bin_width;
        if (freq > 5000.0f) break;
        float val  = fft_output[i] * 15000.0f;
        if      (val > 10.0f) Serial.print("H");
        else if (val >  2.0f) Serial.print("x");
        else                  Serial.print(".");
        total_energy += fft_output[i];
    }

    // Focus zone crack (1.5–4 kHz)
    Serial.print("\nFocus  :           ");
    for (int i = 0; i < fft_n / 2; i += 3) {
        float freq = i * bin_width;
        if (freq > 5000.0f) break;
        if (freq >= 1500.0f && freq <= 4000.0f) {
            float val = fft_output[i] * 25000.0f;
            if (val > 5.0f) {
                Serial.print("^");
                if (val > max_crack_energy) max_crack_energy = val;
            } else {
                Serial.print(" ");
            }
        } else {
            Serial.print(" ");
        }
    }

    // Barre d'intensité
    Serial.print("\nImpact : ");
    int power = (int)(max_crack_energy * 4);
    if (power > 40) power = 40;
    Serial.print("[");
    for (int j = 0; j < 40; j++) Serial.print(j < power ? "=" : " ");
    Serial.printf("] +%.1f dB\n", 20.0f * log10f(max_crack_energy + 0.0001f));
    Serial.println("--------------------------------------------------");
}

// ---------------------------------------------------------------------------
// monitorAudioTask
// ---------------------------------------------------------------------------
void monitorAudioTask(void *pvParameters)
{
    // FIX #10 : buffers statiques pour ne pas consommer la stack task
    static float input_f[BUFFER_SIZE];
    static float output_f[BUFFER_SIZE];

    TickType_t last_crack_time      = 0;
    bool       last_status_was_running = false;
    UBaseType_t uxHighWaterMark;
    static int  check_count         = 0;

    // Diagnostic : pic max observé depuis le dernier rapport debug
    static float max_observed       = 0.0f;
    // FIX #8 : last_report sorti de la boucle interne (était static local dans la boucle)
    static uint32_t last_report_ms  = 0;

    Serial.println("Audio process - Monitoring Task starting");
    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    Serial.printf("Free stack : %d words (4 bytes each)\n", uxHighWaterMark);

    initFFT();

    // FIX #10 : raw alloué en heap DMA, partagé pour tout le cycle de vie de la tâche
    int32_t *raw = (int32_t *)heap_caps_malloc(BUFFER_SIZE * sizeof(int32_t), MALLOC_CAP_DMA);
    if (!raw) {
        Serial.println("monitorAudioTask: heap alloc failed, task aborted.");
        monitorTaskHandle = NULL;
        vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        if (isCalibrated && crackCounterStatus)
        {
            // Réinitialisation état filtre à la (re)mise en route
            if (!last_status_was_running) {
                filter_state[0]        = 0.0f;
                filter_state[1]        = 0.0f;
                last_status_was_running = true;
                Serial.println("Audio process - Monitor logic active");
            }

            size_t    bytesRead = 0;
            esp_err_t result    = i2s_read(I2S_PORT, raw,
                                           BUFFER_SIZE * sizeof(int32_t),
                                           &bytesRead, portMAX_DELAY);

            if (result == ESP_OK && bytesRead > 0)
            {
                int n = bytesRead / sizeof(int32_t);

                // Normalisation 24 bits → float [-1 ; 1]
                for (int i = 0; i < n; i++) {
                    input_f[i] = (float)(raw[i] >> 8) / 8388608.0f;
                }

                // FIX #1 filtre DSP : passe-bande centré 2500 Hz, Q=1.2
                // (les coeffs sont générés dans setup() via dsps_biquad_gen_bpf_f32)
                dsps_biquad_f32_ansi(input_f, output_f, n, coeffs, filter_state);

                // FIX #5 diagnostic : rapport debug hors boucle sample, toutes les 2 s,
                // conditionné au flag audioDebugEnabled
                // FIX #8 : last_report_ms est static hors boucle — un seul appel millis() ici
                uint32_t now_ms = millis();
                if (audioDebugEnabled && (now_ms - last_report_ms > 2000)) {
                    // FIX #8 : format string corrigé (%% pour le signe %)
                    float delta = fabsf((max_observed - stats.threshold) / stats.threshold);
                    Serial.printf("Debug | Noise max: %.4f | Threshold: %.4f | Max peak: %.4f | Delta: %.0f%%\n",
                                  max_observed, stats.threshold, max_observed, delta * 100.0f);
                    if (delta <= 0.1f) {
                        Serial.println("Warning: Ambient noise close to threshold, consider re-calibrating.");
                    }
                    last_report_ms = now_ms;
                    max_observed   = 0.0f;
                }

                // Détection de crack — boucle sample
                static float last_amp    = 0.0f;
                bool         crack_found = false;

                for (int i = 0; i < n; i++)
                {
                    float amp = fabsf(output_f[i]);

                    // Mise à jour du pic pour diagnostic (toujours, pas seulement en debug)
                    if (amp > max_observed) max_observed = amp;

                    // Condition : dépasse le seuil ET est un front montant net (×3 vs dernier)
                    if (amp > stats.threshold && amp > (last_amp * 3.0f))
                    {
                        TickType_t now_ticks = xTaskGetTickCount();
                        if ((now_ticks - last_crack_time) > pdMS_TO_TICKS(DSP_DEAD_TIME_MS))
                        {
                            portENTER_CRITICAL(&crack_mux);
                            crack_counter++;
                            int32_t val = crack_counter;
                            portEXIT_CRITICAL(&crack_mux);

                            stats.crack_count = (uint32_t)val;
                            if (amp > stats.peak_amplitude) stats.peak_amplitude = amp;

                            last_crack_time = now_ticks;
                            crack_found     = true;

                            Serial.printf("- mon - CRACK #%d amp=%.4f threshold=%.4f\n",
                                          val, amp, stats.threshold);
                        }
                    }
                    last_amp = amp;
                }

                // FIX #1 : analyzeFrequency appelée UNE SEULE FOIS après la boucle
                if (crack_found) {
                    analyzeFrequency(output_f, n);
                }
            }

            // Surveillance stack périodique (toutes les ~100 lectures)
            if (++check_count >= 100) {
                uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
                Serial.printf("Stack High Water Mark : %d free words\n", uxHighWaterMark);
                check_count = 0;
            }
        }
        else
        {
            // Pause / non calibré
            if (last_status_was_running) {
                last_status_was_running = false;
                Serial.println("Audio process - Monitor logic paused");
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        // Respiration minimale pour BLE / système
        vTaskDelay(1);
    }

    // Unreachable — mais propre si la tâche était un jour stoppée
    heap_caps_free(raw);
    monitorTaskHandle = NULL;
    vTaskDelete(NULL);
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

    case COMMAND_RAISERATIO:
        stats.threshold *= 1.1f;
        Serial.printf("Audio process - threshold raised to %.6f\n", stats.threshold);
        break;

    case COMMAND_DECREASERATIO:
        stats.threshold *= 0.9f;
        Serial.printf("Audio process - threshold lowered to %.6f\n", stats.threshold);
        break;

    case COMMAND_RAISERATIO5:
        stats.threshold *= 1.5f;
        Serial.printf("Audio process - threshold raised x1.5 to %.6f\n", stats.threshold);
        break;

    case COMMAND_DECREASERATIO5:
        stats.threshold *= (1.0f / 1.5f);
        Serial.printf("Audio process - threshold lowered /1.5 to %.6f\n", stats.threshold);
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
    if (!doc["threshold"] || !doc["peak_amplitude"] ||
        !doc["noise_floor_rms"] || !doc["isCalibrated"]) {
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
