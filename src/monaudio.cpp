// audio processing part
#include "monaudio.h"
#include <Wire.h>
#include "common.h"

//#define TESTMODE

/*
Brochage typique INMP441 ↔ ESP32
INMP441	    ESP32	    Fonction
VDD	        3.3 V	    Alimentation
GND	        GND	      Masse
WS (LRCL)	  GPIO 25	  Word Select
SCK (BCLK)	GPIO 33	  Bit Clock
SD (DOUT)	  GPIO 32	  Données
L/R	        GND	      Canal gauche (Left)
*/

static portMUX_TYPE crack_mux = portMUX_INITIALIZER_UNLOCKED;

bool crackCounterStatus = false; // true = running, false = not running
bool isCalibrated = false;       // true = calibration has been done and finished ok, false=calibraton not done, skip counting
bool audioStarted = false;       // true = audio correctly initialized, false = not initialized, therefore no feature working
bool calibrating = false;        // true is calibration is being run and not finished
NimBLECharacteristic *envAudioChar = nullptr;

// audio processing variables
int16_t samples[BUFFER_SIZE];
int16_t crack_counter = 0;       // number of cracks count since last calibration
unsigned long lastCrackTime = 0; // last timestamp a crack is detected

// Global handle for the calibration task
TaskHandle_t calibrateTaskHandle = NULL;
TaskHandle_t monitorTaskHandle = NULL;

// Structure to hold environmental data for BLE transmission
typedef struct __attribute__((packed))
{
  uint16_t header = 0x5555; // Start of message header
  int16_t crack_count;      // nb of crack count (cumulative)
  uint8_t checksum;         // Simple additive checksum for integrity
  uint16_t footer = 0xAAAA; // End of message footer
} AudioData;

typedef struct __attribute__((packed))
{
  uint16_t header = 0x5555; // Start of message header
  int16_t command;          // 0x0000 = CALIBRATE 0x0001 = START COUNTING 0x0002 STOP COUNTING
  uint8_t checksum;         // Simple additive checksum for integrity
  uint16_t footer = 0xAAAA; // End of message footer
} AudioCommand;

float filter_state[2] = {0, 0};
float coeffs[5];

static float fft_input[BUFFER_SIZE * 2]; // La FFT demande un buffer complexe (réel + imaginaire)
static float fft_window[BUFFER_SIZE];
static float fft_output[BUFFER_SIZE];

// Indicateurs de qualité de l'environnement
float noise_std_dev = 0; // Écart-type (stabilité du bruit)
float signal_to_noise_estimated = 0;

AudioStats stats = {0, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f};

#define HISTO_BINS    200        // résolution de l'histogramme
#define HISTO_MAX_AMP 0.5f       // amplitude max attendue pour le bruit


void calibrateTask(void *pvParameters) {
    float sum_rms    = 0;
    float sum_sq_rms = 0;
    int   count      = 0;
    float peak_during_calib = 0;

    // --- Histogramme des amplitudes sample par sample ---
    uint32_t histo[HISTO_BINS] = {0};
    uint32_t total_samples = 0;

    calibrating = true;
    Serial.println("\n>>> Calibration - Stay quiet!");

    unsigned long start = millis();
    unsigned long last_ui_update = 0;

    while (millis() - start < CALIB_TIME_MS) {
        int32_t raw[BUFFER_SIZE];
        size_t bytesRead = 0;
        esp_err_t result = i2s_read(I2S_PORT, raw, sizeof(raw), &bytesRead, portMAX_DELAY);

        if (result == ESP_OK && bytesRead > 0) {
            int n = bytesRead / sizeof(int32_t);
            float current_sum_sq = 0;
            float local_peak = 0;

            for (int i = 0; i < n; i++) {
                float s = (float)(raw[i] >> 8) / 8388608.0f;
                float abs_s = fabsf(s);

                // --- remplir l'histogramme ---
                int bin = (int)(abs_s / HISTO_MAX_AMP * HISTO_BINS);
                if (bin >= HISTO_BINS) bin = HISTO_BINS - 1;
                histo[bin]++;
                total_samples++;

                current_sum_sq += s * s;
                if (abs_s > local_peak) local_peak = abs_s;
            }

            if (local_peak > peak_during_calib) peak_during_calib = local_peak;
            float current_rms = sqrtf(current_sum_sq / n);
            sum_rms    += current_rms;
            sum_sq_rms += current_rms * current_rms;
            count++;

            // UI update (inchangé)
            if (millis() - last_ui_update > 200) {
                last_ui_update = millis();
                int progress = (millis() - start) * 20 / CALIB_TIME_MS;
                int vu_len = (int)(local_peak * 40);
                if (vu_len > 20) vu_len = 20;
                Serial.print("\rProgress: [");
                for (int i = 0; i < 20; i++) Serial.print(i < progress ? "#" : "-");
                Serial.print("] Peak: ");
                for (int i = 0; i < vu_len; i++) Serial.print(">");
                for (int i = vu_len; i < 20; i++) Serial.print(" ");
                Serial.printf(" (%.4f)", local_peak);
                Serial.flush();
            }
        }
        vTaskDelay(1);
    }
    Serial.println();

    if (count == 0 || total_samples == 0) {
        Serial.println("Calibration failed - no data");
        isCalibrated = false;
        calibrating  = false;
        calibrateTaskHandle = NULL;
        vTaskDelete(NULL);
        return;
    }

    // --- Calculs statistiques ---
    stats.noise_floor_rms = sum_rms / count;
    float variance = (sum_sq_rms / count) - (stats.noise_floor_rms * stats.noise_floor_rms);
    float std_dev  = sqrtf(fmaxf(0.0f, variance));

    // --- Percentiles depuis l'histogramme ---
    // P99 = valeur dépassée par 1% des samples
    // P999 = valeur dépassée par 0.1% des samples
    uint32_t target_p99  = (uint32_t)(total_samples * 0.990f);
    uint32_t target_p999 = (uint32_t)(total_samples * 0.999f);
    float p99  = HISTO_MAX_AMP;  // fallback
    float p999 = HISTO_MAX_AMP;
    uint32_t cumul = 0;
    bool found_p99 = false, found_p999 = false;

    for (int b = 0; b < HISTO_BINS; b++) {
        cumul += histo[b];
        float bin_amp = ((float)b / HISTO_BINS) * HISTO_MAX_AMP;
        if (!found_p99 && cumul >= target_p99) {
            p99 = bin_amp;
            found_p99 = true;
        }
        if (!found_p999 && cumul >= target_p999) {
            p999 = bin_amp;
            found_p999 = true;
            break;
        }
    }

    stats.noise_p99  = p99;
    stats.noise_p999 = p999;
    stats.peak_amplitude = peak_during_calib;

    // --- Seuil adaptatif basé sur P99.9 + marge de sécurité ---
    // On prend le max entre :
    //   - P99.9 * 2.5  (marge sur la queue de distribution réelle)
    //   - noise_rms * 8 (plancher minimum pour éviter un seuil trop bas)
    float threshold_from_percentile = p999 * 2.5f;
    float threshold_from_rms        = stats.noise_floor_rms * 8.0f;
    stats.threshold = fmaxf(threshold_from_percentile, threshold_from_rms);

    // --- SNR estimé ---
    // On estime la puissance d'un crack typique à ~0.2 (valeur conservative)
    float crack_ref_amp = 0.2f;
    stats.snr_db = 20.0f * log10f(crack_ref_amp / fmaxf(stats.noise_floor_rms, 1e-9f));

    // --- Rapport de calibration détaillé ---
    Serial.println("========== CALIBRATION REPORT ==========");
    Serial.printf("Samples collectés   : %u\n",   total_samples);
    Serial.printf("Bruit RMS moyen     : %.6f\n", stats.noise_floor_rms);
    Serial.printf("Écart-type RMS      : %.6f\n", std_dev);
    Serial.printf("Pic max capturé     : %.6f\n", peak_during_calib);
    Serial.printf("Percentile P99      : %.6f\n", p99);
    Serial.printf("Percentile P99.9    : %.6f\n", p999);
    Serial.printf("Seuil final         : %.6f\n", stats.threshold);
    Serial.printf("  (from P99.9×2.5)  : %.6f\n", threshold_from_percentile);
    Serial.printf("  (from RMS×8)      : %.6f\n", threshold_from_rms);
    Serial.printf("SNR estimé          : %.1f dB\n", stats.snr_db);

    // --- Alertes contextualisées ---
    Serial.println("----------------------------------------");
    if (stats.snr_db < 15.0f) {
        Serial.println("⚠ SNR faible (<15dB) : détection peu fiable.");
        Serial.println("  → Rapprocher le micro du grain, ou isoler des vibrations.");
    } else if (stats.snr_db < 25.0f) {
        Serial.println("~ SNR acceptable (15-25dB) : détection correcte.");
        Serial.println("  → Résultats possibles, re-calibrer si faux positifs.");
    } else {
        Serial.println("✓ SNR bon (>25dB) : conditions optimales.");
    }
    if (p999 > 0.05f) {
        Serial.println("⚠ Queue de distribution large : bruit impulsionnel présent.");
        Serial.println("  → Vibrations mécaniques ? Seuil automatiquement relevé.");
    }
    Serial.println("========================================\n");

    crack_counter = 0;
    isCalibrated  = true;
    saveCalibrationToFile();

    calibrating = false;
    calibrateTaskHandle = NULL;
    vTaskDelete(NULL);
}

void my_oldcalibrateTask(void *pvParameters)
{
  float sum_rms = 0;
  float sum_sq_rms = 0;
  int count = 0;
  float peak_during_calib = 0;
  
  Serial.println("\n>>> Audio process - Calibrating... Stay quiet!");
  Serial.println("Progress: [--------------------]  Peak Amp");

  unsigned long start = millis();
  unsigned long last_ui_update = 0;

  while (millis() - start < CALIB_TIME_MS)
  {
    int32_t raw[BUFFER_SIZE];
    size_t bytesRead = 0;
    
    esp_err_t result = i2s_read(I2S_PORT, static_cast<void *>(raw), sizeof(raw), &bytesRead, portMAX_DELAY);
    
    if (result == ESP_OK && bytesRead > 0)
    {
      int n = bytesRead / sizeof(int32_t);
      float current_sum_sq = 0;
      float local_peak = 0;

      for (int i = 0; i < n; i++)
      {
        float s = (float)(raw[i] >> 8) / 8388608.0f;
        current_sum_sq += s * s;
        float abs_s = fabsf(s);
        if (abs_s > local_peak) local_peak = abs_s;
      }

      if (local_peak > peak_during_calib) peak_during_calib = local_peak;

      float current_rms = sqrtf(current_sum_sq / n);
      sum_rms += current_rms;
      sum_sq_rms += (current_rms * current_rms);
      count++;

      // --- MISE À JOUR VISUELLE (Toutes les 200ms) ---
      if (millis() - last_ui_update > 200) {
          last_ui_update = millis();
          
          // 1. Calcul de la progression (0 à 20 barres)
          int progress = (int)((millis() - start) * 20 / CALIB_TIME_MS);
          
          // 2. Création d'un petit vumètre pour le pic local
          int vu_len = (int)(local_peak * 40); 
          if (vu_len > 20) vu_len = 20;

          Serial.print("\rProgress: [");
          for(int i=0; i<20; i++) Serial.print(i < progress ? "#" : "-");
          Serial.print("]  Peak: ");
          for(int i=0; i<vu_len; i++) Serial.print(">");
          for(int i=vu_len; i<20; i++) Serial.print(" ");
          Serial.printf(" (%.4f)", local_peak);
          Serial.flush(); // Pour être sûr que l'affichage est immédiat
      }
    }
    vTaskDelay(1); 
  }
  Serial.println(); // Saut de ligne après la fin des barres

  // --- Post-Calibration  ---
  if (count == 0) {
    Serial.println("Audio process - calibration has failed (no data)");
    isCalibrated = false;
  } else {
    stats.noise_floor_rms = sum_rms / count;
    float variance = (sum_sq_rms / count) - (stats.noise_floor_rms * stats.noise_floor_rms);
    float std_dev = sqrtf(fmaxf(0, variance));

//    stats.threshold = (stats.noise_floor_rms * 6.0f) + (std_dev * 3.0f);
    stats.threshold = (stats.noise_floor_rms * 12.0f) + (std_dev * 5.0f);
    stats.peak_amplitude = peak_during_calib;
    // --- AFFICHAGE DU SCORE DE QUALITÉ ---
    Serial.println("---------- RÉSULTATS ----------");
    Serial.printf("Bruit de fond moyen : %.6f\n", stats.noise_floor_rms);
    Serial.printf("Instabilité (StdDev): %.6f\n", std_dev);
    Serial.printf("Pic max détecté     : %.6f\n", peak_during_calib);
    Serial.printf("SEUIL CALCULÉ       : %.6f\n", stats.threshold);
    
    if (stats.noise_floor_rms > 0.05f) {
        Serial.println("ALERTE : Environnement très bruyant ! Éloignez le micro du moteur.");
    } else if (std_dev > (stats.noise_floor_rms * 0.5f)) {
        Serial.println("ALERTE : Bruit instable (trop de ventilation ou atténuer les vibrations).");
    } else {
        Serial.println("QUALITÉ : Environnement stable et clair.");
    }
    Serial.println("-------------------------------\n");
    portENTER_CRITICAL(&crack_mux);
    crack_counter = 0;
    portEXIT_CRITICAL(&crack_mux);
    isCalibrated = true;
    saveCalibrationToFile();   
  }

  calibrating = false;
  calibrateTaskHandle = NULL;
  vTaskDelete(NULL);
}

// run Calibration function
void calibrate()
{
  if (calibrating)
  {
    Serial.println("Audio process - Calibration already running!");
    return;
  }
  // locck access to calibration
  calibrating = true;
  portENTER_CRITICAL(&crack_mux);
  crack_counter = -1; // during calibration, set to -1 to indicate not ready
  portEXIT_CRITICAL(&crack_mux);
  Serial.println("Audio process - Starting Calibration Task...");

  // Create the task
  BaseType_t res = xTaskCreatePinnedToCore(
      calibrateTask,        // Task function
      "CalibrateTask",      // Name for the task
      12288,                // Stack size (increase if needed)
      NULL,                 // Parameter to pass
      1,                    // Priority (1 is usually fine)
      &calibrateTaskHandle, // Task handle (for referencing)
      1                     // Core to run on (Core 1 is good for non-BLE/WiFi tasks)
  );
  if (res != pdPASS) {
    Serial.println("Failed to create calibrate task!");
    calibrating = false;  // ← libérer le verrou
    portENTER_CRITICAL(&crack_mux);
    crack_counter = 0;
    portEXIT_CRITICAL(&crack_mux);
  } 
}

void initFFT() {
    // Initialisation des fenêtres de Hann pour lisser les résultats
    dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    dsps_wind_hann_f32(fft_window, BUFFER_SIZE);
}

void analyzeFrequency(float* data, int n) {
    // 1 & 2: FFT
    for (int i = 0; i < n; i++) {
        fft_input[i * 2 + 0] = data[i] * fft_window[i];
        fft_input[i * 2 + 1] = 0;
    }
    dsps_fft2r_fc32(fft_input, n);
    dsps_bit_rev_fc32(fft_input, n);
    
    for (int i = 0; i < n / 2; i++) {
        float re = fft_input[i * 2 + 0];
        float im = fft_input[i * 2 + 1];
        fft_output[i] = sqrtf(re * re + im * im) / n;
    }

    float bin_width = (float)I2S_SAMPLE_RATE / n;
    float max_crack_energy = 0;
    float total_energy = 0;

    // --- LIGNE 1 : SPECTRE GLOBAL (0 - 5kHz) ---
    Serial.print("\nGlobal : ");
    for (int i = 0; i < n / 2; i += 6) {
        float freq = i * bin_width;
        if (freq > 5000) break;
        float val = fft_output[i] * 15000.0f;
        if (val > 10) Serial.print("H");      // Peak fort
        else if (val > 2) Serial.print("x");  // Activité
        else Serial.print(".");               // Bruit
        total_energy += fft_output[i];
    }

    // --- LIGNE 2 : FOCUS CRACK (1.5kHz - 4kHz) ---
    Serial.print("\nFocus  :           "); // Espacement pour aligner
    for (int i = 0; i < n / 2; i += 6) {
        float freq = i * bin_width;
        if (freq > 5000) break;
        if (freq >= 1500 && freq <= 4000) {
            float val = fft_output[i] * 25000.0f; // Plus de gain sur cette zone
            if (val > 5) {
                Serial.print("^");
                if (val > max_crack_energy) max_crack_energy = val;
            } else Serial.print(" ");
        } else {
            Serial.print(" ");
        }
    }

    // --- LIGNE 3 : BARRE D'INTENSITÉ ---
    Serial.print("\nImpact : ");
    int power = (int)(max_crack_energy * 4);
    if (power > 40) power = 40;
    Serial.print("[");
    for(int j=0; j<40; j++) {
        if (j < power) Serial.print("=");
        else Serial.print(" ");
    }
    Serial.printf("] +%.1f dB\n", 20 * log10f(max_crack_energy + 0.0001f));
    Serial.println("--------------------------------------------------");
}

void monitorAudioTask(void *pvParameters)
{
  static float input_f[BUFFER_SIZE]; 
  static float output_f[BUFFER_SIZE];  uint32_t last_crack_time = 0;
  bool last_status_was_running = false;
  UBaseType_t uxHighWaterMark;
  static int check_count = 0;
  // diagnostic
  static float max_observed = 0;
  

  Serial.println("Audio process - Monitoring Task starting");
  uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

  Serial.printf("Free stack : %d words (4 bytes)\n", uxHighWaterMark);
  
  initFFT();

  while (1)
  {
    // Vérification des conditions : Doit être calibré ET le comptage doit être activé
    if (isCalibrated && crackCounterStatus) 
    {
      // Si on vient de démarrer le comptage, on réinitialise l'état du filtre
      if (!last_status_was_running) {
        filter_state[0] = 0;
        filter_state[1] = 0;
        last_status_was_running = true;
        Serial.println("Audio process - Monitor logic active");
      }

      int32_t raw[BUFFER_SIZE];
      size_t bytesRead = 0;

      // Lecture des données I2S
      esp_err_t result = i2s_read(I2S_PORT, static_cast<void *>(raw), sizeof(raw), &bytesRead, portMAX_DELAY);

      bool crack_in_buffer = false;

      if (result == ESP_OK && bytesRead > 0)
      {
        int n = bytesRead / sizeof(int32_t);
        
        for (int i = 0; i < n; i++)
        {
          // Traitement INMP441 : 24 bits signés alignés à gauche dans un 32 bits
          // On décale de 8 pour supprimer les 8 bits de padding/LSB
          int32_t sample_32 = raw[i] >> 8;
          // Normalisation par 2^23 (8388608.0f) pour obtenir un float entre -1.0 et 1.0
          input_f[i] = (float)sample_32 / 8388608.0f;
        }

        // Filtrage passe-bas avec esp-dsp
        dsps_biquad_f32_ansi(input_f, output_f, n, coeffs, filter_state);
        static float last_amp = 0;
        for (int i = 0; i < n; i++)
        {
          float amp = fabsf(output_f[i]);
          
          // diagnostic
          float current_amp = amp; // l'amplitude filtrée
          if (current_amp > max_observed) max_observed = current_amp;
          static uint32_t last_report = 0;
          if (millis() - last_report > 2000) {
            float delta = abs((current_amp - stats.threshold)/stats.threshold);
            Serial.printf("delta: %.0f% Noise: %.4f | Treshold: %.4f | Max pike: %.4f\n", 
                          delta*100.0f, current_amp, stats.threshold, max_observed);
            if (delta <= 0.1f) {
                Serial.println("Warning: Ambient noise close to threshold, consider re-calibrating, either microphone is too close either raise filter freq.");
            }
            last_report = millis();
            max_observed = 0; 
          }
          // end diagnostic
          // Comparaison avec le seuil calculé lors de la calibration
          if (amp > stats.threshold && amp > (last_amp * 3.0f))
          {
//            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            TickType_t now = xTaskGetTickCount();
            if ((now - last_crack_time) > pdMS_TO_TICKS(DSP_DEAD_TIME_MS))            
            // Gestion du temps mort (Dead Time) pour éviter les doubles comptages d'un même son
//            if (now - last_crack_time > DSP_DEAD_TIME_MS)
            {
              portENTER_CRITICAL(&crack_mux);
              crack_counter++;
              int16_t val = crack_counter;
              portEXIT_CRITICAL(&crack_mux);
              crack_in_buffer = true;
              stats.crack_count = val; // Mise à jour de la structure stats
              if (amp > stats.peak_amplitude)
                stats.peak_amplitude = amp;
              last_crack_time = now;
              analyzeFrequency(output_f, n);
              Serial.printf("- mon - crack detected! count=%d amp=%.4f\n", val, amp);
            }
          }
        }
        if (crack_in_buffer) analyzeFrequency(output_f, n);
      }
      if (++check_count >= 100) {
          uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
          Serial.printf("Stack High Water Mark : %d free words\n", uxHighWaterMark);
          check_count = 0;
      }
    }
    else
    {
      // Si en pause ou non calibré, on relâche le CPU
      if (last_status_was_running) {
          last_status_was_running = false;
          Serial.println("Audio process - Monitor logic paused");
      }
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Petite pause pour laisser les autres tâches (BLE/System) respirer
    vTaskDelay(1); 
  }
  monitorTaskHandle = NULL;
  vTaskDelete(NULL);
}

void AudioDataCallbacks::onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo)
{
  AudioData data;
  // Read or simulate data
  portENTER_CRITICAL(&crack_mux);
  int16_t val = crack_counter;
  portEXIT_CRITICAL(&crack_mux);
  data.crack_count = (!isCalibrated || !audioStarted ? -1 : val);
  // Calculate checksum over the data payload (excluding header and footer)
  // The payload starts right after the header field
  const uint8_t *payloadPtr = reinterpret_cast<const uint8_t *>(&data) + sizeof(data.header);
  // Determine the length of the data fields included in the checksum
  size_t payloadLength = sizeof(data.crack_count);
  data.checksum = calculateChecksum(payloadPtr, payloadLength);
  // Set the characteristic value with the entire structure
  pCharacteristic->setValue(reinterpret_cast<uint8_t *>(&data), sizeof(AudioData));
  // Print current values to the Serial Monitor
  Serial.printf("on read received - crack counter: %d crack(s)\n", (int)data.crack_count);
}

void AudioDataCallbacks::onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo)
{
  if (!audioStarted)
    return;

  std::string rxData = pCharacteristic->getValue();
  if (rxData.length() != sizeof(AudioCommand))
  {
    Serial.printf("WRITE Error: Received packet size (%d) does not match AudioCommand size (%d)\n",
                  rxData.length(), sizeof(AudioCommand));
    return;
  }
  AudioCommand cmd; // Créer une instance locale de la structure
  memcpy(&cmd, rxData.data(), sizeof(AudioCommand));
  //  Serial.println("debug - command copied");
  // Define the payload for checksum calculation
  // Le pointeur pointe vers l'adresse de 'cmd', puis on avance de la taille du header.
  const uint8_t *payloadPtr = reinterpret_cast<const uint8_t *>(&cmd) + sizeof(cmd.header);
  //  Serial.println("debug - payload unpacked");
  // Payload length = size of command field
  size_t payloadLength = sizeof(cmd.command);
  // Calculate the expected checksum
  uint8_t calculatedChecksum = calculateChecksum(payloadPtr, payloadLength);
  //  Serial.println("debug - checksum calculated");

  // Verify Checksum and Header/Footer for data integrity
  if (cmd.header != 0x5555 || cmd.footer != 0xAAAA)
  {
    Serial.printf("WRITE Error: Invalid Header (0x%04X) or Footer (0x%04X)\n", cmd.header, cmd.footer);
    return;
  }
  if (cmd.checksum != calculatedChecksum)
  {
    Serial.printf("WRITE Error: Checksum mismatch. Received: 0x%02X, Calculated: 0x%02X\n", cmd.checksum, calculatedChecksum);
    return;
  }
  // Checksum is valid! Unpack and execute the command
  switch (cmd.command)
  {
  case COMMAND_RUNCALIBRATION:
    crackCounterStatus = false; // Example use of a global state variable
    portENTER_CRITICAL(&crack_mux);
    crack_counter = -1;
    portEXIT_CRITICAL(&crack_mux);

    calibrate();
    break;
  case COMMAND_START_SAMPLING:
    if (isCalibrated)
    {
      if (!crackCounterStatus) {          // ← seulement si pas déjà démarré
            crackCounterStatus = true;
        portENTER_CRITICAL(&crack_mux);
        crack_counter = 0;
        portEXIT_CRITICAL(&crack_mux);
            Serial.println("Audio process - start Sampling/Counting");
        }
      Serial.println("Audio process - start Sampling/Counting");
    }
    break;
  case COMMAND_STOP_SAMPLING:
    if (isCalibrated)
    {
      crackCounterStatus = false;
      Serial.println("Audio process - stop Sampling/Counting");
    }
    break;
  case COMMAND_CALIBRATIONSTATE:
    if (calibrating)
    {
      Serial.println("Audio process - status request, calibration running, please wait!");
    }
    else if (isCalibrated)
    {
      Serial.println("Audio process - status request, calibration OK");
    }
    else
    {
      Serial.println("Audio process - status request, calibration not done yet or KO");
    }
    break;
  default:
    Serial.printf("-Audio process - Unknown command: 0x%04X\n", cmd.command);
    break;
  }
}

void AudioDataCallbacks::onStatus(NimBLECharacteristic *pCharacteristic, int code)
{
  Serial.printf("Notification/Indication return code on audio : %d, %s\n", code, NimBLEUtils::returnCodeToString(code));
}

void AudioDataCallbacks::onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue)
{
  std::string str = "Client ID: ";
  str += connInfo.getConnHandle();
  str += " Address: ";
  str += connInfo.getAddress().toString();
  if (subValue == 0)
  {
    str += " Unsubscribed to ";
  }
  else if (subValue == 1)
  {
    str += " Subscribed to notifications for ";
  }
  else if (subValue == 2)
  {
    str += " Subscribed to indications for ";
  }
  else if (subValue == 3)
  {
    str += " Subscribed to notifications and indications for ";
  }
  str += std::string(pCharacteristic->getUUID());
  Serial.printf("%s\n", str.c_str());
}

/**
 * @brief Sauvegarde les paramètres de calibration (noiseMean, noiseStd, energyThreshold, energycoeff) dans un fichier JSON.
 * @return true si la sauvegarde a réussi, false sinon.
 */
bool saveCalibrationToFile()
{
  // La taille du document dépend de vos données.
  // Pour 4 float (4x8=32 octets) + clés + surcoût, 128 octets sont largement suffisants.
  JsonDocument doc;

  doc["peak_amplitude"] = stats.peak_amplitude;
  doc["noise_floor_rms"] = stats.noise_floor_rms;
  doc["threshold"] = stats.threshold;
  doc["isCalibrated"] = true; // Sauvegarder l'état de calibration
  doc["noise_p99"]  = stats.noise_p99;
  doc["noise_p999"] = stats.noise_p999;
  doc["snr_db"]     = stats.snr_db;

  Serial.printf("Audio process - Saving calibration to %s...\n", CALIBRATION_FILE);

  File file = FILE_SYSTEM.open(CALIBRATION_FILE, FILE_WRITE);
  if (!file)
  {
    Serial.println("Audio process - Failed to open file for writing!");
    return false;
  }

  if (serializeJson(doc, file) == 0)
  {
    Serial.println("Audio process - Failed to write to file!");
    file.close();
    return false;
  }

  file.close();
  Serial.println("Audio process - Calibration saved successfully.");
  return true;
}

/**
 * @brief Charge les paramètres de calibration à partir du fichier JSON.
 * @return true si le chargement a réussi et les paramètres sont valides, false sinon.
 */
bool loadCalibrationFromFile()
{
  if (!FILE_SYSTEM.exists(CALIBRATION_FILE))
  {
    Serial.println("Audio process - Calibration file not found.");
    return false;
  }

  File file = FILE_SYSTEM.open(CALIBRATION_FILE, FILE_READ);
  if (!file)
  {
    Serial.println("Audio process - Failed to open calibration file for reading.");
    return false;
  }

  JsonDocument doc;

  DeserializationError error = deserializeJson(doc, file);
  file.close();

  if (error)
  {
    Serial.printf("Audio process - Failed to read file, error: %s\n", error.c_str());
    return false;
  }

  // Vérification de la présence des clés
  if (!doc["threshold"] || !doc["peak_amplitude"] || !doc["noise_floor_rms"] || !doc["isCalibrated"])
  {
    Serial.println("Audio process - Calibration file is incomplete or invalid.");
    return false;
  }

  // Assignation des valeurs globales
  stats.noise_floor_rms = doc["noise_floor_rms"].as<float>();
  float t = doc["threshold"].as<float>();
  if (t <= 0.0f || isnan(t) || isinf(t)) {
      Serial.println("Invalid threshold in calibration file");
      return false;
  }
  stats.threshold = t;
//  stats.threshold = doc["threshold"].as<float>();
  stats.peak_amplitude = doc["peak_amplitude"].as<float>();
  isCalibrated = doc["isCalibrated"].as<bool>();

  stats.noise_p99  = doc["noise_p99"].as<float>();  // | 0.0f = valeur par défaut
  stats.noise_p999 = doc["noise_p999"].as<float>();  // si clé absente (ancien fichier)
  stats.snr_db     = doc["snr_db"].as<float>();     // si clé absente (ancien fichier)


  Serial.printf("Audio process - Calibration loaded from file: floor=%.2f, Threshold=%.2f, peak=%.2f\n",
                stats.noise_floor_rms, stats.threshold, stats.peak_amplitude);
  return isCalibrated; // Retourne l'état de calibration chargé
}

#if defined(TESTMODE)
int TestAudio(int m)
{
  switch (m)
  {
  case COMMAND_RUNCALIBRATION:
    crackCounterStatus = false; // Example use of a global state variable
    crack_counter = 0;
    calibrate();
    return 1;
  case COMMAND_START_SAMPLING:
    if (isCalibrated)
    {
      crackCounterStatus = true;
      Serial.println("Audip process - received command to start Sampling/Counting");
      return 1;
    }
    break;
  case COMMAND_STOP_SAMPLING:
    if (isCalibrated)
    {
      crackCounterStatus = false;
      Serial.println("Audio process - recieved command to stop Sampling/Counting");
      return 1;
    }
    break;
  case COMMAND_CALIBRATIONSTATE:
    if (calibrating)
    {
      Serial.println("Audio process - status request, calibration running, please wait!");
      return 0;
    }
    else if (isCalibrated)
    {
      // Serial.println("Audio process - status request, calibration OK");
      return 1;
    }
    else
    {
      // Serial.println("Audio process - status request, calibration not done yet or KO");
      return 0;
    }
    break;
  case COMMAND_SAMPLINGSTATUS:
    if (calibrating)
      return 0;
    else if (isCalibrated)
      return (crackCounterStatus ? 1 : 0);
    else
      return 0;
  case COMMAND_GETCRACKCOUNTER:
    if (calibrating)
      return -1;
    else if (isCalibrated)
      if (crackCounterStatus)
        return crack_counter;
      else
        return -1;
  default:
    Serial.printf("-Audio process - status request, Unknown command: 0x%04X\n", m);
    break;
  }
  return 0;
}
#endif