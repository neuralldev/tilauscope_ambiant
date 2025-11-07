// audio processing part
#include "monaudio.h"
#include <Wire.h>

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

bool crackCounterStatus = false; // true = running, false = not running
bool isCalibrated = false;    // true = calibration has been done and finished ok, false=calibraton not done, skip counting
bool audioStarted = false;       // true = audio correctly initialized, false = not initialized, therefore no feature working
bool calibrating  = false;     // true is calibration is being run and not finished
NimBLECharacteristic* envAudioChar = nullptr;

// audio processing variables
int16_t samples[BUFFER_SIZE];
double vReal[BUFFER_SIZE];
double vImag[BUFFER_SIZE];
int16_t crack_counter = 0;       // number of cracks count since last calibration
unsigned long lastCrackTime = 0; // last timestamp a crack is detected
float energyThreshold = 0.0F;
float noiseMean = 0, noiseStd = 0;
uint32_t totalCracks = 0;

// Global handle for the calibration task
TaskHandle_t calibrateTaskHandle = NULL;
TaskHandle_t monitorTaskHandle = NULL;

ArduinoFFT<double> FFT(vReal, vImag, BUFFER_SIZE, I2S_SAMPLE_RATE);

// Structure to hold environmental data for BLE transmission
typedef struct __attribute__((packed))
{
  uint16_t header = 0x5555; // Start of message header
  int16_t crack_count;      // nb of crack count (cumulative)
  uint8_t checksum;         // Simple additive checksum for integrity
  uint16_t footer = 0xAAAA; // End of message footer
} AudioData;

#define COMMAND_RUNCALIBRATION 0x0000
#define COMMAND_START_SAMPLING 0x0001
#define COMMAND_STOP_SAMPLING 0x0002
#define COMMAND_CALIBRATIONSTATE 0x0003

typedef struct __attribute__((packed))
{
  uint16_t header = 0x5555; // Start of message header
  int16_t command;          // 0x0000 = CALIBRATE 0x0001 = START COUNTING 0x0002 STOP COUNTING
  uint8_t checksum;         // Simple additive checksum for integrity
  uint16_t footer = 0xAAAA; // End of message footer
} AudioCommand;


// --- FFT band energy ---
double bandEnergy(double *v, int lowHz, int highHz)
{
  int lowBin = (lowHz * BUFFER_SIZE) / I2S_SAMPLE_RATE;
  int highBin = (highHz * BUFFER_SIZE) / I2S_SAMPLE_RATE;
  double sum = 0;
  for (int i = lowBin; i <= highBin; i++)
    sum += v[i];
  return sum / (highBin - lowBin + 1);
}

void calibrateTask(void *pvParameters)
{
    Serial.println("Calibrating in background task...");

    calibrating = true;
    double accumEnergy = 0, accumSq = 0;
    int count = 0;
    bool bIsCalibrated;

    // Use a loop that checks the current time (non-blocking wait)
    for (unsigned long start = millis(); millis() - start < CALIB_TIME_MS; )
    {
        int32_t raw[BUFFER_SIZE];
        size_t bytesRead = 0;

        esp_err_t result = i2s_read(I2S_PORT, static_cast<void *>(raw), sizeof(raw), &bytesRead, portMAX_DELAY);
        if (result == ESP_OK && bytesRead > 0)
        {
            int n = bytesRead / sizeof(int32_t);
            for (int i = 0; i < n; i++)
            {
                int16_t val = raw[i] >> 14;
                vReal[i] = val;
                vImag[i] = 0;
            }

            FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
            FFT.compute(FFTDirection::Forward);
            FFT.complexToMagnitude();

            double e = bandEnergy(vReal, 300, 2000);
            accumEnergy += e;
            accumSq += e * e;
            count++;
        }
        // Give control back to the scheduler for other tasks (optional but good practice)
        vTaskDelay(1); 
    }

    // --- Post-Calibration Processing ---
    if (count == 0)
    {
        Serial.println("calibration failed");
        bIsCalibrated = false;
    }
    else
    {
        noiseMean = accumEnergy / count;
        noiseStd = sqrt((accumSq / count) - (noiseMean * noiseMean));
        energyThreshold = noiseMean + 3 * noiseStd;
        Serial.printf("Calibration done: average noise = %.2f | treshold = %.2f\n", noiseMean, energyThreshold);
        crack_counter = 0;
        bIsCalibrated = true;
    }
    
    // Clean up
    calibrating = false;
    calibrateTaskHandle = NULL; // Clear the handle
    isCalibrated = bIsCalibrated;  // avoid that monitor starts before all the other parameters are set
    vTaskDelete(NULL);          // Delete the current task
}

// run Calibration function
void calibrate()
{
  if (calibrating) {
        Serial.println("Calibration already running.");
        return;
    }
    
    Serial.println("Starting Calibration Task...");
    
    // Create the task
    xTaskCreatePinnedToCore(
        calibrateTask,          // Task function
        "CalibrateTask",        // Name for the task
        4096,                   // Stack size (increase if needed)
        NULL,                   // Parameter to pass
        1,                      // Priority (1 is usually fine)
        &calibrateTaskHandle,   // Task handle (for referencing)
        1                       // Core to run on (Core 1 is good for non-BLE/WiFi tasks)
    );
}


void monitorAudioTask(void *pvParameters)
{
    Serial.println("Audio Monitoring Task started.");

    while (1) // The task runs forever unless explicitly deleted
    {
        // 1. Check Control Flags: Only run core logic if calibrated AND started
        if (isCalibrated && crackCounterStatus) 
        {
            // --- Core Logic from original monitorAudio() ---
            int32_t raw[BUFFER_SIZE];
            size_t bytesRead = 0;

            // Reading audio from I2S - Use a brief delay instead of portMAX_DELAY 
            // if you want the loop to check flags more often, but portMAX_DELAY is good for audio flow control.
            esp_err_t result = i2s_read(I2S_PORT, static_cast<void *>(raw), sizeof(raw), &bytesRead, portMAX_DELAY);
            
            if (result == ESP_OK && bytesRead > 0) 
            {
                // Convert, FFT, Magnitude, and Crack Detection logic
                int n = bytesRead / sizeof(int32_t);
                if (n > 0)
                {
                    for (int i = 0; i < n; i++)
                    {
                        int16_t val = raw[i] >> 14;
                        vReal[i] = val;
                        vImag[i] = 0;
                    }

                    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
                    FFT.compute(FFTDirection::Forward);
                    FFT.complexToMagnitude();

                    double crackEnergy = bandEnergy(vReal, 2000, 8000);

                    unsigned long now = millis();
                    if (crackEnergy > energyThreshold && (now - lastCrackTime) > REFRACTORY_MS)
                    {
                        // Use a critical section if crack_counter is modified elsewhere 
                        // (though in this design, it's mostly modified here and read in BLE callback)
                        crack_counter++; 
                        lastCrackTime = now;
                        Serial.printf("crack detected! Count = %d (energy %.2f)\n", crack_counter, crackEnergy);

                        // ⚠️ You might want to add a call to pCharacteristic->notify() here 
                        // if you want immediate notification of a crack.
                    }
                }
            }
        }
        else 
        {
            // If not calibrated or counting is stopped, yield to other tasks
            vTaskDelay(100 / portTICK_PERIOD_MS); 
        }
    }
}

// --- Crack detection thread---
void monitorAudio()
{
  if (!isCalibrated) // avoid detecting if calibration failed
    return;

  int32_t raw[BUFFER_SIZE];
  size_t bytesRead = 0;

  // reading audio from I2S
  esp_err_t result = i2s_read(I2S_PORT, static_cast<void *>(raw), sizeof(raw), &bytesRead, portMAX_DELAY);
  if (result != ESP_OK || bytesRead == 0)
    return; // on error abort

  int n = bytesRead / sizeof(int32_t);
  if (n <= 0)
    return; // on null sample abort

  // Convert 24 bits → 16 bits for FFT
  for (int i = 0; i < n; i++)
  {
    int16_t val = raw[i] >> 14; // INMP441 outputs 24bits on 32bits
    vReal[i] = val;
    vImag[i] = 0;
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // Energy in first crack band
  double crackEnergy = bandEnergy(vReal, 2000, 8000);

  unsigned long now = millis();
  if (crackEnergy > energyThreshold && (now - lastCrackTime) > REFRACTORY_MS)
  {
    crack_counter++; //
    lastCrackTime = now;
    Serial.printf("crack detected! Total = %lu (energy %.2f)\n", crack_counter, crackEnergy);
  }
}

void AudioDataCallbacks::onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo)
{
  AudioData data;
  float count_i = 0;
  // Read or simulate data
  data.crack_count = (!isCalibrated || !audioStarted ? ++crack_counter : count_i);
  // Calculate checksum over the data payload (excluding header and footer)
  // The payload starts right after the header field
  //const uint8_t *dataPayload = (const uint8_t *)&data + sizeof(data.header);
  const uint8_t *payloadPtr = reinterpret_cast<const uint8_t *>(&data) + sizeof(data.header);
  // Determine the length of the data fields included in the checksum
  size_t payloadLength = sizeof(data.crack_count);
  //data.checksum = calculateChecksum(dataPayload, payloadLength);
  data.checksum = calculateChecksum(payloadPtr, payloadLength);
  // Set the characteristic value with the entire structure
  //pCharacteristic->setValue((uint8_t *)&data, sizeof(AudioData));
  pCharacteristic->setValue(reinterpret_cast<uint8_t *>(&data), sizeof(AudioData)); 
  // Print current values to the Serial Monitor
  Serial.printf("crack counter: %d crack(s)", (float)data.crack_count);
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

  // --- CORRECTION : Utilisation de memcpy pour la désérialisation ---
  AudioCommand cmd; // Créer une instance locale de la structure
  // Copier les octets du string dans la structure
  memcpy(&cmd, rxData.data(), sizeof(AudioCommand)); 
  // Maintenant, utilisez 'cmd' comme un objet et non plus comme un pointeur

  // Define the payload for checksum calculation
  // Le pointeur pointe vers l'adresse de 'cmd', puis on avance de la taille du header.
  const uint8_t *payloadPtr = reinterpret_cast<const uint8_t *>(&cmd) + sizeof(cmd.header); 
  
  // Payload length = size of command field
  size_t payloadLength = sizeof(cmd.command);
  // Calculate the expected checksum
  uint8_t calculatedChecksum = calculateChecksum(payloadPtr, payloadLength);
  
  // 5. Verify Checksum and Header/Footer for data integrity
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
  Serial.printf("WRITE Success! Valid Command Received: 0x%04X\n", cmd.command);
  switch (cmd.command)
  {
  case COMMAND_RUNCALIBRATION:
    crackCounterStatus = false; // Example use of a global state variable
    crack_counter = 0;
    calibrate();
    break;
  case COMMAND_START_SAMPLING:
    if (isCalibrated)
    {
      crackCounterStatus = true;
      Serial.println("-> Start Sampling/Counting");
      
    }
    break;
  case COMMAND_STOP_SAMPLING:
    if (isCalibrated)
    {
      crackCounterStatus = false;
      Serial.println("-> Stopping Sampling/Counting");
    }
    break;
  case COMMAND_CALIBRATIONSTATE:
    if (calibrating)
    {
      Serial.println("calibration process is currently running ...");
    } else
    if (isCalibrated) {
      Serial.println("calibration done and OK");
    } else
    {
      Serial.println("calibration not done yet or KO");
    }
    break;
  default:
    Serial.printf("-> Warning: Unknown command ID: 0x%04X\n", cmd.command);
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
