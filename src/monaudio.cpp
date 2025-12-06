// audio processing part
#include "monaudio.h"
#include <Wire.h>
#include "common.h"

#define TESTMODE

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
int bandEnergyMin = 4500;
int bandEnergyMax = 8000;
int low_bandEnergyMin = 200;
int low_bandEnergyMax = 2000;
float lowEnergyGate = 35000.0; // <-- AUGMENTÉ : Seuil absolu au-dessus du bruit moyen (~21000)
float energycoeff = 1.70;      // <-- AUGMENTÉ : Ratio minimal pour être un crack (avant à 1.6)

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
    Serial.println("Audio process - calibrating in background (30 seconds lock on audio features)...");

    calibrating = true; // just in case it, normaly set y task launcher
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

            double e = bandEnergy(vReal, bandEnergyMin, bandEnergyMax);
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
        Serial.println("Audio process - calibration has failed, no data was fetched");
        bIsCalibrated = false; // cannot unlock other audio features 
    }
    else
    {
        noiseMean = accumEnergy / count;
        noiseStd = sqrt((accumSq / count) - (noiseMean * noiseMean));
        energyThreshold = noiseMean + 3 * noiseStd;
        Serial.printf("Audio process - calibration is done on %d packets : average noise = %.2f | treshold = %.2f\n", count, noiseMean, energyThreshold);
        crack_counter = 0; // reset crack counter
        bIsCalibrated = true; // set flag to allow to unlock audio features
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
        Serial.println("Audio process - Calibration already running!");
        return;
    }
  // locck access to calibration
  calibrating = true;
  Serial.println("Audio process - Starting Calibration Task...");

  // Create the task
  xTaskCreatePinnedToCore(
      calibrateTask,          // Task function
      "CalibrateTask",        // Name for the task
      12288,                   // Stack size (increase if needed)
      NULL,                   // Parameter to pass
      1,                      // Priority (1 is usually fine)
      &calibrateTaskHandle,   // Task handle (for referencing)
      1                       // Core to run on (Core 1 is good for non-BLE/WiFi tasks)
  );
}


void monitorAudioTask(void *pvParameters)
{
    Serial.println("Audio process - Monitoring Task starting");

    while (1) // The task runs forever unless explicitly deleted
    {
        // Check Control Flags: Only run core logic if calibrated AND started
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

                    // 1. Calculer l'énergie de la bande CRACK (Haute Fréquence)
                    double highEnergy = bandEnergy(vReal, bandEnergyMin, bandEnergyMax); 

                    // 2. Calculer l'énergie de la bande BRUIT (Basse Fréquence)
                    // Élargissement de la bande pour capturer plus de bruit de fond stable (e.g., 200 Hz à 2000 Hz)
                    double lowEnergy = bandEnergy(vReal, low_bandEnergyMin, low_bandEnergyMax); // 300, 1500 -> 200, 2000

                    // 3. Calculer le Ratio
                    double ratio = 0.0;
                    // Garde-fou : Le dénominateur doit être suffisant pour éviter une division erronée
                    if (lowEnergy > 5000.0) { // Augmenté pour garantir un dénominateur stable et significatif
                        ratio = highEnergy / lowEnergy;
                    }

                    unsigned long now = millis();
                    
                    const double MIN_CRACK_ENERGY_GATE = lowEnergyGate; // Garde-fou de l'énergie
                    const double MIN_RATIO_THRESHOLD = energycoeff;     // Seuil de ratio (LE POINT CRITIQUE À AJUSTER)
                    //if (ratio > 0)
                    Serial.printf("Audio process - mon - (E_high: %.2f / E_low: %.2f / Ratio: %.2f)\n", highEnergy, lowEnergy, ratio);
                    if (highEnergy > MIN_CRACK_ENERGY_GATE && 
                        ratio > MIN_RATIO_THRESHOLD && 
                        (now - lastCrackTime) > REFRACTORY_MS) // 150ms
                    {
                        crack_counter++; 
                        lastCrackTime = now;
                        Serial.printf("Audio process - mon - crack detected! count=%d (E_high: %.2f / E_low: %.2f / Ratio: %.2f)\n", 
                                      crack_counter, highEnergy, lowEnergy, ratio);
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

void AudioDataCallbacks::onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo)
{
  AudioData data;
  // Read or simulate data
  data.crack_count = (!isCalibrated || !audioStarted ? 0 : crack_counter);
  // Calculate checksum over the data payload (excluding header and footer)
  // The payload starts right after the header field
  const uint8_t *payloadPtr = reinterpret_cast<const uint8_t *>(&data) + sizeof(data.header);
  // Determine the length of the data fields included in the checksum
  size_t payloadLength = sizeof(data.crack_count);
  data.checksum = calculateChecksum(payloadPtr, payloadLength);
  // Set the characteristic value with the entire structure
  pCharacteristic->setValue(reinterpret_cast<uint8_t *>(&data), sizeof(AudioData)); 
  // Print current values to the Serial Monitor
  Serial.printf("on read received - crack counter: %d crack(s)\n", (float)data.crack_count);
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
  //Serial.printf("WRITE Success! Valid Command Received: 0x%04X\n", cmd.command);
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
      Serial.println("Audip process - received command to start Sampling/Counting");
    }
    break;
  case COMMAND_STOP_SAMPLING:
    if (isCalibrated)
    {
      crackCounterStatus = false;
      Serial.println("Audio process - recieved command to stop Sampling/Counting");
    }
    break;
  case COMMAND_CALIBRATIONSTATE:
    if (calibrating)
    {
      Serial.println("Audio process - status request, calibration running, please wait!");
    } else
    if (isCalibrated) {
      Serial.println("Audio process - status request, calibration OK");
    } else
    {
      Serial.println("Audio process - status request, calibration not done yet or KO");
    }
    break;
  default:
    Serial.printf("-Audio process - status request, Unknown command: 0x%04X\n", cmd.command);
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

#if defined(TESTMODE)
int TestAudio(int m) {
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
    } else
    if (isCalibrated) {
      //Serial.println("Audio process - status request, calibration OK");
      return 1;
    } else
    {
      //Serial.println("Audio process - status request, calibration not done yet or KO");
      return 0;
    }
    break;
  case COMMAND_SAMPLINGSTATUS:
    if (calibrating)
      return 0;
    else
      if (isCalibrated) 
        return (crackCounterStatus?1:0);
      else
        return 0;
  case COMMAND_GETCRACKCOUNTER:
    if (calibrating)
      return -1;
    else
      if (isCalibrated) 
        if (crackCounterStatus)
          return crack_counter;
        else
          return -1;
  case COMMAND_RAISERATIO:
    energycoeff += 0.1;
    Serial.printf("Audio process - coefficient set to %f\n",energycoeff);
    return 1;
  case COMMAND_DECREASERATIO:
    energycoeff -= 0.1;
    Serial.printf("Audio process - coefficient set to %f\n",energycoeff);
    return 1;
  case COMMAND_RAISERATIO5:
    energycoeff += 0.5;
    Serial.printf("Audio process - coefficient set to %f\n",energycoeff);
    return 1;
  case COMMAND_DECREASERATIO5:
    energycoeff -= 0.1;
    Serial.printf("Audio process - coefficient set to %f\n",energycoeff);
    return 1;
  default:
    Serial.printf("-Audio process - status request, Unknown command: 0x%04X\n", m);
    break;
  }
    return 0;
}
#endif