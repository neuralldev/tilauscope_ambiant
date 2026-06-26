#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <NimBLEDevice.h>
#include <numeric>
#include <WiFi.h>   // Used only for MAC address to generate unique device name
#include <algorithm>   // Used for string manipulation (std::remove)
#include <string>
#include <cmath> // Used for simulation functions (sin, cos)
// #include <esp_system.h>
#include "esp_log.h" // Include this header at the top of main.cpp
#include "common.h"
#include "monaudio.h"

// --- Definitions ---
// Standard atmospheric pressure at sea level in hPa (used for altitude calculation)
// Pour recalibrer : p_sea = p_local / (1 - alt_NGF/44330)^5.255
#define SEALEVELPRESSURE_HPA (1016.6)
// BLE Service UUID for the environmental data service
#define SERVICE_UUID "f3b6e2A0-8c4e-4e1f-9c2d-1a7f5b9a1c01"
// BLE Characteristic UUID for the environmental data
#define ENV_DATA_CHAR_UUID "F3B6A2A0-8C4E-4E1F-9C2D-1A7F5B9A1C05"

#define RGB_LED_PIN 48      // LED RGB WS2812 intégrée du DevKitC-1 (statut couleur)
#define BLINK_INTERVAL 1000 // LED blink interval in milliseconds (1 second)

// Câblage BME280 (I2C, 3.3V) sur ESP32-S3 DevKitC-1 + terminal adapter :
//   VIN  (fil bleu)   -> 3V3   (bloc gauche, borne 3.3V)
//   GND  (fil vert)   -> GND   (bloc gauche, borne GND)
//   SCL  (fil jaune)  -> GPIO9 (bloc gauche, borne IO9)
//   SDA  (fil orange) -> GPIO8 (bloc gauche, borne IO8)
#define SDA_PIN 8 // I2C Data Pin for the BME280  (fil orange)
#define SCL_PIN 9 // I2C Clock Pin for the BME280 (fil jaune)

// --- Global Variables for BME support ---
Adafruit_BME280 bme;               // BME280 sensor object
NimBLECharacteristic *envDataChar; // Pointer to the BLE Characteristic for ambiant data
NimBLEAdvertising *pAdvertising;   // Pointer to the BLE Advertising object

bool deviceConnected = false;
unsigned long previousMillis = 0;  // For non-blocking timing (LED blink)
unsigned long previousMillis1 = 0; // For non-blocking tempo display
bool simulation = false;            // Flag to indicate if simulation mode is active (sensor failure)
bool previous = true;              // used in test mode only

Adafruit_Sensor *bme_temp;
Adafruit_Sensor *bme_pressure;
Adafruit_Sensor *bme_humidity;

// --- Data Structure ---
// Structure to hold environmental data for BLE transmission
typedef struct __attribute__((packed))
{
  uint16_t header = 0x5555; // Start of message header
  int16_t temp_x10;         // Temperature multiplied by 10 (e.g., 25.4°C -> 254)
  int16_t hum_x10;          // Humidity multiplied by 10
  int32_t press_x10;        // Pressure multiplied by 10
  int32_t alt_x10;          // Altitude multiplied by 10
  uint8_t checksum;         // Simple additive checksum for integrity
  uint16_t footer = 0xAAAA; // End of message footer
} EnvData;

sensors_event_t temp_event, pressure_event, humidity_event;

// --- Characteristic Callbacks (Handles READ request) ---
class EnvironmentDataCallbacks : public NimBLECharacteristicCallbacks
{
  /**
   * Called when a connected peer sends a READ request to the Characteristic.
   * Reads sensor data (or simulated data) and prepares the EnvData structure.
   */
  void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo)
  {
    // Serial.println(">>> READ Request received. Updating data...");

    EnvData data;
    float temp_f = 0.0F;
    float hum_f = 0.0F;
    float press_f = 0.0F;
    float alt_f = 0.0F;

    // Read or simulate data
    if (simulation)
    {
      // Sinusoidal simulation for testing without a sensor
      temp_f = 20.0 + sin(millis() / 5000.0) * 2.0;
      hum_f = 77.1 + cos(millis() / 7000.0) * 3.0;
      press_f = 990.4 + sin(millis() / 10000.0) * 5.0;
      alt_f = 1013 + cos(millis() / 4000.0) * 2.5;
    }
    else
    {
      // Actual sensor reading
      temp_f = bme.readTemperature();
      hum_f = bme.readHumidity();
      press_f = bme.readPressure() / 100.0F; // Convert Pa to hPa
      alt_f = bme.readAltitude(SEALEVELPRESSURE_HPA);
      if (isnan(temp_f) || isnan(hum_f) || isnan(press_f) || isnan(alt_f))
      {
        // temp_f = hum_f = press_f = alt_f = 0.0F;
        Serial.printf("onread error %f %f %f %f\n", temp_f, hum_f, press_f, alt_f);
      }
      else if (!(isCalibrated && crackCounterStatus))
        Serial.printf("onread debug %f %f %f %f\n", temp_f, hum_f, press_f, alt_f);
    }

    // Convert float to fixed-point integer (x10) and store in structure
    // Adding 0.5 performs rounding to the nearest integer
    data.temp_x10 = (int16_t)(temp_f * 10.0 + 0.5);
    data.hum_x10 = (int16_t)(hum_f * 10.0 + 0.5);
    data.press_x10 = (int32_t)(press_f * 10.0 + 0.5);
    data.alt_x10 = (int32_t)(alt_f * 10.0 + 0.5);

    // Calculate checksum over the data payload (excluding header and footer)
    // The payload starts right after the header field
    const uint8_t *dataPayload = reinterpret_cast<const uint8_t *>(&data) + sizeof(data.header);
    // const uint8_t *dataPayload = (const uint8_t *)&data + sizeof(data.header);
    //  Determine the length of the data fields included in the checksum
    size_t payloadLength = sizeof(EnvData) - sizeof(data.header) - sizeof(data.checksum) - sizeof(data.footer);
    data.checksum = calculateChecksum(dataPayload, payloadLength);

    // Set the characteristic value with the entire structure
    pCharacteristic->setValue(reinterpret_cast<uint8_t *>(&data), sizeof(EnvData));
    // pCharacteristic->setValue((uint8_t *)&data, sizeof(EnvData));

    // Print current values to the Serial Monitor
    if (!(isCalibrated && crackCounterStatus))
      Serial.printf("T: %.1f °C | H: %.1f %% | P: %.1f hPa | A: %.1f masl | Checksum: 0x%02X\n",
                    (float)data.temp_x10 / 10.0, (float)data.hum_x10 / 10.0, (float)data.press_x10 / 10.0, (float)data.alt_x10 / 10.0, data.checksum);
  }

  /**
   * Called when a notification/indication status is received from the BLE stack.
   */
  void onStatus(NimBLECharacteristic *pCharacteristic, int code) override
  {
    Serial.printf("Notification/Indication return code: %d, %s\n", code, NimBLEUtils::returnCodeToString(code));
  }
};

// --- Server Callbacks (Handles Connection/Disconnection) ---
class ServerCallbacks : public NimBLEServerCallbacks
{
  /**
   * Called when a BLE client connects to the server.
   */
  void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo)
  {
    Serial.println("Client connected. Advertising stopped.");
    deviceConnected = true; // Update global state

    // Update connection parameters for potentially faster communication (optional)
    pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 180);
  }

  /**
   * Called when a BLE client disconnects from the server.
   */
  void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override
  {
    Serial.printf("Client disconnected - start advertising\n");
    deviceConnected = false;          // Update global state
    NimBLEDevice::startAdvertising(); // Restart advertising so other devices can connect
  }

  /**
   * Called when the Maximum Transmission Unit (MTU) is negotiated.
   */
  void onMTUChange(uint16_t MTU, NimBLEConnInfo &connInfo) override
  {
    Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
  }
};

// --- Setup ---
void setup()
{
  Serial.begin(115200);
  // Activer le niveau DEBUG pour le tag AUDIO sur le port debug-console
  // Permet de voir ESP_LOGD(TAG_MON, ...) depuis /dev/cu.debug-console
  esp_log_level_set("AUDIO", ESP_LOG_DEBUG);
  esp_log_level_set("CAL",   ESP_LOG_DEBUG);
  esp_log_level_set("MON",   ESP_LOG_DEBUG);
  Serial.println(">>-------------------------------------------------------------------------------------");
  Serial.println("TilauScope Ambiant booting");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100 kHz Standard I2C speed for better stability
  delay(100);
  // Attempt to initialize the BME280 sensor
  if (!bme.begin(BME280_ADDRESS))
  {
    Serial.println("BME280 not detected on main address");
    // Try alternate I2C address if primary fails
    if (!bme.begin(BME280_ADDRESS_ALTERNATE))
    {
      Serial.println("BME280 not detected no secondary, enter simulation mode");
      simulation = true; // Set simulation flag if sensor initialization fails
    }
    else
    {
      Serial.println("BME280 found on alternate address (0x76)");
    }
  }
  else
  {
    Serial.println("BME280 found on main address (0x77)");
  }
  if (!simulation)
  {
    Serial.println("Setting BME280 explicit configuration...");

    // Configurer le capteur pour une lecture stable et complète
    bme.setSampling(
        Adafruit_BME280::MODE_NORMAL,   // Mode Normal (lecture périodique automatique)
        Adafruit_BME280::SAMPLING_X2,   // Température sursampling x2
        Adafruit_BME280::SAMPLING_X16,  // Pression sursampling x16 (Haute résolution)
        Adafruit_BME280::SAMPLING_X2,   // Humidité sursampling x2
        Adafruit_BME280::FILTER_X4,     // Filtre IIR x4 (pour améliorer la stabilité)
        Adafruit_BME280::STANDBY_MS_500 // Intervalle de 500ms entre les lectures
    );
    Serial.println("BME280 configured for stable operation.");
    bme_temp = bme.getTemperatureSensor();
    bme_pressure = bme.getPressureSensor();
    bme_humidity = bme.getHumiditySensor();
    bme_temp->printSensorDetails();
    bme_pressure->printSensorDetails();
    bme_humidity->printSensorDetails();
  }
  delay(100);

  // LED RGB de statut (WS2812 intégrée) — éteinte au boot
  rgbLedWrite(RGB_LED_PIN, 0, 0, 0);
  Serial.println("RGB status LED configured (GPIO48)");

  // Disable WiFi to free the radio for BLE — the MAC stays readable from efuse
  WiFi.mode(WIFI_MODE_NULL);
  delay(100);
  std::string macFull = WiFi.macAddress().c_str();
  // generate unique name based on MAC address
  std::string identifier = macFull.substr(macFull.length() - 8);
  identifier.erase(std::remove(identifier.begin(), identifier.end(), ':'), identifier.end());
  std::string baseName = "TLSCAM";
  std::string finalDeviceName = baseName + identifier;
  Serial.println(">>-------------------------------------------------------------------------------------");
  Serial.printf("Device MAC: %s\n", macFull.c_str());
  Serial.printf("Device Name: %s\n", finalDeviceName.c_str());
  Serial.println(">>-------------------------------------------------------------------------------------");

#if defined(TILAUONAUDIO_H)

  // now working on audio threads — I2S RX en nouvelle API "standard" (IDF 5.x)
  // pour acquérir les 24 bits utiles (dans des trames de 32 bits) de l'INMP441
  esp_err_t err = ESP_OK;

  // 1) Canal RX (handle global rx_chan, partagé avec les tâches audio)
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num  = 8;    // ex-dma_buf_count
  chan_cfg.dma_frame_num = 256;  // ex-dma_buf_len

  Serial.println("try to install I2S (std) RX channel");
  err = i2s_new_channel(&chan_cfg, NULL, &rx_chan);   // RX uniquement (handle TX = NULL)
  if (err != ESP_OK)
  {
    Serial.printf("Erreur i2s_new_channel: %d\n", err);
    audioStarted = false;
  }
  else
  {
    // 2) Mode standard Philips, 32 bits, mono. INMP441 L/R=GND -> canal gauche.
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)I2S_SCK_PIN,
            .ws   = (gpio_num_t)I2S_WS_PIN,
            .dout = I2S_GPIO_UNUSED,
            .din  = (gpio_num_t)I2S_SD_PIN,
            .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false},
        },
    };
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;   // INMP441 sur le canal gauche

    err = i2s_channel_init_std_mode(rx_chan, &std_cfg);
    if (err != ESP_OK)
    {
      Serial.printf("Erreur i2s_channel_init_std_mode: %d\n", err);
      audioStarted = false;
    }
    else if ((err = i2s_channel_enable(rx_chan)) != ESP_OK)
    {
      Serial.printf("Erreur i2s_channel_enable: %d\n", err);
      audioStarted = false;
    }
    else
    { // configure DSP filter
      // FIX : passe-bande centré 2500 Hz, Q=1.2 (~bande 1.5–4 kHz couvrant les fréquences de crack)
      // dsps_biquad_gen_bpf_f32 génère un vrai BPF avec gain non-nul (vs bpf0db 0 dB peak)
      err = dsps_biquad_gen_bpf_f32(coeffs, BPF_CENTER_FREQ / float(I2S_SAMPLE_RATE), BPF_Q);
      if (err == ESP_OK)
      {
        audioStarted = true;
        Serial.println("I2S (std) started, listening to INMP441");
      }
      else
      {
        Serial.printf("Erreur dsps_biquad_gen_bpf_f32: %d\n", err);
        audioStarted = false;
      }
    }
  };

  Serial.println("Initializing File System...");
  // Tenter de monter LittleFS
  if (!FILE_SYSTEM.begin())
  {
    Serial.println("Failed to mount LittleFS. Attempting to format...");

    // --- LIGNE À AJOUTER TEMPORAIREMENT ---
    if (FILE_SYSTEM.format())
    {
      Serial.println("LittleFS formatted successfully! Trying mount again...");
      if (FILE_SYSTEM.begin())
      {
        Serial.println("LittleFS mounted successfully after format.");
      }
      else
      {
        Serial.println("Fatal: LittleFS mount failed even after format.");
      }
    }
    else
    {
      Serial.println("Fatal: LittleFS format failed.");
    }
    // --- FIN LIGNES À AJOUTER ---
  }
  else
  {
    Serial.println("File system mounted successfully.");
  }

  // FIX #9 : tentative de chargement de la calibration existante au boot
  // Evite une recalibration si l'ESP32 redémarre en cours de torréfaction
  if (loadCalibrationFromFile()) {
    Serial.println("Boot calibration restored from file - ready to START without CAL.");
  } else {
    Serial.println("No valid calibration on file - send CAL command before START.");
  }
#endif

  Serial.println("initializing BLE");
  // Initialize the Device and Server
  bool initerr = NimBLEDevice::init(finalDeviceName);
  if (initerr)
    Serial.println("BLE init done");
  else
  {
    Serial.println("BLE init failed, abort setup");
    audioStarted = false;
    return; // stop initialization as there is no radio
  }
  NimBLEServer *server = NimBLEDevice::createServer();
  // Set the server callback handler
  server->setCallbacks(new ServerCallbacks());
  // Create the Services and Characteristics
  NimBLEService *envService = server->createService(SERVICE_UUID);
  envDataChar = envService->createCharacteristic(
      ENV_DATA_CHAR_UUID,
      NIMBLE_PROPERTY::READ // The characteristic is only readable by the client
  );
  envDataChar->setCallbacks(new EnvironmentDataCallbacks());

  // if audio is there, add the audio characteristics
#if defined(TILAUONAUDIO_H)
  envAudioChar = envService->createCharacteristic(
      ENV_AUDIO_CHAR_UUID,
      NIMBLE_PROPERTY::READ | WRITE // READ : crack counter | WRITE : commandes audio
  );
  envAudioChar->setCallbacks(new AudioDataCallbacks());
#endif

  if (envService->start())
    Serial.println("BLE server started");
  else
  {
    Serial.println("BLE server failed to start, aborting");
    return; // failed to start server, abort
  }
  Serial.println(">>-------------------------------------------------------------------------------------");

  // now start advertising
  Serial.println("BLE advertising init");
  // Configure and Start Advertising
  pAdvertising = NimBLEDevice::getAdvertising();
  if (!pAdvertising->addServiceUUID(SERVICE_UUID))
  {
    Serial.println("Advertising failed to add Service UUID, abort");
    return;
  }
  pAdvertising->setDiscoverableMode(BLE_GAP_DISC_MODE_GEN);
  pAdvertising->enableScanResponse(true);
  pAdvertising->setName(finalDeviceName);
  Serial.printf("BLE advertising starting on %s\n", SERVICE_UUID);
  if (pAdvertising->start())
  {
    Serial.println("Advertising started successfully");
  }
  else
  {
    Serial.println("Advertising failed to start!, abort");
    return;
  }
  Serial.printf("TilauScope Ambiant BLE service now fully ready, answer on nName: %s\n", finalDeviceName.c_str());
  Serial.println(">>-------------------------------------------------------------------------------------");
};

// --- LED RGB de statut (WS2812 intégrée) ---
//   calibration : rouge clignotant rapide
//   advertising : bleu clignotant 1 Hz
//   connecté    : vert fixe (cyan si détection de cracks active)
static void updateStatusLed()
{
  static uint32_t lastToggle = 0;
  static bool     blinkOn    = false;
  static int      lastMode   = -1;
  uint32_t        now        = millis();

  // 1=calibration, 0=advertising, 3=connecté+détection, 2=connecté
  int mode = calibrating ? 1 : (!deviceConnected ? 0 : (crackCounterStatus ? 3 : 2));

  if (mode == 1) { // calibration : rouge clignotant rapide (250 ms)
    if (now - lastToggle >= 250) {
      lastToggle = now; blinkOn = !blinkOn;
      rgbLedWrite(RGB_LED_PIN, blinkOn ? 80 : 0, 0, 0);
    }
  } else if (mode == 0) { // advertising : bleu clignotant 1 Hz
    if (now - lastToggle >= BLINK_INTERVAL) {
      lastToggle = now; blinkOn = !blinkOn;
      rgbLedWrite(RGB_LED_PIN, 0, 0, blinkOn ? 60 : 0);
    }
  } else if (mode != lastMode) { // états fixes : écrire une seule fois (pas à chaque loop)
    if (mode == 3) rgbLedWrite(RGB_LED_PIN, 0, 40, 40); // cyan = détection active
    else           rgbLedWrite(RGB_LED_PIN, 0, 60, 0);  // vert = connecté
  }
  lastMode = mode;
}

// --- Loop ---
void loop()
{
  // LED Blinking Management (Non-blocking)
  unsigned long currentMillis = millis();
  if ((monitorTaskHandle == NULL) && (audioStarted && isCalibrated))
  {
    Serial.println("Starting Monitor Audio Task...");
    BaseType_t xReturned = xTaskCreatePinnedToCore(
        monitorAudioTask,
        "MonitorAudioTask",
        12288,
        NULL,
        2,
        &monitorTaskHandle,
        1);
    if (xReturned != pdPASS)
    {
      Serial.println("Failed to create Monitor Task!");
    }
  }
  (void)currentMillis;
  updateStatusLed();
}