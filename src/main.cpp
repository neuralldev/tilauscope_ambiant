#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <NimBLEDevice.h>
#include <numeric>
#include <esp_netif.h> // Required for esp_netif_init() and esp_netif_create_default_wifi_sta()
#include <esp_wifi.h>  // Required for esp_wifi_get_mac()
#include <algorithm>   // Used for string manipulation (std::remove)
#include <string>
#include <cmath>        // Used for simulation functions (sin, cos)
//#include <esp_system.h> // Include this header at the top of main.cpp
#include "common.h"
#include "monaudio.h"

// --- Definitions ---
// Standard atmospheric pressure at sea level in hPa (used for altitude calculation)
#define SEALEVELPRESSURE_HPA (1013.25)
// BLE Service UUID for the environmental data service
#define SERVICE_UUID        "f3b6e2A0-8c4e-4e1f-9c2d-1a7f5b9a1c01"
// BLE Characteristic UUID for the environmental data
#define ENV_DATA_CHAR_UUID "F3B6A2A0-8C4E-4E1F-9C2D-1A7F5B9A1C05"

#define LED_PIN 2           // Built-in LED pin on most ESP32 Dev Kits
#define BLINK_INTERVAL 1000 // LED blink interval in milliseconds (1 second)

#define SDA_PIN 21 // I2C Data Pin for the BME280
#define SCL_PIN 22 // I2C Clock Pin for the BME280

// --- Global Variables for BME support ---
Adafruit_BME280 bme;               // BME280 sensor object
NimBLECharacteristic *envDataChar; // Pointer to the BLE Characteristic for ambiant data
NimBLEAdvertising *pAdvertising;   // Pointer to the BLE Advertising object

bool deviceConnected = false;
unsigned long previousMillis  = 0; // For non-blocking timing (LED blink)
unsigned long previousMillis1 = 0; // For non-blocking tempo display
int simulation = false;           // Flag to indicate if simulation mode is active (sensor failure)

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
      if (isnan(temp_f) || isnan(hum_f) || isnan(press_f) || isnan(alt_f)) {
        //temp_f = hum_f = press_f = alt_f = 0.0F;
        Serial.printf("onread error %f %f %f %f\n",temp_f, hum_f, press_f, alt_f);

      }
      else
        Serial.printf("onread debug %f %f %f %f\n",temp_f, hum_f, press_f, alt_f);
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
    //const uint8_t *dataPayload = (const uint8_t *)&data + sizeof(data.header);
    // Determine the length of the data fields included in the checksum
    size_t payloadLength = sizeof(EnvData) - sizeof(data.header) - sizeof(data.checksum) - sizeof(data.footer);
    data.checksum = calculateChecksum(dataPayload, payloadLength);

    // Set the characteristic value with the entire structure
    pCharacteristic->setValue(reinterpret_cast<uint8_t *>(&data), sizeof(EnvData));
    //pCharacteristic->setValue((uint8_t *)&data, sizeof(EnvData));

    // Print current values to the Serial Monitor
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
  Serial.println(">>-------------------------------------------------------------------------------------");
  Serial.println("TilauScope Ambiant booting");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100 kHz Standard I2C speed for better stability
  delay(100);
  // Attempt to initialize the BME280 sensor
  if (!bme.begin(BME280_ADDRESS)) {
    Serial.println("BME280 not detected on main address");
    // Try alternate I2C address if primary fails
    if (!bme.begin(BME280_ADDRESS_ALTERNATE)) {
        Serial.println("BME280 not detected no secondary, enter simulation mode");
        simulation = true; // Set simulation flag if sensor initialization fails
    } else {
        Serial.println("BME280 found on alternate address (0x76)");
    }
  } else {
    Serial.println("BME280 found on main address (0x77)");
  }
  if (!simulation) {
      Serial.println("Setting BME280 explicit configuration...");
      
      // Configurer le capteur pour une lecture stable et complète
      bme.setSampling(
          Adafruit_BME280::MODE_NORMAL,      // Mode Normal (lecture périodique automatique)
          Adafruit_BME280::SAMPLING_X2,      // Température sursampling x2
          Adafruit_BME280::SAMPLING_X16,     // Pression sursampling x16 (Haute résolution)
          Adafruit_BME280::SAMPLING_X2,      // Humidité sursampling x2
          Adafruit_BME280::FILTER_X4,        // Filtre IIR x4 (pour améliorer la stabilité)
          Adafruit_BME280::STANDBY_MS_500    // Intervalle de 500ms entre les lectures
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

  // Configure the LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Ensure LED is off initially
  Serial.println("LED configured to blink every second");

  
  uint8_t mac[6];
  char macStr[18] = {0};

  // Initialise Netif
  esp_err_t err = esp_netif_init(); 
  if (err == ESP_OK) {
    Serial.println("tcp/ip stack initialized");
  } else {
    Serial.println("tcp/ip stack initialization failed");
  }

  // Initialise Wi-Fi
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  err = esp_wifi_init(&cfg); 
  if (err == ESP_OK) {
    Serial.println("WiFi driver initialized");
  } else {
    Serial.printf("WiFi driver initialization failed: %d\n", err);
  }

  // get mac address 
  err = esp_wifi_get_mac(WIFI_IF_STA, mac);
  if (err == ESP_OK) {
    Serial.println("MAC address fetched from Wifi chipset");
  } else {
    Serial.printf("MAC retrieval failed: %d\n", err);
  }
  // remove wifi driver to free radio for BLE
  esp_wifi_deinit();
  Serial.println("WiFi driver de-initialized (radio freed)");
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  std::string macFull = macStr;
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
  // now working on audio threads
  //set I2S default values for acquiring 24 bits data from microphone

  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = I2S_SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,     // read on 32 bits, 24 bits are available
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,      // microphone is mono and sends data on left channel
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = 0,
      .dma_buf_count = 8,
      .dma_buf_len = 256,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0};

  i2s_pin_config_t pin_config = {           // change pin settings in .H header file if necessary (13,14,34)
      .bck_io_num = I2S_BCK_PIN,
      .ws_io_num = I2S_WS_PIN,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = I2S_SD_PIN};

  Serial.println("try to install I2S interface");
  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK)
  {
    Serial.printf("Erreur i2s_driver_install: %d\n", err);
    audioStarted = false;
  }
  else
  {
    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK)
    {
      Serial.printf("Erreur i2s_set_pin: %d\n", err);
      audioStarted = false;
    }
    else {
      audioStarted = true;
      Serial.println("I2S started, listening to INMP441");
    }
  };
  Serial.println("-> Start Sampling/Counting task");
  crackCounterStatus = true;
  xTaskCreatePinnedToCore(
      monitorAudioTask,   // Task function for audio monitoring, send to a separate thread on a dedicated cpu
      "MonitorAudioTask", // Name
      8192,               // Stack size (increased for FFT use)
      NULL,               // Parameter
      2,                  // Priority (to calibrate)
      &monitorTaskHandle, // Task handle
      1                   // Core 1 (Recommended for heavy lifting)
  );
  Serial.println("-> Sampling/Counting task started");
#endif

  Serial.println("initializing BLE");
  // Initialize the Device and Server
  bool initerr = NimBLEDevice::init(finalDeviceName);
  if (initerr)
    Serial.println("BLE init done");
  else
  {
    Serial.println("BLE init failed, abort setup");
    return; // stop initialization as there is no radio
  }
  NimBLEServer *server = NimBLEDevice::createServer();
  // Set the server callback handler
  server->setCallbacks(new ServerCallbacks());
  // Create the Services and Characteristics
  NimBLEService *envService = server->createService(SERVICE_UUID);
  envDataChar = envService->createCharacteristic(
      ENV_DATA_CHAR_UUID,
      NIMBLE_PROPERTY::READ | WRITE // The characteristic is only readable by the client
  );
  envDataChar->setCallbacks(new EnvironmentDataCallbacks());

  // if audio is there, add the audio characteristics 
#if defined(TILAUONAUDIO_H)
  envAudioChar = envService->createCharacteristic(
      ENV_AUDIO_CHAR_UUID,
      NIMBLE_PROPERTY::READ | WRITE // The characteristic is only readable by the client
  );
  envAudioChar->setCallbacks(new AudioDataCallbacks());
#endif

  if (envService->start())
    Serial.println("BLE server started");
  else {
    Serial.println("BLE server failed to start, aborting");
    return; // failed to start server, abort 
  }
    Serial.println(">>-------------------------------------------------------------------------------------");

  // now start advertising
  Serial.println("BLE advertising init");
  // Configure and Start Advertising
  pAdvertising = NimBLEDevice::getAdvertising();
  if (!pAdvertising->addServiceUUID(SERVICE_UUID)) {
    Serial.println("Advertising failed to add Service UUID, abort");
    return;
  }
  pAdvertising->setDiscoverableMode(BLE_GAP_DISC_MODE_GEN);
  pAdvertising->enableScanResponse(true); 
  pAdvertising->setName(finalDeviceName);
  Serial.printf("BLE advertising starting on %s\n",SERVICE_UUID);
  if (pAdvertising->start()) {
    Serial.println("Advertising started successfully");
  } else {
    Serial.println("Advertising failed to start!, abort");
    return;
  }
  Serial.printf("TilauScope Ambiant BLE service now fully ready, answer on nName: %s\n", finalDeviceName.c_str());
  Serial.println(">>-------------------------------------------------------------------------------------");
}

// to test /Users/thierrygluzman/Documents/Dev/btitop/.venv/bin/python /Users/thierrygluzman/Documents/Dev/btitop/scan_bluetooth.py

// --- Loop ---
void loop()
{
  // LED Blinking Management (Non-blocking)
  unsigned long currentMillis = millis();
  if (!deviceConnected)
  {
    // If device is not connected, blink the LED (indicates advertising/ready state)
    if (currentMillis - previousMillis >= BLINK_INTERVAL)
    {
      // Save the current time
      previousMillis = currentMillis;
      // Toggle the LED state
      int ledState = digitalRead(LED_PIN);
      digitalWrite(LED_PIN, !ledState);
    }
    if (currentMillis - previousMillis >= 10*BLINK_INTERVAL)
      if (pAdvertising->isAdvertising())
        Serial.println("advertising on");
      else
        Serial.println("advertising off");
  }
  else
  {
    // A device is connected: keep the LED ON steadily
    digitalWrite(LED_PIN, HIGH);
  } 
 /*if (currentMillis - previousMillis1 >= 5000) {
    previousMillis1 = currentMillis;
    bme_temp->getEvent(&temp_event);
    bme_pressure->getEvent(&pressure_event);
    bme_humidity->getEvent(&humidity_event);
    
    Serial.print(F("Temperature = "));
    Serial.print(temp_event.temperature);
    Serial.println(" *C");

    Serial.print(F("Humidity = "));
    Serial.print(humidity_event.relative_humidity);
    Serial.println(" %");

    Serial.print(F("Pressure = "));
    Serial.print(pressure_event.pressure);
    Serial.println(" hPa");
    Serial.println();
  }  */
}
