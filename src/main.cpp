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
#include <esp_system.h> // Include this header at the top of main.cpp
#include "monaudio.h"

// --- Definitions ---
// Standard atmospheric pressure at sea level in hPa (used for altitude calculation)
#define SEALEVELPRESSURE_HPA (1013.25)
// BLE Service UUID for the environmental data service
#define SERVICE_UUID "f3b6e2a0-8c4e-4e1f-9c2d-1a7f5b9a1c01"
// BLE Characteristic UUID for the environmental data
#define ENV_DATA_CHAR_UUID "f3b6e2a0-8c4e-4e1f-9c2d-1a7f5b9a1c05"

#define LED_PIN 2           // Built-in LED pin on most ESP32 Dev Kits
#define BLINK_INTERVAL 1000 // LED blink interval in milliseconds (1 second)

#define SDA_PIN 21 // I2C Data Pin for the BME280
#define SCL_PIN 22 // I2C Clock Pin for the BME280

// --- Global Variables for BME support ---
Adafruit_BME280 bme;               // BME280 sensor object
NimBLECharacteristic *envDataChar; // Pointer to the BLE Characteristic for ambiant data
NimBLEAdvertising *pAdvertising;   // Pointer to the BLE Advertising object

bool deviceConnected = false;
unsigned long previousMillis = 0; // For non-blocking timing (LED blink)
int simulation = false;           // Flag to indicate if simulation mode is active (sensor failure)

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
   * Called when a connected peer sends a WRITE request to the Characteristic.
   * Implemented for completeness, although this Characteristic is READ-only.
   */
  void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
  {
    Serial.printf("%s : onWrite(), value: %s\n",
                  pCharacteristic->getUUID().toString().c_str(),
                  pCharacteristic->getValue().c_str());
  }

  /**
   * Called when a notification/indication status is received from the BLE stack.
   */
  void onStatus(NimBLECharacteristic *pCharacteristic, int code) override
  {
    Serial.printf("Notification/Indication return code: %d, %s\n", code, NimBLEUtils::returnCodeToString(code));
  }

  /** Called when a peer subscribes or unsubscribes to notifications/indications. */
  void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue) override
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
  Serial.println(">>-------------------------------------------");
  Serial.println("TilauScope Ambiant booting");
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);
  // Attempt to initialize the BME280 sensor
  if (!bme.begin(BME280_ADDRESS))
    Serial.println("BME280 not detected on main address");
  // Try alternate I2C address if primary fails
  if (!bme.begin(BME280_ADDRESS_ALTERNATE))
  {
    Serial.println("BME280 not detected no secondary, enter simulation mode");
    simulation = true; // Set simulation flag if sensor initialization fails
  }
  delay(100);

  // Configure the LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Ensure LED is off initially
  Serial.println("LED configured to blink every second");

  esp_err_t err = esp_netif_init();
  if (err == ESP_OK)
  {
    Serial.println("tcp/ip stack initialized");
  }
  else
  {
    Serial.println("tcp/ip stack initialization failed");
  }

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  err = esp_wifi_init(&cfg); // Pass the address of the local struct
  if (err == ESP_OK)
  {
    Serial.println("WiFi driver initialized");
  }
  else
  {
    Serial.printf("WiFi driver initialization failed: %d\n", err);
  }

  const esp_netif_t *esp = esp_netif_create_default_wifi_sta();
  if (esp == NULL)
  {
    Serial.println("failed to create default wifi sta profile");
  }
  uint8_t mac[6];
  // 3. Get the MAC address from the created interface
  err = esp_wifi_get_mac(WIFI_IF_STA, mac);
  if (err == ESP_OK)
  {
    Serial.println("MAC address fetched from Wifi chipset");
  }
  else if (err == ESP_ERR_WIFI_NOT_INIT)
  {
    Serial.println("wifi not initialized, cannot fetch MAC");
  }
  else if (err == ESP_ERR_WIFI_IF)
  {
    Serial.println("invalid interface detected");
  }
  else if (err == ESP_ERR_INVALID_ARG)
  {
    Serial.println("invalid argument passed to MAC retrieval");
  };

  // Format the MAC address into a standard string
  char macStr[18] = {0}; // XX:XX:XX:XX:XX:XX + null terminator
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  std::string macFull = macStr;

  // 1. Generate a unique device name: "TilauScope-Ambiant-" + last 4 chars of MAC address (no colons)
  // This logic remains the same: it extracts the last 4 characters (e.g., :AB:CD) and removes the colon.
  std::string identifier = macFull.substr(macFull.length() - 8);
  identifier.erase(std::remove(identifier.begin(), identifier.end(), ':'), identifier.end());
  std::string baseName = "TilauScope-Amb-";
  std::string finalDeviceName = baseName + identifier;
  Serial.println(">>-------------------------------------------");
  Serial.printf("Device MAC: %s\n", macFull.c_str());
  Serial.printf("Device Name: %s\n", finalDeviceName.c_str());
  Serial.println(">>-------------------------------------------");

#if defined(TILAUONAUDIO_H)
  // now working on audio threads
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
      .bck_io_num = I2S_BCK_PIN,
      .ws_io_num = I2S_WS_PIN,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = I2S_SD_PIN};

  Serial.println("try to install I2S interface");

  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

  Serial.printf("checking I2S status (%d)\n", err);

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
  Serial.println("-> Start Sampling/Counting");
  crackCounterStatus = true;
  xTaskCreatePinnedToCore(
      monitorAudioTask,   // Task function
      "MonitorAudioTask", // Name
      8192,               // Stack size (increased for FFT)
      NULL,               // Parameter
      2,                  // Priority (higher than calibrate, since it's continuous)
      &monitorTaskHandle, // Task handle
      1                   // Core 1 (Recommended for heavy lifting)
  );
  Serial.println("-> Sampling/Counting started");

#endif

  Serial.println("initializing BLE");
  // 2. Initialize the Device and Server
  bool initerr = NimBLEDevice::init(finalDeviceName);
  if (initerr)
    Serial.println("BLE init done");
  else
  {
    Serial.println("BLE init failed");
    delay(5000);
  }

  NimBLEServer *server = NimBLEDevice::createServer();

  Serial.println("server BLE started");
  Serial.println(">>-------------------------------------------");

  // Set the server callback handler
  server->setCallbacks(new ServerCallbacks());

  // 3. Create the Services and Characteristics
  NimBLEService *envService = server->createService(SERVICE_UUID);
  envDataChar = envService->createCharacteristic(
      ENV_DATA_CHAR_UUID,
      NIMBLE_PROPERTY::READ | WRITE // The characteristic is only readable by the client
  );
  envDataChar->setCallbacks(new EnvironmentDataCallbacks());

#if defined(TILAUONAUDIO_H)
  envAudioChar = envService->createCharacteristic(
      ENV_AUDIO_CHAR_UUID,
      NIMBLE_PROPERTY::READ | WRITE // The characteristic is only readable by the client
  );
  envAudioChar->setCallbacks(new AudioDataCallbacks());

#endif

  envService->start();

  Serial.println("service advertising BLE init");

  // 4. Configure and Start Advertising
  pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID); // Advertise the main service UUID
  pAdvertising->setName(finalDeviceName);     // Set the advertising name
  pAdvertising->enableScanResponse(true);     // Allow more data in scan response (like the full name)

  Serial.println("service advertising BLE starting");

  uint16_t min_interval = 1600; // 1600 * 0.625ms = 1000ms (1 second)
  uint16_t max_interval = 1600; // Set min and max to the same value for a fixed interval

  // Use setInterval to set both min and max advertising interval
  pAdvertising->setAdvertisingInterval(min_interval);
  pAdvertising->start(); // Start broadcasting

  Serial.println("Advertising started");

  Serial.printf("BLE TilauScope Ambiant ready - Name: %s\n", finalDeviceName);
  Serial.println(">>-------------------------------------------");
}

// --- Loop ---
void loop()
{
  // LED Blinking Management (Non-blocking)
  if (!deviceConnected)
  {
    // If device is not connected, blink the LED (indicates advertising/ready state)
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= BLINK_INTERVAL)
    {
      // Save the current time
      previousMillis = currentMillis;
      // Toggle the LED state
      int ledState = digitalRead(LED_PIN);
      digitalWrite(LED_PIN, !ledState);
    }
  }
  else
  {
    // Device is connected: keep the LED ON steadily
    digitalWrite(LED_PIN, HIGH);
  }
}
