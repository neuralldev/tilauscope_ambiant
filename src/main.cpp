#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <NimBLEDevice.h>
#include <numeric> 
#include <WiFi.h>       // Used only for MAC address to generate unique device name
#include <algorithm>    // Used for string manipulation (std::remove)
#include <string>       
#include <cmath>        // Used for simulation functions (sin, cos)

// --- Definitions ---
// Standard atmospheric pressure at sea level in hPa (used for altitude calculation)
#define SEALEVELPRESSURE_HPA (1013.25) 
// BLE Service UUID for the environmental data service
#define SERVICE_UUID        "f3b6e2a0-8c4e-4e1f-9c2d-1a7f5b9a1c01"
// BLE Characteristic UUID for the environmental data 
#define ENV_DATA_CHAR_UUID  "f3b6e2a0-8c4e-4e1f-9c2d-1a7f5b9a1c05" 

#define LED_PIN 2             // Built-in LED pin on most ESP32 Dev Kits
#define BLINK_INTERVAL 1000   // LED blink interval in milliseconds (1 second)

#define SDA_PIN 21            // I2C Data Pin for the BME280
#define SCL_PIN 22            // I2C Clock Pin for the BME280
#define I2C_ADDRESS 0x76      // Primary I2C address for the BME280

// --- Global Variables ---
Adafruit_BME280 bme;            // BME280 sensor object
NimBLECharacteristic* envDataChar; // Pointer to the BLE Characteristic
NimBLEAdvertising* pAdvertising;   // Pointer to the BLE Advertising object
int16_t loop_counter = 0;
bool deviceConnected = false;     
unsigned long previousMillis = 0; // For non-blocking timing (LED blink)
int simulation = false;           // Flag to indicate if simulation mode is active (sensor failure)

// --- Data Structure ---
// Structure to hold environmental data for BLE transmission
typedef struct __attribute__((packed)) {
  uint16_t header = 0x5555;     // Start of message header
  int16_t temp_x10;             // Temperature multiplied by 10 (e.g., 25.4°C -> 254)
  int16_t hum_x10;              // Humidity multiplied by 10
  int32_t press_x10;            // Pressure multiplied by 10
  int32_t alt_x10;              // Altitude multiplied by 10
  uint8_t checksum;             // Simple additive checksum for integrity
  uint16_t footer = 0xAAAA;     // End of message footer
} EnvData;

// --- Utility Function ---
/**
 * Calculates a simple additive checksum for a given byte array.
 * @param data Pointer to the data array.
 * @param length Length of the data to include in the checksum.
 * @return The calculated 8-bit checksum.
 */
uint8_t calculateChecksum(const uint8_t* data, size_t length) {
  uint8_t sum = 0;
  for (size_t i = 0; i < length; i++) {
    sum += data[i];
  }
  return sum;
}

// --- Characteristic Callbacks (Handles READ request) ---
class EnvironmentDataCallbacks: public NimBLECharacteristicCallbacks {
  /**
   * Called when a connected peer sends a READ request to the Characteristic.
   * Reads sensor data (or simulated data) and prepares the EnvData structure.
   */
  void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) {
    //Serial.println(">>> READ Request received. Updating data...");
    
    EnvData data;
    float temp_f = 0.0F;
    float hum_f = 0.0F;
    float press_f = 0.0F;
    float alt_f = 0.0F;

    // Read or simulate data
    if (simulation) {
      // Sinusoidal simulation for testing without a sensor
      temp_f = 20.0 + sin(millis() / 5000.0) * 2.0; 
      hum_f = 77.1 + cos(millis() / 7000.0) * 3.0; 
      press_f = 990.4 + sin(millis() / 10000.0) * 5.0; 
      alt_f = 1013 + cos(millis() / 4000.0) * 2.5;
    } else {
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
    const uint8_t* dataPayload = (const uint8_t*)&data + sizeof(data.header);
    // Determine the length of the data fields included in the checksum
    size_t payloadLength = sizeof(data.temp_x10) + sizeof(data.hum_x10) + sizeof(data.press_x10)+ sizeof(data.alt_x10);
    data.checksum = calculateChecksum(dataPayload, payloadLength);

    // Set the characteristic value with the entire structure
    pCharacteristic->setValue((uint8_t*)&data, sizeof(EnvData));

    // Print current values to the Serial Monitor
    Serial.printf("T: %.1f °C | H: %.1f %% | P: %.1f hPa | A: %.1f masl | Checksum: 0x%02X\n", 
      (float)data.temp_x10 / 10.0, (float)data.hum_x10 / 10.0, (float)data.press_x10 / 10.0,  (float)data.alt_x10 / 10.0,data.checksum);
  }

    /**
     * Called when a connected peer sends a WRITE request to the Characteristic.
     * Implemented for completeness, although this Characteristic is READ-only.
     */
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        Serial.printf("%s : onWrite(), value: %s\n",
                      pCharacteristic->getUUID().toString().c_str(),
                      pCharacteristic->getValue().c_str());
    }

    /**
     * Called when a notification/indication status is received from the BLE stack.
     */
    void onStatus(NimBLECharacteristic* pCharacteristic, int code) override {
        Serial.printf("Notification/Indication return code: %d, %s\n", code, NimBLEUtils::returnCodeToString(code));
    }

    /** Called when a peer subscribes or unsubscribes to notifications/indications. */
    void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override {
        std::string str  = "Client ID: ";
        str             += connInfo.getConnHandle();
        str             += " Address: ";
        str             += connInfo.getAddress().toString();
        if (subValue == 0) {
            str += " Unsubscribed to ";
        } else if (subValue == 1) {
            str += " Subscribed to notifications for ";
        } else if (subValue == 2) {
            str += " Subscribed to indications for ";
        } else if (subValue == 3) {
            str += " Subscribed to notifications and indications for ";
        }
        str += std::string(pCharacteristic->getUUID());

        Serial.printf("%s\n", str.c_str());
    }
};

// --- Server Callbacks (Handles Connection/Disconnection) ---
class ServerCallbacks: public NimBLEServerCallbacks {
    /**
     * Called when a BLE client connects to the server.
     */
    void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) {
        Serial.println("Client connected. Advertising stopped.");
        deviceConnected = true; // Update global state
        
        // Update connection parameters for potentially faster communication (optional)
        pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 180);    }
    
    /**
     * Called when a BLE client disconnects from the server.
     */
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        Serial.printf("Client disconnected - start advertising\n");
        deviceConnected = false; // Update global state
        NimBLEDevice::startAdvertising(); // Restart advertising so other devices can connect
    }

    /**
     * Called when the Maximum Transmission Unit (MTU) is negotiated.
     */
    void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override {
        Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
    }
};

// --- Setup ---
void setup() {
  Serial.begin(115200);
  // Initialize I2C (Wire) with custom pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("TilauScope Ambiant booting");

  // Attempt to initialize the BME280 sensor
  if (!bme.begin(BME280_ADDRESS)) 
    // Try alternate I2C address if primary fails
    if (!bme.begin(BME280_ADDRESS_ALTERNATE)) 
      {
        Serial.println("BME280 not detected, errorr, enter simulation mode");
        simulation = true; // Set simulation flag if sensor initialization fails
      }

  // Configure the LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Ensure LED is off initially
  
  // Disable WiFi to free up resources and avoid potential conflicts with BLE
  //Serial.println("Shuting down wifi to use BLE");
  WiFi.mode(WIFI_MODE_NULL); 
  delay(100); 
  //Serial.println("wifi off");
  
  // 1. Generate a unique device name: "TilauScope-Ambiant-" + last 4 chars of MAC address (no colons)
  std::string macFull = WiFi.macAddress().c_str(); 
  std::string identifier = macFull.substr(macFull.length() - 5);
  // Remove the colon from the identifier (e.g., ":AB:CD" -> "ABCD")
  identifier.erase(std::remove(identifier.begin(), identifier.end(), ':'), identifier.end());
  std::string baseName = "TilauScope-Ambiant-";
  std::string finalDeviceName = baseName + identifier;

  Serial.println("initializing BLE");

  // 2. Initialize the Device and Server
  NimBLEDevice::init(finalDeviceName.c_str());
  NimBLEServer *server = NimBLEDevice::createServer();
  
  // Set the server callback handler
  server->setCallbacks(new ServerCallbacks()); 
  
  // 3. Create the Service and Characteristic
  NimBLEService *envService = server->createService(SERVICE_UUID);
  envDataChar = envService->createCharacteristic(
      ENV_DATA_CHAR_UUID,
      NIMBLE_PROPERTY::READ // The characteristic is only readable by the client
  );
  
  // Set the characteristic callback handler (to handle READ requests)
  envDataChar->setCallbacks(new EnvironmentDataCallbacks());
  envService->start();
  
 // Serial.println("Service and Characteristic created.");

  // 4. Configure and Start Advertising
  pAdvertising = NimBLEDevice::getAdvertising(); 
  pAdvertising->addServiceUUID(SERVICE_UUID); // Advertise the main service UUID
  pAdvertising->setName(finalDeviceName.c_str()); // Set the advertising name
  pAdvertising->enableScanResponse(true); // Allow more data in scan response (like the full name)
  pAdvertising->start(); // Start broadcasting

  Serial.println("Advertising started");

  Serial.printf("BLE TilauScope Ambiant ready - Name: %s\n", finalDeviceName.c_str());
  
}

// --- Loop ---
void loop() {
 loop_counter++;
    // LED Blinking Management (Non-blocking)
    if (!deviceConnected) {
        // If device is not connected, blink the LED (indicates advertising/ready state)
        unsigned long currentMillis = millis();
        
        if (currentMillis - previousMillis >= BLINK_INTERVAL) {
            // Save the current time
            previousMillis = currentMillis;

            // Toggle the LED state
            int ledState = digitalRead(LED_PIN);
            digitalWrite(LED_PIN, !ledState);
        }
    } else {
        // Device is connected: keep the LED ON steadily
        digitalWrite(LED_PIN, HIGH);
    }
}
