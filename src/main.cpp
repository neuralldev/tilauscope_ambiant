#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <NimBLEDevice.h>
#include <numeric> 
#include <WiFi.h>       
#include <algorithm>    
#include <string>       
#include <cmath> 

// --- Définitions ---
#define SEALEVELPRESSURE_HPA (1013.25)
#define SERVICE_UUID        "f3b6e2a0-8c4e-4e1f-9c2d-1a7f5b9a1c01"
#define ENV_DATA_CHAR_UUID  "f3b6e2a0-8c4e-4e1f-9c2d-1a7f5b9a1c05" 

#define LED_PIN 2             // Broche de la LED intégrée sur la plupart des ESP32 Dev Kits (parfois c'est 22 ou 16)
#define BLINK_INTERVAL 1000   // Intervalle de clignotement en millisecondes (1000 ms = 1 seconde)

#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_ADDRESS 0x76

// --- Variables Globales ---
Adafruit_BME280 bme;
NimBLECharacteristic* envDataChar;
NimBLEAdvertising* pAdvertising; 
int16_t loop_counter = 0;
bool deviceConnected = false;     
unsigned long previousMillis = 0; // Pour la gestion du temps non bloquante
int simulation = false;

// --- Structure de Données ---
typedef struct __attribute__((packed)) {
  uint16_t header = 0x5555; 
  int16_t temp_x10;         
  int16_t hum_x10;          
  int32_t press_x10;       
  int32_t alt_x10; 
  uint8_t checksum;         
  uint16_t footer = 0xAAAA; 
} EnvData;

// --- Fonction Utilitaire ---
uint8_t calculateChecksum(const uint8_t* data, size_t length) {
  uint8_t sum = 0;
  for (size_t i = 0; i < length; i++) {
    sum += data[i];
  }
  return sum;
}

// --- Callbacks de Caractéristique (Gestion READ) ---
class EnvironmentDataCallbacks: public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) {
    //Serial.println(">>> READ Request received. Updating data...");
    
    EnvData data;
    float temp_f = 0.0F;
    float hum_f = 0.0F;
    float press_f = 0.0F;
    float alt_f = 0.0F;

    if (simulation) {
      temp_f = 20.0 + sin(millis() / 5000.0) * 2.0; 
      hum_f = 77.1 + cos(millis() / 7000.0) * 3.0; 
      press_f = 990.4 + sin(millis() / 10000.0) * 5.0; 
      alt_f = 1013 + cos(millis() / 4000.0) * 2.5;
    } else {
      temp_f = bme.readTemperature();
      hum_f = bme.readHumidity();
      press_f = bme.readPressure() / 100.0F;
      alt_f = bme.readAltitude(SEALEVELPRESSURE_HPA);
    }

    data.temp_x10 = (int16_t)(temp_f * 10.0 + 0.5); 
    data.hum_x10 = (int16_t)(hum_f * 10.0 + 0.5);
    data.press_x10 = (int32_t)(press_f * 10.0 + 0.5);
    data.alt_x10 = (int32_t)(alt_f * 10.0 + 0.5);

    const uint8_t* dataPayload = (const uint8_t*)&data + sizeof(data.header);
    size_t payloadLength = sizeof(data.temp_x10) + sizeof(data.hum_x10) + sizeof(data.press_x10)+ sizeof(data.alt_x10);
    data.checksum = calculateChecksum(dataPayload, payloadLength);

    pCharacteristic->setValue((uint8_t*)&data, sizeof(EnvData));

    Serial.printf("T: %.1f °C | H: %.1f %% | P: %.1f hPa | A: %.1f masl | Checksum: 0x%02X\n", 
      (float)data.temp_x10 / 10.0, (float)data.hum_x10 / 10.0, (float)data.press_x10 / 10.0,  (float)data.alt_x10 / 10.0,data.checksum);
  }

    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        Serial.printf("%s : onWrite(), value: %s\n",
                      pCharacteristic->getUUID().toString().c_str(),
                      pCharacteristic->getValue().c_str());
    }

    /**
     *  The value returned in code is the NimBLE host return code.
     */
    void onStatus(NimBLECharacteristic* pCharacteristic, int code) override {
        Serial.printf("Notification/Indication return code: %d, %s\n", code, NimBLEUtils::returnCodeToString(code));
    }

    /** Peer subscribed to notifications/indications */
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

// --- Callbacks de Serveur (CLÉ : Gestion de la déconnexion et Timeout) ---
class ServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) {
        Serial.println("Client connected. Advertizing stopped.");
        
       pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 180);    }
    
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        Serial.printf("Client disconnected - start advertising\n");
        NimBLEDevice::startAdvertising();
    }

    void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override {
        Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
    }
};

// --- Setup ---
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("TilauScope Ambiant booting");

  // try the two addresses where BME is supposed to answer
  if (!bme.begin(BME280_ADDRESS)) 
    if (!bme.begin(BME280_ADDRESS_ALTERNATE)) 
      {
        Serial.println("BME280 not detected, errorr, enter simulation mode");
        simulation = true;
      }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Assurez-vous qu'elle est éteinte au départ
  
  //Serial.println("Shuting down wifi to use BLE");
  WiFi.mode(WIFI_MODE_NULL); 
  delay(100); 
  //Serial.println("wifi off");
  
  // 1. Génération du nom unique
  std::string macFull = WiFi.macAddress().c_str(); 
  std::string identifier = macFull.substr(macFull.length() - 5);
  identifier.erase(std::remove(identifier.begin(), identifier.end(), ':'), identifier.end());
  std::string baseName = "TilauScope-Ambiant-";
  std::string finalDeviceName = baseName + identifier;

  Serial.println("initializing BLE");

  // 2. Initialisation du Périphérique et du Serveur
  NimBLEDevice::init(finalDeviceName.c_str());
  NimBLEServer *server = NimBLEDevice::createServer();
  
  server->setCallbacks(new ServerCallbacks()); 
  
  // 3. Création du Service et de la Caractéristique
  NimBLEService *envService = server->createService(SERVICE_UUID);
  envDataChar = envService->createCharacteristic(
      ENV_DATA_CHAR_UUID,
      NIMBLE_PROPERTY::READ 
  );
  
  envDataChar->setCallbacks(new EnvironmentDataCallbacks());
  envService->start();
  
 // Serial.println("Service and Characteristic created.");

  // 4. Configuration et Démarrage de la Publicité
  pAdvertising = NimBLEDevice::getAdvertising(); 
  pAdvertising->addServiceUUID(SERVICE_UUID); 
  pAdvertising->setName(finalDeviceName.c_str()); 
  pAdvertising->enableScanResponse(true);
  pAdvertising->start(); 

  Serial.println("Advertising started");

  Serial.printf("BLE TilauScope Ambiant ready - Name: %s\n", finalDeviceName.c_str());
  
}

// --- Loop ---
void loop() {
 loop_counter++;
    // Gestion du clignotement de la LED (lorsque l'appareil fait de la publicité)
    if (!deviceConnected) {
        unsigned long currentMillis = millis();
        
        if (currentMillis - previousMillis >= BLINK_INTERVAL) {
            // Sauvegarder l'heure actuelle
            previousMillis = currentMillis;

            // Inverser l'état de la LED
            int ledState = digitalRead(LED_PIN);
            digitalWrite(LED_PIN, !ledState);
        }
    } else {
        // L'appareil est connecté : allumez la LED de manière stable
        digitalWrite(LED_PIN, HIGH);
    }
}