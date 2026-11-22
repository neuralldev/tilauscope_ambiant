#include <Wire.h>
#ifndef TILAU_AUDIO_ONLY
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#endif
#include <NimBLEDevice.h>
#include <numeric>
#include <WiFi.h>   // Used only for MAC address to generate unique device name
#include <algorithm>   // Used for string manipulation (std::remove)
#include <string>
#include <cmath> // Used for simulation functions (sin, cos)
#include "door.h"
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

#define BLINK_INTERVAL 1000 // LED blink interval in milliseconds (1 second)

// --- LED de statut (board-dependent) ---
//  - ESP32-S3 DevKitC-1 : WS2812 RGB intégrée sur GPIO48 (couleurs via neopixelWrite)
//  - ESP32-WROOM-32D     : LED simple monochrome sur GPIO2 (allumée si couleur != noir)
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #define RGB_LED_PIN 48
  static inline void statusLedBegin() {}
  static inline void statusLed(uint8_t r, uint8_t g, uint8_t b) { neopixelWrite(RGB_LED_PIN, r, g, b); }
#else
  #define STATUS_LED_PIN 2
  static inline void statusLedBegin() { pinMode(STATUS_LED_PIN, OUTPUT); }
  static inline void statusLed(uint8_t r, uint8_t g, uint8_t b) { digitalWrite(STATUS_LED_PIN, (r | g | b) ? HIGH : LOW); }
#endif

#ifndef TILAU_AUDIO_ONLY
// Câblage BME280 (I2C, 3.3V) sur ESP32-S3 DevKitC-1 + terminal adapter :
//   VIN  (fil bleu)   -> 3V3   (bloc gauche, borne 3.3V)
//   GND  (fil vert)   -> GND   (bloc gauche, borne GND)
//   SCL  (fil jaune)  -> GPIO9 (bloc gauche, borne IO9)
//   SDA  (fil orange) -> GPIO8 (bloc gauche, borne IO8)
#define SDA_PIN 8 // I2C Data Pin for the BME280  (fil orange)
#define SCL_PIN 9 // I2C Clock Pin for the BME280 (fil jaune)
#endif

// --- Global Variables for BME support ---
NimBLEAdvertising *pAdvertising;   // Pointer to the BLE Advertising object

volatile uint8_t connectedClientCount = 0;
unsigned long previousMillis = 0;  // For non-blocking timing (LED blink)
unsigned long previousMillis1 = 0; // For non-blocking tempo display
bool previous = true;              // used in test mode only

#ifndef TILAU_AUDIO_ONLY
Adafruit_BME280 bme;               // BME280 sensor object
NimBLECharacteristic *envDataChar; // Pointer to the BLE Characteristic for ambiant data
bool simulation = false;           // Flag to indicate if simulation mode is active (sensor failure)

Adafruit_Sensor *bme_temp;
Adafruit_Sensor *bme_pressure;
Adafruit_Sensor *bme_humidity;
#endif

#ifndef TILAU_AUDIO_ONLY
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

    // Convert float to fixed-point integer (x10). lroundf arrondit aussi
    // correctement les valeurs negatives (+0.5 puis troncature ne le fait pas).
    // Mesure invalide (NaN/inf) : convertir NaN en entier est un comportement
    // indefini, le client recevrait des valeurs arbitraires avec un checksum
    // valide. On envoie une sentinelle sur les quatre champs ; temp_x10 =
    // INT16_MIN sort de la plage plausible de tilauambient.py, qui rejette
    // alors toute la lecture.
    if (isfinite(temp_f) && isfinite(hum_f) && isfinite(press_f) && isfinite(alt_f) &&
        fabsf(temp_f) < 3000.0f && fabsf(hum_f) < 3000.0f)
    {
      data.temp_x10 = (int16_t)lroundf(temp_f * 10.0f);
      data.hum_x10 = (int16_t)lroundf(hum_f * 10.0f);
      data.press_x10 = (int32_t)lroundf(press_f * 10.0f);
      data.alt_x10 = (int32_t)lroundf(alt_f * 10.0f);
    }
    else
    {
      data.temp_x10 = INT16_MIN;
      data.hum_x10 = INT16_MIN;
      data.press_x10 = INT32_MIN;
      data.alt_x10 = INT32_MIN;
    }

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
#endif  // !TILAU_AUDIO_ONLY

// --- Server Callbacks (Handles Connection/Disconnection) ---
class ServerCallbacks : public NimBLEServerCallbacks
{
  /**
   * Called when a BLE client connects to the server.
   */
  void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo)
  {
    connectedClientCount = pServer->getConnectedCount();
    Serial.printf("BLE client connected: handle=%u address=%s clients=%u/%u\n",
                  connInfo.getConnHandle(), connInfo.getAddress().toString().c_str(),
                  connectedClientCount, CONFIG_BT_NIMBLE_MAX_CONNECTIONS);

    // Update connection parameters for potentially faster communication (optional)
    pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 180);

    // Une connexion BLE arrête normalement l'advertising. Le relancer tant
    // qu'une place reste libre permet à TilauScope et au banc de se connecter.
    if (connectedClientCount < CONFIG_BT_NIMBLE_MAX_CONNECTIONS &&
        !pAdvertising->isAdvertising()) {
      if (NimBLEDevice::startAdvertising())
        Serial.println("BLE advertising continues for a second client.");
      else
        Serial.println("BLE warning: unable to resume advertising for second client.");
    }
  }

  /**
   * Called when a BLE client disconnects from the server.
   */
  void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override
  {
    connectedClientCount = pServer->getConnectedCount();
    Serial.printf("BLE client disconnected: handle=%u reason=%d clients=%u/%u\n",
                  connInfo.getConnHandle(), reason, connectedClientCount,
                  CONFIG_BT_NIMBLE_MAX_CONNECTIONS);

    // Après toute déconnexion il existe au moins une place. L'autre client,
    // s'il est encore connecté, reste pleinement opérationnel.
    if (!pAdvertising->isAdvertising() && !NimBLEDevice::startAdvertising())
      Serial.println("BLE warning: unable to restart advertising.");
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
#if defined(TILAU_AUDIO_STREAM)
  Serial.setTxBufferSize(1024);   // marge pour les frames audio (261 B/hop)
  Serial.begin(921600);           // 115200 ne tient pas le débit du stream audio brut
#else
  Serial.begin(115200);
#endif
  // Activer le niveau DEBUG pour le tag AUDIO sur le port debug-console
  // Permet de voir ESP_LOGD(TAG_MON, ...) depuis /dev/cu.debug-console
  esp_log_level_set("AUDIO", ESP_LOG_DEBUG);
  esp_log_level_set("CAL",   ESP_LOG_DEBUG);
  esp_log_level_set("MON",   ESP_LOG_DEBUG);
  Serial.println(">>-------------------------------------------------------------------------------------");
  Serial.println("TilauScope Ambiant booting");
#ifndef TILAU_AUDIO_ONLY
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100 kHz Standard I2C speed for better stability
  delay(100);
  // Diagnostic câblage : scan du bus I2C + chip ID (0x60 = BME280, 0x58 = BMP280)
  {
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++)
    {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() != 0)
        continue;
      found++;
      int chipId = -1;
      if (addr == 0x76 || addr == 0x77)
      {
        Wire.beginTransmission(addr);
        Wire.write(0xD0); // chip ID register
        if (Wire.endTransmission() == 0 && Wire.requestFrom(addr, (uint8_t)1) == 1)
          chipId = Wire.read();
      }
      Serial.printf("I2C device at 0x%02X%s", addr, chipId >= 0 ? "" : "\n");
      if (chipId >= 0)
        Serial.printf(" chip ID 0x%02X (%s)\n", chipId,
                      chipId == 0x60 ? "BME280" : chipId == 0x58 ? "BMP280" : "inconnu");
    }
    if (!found)
      Serial.printf("I2C scan: aucun périphérique sur SDA=%d SCL=%d (câblage/alim ?)\n", SDA_PIN, SCL_PIN);
  }
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
  doorBegin();
#else
  Serial.println("Build audio-only : BME280 désactivé.");
#endif  // !TILAU_AUDIO_ONLY

  // LED de statut — éteinte au boot (WS2812 sur S3, LED simple sur WROOM-32D)
  statusLedBegin();
  statusLed(0, 0, 0);
  Serial.println("Status LED configured");

  // Disable WiFi to free the radio for BLE — the MAC stays readable from efuse
  WiFi.mode(WIFI_MODE_NULL);
  delay(100);
  std::string macFull = WiFi.macAddress().c_str();
  // generate unique name based on MAC address
  // Garde : sur echec de lecture efuse, macAddress() renvoie une chaine vide et
  // `length() - 8` deborde en size_t -> substr leve out_of_range, soit un
  // abort() en boucle de boot avec le -fno-exceptions du core Arduino.
  std::string identifier;
  if (macFull.length() >= 8) {
    identifier = macFull.substr(macFull.length() - 8);
    identifier.erase(std::remove(identifier.begin(), identifier.end(), ':'), identifier.end());
  } else {
    Serial.println("MAC unreadable, falling back to a generic device name");
    identifier = "000000";
  }
  std::string baseName = "TLSCAM";
  std::string finalDeviceName = baseName + identifier;
  Serial.println(">>-------------------------------------------------------------------------------------");
  Serial.printf("Device MAC: %s\n", macFull.c_str());
  Serial.printf("Device Name: %s\n", finalDeviceName.c_str());
  Serial.println(">>-------------------------------------------------------------------------------------");

#if defined(TILAUONAUDIO_H)

  Serial.println("try to install audio interface");
  audioStarted = (audioHwInit() == ESP_OK);

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
    Serial.println("Boot dual calibration restored (EMPTY + MAILLARD) - ready to START.");
  } else {
    Serial.println("Dual calibration incomplete - EMPTY then MAILLARD required before START.");
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
#ifndef TILAU_AUDIO_ONLY
  envDataChar = envService->createCharacteristic(
      ENV_DATA_CHAR_UUID,
      NIMBLE_PROPERTY::READ // The characteristic is only readable by the client
  );
  envDataChar->setCallbacks(new EnvironmentDataCallbacks());
  doorAttach(envService);
#endif

  // if audio is there, add the audio characteristics
#if defined(TILAUONAUDIO_H)
  envAudioChar = envService->createCharacteristic(
      ENV_AUDIO_CHAR_UUID,
      NIMBLE_PROPERTY::READ | WRITE // READ : crack counter | WRITE : commandes audio
  );
  envAudioChar->setCallbacks(new AudioDataCallbacks());
#endif

  // NimBLE 2.x : NimBLEService::start() est un no-op (déprécié). Les services
  // sont activés au démarrage du serveur -> on démarre le serveur explicitement.
  server->start();
  Serial.println("BLE server started");
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
  int mode = calibrating ? 1 : (connectedClientCount == 0 ? 0 : (crackCounterStatus ? 3 : 2));

  if (mode == 1) { // calibration : rouge clignotant rapide (250 ms)
    if (now - lastToggle >= 250) {
      lastToggle = now; blinkOn = !blinkOn;
      statusLed(blinkOn ? 80 : 0, 0, 0);
    }
  } else if (mode == 0) { // advertising : bleu clignotant 1 Hz
    if (now - lastToggle >= BLINK_INTERVAL) {
      lastToggle = now; blinkOn = !blinkOn;
      statusLed(0, 0, blinkOn ? 60 : 0);
    }
  } else if (mode != lastMode) { // états fixes : écrire une seule fois (pas à chaque loop)
    if (mode == 3) statusLed(0, 40, 40); // cyan = détection active
    else           statusLed(0, 60, 0);  // vert = connecté
  }
  lastMode = mode;
}

// --- Loop ---
void loop()
{
  // LED Blinking Management (Non-blocking)
  unsigned long currentMillis = millis();
  // selftestRequested : l'auto-test d'acquisition (COMMAND_AUDIO_SELFTEST) doit
  // pouvoir tourner AVANT toute calibration -- c'est meme son seul interet.
  // On cree donc aussi la tache dans ce cas ; elle reste ensuite en idle tant
  // que crackCounterStatus est faux.
  // monitorAudioFatal : la tache s'est abandonnee sur un echec d'init (FFT ou
  // buffer DMA). Les conditions ci-dessous restent vraies, donc sans ce garde
  // on la recreerait des milliers de fois par seconde.
  // monitorRetryAt : meme probleme pour un echec de xTaskCreate (heap epuisee),
  // sauf que celui-la est transitoire -> nouvelle tentative toutes les 2 s.
  static unsigned long monitorRetryAt = 0;
  // Course BLE (coeur 0) / tache monitor (coeur 1) : un SELFTEST peut passer
  // le test monitorAudioFatal du callback juste avant que la tache meure.
  // Personne d'autre ne l'executera : on le refuse ici.
  if (monitorAudioFatal)
    audioSelftestAbort("monitor_fatal");
  if ((monitorTaskHandle == NULL) && audioStarted && !monitorAudioFatal &&
      (isCalibrated || selftestRequested) && (long)(currentMillis - monitorRetryAt) >= 0)
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
      monitorRetryAt = currentMillis + 2000;
      // Un auto-test en attente bloquerait toute calibration jusqu'a ce que la
      // tache existe : on le refuse tout de suite, l'hote peut le redemander.
      audioSelftestAbort("task_create");
    }
  }
#if defined(TILAUONAUDIO_H)
  audioPendingCalibrationService();   // barriere I2S + creation de calibrateTask
  audioPendingSaveService();          // ecriture LittleFS demandee par un callback BLE
#endif
#ifndef TILAU_AUDIO_ONLY
  doorService();
#endif
  updateStatusLed();
}
