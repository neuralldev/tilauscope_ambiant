# tilauscope_ambiant
TilauScope Ambiant probe support

# 📡 ESP32 + BME280 BLE Environmental Monitor

Ce projet utilise un **ESP32 Weil** pour lire les données environnementales (température, humidité, pression) via un capteur **BME280** en I2C, et les diffuser en **Bluetooth Low Energy (BLE)** avec des **UUID personnalisés**. Il est conçu pour s’intégrer facilement dans des workflows comme **Artisan** pour le suivi de la torréfaction.

---

## 🔧 Matériel

- ESP32 Weil
- BME280 (mode I2C)

### Câblage

| BME280 | ESP32 Weil | Fonction         |
|--------|-------------|------------------|
| VCC    | 3.3V        | Alimentation     |
| GND    | GND         | Masse            |
| SDA    | GPIO 21     | Données série    |
| SCL    | GPIO 22     | Horloge série    |

---

## 🧬 UUID BLE personnalisés

| Caractéristique | UUID                                     |
|-----------------|------------------------------------------|
| Service         | `f3b6e2a0-8c4e-4e1f-9c2d-1a7f5b9a1c01`   |
| data            | `f3b6e2a0-8c4e-4e1f-9c2d-1a7f5b9a1c05`   |

---

## ⚙️ Compilation avec PlatformIO

1. Cloner le dépôt :
   ```bash
   git clone https://github.com/tonpseudo/esp32-bme280-ble.git
   cd esp32-bme280-ble
