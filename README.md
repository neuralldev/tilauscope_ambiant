# tilauscope_ambiant

TilauScope Ambiant probe support

## ESP32 + BME280 BLE Environmental Monitor

The TilauScope Ambiant is an ESP32-based device acting as a Bluetooth Low Energy (BLE) peripheral to broadcast environmental data (Temperature, Humidity, Pressure, and Altitude). It utilizes the Adafruit BME280 sensor for measurements.

## Features

**Wireless Data**: Uses BLE (NimBLE stack) for low-power, short-range data transmission.
**BME280 Sensor Integration**: Reads Temperature, Humidity, Pressure, and Altitude.
**Unique Naming**: Generates a unique device name based on the ESP32's MAC address (TilauScope-Ambiant-XXXX).
**Efficient Data Format**: Transmits all sensor data in a single, compact, 17-byte packed structure with fixed-point scaling and a checksum.
**Status Indicator**: Uses the onboard LED for connection and advertising status.
**Robustness**: Includes a Simulation Mode fallback if the BME280 sensor is not detected.

## Hardware Requirements

|Component              |Function                |Configuration                                 |
|-----------------------|------------------------|----------------------------------------------|
|Microcontroller        |ESP32 Development Board |NimBLE Stack                                  |
|Environmental Sensor   |Adafruit BME280 (I2C)   |Addresses: 0x76 (Primary) or 0x77 (Alternate) |
|I2C SDA Pin            |Data line for BME280    |GPIO 21                                       |
|I2C SCL Pin            |Clock line for BME280   |GPIO 22                                       |

## software and dependencies

This project is built for the Arduino IDE or PlatformIO, targeting the ESP32.

## How to flash

Flash using any utility. It must be done with SPI DIO Mode, not QIO. bootloader must be flashed first, then reset the module it will put itself in flashing mode and flash partitiontable, reboot it again and firmware.bin. 

| File.                | Adres       |
| -------------------- | ----------- |
| `bootloader.bin`     | **0x1000**  | 
| `partitiontable.bin` | **0x8000**  | 
| `firmware.bin`       | **0x10000** | 

### Required Librarie

To work on the code, install the following libraries via the Arduino Library Manager or PlatformIO Registry:

- NimBLE-Arduino (or ESP32 BLE Library): For the Bluetooth Low Energy stack.
- Adafruit BME280 Library
- Adafruit Unified Sensor Library
- ArduinoJson Library (for calibration persistence)
- ESP-DSP (for FFT and biquad filtering)

### Data Structure and Format

Data of ambiant information is packaged into a (17-bytes) struct (EnvData) using fixed-point representation (x10 scaling) for efficiency. Data of audio information is packaged into a (7-bytes) struct (AudioData) using int16 representation.
Data of audio information is packaged into a (21-bytes) struct (AudioData) using `int32_t` representation for all the values.

### Decoding the Data

To retrieve the actual floating-point values, the received integer value must be divided by x 10.Checksum Verification: The checksum is calculated over the (12-byte) data payload (from temp_x10 to alt_x10). Clients should re-calculate this checksum and compare it to the received checksum field to ensure data integrity. all the values are coded in Little-Endian

Ambiant payload is structured this way:
| HEADER    | temp 28.8°C | hum 77.1%.  | press 990.1 mPa | alt 30masl   | checksum | footer   |
|-----------|-------------|-------------|-----------------|--------------|----------|----------|
| 0x55 0x55 | 00 00 01 20 | 00 00 03 03 | 00 00 26 AD     |  00 00 00 1E |    nn    | 0xaa 0xaa|

Audio payload to retrieve contains the actual crack counter when you read the audio characteristic

| HEADER    | crack count | checksum | footer   |
|-----------|-------------|----------|----------|
| 0x55 0x55 | 00 00 00 10 |    nn    | 0xaa 0xaa|

(crack count is int32 structure, here it is 0x10 -> 17 in decimal)

when writing to the audio characteristic, sending a packet structure can trigger the crack counter logic:

| HEADER    | feature.    | checksum | footer   |function                                                                  |
|-----------|-------------|----------|-------------------------------------------------------------------------------------|
| 0x55 0x55 | 00 00       |    nn    | 0xaa 0xaa| Run calibration. the process last after 30 seconds                       |
|                                               | function 1 and 2 are ignored until calibration has been successfully run |
|-----------|-------------|----------|----------|--------------------------------------------------------------------------|
| 0x55 0x55 | 00 01       |    nn    | 0xaa 0xaa| ask the probe to start to detect cracks, the counter is incremented      |
|-----------|-------------|----------|----------|--------------------------------------------------------------------------|
| 0x55 0x55 | 00 02       |    nn    | 0xaa 0xaa| ask the probe to stop to detect cracks, the counter is reset to 0        |
|-----------|-------------|----------|----------|--------------------------------------------------------------------------|
| 0x55 0x55 | 00 06       |    nn    | 0xaa 0xaa| raise ratio of detection by 0.1                                          |
|-----------|-------------|----------|----------|--------------------------------------------------------------------------|
| 0x55 0x55 | 00 07       |    nn    | 0xaa 0xaa| decrease ratio of detection by 0.1                                       |
|-----------|-------------|----------|----------|--------------------------------------------------------------------------|
| 0x55 0x55 | 05 06       |    nn    | 0xaa 0xaa| raise ratio of detection by 0.5                                          |
|-----------|-------------|----------|----------|--------------------------------------------------------------------------|
| 0x55 0x55 | 05 07       |    nn    | 0xaa 0xaa| decrease ratio of detection by 0.5                                       |
|-----------|-------------|----------|----------|--------------------------------------------------------------------------|
| 0x55 0x55 | 00 10       |    nn    | 0xaa 0xaa| set debug on serial port, else save cpu for work                         |
|-----------|-------------|----------|----------|--------------------------------------------------------------------------|
| 0x55 0x55 | 00 11       |    nn    | 0xaa 0xaa| stop debug on serial port.                                               |
|-----------|-------------|----------|----------|--------------------------------------------------------------------------|
 
(The `command` field is a 2-byte `int16_t` value. Write to GATT requires asking for a response (response=True) if awaited.)

### Operation & Status Indicators

The built-in LED **GPIO2** provides a visual indication of the device's state. When the device is correctly started, it should blink once per second.

To operate, use the following:

- Wait for the probe to boot (automatic using TilauScope fork).
- Try to detect the probe on the service UUID (automatic using TilauScope fork).
- Start collecting data by reading the Ambiant UUID (automatic using TilauScope fork, mapping an extra device to the 4 values).
- At DE (Development Environment), call CALIBRATION (function 0) by writing to the Audio UUID.
- At DE+1mn, call START DETECTION (function 1) by writing to the Audio UUID.
- At DROP, call STOP DETECTION (function 2) by writing to the Audio UUID.

Calibration is stored in a file on the ESP32 file system. When the probe starts, it loads any previously saved calibration to skip this stage if the roast is going to be done under the same conditions (microphone placement, roaster at the same place, exhaust system at the same distance). We encourage you to take note of the exact placement of devices because of the nature of wave propagation, which can change a lot when shifting a device by a few centimeters. Calibration lasts for 30 seconds or so. When trying to find the right place, monitor your serial port to observe the detected signal shape and intensity. There are a lot of debug messages with graphical views to help you fine-tune the optimal position.

## Simulation Mode

If the device fails to initialize the BME280 sensor (e.g., wiring error or sensor not present), it will automatically switch to Simulation Mode.

    - Function: The onRead callback will generate smooth, sinusoidal, dummy environmental data instead of reading the sensor.

    - Purpose: Allows for testing the BLE connectivity and data parsing logic of the client application without requiring the physical sensor.

    - Serial Output: The device will log the message: BME280 not detected, errorr, enter simulation mode during setup.

### cabling information

| BME280 | ESP32 WROOM | Fonction          |
|--------|-------------|-------------------|
| VCC    | 3.3V -5V    | Power             |
| GND    | GND         | Ground            |
| SDA    | GPIO 21     | Data              |
| SCL    | GPIO 22     | Clock             |       Example of cabling for microphone and proposed color coding for cable 

|NMP441     | ESP32    | Fonction          |       Green    L/R                   GND  Brown
|-----------|----------|-------------------|
|VDD        | 3.3 V    | Power             |       Yellow    WS                   VDD  Red
|GND        | GND      | Ground            |
|WS (LRCL)  | GPIO 10  | Word Select       |       Orange    SCK    MICROPHONE     SD  Black
|SCK (BCLK) | GPIO 11  | Bit Clock         |
|SD (DOUT)  | GPIO 25  | Data              |               ------  FACE VIEW ------
|L/R        | GND      | Left channel only |

[esp32 module](esp32.png)
[bme280 probe](bme280.png)
[nmp441](nmp441.png)

---

## UUID BLE used by project

| Caractéristique | UUID                                   |
|-----------------|----------------------------------------|
| Service         | `f3b6e2a0-8c4e-4e1f-9c2d-1a7f5b9a1c01` |
| ambiant         | `f3b6e2a0-8c4e-4e1f-9c2d-1a7f5b9a1c05` |
| audio           | `f3b6e2a0-8c4e-4e1f-9c2d-1a7f5b9a1c04` |

The **service** UUID is used only for advertising.
Data exchange for environmental data is done using the **ambiant** UUID, and for audio data using the **audio** UUID. The ambiant data characteristic only accepts READ requests to query for values. The audio characteristic accepts both READ and WRITE operations. The payload is returned and must be unpacked.

## explanation on Audio-Based First Crack Detection: Algorithm Overview

1. Problem Context

During coffee roasting, first crack (FC) is a critical thermophysical event corresponding to rapid water vapor expansion and structural fracturing of the bean. Acoustically, it manifests as short, high-energy impulsive sounds (sharp “pops”) embedded in a background of continuous noise generated by the roaster (drum rotation, airflow, burner, cooling fan).

The objective of the algorithm is to reliably detect the onset of first crack in real time using a microphone, despite a noisy and thermally harsh environment, and to do so with low latency suitable for roast control.

The raw audio signal undergoes several preprocessing steps: 
 
    - Band-pass filtering. First crack energy is concentrated in the mid-to-high frequency range (typically ~1–6 kHz). Low-frequency components (drum rotation, airflow rumble) and very high-frequency noise are attenuated.

    - Framing and windowing. Audio is processed in short, overlapping frames (e.g., 10–50 ms). This allows time-localized analysis of impulsive events.

    - Amplitude normalization / AGC (optional). Compensates for gradual changes in overall noise level as airflow or burner power changes.

For each audio frame, the algorithm computes features that emphasize impulsive, crack-like events:
 
    - Short-Time Energy (STE). Measures sudden bursts of acoustic energy.

    - Spectral representation. Captures rapid changes in the frequency spectrum, characteristic of cracks.

    - Zero-Crossing Rate (ZCR). Higher for sharp transient signals than for steady mechanical noise.

    - High-frequency energy ratio. Ratio of energy above a given frequency threshold versus total energy.

The core detection mechanism combines thresholding and temporal validation:
 
    - Adaptive thresholding. Thresholds are not fixed. A rolling baseline is computed from recent frames to model current background noise. A frame is marked as a candidate crack if one or more features exceed the baseline by a defined margin.

    - Impulse validation. Candidate events must be short in duration, isolated (not sustained noise), and spectrally consistent with crack signatures.

    - Burst detection. First crack is not a single event but the start of a statistically significant increase in crack rate. The algorithm tracks crack counts over a sliding time window. First crack is declared when the event density exceeds a minimum rate for a sustained period (e.g., several cracks within a few seconds).

In practice, the detector is implemented as a state machine:
 
    - Idle / Pre-FC. Monitoring noise baseline; isolated pops are ignored.

    - FC Candidate. Increased impulsive activity is detected; waiting for confirmation.

    - First Crack Confirmed. Crack rate exceeds threshold; timestamp is latched and reported.

Once first crack is detected:

    - A timestamp (relative or absolute) is generated. This event can be:
        - Logged for roast profiling.
        - Sent via serial, MQTT, or REST to a roast controller.
        - Used to trigger automated roast phase transitions (e.g., power or airflow changes).
    - The algorithm is typically implemented in:
        - C++ for embedded or low-latency systems.
        - Python for prototyping, visualization, and offline validation.

Key Challenges and Limitations
    - Acoustic variability between roasters and microphone placements
    - Overlapping noise from cooling fans or exhaust systems
    - Second crack overlap in darker roasts
    - Need for per-machine calibration

As a result, most practical systems rely on tunable parameters rather than fully generic models.
