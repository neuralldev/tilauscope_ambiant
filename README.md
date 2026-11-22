# tilauscope_ambiant

TilauScope Ambiant probe support

## ESP32-S3 BLE probe: ambient sensor, first-crack detector, drop door

TilauScope Ambiant is an ESP32-S3 Bluetooth Low Energy (BLE) peripheral used next to the roaster by TilauScope (a fork of Artisan). It provides:

- ambient Temperature, Humidity, Pressure and Altitude from a BME280;
- acoustic detection of coffee **first crack** with an ICS-43434 I2S MEMS microphone;
- the state of the roaster's drop door (reed contact), so DROP is dated on the opening itself.

## Features

- **Wireless data**: NimBLE 2.x stack, up to two simultaneous clients (TilauScope plus the bench capture tool).
- **Unique naming**: the device name is built from the MAC address (`TLSCAM` + last 3 bytes of the MAC in hex).
- **Compact frames**: packed little-endian structures with fixed-point scaling and an additive checksum.
- **Status LED**: onboard WS2812 RGB LED shows advertising, connection, detection and calibration.
- **Persistent calibration**: the two calibration profiles are stored in LittleFS and restored at boot.
- **Simulation mode**: if the BME280 is missing, the ambient characteristic returns synthetic values.

## Hardware

| Component            | Function                       | Configuration |
|----------------------|--------------------------------|---------------|
| Microcontroller      | ESP32-S3-WROOM-1-**N8R8** dev board (board `esp32-s3-devkitc-1`) | 8 MB flash, 8 MB octal PSRAM (PSRAM not enabled in the build). Sold as "N16R8", but the flash is 8 MB |
| Status LED           | Onboard WS2812 RGB             | GPIO 48 |
| Environmental sensor | BME280 (I2C, 3.3 V)            | Address 0x77, or 0x76 as fallback |
| I2C SDA / SCL        | BME280 data / clock            | GPIO 8 / GPIO 9 |
| I2S microphone       | ICS-43434 (WS / SCK / SD)      | GPIO 5 / 6 / 4, L/R to GND |
| Drop door            | Reed switch (form A) to GND, 10 kΩ pull-up to 3V3 | GPIO 10 (LOW = door closed) |

### BME280 wiring (screw-terminal adapter)

The BME280 is powered at **3.3 V**, never 5 V: SDA/SCL must stay at 3.3 V logic.

| Wire color | Sensor pin | ESP32-S3 pin | Screw terminal (left block) |
|------------|-----------|--------------|------------------------------|
| Blue       | VIN       | 3V3          | 3.3V terminal                |
| Green      | GND       | GND          | GND terminal                 |
| Yellow     | SCL       | GPIO9        | IO9 terminal                 |
| Orange     | SDA       | GPIO8        | IO8 terminal                 |

> Do not trust the silkscreen printed on the adapter (CLK, P15, SD0…): it is generic and misleading. Match the GPIO numbers on the ESP32-S3 module; each screw connects to the module pin in line with it. GPIO 26–37 are reserved for flash/PSRAM and must not be used.

### ICS-43434 microphone wiring

| ICS-43434  | ESP32-S3 | Function          |
|------------|----------|-------------------|
| VDD        | 3V3      | Power             |
| GND        | GND      | Ground            |
| WS (LRCL)  | GPIO 5   | Word select       |
| SCK (BCLK) | GPIO 6   | Bit clock         |
| SD (DOUT)  | GPIO 4   | Data              |
| L/R        | GND      | Left channel only |

**Placement**: 30–40 cm from the roaster, at ear height, with no contact with the body of the machine. Mounted on the machine, the fan noise carried by the sheet metal masks the cracks.

Pictures: [ESP32-S3 board](include/boardesp32.jpg), [BME280](include/bme280.png), [ICS-43434](include/ICS-43434.jpg).

## Build and flash

PlatformIO project, Arduino-ESP32 2.0.x (espressif32 6.x, legacy I2S driver). Three environments:

| Environment       | Purpose | Serial speed |
|-------------------|---------|--------------|
| `esp32s3`         | Roast probe (BME280 + audio + door) | 115200 |
| `esp32s3_devtest` | Test bench: also streams the raw audio on the UART for `tools/bench_capture.py` | 921600 |
| `wroom32d`        | ESP32-WROOM-32D, audio only (no BME280, no door). I2S on WS=25, SCK=33, SD=32 | 115200 |

The S3 board has two USB-C ports. Serial output goes to UART0, that is the **"USB to serial" (CH343)** port; the native-USB port is used for flashing in `env:esp32s3` but shows no serial output.

Easiest: `pio run -e esp32s3 -t upload`. For a manual flash, the ESP32-S3 uses **DIO** flash mode and the **bootloader sits at 0x0** (not 0x1000 as on the classic ESP32):

| File                 | Address (ESP32-S3) |
|----------------------|--------------------|
| `bootloader.bin`     | **0x0**            |
| `partitions.bin`     | **0x8000**         |
| `firmware.bin`       | **0x10000**        |

Libraries (installed by PlatformIO): NimBLE-Arduino 2.x, Adafruit BME280, Adafruit Unified Sensor, ArduinoJson 7, ESP-DSP (bundled with the core).

## BLE interface

| Characteristic | UUID                                   | Access |
|----------------|----------------------------------------|--------|
| Service        | `f3b6e2a0-8c4e-4e1f-9c2d-1a7f5b9a1c01` | advertised |
| Ambient        | `f3b6a2a0-8c4e-4e1f-9c2d-1a7f5b9a1c05` | READ |
| Audio          | `f3b6e2a0-8c4e-4e1f-9c2d-1a7f5b9a1c04` | READ / WRITE |
| Drop door      | `f3b6e2a0-8c4e-4e1f-9c2d-1a7f5b9a1c06` | READ |

Note that the ambient UUID starts with `f3b6a2a0`, not `f3b6e2a0` like the others. That is the value the TilauScope client uses.

All frames are little-endian, start with header `0x5555` and end with footer `0xAAAA`. The checksum is the 8-bit sum of the payload bytes between header and checksum.

### Ambient frame (17 bytes, READ)

| header | temp_x10 (int16) | hum_x10 (int16) | press_x10 (int32, hPa) | alt_x10 (int32, m) | checksum | footer |
|--------|------------------|-----------------|------------------------|--------------------|----------|--------|

Divide the values by 10 to get °C, %RH, hPa and metres.

### Audio frame (9 bytes, READ)

| header | crack_count (int32) | checksum | footer |
|--------|---------------------|----------|--------|

`crack_count` is -1 while the probe is not calibrated (both profiles) or the microphone did not start, and during a calibration.

### Drop door frame (12 bytes, READ)

| header | open (u8) | open_count (u16) | since_open_ms (u32) | checksum | footer |
|--------|-----------|------------------|---------------------|----------|--------|

`open_count` counts openings since boot; `since_open_ms` is the time since the last opening began, `0xFFFFFFFF` if none.

### Audio commands (7 bytes, WRITE)

| header | command (int16) | checksum | footer |
|--------|-----------------|----------|--------|

| Command        | Function |
|----------------|----------|
| `0x0000`       | EMPTY calibration: empty drum, machine running, 30 s |
| `0x0006`       | MAILLARD calibration: beans loaded, after DE, 30 s. Needs a valid EMPTY profile |
| `0x0001`       | Start crack detection (needs both profiles; refused while a calibration is pending or running) |
| `0x0002`       | Stop crack detection |
| `0x0003`       | Print calibration state |
| `0x0004`       | Print sampling state and build identification |
| `0x0005`       | Print crack counter |
| `0x0007`       | Acquisition self-test (~2 s): prints `#TILAU_SELFTEST frames=… rms=… peak=…` |
| `0x0010` / `0x0011` | Verbose serial debug on / off |
| `0x0021`       | Print the confidence threshold |
| `0x0100 + pct` | Set the confidence threshold to pct % (0–100), saved with the calibration |

Answers are printed on the serial port only. Write with response if you need to know the command was received.

## Operation

LED: blue blinking = advertising, green = client connected, cyan = crack detection running, red fast blinking = calibration.

Typical roast, driven automatically by TilauScope:

1. Once per setup (or after moving the microphone): with the drum empty and the machine running, send **EMPTY calibration** (`0x0000`).
2. After DE, with the beans in, send **MAILLARD calibration** (`0x0006`). Pick a calm moment.
3. Send **START** (`0x0001`) after the MAILLARD calibration has finished.
4. At DROP, send **STOP** (`0x0002`).

Both profiles are saved and restored at boot, so the calibrations can be skipped while the setup stays the same (microphone position, roaster, exhaust). Small changes in microphone position change the acoustics a lot: write the placement down. Calibrate again after any change.

First crack itself is reported on the serial port (`#TILAU_FC candidate …`, `#TILAU_FC confirmed …`); over BLE only the crack counter is available.

## Simulation mode

If the BME280 is not found at boot, the ambient characteristic returns smooth sinusoidal values instead of measurements, so the BLE link and the client parsing can be tested without the sensor. The boot log then shows `BME280 not detected no secondary, enter simulation mode`.

## First-crack detection algorithm

First crack shows up as short impulsive "pops" over the continuous noise of the roaster (drum, fan, burner). With the microphone at 30–40 cm, cracks stand out best between 4 and 8 kHz, while 1.5–4 kHz is dominated by the fan.

Pipeline, every 8 ms:

1. I2S at 16 kHz, 24-bit samples normalised to [-1, 1].
2. Continuous band-pass biquad centred at 5.5 kHz (Q = 1.2).
3. 256-sample Hann-windowed FFT with 50 % overlap.
4. Features: spectral flux over 4–7.9 kHz, crest factor (peak/RMS) of the band-passed signal, spectral centroid of the flux band.
5. Threshold: the larger of an adaptive EMA of the flux and a floor of 1.35 × the flux P99 measured during the MAILLARD calibration. The EMA is frozen while the flux is above threshold, so a burst of cracks does not raise it.
6. Composite score: 0.45 × flux + 0.30 × crest + 0.25 × centroid sub-scores, compared to the confidence threshold (default 60 %, usable range 55–65 %).
7. A crack is counted when the flux is above threshold, rising, the score is above the confidence threshold, and 150 ms have passed since the previous crack.
8. A state machine turns crack density into a first-crack event: MONITORING → CANDIDATE (2 cracks in 5 s or 5 in 10 s) → CONFIRMED (6 cracks in 20 s, with 2 in the last 5 s). A candidate falls back after 8 s of silence or 25 s without confirmation.

Each calibration records the same features over 30 s. The MAILLARD calibration removes transient peaks (louder than both the empty-drum profile and the loaded-drum background) before computing its statistics.

Settings are tuned by replaying recorded roasts offline (`tools/bench_capture.py` to record on the test bench, `tools/replay_detector.py` to replay) before flashing.
