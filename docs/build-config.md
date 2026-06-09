# Build- en upload-configuratie

> SparkFun ESP32-S3 Thing Plus, voor Martijns arm v18+ firmware.

## Arduino IDE — board en Tools menu

**Board selectie:**
```
Tools → Board → esp32 → "SparkFun ESP32-S3 Thing Plus"
Tools → Port  → /dev/cu.usbmodem1101    (native USB-CDC, niet usbserial-0001)
```

**Tools menu instellingen** (zoals door Martijn gezet):

| Optie | Waarde |
|---|---|
| **USB CDC On Boot** | **Enabled** ← afwijkend van default |
| CPU Frequency | 240MHz (WiFi) |
| Core Debug Level | None |
| USB DFU On Boot | Disabled |
| Erase All Flash Before Sketch Upload | Disabled |
| Events Run On | Core 1 |
| Flash Mode | QIO 80MHz |
| JTAG Adapter | Disabled |
| Arduino Runs On | Core 1 |
| USB Firmware MSC On Boot | Disabled |
| Partition Scheme | Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS) |
| PSRAM | QSPI PSRAM |
| **Upload Mode** | **USB-OTG CDC (TinyUSB)** ← afwijkend van default |
| Upload Speed | 921600 |
| USB Mode | Hardware CDC and JTAG |
| Zigbee Mode | Disabled |

**Waarom de twee afwijkende settings:**
- `USB CDC On Boot = Enabled` + `Upload Mode = USB-OTG CDC (TinyUSB)` → het board verschijnt op macOS direct als `/dev/cu.usbmodem1101` en kan **zonder BOOT-knop trick** geflasht worden. Sketch en Serial Monitor delen dezelfde poort.

## arduino-cli (1-op-1 equivalent)

```bash
# Board details checken
arduino-cli board details -b esp32:esp32:sparkfun_esp32s3_thing_plus

# Volledige FQBN met dezelfde opties als de IDE
FQBN="esp32:esp32:sparkfun_esp32s3_thing_plus:UploadMode=cdc,CDCOnBoot=cdc"

# Compile
arduino-cli compile -b "$FQBN" firmware-original/

# Upload (board moet beschikbaar zijn op /dev/cu.usbmodem1101)
arduino-cli upload -b "$FQBN" -p /dev/cu.usbmodem1101 firmware-original/

# Combineer
arduino-cli compile -b "$FQBN" -u -p /dev/cu.usbmodem1101 firmware-original/
```

Alle andere opties zijn de defaults van het board en hoeven niet expliciet meegegeven te worden. Wil je ze toch zien:

```bash
# Volledige expansie van alle defaults
FQBN_FULL="esp32:esp32:sparkfun_esp32s3_thing_plus:UploadSpeed=921600,USBMode=default,CDCOnBoot=cdc,MSCOnBoot=default,DFUOnBoot=default,UploadMode=cdc,CPUFreq=240,FlashMode=qio,PartitionScheme=default,DebugLevel=none,PSRAM=enabled,LoopCore=1,EventsCore=1,EraseFlash=none,JTAGAdapter=default,ZigbeeMode=default"
```

## Libraries

```bash
arduino-cli lib install "TMC2209" "FastLED"
arduino-cli lib list                                   # verifieer
```

`Wire`, `Preferences`, `FS`, `SD_MMC` zijn stock onderdeel van de ESP32 core.

## Serial Monitor

```bash
# Native (niet door arduino-cli — die heeft geen monitor command op alle versies)
screen /dev/cu.usbmodem1101 115200             # ctrl+A k om te exiten

# Of via Python pyserial
python3 -m serial.tools.miniterm /dev/cu.usbmodem1101 115200
```

## Belangrijk: data dir wordt gedeeld met IDE

`arduino-cli` gebruikt standaard `~/Library/Arduino15/` als data directory — dezelfde als de IDE. Cores en libs die je in de IDE installeert zijn direct beschikbaar in CLI en omgekeerd. Geen dubbele downloads.

## Visualisatie met Foxglove Studio

Foxglove Studio is de visualisatie-tool voor ROS2 topics — plots, 3D-views, raw messages, alles in één app. De koppeling met ROS2 loopt via `foxglove_bridge`, een WebSocket-server die in de container draait.

**Eenmalig: app installeren** (al gedaan via Homebrew):
```bash
brew install --cask foxglove-studio
```

**Workflow elke keer:**

1. Zorg dat de ros2-jazzy container draait (`./scripts/up.sh` als dat nog niet zo is).
2. Start de bridge in een aparte terminal:
   ```bash
   ./scripts/foxglove-bridge.sh
   ```
   Hij blijft op de voorgrond — laat het venster open en stop met Ctrl+C als je klaar bent.
3. Open de Foxglove app (`open -a Foxglove` of via Launchpad).
4. In Foxglove: **Open connection...** → kies **Foxglove WebSocket** → vul in:
   ```
   ws://localhost:8765
   ```
5. Klik **Open**. De bridge ontdekt automatisch alle topics die op `ROS_DOMAIN_ID=42` actief zijn — geen handmatige topic-lijst nodig.

**Handige panels voor PID-tuning:** voeg een **Plot** panel toe en sleep numerieke velden (bv. setpoint, measured position, error) erin om time-series naast elkaar te zien — ideaal om P/I/D termen visueel af te stellen.
