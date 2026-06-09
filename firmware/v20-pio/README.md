# Martijns arm — v20 firmware (PlatformIO)

Modulaire C++ herstructurering van v19's monolithische `.ino`. Doel: **testbaar**, **debugbaar**, **onderhoudbaar**.

## Quick start

```bash
# Build
pio run -e esp32s3

# Flash naar arm (ESP32-S3, SparkFun Thing Plus, USB-C)
pio run -e esp32s3 -t upload

# Serial monitor
pio device monitor

# Native unit tests (geen hardware nodig)
pio test -e native
```

## Architectuur

Zie [`docs/architecture.md`](docs/architecture.md) voor module-overview, dataflow en design-rationale.

Serial protocol: [`docs/protocol.md`](docs/protocol.md).

Build/flash/test details: [`docs/build.md`](docs/build.md).

## Status

Werk-in-uitvoering — modulaire herstructurering vanuit v19. Werkende v19 firmware staat in `../v19/robot_arm_v19/robot_arm_v19.ino` als referentie.
