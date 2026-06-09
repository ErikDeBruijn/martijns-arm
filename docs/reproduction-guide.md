# Reproduceer dit project van nul

> **Doel:** Martijn (of iedereen anders) kan met deze handleiding van een lege Mac
> naar werkende ROS2 + Foxglove + arm-firmware komen, in dezelfde staat als dit
> repo nu is.
>
> **Werktijd:** ~30 min installatie + Docker image build (eenmalig ~10 min download).
>
> **Doelplatform:** macOS op Apple Silicon (M-serie). Andere Macs werken ook,
> dezelfde stappen. Linux gebruikers slaan macOS-specifieke dingen over.

---

## 0. Wat we hier hebben

Een Docker-based ROS2 Jazzy ontwikkelomgeving voor Martijn's 3-DOF arm:

```
martijns-arm/
├── docker/                     # Dockerfile + compose
├── docs/                       # alle documentatie (deze map)
├── firmware-original/          # v18.1 zoals hij draaide — read-only referentie
├── firmware/v19/               # nieuwe firmware met serial command parser
├── ros2_ws/src/                # ROS2 packages
│   ├── martijns_arm_description/   # URDF + launch om de arm te visualiseren
│   └── arm_serial_bridge/          # Python brug: USB → ROS topics
└── scripts/                    # convenience scripts (start, bridge, etc.)
```

## 1. Prerequisites op je Mac

Installeer eenmalig:

```bash
# Docker Desktop (download van docker.com, ZSH GUI installer)
# – of via brew:
brew install --cask docker

# Homebrew tools die we gebruiken
brew install arduino-cli socat
brew install --cask foxglove-studio

# Verify
docker --version              # >= 24
arduino-cli version           # >= 1.4
socat -V | head -1            # any
ls /Applications/Foxglove.app # bestaat
```

**Open Docker Desktop één keer** zodat de daemon draait. Geen andere config nodig.

## 2. Project ophalen

Voor nu: `~/Dev/robotics/martijns-arm/` is de werkdirectory. Dit hele blok structureel:

```bash
mkdir -p ~/Dev/robotics
cd ~/Dev/robotics
# In de toekomst: git clone <repo>
# Voor nu: kopieer de hele martijns-arm/ directory hier
```

## 3. Docker container bouwen

```bash
cd ~/Dev/robotics/martijns-arm/docker
docker compose build
```

Eerste build duurt ~5 minuten (ROS2 Jazzy base image is ~2GB). Vervolgens:

```bash
docker compose up -d        # start in achtergrond
docker compose ps           # verify Up
```

Container heet `ros2-jazzy`. Stop met `docker compose down`.

## 4. ROS2 workspace bouwen (in de container)

```bash
docker exec ros2-jazzy bash -ic '
  cd /home/ros/ros2_ws
  colcon build --symlink-install
'
```

`--symlink-install` betekent: edits aan Python files in `src/` werken zonder rebuild.
Wel rebuilden bij wijzigingen aan URDF, package.xml, of CMakeLists.txt.

## 5. Foxglove Studio openen + connecten

```bash
# Eenmalig: bridge starten in container
docker exec -d ros2-jazzy bash -ic '
  ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
'

# Open de Mac app
open -a Foxglove
```

In Foxglove:
- Skip de framework wizard ("Go to Dashboard")
- **Open connection** → **Foxglove WebSocket** → `ws://localhost:8765` → **Open**
- Je ziet `/rosout` en `/parameter_events` minimum.

## 6. Arm visualiseren (URDF)

```bash
# In container:
docker exec -d ros2-jazzy bash -ic '
  source /opt/ros/jazzy/setup.bash
  source /home/ros/ros2_ws/install/setup.bash
  ros2 launch martijns_arm_description view_arm.launch.py use_gui:=false
'
```

In Foxglove:
1. Klik **+** rechtsboven (naast "Default") → **3D**
2. Settings (⚙) van het 3D panel → **Frame** → kies `world`
3. Onder **Topics** → enable `/robot_description` en `/tf`
4. Je ziet nu de arm — in nul-positie (alle joints op 0°)

Om de joints te bewegen voor visuele test:
```bash
# Publish wisselende joint angles
docker exec ros2-jazzy bash -ic '
  ros2 topic pub --rate 1 /joint_states sensor_msgs/JointState "{
    name: [joint_1, joint_2, joint_3],
    position: [0.5, 0.3, -0.4]
  }"
'
```

## 7. Firmware op de ESP32 zetten

### Optie A: Arduino IDE (jouw huidige flow, Martijn)

In de IDE:
- **Tools → Board → esp32 → SparkFun ESP32-S3 Thing Plus**
- **Tools → Port → /dev/cu.usbmodem1101** (kan ander nummer zijn — pak de `usbmodem*`, niet `usbserial-*`)
- Settings exact zoals in `docs/build-config.md` — vooral:
  - Upload Mode: **USB-OTG CDC (TinyUSB)**
  - USB CDC On Boot: **Enabled**
- Sketch openen: `firmware/v19/robot_arm_v19/robot_arm_v19.ino`
- Verify (✓), dan Upload (→)

### Optie B: arduino-cli vanaf terminal

```bash
cd ~/Dev/robotics/martijns-arm
FQBN="esp32:esp32:sparkfun_esp32s3_thing_plus:UploadMode=cdc,CDCOnBoot=cdc"
arduino-cli compile -b "$FQBN" firmware/v19/robot_arm_v19/
arduino-cli upload  -b "$FQBN" -p /dev/cu.usbmodem1101 firmware/v19/robot_arm_v19/
```

## 8. Live data van de echte arm in ROS

Drie terminals nodig.

**Terminal 1** — Mac side, USB → TCP brug:
```bash
~/Dev/robotics/martijns-arm/scripts/serial-tcp-bridge.sh
# Houdt de USB serial bezet en deelt het via TCP :9999.
# Ctrl+C om te stoppen.
```

**Terminal 2** — container side, parser:
```bash
~/Dev/robotics/martijns-arm/scripts/run-arm-bridge.sh
# Connect met host.docker.internal:9999, parse PB-format,
# publiceer /joint_states + /arm/m{1,2,3}/{ref,enc,err,spd,cmd}
```

**Terminal 3** — Foxglove app (al open). Topics als `/arm/m2/err_deg`
verschijnen automatisch — sleep ze in een **Plot** panel.

## 9. v19 commando's testen

Met de bridge actief (terminal 2 hierboven), open een **vierde** terminal:

```bash
# Connect direct met de TCP bridge (alternatief voor ros2 topic pub)
nc localhost 9999
> >HELP
< <OK help
<   >HELP                       dit overzicht
<   >STATUS                     mode/home/motion/PID
<   ...
> >STATUS
< <OK status mode=IDLE home=1 motion=1 kp=[6.50,3.00,6.80] ...
> >TUNE KP 1 5.0
< <OK tune KP[1]=5.0000
```

PID-tuning gaat dus zonder reflashen — verander `KP_SPEED[1]` live, kijk in
Foxglove naar de respons.

## 10. Stop alles

```bash
cd ~/Dev/robotics/martijns-arm/docker
docker compose down       # stop ros2-jazzy container
# Mac side scripts: Ctrl+C in hun terminals
```

---

## Bekende issues

### Foxglove ziet geen topics
- Bridge draait? Check: `docker exec ros2-jazzy pgrep -af foxglove_bridge`
- Port mapping actief? Check: `docker ps` moet `0.0.0.0:8765->8765/tcp` tonen
- Connection in Foxglove echt geopend (groene status)?

### `/dev/cu.usbmodem1101` bestaat niet
- ESP32 niet aangesloten, of Arduino IDE/Serial Monitor heeft 'm vast
- Check: `ls /dev/cu.usbmodem*`
- Sluit Arduino Monitor, of de andere app, of plug ESP32 opnieuw in

### Arduino IDE upload faalt met "Failed to connect"
- Probeer Reset+BOOT trick:
  1. Hou BOOT ingedrukt
  2. Druk EN/RST kort
  3. Laat BOOT los → board zit in download mode
  4. Upload
- Als dit constant nodig is: `Upload Mode = USB-OTG CDC (TinyUSB)` ontbreekt

### `network_mode: host` werkt niet op macOS
- Klopt — Docker Desktop op Mac doet host networking in de Linux VM, niet op je
  Mac. Wij gebruiken expliciete `ports:` mapping voor :8765 en :9090. Zie
  `docker/docker-compose.yml`.

### URDF wijzigingen worden niet zichtbaar
- Na elke xacro edit: `docker exec ros2-jazzy bash -ic 'cd /home/ros/ros2_ws && colcon build --packages-select martijns_arm_description --symlink-install'`
- Dan launch opnieuw

### `ros2 topic list` toont oude topics
- Daemon caching. Reset: `docker exec ros2-jazzy bash -ic 'ros2 daemon stop && ros2 daemon start'`

---

## Verdere documentatie

- `docs/architecture.md` — design beslissingen voor de ROS2 architectuur
- `docs/build-config.md` — Arduino IDE en arduino-cli configuratie details
- `docs/sessie_robotarm_v18_m2.md` — Martijn's vorige diagnose voor M2 jitter
- `firmware/v19/robot_arm_v19/robot_arm_v19.ino` — header heeft volledige changelog
