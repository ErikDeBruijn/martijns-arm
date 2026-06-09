# Martijn's Arm — ROS2 dev environment

Lokale Docker-based ROS2 Jazzy setup om de robot arm aan te sturen.

## Layout

```
martijns-arm/
├── docker/                 # Dockerfile + docker-compose.yml
├── ros2_ws/                # ROS2 workspace, gemount in container
│   └── src/                # eigen packages hier
├── scripts/                # convenience scripts
└── docs/                   # firmware notes, hardware docs
```

## Eerste keer opstarten

```bash
# 1) XQuartz voor RViz/Gazebo GUI
./scripts/start-xquartz.sh

# 2) Container builden + binnenstappen
./scripts/up.sh

# In container:
ros2 run demo_nodes_cpp talker        # in één terminal
docker compose exec ros2 bash         # tweede terminal
ros2 run demo_nodes_cpp listener
```

## Hardware

- Arm: TBD (Martijn's eigen build / commercieel?)
- Interface: TBD (USB serial / CAN-USB / Ethernet)
- Firmware repo: TBD

## Gotchas op macOS

- USB passthrough naar Docker werkt niet direct. Voor echte arm-aansluiting:
  - Optie A: arm via TCP/UDP (firmware moet dat ondersteunen)
  - Optie B: ROS2 op Linux machine (Proxmox VM zodra firewall gefixed)
  - Optie C: native ROS2 op Mac via RoboStack (mamba)
- X11 GUI vereist XQuartz draaiend met "Allow connections from network clients" aan.
