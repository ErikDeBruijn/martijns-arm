# End-to-end pipeline: Simulatie → Training → Real Robot

> Dit project mikt op één **gemeenschappelijke ROS2 stack** waarmee je een
> robotarm kunt:
>
> 1. Visualiseren (URDF in Foxglove)
> 2. Simuleren (Gazebo physics + Isaac Sim hoge-fidelity)
> 3. Trainen (reinforcement learning in Isaac Lab op pve03)
> 4. Deployen op echte hardware (zonder code-wijziging)
>
> Twee fysieke arms zijn target: **Martijns custom 3-DOF arm** (ESP32 +
> TMC2209 + AS5600) en **Eriks 7Bot** (Arduino Due + digitale servos,
> bij hem thuis).

## Architectuur — één topic interface, drie hardware backends

```
                                 ┌──────────────────────┐
                                 │  ROS2 controllers    │
                                 │  (joint_trajectory_  │
   ┌───────────────────────┐     │   controller, etc.)  │     ┌──────────────────┐
   │  RL trainer           │────▶│                      │────▶│   /joint_states  │
   │  (Isaac Lab op pve03) │     │  /arm_controller/    │     │                  │
   └───────────────────────┘     │   joint_trajectory   │◀────│ (sensor feedback)│
                                 └──────────┬───────────┘     └──────────────────┘
                                            │
            ┌───────────────────────────────┼─────────────────────────────────────┐
            ▼                               ▼                                     ▼
    ┌──────────────┐              ┌──────────────────┐                  ┌─────────────────┐
    │ Gazebo Sim   │              │ Isaac Sim        │                  │ Echte hardware  │
    │ gz_ros2_     │              │ omni.isaac.      │                  │                 │
    │ control      │              │  ros2_bridge     │                  │ Martijn: ESP32  │
    │ plugin       │              │ (op pve03 LXC)   │                  │   micro-ROS     │
    │              │              │                  │                  │ Erik: 7Bot via  │
    │              │              │                  │                  │   sevenbot_     │
    │              │              │                  │                  │   bridge node   │
    └──────────────┘              └──────────────────┘                  └─────────────────┘
```

**Het kernpunt:** een policy die getraind is in simulatie publiceert
`/arm_controller/joint_trajectory` exact zoals een handgeschreven script,
en de echte arm reageert via dezelfde subscribe-en-execute logica.

## Status van elk onderdeel

| Component | Status | Pakket / locatie |
|---|---|---|
| URDF Martijn arm | ✅ | `ros2_ws/src/martijns_arm_description/` |
| URDF 7Bot | ✅ | `ros2_ws/src/sevenbot_description/` |
| Gazebo simulatie | ✅ | `ros2_ws/src/martijns_arm_gazebo/` |
| 7Bot bridge (USB → ROS) | ✅ | `ros2_ws/src/sevenbot_bridge/` |
| Martijn arm bridge (Path 1, ASCII) | ✅ | `ros2_ws/src/arm_serial_bridge/` |
| Martijn v19 firmware (host commando's) | ✅ compileert, klaar voor flash | `firmware/v19/robot_arm_v19/` |
| Isaac Sim LXC op pve03 | ✅ container draait | CT 140 op pve03 |
| Isaac Sim install | ⚠ TODO Erik (NGC + Tailscale) | `docs/isaac-sim-install.md` |
| Isaac Lab integration | ⚠ TODO | — |
| Foxglove visualisatie | ✅ | `ws://localhost:8765` |

## De drie modus operandi

### A. Lokaal alleen simulatie (geen hardware)

```bash
# Container + bridge
cd ~/Dev/robotics/martijns-arm/docker && docker compose up -d
docker exec -d ros2-jazzy bash -ic '
  ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
'

# Gazebo arm spawnen
docker exec -d ros2-jazzy bash -ic '
  ros2 launch martijns_arm_gazebo sim.launch.py
'

# Open Foxglove → ws://localhost:8765 → 3D panel met /robot_description
```

Bewegen via:
```bash
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory '{
    joint_names: [joint_1, joint_2, joint_3],
    points: [{positions: [0.5, 0.3, -0.4], time_from_start: {sec: 2}}]
  }'
```

### B. 7Bot thuis (Erik)

```bash
# Plug 7Bot USB in. Controleer /dev/cu.usbserial-*
ls /dev/cu.usbserial-*

# Container + bridge starten
docker compose up -d

# Bridge node tegen de fysieke 7Bot
docker exec ros2-jazzy bash -ic '
  ros2 run sevenbot_bridge sevenbot_bridge --port /dev/cu.usbserial-XXXX
'

# Kalibratie: zet alle joints op 90 graden = 1.5708 rad
ros2 topic pub --once /sevenbot/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory '{
    joint_names: [joint_0, joint_1, joint_2, joint_3, joint_4, joint_5],
    points: [{positions: [1.5708, 1.5708, 1.5708, 1.5708, 1.5708, 1.5708], time_from_start: {sec: 1}}]
  }'

# Vacuüm aanzetten via service
ros2 service call /sevenbot/set_vacuum std_srvs/srv/SetBool '{data: true}'
```

### C. Martijns arm (full chain)

```bash
# Mac: USB → TCP brug
~/Dev/robotics/martijns-arm/scripts/serial-tcp-bridge.sh

# Container: parser
~/Dev/robotics/martijns-arm/scripts/run-arm-bridge.sh

# In Foxglove zie je nu /joint_states en alle /arm/m{1,2,3}/* topics
# Voor PID tuning: nc localhost 9999, dan >TUNE KP 1 5.0 etc.
```

## Training pipeline (Isaac Sim + Isaac Lab)

> **Voorwaarden** voordat dit werkt:
> 1. Tailscale geactiveerd in CT 140 (Erik: paste auth key in `/root/TODO-erik.txt`)
> 2. NGC account aangemaakt (gratis), API key
> 3. `pip install isaacsim --extra-index-url https://pypi.nvidia.com` in CT
> 4. Isaac Lab geïnstalleerd (apart, see https://isaac-sim.github.io/IsaacLab/)

Daarna, generieke RL trainings-cyclus:

1. **Asset preparen**: URDF → USD via `omniverse_kit_app urdf_importer`
2. **Environment definiëren** in Isaac Lab (bijv. PickAndPlace task)
3. **Train** via PPO/SAC (Isaac Lab bundelt deze) — ~~uren tot dagen op de RTX 6000~~
4. **Export policy** als ONNX of PyTorch checkpoint
5. **Inference node** in ROS2 die de policy laadt en `/joint_trajectory` publiceert
6. **Test in Gazebo** eerst (sim-to-sim), dan op echte hardware

## Sim-to-real fidelity overwegingen

Hoe kleiner de delta tussen sim en echt, hoe beter een sim-getrainde policy
op de echte arm werkt. Onze fidelity strategie:

| Domein | Aanpak |
|---|---|
| **Geometrie** | URDF link lengtes uit firmware constanten (7Bot) en hardware blueprints (Martijn). STL meshes als beschikbaar. |
| **Massa/inertia** | Educated guesses; refine als arm-load relevant wordt. |
| **Joint limits** | 1-op-1 uit firmware (`thetaMin/Max` voor 7Bot, KP/KD bands voor Martijn). |
| **Servo dynamiek** | Default ros2_control PID; tunen met systeem-identificatie van echte data later. |
| **Latency** | USB serial introduceert ~5-15ms; modelleren als delay buffer in sim. |
| **Domain randomization** | Variëren van massa, friction, sensor noise tijdens training — robuuster transfer. |

## Volgende stappen (in volgorde)

1. ⏳ **Erik configureert Tailscale + NGC** — unblocks Isaac Sim install
2. ⏳ **Mesh agent** afmaken voor mooiere visualisatie (loopt al)
3. **Eerste task definiëren**: bv. "raak een doel aan met de end effector" — simpele PPO baseline
4. **Train in Gazebo eerst** als smoke test (snel, geen Isaac Sim nodig)
5. **Migrate naar Isaac Sim** voor fotorealistische rendering + parallel envs (faster training)
6. **Sim-to-real** op 7Bot eerst (veiliger dan Martijns experimentele arm)
7. **Iteratief verfijnen**

## Bestanden om te lezen

- `docs/architecture.md` — design beslissingen
- `docs/reproduction-guide.md` — Martijn kan dit project van nul opzetten
- `docs/build-config.md` — Arduino IDE / arduino-cli config
- `docs/isaac-sim-vm-plan.md` — keuze proces voor LXC vs VM
- `docs/isaac-sim-install.md` — wat Erik moet doen voor Isaac Sim
- `docs/isaac-sim-lxc-status.md` — huidige staat van CT 140
- `docs/sevenbot-mesh-hunt.md` — mesh hunt resultaten (in progress)
