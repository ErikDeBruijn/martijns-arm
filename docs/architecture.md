# ROS2-architectuur voor Martijns 3-DOF arm

> **Uitgangspunt:** v18.1 firmware werkt (modulo PID-tuning na de 200-step VACTUAL fix). We bouwen het niet vanaf nul opnieuw, we **migreren incrementeel** zodat elke stap los te valideren is. Behoud van de teach-and-repeat workflow en log-gedreven werkwijze is leidend.

## Waarom überhaupt ROS2?

De huidige .ino doet alles:
- 200Hz PID-loop per joint
- Trajectory recording → SD-kaart CSV
- Playback met time-scaling
- Homing op arm-encoder
- 3 knoppen (REC/PLAY/DEL) + LED status

Werkt, maar:
- **Diagnose is moeilijk** — alle logs via Serial, plotten in spreadsheet
- **Kan niet samenwerken met de rest** — geen IK, geen RViz, geen sim
- **Iteratief tunen kost veel flashes**

Met ROS2 krijg je:
- **Live plot via PlotJuggler/Foxglove** — sleep een topic, zie de PID-respons in real-time
- **`ros2 bag record`** vervangt SD-kaart CSV; replay vervangt playback
- **MoveIt2** voor IK/planning later (zodat Claude → end-effector commando's kan)
- **URDF/RViz** om de arm te visualiseren én een digital twin te bouwen
- **Live parameter-tuning** zonder opnieuw te flashen

## Splitsing tussen ESP32 en host

| Component | Waar | Reden |
|---|---|---|
| TMC2209 UART driver | ESP32 | Hardware-dichtbij |
| AS5600 + TCA9548A I2C lezen | ESP32 | Hardware-dichtbij, 200Hz strakke timing |
| **PID per joint** | **ESP32** | Lage latency (host-trip via USB = 1-2ms jitter); werkt door als USB even los is |
| Time-scaling op trajectory | host (ROS node) | Eenvoudiger te tunen, geen flash-cycle |
| Trajectory recording | host (`ros2 bag`) | Replace SD-kaart CSV |
| Playback | host (ROS node) | Streamt `/joint_command` |
| Homing state machine | ESP32 | Veiligheid: niet afhankelijk van host |
| Buttons (REC/PLAY/DEL) | ESP32 → publishes events | Tactiel feedback, host hoeft niet te draaien |
| URDF, RViz, MoveIt | host | GUI-spul |
| Hogere niveau (Claude) | host | Geen MCU-gedoe |

Kort: **ESP32 doet de hard-real-time loop, host doet de slimme dingen.**

## Communicatie: micro-ROS over USB serial

```
┌─────────────────────────────────────┐         ┌──────────────────────────────┐
│  Mac (Docker container ros2-jazzy)  │         │  ESP32-S3 (micro-ROS client) │
│                                     │  USB    │                              │
│  micro_ros_agent ←──serial──────────┼─────────┼──→ uxr session              │
│       ↕  DDS                        │  ACM0   │       ↕                      │
│  /joint_states (200Hz)              │         │  Encoders → publish          │
│  /joint_command (200Hz)             │         │  Subscribe → PID setpoint    │
│  /arm_buttons (event)               │         │  Buttons → publish           │
│  /arm_diagnostics (10Hz)            │         │  PID errors, currents        │
└─────────────────────────────────────┘         └──────────────────────────────┘
```

**Waarom USB serial, niet WiFi UDP:**
- Deterministische latency (~1ms vs 5-50ms over WiFi)
- Werkt zonder netwerk
- Geen DDS multicast over WireGuard headaches
- Voor remote/mobile werk later kan WiFi UDP er als 2e transport bij komen

**Waarom micro-ROS, niet rosserial of custom protocol:**
- micro-ROS is officiële ROS2 spec voor MCU's
- Native ROS2 messages (`sensor_msgs/JointState`, `trajectory_msgs/JointTrajectory`)
- Werkt met `colcon build` workflow én PlatformIO via `micro_ros_platformio`
- Toekomst-bestendig (ROS2 community standaard)

## Topics

| Topic | Type | Direction | Rate | Doel |
|---|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | ESP32 → host | 200Hz | actuele positie, snelheid, effort per joint (arm + motor encoders) |
| `/joint_command` | `trajectory_msgs/JointTrajectory` | host → ESP32 | 200Hz | gewenste positie/snelheid setpoints |
| `/arm_buttons` | `std_msgs/UInt8` | ESP32 → host | event | bitmask van 3 knoppen (rec/play/del) |
| `/arm_diagnostics` | `diagnostic_msgs/DiagnosticArray` | ESP32 → host | 10Hz | TMC currents, errors, dead-band states, time-scale, raw vs filtered |
| `/arm_mode` | `std_msgs/String` | host → ESP32 | event | `IDLE`/`HOMING`/`RECORDING`/`PLAYBACK` |

`/joint_states` publiceert **6 joints** (3 arm-side + 3 motor-side) — dat houdt backlash-detectie open en geeft visualisatie van de mechanische speling.

## Migratie van v18.1 functionaliteit

| v18.1 functie | Wordt in ROS2 |
|---|---|
| `setMotorSpeed()` met `MOTOR_VACTUAL_CORR` | Blijft in firmware. ROS subscribe → PID → VACTUAL output |
| PID loop in `loop()` | Blijft op ESP32 (200Hz, deterministic) |
| Time-scaling op `refVel` | **Naar host** — een ROS node berekent scale uit `/arm_diagnostics` errors en past `/joint_command` aan voor verzending |
| `recordToSD()` + CSV op SD-kaart | **Vervangen** door `ros2 bag record /joint_states /arm_buttons` |
| `playbackFromSD()` | **Vervangen** door host node die uit een bag of trajectory file streamt |
| Homing state machine | Blijft op ESP32, getriggerd via `/arm_mode` |
| 3 knoppen | ESP32 publisht events; host node mapt naar acties (start record / start playback / delete bag) |
| LED status (FastLED) | Blijft op ESP32, gestuurd door interne state |
| NVS home opslag | Blijft (firmware is autoritair voor home) |
| `DENSE_CSV_LOG` | Vervangen door `/arm_diagnostics` op 200Hz topic; opnemen via `ros2 bag` |

## De huidige jitter — diagnose-pad in ROS

Martijns doc identificeert M2 als onder-getuned ná de v18.1 VACTUAL fix. ROS gaat dit makkelijker maken:

1. **Live plot** in PlotJuggler: `/joint_states` (3 motors × {ref,actual,err}) + time_scale uit diagnostics
2. **Bag record** een trage en een snelle trajectory, repliceer Martijns metingen
3. **Side-by-side plot** v18.1 met huidige PID vs aangepaste PID
4. **Per-as parameter-aanpassing live** via `ros2 param set` (geen flash nodig als we `dynamic_reconfigure`-style gebruiken — ESP32 subscribet op `/arm_pid_params`)

Dit is exact "eerst diagnose, dan code" maar met betere tools dan Serial+spreadsheet.

## Bouwvolgorde (incremental, elk apart te valideren)

| # | Wat | Validatie |
|---|---|---|
| 1 | Hello-world ROS2 container ✅ | `ros2 topic echo /chatter` werkt |
| 2 | URDF van de arm in `ros2_ws/src/martijns_arm_description/` | RViz toont de arm met joint sliders |
| 3 | micro-ROS firmware skeleton: alleen encoders → `/joint_states` | RViz volgt fysieke beweging als je de arm met de hand beweegt |
| 4 | micro-ROS firmware: `/joint_command` → setMotorSpeed (open loop) | Joint commando vanaf host beweegt motor |
| 5 | Port PID-loop naar firmware, integreer met topics | M2 jitter zichtbaar in PlotJuggler — exact te tunen |
| 6 | Recording: `ros2 bag record` van een teach-sessie | Bag bevat joint_states + button events |
| 7 | Playback node: bag → `/joint_command` met time-scaling | Originele teach-and-repeat workflow herleeft |
| 8 | Homing als micro-ROS state, getriggerd via `/arm_mode` | Knop op breadboard triggert homing |
| 9 | Buttons → events op `/arm_buttons` | LED reflecteert via firmware state |
| 10 | URDF + Foxglove dashboard | Tijdens teach zie je de arm bewegen in 3D |

Stap 1-2 zijn host-only en kunnen los. Stap 3-4 vereisen ESP32 met micro-ROS firmware. Stap 5+ is de echte migratie.

## Wat blijft expliciet **niet** veranderen

- Mechanische setup (foto's in `docs/photos/`)
- TMC2209 UART pinning (GPIO 17/16/15)
- AS5600 mux kanalen ({0,3,7} motor, {1,2,6} arm)
- 256 microsteps, 200-step motoren
- Homing logica (NVS opslag, arm-encoder als ground truth)
- Knoppen-positie/-functie

## Volgende concrete stap

**Stap 2** uit de bouwvolgorde: een minimale URDF schrijven van de arm (3 joints, parallelle linkage versimpeld als enkele revolute joints met een mimic joint voor het parallellogram). Dat geeft direct visualisatie in RViz en is voorwaarde voor MoveIt later.

Alternatief: eerst **stap 5/PID-tuning op de huidige firmware** afronden zodat we een werkende baseline hebben — dan migreren we niet bovenop een onopgelost bug.
