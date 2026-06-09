# Firmware architectuur

## Doelstelling

Modulaire herstructurering van v19's 1908-regel monoliet. Iedere module:

- Heeft een **korte, expliciete API** (header)
- Is **unit-testbaar** waar mogelijk (pure logica → native Unity tests)
- Heeft **één duidelijke verantwoordelijkheid** (single responsibility)
- Bevat **geen documentatie-blokken in code** — alles in markdown (deze map)

`main.cpp` blijft klein: alleen `setup()` + `loop()` + globale wiring tussen modules.

## Modules

| Module           | Verantwoordelijkheid                                     | Testbaar? |
|------------------|----------------------------------------------------------|-----------|
| `config`         | Pin-assignments, channel-mappings, sample rates, tolerances | N/A (data) |
| `led`            | LED-state machine, knipperpatronen voor mode-feedback     | gedeeltelijk (state only) |
| `nvs_storage`    | Wrapper rond `Preferences` — home-pose, soft endstops    | mock-baar |
| `encoders`       | AS5600 lezen via TCA9548A mux. Motor- én arm-as kanalen | mock-baar (I2C abstr) |
| `tmc_drivers`    | TMC2209 setup, current control, edge-detect status poll  | mock-baar |
| `pid`            | Per-as PID controller (KP/KI/KD, antiwindup, deadband)   | **volledig** |
| `soft_endstops`  | Range-check + clamp-logica voor arm-as posities          | **volledig** |
| `motion_file`    | CSV opslag/lezen van trajectories op SD                  | gedeeltelijk |
| `recording`      | Recording FSM (start/poll/stop, filter samples)          | met mock-deps |
| `playback`      | Playback FSM (Hermite-smoother, ref-generation)          | met mock-deps |
| `homing`         | Homing detectie + zeroing van encoders                   | met mock-deps |
| `commands`       | Serial `>VERB` parser + dispatcher naar handlers         | **volledig** (pure string-in) |

## Dataflow

```
   ┌─────────┐    ┌──────────┐    ┌─────┐    ┌──────────────┐
   │ encoders├───▶│ playback │───▶│ pid ├───▶│ tmc_drivers  │
   └─────────┘    └──────────┘    └─────┘    └──────────────┘
        │              ▲                              │
        │              │                              ▼
        │         ┌────┴──────────┐               (TMC2209)
        │         │ motion_file   │
        │         └───────────────┘
        │
        └────────▶ soft_endstops ─────▶ playback (clamp ref)
```

Tijdens **RECORDING**: `encoders → recording → motion_file`.
Tijdens **PLAYBACK**: `motion_file → playback → (clamp by soft_endstops) → pid → tmc_drivers`.
Tijdens **HOMING**: `encoders → homing → pid → tmc_drivers`, met state in `nvs_storage`.

## Design-keuzes

### Geen documentatie-blokken in source

Implementation-details horen in code: variabele-namen, korte commentaarregels bij "waarom" (niet "wat"). Architecturele context, motivatie, contracten: in markdown.

**Reden:** code drift sneller dan context-blokken; markdown evolueert per release, code per regel.

### Pure functions waar mogelijk

`pid::compute()`, `soft_endstops::clamp()`, `commands::parse()` hebben **geen side-effects** en alleen value parameters. Maakt unit-tests triviaal.

### State in expliciete structs

Geen globals binnen modules. Iedere module exposeert een `State`-struct die `main.cpp` instantieert en doorgeeft. Maakt mocking en isolation makkelijker.

### TMC2209 lib: declarative via platformio.ini

`lib_deps = janelia-arduino/TMC2209` in `platformio.ini` — geen handmatig kopiëren van library bestanden zoals bij Arduino IDE.

## Referentie

v19's monoliet ligt in [`../v19/robot_arm_v19/robot_arm_v19.ino`](../../v19/robot_arm_v19/robot_arm_v19.ino) als historische referentie.
