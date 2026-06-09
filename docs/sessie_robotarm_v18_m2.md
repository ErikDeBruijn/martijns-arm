# Robotarm v18 — sessie-overdracht

## Context

Derde iteratie robotarm: zwaarder en stijver dan vorige. **M2 (schouder)** draagt het hele arm-gewicht. Probleem: traag en slingerig playback. Eerdere v14-tuning (lichtere arm) werkt niet meer.

## Hardware

- ESP32-S3 SparkFun Thing Plus
- 3× TMC2209 via UART (Serial1, TX=GPIO17 met 1kΩ, RX=GPIO16, EN=GPIO15, R_SENSE=0.11)
- 256 microsteps
- AS5600 op TCA9548A mux: motor-shaft kanalen 7/6/5, arm-shaft kanalen 4/3/2
- 3.5:1 reductie via tandriemen (was eerst tandwielen — minder speling nu)

## Diagnose (gecorrigeerd)

**M2 is onder-getuned**, niet over-getuned. De time-scaler vertraagt het hele traject omdat M2 niet kan bijbenen tegen de zwaartekracht-belasting.

### Bewijs uit v18.ino

```cpp
// regel 252-254
static float KP_SPEED[3] = { 6.5f,  3.0f,  6.8f };  // M2 = laagste
static float KD_SPEED[3] = { 3.0f,  2.5f,  3.5f };  // M2 = laagste
```

M2 (zwaarste belasting) heeft de **laagste** gains van alle drie de motoren. Dit is in lijn met de tuning-historie (lijn 248-251 commentaar): na overstap van tandwielen→tandriemen werden gains stapsgewijs verlaagd om vermeende oscillatie te dempen, maar daardoor is de regelaar nu te slap voor gravity load.

### Bewijs uit logs

**Snelle opname (7.5s → 57s playback):**
- M2 maxErr = **15.75°** (M1=0.68°, M3=1.07°)
- M2 maxCmdPre = 57% (geen clipping — headroom over)
- minTimeScaleLP = 0.701

**Trage opname (8.9s → 68.7s playback):**
- Alleen M2 bewogen, maxErr = **3.15°**
- minTimeScaleLP = 0.732
- M2 maxCmdPre = 12%

Beide tonen hetzelfde patroon: M2 sleept consistent achter, time-scaler vertraagt traject. Zelfs bij langzame opname blijft het probleem bestaan → bevestigt structurele tuning-mismatch.

### Keten van het symptoom

1. M2 KP=3.0 te slap voor gravity load
2. err groeit (3-15° afhankelijk van snelheid)
3. `errorRatio → 1` → `timeScale → 0.70-0.73`
4. `refVelScaled = refVel * timeScale` → traject vertraagt
5. Visueel: trage, schokkerige playback (sc springt 0.71↔1.00)

## Open vragen

### 1. Steps/rev inconsistentie

```cpp
// regel 218
static const long STEPS_PER_REV = 102400;   // 256 × 400 fullsteps

// regel 258 commentaar
// Snelheidslimieten in steps/s (bij 51200 steps/motoromw)
```

51200 = 256 × 200 fullsteps. Eerder gaf Martijn aan: "motoren niet allemaal 400 staps". Vraag: is M2 daadwerkelijk een 1.8°-motor (200 fullsteps), terwijl de firmware 0.9° (400 fullsteps) aanneemt? Dat zou de feedforward `refVel` voor M2 op 2× verkeerde schaal zetten.

**Test om uit te zoeken:** kijk op het label van de M2-motor zelf, of meet 1 omwenteling met de calibratie-sketch uit eerdere sessie (`robotarm_steps_calibration.ino` op /mnt/user-data/outputs/).

### 2. PID-aanpassing voor M2

Nog niet uitgevoerd. Wachten tot steps/rev mismatch opgelost is — anders tunen we PID rond een verkeerde fysica.

**Indien steps/rev=400 klopt voor M2:**
- KP_SPEED[1]: 3.0 → 5.0 (proefstap)
- KD_SPEED[1]: 2.5 → 3.5
- Eerst proeven, dan eventueel bijschaven

**Indien M2 = 200 stepper:**
- Per-as STEPS_PER_REV introduceren (array i.p.v. constante)
- Of `refVel` voor M2 met factor 2 corrigeren in de feedforward

## Werkprincipes (Martijn)

- Eerst diagnose, dan code
- Monolithische .ino files
- Hands-on, iteratief, log-gedreven
- Werktaal: Nederlands

## Bestanden

- `/mnt/user-data/uploads/1777816729646_robot_arm_v18.ino` — actuele firmware (1481 regels)
- Snelle log + slow log: in conversatie-history van vorige sessie
- `robotarm_steps_calibration.ino` (eerdere sessie): VACTUAL-gebaseerde steps/rev meet-sketch

## Direct volgende stap

**M2 motor-label checken** of de calibratie-sketch flashen. Pas daarna PID-aanpassing.
