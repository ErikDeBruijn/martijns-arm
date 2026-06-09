# Bestellijst — Iteratie 3 hardware

> Op basis van wat Martijn voor v18 gebruikt (zie `martijns-arm-hardware`
> memory). Erik heeft al een paar NEMA-17 stepper motoren, dus die zijn
> mogelijk niet nodig.
>
> **Wat Erik thuis heeft (aannames, te verifiëren):**
> - Enkele NEMA-17 stepper motors (model? 200-step / 400-step?)
> - Soldeer setup, 3D printer, multimeter
>
> **Wat Martijn levert / al gedaan heeft:**
> - 3D-print mechanische onderdelen
> - PCB / breadboard layout
> - Firmware (v18.1, v19 in voorbereiding)
> - Bouten/moertjes? — vragen aan hem

## Elektronica core (per arm-instance)

| # | Component | Model / specificatie | Aantal | Bron NL/EU |
|---|---|---|---|---|
| 1 | Microcontroller | **SparkFun ESP32-S3 Thing Plus** | 1 | TinyTronics, Floris.cc, SparkFun direct |
| 2 | Stepper drivers | **TMC2209 SilentStepStick v3.0** (BIGTREETECH) | 3 (4 voor reserve) | Kiwi Electronics, 123-3D, Aliexpress |
| 3 | Encoder ICs | **AS5600 magnetic angle sensor breakout** (Adafruit) | 6 | Kiwi Electronics, Adafruit, Mouser |
| 4 | I2C multiplexer | **TCA9548A breakout** (Adafruit ADA-2717) | 1 | Kiwi Electronics, Adafruit |
| 5 | Encoder magneten | **Diametrisch gemagnetiseerde schijfmagneet 6×3mm** (D6x3DM) | 6 (10 voor reserve) | Supermagnete.de |
| 6 | Stepper motors | **NEMA-17 200-step (1.8°) Kysan 1124090** of equivalent | 3 minus wat Erik heeft | Kiwi Electronics, RobotShop |

## Voeding & connectors

| # | Component | Specificatie | Aantal |
|---|---|---|---|
| 7 | Voeding | **24V 5A DC** (120W meanwell of equivalent) | 1 |
| 8 | XT60 plug + chassis socket | M+F paar | 2 paar |
| 9 | Step-down naar 5V | **buck converter 24→5V 3A** | 1 |
| 10 | Decoupling caps | 470µF 35V elcaps voor TMC drivers | 6 |

## Bekabeling

| # | Component | Specificatie | Aantal |
|---|---|---|---|
| 11 | Silicone wire | AWG18 rood/zwart 5m + AWG24 multi-color 10m | 1 set |
| 12 | JST-XH 4-pin connectors | voor stepper motor cables | 6 paar |
| 13 | JST-SH 4-pin connectors | voor I2C/AS5600 connecties | 12 paar |
| 14 | Krimpkous-set | gemengde maten | 1 |

## Mechanisch (vragen aan Martijn)

| # | Component | Aantal |
|---|---|---|
| 15 | M3 inbus bouten 8/12/16mm | 50 elk |
| 16 | M3 zelfborgende moeren | 50 |
| 17 | M3 messing inserts (heat-set) | 30 |
| 18 | GT2 timing belts (open + closed) | per asreductie |
| 19 | GT2 pulley 20T 5mm bore (motor side) | 3 |
| 20 | GT2 pulley 70T (3.5:1 reductie, arm side) | 3 |
| 21 | Lager 6800 / 608 / 6700 | per joint |

## Experimenteer-extras (optioneel)

| # | Component | Voor |
|---|---|---|
| 22 | Tweede ESP32-S3 dev board | E S P NOW dongle (wireless link laptop ↔ arm) |
| 23 | Logic analyzer (Saleae 8/16) | UART / I2C debugging |
| 24 | Bench power supply | precieze 0-30V regelbaar voor tests |
| 25 | Reservedrijver TMC5160 | voor latere upgrade — meer torque dan 2209 |
| 26 | Extra AS5600 + magneet | reserve voor breken/ruil |

## Geschatte totalen (NL bestelling)

- Elektronica core: **~€150-180**
- Voeding+kabels: **~€60**
- Mechanisch (afhankelijk van Martijn's antwoord): **€30-100**
- Optioneel experimenteer: **+€100-200**

## Verifiëren bij Martijn

- Welke NEMA-17 motoren past bij zijn brackets? (boutgaten, schachtdiameter)
- Welke timing belt lengte heeft hij? (afhankelijk van armgeometrie)
- Levert hij de mechanische onderdelen of moet Erik zelf 3D-printen?
- Welke moertjes/boutjes heeft hij — Erik bestellen of komt het uit zijn stock?
