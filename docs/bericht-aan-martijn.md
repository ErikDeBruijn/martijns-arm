# Concept bericht aan Martijn

Korte versie (Whatsapp/Telegram):

---

> Hey Martijn,
>
> Ik heb besloten dat ik thuis ook met de robotarm aan de gang wil — handig om door te kunnen werken zonder dat jouw arm hier op tafel staat. Voor v3 zou ik in elk geval willen bestellen:
>
> - SparkFun ESP32-S3 Thing Plus
> - 3× TMC2209 (BTT v3) drivers
> - 6× AS5600 + diametrische magneten
> - TCA9548A I2C mux
> - 24V/5A voeding, XT60, 5V buck
>
> Vraag: welke NEMA-17 motors gebruik jij precies? Ik heb hier nog wat 17's liggen maar weet niet zeker of die qua boutgaten/schachtdiameter passen op jouw brackets. En GT2 belts/pulleys — welke maten?
>
> En wat doen we met moertjes/boutjes en bearings — bestel ik die zelf mee, of heb jij voorraad waarvan een setje meekan?
>
> Heb een uitgebreide bestellijst klaar liggen die ik kan delen, dan kun je aanvullen wat ik mis.
>
> Cheers,
> Erik

---

Iets formeler (mail):

---

> Onderwerp: Bestellijst hardware voor mijn eigen arm-instance v3
>
> Hoi Martijn,
>
> Tussen de sessies door werk ik thuis aan dezelfde ROS2 / firmware stack. Om ook hands-on te kunnen doen wil ik een eigen kopie van iteratie 3 hier hebben. Ik heb de elektronica al op een rij staan op basis van wat we tot nu toe hebben gebruikt:
>
> - SparkFun ESP32-S3 Thing Plus
> - 3× TMC2209 SilentStepStick (BTT v3)
> - 6× AS5600 (Adafruit) + 6× D6×3DM diametrische magneten
> - TCA9548A I2C multiplexer
> - 24V/5A meanwell + 5V buck + XT60
> - Bekabeling, JST connectors
>
> Ik heb hier al een paar NEMA-17 motoren liggen, maar ik weet niet of die identiek zijn aan wat jij gebruikt — kun je bevestigen welk model (Kysan 1124090?) en het schachtdiameter dat in jouw brackets past?
>
> Wat ik nog niet weet:
> - GT2 timing belt lengtes (afhankelijk van jouw arm-geometrie)
> - Pulley aantallen + maten — 20T motor / 70T arm voor 3.5:1, klopt dat?
> - Bearings per joint (6800 / 608 / 6700?)
> - M3 bouten en messing inserts — bestel ik mee of heb jij die?
>
> De volledige bestellijst staat in `docs/bestellijst-iteratie3.md` van het project (kan ik mailen). Voel je vrij om aan te vullen of te corrigeren.
>
> Groeten,
> Erik
