# Wiring

![Apiary Ears wiring](wiring.svg)

Two sensors, ten wires, two resistors.

## Microphone — INMP441 (I²S)

| mic pin | goes to | note |
|---|---|---|
| SD  | **GPIO4** | data out of the mic |
| WS  | **GPIO5** | word select |
| SCK | **GPIO6** | bit clock |
| L/R | **GND**   | selects the left channel — the firmware reads left only |
| GND | **GND**   | |
| VDD | **3V3**   | |

I²S is a driven bus: no pull-ups, no capacitor. The INMP441 needs a bit clock above about
1 MHz or it sleeps and returns silence — 16 kHz × 32 bit gives it enough, but drop the
sample rate much further and the mic goes quiet with nothing in the logs to say why.

## Smell, temperature, humidity, pressure — BME688 (I²C)

| BME688 pin | goes to | note |
|---|---|---|
| VCC | **3V3** | |
| GND | **GND** | |
| SCL | **GPIO13** | clock |
| SDA (labelled **MOSI** on many boards) | **GPIO21** | **the pin marked MOSI is the I²C data line** |
| MISO, CS | — | leave empty (SPI only) |

**2.2 kΩ from SDA to 3V3 and from SCL to 3V3**, at the *sensor* end of the cable. Many
breakouts have them fitted already — check before adding a second pair, since two sets in
parallel pull the bus too hard.

Most modules answer at **0x77**; some have a switch for **0x76**. The firmware tries both
and, if neither answers, scans the whole bus and prints what it found.

**Do not touch the metal cap on the sensor.** Skin oils contaminate the gas element and the
contamination does not wash off.

## Placement

- **Mic inside**, through a small hole in the hive wall or just inside the entrance. Bees
  propolise anything they can reach, so a grille or fine mesh helps.
- **The BME688 inside too**, but not in the airflow of the entrance — it needs the hive's
  air, not the wind's. Away from the mic hole.
- **The board outside**, in a vented enclosure in the shade.
- **Wind is the enemy** of the microphone. Recess it slightly.
