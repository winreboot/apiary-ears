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

## Optional: rail voltage monitor (two resistors)

An ESP32 cannot read its own supply, so seeing the rail needs a divider. It is worth doing
on any node with a long cable, because **the number that matters is the sag**: a rail that
reads 3.30 V but dips to 2.95 V while the BME688's heater pulses is what resets sensors,
and nothing else on the page will tell you that is happening.

**3.3 V rail:** two **100 kΩ** resistors in series from 3V3 to GND. Take the midpoint to a
spare ADC pin — GPIO7 is a reasonable choice on most S3 boards. The midpoint sits at half
the rail, about 1.65 V, comfortably inside the ADC's range.

**5 V rail (optional):** **100 kΩ** from 5V to the pin and **47 kΩ** from that pin to GND,
which divides 5 V down to about 1.6 V.

Then set the pins at the top of the sketch:

```cpp
#define VMON_33_PIN     7          // -1 = not fitted
#define VMON_33_RATIO   2.00f      // 100k / 100k
#define VMON_5V_PIN     -1
#define VMON_5V_RATIO   3.13f      // 100k / 47k
```

Leave them at `-1` and the Node panel simply says "not wired" rather than inventing a
figure. With them fitted you get a coloured reading:

| colour | 3.3 V rail | means |
|---|---|---|
| green | 3.20 – 3.40 V | healthy |
| honey | 3.05 – 3.50 V | drooping, or the regulator is running high |
| red | below 3.05 or above 3.50 V | this is where sensors reset and readings go strange |

The panel also shows the **minimum seen in the last minute** whenever it differs from the
current reading by more than 50 mV. That is the dip, and it is the thing to watch.

Accuracy is a few tens of millivolts — `analogReadMilliVolts()` uses the chip's factory
calibration. Good enough to spot a sagging supply, not a substitute for a meter when
chasing a fault.
