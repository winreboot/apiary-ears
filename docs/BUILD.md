# Building and flashing

## Parts

| what | notes |
|---|---|
| **ESP32-S3** devkit | PSRAM is worth having: without it, clips drop from 10 s to 2 s |
| **INMP441** I²S microphone | the common breakout is fine |
| **BME688** gas sensor | Adafruit, SparkFun, Seengreat and the generic CJMCU boards all work |
| 2 × 2.2 kΩ resistors | pull-ups for the I²C side, if your breakout lacks them |

About $40. An afternoon.

## Arduino IDE

1. Install **Arduino IDE 2.x**.
2. **File → Preferences → Additional board manager URLs**:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
3. **Boards Manager** → install **esp32 by Espressif**.
4. **Tools → Board → ESP32S3 Dev Module**. If your board has PSRAM, set
   **Tools → PSRAM → Enabled** — that is what gives you ten-second clips.
5. **Library Manager** → install **bsec2** (Bosch Sensortec). Accept the **BME68x Sensor
   library** dependency. That is the only library this project needs.

## Flash

Set your network at the top of `firmware/apiary_ears/apiary_ears.ino`:

```cpp
#define WIFI_SSID  "YOUR_WIFI"
#define WIFI_PASS  "YOUR_PASSWORD"
```

Upload, then open the Serial Monitor at **115200**:

```
=== apiary-ears fw 1.0.0 ===
[wifi] connecting....
[open] http://192.168.1.61/
[bme688] found at 0x77, scan mode (10 heater steps, ~11 s per scan)
[bme688] a new sensor needs 24-48 h of running before its gas readings settle.
[http] listening on :80
[nose] scan 1 (10/10 steps, burst pos 1) mean 214 kOhm, dev 0.00
```

**`10/10 steps` is the number to check** — it means the whole fingerprint is being captured.

## If something is wrong

**Score at 0 and the level reads −120 dBFS** — the mic is silent. Check L/R is tied to GND
and that SD/WS/SCK are on the right pins. A mis-wired INMP441 fails quietly.

**`[bme688] nothing answers`** — the firmware then scans the bus and lists what it found.
Nothing at all means power or the SDA/SCL pair; other addresses but not the sensor means the
address switch, or SDA and SCL swapped. Remember the pin marked **MOSI** is the data line.

**`x/10 steps` with x below 10** — some heater steps are not arriving; usually a marginal
supply. Shorten the cable or add a capacitor at the sensor.

**Clips are only 2 seconds** — no PSRAM, or PSRAM disabled in Tools.

**Score pinned high** — the baseline has not settled (give it a minute), or the mic is
clipping. `dBFS` near 0 with "CLIPPING" means the mic is too close to the entrance.

**Smell readings that drift for a day** — that is burn-in, not a fault. Bosch advise 24–48 h
of continuous running on a new sensor. Leave it going.

**Live listen plays nothing** — some browsers refuse an endless WAV. Open
`http://<board>/listen.wav` directly, or use VLC → Open Network Stream.
