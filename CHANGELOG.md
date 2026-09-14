# Changelog

Versions are the `FW_VERSION` in `firmware/apiary_ears/apiary_ears.ino`.

## 1.1.0 — 2026-09-14

**Sharing recordings, and an honest account of CO₂.**

- **Clip submissions.** A recording can now be contributed as a pair of files: the WAV
  itself and a JSON sidecar. See [docs/CLIPS.md](docs/CLIPS.md) and
  [schema/acoustic-clip-v1.schema.json](schema/acoustic-clip-v1.schema.json). The audio is
  the portable part — band energies depend on whichever firmware produced them, raw
  samples do not — and the sidecar carries the event type from a fixed vocabulary, the
  microphone placement, the site and the capture context.
- **Documented why there is no CO₂ reading.** A BME688's CO₂-equivalent needs BSEC's IAQ
  mode running continuously to reach an internal run-in. A node that tried ran fourteen
  scheduled windows — 15 minutes, then 30 — and produced nothing at all, because every
  return to scan mode discards the progress. Fit an SCD41 if you want CO₂; it measures
  rather than infers, and does not compete for the heater.
- **Stated plainly that both sensors are required.** No build option runs the microphone
  or the BME688 alone: the pairing is the point of the build.

No change to scanning, scoring, event detection or the web page. Flashing is optional.

## 1.0.0 — 2026-09-14

First release.

- INMP441 microphone and BME688 on one ESP32-S3, both required
- Twelve Goertzel bands, 60–800 Hz, each measured against that hive's own rolling baseline
- Score from fanning, agitation, piping and an environment term built from the smell and
  the temperature trend — documented as a hypothesis, not a prediction, with raw band
  deviations stored so any event can be re-scored later
- Three ways to capture: live listen, manual ten-second clips, and automatic events
- Burst position recorded with every fingerprint, and kept out of the smell baseline: the
  first scan after the sensor rests reads several times high for reasons that have nothing
  to do with the air
- Self-hosted page, minute-by-minute history, JSON export matching the session schema
