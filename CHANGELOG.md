# Changelog

Versions are the `FW_VERSION` in `firmware/apiary_ears/apiary_ears.ino`.

## 1.3.0 — 2026-09-14

**Fix things without walking back to the computer.**

- **↻ Self-check.** Re-probes the I²C bus, retries the BME688 if it had gone missing, and
  takes a fresh microphone reading. Plug a cable back in, press it, and the diagnostics
  update — no reboot, no reflash. It reports what changed ("missing → ok. Found it.").
- **Restart node** from the page, with a confirmation that says what survives (recordings)
  and what does not (the score baseline, the events list). The page then watches for the
  board coming back and tells you when it is up.
- **Rail voltage, if you fit two resistors.** See [docs/WIRING.md](docs/WIRING.md). The
  Node panel shows 3.3 V and optionally 5 V, coloured green / honey / red, plus **the
  minimum seen in the last minute** whenever it differs — because the sag during a heater
  pulse is what resets sensors, not the average. Unfitted monitors say "not wired" rather
  than showing a made-up figure.

Flash this one.

## 1.2.0 — 2026-09-14

**Recordings survive a reboot, the board manages its own flash, and the page gained the
two visualisations the smell side was missing.**

- **Fingerprint over time.** A spectrogram of the last few hundred scans: one column per
  scan, one row per heater step. A smell arriving shows as a vertical band.
- **Recurring patterns.** The same automatic clustering the nose project uses — fingerprint
  shapes with intensity divided out, grouped into four. A new pattern appearing means
  something changed, before anything has been trained.
- **Recordings are written to LittleFS** instead of living in RAM, so they survive a
  reboot, and there can be more than one. They are listed on the page with their size and
  a download link.
- **The board manages storage itself.** Flash is small — about five ten-second clips fit in
  a typical partition — so before every recording it deletes the oldest until there is
  room, keeping at most `CLIP_KEEP` (6) and never leaving less than `CLIP_FREE_MIN` free.
  The page shows what is used and what is left, coloured as it fills.
- **Node panel with per-input diagnostics.** Uptime, die temperature, heap, PSRAM, signal
  and IP, plus a verdict for the microphone, the BME688 and the clock — each with the pins
  involved and what to actually do when it is unhappy, rather than just a red light.
- **Live listen plays in the page.** It no longer opens a second window, so the charts keep
  updating while you listen. Pressing it again stops the stream.

Flash this one: the changes are in the firmware, not the documentation.

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
