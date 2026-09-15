# Changelog

## 1.6.4 — 2026-09-15

- Optional AI hive analysis: **Analyze hive** button and a Configure AI dialog on the page.
  Providers: OpenRouter (default model `openrouter/free`) or the OpenAI API (default
  `gpt-5.6-luna`). Bring your own key; it is stored in NVS, never returned by the API, never
  exported. New endpoints `/api/ai/status|config|test|analyze|delete`. Requests run in their
  own FreeRTOS task with a 60 s timeout and a 6000-character result cap.
- The prompt sends numbers only (score, components, band deviations, gas fingerprint, 60-minute
  summary, newest five events with keeper notes) and instructs the model to treat the score as
  a hypothesis and to avoid diagnoses. Documented in docs/AI-ANALYSIS.md.
- 1.6.4 specifically: compatibility with OpenRouter's free router when it selects a reasoning
  model — reasoning is requested at minimal effort and excluded from the reply, content parts
  are accepted as well as a single string, and a reply with no final text is retried once.
- TLS to the provider is not certificate-verified in this version (documented).

Versions are the `FW_VERSION` in `firmware/apiary_ears/apiary_ears.ino`.

## 1.5.0 — 2026-09-15

**One settled sample per burst — the sawtooth had a deeper cause than position 1.**

Measured on a running node: **10368 kΩ at burst position 3, 13152 kΩ at position 5** — a
27 % spread between scans in the same burst, with nothing in the air changing. Resistance
climbs all the way through a burst as the sensing surface recovers, so excluding only
position 1 (v1.4.1) was not enough: any average over a mixed bag of positions jumps with
whatever mix that minute happened to hold, which is what produced the spikes in groups.

- **Trends now use exactly one scan per burst**: the last one before the rest, the most
  equilibrated. Every point is then comparable with every other. That is one sample about
  every 150 s — ample for something that moves over hours.
- **The smell baseline follows the same rule.** It was being moved by whichever positions
  happened to arrive, which is why the deviation figure wandered.
- **Both charts now default to 24 hours**, which is the view that shows the shape of a day.
  "Last hour" is still there and still raw: every scan as measured, including the climb
  through each burst, because that is what raw means.
- **Clearer wording.** "−27 % below baseline" now reads "27 % less gas than usual", which
  says which way it moved without needing to reason about the sign.

Flash this one.

## 1.4.2 — 2026-09-14

**Fixes a compile error in 1.4.1.**

`mean` was used one block above where it is declared, so 1.4.1 would not build:
`error: 'mean' was not declared in this scope`. The mean of the ten heater steps is now
computed once, at the top of the block, before anything uses it. A forward declaration was
also added for `writeWavHeader()`, which `clipSave()` calls before its definition.

Both were found by compiling the sketch against stub headers rather than by reading it —
brace counting cannot catch a scope error, and now does not have to.

## 1.4.1 — 2026-09-14

**The score/smell/temperature chart was drawing mountains out of nothing.**

Three causes, all now fixed:

- **Temperature was stored rounded to whole degrees.** A hive drifting across 24↔25 °C
  became a full-height square wave. Minute rows now keep tenths.
- **The minute's smell figure was sampled, not averaged.** Whatever `nose_mean` happened to
  hold when the minute ended went into the row — including the first scan after the sensor
  rests, which reads several times high for reasons that have nothing to do with the air.
  Each minute now averages only settled scans.
- **Auto-scaling magnified whatever was left.** Fitting a nearly-flat series to the full
  height of the chart turns a rounding wobble into a range of peaks. Every series now has a
  minimum span — at least 2 °C for temperature, 8 % of the reading for smell — so flat
  things look flat and real movement still fills the chart.

Flash this one: two of the three are in the firmware.

## 1.4.0 — 2026-09-14

**A day of smell, a full snapshot with every recording, and the rail monitor removed.**

- **Rail voltage monitoring is gone.** It needed a resistor divider that has to be built,
  and a panel should not advertise hardware that is not there. Removed from the firmware,
  the page and the wiring guide.
- **A day of fingerprints.** The spectrogram gained a window switch: *last hour* at one
  column per scan, or *24 hours* at one column per minute, averaged. A day of raw scans
  would not fit in memory and would not say more — smell moves over hours, so a minute is
  already finer than the signal. Costs about 35 KB of RAM.
- **Every recording now saves a snapshot of everything**, not just audio. Beside each WAV
  the board writes a small JSON with the score and its four components, all twelve band
  levels and their deviations from baseline, the ten-step fingerprint with its burst
  position, the smell baseline and deviation, temperature, humidity, pressure and the
  temperature trend. About a kilobyte beside a 320 KB clip.
- **Retention treats a recording as a set.** The audio and its snapshot are deleted
  together, because either one alone is much less useful than the pair.

Flash this one.

## 1.3.1 — 2026-09-14

**Wording fix: "not wired" read as a complaint about the sensor.**

The Node panel said "3.3 V: not wired" when no rail monitor was fitted, which looks like it
is reporting that something is unpowered — confusing when your BME688 is plainly connected
to 3.3 V and working. It now says "3.3 V rail monitor: not fitted (optional)", and the note
underneath explains the distinction: an ESP32 cannot read its own supply, so measuring the
rail needs two resistors and one `#define`. Nothing about the sensors changed.

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
