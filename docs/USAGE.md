# Using it

## Three ways to capture

**Live listen** streams the microphone to your browser. It is the thing a beekeeper already
does — an ear against the box — without opening it. Five minutes per connection.

**Record 10 s** captures a labelled clip into memory. Download it before the board reboots:
clips live in RAM, not on disk.

**Automatic events** need nothing from you. The board watches its own score and records an
event whenever the colony sounds unusual *for this hive*, keeping the band deviations, the
fingerprint, the conditions and a clip. See [SCORING.md](SCORING.md).

## Reading the page

**Score** — 0 to 100 against this hive's recent baseline. Green below 45, honey to 70, red
above. A quiet colony sits near zero all day.

**Components** — fanning, agitation, piping, environment. The breakdown matters more than
the total: a 60 that is all *fanning* on a hot afternoon is ventilation; a 60 with *piping*
in it is worth walking out to look.

**Sound — twelve bands.** Bars turn honey then red as they rise above baseline.

**Smell — ten heater steps.** The BME688's fingerprint, with its burst position underneath.
Position 1 is the first scan after the sensor rested; it reads high for reasons that have
nothing to do with the air, so it is shown but kept out of the baseline.

**Score, smell and temperature** on one chart. The smell line is inverted so that *more gas
is higher*, which makes it read the same direction as the score.

**Events** — what the board caught on its own. Click the note column and say what was
happening.

## The habit that makes this worth doing

Annotate events the same day. "Inspection", "mower next door", "nothing — I was there and
they were calm." All three are valuable, and a week later you will not remember.

A dataset of unlabelled score spikes teaches nobody anything. One where half the events say
what happened is the first of its kind.

## Recordings and storage

Clips are written to the board's flash, so they survive a reboot. Flash is small: a
ten-second 16 kHz mono clip is about 320 KB, and a typical partition holds four or five.

The board manages that itself. Before each recording it deletes the oldest clips until
there is room, keeps at most six, and never fills the partition completely. The page shows
usage as a bar — green, then honey, then red as it fills — with a download link beside each
recording. **Download anything worth keeping**; it will eventually be deleted to make room.

To keep more, raise `CLIP_KEEP` and choose a partition scheme with a larger filesystem
(Tools → Partition Scheme), or lower `CLIP_SECONDS`.

## The Node panel

Uptime, die temperature, heap, PSRAM, signal strength and IP, plus a verdict for each
input: the microphone, the BME688 and the clock. Each says which pins are involved and
what to do when it is unhappy — a silent microphone suggests checking L/R is tied to GND
before anything else, a missing BME688 suggests measuring 3.3 V at the sensor's own pins.

## Exporting

Label it, choose the minutes, download. You get minute-by-minute band energies, scores,
smell and conditions, plus every event with its full breakdown. Validate with
`python tools/validate.py yourfile.json`.

## API

| endpoint | does |
|---|---|
| `GET /api/now` | score, components, bands, fingerprint, conditions |
| `GET /api/history?minutes=180` | per-minute rows |
| `GET /api/events` | recorded events |
| `POST /api/note?epoch=…&text=…` | annotate an event |
| `POST /api/record?label=…` | start a 10 s clip |
| `GET /clip.wav` | download the last clip |
| `GET /listen.wav` | live stream |
| `GET /api/export?label=…&minutes=…` | full session JSON |
