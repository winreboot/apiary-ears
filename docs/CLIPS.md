# Sharing recordings

A clip submission is **two files with the same name**: the audio, and a small JSON beside
it saying what it is.

```
data/<apiary-id>/clips/20260911-140301-hive1-swarm-preparation-suspected.wav
data/<apiary-id>/clips/20260911-140301-hive1-swarm-preparation-suspected.json
```

## Why the audio, and not just the numbers

Band energies, scores and thresholds all depend on the firmware that produced them. Mine
uses twelve Goertzel bands and a particular baseline; yours may not. Published side by side,
those numbers cannot be compared.

**Raw audio can.** Anyone can recompute any band scheme, any score, any model from a WAV,
years later, with tools that do not exist yet. So the WAV is the contribution and the JSON
is the label on it.

## The event vocabulary

`event.type` comes from a fixed list, deliberately:

| type | means |
|---|---|
| `routine` | nothing in particular. The baseline, and the most under-supplied class |
| `inspection` | the hive was opened |
| `swarm-issued` | a swarm actually left, and you saw it |
| `swarm-preparation-suspected` | queen cells, bearding, your judgement — say why in the notes |
| `queenless-suspected` | roaring, no eggs, your judgement |
| `robbing` | robbing in progress |
| `wasp-attack` | wasps or hornets at the entrance |
| `weather` | wind, rain or thunder dominating the recording |
| `machinery-nearby` | mower, chainsaw, traffic — worth keeping, teaches what to ignore |
| `other` | something real that none of the above covers; explain in the notes |
| `unknown` | you genuinely do not know. Honest and still useful |

Free text cannot be grouped across contributors; a fixed vocabulary can. If a type is
missing that you need, open an issue and it can be added to the list.

## `risk` and why it is only half-comparable

`event.risk` is 0–100 from whatever swarm-risk model the contributor runs, and
`risk_scale` says so in the file. Treat it as a **trend within one apiary**, not as a number
you can compare directly with someone else's — unless they have documented the same model.

The same caution applies to `conditions_at_capture`: temperature, CO₂ and weight come from
the contributor's own sensors and calibration.

## What makes a submission worth having

- **`routine` recordings.** The unglamorous class, and the one that makes every other label
  meaningful. If you only ever share the dramatic ones, nothing can be learned.
- **`swarm-issued`, above all.** A recording from a colony that actually swarmed, with the
  hours before it, is the single most valuable file this project can receive. Nobody has
  published one.
- **Honest `unsure`.** A confident wrong label is worse than no label.
- **Say where the mic is.** A microphone at the entrance and one over the brood nest hear
  different hives.

## Submitting

From the Dusk Apiary dashboard: **Listen → ↑** on any recording. It asks for the event type,
the risk and a note, then commits both files.

By hand: drop the pair into `data/<your-apiary-id>/clips/`, run
`python tools/validate.py` over the JSON, and open a pull request.

## Privacy

Audio recorded at a hive can pick up conversation. Listen before you publish.
