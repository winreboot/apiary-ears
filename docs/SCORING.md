# How the score works, and what it is worth

## The short version

Every two seconds the board measures energy in twelve frequency bands, compares each
against **that hive's own rolling baseline**, and combines the deviations:

```
score = 0.25 × fanning  +  0.35 × agitation  +  0.30 × piping  +  0.10 × environment
```

| component | source | meant to catch |
|---|---|---|
| fanning | 60–180 Hz | wing fanning and the general colony roar |
| agitation | 225–315 Hz | worker agitation; rises when a colony is disturbed |
| piping | 360–500 Hz | queen tooting, with quacking a little lower |
| environment | BME688 | the smell moving, and temperature climbing |

Acoustic baselines track over roughly half an hour; the smell baseline tracks over hours,
because that is the timescale smell actually moves on. Everything is relative to the hive's
own normal, so a loud apiary and a quiet one are comparable.

## Why sound and smell together

They answer different questions on different clocks:

- **Sound changes in minutes.** Someone knocks the hive, a wasp gets in, the queens pipe.
- **Smell changes over hours and days.** Brood, nectar coming in, something decaying.

That difference is the useful part. A score spike with **no** movement in the fingerprint is
most likely mechanical or weather. A spike where **both** move is a colony doing something.
The page shows them on the same chart for exactly this reason.

## What this is not

**The weights are a hypothesis.** They come from published descriptions of pre-swarm
acoustics, not from validated measurement on this hardware. They may be wrong.

That is why every event stores its **raw band deviations in dB and the raw ten-step
fingerprint in kΩ**. If the weights are wrong — or someone trains something better on the
shared data — the same events can be re-scored without re-recording anything.

**A high score is not a swarm.** An inspection, a lawnmower, a thunderstorm and a wasp raid
all raise it. That is why events carry a note field, and why annotating them is the most
valuable thing a contributor does.

## The burst cycle, and why the smell baseline ignores it

BSEC scan mode does not run continuously: it scans a few times, then rests for about a
minute and a half. During the rest the sensing surface recovers, so the **first scan after a
pause reads several times higher** than the last one before it — a swing far larger than
most smells produce.

Every scan records its position in the burst. Position 1 is kept in the record but excluded
from the baseline and from the environment component, so the rest cycle cannot masquerade as
a change in the air.

## Events

An event opens when the score stays at or above **55** for three consecutive windows and
closes when it falls below **35**, with a five-minute cooldown. Each event records:

- the peak score and its four components
- the band deviations at the peak, in dB
- the ten-step fingerprint at the peak, in kΩ, plus how far below baseline the smell was
- temperature, humidity, and the rate of temperature change
- your note

Thresholds are `EVENT_ON`, `EVENT_OFF` and `EVENT_COOLDOWN_S` at the top of the sketch.

## Swarm prediction — the honest position

The goal is to catch a swarm **before** it happens. What is known: colonies preparing to
swarm grow noisier and more agitated over days, and queen piping in the final day or two is
well documented and distinctive. What is not known is whether a cheap microphone in a hive,
scored this way, separates that reliably from an ordinary busy day — across different hives,
climates and seasons.

Answering that needs recordings from swarms that actually happened, labelled by beekeepers
who saw them. That is the point of the dataset. Until then, treat the score as something to
look at alongside your own eyes, not as a warning system.

If you catch a swarm on this, the recording either side of it is the single most valuable
file this project could receive.
