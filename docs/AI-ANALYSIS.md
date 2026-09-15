# AI hive analysis (optional)

Since firmware **1.6.x** the Apiary Ears page has an **Analyze hive** button. When you press it,
the board packages the current sensor state into a text prompt, sends it to a language model
over HTTPS, and shows the reply on the page. Nothing happens until you press the button, and
nothing happens at all until you have configured a provider.

The firmware ships with **no API key**. You bring your own, from one of two providers:

| Provider | Get a key | Cost | Default model |
|---|---|---|---|
| **OpenRouter** (recommended) | [openrouter.ai/settings/keys](https://openrouter.ai/settings/keys) | free models available; the default routes to whatever free model is up | `openrouter/free` |
| **OpenAI API** | [platform.openai.com/api-keys](https://platform.openai.com/api-keys) | pay-per-use; a ChatGPT Plus/Pro subscription does **not** include API access, you need billing enabled on the API account | `gpt-5.6-luna` |

Either one works on its own. You do not need both.

## Setting it up

1. Open the board's page, scroll to **AI hive analysis**, press **✦ Configure AI**.
2. Pick the provider, paste the key, leave the model at its default (or type another model
   name that provider offers), press **Save & test connection**.
3. The badge turns **connected** when the test call succeeds. If it fails, the error from the
   provider is shown verbatim — usually a wrong key, a model name the account can't use, or
   no billing on an OpenAI account.
4. Press **✦ Analyze hive** whenever you want a second opinion. The board is busy for ten to
   sixty seconds; the page polls and fills in the result.

The key is stored in the ESP32's NVS (`apiary-ai` namespace), survives reboots and OTA, is
never returned by any API, and is never included in exports or recordings. **Delete AI
connection** wipes it. Wi-Fi credentials stay in the `.ino` exactly as before; AI credentials
are deliberately runtime-only so the same compiled firmware can be flashed by anyone.

Change the model without re-entering the key by leaving the key field blank when you save.

## What gets sent

The prompt is built by `buildHivePrompt()` in the firmware and contains only numbers and
labels — no audio, no recordings, no Wi-Fi or network details:

- current score, peak since boot, baseline-ready and in-event flags, the four score components
- microphone level (dBFS) and clipping flag
- temperature, humidity, pressure, temperature rate of change
- BME688 availability, burst position, mean and baseline resistance, relative gas change
- the twelve band deviations from this hive's rolling baseline (dB)
- the last ten-step gas fingerprint (kΩ per heater temperature)
- a summary of the last 60 minute-rows: score average and peak, temperature, humidity and
  settled-gas ranges
- the newest five board-detected events with peak, duration, components and your keeper note

The prompt tells the model, in plain words, that the score is an experimental hypothesis and
that it must not diagnose disease, queen status, swarming, poisoning or colony failure as fact.
The system message asks for a final answer rather than hidden reasoning; for OpenRouter the
request sets `reasoning: {effort: minimal, exclude: true}` so free reasoning models don't spend
their token budget thinking instead of answering.

## What comes back

The model is asked for five fixed headings, under 450 words:

```
STATUS — NORMAL, WATCH, or CHECK SOON, with one sentence why.
WHAT STANDS OUT — 2-4 bullets grounded only in the supplied measurements.
POSSIBLE EXPLANATIONS — alternatives, labelled as possibilities not diagnoses.
WHAT TO CHECK — non-destructive observations, highest priority first.
CONFIDENCE & LIMITS — low/medium/high, and what data is missing.
```

Treat it exactly as the heading says: a structured reading of numbers you can already see,
not an inspection. It cannot smell the hive or hear a queen pipe; it has the same twelve
numbers you do. A good use is "the score jumped, is anything else moving?" A bad use is
deciding whether to requeen.

## HTTP API

| Method | Path | Body / result |
|---|---|---|
| GET | `/api/ai/status` | provider, model, busy, last result/error, timestamps — never the key |
| POST | `/api/ai/config` | form fields `provider` (`openrouter`/`openai`), `key`, `model` |
| POST | `/api/ai/test` | starts a "reply with OK" round-trip; poll status |
| POST | `/api/ai/analyze` | starts an analysis; returns 202, poll status for the result |
| POST | `/api/ai/delete` | wipes provider, key, model and results from NVS |

Only one AI request runs at a time (409 while busy). Results are capped at 6000 characters on
the device; the HTTP timeout is 60 s.

## Limits worth knowing

- **TLS is not certificate-verified.** The board calls `setInsecure()`, so the connection is
  encrypted but a hostile network could impersonate the provider. Configure and use it on a
  private Wi-Fi you trust. (Bundling and rotating root CAs on an ESP32 is doable; it is not
  done in this version.)
- **The page itself is plain HTTP.** The key crosses your local Wi-Fi once, when you paste it.
- `openrouter/free` picks from whatever free models are available at that moment. Quality
  varies, and some return no final text on the first try — the firmware retries once and
  otherwise tells you to try again or pin a specific model.
- Model names are just strings passed through; if a default stops existing at a provider,
  type another one in the Model field. Nothing needs reflashing.
- A working analysis needs the baseline to be ready. Before that the deviations are all zero
  and the model has nothing to say.
