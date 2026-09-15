/*
 * =============================================================================
 *  Apiary Ears — listening to a colony, on one ESP32-S3
 * =============================================================================
 *
 *  An I2S microphone in the hive and a Bosch BME688 alongside it. The board
 *  scores what it hears every two seconds, records events on its own, and
 *  serves a page that draws all of it. No broker, no database, no server.
 *
 *  WHY SOUND AND SMELL TOGETHER
 *  They move on different clocks, which is exactly why they complement each
 *  other. Acoustics change in MINUTES - agitation, piping, a colony disturbed.
 *  Smell changes over HOURS AND DAYS - brood, nectar, decay. An event with a
 *  noise spike and no change in the fingerprint is probably mechanical or
 *  weather. An event where both move is a colony doing something.
 *
 *  WHAT THIS CLAIMS AND DOES NOT
 *  It measures energy in twelve frequency bands, compares each against THIS
 *  hive's own rolling baseline, and combines the deviations into a score. The
 *  score is a HYPOTHESIS, not a prediction. The weights come from published
 *  descriptions of pre-swarm acoustics, not from validated measurement on this
 *  hardware. Whether they anticipate a swarm is what the shared dataset exists
 *  to find out. Every event stores its raw band deviations and the raw ten-step
 *  fingerprint, so the same data can be re-scored later by anyone.
 *
 *  THREE WAYS TO CAPTURE
 *    live listen   - stream the microphone to your browser, right now
 *    manual        - press record, get a WAV plus the features around it
 *    automatic     - the board watches the score and records events itself
 *
 *  WIRING (docs/WIRING.md has the diagram)
 *    INMP441   SD  -> GPIO4    WS -> GPIO5    SCK -> GPIO6
 *              L/R -> GND      VDD -> 3V3     GND -> GND
 *    BME688    SDA -> GPIO21   SCL -> GPIO13  VCC -> 3V3   GND -> GND
 *              (the pin marked MOSI IS the I2C data line on these boards;
 *               MISO and CS stay empty. 2.2k pull-ups at the sensor end.)
 *
 *  LIBRARIES
 *    bsec2  (Library Manager, Bosch Sensortec - accept the BME68x dependency)
 *
 *  THE BURST CYCLE, which will fool you if you do not know about it
 *  BSEC scan mode does not run continuously: it scans a few times, then rests
 *  for about a minute and a half. During the rest the sensing surface recovers,
 *  so the FIRST scan after a pause reads several times higher than the last one
 *  before it. Every scan records its position in the burst, and the smell
 *  baseline ignores position 1 so the rest cycle cannot masquerade as a smell.
 *
 *  BOTH SENSORS ARE REQUIRED, ON PURPOSE
 *  This is one firmware for one board carrying a microphone AND a BME688. There
 *  is no build option to run either alone, because the whole argument for the
 *  build is the pair: sound moves in minutes, smell over hours, and a spike in
 *  one without the other tells you which kind of thing just happened.
 *
 *  WHY THERE IS NO CO2 READING
 *  The BME688 can report a CO2-EQUIVALENT, but only through BSEC's IAQ mode and
 *  only after that algorithm completes an internal run-in. Measured on a node
 *  that tried: fourteen scheduled IAQ windows, first at 15 minutes and then at
 *  30, produced no value at all - every return to scan mode discards the
 *  progress. Reaching run-in appears to need IAQ running CONTINUOUSLY, which
 *  would cost the fingerprint entirely. So this firmware does not offer CO2
 *  rather than offering a figure that is usually stale. If you want CO2 in a
 *  hive, an SCD41 on the same two wires measures it with an NDIR sensor and
 *  does not compete for the heater.
 *
 *  RECORDINGS ARE KEPT ON THE BOARD
 *  Clips are written to LittleFS, not held in RAM, so they survive a reboot.
 *  Flash is small: a ten-second 16 kHz mono clip is about 320 KB, so a typical
 *  1.5 MB partition holds four or five. The board therefore MANAGES the space
 *  itself - before every recording it makes room by deleting the oldest clips,
 *  keeping at most CLIP_KEEP of them and never filling the partition. The page
 *  shows what is used and what is left. Download anything you want to keep.
 *
 *  ONE SAMPLE PER BURST, NOT ONE PER SCAN
 *  Resistance climbs throughout a burst as the sensing surface recovers - on one
 *  node, 10368 kOhm at position 3 and 13152 kOhm at position 5, a 27 % spread
 *  with nothing in the air changing. Excluding only position 1 was not enough:
 *  any average over a mixed bag of positions jumps around with whatever mix that
 *  minute happened to contain.
 *
 *  So trends use exactly ONE scan per burst - the last one before the rest, the
 *  most equilibrated - which makes every point comparable with every other. That
 *  is one sample every ~150 s, ample for something that moves over hours. The
 *  raw per-scan view still shows everything, because that is what raw means.
 *
 *  v1.6.4 — OpenRouter free-router reasoning/content compatibility
 *  v1.7.0 — page: equaliser bars with peak-hold ticks that fade, average ticks
 *           on the score components, sound-over-time heatmap; per-minute
 *           component averages kept in the history ring (/api/history "c")
 * =============================================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <LittleFS.h>
#include <bsec2.h>
#include <math.h>
#include <time.h>
#include <esp_heap_caps.h>

// ---------------------------------------------------------------- settings --
#define WIFI_SSID       "YOUR_WIFI"
#define WIFI_PASS       "YOUR_PASSWORD"
#define NODE_NAME       "apiary-ears"
#define FW_VERSION      "1.7.0"

#define I2S_SD          4
#define I2S_WS          5
#define I2S_SCK         6
#define SAMPLE_RATE     16000
#define WIN_SAMPLES     4096       // 256 ms per analysis window

#define I2C_SDA         21
#define I2C_SCL         13
#define BME_ADDR_A      0x77
#define BME_ADDR_B      0x76

#define NOSE_STEPS      10
#define BAND_COUNT      12
#define MINUTE_RING     1440       // 24 h of per-minute summaries
#define EVENT_MAX       40
#define CLIP_SECONDS    10
#define CLIP_KEEP       6          // most recordings kept on flash; oldest go first
#define CLIP_FREE_MIN   65536      // never leave the partition with less than this free
#define NOSE_RING       240        // raw fingerprints: the last hour or so, full detail
#define NOSE_DAY        1440       // one averaged fingerprint a minute = a full day (~35 KB)
#define BURST_GAP_MS    30000UL    // a longer pause means a new burst began

#define EVENT_ON        55         // score at or above this opens an event
#define EVENT_OFF       35         // and it closes when the score falls below
#define EVENT_ON_WINS   3
#define EVENT_COOLDOWN_S 300

// Optional AI. Wi-Fi credentials remain compile-time settings above; AI credentials
// are entered later from the web page and kept in ESP32 NVS. Nothing here is a
// shared project key, so the same firmware can be published and flashed by anyone.
#define AI_HTTP_TIMEOUT_MS 60000
#define AI_MAX_RESULT_CHARS 6000
#define AI_OPENROUTER_MODEL "openrouter/free"
#define AI_OPENAI_MODEL     "gpt-5.6-luna"

// The bands, in Hz:
//   60-180   fanning and general colony roar
//   225-315  worker agitation and piping
//   360-500  queen tooting; quacking sits a little lower
//   600-800  broadband, a control for "is something else going on"
static const float BAND_HZ[BAND_COUNT] = {
  60, 100, 140, 180, 225, 270, 315, 360, 420, 500, 600, 800
};
// Bosch's standard scan profile, HP-354.
static const int HEATER_C[NOSE_STEPS] = {320, 100, 100, 100, 200, 200, 200, 320, 320, 320};

const uint8_t bsecConfig[] = {
  #include "config/FieldAir_HandSanitizer/bsec_selectivity.txt"
};

// ----------------------------------------------------------------- globals --
WebServer server(80);
Bsec2     nose;
Preferences aiPrefs;

// AI configuration is deliberately separate from Wi-Fi configuration. The public
// firmware never contains somebody else's key. Keys entered in the page are stored
// in NVS and are never returned by the status API.
SemaphoreHandle_t g_aiMutex = nullptr;
String   g_aiProvider;                 // "openrouter" or "openai"
String   g_aiKey;
String   g_aiModel;
String   g_aiLastResult;
String   g_aiLastError;
String   g_aiNotice;
String   g_aiTask = "idle";
String   g_aiLastAction;              // "test" or "analysis"
volatile bool g_aiBusy = false;
bool     g_aiLastOk = false;
uint32_t g_aiLastEpoch = 0;

float    g_band[BAND_COUNT], g_base[BAND_COUNT], g_dev[BAND_COUNT];
bool     g_baseReady = false;
uint32_t g_windows = 0;

float    g_score = 0, g_scorePeak = 0;
float    g_scComp[4] = {0, 0, 0, 0};   // fanning, agitation, piping, environment
float    g_rms = 0, g_dbfs = -120;
bool     g_clip = false;

// --- the nose -------------------------------------------------------------
float    g_spec[NOSE_STEPS], g_specLast[NOSE_STEPS];
uint8_t  g_nosePos = 0, g_noseGot = 0;
uint32_t g_noseSeq = 0, g_noseLastMs = 0;
float    g_noseMean = NAN;             // mean kOhm of the last settled scan
float    g_noseBase = NAN;             // slow baseline of that mean
float    g_noseDev = 0;                // 0..1, how far below baseline (more gas)
float    g_tempC = NAN, g_rh = NAN, g_hpa = NAN;
float    g_tempPrev = NAN, g_tempRate = 0;
uint32_t g_tempRateMs = 0;
bool     g_noseOk = false;
uint8_t  g_bmeAddr = 0;

struct NoseRow {                         // one completed fingerprint
  uint32_t epoch;
  uint16_t kohm[NOSE_STEPS];             // clipped to 65535, plenty for this sensor
  uint8_t  pos;                          // burst position; 1 = first after a rest
};
NoseRow  g_noseRing[NOSE_RING];
uint16_t g_noseHead = 0, g_noseCount = 0;

// A day at one-minute resolution. Raw scans are far too many to keep for 24 h,
// but a minute's worth averaged paints the map perfectly well - smell moves over
// hours, so a minute is already finer than the signal.
struct NoseDayRow { uint32_t epoch; uint16_t kohm[NOSE_STEPS]; };
NoseDayRow g_noseDay[NOSE_DAY];
uint16_t   g_dayHead = 0, g_dayCount = 0;
// The last scan of a burst is the settled one. We only know a scan was the last
// when the NEXT one arrives with position 1, so keep it until then.
uint16_t   g_lastOfBurst[NOSE_STEPS] = {0};
float      g_lastOfBurstMean = NAN;
uint32_t   g_lastOfBurstEpoch = 0;
bool       g_haveLastOfBurst = false;
uint32_t   g_dayAcc[NOSE_STEPS] = {0};
uint16_t   g_dayAccN = 0;
uint32_t   g_dayStartMs = 0;

struct MinuteRow {
  uint32_t epoch;
  uint8_t  band[BAND_COUNT];
  uint8_t  score;
  uint8_t  comp[4];                    // minute-average of the four score components
  uint16_t noseMean;                   // kOhm, clipped to 65535
  int16_t  t10;                          // tenths of a degree C; whole degrees made a
                                         // gently drifting room look like a square wave
  uint8_t  rh;
};
MinuteRow g_min[MINUTE_RING];
uint16_t  g_minHead = 0, g_minCount = 0;
uint32_t  g_minAccN = 0;
float     g_minAcc[BAND_COUNT], g_minScorePeak = 0;
float     g_minCompAcc[4] = {0, 0, 0, 0};
// Post-rest scans read several times high, so a minute that happens to end on one
// would spike. Average the settled scans instead of sampling whatever was last.
double    g_minNoseAcc = 0;
uint16_t  g_minNoseN = 0;
uint32_t  g_minStartMs = 0;

struct Event {
  uint32_t epoch, startMs;
  uint16_t durS;
  uint8_t  peak, comp[4];
  float    band[BAND_COUNT];           // band deviations at the peak, dB
  float    spec[NOSE_STEPS];           // the fingerprint at the peak, kOhm
  float    noseDev, tempC, rh, hpa, tempRate;
  char     note[48];
};
Event    g_events[EVENT_MAX];
uint8_t  g_evCount = 0;
bool     g_inEvent = false;
uint8_t  g_onStreak = 0;
uint32_t g_lastEventEnd = 0;

int16_t* g_clipBuf = nullptr;
size_t   g_clipCap = 0, g_clipLen = 0;
bool     g_clipRecording = false;
uint32_t g_clipEpoch = 0;
char     g_clipLabel[40] = "";

static int32_t i2sBuf[WIN_SAMPLES];

// ------------------------------------------------------------------- audio --
static void audioBegin() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 512;
  cfg.use_apll = false;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = I2S_SCK;
  pins.ws_io_num = I2S_WS;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = I2S_SD;

  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

// Goertzel: cheaper than an FFT when you want twelve specific frequencies
// rather than the whole spectrum.
static float goertzel(const float* x, int n, float freq) {
  const float k = 2.0f * cosf(2.0f * (float)M_PI * freq / SAMPLE_RATE);
  float s0 = 0, s1 = 0, s2 = 0;
  for (int i = 0; i < n; i++) { s0 = x[i] + k * s1 - s2; s2 = s1; s1 = s0; }
  float p = s1 * s1 + s2 * s2 - k * s1 * s2;
  return p > 0 ? p : 0;
}

static bool readWindow(float* mono, int n) {
  size_t got = 0;
  if (i2s_read(I2S_NUM_0, i2sBuf, n * sizeof(int32_t), &got, pdMS_TO_TICKS(500)) != ESP_OK) return false;
  int have = got / sizeof(int32_t);
  if (have < n / 2) return false;
  double sum = 0, sq = 0;
  int32_t pk = 0;
  for (int i = 0; i < have; i++) {
    int32_t v = i2sBuf[i] >> 8;             // INMP441 gives 24 bits left-aligned
    mono[i] = (float)v;
    sum += v; sq += (double)v * v;
    if (abs(v) > pk) pk = abs(v);
  }
  float mean = sum / have;
  for (int i = 0; i < have; i++) mono[i] -= mean;
  for (int i = have; i < n; i++) mono[i] = 0;
  g_rms = sqrtf((float)(sq / have));
  g_dbfs = 20.0f * log10f(fmaxf(g_rms, 1.0f) / 8388608.0f);
  g_clip = pk > 8000000;
  return true;
}

// ------------------------------------------------------------------ scoring --
static void scoreWindow() {
  static float mono[WIN_SAMPLES];
  if (!readWindow(mono, WIN_SAMPLES)) return;

  for (int b = 0; b < BAND_COUNT; b++)
    g_band[b] = 10.0f * log10f(goertzel(mono, WIN_SAMPLES, BAND_HZ[b]) + 1.0f);

  if (!g_baseReady) {
    for (int b = 0; b < BAND_COUNT; b++) g_base[b] = g_band[b];
    if (++g_windows > 40) g_baseReady = true;        // ~10 s to settle
    return;
  }
  const float a = 0.0015f;                            // baseline tracks ~30 min
  for (int b = 0; b < BAND_COUNT; b++) {
    g_dev[b] = g_band[b] - g_base[b];
    g_base[b] = (1 - a) * g_base[b] + a * g_band[b];
  }

  float fanning = 0, agitation = 0, piping = 0;
  for (int b = 0; b < 4; b++)  fanning   += g_dev[b];
  for (int b = 4; b < 7; b++)  agitation += g_dev[b];
  for (int b = 7; b < 10; b++) piping    += g_dev[b];
  fanning /= 4; agitation /= 3; piping /= 3;

  // Environment: smell moving and temperature climbing. Both are slow signals,
  // which is why they carry the least weight - they set context rather than
  // trigger. g_noseDev is 0..1, already relative to the hive's own baseline.
  float env = fminf(fmaxf(g_noseDev, 0.0f), 1.0f) * 60.0f;
  if (!isnan(g_tempRate)) env += fminf(fmaxf(g_tempRate / 1.5f, 0.0f), 1.0f) * 40.0f;

  auto clamp01 = [](float v, float full) { return fminf(fmaxf(v / full, 0.0f), 1.0f); };
  g_scComp[0] = clamp01(fanning,   6.0f) * 100.0f;
  g_scComp[1] = clamp01(agitation, 6.0f) * 100.0f;
  g_scComp[2] = clamp01(piping,    5.0f) * 100.0f;
  g_scComp[3] = fminf(env, 100.0f);

  g_score = 0.25f * g_scComp[0] + 0.35f * g_scComp[1] +
            0.30f * g_scComp[2] + 0.10f * g_scComp[3];
  if (g_score > g_scorePeak) g_scorePeak = g_score;

  for (int b = 0; b < BAND_COUNT; b++) g_minAcc[b] += g_band[b];
  for (int i = 0; i < 4; i++) g_minCompAcc[i] += g_scComp[i];
  if (g_score > g_minScorePeak) g_minScorePeak = g_score;
  g_minAccN++;
}

static void minuteRoll() {
  if (!g_minAccN || millis() - g_minStartMs < 60000UL) return;
  MinuteRow& r = g_min[g_minHead];
  time_t tt = time(nullptr);
  r.epoch = (tt > 1600000000) ? (uint32_t)tt : (millis() / 1000UL);
  for (int b = 0; b < BAND_COUNT; b++)
    r.band[b] = (uint8_t)fminf(fmaxf((g_minAcc[b] / g_minAccN) * 1.8f, 0.0f), 255.0f);
  r.score = (uint8_t)fminf(g_minScorePeak, 255.0f);
  for (int i = 0; i < 4; i++) r.comp[i] = (uint8_t)fminf(fmaxf(g_minCompAcc[i] / g_minAccN, 0.0f), 255.0f);
  r.noseMean = g_minNoseN ? (uint16_t)fminf((float)(g_minNoseAcc / g_minNoseN), 65535.0f)
                          : (isnan(g_noseMean) ? 0 : (uint16_t)fminf(g_noseMean, 65535.0f));
  r.t10 = isnan(g_tempC) ? -990 : (int16_t)roundf(g_tempC * 10.0f);
  r.rh = isnan(g_rh) ? 0 : (uint8_t)roundf(g_rh);
  g_minHead = (g_minHead + 1) % MINUTE_RING;
  if (g_minCount < MINUTE_RING) g_minCount++;
  for (int b = 0; b < BAND_COUNT; b++) g_minAcc[b] = 0;
  for (int i = 0; i < 4; i++) g_minCompAcc[i] = 0;
  g_minAccN = 0; g_minScorePeak = 0;
  g_minNoseAcc = 0; g_minNoseN = 0;
  g_minStartMs = millis();
}

// ------------------------------------------------------------------- nose ---
void onNoseData(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec) {
  // Capture the raw frame FIRST: in scan mode BSEC emits its virtual outputs
  // only at the end of a scan, so steps 0..8 arrive with nOutputs == 0 and an
  // early return here would lose nine tenths of every fingerprint.
  int idx = data.gas_index;
  if (idx >= 0 && idx < NOSE_STEPS) {
    g_spec[idx] = data.gas_resistance / 1000.0f;
    g_noseGot++;
    if (idx == NOSE_STEPS - 1) {
      uint32_t now = millis();
      uint32_t gap = g_noseLastMs ? (now - g_noseLastMs) : 0;
      if (!g_noseLastMs || gap > BURST_GAP_MS) g_nosePos = 1;
      else if (g_nosePos < 255) g_nosePos++;
      g_noseLastMs = now;
      memcpy(g_specLast, g_spec, sizeof(g_spec));
      g_noseSeq++;

      // Computed once, here, because everything below uses it.
      float sum = 0; int n = 0;
      for (int k = 0; k < NOSE_STEPS; k++)
        if (!isnan(g_spec[k]) && g_spec[k] > 0) { sum += g_spec[k]; n++; }
      float mean = n ? sum / n : NAN;

      NoseRow& nr = g_noseRing[g_noseHead];
      time_t tt = time(nullptr);
      nr.epoch = (tt > 1600000000) ? (uint32_t)tt : (millis() / 1000UL);
      for (int k = 0; k < NOSE_STEPS; k++)
        nr.kohm[k] = isnan(g_spec[k]) ? 0 : (uint16_t)fminf(fmaxf(g_spec[k], 0.0f), 65535.0f);
      nr.pos = g_nosePos;
      g_noseHead = (g_noseHead + 1) % NOSE_RING;
      if (g_noseCount < NOSE_RING) g_noseCount++;

      // A burst just ended: the scan we were holding was its last, and therefore
      // its most settled. That is the one that goes into the trends.
      if (g_nosePos == 1 && g_haveLastOfBurst) {
        for (int k = 0; k < NOSE_STEPS; k++) g_dayAcc[k] += g_lastOfBurst[k];
        g_dayAccN++;
        if (!isnan(g_lastOfBurstMean)) { g_minNoseAcc += g_lastOfBurstMean; g_minNoseN++; }
        g_haveLastOfBurst = false;
      }
      // Hold this scan in case it turns out to be the last of the current burst.
      if (g_nosePos >= 2) {
        for (int k = 0; k < NOSE_STEPS; k++) g_lastOfBurst[k] = nr.kohm[k];
        g_lastOfBurstMean = mean;
        g_lastOfBurstEpoch = nr.epoch;
        g_haveLastOfBurst = true;
      }
      if (g_dayAccN && millis() - g_dayStartMs >= 60000UL) {
        NoseDayRow& dr = g_noseDay[g_dayHead];
        dr.epoch = nr.epoch;
        for (int k = 0; k < NOSE_STEPS; k++) dr.kohm[k] = (uint16_t)(g_dayAcc[k] / g_dayAccN);
        g_dayHead = (g_dayHead + 1) % NOSE_DAY;
        if (g_dayCount < NOSE_DAY) g_dayCount++;
        for (int k = 0; k < NOSE_STEPS; k++) g_dayAcc[k] = 0;
        g_dayAccN = 0;
        g_dayStartMs = millis();
      }

      // Position 1 is the first scan after the sensor rested: the surface
      // recovered, so it reads high for reasons that have nothing to do with
      // the air. Keep it in the record, keep it out of the baseline.
      g_noseMean = mean;                       // always show the live reading
      // ...but only move the baseline on a like-for-like sample: the last scan
      // of a burst. Mixing positions into the baseline is what made the trend
      // sawtooth even after position 1 was excluded.
      if (g_nosePos == 1 && !isnan(g_lastOfBurstMean)) {
        float m = g_lastOfBurstMean;
        if (isnan(g_noseBase)) g_noseBase = m;
        else g_noseBase = 0.99f * g_noseBase + 0.01f * m;
        g_noseDev = (g_noseBase > 1.0f) ? (g_noseBase - m) / g_noseBase : 0.0f;
      }
      if (g_noseSeq <= 3 || (g_noseSeq % 20) == 0)
        Serial.printf("[nose] scan %lu (%u/10 steps, burst pos %u) mean %.0f kOhm, dev %.2f\n",
                      (unsigned long)g_noseSeq, g_noseGot, g_nosePos,
                      isnan(mean) ? 0.0f : mean, g_noseDev);
      g_noseGot = 0;
    }
  }

  if (!outputs.nOutputs) return;
  for (uint8_t i = 0; i < outputs.nOutputs; i++) {
    const bsecData& o = outputs.output[i];
    switch (o.sensor_id) {
      case BSEC_OUTPUT_RAW_TEMPERATURE: g_tempC = o.signal; break;
      case BSEC_OUTPUT_RAW_HUMIDITY:    g_rh    = o.signal; break;
      case BSEC_OUTPUT_RAW_PRESSURE:    g_hpa   = o.signal; break;
      default: break;
    }
  }
  if (!isnan(g_tempC)) {
    if (isnan(g_tempPrev)) { g_tempPrev = g_tempC; g_tempRateMs = millis(); }
    else if (millis() - g_tempRateMs > 300000UL) {          // every 5 minutes
      float hrs = (millis() - g_tempRateMs) / 3600000.0f;
      g_tempRate = (g_tempC - g_tempPrev) / hrs;
      g_tempPrev = g_tempC; g_tempRateMs = millis();
    }
  }
}

static bool i2cPing(uint8_t a) { Wire.beginTransmission(a); return Wire.endTransmission() == 0; }

static void noseBegin() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(100);
  g_bmeAddr = i2cPing(BME_ADDR_A) ? BME_ADDR_A : (i2cPing(BME_ADDR_B) ? BME_ADDR_B : 0);
  if (!g_bmeAddr) {
    Serial.println("[bme688] nothing answers at 0x77 or 0x76.");
    Serial.println("         Check VCC and GND at the sensor's own pins, then SDA (the pin");
    Serial.println("         marked MOSI) and SCL, then the address switch.");
    Serial.print("[i2c] devices on the bus:");
    uint8_t found = 0;
    for (uint8_t a = 1; a < 0x78; a++) if (i2cPing(a)) { Serial.printf(" 0x%02X", a); found++; }
    Serial.println(found ? "" : " none");
    return;
  }
  if (!nose.begin(g_bmeAddr, Wire)) { Serial.println("[bme688] BSEC begin failed"); return; }
  if (!nose.setConfig(bsecConfig)) Serial.println("[bme688] setConfig warning");
  bsecSensor list[] = {
    BSEC_OUTPUT_RAW_GAS, BSEC_OUTPUT_RAW_GAS_INDEX, BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_HUMIDITY, BSEC_OUTPUT_RAW_PRESSURE
  };
  if (!nose.updateSubscription(list, 5, BSEC_SAMPLE_RATE_SCAN)) {
    Serial.println("[bme688] subscription failed"); return;
  }
  nose.attachCallback(onNoseData);
  for (int k = 0; k < NOSE_STEPS; k++) { g_spec[k] = NAN; g_specLast[k] = NAN; }
  g_noseOk = true;
  Serial.printf("[bme688] found at 0x%02X, scan mode (10 heater steps, ~11 s per scan)\n", g_bmeAddr);
  Serial.println("[bme688] a new sensor needs 24-48 h of running before its gas readings settle.");
}

// ------------------------------------------------------------------- clips --
static String specJson(const float* s);          // defined with the web handlers
static void writeWavHeader(uint8_t*, uint32_t);  // defined below, used by clipSave()
static void clipSave();                          // called from clipService()

// ---- recordings on flash ---------------------------------------------------
// Flash is small and a clip is ~320 KB, so the board keeps its own house: make
// room before recording rather than failing afterwards.
static size_t fsTotal() { return LittleFS.totalBytes(); }
static size_t fsUsed()  { return LittleFS.usedBytes(); }
static size_t fsFree()  { size_t t = fsTotal(), u = fsUsed(); return t > u ? t - u : 0; }

static int clipCount() {                     // counts recordings, not files
  int n = 0;
  File d = LittleFS.open("/clips");
  if (!d || !d.isDirectory()) return 0;
  for (File f = d.openNextFile(); f; f = d.openNextFile()) {
    if (f.isDirectory()) continue;
    String nm = f.name();
    if (nm.endsWith(".wav")) n++;
  }
  return n;
}

// The oldest recording is the one whose name sorts first: names begin with the
// epoch. A recording is a SET - the audio and the snapshot of every sensor at
// that moment - so they are deleted together or the survivor is meaningless.
static bool clipDeleteOldest() {
  String oldest;
  File d = LittleFS.open("/clips");
  if (!d || !d.isDirectory()) return false;
  for (File f = d.openNextFile(); f; f = d.openNextFile()) {
    if (f.isDirectory()) continue;
    String n = f.name();
    if (n.startsWith("/")) n = n.substring(1);
    if (!n.endsWith(".wav")) continue;              // the audio leads; its json follows
    if (!oldest.length() || n < oldest) oldest = n;
  }
  if (!oldest.length()) return false;
  String stem = oldest.substring(0, oldest.length() - 4);
  Serial.printf("[clip] making room: deleting %s (audio and snapshot)\n", stem.c_str());
  LittleFS.remove("/clips/" + stem + ".json");
  return LittleFS.remove("/clips/" + oldest);
}

static void clipMakeRoom(size_t needed) {
  int guard = 0;
  while (guard++ < 32) {
    bool tooMany = clipCount() >= CLIP_KEEP;
    bool tooFull = fsFree() < needed + CLIP_FREE_MIN;
    if (!tooMany && !tooFull) return;
    if (!clipDeleteOldest()) return;            // nothing left to delete
  }
}

static void clipStart(const char* label) {
  if (!g_clipBuf || g_clipRecording) return;
  g_clipLen = 0;
  g_clipRecording = true;
  time_t tt = time(nullptr);
  g_clipEpoch = (tt > 1600000000) ? (uint32_t)tt : (millis() / 1000UL);
  strncpy(g_clipLabel, label ? label : "", sizeof(g_clipLabel) - 1);
  g_clipLabel[sizeof(g_clipLabel) - 1] = 0;
  Serial.printf("[clip] recording %d s (%s)\n", CLIP_SECONDS, g_clipLabel);
}

static void clipService() {
  if (!g_clipRecording || !g_clipBuf) return;
  size_t got = 0;
  if (i2s_read(I2S_NUM_0, i2sBuf, 1024 * sizeof(int32_t), &got, pdMS_TO_TICKS(50)) != ESP_OK) return;
  int have = got / sizeof(int32_t);
  for (int i = 0; i < have && g_clipLen < g_clipCap; i++)
    g_clipBuf[g_clipLen++] = (int16_t)(i2sBuf[i] >> 16);
  if (g_clipLen >= g_clipCap) {
    g_clipRecording = false;
    Serial.printf("[clip] done, %u samples\n", (unsigned)g_clipLen);
    clipSave();
  }
}

// Name: <epoch>-<label>.wav, so a plain sort is a sort by age.
static void clipSave() {
  if (!g_clipLen || !g_clipBuf) return;
  uint32_t bytes = g_clipLen * 2;
  clipMakeRoom(bytes + 44);
  if (fsFree() < bytes + 44) {
    Serial.println("[clip] not enough flash even after housekeeping - clip kept in RAM only");
    return;
  }
  char safe[32];
  size_t j = 0;
  for (size_t i = 0; g_clipLabel[i] && j < sizeof(safe) - 1; i++) {
    char c = g_clipLabel[i];
    safe[j++] = (isalnum((int)c)) ? c : '-';
  }
  safe[j] = 0;
  char path[80];
  snprintf(path, sizeof(path), "/clips/%lu-%s.wav", (unsigned long)g_clipEpoch, safe);
  File f = LittleFS.open(path, "w");
  if (!f) { Serial.printf("[clip] could not open %s for writing\n", path); return; }
  uint8_t hdr[44];
  writeWavHeader(hdr, bytes);
  f.write(hdr, 44);
  const size_t CH = 4096;
  for (size_t off = 0; off < g_clipLen; off += CH) {
    size_t n = min(CH, g_clipLen - off);
    f.write((const uint8_t*)(g_clipBuf + off), n * 2);
  }
  f.close();

  // Everything the board knew at that moment, beside the audio. Small - about a
  // kilobyte - so it costs nothing next to a 320 KB clip, and without it the
  // recording is just a noise with a date on it.
  char jpath[80];
  snprintf(jpath, sizeof(jpath), "/clips/%lu-%s.json", (unsigned long)g_clipEpoch, safe);
  File jf = LittleFS.open(jpath, "w");
  if (jf) {
    String o = "{\n  \"schema_version\": 1,\n  \"kind\": \"event_snapshot\",\n";
    o += "  \"node\": \"" NODE_NAME "\",\n  \"firmware\": \"apiary_ears " FW_VERSION "\",\n";
    o += "  \"epoch\": " + String(g_clipEpoch) + ",\n";
    o += "  \"label\": \"" + String(g_clipLabel) + "\",\n";
    o += "  \"audio_file\": \"" + String(path).substring(7) + "\",\n";
    o += "  \"audio\": {\"sample_rate_hz\": " + String(SAMPLE_RATE) +
         ", \"channels\": 1, \"bit_depth\": 16, \"duration_s\": " +
         String(g_clipLen / (float)SAMPLE_RATE, 1) + "},\n";
    o += "  \"score\": " + String(g_score, 1) + ",\n";
    o += "  \"components\": {\"fanning\": " + String(g_scComp[0], 0) +
         ", \"agitation\": " + String(g_scComp[1], 0) +
         ", \"piping\": " + String(g_scComp[2], 0) +
         ", \"environment\": " + String(g_scComp[3], 0) + "},\n";
    o += "  \"bands_hz\": [";
    for (int b = 0; b < BAND_COUNT; b++) { if (b) o += ','; o += String(BAND_HZ[b], 0); }
    o += "],\n  \"band_db\": [";
    for (int b = 0; b < BAND_COUNT; b++) { if (b) o += ','; o += String(g_band[b], 1); }
    o += "],\n  \"band_dev_db\": [";
    for (int b = 0; b < BAND_COUNT; b++) { if (b) o += ','; o += String(g_dev[b], 1); }
    o += "],\n  \"dbfs\": " + String(g_dbfs, 1) + ",\n";
    o += "  \"nose_steps_c\": [";
    for (int k = 0; k < NOSE_STEPS; k++) { if (k) o += ','; o += String(HEATER_C[k]); }
    o += "],\n  \"nose_spec_kohm\": " + specJson(g_specLast) + ",\n";
    o += "  \"nose_burst_pos\": " + String(g_nosePos) + ",\n";
    o += "  \"nose_mean_kohm\": " + (isnan(g_noseMean) ? String("null") : String(g_noseMean, 1)) + ",\n";
    o += "  \"nose_baseline_kohm\": " + (isnan(g_noseBase) ? String("null") : String(g_noseBase, 1)) + ",\n";
    o += "  \"nose_dev\": " + String(g_noseDev, 3) + ",\n";
    o += "  \"temp_c\": " + (isnan(g_tempC) ? String("null") : String(g_tempC, 1)) + ",\n";
    o += "  \"rh_pct\": " + (isnan(g_rh) ? String("null") : String(g_rh, 1)) + ",\n";
    o += "  \"hpa\": " + (isnan(g_hpa) ? String("null") : String(g_hpa, 1)) + ",\n";
    o += "  \"temp_rate_h\": " + String(g_tempRate, 2) + ",\n";
    o += "  \"note\": \"\"\n}\n";
    jf.print(o);
    jf.close();
  }

  Serial.printf("[clip] saved %s + snapshot (%u KB) - %d recording(s) on flash, %u KB free\n",
                path, (unsigned)((bytes + 44) / 1024), clipCount(), (unsigned)(fsFree() / 1024));
}

static void writeWavHeader(uint8_t* h, uint32_t dataBytes) {
  const uint32_t sr = SAMPLE_RATE, br = sr * 2;
  memcpy(h, "RIFF", 4);
  uint32_t riff = dataBytes + 36; memcpy(h + 4, &riff, 4);
  memcpy(h + 8, "WAVEfmt ", 8);
  uint32_t fmtlen = 16; memcpy(h + 16, &fmtlen, 4);
  uint16_t fmt = 1, ch = 1, bits = 16, align = 2;
  memcpy(h + 20, &fmt, 2); memcpy(h + 22, &ch, 2);
  memcpy(h + 24, &sr, 4);  memcpy(h + 28, &br, 4);
  memcpy(h + 32, &align, 2); memcpy(h + 34, &bits, 2);
  memcpy(h + 36, "data", 4); memcpy(h + 40, &dataBytes, 4);
}

// ------------------------------------------------------------------ events --
static void eventOpen() {
  if (g_evCount >= EVENT_MAX) {
    for (uint8_t i = 1; i < g_evCount; i++) g_events[i - 1] = g_events[i];
    g_evCount--;
  }
  Event& e = g_events[g_evCount];
  time_t tt = time(nullptr);
  e.epoch = (tt > 1600000000) ? (uint32_t)tt : (millis() / 1000UL);
  e.startMs = millis();
  e.durS = 0;
  e.peak = (uint8_t)g_score;
  for (int i = 0; i < 4; i++) e.comp[i] = (uint8_t)g_scComp[i];
  for (int b = 0; b < BAND_COUNT; b++) e.band[b] = g_dev[b];
  for (int k = 0; k < NOSE_STEPS; k++) e.spec[k] = g_specLast[k];
  e.noseDev = g_noseDev; e.tempC = g_tempC; e.rh = g_rh; e.hpa = g_hpa;
  e.tempRate = g_tempRate;
  e.note[0] = 0;
  g_evCount++;
  g_inEvent = true;
  Serial.printf("[event] opened, score %.0f (fan %.0f agit %.0f pipe %.0f env %.0f)\n",
                g_score, g_scComp[0], g_scComp[1], g_scComp[2], g_scComp[3]);
  clipStart("auto event");
}

static void eventService() {
  if (!g_baseReady) return;
  if (!g_inEvent) {
    bool cool = (g_lastEventEnd == 0) || ((millis() - g_lastEventEnd) / 1000UL > EVENT_COOLDOWN_S);
    if (g_score >= EVENT_ON && cool) { if (++g_onStreak >= EVENT_ON_WINS) { g_onStreak = 0; eventOpen(); } }
    else g_onStreak = 0;
  } else {
    Event& e = g_events[g_evCount - 1];
    e.durS = (millis() - e.startMs) / 1000UL;
    if (g_score > e.peak) {
      e.peak = (uint8_t)g_score;
      for (int i = 0; i < 4; i++) e.comp[i] = (uint8_t)g_scComp[i];
      for (int b = 0; b < BAND_COUNT; b++) e.band[b] = g_dev[b];
      for (int k = 0; k < NOSE_STEPS; k++) e.spec[k] = g_specLast[k];
      e.noseDev = g_noseDev;
    }
    if (g_score < EVENT_OFF) {
      g_inEvent = false;
      g_lastEventEnd = millis();
      Serial.printf("[event] closed after %u s, peak %u\n", e.durS, e.peak);
    }
  }
}

// --------------------------------------------------------------------- AI ---
// The ESP32 never makes autonomous hive-management decisions. AI is an optional,
// on-demand second opinion over the sensor features this firmware already computes.
// The prompt explicitly states that the score and fingerprint are experimental.

static String jsonEscape(const String& in) {
  String out;
  out.reserve(in.length() + 32);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"':  out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if ((uint8_t)c >= 0x20) out += c;
        else out += ' ';
    }
  }
  return out;
}

static int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return 10 + c - 'a';
  if (c >= 'A' && c <= 'F') return 10 + c - 'A';
  return -1;
}

static void appendUtf8(String& out, uint16_t cp) {
  if (cp < 0x80) out += (char)cp;
  else if (cp < 0x800) {
    out += (char)(0xC0 | (cp >> 6));
    out += (char)(0x80 | (cp & 0x3F));
  } else {
    out += (char)(0xE0 | (cp >> 12));
    out += (char)(0x80 | ((cp >> 6) & 0x3F));
    out += (char)(0x80 | (cp & 0x3F));
  }
}

// Extract a JSON string value without adding ArduinoJson as another required library.
// This is enough for Chat Completions' choices[0].message.content and error.message.
static bool jsonStringAfter(const String& body, const char* key, String& out, int startAt=0) {
  int p = body.indexOf(key, startAt);
  if (p < 0) return false;
  p = body.indexOf(':', p + strlen(key));
  if (p < 0) return false;
  p++;
  while (p < (int)body.length() && (body[p] == ' ' || body[p] == '\n' || body[p] == '\r' || body[p] == '\t')) p++;
  if (p >= (int)body.length() || body[p] != '"') return false;
  p++;
  out = "";
  out.reserve(min((int)body.length() - p, AI_MAX_RESULT_CHARS));
  bool esc = false;
  for (; p < (int)body.length() && out.length() < AI_MAX_RESULT_CHARS; p++) {
    char c = body[p];
    if (!esc) {
      if (c == '\\') { esc = true; continue; }
      if (c == '"') return true;
      out += c;
      continue;
    }
    esc = false;
    switch (c) {
      case 'n': out += '\n'; break;
      case 'r': out += '\r'; break;
      case 't': out += '\t'; break;
      case 'b': out += '\b'; break;
      case 'f': out += '\f'; break;
      case '\\': out += '\\'; break;
      case '"': out += '"'; break;
      case '/': out += '/'; break;
      case 'u': {
        if (p + 4 < (int)body.length()) {
          int a=hexNibble(body[p+1]), b=hexNibble(body[p+2]), d=hexNibble(body[p+3]), e=hexNibble(body[p+4]);
          if (a>=0 && b>=0 && d>=0 && e>=0) {
            appendUtf8(out, (uint16_t)((a<<12)|(b<<8)|(d<<4)|e));
            p += 4;
          } else out += '?';
        } else out += '?';
        break;
      }
      default: out += c; break;
    }
  }
  return out.length() > 0;
}

// Extract assistant final text from the Chat Completions response. OpenRouter is
// normally message.content as a string, but this also tolerates a content-parts
// array/object with a nested "text" string. Reasoning fields are deliberately
// ignored and are never displayed as the hive analysis.
static bool extractAssistantText(const String& body, String& out) {
  int choices = body.indexOf("\"choices\"");
  int start = choices >= 0 ? choices : 0;
  int message = body.indexOf("\"message\"", start);
  if (message >= 0) start = message;
  int content = body.indexOf("\"content\"", start);
  if (content < 0) return false;

  if (jsonStringAfter(body, "\"content\"", out, content) && out.length()) return true;

  // Some OpenAI-compatible providers may return content parts instead of one string.
  int finish = body.indexOf("\"finish_reason\"", content);
  int textKey = body.indexOf("\"text\"", content);
  if (textKey >= 0 && (finish < 0 || textKey < finish)) {
    if (jsonStringAfter(body, "\"text\"", out, textKey) && out.length()) return true;
  }
  out = "";
  return false;
}

static bool aiConfigured() {
  return (g_aiProvider == "openrouter" || g_aiProvider == "openai") && g_aiKey.length() >= 8 && g_aiModel.length();
}

static String aiProviderLabel(const String& p) {
  if (p == "openrouter") return "OpenRouter";
  if (p == "openai") return "OpenAI API";
  return "Not configured";
}

static String fMaybe(float v, unsigned int digits=1) {
  return isnan(v) ? String("unknown") : String(v, digits);
}

static String buildHivePrompt() {
  String p;
  p.reserve(5200);
  p += "Analyze this honey-bee hive sensor snapshot. This is Apiary Ears firmware, which combines microphone frequency-band features with a BME688 gas-resistance fingerprint. ";
  p += "IMPORTANT: the firmware's acoustic score and band weights are experimental hypotheses, not validated swarm or disease predictors. Do not diagnose disease, queen status, swarming, poisoning, or colony failure as a fact. Use cautious language and suggest physical inspection when warranted.\n\n";
  p += "CURRENT\n";
  p += "score=" + String(g_score,1) + "/100; peak_since_boot=" + String(g_scorePeak,1) + "; baseline_ready=" + String(g_baseReady ? "yes" : "no") + "; in_event=" + String(g_inEvent ? "yes" : "no") + "\n";
  p += "components: fanning=" + String(g_scComp[0],0) + ", agitation=" + String(g_scComp[1],0) + ", piping=" + String(g_scComp[2],0) + ", environment=" + String(g_scComp[3],0) + "\n";
  p += "microphone_level_dbfs=" + String(g_dbfs,1) + "; clipping=" + String(g_clip ? "yes" : "no") + "\n";
  p += "temperature_C=" + fMaybe(g_tempC) + "; humidity_pct=" + fMaybe(g_rh,0) + "; pressure_hPa=" + fMaybe(g_hpa,1) + "; temp_rate_C_per_h=" + String(g_tempRate,2) + "\n";
  p += "BME688: available=" + String(g_noseOk ? "yes" : "no") + "; burst_position=" + String(g_nosePos) + "; mean_kOhm=" + fMaybe(g_noseMean,1) + "; baseline_kOhm=" + fMaybe(g_noseBase,1) + "; relative_gas_change=" + String(g_noseDev*100.0f,1) + "% more-gas direction\n";
  p += "12 band deviations from this hive's rolling baseline (dB): ";
  for (int b=0; b<BAND_COUNT; b++) {
    if (b) p += ", ";
    p += String(BAND_HZ[b],0) + "Hz=" + String(g_dev[b],1);
  }
  p += "\nlast 10-step gas fingerprint kOhm: ";
  for (int k=0; k<NOSE_STEPS; k++) {
    if (k) p += ", ";
    p += String(HEATER_C[k]) + "C=" + fMaybe(g_specLast[k],1);
  }
  p += "\n\n";

  uint16_t take = min<uint16_t>(60, g_minCount);
  if (take) {
    float scoreSum=0, scoreMax=0, tMin=999, tMax=-999, rhMin=999, rhMax=-999;
    float noseMin=1e9, noseMax=0; uint16_t noseN=0;
    for (uint16_t i=0; i<take; i++) {
      const MinuteRow& r = g_min[(g_minHead + MINUTE_RING - take + i) % MINUTE_RING];
      scoreSum += r.score; if (r.score > scoreMax) scoreMax = r.score;
      float t = r.t10 / 10.0f; if (t < tMin) tMin=t; if (t > tMax) tMax=t;
      if (r.rh < rhMin) rhMin=r.rh; if (r.rh > rhMax) rhMax=r.rh;
      if (r.noseMean) { if (r.noseMean<noseMin) noseMin=r.noseMean; if (r.noseMean>noseMax) noseMax=r.noseMean; noseN++; }
    }
    p += "RECENT HISTORY (last " + String(take) + " minute summaries)\n";
    p += "score_avg=" + String(scoreSum/take,1) + "; score_peak=" + String(scoreMax,0);
    p += "; temp_range_C=" + String(tMin,1) + ".." + String(tMax,1);
    p += "; humidity_range_pct=" + String(rhMin,0) + ".." + String(rhMax,0);
    if (noseN) p += "; settled_gas_mean_range_kOhm=" + String(noseMin,0) + ".." + String(noseMax,0);
    p += "\n\n";
  }

  uint8_t evTake = min<uint8_t>(5, g_evCount);
  p += "RECENT BOARD-DETECTED EVENTS: " + String(g_evCount) + " retained";
  if (!evTake) p += "; none yet.\n";
  else {
    p += "; newest " + String(evTake) + ":\n";
    for (uint8_t n=0; n<evTake; n++) {
      const Event& e = g_events[g_evCount - evTake + n];
      p += "- peak=" + String(e.peak) + ", duration_s=" + String(e.durS) + ", components=" + String(e.comp[0]) + "/" + String(e.comp[1]) + "/" + String(e.comp[2]) + "/" + String(e.comp[3]);
      if (e.note[0]) p += ", keeper_note=\"" + String(e.note) + "\"";
      p += "\n";
    }
  }

  p += "\nReturn a concise beekeeper-facing analysis with exactly these headings:\n";
  p += "STATUS — NORMAL, WATCH, or CHECK SOON, with one sentence why.\n";
  p += "WHAT STANDS OUT — 2-4 bullets grounded only in the supplied measurements.\n";
  p += "POSSIBLE EXPLANATIONS — plausible alternatives, clearly labeled possibilities rather than diagnoses.\n";
  p += "WHAT TO CHECK — practical non-destructive observations/inspection steps, highest priority first.\n";
  p += "CONFIDENCE & LIMITS — low/medium/high confidence and what data is missing.\n";
  p += "Do not recommend pesticide, medication, treatment, queen replacement, feeding, or opening the hive solely because an AI says so. Keep the whole response under 450 words.";
  return p;
}

static String aiMemDiag() {
  size_t freeHeap = ESP.getFreeHeap();
  size_t largestInternal = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  size_t freeInternal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  size_t freePsram = ESP.getFreePsram();
  return "heap " + String((unsigned long)freeHeap) +
         " B, internal " + String((unsigned long)freeInternal) +
         " B, largest internal block " + String((unsigned long)largestInternal) +
         " B, PSRAM " + String((unsigned long)freePsram) + " B";
}

static bool aiResolveHost(const String& provider, String& host, String& error) {
  host = provider == "openrouter" ? "openrouter.ai" : "api.openai.com";
  IPAddress ip;
  if (!WiFi.hostByName(host.c_str(), ip)) {
    error = "DNS lookup failed for " + host + ". RSSI " + String(WiFi.RSSI()) +
            " dBm; " + aiMemDiag() + ".";
    return false;
  }
  Serial.printf("[ai] DNS %s -> %s; %s\n", host.c_str(), ip.toString().c_str(), aiMemDiag().c_str());
  return true;
}

static bool aiHttpCall(const String& provider, const String& key, const String& model,
                       const String& prompt, String& answer, String& error) {
  if (WiFi.status() != WL_CONNECTED) { error = "Wi-Fi is not connected."; return false; }
  String url, host;
  if (provider == "openrouter") url = "https://openrouter.ai/api/v1/chat/completions";
  else if (provider == "openai") url = "https://api.openai.com/v1/chat/completions";
  else { error = "Unknown AI provider."; return false; }
  if (!aiResolveHost(provider, host, error)) return false;

  const String system = "You are a careful assistant interpreting experimental honey-bee hive telemetry. Distinguish measured facts from hypotheses. Never present sensor patterns as a veterinary or apiary diagnosis. Return a final beekeeper-facing answer, not hidden reasoning.";
  const uint8_t generationAttempts = provider == "openrouter" ? 2 : 1;
  String lastRoutedModel, lastFinish;

  for (uint8_t generationAttempt=1; generationAttempt<=generationAttempts; generationAttempt++) {
    String body;
    body.reserve(prompt.length() + 1100);
    body = "{\"model\":\"" + jsonEscape(model) + "\",\"stream\":false,";
    if (provider == "openrouter") {
      // openrouter/free may select a reasoning model. Keep reasoning minimal and
      // excluded so the output budget is reserved for the final hive report.
      body += "\"max_tokens\":1800,\"reasoning\":{\"effort\":\"minimal\",\"exclude\":true},";
    } else {
      body += "\"max_completion_tokens\":1200,";
    }
    body += "\"messages\":[";
    body += "{\"role\":\"system\",\"content\":\"" + jsonEscape(system) + "\"},";
    body += "{\"role\":\"user\",\"content\":\"" + jsonEscape(prompt) + "\"}]}";

    int code = -1;
    String resp;
    String transportError;
    String tlsDetail;
    for (uint8_t attempt=1; attempt<=2; attempt++) {
      WiFiClientSecure requestTls;
      requestTls.setInsecure();
      requestTls.setHandshakeTimeout(30);
      requestTls.setTimeout(AI_HTTP_TIMEOUT_MS);

      HTTPClient http;
      if (!http.begin(requestTls, url)) {
        transportError = "Could not start HTTPS connection.";
        code = -1;
      } else {
        http.setConnectTimeout(30000);
        http.setTimeout(AI_HTTP_TIMEOUT_MS);
        http.setReuse(false);
        http.addHeader("Content-Type", "application/json");
        http.addHeader("Authorization", "Bearer " + key);
        if (provider == "openrouter") {
          http.addHeader("X-Title", "Apiary Ears");
          http.addHeader("HTTP-Referer", "https://github.com/");
        }

        Serial.printf("[ai] POST %s generation %u/%u transport %u/2 payload=%u bytes; %s; RSSI=%d dBm\n",
                      host.c_str(), generationAttempt, generationAttempts, attempt,
                      (unsigned)body.length(), aiMemDiag().c_str(), WiFi.RSSI());
        code = http.POST(body);
        if (code > 0) {
          resp = http.getString();
        } else {
          transportError = HTTPClient::errorToString(code);
          char tlsBuf[160] = {0};
          int tlsCode = requestTls.lastError(tlsBuf, sizeof(tlsBuf));
          if (tlsCode != 0) tlsDetail = "TLS " + String(tlsCode) + " (" + String(tlsBuf) + ")";
        }
        http.end();
      }

      if (code > 0) break;
      Serial.printf("[ai] transport attempt %u failed: HTTP %d (%s) %s; %s; RSSI=%d dBm\n",
                    attempt, code, transportError.c_str(), tlsDetail.c_str(), aiMemDiag().c_str(), WiFi.RSSI());
      if (attempt < 2) { delay(900); yield(); }
    }

    if (code <= 0) {
      if (!transportError.length()) transportError = "connection failed";
      error = "HTTPS transport failed: " + String(code) + " (" + transportError + ")";
      if (tlsDetail.length()) error += ". " + tlsDetail;
      error += ". Host " + host + ", RSSI " + String(WiFi.RSSI()) + " dBm; " + aiMemDiag() + ".";
      return false;
    }
    if (code < 200 || code >= 300) {
      String msg;
      if (!jsonStringAfter(resp, "\"message\"", msg)) msg = resp.substring(0, min((int)resp.length(), 500));
      error = "HTTP " + String(code) + (msg.length() ? ": " + msg : "");
      return false;
    }

    answer = "";
    if (extractAssistantText(resp, answer) && answer.length()) {
      if (answer.length() >= AI_MAX_RESULT_CHARS) answer += "\n[response truncated on device]";
      return true;
    }

    lastRoutedModel = "";
    lastFinish = "";
    jsonStringAfter(resp, "\"model\"", lastRoutedModel);
    jsonStringAfter(resp, "\"finish_reason\"", lastFinish);
    Serial.printf("[ai] HTTP success but no final content; routed_model=%s finish_reason=%s response_bytes=%u\n",
                  lastRoutedModel.c_str(), lastFinish.c_str(), (unsigned)resp.length());

    // openrouter/free can route to different free models. If one returns no final
    // content, make one fresh generation request rather than displaying reasoning.
    if (provider == "openrouter" && generationAttempt < generationAttempts) {
      delay(700);
      yield();
      continue;
    }
  }

  error = "The provider returned success but no final text response";
  if (lastRoutedModel.length()) error += " (model " + lastRoutedModel + ")";
  if (lastFinish.length()) error += ", finish_reason=" + lastFinish;
  error += ". OpenRouter was reached successfully; try Analyze Hive again or choose a specific model instead of openrouter/free.";
  return false;
}

static bool aiTestConnection(const String& provider, const String& key, const String& model, String& error) {
  if (WiFi.status() != WL_CONNECTED) { error = "Wi-Fi is not connected."; return false; }
  String url, host;
  if (provider == "openrouter") url = "https://openrouter.ai/api/v1/key";
  else if (provider == "openai") url = "https://api.openai.com/v1/models/" + model;
  else { error = "Unknown AI provider."; return false; }
  if (!aiResolveHost(provider, host, error)) return false;

  WiFiClientSecure tls;
  tls.setInsecure();
  tls.setHandshakeTimeout(30);
  tls.setTimeout(AI_HTTP_TIMEOUT_MS);
  HTTPClient http;
  if (!http.begin(tls, url)) { error = "Could not start HTTPS connection to " + host + "."; return false; }
  http.setConnectTimeout(30000);
  http.setTimeout(AI_HTTP_TIMEOUT_MS);
  http.setReuse(false);
  http.addHeader("Authorization", "Bearer " + key);
  if (provider == "openrouter") {
    http.addHeader("X-Title", "Apiary Ears");
    http.addHeader("HTTP-Referer", "https://github.com/");
  }
  Serial.printf("[ai] TEST %s; %s; RSSI=%d dBm\n", host.c_str(), aiMemDiag().c_str(), WiFi.RSSI());
  int code = http.GET();
  String resp = code > 0 ? http.getString() : String();
  String transportError = code <= 0 ? HTTPClient::errorToString(code) : String();
  char tlsBuf[160] = {0};
  int tlsCode = code <= 0 ? tls.lastError(tlsBuf, sizeof(tlsBuf)) : 0;
  http.end();
  if (code >= 200 && code < 300) return true;
  if (code <= 0) {
    if (!transportError.length()) transportError = "connection failed";
    error = "HTTPS transport failed: " + String(code) + " (" + transportError + ")";
    if (tlsCode != 0) error += ". TLS " + String(tlsCode) + " (" + String(tlsBuf) + ")";
    error += ". Host " + host + ", RSSI " + String(WiFi.RSSI()) + " dBm; " + aiMemDiag() + ".";
    return false;
  }
  String msg;
  if (!jsonStringAfter(resp, "\"message\"", msg)) msg = resp.substring(0, min((int)resp.length(), 500));
  error = "HTTP " + String(code) + (msg.length() ? ": " + msg : "");
  return false;
}

struct AiWork {
  String provider, key, model, prompt;
  bool test;
};

static void aiWorker(void* pv) {
  AiWork* w = (AiWork*)pv;
  String answer, error;
  bool ok = w->test ? aiTestConnection(w->provider, w->key, w->model, error)
                    : aiHttpCall(w->provider, w->key, w->model, w->prompt, answer, error);
  if (g_aiMutex) xSemaphoreTake(g_aiMutex, portMAX_DELAY);
  g_aiLastOk = ok;
  g_aiLastError = ok ? "" : error;
  g_aiLastAction = w->test ? "test" : "analysis";
  if (w->test) {
    g_aiNotice = ok ? "Connection test successful." : "Connection test failed.";
  } else {
    if (ok) {
      g_aiLastResult = answer;
      time_t tt = time(nullptr);
      g_aiLastEpoch = (tt > 1600000000) ? (uint32_t)tt : (millis()/1000UL);
      g_aiNotice = "Analysis complete.";
    } else {
      g_aiLastResult = "";
      g_aiLastEpoch = 0;
      g_aiNotice = "Analysis failed.";
    }
  }
  g_aiTask = "idle";
  g_aiBusy = false;
  if (g_aiMutex) xSemaphoreGive(g_aiMutex);
  delete w;
  vTaskDelete(nullptr);
}

static bool aiStart(bool test, String& error) {
  if (g_aiBusy) { error = "AI is already working."; return false; }
  String provider, key, model;
  if (g_aiMutex) xSemaphoreTake(g_aiMutex, portMAX_DELAY);
  provider=g_aiProvider; key=g_aiKey; model=g_aiModel;
  if (g_aiMutex) xSemaphoreGive(g_aiMutex);
  if ((provider != "openrouter" && provider != "openai") || key.length() < 8 || !model.length()) {
    error = "AI is not configured."; return false;
  }
  AiWork* w = new AiWork();
  if (!w) { error = "Not enough memory to start AI task."; return false; }
  w->provider=provider; w->key=key; w->model=model; w->test=test;
  w->prompt = test ? "Reply with exactly: OK" : buildHivePrompt();

  if (g_aiMutex) xSemaphoreTake(g_aiMutex, portMAX_DELAY);
  g_aiBusy = true;
  g_aiTask = test ? "test" : "analysis";
  g_aiLastError = "";
  g_aiLastAction = test ? "test" : "analysis";
  if (!test) { g_aiLastResult = ""; g_aiLastEpoch = 0; }
  g_aiNotice = test ? "Testing AI connection..." : "Analyzing current hive data...";
  if (g_aiMutex) xSemaphoreGive(g_aiMutex);

  BaseType_t made = xTaskCreate(aiWorker, "ai-http", 8192, w, 1, nullptr);
  if (made != pdPASS) {
    if (g_aiMutex) xSemaphoreTake(g_aiMutex, portMAX_DELAY);
    g_aiBusy=false; g_aiTask="idle"; g_aiLastError="Could not create AI task.";
    if (g_aiMutex) xSemaphoreGive(g_aiMutex);
    delete w; error="Could not create AI task."; return false;
  }
  return true;
}

static void sendJsonCode(int code, const String& s) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(code, "application/json", s);
}

static void handleAIStatus() {
  String provider, model, result, err, notice, task, lastAction; bool busy, lastOk, configured; uint32_t ep;
  if (g_aiMutex) xSemaphoreTake(g_aiMutex, portMAX_DELAY);
  provider=g_aiProvider; model=g_aiModel; result=g_aiLastResult; err=g_aiLastError; notice=g_aiNotice; task=g_aiTask; lastAction=g_aiLastAction;
  busy=g_aiBusy; lastOk=g_aiLastOk; ep=g_aiLastEpoch; configured=aiConfigured();
  if (g_aiMutex) xSemaphoreGive(g_aiMutex);
  String j="{\"ok\":true,\"configured\":" + String(configured?"true":"false") +
           ",\"provider\":\"" + jsonEscape(provider) + "\",\"provider_label\":\"" + jsonEscape(aiProviderLabel(provider)) +
           "\",\"model\":\"" + jsonEscape(model) + "\",\"busy\":" + String(busy?"true":"false") +
           ",\"task\":\"" + jsonEscape(task) + "\",\"last_action\":\"" + jsonEscape(lastAction) + "\",\"last_ok\":" + String(lastOk?"true":"false") +
           ",\"last_epoch\":" + String(ep) + ",\"notice\":\"" + jsonEscape(notice) +
           "\",\"error\":\"" + jsonEscape(err) + "\",\"result\":\"" + jsonEscape(result) + "\"}";
  sendJsonCode(200,j);
}

static void handleAIConfig() {
  if (g_aiBusy) { sendJsonCode(409,"{\"ok\":false,\"error\":\"AI is busy; try again when it finishes.\"}"); return; }
  String provider=server.arg("provider"), key=server.arg("key"), model=server.arg("model");
  provider.trim(); key.trim(); model.trim();
  if (provider != "openrouter" && provider != "openai") {
    sendJsonCode(400,"{\"ok\":false,\"error\":\"Choose OpenRouter or OpenAI.\"}"); return;
  }
  if (!model.length()) model = provider=="openrouter" ? AI_OPENROUTER_MODEL : AI_OPENAI_MODEL;
  if (model.length() > 120 || key.length() > 300) {
    sendJsonCode(400,"{\"ok\":false,\"error\":\"AI model or key is too long.\"}"); return;
  }
  if (g_aiMutex) xSemaphoreTake(g_aiMutex, portMAX_DELAY);
  if (!key.length() && provider == g_aiProvider && g_aiKey.length()) key = g_aiKey; // edit model without re-entering secret
  if (key.length() < 8) {
    if (g_aiMutex) xSemaphoreGive(g_aiMutex);
    sendJsonCode(400,"{\"ok\":false,\"error\":\"Paste an API key first.\"}"); return;
  }
  g_aiProvider=provider; g_aiKey=key; g_aiModel=model;
  g_aiLastResult=""; g_aiLastEpoch=0; g_aiLastError=""; g_aiLastOk=false; g_aiLastAction="";
  g_aiNotice="Saved. Test the connection.";
  aiPrefs.putString("provider",g_aiProvider); aiPrefs.putString("key",g_aiKey); aiPrefs.putString("model",g_aiModel);
  if (g_aiMutex) xSemaphoreGive(g_aiMutex);
  sendJsonCode(200,"{\"ok\":true}");
}

static void handleAIDelete() {
  if (g_aiBusy) { sendJsonCode(409,"{\"ok\":false,\"error\":\"AI is busy; wait for it to finish first.\"}"); return; }
  if (g_aiMutex) xSemaphoreTake(g_aiMutex, portMAX_DELAY);
  g_aiProvider=""; g_aiKey=""; g_aiModel=""; g_aiLastResult=""; g_aiLastError=""; g_aiNotice="AI connection deleted."; g_aiLastAction=""; g_aiLastEpoch=0; g_aiLastOk=false;
  aiPrefs.clear();
  if (g_aiMutex) xSemaphoreGive(g_aiMutex);
  sendJsonCode(200,"{\"ok\":true}");
}

static void handleAITest() {
  String err;
  if (!aiStart(true,err)) { sendJsonCode(g_aiBusy?409:400,"{\"ok\":false,\"error\":\""+jsonEscape(err)+"\"}"); return; }
  sendJsonCode(202,"{\"ok\":true,\"started\":true}");
}

static void handleAIAnalyze() {
  String err;
  if (!aiStart(false,err)) { sendJsonCode(g_aiBusy?409:400,"{\"ok\":false,\"error\":\""+jsonEscape(err)+"\"}"); return; }
  sendJsonCode(202,"{\"ok\":true,\"started\":true}");
}

// -------------------------------------------------------------------- web ---
static void sendJson(const String& s) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", s);
}

static String specJson(const float* s) {
  String j = "[";
  for (int k = 0; k < NOSE_STEPS; k++) {
    if (k) j += ',';
    j += (isnan(s[k]) ? String("null") : String(s[k], 1));
  }
  return j + "]";
}

static void handleNow() {
  String j = "{\"ok\":true,\"fw\":\"" FW_VERSION "\",\"node\":\"" NODE_NAME "\",";
  j += "\"score\":" + String(g_score, 1) + ",\"peak\":" + String(g_scorePeak, 1) + ",\"comp\":[";
  for (int i = 0; i < 4; i++) { if (i) j += ','; j += String(g_scComp[i], 0); }
  j += "],\"dbfs\":" + String(g_dbfs, 1) + ",\"clip\":" + String(g_clip ? "true" : "false");
  j += ",\"baseline_ready\":" + String(g_baseReady ? "true" : "false");
  j += ",\"bands_hz\":[";
  for (int b = 0; b < BAND_COUNT; b++) { if (b) j += ','; j += String(BAND_HZ[b], 0); }
  j += "],\"band_db\":[";
  for (int b = 0; b < BAND_COUNT; b++) { if (b) j += ','; j += String(g_band[b], 1); }
  j += "],\"band_dev\":[";
  for (int b = 0; b < BAND_COUNT; b++) { if (b) j += ','; j += String(g_dev[b], 1); }
  j += "],\"steps_c\":[";
  for (int k = 0; k < NOSE_STEPS; k++) { if (k) j += ','; j += String(HEATER_C[k]); }
  j += "],\"nose\":{\"ok\":" + String(g_noseOk ? "true" : "false") +
       ",\"scans\":" + String(g_noseSeq) + ",\"burst_pos\":" + String(g_nosePos) +
       ",\"spec\":" + specJson(g_specLast) +
       ",\"mean_kohm\":" + (isnan(g_noseMean) ? String("null") : String(g_noseMean, 1)) +
       ",\"baseline_kohm\":" + (isnan(g_noseBase) ? String("null") : String(g_noseBase, 1)) +
       ",\"dev\":" + String(g_noseDev, 3) + "}";
  j += ",\"temp_c\":" + (isnan(g_tempC) ? String("null") : String(g_tempC, 1));
  j += ",\"rh\":" + (isnan(g_rh) ? String("null") : String(g_rh, 1));
  j += ",\"hpa\":" + (isnan(g_hpa) ? String("null") : String(g_hpa, 1));
  j += ",\"temp_rate_h\":" + String(g_tempRate, 2);
  j += ",\"in_event\":" + String(g_inEvent ? "true" : "false");
  j += ",\"recording\":" + String(g_clipRecording ? "true" : "false") + "}";
  sendJson(j);
}

static void handleHistory() {
  uint32_t mins = server.hasArg("minutes") ? (uint32_t)server.arg("minutes").toInt() : 180;
  if (mins < 5) mins = 5;
  if (mins > MINUTE_RING) mins = MINUTE_RING;
  uint16_t take = (uint16_t)min<uint32_t>(mins, g_minCount);
  String j = "{\"ok\":true,\"rows\":[";
  for (uint16_t i = 0; i < take; i++) {
    const MinuteRow& r = g_min[(g_minHead + MINUTE_RING - take + i) % MINUTE_RING];
    if (i) j += ',';
    j += "{\"epoch\":" + String(r.epoch) + ",\"score\":" + String(r.score) +
         ",\"nose\":" + String(r.noseMean) + ",\"t\":" + String(r.t10 / 10.0f, 1) +
         ",\"rh\":" + String(r.rh) + ",\"b\":[";
    for (int b = 0; b < BAND_COUNT; b++) { if (b) j += ','; j += String(r.band[b]); }
    j += "],\"c\":[";
    for (int i = 0; i < 4; i++) { if (i) j += ','; j += String(r.comp[i]); }
    j += "]}";
  }
  j += "]}";
  sendJson(j);
}

static void handleEvents() {
  String j = "{\"ok\":true,\"events\":[";
  for (uint8_t i = 0; i < g_evCount; i++) {
    const Event& e = g_events[i];
    if (i) j += ',';
    j += "{\"epoch\":" + String(e.epoch) + ",\"dur_s\":" + String(e.durS) +
         ",\"peak\":" + String(e.peak) + ",\"comp\":[";
    for (int k = 0; k < 4; k++) { if (k) j += ','; j += String(e.comp[k]); }
    j += "],\"band_dev\":[";
    for (int b = 0; b < BAND_COUNT; b++) { if (b) j += ','; j += String(e.band[b], 1); }
    j += "],\"spec\":" + specJson(e.spec) +
         ",\"nose_dev\":" + String(e.noseDev, 3) +
         ",\"temp_c\":" + (isnan(e.tempC) ? String("null") : String(e.tempC, 1)) +
         ",\"rh\":" + (isnan(e.rh) ? String("null") : String(e.rh, 1)) +
         ",\"temp_rate_h\":" + String(e.tempRate, 2);
    j += ",\"note\":\"" + String(e.note) + "\"}";
  }
  j += "]}";
  sendJson(j);
}

static void handleNote() {
  uint32_t ep = (uint32_t)server.arg("epoch").toInt();
  String txt = server.arg("text");
  for (uint8_t i = 0; i < g_evCount; i++) if (g_events[i].epoch == ep) {
    strncpy(g_events[i].note, txt.c_str(), sizeof(g_events[i].note) - 1);
    g_events[i].note[sizeof(g_events[i].note) - 1] = 0;
  }
  sendJson("{\"ok\":true}");
}

static void handleRecord() {
  if (g_clipRecording) { sendJson("{\"ok\":false,\"error\":\"already recording\"}"); return; }
  String label = server.arg("label");
  clipStart(label.length() ? label.c_str() : "manual");
  sendJson("{\"ok\":true,\"seconds\":" + String(CLIP_SECONDS) + "}");
}

static void handleClipWav() {
  if (!g_clipBuf || !g_clipLen) { server.send(404, "text/plain", "no clip recorded yet\n"); return; }
  uint32_t bytes = g_clipLen * 2;
  uint8_t hdr[44];
  writeWavHeader(hdr, bytes);
  server.setContentLength(44 + bytes);
  server.sendHeader("Content-Disposition",
                    "attachment; filename=\"" + String(g_clipLabel) + "-" + String(g_clipEpoch) + ".wav\"");
  server.send(200, "audio/wav", "");
  WiFiClient c = server.client();
  c.write(hdr, 44);
  const size_t CH = 2048;
  for (size_t off = 0; off < g_clipLen; off += CH) {
    size_t n = min(CH, g_clipLen - off);
    c.write((const uint8_t*)(g_clipBuf + off), n * 2);
  }
}

static void handleListen() {
  WiFiClient c = server.client();
  c.print("HTTP/1.1 200 OK\r\nContent-Type: audio/wav\r\nConnection: close\r\n\r\n");
  uint8_t hdr[44];
  writeWavHeader(hdr, 0xFFFFFFFF - 44);
  c.write(hdr, 44);
  uint32_t started = millis();
  static int16_t out[512];
  while (c.connected() && millis() - started < 300000UL) {     // five minutes
    size_t got = 0;
    if (i2s_read(I2S_NUM_0, i2sBuf, 512 * sizeof(int32_t), &got, pdMS_TO_TICKS(200)) != ESP_OK) break;
    int have = got / sizeof(int32_t);
    for (int i = 0; i < have; i++) out[i] = (int16_t)(i2sBuf[i] >> 16);
    if (c.write((const uint8_t*)out, have * 2) <= 0) break;
  }
  c.stop();
}

// GET /api/nose/history?n=200 - fingerprints for the spectrogram and clustering
static void handleNoseHistory() {
  uint16_t want = server.hasArg("n") ? (uint16_t)server.arg("n").toInt() : 200;
  if (want < 10) want = 10;
  if (want > NOSE_DAY) want = NOSE_DAY;
  bool day = server.arg("window") == "day";
  uint16_t take = day ? (uint16_t)min<uint16_t>(want, g_dayCount)
                      : (uint16_t)min<uint16_t>(want, g_noseCount);
  bool settled = server.arg("settled") != "0";        // hide post-rest scans by default

  String j = "{\"ok\":true,\"window\":\"" + String(day ? "day" : "recent") + "\",\"steps_c\":[";
  for (int k = 0; k < NOSE_STEPS; k++) { if (k) j += ','; j += String(HEATER_C[k]); }
  j += "],\"scans\":[";
  bool first = true;
  for (uint16_t i = 0; i < take; i++) {
    uint32_t ep; const uint16_t* g; uint8_t pos;
    if (day) {
      const NoseDayRow& r = g_noseDay[(g_dayHead + NOSE_DAY - take + i) % NOSE_DAY];
      ep = r.epoch; g = r.kohm; pos = 2;             // already averaged, all settled
    } else {
      const NoseRow& r = g_noseRing[(g_noseHead + NOSE_RING - take + i) % NOSE_RING];
      if (settled && r.pos == 1) continue;
      ep = r.epoch; g = r.kohm; pos = r.pos;
    }
    if (!first) j += ',';
    first = false;
    j += "{\"epoch\":" + String(ep) + ",\"pos\":" + String(pos) + ",\"g\":[";
    for (int k = 0; k < NOSE_STEPS; k++) { if (k) j += ','; j += String(g[k]); }
    j += "]}";
  }
  j += "]}";
  sendJson(j);
}

// GET /api/clips - what is on flash, and how much room is left
static void handleClipList() {
  String j = "{\"ok\":true,\"storage\":{\"total\":" + String((unsigned long)fsTotal()) +
             ",\"used\":" + String((unsigned long)fsUsed()) +
             ",\"free\":" + String((unsigned long)fsFree()) +
             ",\"keep\":" + String(CLIP_KEEP) + "},\"clips\":[";
  File d = LittleFS.open("/clips");
  bool first = true;
  if (d && d.isDirectory()) {
    for (File f = d.openNextFile(); f; f = d.openNextFile()) {
      if (f.isDirectory()) continue;
      String n = f.name();
      if (n.startsWith("/")) n = n.substring(1);
      if (!n.endsWith(".wav")) continue;              // the json is listed with its audio
      int dash = n.indexOf('-');
      String ep = dash > 0 ? n.substring(0, dash) : "0";
      String lb = dash > 0 ? n.substring(dash + 1) : n;
      lb.replace(".wav", "");
      if (!first) j += ',';
      first = false;
      String stem = n.substring(0, n.length() - 4);
      bool hasSnap = LittleFS.exists("/clips/" + stem + ".json");
      j += "{\"name\":\"" + n + "\",\"epoch\":" + ep +
           ",\"label\":\"" + lb + "\",\"bytes\":" + String((unsigned long)f.size()) +
           ",\"snapshot\":" + String(hasSnap ? "true" : "false") +
           ",\"snapshot_name\":\"" + stem + ".json\"}";
    }
  }
  j += "]}";
  sendJson(j);
}

static void handleClipFile() {
  String n = server.arg("name");
  if (!n.length() || n.indexOf("..") >= 0 || n.indexOf('/') >= 0) {
    server.send(400, "text/plain", "bad name\n"); return;
  }
  String path = "/clips/" + n;
  File f = LittleFS.open(path, "r");
  if (!f) { server.send(404, "text/plain", "no such recording\n"); return; }
  bool isJson = n.endsWith(".json");
  if (!isJson) server.sendHeader("Content-Disposition", "attachment; filename=\"" + n + "\"");
  server.streamFile(f, isJson ? "application/json" : "audio/wav");
  f.close();
}

static void handleClipDelete() {
  String n = server.arg("name");
  if (!n.length() || n.indexOf("..") >= 0 || n.indexOf('/') >= 0) {
    sendJson("{\"ok\":false,\"error\":\"bad name\"}"); return;
  }
  bool ok = LittleFS.remove("/clips/" + n);
  if (n.endsWith(".wav")) LittleFS.remove("/clips/" + n.substring(0, n.length() - 4) + ".json");
  sendJson(String("{\"ok\":") + (ok ? "true" : "false") + "}");
}

// GET /api/node - uptime, board health, and a verdict per input with what to do
static void handleNode() {
  uint32_t up = (millis()) / 1000UL;
  bool micOk = (g_dbfs > -119.0f);
  bool noseFresh = g_noseLastMs && (millis() - g_noseLastMs < 180000UL);

  String j = "{\"ok\":true,\"fw\":\"" FW_VERSION "\",\"node\":\"" NODE_NAME "\",";
  j += "\"uptime_s\":" + String(up) +
       ",\"die_c\":" + String(temperatureRead(), 1) +
       ",\"heap\":" + String((unsigned long)ESP.getFreeHeap()) +
       ",\"psram\":" + String((unsigned long)ESP.getFreePsram()) +
       ",\"rssi\":" + String(WiFi.RSSI()) +
       ",\"ip\":\"" + WiFi.localIP().toString() + "\"";
  j += ",\"storage\":{\"total\":" + String((unsigned long)fsTotal()) +
       ",\"used\":" + String((unsigned long)fsUsed()) +
       ",\"free\":" + String((unsigned long)fsFree()) +
       ",\"clips\":" + String(clipCount()) + ",\"keep\":" + String(CLIP_KEEP) + "}";

  j += ",\"inputs\":[";
  // microphone
  j += String("{\"name\":\"INMP441 microphone\",\"pins\":\"SD GPIO") + String(I2S_SD) +
       " / WS GPIO" + String(I2S_WS) + " / SCK GPIO" + String(I2S_SCK) + " / L\u002fR to GND\"" +
       ",\"ok\":" + (micOk ? "true" : "false") +
       ",\"detail\":\"" + String(g_dbfs, 0) + " dBFS" + (g_clip ? ", CLIPPING" : "") + "\"" +
       ",\"verdict\":\"" + String(micOk
         ? (g_clip ? "Audio is arriving but the input is clipping. Move the microphone further from the entrance or recess it behind a baffle.\""
                   : "Audio is arriving.\"")
         : "Silent. Check L\u002fR is tied to GND, then that SD, WS and SCK are on the pins above. "
           "A mis-wired INMP441 fails quietly. The mic also sleeps if the bit clock drops below about 1 MHz, "
           "so do not lower SAMPLE_RATE much.\"") + "}";
  // BME688
  j += String(",{\"name\":\"BME688 gas sensor\",\"pins\":\"SDA GPIO") + String(I2C_SDA) +
       " / SCL GPIO" + String(I2C_SCL) + " / VCC 3V3 / GND\"" +
       ",\"ok\":" + ((g_noseOk && noseFresh) ? "true" : "false") +
       ",\"detail\":\"" + (g_bmeAddr ? ("0x" + String(g_bmeAddr, HEX) + ", " + String(g_noseSeq) + " scans")
                                        : String("not found on the bus")) + "\"" +
       ",\"verdict\":\"" + String(!g_bmeAddr
         ? "Nothing answers at 0x77 or 0x76. Measure 3.3 V at the sensor's own pins first. "
           "Then check SDA - on many breakouts it is the pin marked MOSI - and SCL. "
           "Then the address switch. Reseating the connector has fixed this before.\""
         : (!noseFresh ? "On the bus but no recent scan. Usually a marginal supply on a long cable: "
                         "shorten it, or fit a capacitor at the sensor.\""
                       : "Scanning normally.\"")) + "}";
  // clock, because timestamps in exports depend on it
  time_t tt = time(nullptr);
  bool clockOk = tt > 1600000000;
  j += String(",{\"name\":\"Clock (NTP)\",\"pins\":\"network\"") +
       ",\"ok\":" + (clockOk ? "true" : "false") +
       ",\"detail\":\"" + (clockOk ? String("synced") : String("not set")) + "\"" +
       ",\"verdict\":\"" + String(clockOk
         ? "Recordings and labels carry real timestamps.\""
         : "No NTP yet, so timestamps fall back to seconds since boot. Harmless locally, "
           "but exports are harder to line up with anyone else's. Check the network allows outbound NTP.\"") + "}";
  j += "]}";
  sendJson(j);
}

// POST /api/selfcheck - look again, now that you have changed something.
// Re-probes the bus and retries the sensor if it was missing, so plugging a
// cable back in does not need a reboot.
static void handleSelfCheck() {
  String before = g_noseOk ? "ok" : "missing";
  if (!g_noseOk) {
    Serial.println("[selfcheck] BME688 was missing - probing again");
    noseBegin();
  } else {
    g_bmeAddr = i2cPing(BME_ADDR_A) ? BME_ADDR_A : (i2cPing(BME_ADDR_B) ? BME_ADDR_B : 0);
    if (!g_bmeAddr) {
      Serial.println("[selfcheck] BME688 stopped answering - re-initialising");
      g_noseOk = false;
      noseBegin();
    }
  }
  // give the microphone a moment to prove itself rather than reporting a stale level
  static float mono[WIN_SAMPLES];
  readWindow(mono, WIN_SAMPLES);
  sendJson(String("{\"ok\":true,\"was\":\"") + before + "\",\"now\":\"" +
           (g_noseOk ? "ok" : "missing") + "\",\"mic_dbfs\":" + String(g_dbfs, 1) + "}");
}

// POST /api/restart
static void handleRestart() {
  sendJson("{\"ok\":true,\"message\":\"restarting\"}");
  Serial.println("[node] restart requested from the page");
  delay(250);
  ESP.restart();
}

static void handleExport() {
  uint32_t mins = server.hasArg("minutes") ? (uint32_t)server.arg("minutes").toInt() : 60;
  if (mins < 1) mins = 1;
  if (mins > MINUTE_RING) mins = MINUTE_RING;
  String label = server.arg("label"); label.trim();
  if (!label.length()) { server.send(400, "text/plain", "a label is required\n"); return; }
  uint16_t take = (uint16_t)min<uint32_t>(mins, g_minCount);

  String o = "{\n  \"schema_version\": 1,\n  \"kind\": \"acoustic\",\n";
  o += "  \"node\": \"" NODE_NAME "\",\n  \"firmware\": \"apiary_ears " FW_VERSION "\",\n";
  o += "  \"label\": \"" + label + "\",\n  \"notes\": \"" + server.arg("notes") + "\",\n";
  o += "  \"bands_hz\": [";
  for (int b = 0; b < BAND_COUNT; b++) { if (b) o += ','; o += String(BAND_HZ[b], 0); }
  o += "],\n  \"nose_steps_c\": [";
  for (int k = 0; k < NOSE_STEPS; k++) { if (k) o += ','; o += String(HEATER_C[k]); }
  o += "],\n  \"score_weights\": {\"fanning\":0.25,\"agitation\":0.35,\"piping\":0.30,\"environment\":0.10},\n";
  o += "  \"minutes\": [\n";
  for (uint16_t i = 0; i < take; i++) {
    const MinuteRow& r = g_min[(g_minHead + MINUTE_RING - take + i) % MINUTE_RING];
    if (i) o += ",\n";
    o += "    {\"epoch\": " + String(r.epoch) + ", \"score\": " + String(r.score) +
         ", \"nose_mean_kohm\": " + String(r.noseMean) +
         ", \"temp_c\": " + String(r.t10 / 10.0f, 1) + ", \"rh\": " + String(r.rh) + ", \"band_db\": [";
    for (int b = 0; b < BAND_COUNT; b++) { if (b) o += ','; o += String(r.band[b] / 1.8f, 1); }
    o += "]}";
  }
  o += "\n  ],\n  \"events\": [\n";
  bool first = true;
  for (uint8_t i = 0; i < g_evCount; i++) {
    const Event& e = g_events[i];
    if (!first) o += ",\n";
    first = false;
    o += "    {\"epoch\": " + String(e.epoch) + ", \"dur_s\": " + String(e.durS) +
         ", \"peak_score\": " + String(e.peak) +
         ", \"components\": {\"fanning\": " + String(e.comp[0]) + ", \"agitation\": " + String(e.comp[1]) +
         ", \"piping\": " + String(e.comp[2]) + ", \"environment\": " + String(e.comp[3]) + "}, \"band_dev_db\": [";
    for (int b = 0; b < BAND_COUNT; b++) { if (b) o += ','; o += String(e.band[b], 1); }
    o += "], \"nose_spec_kohm\": " + specJson(e.spec) +
         ", \"nose_dev\": " + String(e.noseDev, 3) +
         ", \"temp_c\": " + (isnan(e.tempC) ? String("null") : String(e.tempC, 1)) +
         ", \"rh\": " + (isnan(e.rh) ? String("null") : String(e.rh, 1)) +
         ", \"temp_rate_h\": " + String(e.tempRate, 2) +
         ", \"note\": \"" + String(e.note) + "\"}";
  }
  o += "\n  ]\n}\n";
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + label + ".json\"");
  server.send(200, "application/json", o);
}

static const char PAGE[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Apiary Ears</title>
<style>
.pat{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:8px}
.pat>div{border:1px solid var(--edge);border-radius:10px;padding:8px 10px}
.pat .bars{height:34px}
.ok{color:#5fd39a}.bad{color:#e04e3a}
.diag{border:1px solid var(--edge);border-radius:10px;padding:9px 11px;margin-bottom:8px}
:root{--bg:#14110c;--pan:#1b1712;--edge:rgba(232,185,35,.22);--honey:#e8b923;--cream:#e9dcc3;--mut:#9a8a6a;--crit:#e04e3a}
*{box-sizing:border-box}
body{background:var(--bg);color:var(--cream);font:14px/1.5 system-ui,sans-serif;margin:auto;padding:18px;max-width:880px}
h1{color:var(--honey);font-size:22px;margin:6px 0 2px}
h2{color:var(--honey);font-size:12px;letter-spacing:.08em;text-transform:uppercase;margin:0 0 8px}
.sub{color:var(--mut);font-size:12px}
.card{background:var(--pan);border:1px solid var(--edge);border-radius:12px;padding:13px 15px;margin-top:12px}
button,input,select,textarea{background:transparent;color:var(--honey);border:1px solid var(--edge);border-radius:999px;padding:6px 13px;margin:0 6px 6px 0;font-size:12px;font-family:inherit;cursor:pointer}
input,select,textarea{border-radius:8px;color:var(--cream);cursor:text}
select option{background:var(--pan);color:var(--cream)}
.topbar{display:flex;align-items:flex-start;justify-content:space-between;gap:12px}
.aiout{white-space:pre-wrap;background:rgba(0,0,0,.28);border:1px solid var(--edge);border-radius:10px;padding:11px;margin-top:10px;min-height:50px;color:var(--cream)}
.modal{display:none;position:fixed;inset:0;background:rgba(0,0,0,.72);z-index:20;padding:18px;overflow:auto}
.modal.show{display:flex;align-items:center;justify-content:center}
.modalbox{width:min(620px,100%);background:var(--pan);border:1px solid var(--edge);border-radius:14px;padding:16px 18px;box-shadow:0 20px 60px rgba(0,0,0,.55)}
.modalbox label{display:block;color:var(--mut);font-size:11px;margin:10px 0 4px}
.modalbox input,.modalbox select{width:100%;padding:9px 10px;margin:0}
.warn{border-left:3px solid var(--honey);padding-left:9px}
button.on{background:var(--honey);color:var(--bg);font-weight:600}
.big{font-size:46px;font-weight:700;line-height:1}
.row{display:flex;gap:18px;flex-wrap:wrap;align-items:flex-end}
.meter{height:9px;background:rgba(255,255,255,.08);border-radius:5px;overflow:visible;margin-top:5px;position:relative}
.meter i{display:block;height:100%;background:var(--honey);transition:width .3s;border-radius:5px}
.meter .t{position:absolute;top:-3px;width:2px;height:15px;background:var(--cream);border-radius:1px;transition:left .6s}
.meter .t.w{height:11px;top:-1px;background:rgba(233,220,195,.4)}
.eq{display:flex;gap:3px;height:90px;align-items:flex-end;position:relative}
.eq .b{flex:1;position:relative;height:100%}
.eq .f{position:absolute;left:0;right:0;bottom:0;min-height:2px;border-radius:3px 3px 0 0;transition:height .25s,background-color .4s;background-image:repeating-linear-gradient(to top,transparent 0 5px,var(--bg) 5px 7px)}
.eq .p{position:absolute;left:0;right:0;height:2px;background:var(--cream);border-radius:1px;pointer-events:none}
.eq .a{position:absolute;left:12%;right:12%;height:0;border-top:1px dashed rgba(233,220,195,.5);pointer-events:none;transition:bottom .6s}
.eq .g{position:absolute;left:0;right:0;bottom:0;top:0;border-radius:3px 3px 0 0;background:rgba(255,255,255,.035)}
.bars{display:flex;gap:3px;align-items:flex-end;height:90px}
.bars i{flex:1;border-radius:3px 3px 0 0;min-height:2px;transition:height .25s}
canvas{width:100%;border:1px solid var(--edge);border-radius:10px;background:rgba(0,0,0,.3);display:block}
table{width:100%;border-collapse:collapse;font-size:12px}
td,th{padding:5px 6px;border-bottom:1px solid rgba(255,255,255,.07);text-align:left;vertical-align:top}
th{color:var(--mut);font-weight:500}
.lab{display:flex;justify-content:space-between;font-size:10px;color:var(--mut);margin-top:4px}
.tag{font-size:10px;padding:1px 8px;border-radius:999px;background:rgba(232,185,35,.2);color:var(--honey)}
.note{color:var(--mut);font-size:12px;margin-top:8px}
a{color:var(--honey)}
</style></head><body>
<div class="topbar">
  <div><h1>Apiary Ears</h1><div class="sub" id="hdr">connecting…</div></div>
  <button onclick="openAI()" title="Configure optional AI hive analysis">✦ Configure AI</button>
</div>

<div class="card">
  <div class="row">
    <div><div class="sub">Score now</div><div class="big" id="score">–</div></div>
    <div style="flex:1;min-width:230px">
      <div class="sub">fanning · agitation · piping · environment <span style="opacity:.7">· live fill, bright tick = 10-min average, faint tick = window average</span></div>
      <div id="comps"></div>
    </div>
    <div><div class="sub">level</div><div id="dbfs" class="sub">–</div><div class="sub" id="evstate"></div></div>
  </div>
  <div class="note" id="baseNote"></div>
</div>

<div class="card">
  <div style="display:flex;justify-content:space-between;gap:10px;align-items:center">
    <h2 style="margin:0">AI hive analysis</h2><span class="tag" id="aiBadge">not configured</span>
  </div>
  <div class="sub" id="aiInfo" style="margin:7px 0 9px">Optional. Your sensor features stay local until you press Analyze Hive.</div>
  <button id="aiAnalyzeBtn" onclick="aiAnalyze()">✦ Analyze hive</button>
  <button onclick="openAI()">⚙ Configure AI</button>
  <button id="aiTestBtn" onclick="aiTest()">↻ Test / reconnect</button>
  <div class="note" id="aiWork"></div>
  <div class="aiout" id="aiResult">No AI analysis yet.</div>
  <div class="note">Only the numeric sensor summary, recent minute statistics and event summaries are sent. Audio recordings are not uploaded.</div>
</div>

<div class="card">
  <h2>Sound — twelve bands</h2>
  <div class="eq" id="bars"></div>
  <div class="lab" id="bandLab"></div>
  <div class="note">Bars turn honey then red as a band rises above this hive's own baseline. The absolute height means little; the colour means something. The bright tick is the recent peak — it holds, then sinks and fades until a new high sets it again. The dashed tick is the last ten minutes' average.</div>
  <canvas id="bandHist" width="840" height="120" style="margin-top:10px"></canvas>
  <div class="lab"><span id="bandFrom"></span><span>rows = bands, low at the top · brighter = louder in that band</span><span>now</span></div>
</div>

<div class="card">
  <h2>Smell — the ten heater steps</h2>
  <div class="eq" id="nose" style="height:60px"></div>
  <div class="lab" id="noseLab"></div>
  <div class="note" id="noseMsg"></div>
</div>

<div class="card">
  <h2>Fingerprint over time</h2>
  <div id="specWin" style="margin-bottom:8px"></div>
  <canvas id="spec" width="840" height="190"></canvas>
  <div class="lab"><span id="specFrom"></span><span>rows = heater steps · brighter = more gas at that temperature</span><span>now</span></div>
  <div class="note">Each column is one scan. A smell arriving shows as a vertical band; which rows light up is the shape that tells one smell from another.</div>
</div>

<div class="card">
  <h2>Recurring patterns</h2>
  <div class="pat" id="pats"></div>
  <div class="note">Fingerprint shapes grouped automatically, intensity divided out. Same colour = the same kind of mixture. A new pattern appearing means something changed, before anything has been trained.</div>
</div>

<div class="card">
  <h2>Score, smell and temperature</h2>
  <canvas id="hist" width="840" height="190"></canvas>
  <div class="lab"><span id="histFrom"></span><span>honey = score · green = smell (lower resistance is more gas) · grey = temperature</span><span>now</span></div>
  <div id="win" style="margin-top:8px"></div>
</div>

<div class="card">
  <h2>Listen and record</h2>
  <button id="listenBtn" onclick="listen()">▶ Live listen</button>
  <audio id="liveAudio" style="display:none;width:100%;margin-top:8px" controls></audio>
  <input id="recLabel" placeholder="what is this? e.g. inspection, calm evening" maxlength="39" style="width:42%">
  <button onclick="record()">● Record 10 s</button>
  <div class="note" id="recMsg">Live listen streams the microphone straight to your browser. Recording captures ten seconds into memory — download it before the board reboots.</div>
</div>

<div class="card">
  <h2>Recordings kept on the board</h2>
  <div class="meter" style="height:12px"><i id="stBar" style="width:0%"></i></div>
  <div class="lab"><span id="stText">reading storage…</span><span id="stKeep"></span></div>
  <div id="clips" style="margin-top:8px"></div>
  <div class="note">Every recording is a pair: the audio, and a snapshot of what every sensor
  said at that moment — score and its parts, all twelve bands, the ten-step fingerprint,
  temperature, humidity and pressure. Flash is small, about five pairs, so the board deletes
  the oldest set before recording a new one. Download anything worth keeping.</div>
</div>

<div class="card">
  <h2>Node</h2>
  <div id="nodeSummary" class="sub">…</div>
  <button onclick="toggleNode()" id="nodeBtn">Show diagnostics</button>
  <button onclick="selfCheck()" id="scBtn" title="Look again — use this after plugging a cable back in">&#8635; Self-check</button>
  <button onclick="restartNode()" style="border-color:rgba(224,78,58,.5);color:#e04e3a">Restart node</button>
  <div class="sub" id="scMsg" style="margin-top:6px"></div>
  <div id="nodeDetail" style="display:none;margin-top:8px"></div>
</div>

<div class="card">
  <h2>Events the board recorded on its own</h2>
  <div id="events"></div>
  <div class="note">An event opens when the score stays high for three windows and closes when it falls back. Each keeps the band deviations, the fingerprint and the conditions at its peak, so it can be re-scored later with different weights.</div>
</div>

<div class="card">
  <h2>Export</h2>
  <input id="xpLabel" placeholder="label" maxlength="39" style="width:34%">
  <input id="xpMins" type="number" min="5" max="1440" value="180" style="width:88px"> <span class="sub">minutes</span>
  <button onclick="exportJson()">Download JSON</button>
  <div class="note">Minute-by-minute bands, scores, smell and conditions, plus every event — the format the shared dataset uses.</div>
</div>

<div id="aiModal" class="modal" onclick="if(event.target===this)closeAI()">
  <div class="modalbox">
    <div style="display:flex;justify-content:space-between;align-items:center;gap:12px">
      <h1 style="font-size:18px;margin:0">Configure AI</h1><button onclick="closeAI()">✕</button>
    </div>
    <div class="note">Wi-Fi stays configured in the .ino exactly as before. This only adds optional AI credentials after flashing.</div>
    <label>Provider</label>
    <select id="aiProvider" onchange="aiProviderChanged(true)">
      <option value="openrouter">OpenRouter — recommended / free-model option</option>
      <option value="openai">OpenAI API — advanced / paid API</option>
    </select>
    <div class="note warn" id="aiHelp"></div>
    <label>API key</label>
    <input id="aiKey" type="password" autocomplete="off" placeholder="paste key here">
    <div class="note">The key is stored in this ESP32's NVS and is never shown back in the page. If already configured, leave this blank to keep the existing key.</div>
    <label>Model</label>
    <input id="aiModel" maxlength="120" placeholder="model name">
    <div style="margin-top:14px">
      <button class="on" onclick="aiSave()" id="aiSaveBtn">Save & test connection</button>
      <button onclick="closeAI()">Cancel</button>
      <button id="aiDeleteBtn" onclick="aiDelete()" style="border-color:rgba(224,78,58,.5);color:#e04e3a">Delete AI connection</button>
    </div>
    <div class="note" id="aiCfgMsg"></div>
    <div class="note">Security: configure keys only on a trusted private Wi-Fi network. This page itself is HTTP, and the key is stored locally in flash/NVS.</div>
  </div>
</div>

<div class="sub" style="margin:18px 0 30px">
  The score is a hypothesis, not a prediction. Sound moves in minutes and smell over hours, so a
  spike with no change in the fingerprint is probably weather or machinery, while both moving together
  is a colony doing something. Whether any of it anticipates a swarm is the open question.
</div>

<script>
let MIN=1440, N=null, H=null, AI=null;
// Peak-hold state for the equalisers. Each entry: cur (live fraction), pk (peak fraction), t (ms when peak was set).
const EQ={bars:{s:[],mx:0,hold:8000,fade:45000,sink:.06},nose:{s:[],mx:0,hold:120000,fade:1200000,sink:.02}};
function eqBuild(id,n){ const e=el(id); if(e.children.length===n) return;
  e.innerHTML=Array.from({length:n},()=>'<div class="b"><div class="g"></div><div class="f"></div><div class="a" style="display:none"></div><div class="p" style="display:none"></div></div>').join('');
  EQ[id].s=Array.from({length:n},()=>({cur:0,pk:0,t:0})); }
function eqSet(id,fracs,cols,titles,avgs){ const q=EQ[id], now=Date.now(); eqBuild(id,fracs.length);
  [...el(id).children].forEach((b,i)=>{ const st=q.s[i], f=Math.max(0,Math.min(1,fracs[i]||0)); st.cur=f;
    if(f>=st.pk){ st.pk=f; st.t=now; }
    const F=b.children[1]; F.style.height=Math.max(2,Math.round(100*f))+'%'; F.style.backgroundColor=cols[i]; b.title=titles[i];
    const A=b.children[2]; if(avgs&&avgs[i]!=null){ A.style.display='block'; A.style.bottom=(100*Math.max(0,Math.min(1,avgs[i])))+'%'; } else A.style.display='none'; }); }
function eqAnimate(){ const now=Date.now();
  for(const id in EQ){ const q=EQ[id], e=el(id); if(!e) continue;
    q.s.forEach((st,i)=>{ const b=e.children[i]; if(!b) return; const P=b.children[3];
      const age=now-st.t; let op=1;
      if(age>q.hold){ op=Math.max(0,1-(age-q.hold)/q.fade); st.pk=Math.max(st.cur, st.pk-(st.pk-st.cur)*q.sink); }
      if(st.pk<=st.cur+0.005||op<=0){ P.style.display='none'; return; }
      P.style.display='block'; P.style.bottom=(100*st.pk)+'%'; P.style.opacity=op.toFixed(2); }); } }
setInterval(eqAnimate,120);
function compAvg(rows){ if(!rows||!rows.length||!rows[0].c) return null; const a=[0,0,0,0]; rows.forEach(r=>r.c.forEach((v,i)=>a[i]+=v)); return a.map(v=>v/rows.length); }
const el=i=>document.getElementById(i);
const WINS=[[60,'1 h'],[180,'3 h'],[720,'12 h'],[1440,'24 h']];
function sc(v){return v>=70?'#e04e3a':v>=45?'#e8b923':'#5fd39a'}
async function tick(){
  try{ N=await (await fetch('/api/now')).json(); }catch(e){ el('hdr').textContent='node unreachable'; return; }
  el('hdr').innerHTML=`fw ${N.fw} · mic ${N.dbfs>-119?'ok':'<span style="color:#e04e3a">silent</span>'}`
    + ` · nose ${N.nose.ok?N.nose.scans+' scans':'<span style="color:#e04e3a">not found</span>'}`
    + (N.temp_c!=null?` · ${N.temp_c.toFixed(1)} °C`:'') + (N.rh!=null?` · ${N.rh.toFixed(0)} % RH`:'');
  el('score').textContent=N.score.toFixed(0);
  el('score').style.color=sc(N.score);
  const names=['fanning','agitation','piping','environment'];
  const rows=(H&&H.rows)||[], a10=compAvg(rows.slice(-10)), aW=compAvg(rows), wl=(WINS.find(w=>w[0]===MIN)||[0,''])[1];
  el('comps').innerHTML=N.comp.map((v,i)=>{
    const t10=a10?`<span class="t" style="left:${a10[i].toFixed(0)}%" title="10-min average ${a10[i].toFixed(0)}"></span>`:'';
    const tW=aW?`<span class="t w" style="left:${aW[i].toFixed(0)}%" title="${wl} average ${aW[i].toFixed(0)}"></span>`:'';
    return `<div style="font-size:11px;color:var(--mut)">${names[i]} <b style="color:var(--cream)">${v}</b>${a10?` <span style="opacity:.7">avg ${a10[i].toFixed(0)}</span>`:''}
     <div class="meter"><i style="width:${v}%"></i>${t10}${tW}</div></div>`;}).join('');
  el('dbfs').textContent=N.dbfs.toFixed(0)+' dBFS'+(N.clip?' · CLIPPING':'');
  el('evstate').innerHTML=N.in_event?'<span class="tag">event in progress</span>':(N.recording?'<span class="tag">recording</span>':'');
  el('baseNote').textContent=N.baseline_ready?'':'Learning this hive\u2019s baseline — scores settle after a minute or so.';
  // Scale relaxes slowly (about a minute) so bars don't jump every poll and peaks stay comparable.
  EQ.bars.mx=Math.max(Math.max(...N.band_db,1), EQ.bars.mx*0.99);
  const b10=rows.slice(-10), bAvg=b10.length&&b10[0].b?N.band_db.map((_,i)=>(b10.reduce((s,r)=>s+r.b[i],0)/b10.length/1.8)/EQ.bars.mx):null;
  eqSet('bars', N.band_db.map(v=>v/EQ.bars.mx),
    N.band_dev.map(d=>d>3?'#e04e3a':d>1?'#e8b923':'#7a6a46'),
    N.band_db.map((v,i)=>`${N.bands_hz[i]} Hz: ${v.toFixed(1)} dB (${N.band_dev[i]>=0?'+':''}${N.band_dev[i].toFixed(1)} vs baseline)`), bAvg);
  el('bandLab').innerHTML=N.bands_hz.map(h=>`<span>${h}</span>`).join('');
  const sp=N.nose.spec.map(v=>v==null?0:v);
  EQ.nose.mx=Math.max(Math.max(...sp,1), EQ.nose.mx*0.998);
  eqSet('nose', sp.map(v=>v/EQ.nose.mx), sp.map(()=>'#e8b923'),
    sp.map((v,i)=>`step ${i} (${N.steps_c[i]}°): ${v.toFixed(0)} kΩ`), null);
  el('noseLab').innerHTML=N.steps_c.map(c=>`<span>${c}°</span>`).join('');
  el('noseMsg').textContent = N.nose.ok
    ? `burst position ${N.nose.burst_pos}${N.nose.burst_pos===1?' (first after a rest — reads high, kept out of the baseline)':''}`
      + (N.nose.mean_kohm!=null?` · mean ${N.nose.mean_kohm.toFixed(0)} kΩ`:'')
      + (N.nose.baseline_kohm!=null?` · baseline ${N.nose.baseline_kohm.toFixed(0)} kΩ`:'')
      + (N.nose.dev>=0
          ? ` · ${(N.nose.dev*100).toFixed(0)}% more gas than usual`
          : ` · ${(-N.nose.dev*100).toFixed(0)}% less gas than usual`)
    : 'No BME688 found — sound still works, the environment component sits at zero.';
}
async function loadHist(){
  el('win').innerHTML=WINS.map(w=>`<button class="${MIN===w[0]?'on':''}" onclick="MIN=${w[0]};loadHist()">${w[1]}</button>`).join('');
  try{ H=await (await fetch('/api/history?minutes='+MIN)).json(); }catch(e){ return; }
  drawHist(); drawBandHist();
}
function drawHist(){
  const cv=el('hist'), c=cv.getContext('2d'), W=cv.width, Ht=cv.height, R=H.rows, n=R.length;
  c.fillStyle='#0d0b07'; c.fillRect(0,0,W,Ht);
  if(!n){ c.fillStyle='#9a8a6a'; c.font='12px system-ui'; c.fillText('no history yet — a row is written every minute',16,Ht/2); return; }
  c.strokeStyle='rgba(255,255,255,.10)'; c.setLineDash([4,4]);
  [45,70].forEach(v=>{const y=Ht-10-(Ht-26)*v/100; c.beginPath(); c.moveTo(30,y); c.lineTo(W-6,y); c.stroke();
    c.fillStyle='#9a8a6a'; c.font='10px system-ui'; c.fillText(v,8,y+3);});
  c.setLineDash([]);
  // Auto-scaling a nearly-flat series turns a rounding wobble into a mountain
  // range. Give every series a minimum span - a floor in its own units, and at
  // least a few percent of its own magnitude - so flat things look flat.
  function span(vals, floorAbs, floorFrac){
    if(!vals.length) return [0,1];
    let lo=Math.min(...vals), hi=Math.max(...vals);
    const mid=(lo+hi)/2;
    const need=Math.max(floorAbs, Math.abs(mid)*floorFrac);
    if(hi-lo < need){ lo = mid-need/2; hi = mid+need/2; }
    const pad=(hi-lo)*0.10;                        // breathing room top and bottom
    return [lo-pad, hi+pad];
  }
  const nz=R.map(r=>r.nose>0?r.nose:null).filter(v=>v!=null);
  const [nLo,nHi]=span(nz, 5, 0.08);               // 8 % of the reading, or 5 kOhm
  const tv=R.map(r=>r.t>-99?r.t:null).filter(v=>v!=null);
  const [tLo,tHi]=span(tv, 2.0, 0.02);             // at least a 2 degree window
  const X=i=>30+(W-36)*i/Math.max(1,n-1);
  function line(get,lo,hi,col,w,invert){ c.strokeStyle=col; c.lineWidth=w; c.beginPath(); let on=false;
    R.forEach((r,i)=>{ const v=get(r); if(v==null) return;
      let f=(v-lo)/Math.max(1e-6,hi-lo); if(invert) f=1-f;
      const y=Ht-10-(Ht-26)*f; on?c.lineTo(X(i),y):c.moveTo(X(i),y); on=true; }); c.stroke(); }
  line(r=>r.nose>0?r.nose:null, nLo, nHi, 'rgba(95,211,154,.75)', 1.5, true);   // inverted: more gas = higher
  line(r=>r.t>-99?r.t:null, tLo, tHi, 'rgba(200,190,170,.45)', 1.5, false);
  line(r=>r.score, 0, 100, '#e8b923', 2.5, false);
  el('histFrom').textContent=R[0].epoch>1600000000?new Date(R[0].epoch*1000).toLocaleString():'';
}
async function loadEvents(){
  let E; try{ E=await (await fetch('/api/events')).json(); }catch(e){ return; }
  const rows=E.events.slice().reverse();
  el('events').innerHTML = rows.length ? '<table><tr><th>when</th><th>peak</th><th>for</th><th>smell &amp; conditions</th><th>note</th></tr>'+
    rows.map(e=>{
      const when=e.epoch>1600000000?new Date(e.epoch*1000).toLocaleString():e.epoch+' s';
      const comps=['fan','agit','pipe','env'].map((k,i)=>`${k} ${e.comp[i]}`).join(' · ');
      const cond=[ (e.nose_dev!=null?`smell ${(e.nose_dev*100).toFixed(0)}%`:null),
                   (e.temp_c!=null?e.temp_c.toFixed(1)+' °C':null),
                   (e.temp_rate_h?`${e.temp_rate_h>0?'+':''}${e.temp_rate_h.toFixed(1)} °C/h`:null)
                 ].filter(Boolean).join(' · ');
      return `<tr><td>${when}</td><td><b style="color:${sc(e.peak)}">${e.peak}</b><div class="sub">${comps}</div></td>
              <td>${e.dur_s} s</td><td>${cond}</td>
              <td><span onclick="note(${e.epoch},'${(e.note||'').replace(/'/g,'')}')" style="cursor:pointer">${e.note||'<span class="sub">add…</span>'}</span></td></tr>`;
    }).join('')+'</table>'
    : '<div class="note">Nothing yet. The board opens an event when the score stays high for three windows in a row.</div>';
}
async function note(ep,cur){
  const t=prompt('What was happening? (this travels with the event when exported)',cur||'');
  if(t===null) return;
  await fetch(`/api/note?epoch=${ep}&text=${encodeURIComponent(t)}`,{method:'POST'});
  loadEvents();
}
// Live listen plays in the page rather than opening a window: the charts keep
// updating while you listen, which is half the point of listening.
function listen(){
  const a = el('liveAudio'), b = el('listenBtn');
  if(!a.paused && a.src){ a.pause(); a.removeAttribute('src'); a.load(); a.style.display='none';
                          b.textContent='\u25b6 Live listen'; b.classList.remove('on'); return; }
  a.src = '/listen.wav?t=' + Date.now();     // fresh URL so the browser does not reuse a finished stream
  a.style.display = 'block';
  a.play().then(()=>{ b.textContent='\u25a0 Stop listening'; b.classList.add('on'); })
          .catch(e=>{ el('recMsg').textContent =
            'The browser would not play the stream. Open http://'+location.host+'/listen.wav directly, or use VLC.'; });
}
function colour(f){
  const st=[[0,[27,16,48]],[.3,[42,111,151]],[.55,[29,158,117]],[.8,[232,185,35]],[1,[224,78,58]]];
  f=Math.max(0,Math.min(1,f)); let a=st[0],b=st[st.length-1];
  for(let i=0;i<st.length-1;i++) if(f>=st[i][0]&&f<=st[i+1][0]){a=st[i];b=st[i+1];break}
  const t=(f-a[0])/Math.max(1e-6,b[0]-a[0]), c=a[1].map((v,i)=>Math.round(v+(b[1][i]-v)*t));
  return `rgb(${c[0]},${c[1]},${c[2]})`;
}
let NH=null, SPECWIN='day';
function specWinBtns(){
  el('specWin').innerHTML = [['day','24 hours'],['recent','last hour']].map(w=>
    `<button class="${SPECWIN===w[0]?'on':''}" onclick="SPECWIN='${w[0]}';loadNose()">${w[1]}</button>`).join('')
    + `<span class="sub" style="margin-left:8px">${SPECWIN==='day'
        ? 'one settled scan per burst, a column a minute \u2014 the map of the day'
        : 'every scan as measured, including the climb through each burst'}</span>`;
}
async function loadNose(){
  specWinBtns();
  const n = SPECWIN==='day' ? 1440 : 240;
  try{ NH = await (await fetch(`/api/nose/history?n=${n}&window=${SPECWIN}`)).json(); }catch(e){ return; }
  drawSpec(); patterns();
}
function drawBandHist(){
  const cv=el('bandHist'); if(!cv) return; const c=cv.getContext('2d'), W=cv.width, Ht=cv.height, R=(H&&H.rows)||[], n=R.length;
  c.fillStyle='#0d0b07'; c.fillRect(0,0,W,Ht);
  if(!n||!R[0].b){ c.fillStyle='#9a8a6a'; c.font='12px system-ui'; c.fillText('no sound history yet — a row is written every minute',16,Ht/2); return; }
  const rows=R[0].b.length, rh=(Ht-4)/rows, cw=Math.max(1,W/n), lo=[], hi=[];
  for(let k=0;k<rows;k++){ const col=R.map(r=>r.b[k]); lo[k]=Math.min(...col); hi[k]=Math.max(...col); }
  for(let i=0;i<n;i++) for(let k=0;k<rows;k++){
    const v=R[i].b[k]; c.fillStyle=colour(1-(v-lo[k])/Math.max(1,hi[k]-lo[k]));
    c.fillRect(i*cw,k*rh,Math.ceil(cw),Math.ceil(rh)); }
  c.fillStyle='#9a8a6a'; c.font='10px system-ui';
  const hz=(N&&N.bands_hz)||[];
  for(let k=0;k<rows;k++) if(k%2===0) c.fillText(hz[k]||'',3,k*rh+rh/2+3);
  el('bandFrom').textContent=R[0].epoch>1600000000?new Date(R[0].epoch*1000).toLocaleString():'';
}
function drawSpec(){
  const cv=el('spec'), c=cv.getContext('2d'), W=cv.width, H=cv.height;
  c.fillStyle='#0d0b07'; c.fillRect(0,0,W,H);
  const S=(NH&&NH.scans)||[]; const n=S.length;
  if(!n){ c.fillStyle='#9a8a6a'; c.font='12px system-ui';
          c.fillText('waiting for the first complete scan',16,H/2); return; }
  const rows=10, rh=(H-4)/rows, cw=Math.max(1,W/n), lo=[], hi=[];
  for(let k=0;k<rows;k++){ const col=S.map(s=>s.g[k]).filter(v=>v>0);
    lo[k]=col.length?Math.min(...col):0; hi[k]=col.length?Math.max(...col):1; }
  for(let i=0;i<n;i++) for(let k=0;k<rows;k++){
    const v=S[i].g[k]; if(!v) continue;
    c.fillStyle=colour(1-(v-lo[k])/Math.max(1e-6,hi[k]-lo[k]));
    c.fillRect(i*cw,k*rh,Math.ceil(cw),Math.ceil(rh));
  }
  c.fillStyle='#9a8a6a'; c.font='10px system-ui';
  for(let k=0;k<rows;k++) c.fillText(`${k}\u00b7${(NH.steps_c||[])[k]||''}\u00b0`,3,k*rh+rh/2+3);
  const t0=S[0].epoch;
  el('specFrom').textContent = t0>1600000000 ? new Date(t0*1000).toLocaleString() : '';
}
function patterns(){
  const S=(NH&&NH.scans)||[], n=S.length, k=4;
  if(n<8){ el('pats').innerHTML='<div class="note">Needs at least eight scans to group shapes.</div>'; return; }
  const mean=g=>{const v=g.filter(x=>x>0); return v.length?v.reduce((a,b)=>a+b,0)/v.length:1};
  const rows=S.map(s=>{const m=mean(s.g); return s.g.map(v=>v>0?v/m:1)});
  const order=rows.map((r,i)=>[mean(S[i].g),i]).sort((a,b)=>a[0]-b[0]);
  let P=[0,1,2,3].map(j=>rows[order[Math.floor((j+.5)*n/k)][1]].slice());
  let A=new Array(n).fill(0);
  const dist=(a,b)=>a.reduce((s,x,i)=>s+(x-b[i])*(x-b[i]),0);
  for(let it=0;it<12;it++){
    const sum=P.map(()=>new Array(10).fill(0)), cnt=new Array(k).fill(0);
    rows.forEach((r,i)=>{ let bi=0,bd=Infinity; P.forEach((p,j)=>{const d=dist(r,p); if(d<bd){bd=d;bi=j}});
      A[i]=bi; cnt[bi]++; r.forEach((x,c2)=>sum[bi][c2]+=x); });
    P=P.map((p,j)=>cnt[j]?sum[j].map(x=>x/cnt[j]):p);
  }
  const cols=['#7A9BFF','#5fd39a','#F2A623','#E06AA3'], names=['A','B','C','D'];
  el('pats').innerHTML=P.map((p,j)=>{
    const cnt=A.filter(x=>x===j).length, mx=Math.max(...p);
    return `<div><div style="display:flex;justify-content:space-between;font-size:11px;margin-bottom:4px">
      <b style="color:${cols[j]}">Pattern ${names[j]}</b><span>${Math.round(100*cnt/n)}% of scans</span></div>
      <div class="bars">${p.map(v=>`<i style="height:${Math.max(2,Math.round(34*v/mx))}px;background:${cols[j]}"></i>`).join('')}</div></div>`;
  }).join('');
}
function kb(b){ return b>=1048576 ? (b/1048576).toFixed(1)+' MB' : Math.round(b/1024)+' KB'; }
async function loadClips(){
  let C; try{ C=await (await fetch('/api/clips')).json(); }catch(e){ return; }
  const st=C.storage||{total:0,used:0,free:0};
  const pct = st.total ? Math.round(100*st.used/st.total) : 0;
  el('stBar').style.width = pct+'%';
  el('stBar').style.background = pct>85 ? '#e04e3a' : (pct>65 ? '#e8b923' : '#5fd39a');
  el('stText').textContent = `${kb(st.used)} used of ${kb(st.total)} \u00b7 ${kb(st.free)} free`;
  el('stKeep').textContent = `${(C.clips||[]).length} of ${st.keep} kept`;
  const rows=(C.clips||[]).slice().sort((a,b)=>b.epoch-a.epoch);
  el('clips').innerHTML = rows.length ? '<table>'+rows.map(c=>
    `<tr><td>${c.epoch>1600000000?new Date(c.epoch*1000).toLocaleString():c.epoch+' s'}</td>
     <td>${c.label}${c.snapshot?'':' <span class="sub">(audio only)</span>'}</td><td>${kb(c.bytes)}</td>
     <td style="text-align:right"><a href="/api/clip?name=${encodeURIComponent(c.name)}">audio</a>
     ${c.snapshot?` \u00b7 <a href="/api/clip?name=${encodeURIComponent(c.snapshot_name)}">snapshot</a>`:''}
     <span class="x" title="delete both" onclick="delClip('${c.name}')">\u2715</span></td></tr>`).join('')+'</table>'
    : '<div class="note">Nothing recorded yet.</div>';
}
async function delClip(n){
  if(!confirm('Delete this recording from the board?')) return;
  await fetch('/api/clip/delete?name='+encodeURIComponent(n), {method:'POST'});
  loadClips();
}
function toggleNode(){
  const d=el('nodeDetail'), b=el('nodeBtn');
  const show = d.style.display==='none';
  d.style.display = show ? 'block' : 'none';
  b.textContent = show ? 'Hide diagnostics' : 'Show diagnostics';
  if(show) loadNode();
}
function dur(s){
  const d=Math.floor(s/86400), h=Math.floor(s%86400/3600), m=Math.floor(s%3600/60);
  return (d?d+'d ':'')+(h||d?h+'h ':'')+m+'m';
}
async function selfCheck(){
  const b=el('scBtn'); b.textContent='\u21bb Checking\u2026'; b.disabled=true;
  el('scMsg').textContent='';
  try{
    const r = await (await fetch('/api/selfcheck',{method:'POST'})).json();
    let msg = `Sensor: ${r.was} \u2192 ${r.now}. Microphone at ${r.mic_dbfs.toFixed(0)} dBFS.`;
    if(r.was!=='ok' && r.now==='ok') msg += ' Found it — it is back.';
    else if(r.now!=='ok') msg += ' Still nothing on the bus; see the diagnostics below.';
    el('scMsg').textContent = msg;
  }catch(e){ el('scMsg').textContent='Could not reach the node.'; }
  b.textContent='\u21bb Self-check'; b.disabled=false;
  if(el('nodeDetail').style.display==='none') toggleNode(); else loadNode();
  loadNose();
}
async function restartNode(){
  if(!confirm('Restart the node?\n\nRecordings on flash survive. The score baseline and the '+
              'events list do not — they are rebuilt after about a minute.')) return;
  el('scMsg').textContent='Restarting\u2026';
  try{ await fetch('/api/restart',{method:'POST'}); }catch(e){}
  let n=0;
  const t=setInterval(async()=>{
    n++;
    try{ await fetch('/api/node',{cache:'no-store'}); clearInterval(t);
         el('scMsg').textContent='Back up.'; loadNode(); tick(); }
    catch(e){ el('scMsg').textContent='Restarting\u2026 ('+n+'s)'; }
    if(n>40){ clearInterval(t); el('scMsg').textContent='Not back yet — reload the page in a moment.'; }
  }, 1000);
}
async function loadNode(){
  let N2; try{ N2=await (await fetch('/api/node')).json(); }catch(e){ return; }
  el('nodeSummary').textContent =
    `up ${dur(N2.uptime_s)} \u00b7 ${N2.die_c.toFixed(0)} \u00b0C die \u00b7 ${kb(N2.heap)} heap \u00b7 ${N2.rssi} dBm \u00b7 ${N2.ip}`;
  const st=N2.storage||{};
  el('nodeDetail').innerHTML =
    (N2.inputs||[]).map(i=>
      `<div class="diag"><div style="display:flex;justify-content:space-between">
         <b class="${i.ok?'ok':'bad'}">${i.ok?'OK':'CHECK'} \u00b7 ${i.name}</b>
         <span class="sub">${i.detail}</span></div>
       <div style="margin-top:3px">${i.verdict}</div>
       <div class="sub" style="margin-top:3px;font-family:ui-monospace,monospace">${i.pins}</div></div>`).join('')
    + `<div class="diag"><b>Storage</b><div>${kb(st.used)} of ${kb(st.total)} used, ${kb(st.free)} free, `
    + `${st.clips} recording(s) of at most ${st.keep}. The oldest are deleted automatically before a new one is recorded.</div></div>`;
}
setInterval(()=>{ if(el('nodeDetail').style.display!=='none') loadNode(); }, 10000);
async function record(){
  const l=el('recLabel').value.trim()||'manual';
  const r=await (await fetch('/api/record?label='+encodeURIComponent(l),{method:'POST'})).json();
  el('recMsg').textContent = r.ok ? `Recording ${r.seconds} s…` : (r.error||'could not start');
  setTimeout(()=>{ el('recMsg').innerHTML='Done — <a href="/clip.wav">download the clip</a>.'; }, 11000);
}
function aiProviderChanged(forceModel=false){
  const p=el('aiProvider').value;
  if(forceModel || !el('aiModel').value.trim()) el('aiModel').value=p==='openrouter'?'openrouter/free':'gpt-5.6-luna';
  el('aiHelp').innerHTML = p==='openrouter'
    ? 'Recommended for the public project. <a href="https://openrouter.ai/settings/keys" target="_blank" rel="noopener">Sign in to OpenRouter and create/copy a key</a>, then paste it below. The default <b>openrouter/free</b> router chooses an available free model.'
    : 'Advanced option. <a href="https://platform.openai.com/api-keys" target="_blank" rel="noopener">Create/copy an OpenAI API key</a>. ChatGPT subscriptions and API billing are separate. The low-cost default here is <b>gpt-5.6-luna</b>.';
}
function openAI(){
  const p=(AI&&AI.provider)||'openrouter';
  el('aiProvider').value=p;
  el('aiModel').value=(AI&&AI.model)||(p==='openrouter'?'openrouter/free':'gpt-5.6-luna');
  el('aiKey').value=''; el('aiCfgMsg').textContent='';
  el('aiDeleteBtn').style.display=(AI&&AI.configured)?'inline-block':'none';
  aiProviderChanged(false); el('aiModal').classList.add('show');
}
function closeAI(){ el('aiModal').classList.remove('show'); }
async function loadAI(){
  try{ AI=await (await fetch('/api/ai/status',{cache:'no-store'})).json(); }catch(e){ return; }
  const on=!!AI.configured;
  const aiErr=on && !AI.busy && !!AI.error;
  const connErr=aiErr && AI.last_action==='test';
  const analysisErr=aiErr && AI.last_action==='analysis';
  el('aiBadge').textContent=!on?'not configured':(AI.busy?'working…':(connErr?'connection error':(analysisErr?'analysis error':(AI.last_ok?'connected':'configured'))));
  el('aiBadge').style.background=!on?'rgba(232,185,35,.2)':(aiErr?'rgba(224,78,58,.18)':'rgba(95,211,154,.18)');
  el('aiBadge').style.color=!on?'#e8b923':(aiErr?'#e04e3a':'#5fd39a');
  el('aiInfo').textContent=on?`${AI.provider_label} · ${AI.model}`:'Optional. Configure a provider once, then press Analyze Hive whenever you want a second opinion.';
  el('aiAnalyzeBtn').style.display=on?'inline-block':'none';
  el('aiTestBtn').style.display=on?'inline-block':'none';
  el('aiAnalyzeBtn').disabled=!!AI.busy; el('aiTestBtn').disabled=!!AI.busy;
  let m=AI.notice||''; if(AI.error) m+=(m?' ':'')+'Error: '+AI.error;
  el('aiWork').textContent=m;
  if(AI.busy && AI.task==='analysis') el('aiResult').textContent='Analyzing current hive data…';
  else if(AI.result) el('aiResult').textContent=AI.result;
  else if(!on) el('aiResult').textContent='Configure AI to enable on-demand hive analysis.';
  else if(!AI.busy && AI.error && AI.last_action==='analysis') el('aiResult').textContent='This analysis failed. No previous AI result is being shown.\n\n'+AI.error;
  else if(!AI.busy && AI.error) el('aiResult').textContent='AI connection needs attention. Press Test / reconnect.\n\n'+AI.error;
  else if(!AI.busy) el('aiResult').textContent=(AI.last_ok?'Connected. ':'Configured. ')+'Press Analyze Hive when you want an AI interpretation of the current sensor data.';
}
async function aiSave(){
  const b=el('aiSaveBtn'); b.disabled=true; el('aiCfgMsg').textContent='Saving…';
  const body=new URLSearchParams({provider:el('aiProvider').value,key:el('aiKey').value.trim(),model:el('aiModel').value.trim()});
  try{
    const r=await fetch('/api/ai/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});
    const j=await r.json();
    if(!r.ok||!j.ok){ el('aiCfgMsg').textContent=j.error||'Could not save AI settings.'; b.disabled=false; return; }
    await loadAI(); el('aiCfgMsg').textContent='Saved. Testing connection…'; closeAI(); await aiTest();
  }catch(e){ el('aiCfgMsg').textContent='Could not reach the node.'; }
  b.disabled=false;
}
async function aiPoll(){
  for(let i=0;i<70;i++){
    await new Promise(r=>setTimeout(r,1000)); await loadAI();
    if(AI && !AI.busy) return;
  }
  el('aiWork').textContent='The AI request is taking longer than expected. You can refresh this page and check again.';
}
async function aiTest(){
  el('aiWork').textContent='Starting connection test…';
  try{
    const r=await fetch('/api/ai/test',{method:'POST'}), j=await r.json();
    if(!r.ok||!j.ok){ el('aiWork').textContent=j.error||'Could not start test.'; return; }
    await loadAI(); aiPoll();
  }catch(e){ el('aiWork').textContent='Could not reach the node.'; }
}
async function aiAnalyze(){
  if(!AI||!AI.configured){ openAI(); return; }
  el('aiResult').textContent='Analyzing current hive data…';
  try{
    const r=await fetch('/api/ai/analyze',{method:'POST'}), j=await r.json();
    if(!r.ok||!j.ok){ el('aiWork').textContent=j.error||'Could not start analysis.'; return; }
    await loadAI(); aiPoll();
  }catch(e){ el('aiWork').textContent='Could not reach the node.'; }
}
async function aiDelete(){
  if(!confirm('Delete the AI connection and stored API key from this ESP32?')) return;
  try{
    const r=await fetch('/api/ai/delete',{method:'POST'}), j=await r.json();
    if(!r.ok||!j.ok){ el('aiCfgMsg').textContent=j.error||'Could not delete AI settings.'; return; }
    closeAI(); await loadAI();
  }catch(e){ el('aiCfgMsg').textContent='Could not reach the node.'; }
}
function exportJson(){
  const l=el('xpLabel').value.trim(); if(!l){ alert('Give the export a label first.'); return; }
  window.location=`/api/export?label=${encodeURIComponent(l)}&minutes=${el('xpMins').value}`;
}
tick(); loadHist(); loadEvents(); loadNose(); loadClips(); loadNode(); loadAI();
setInterval(tick,2000); setInterval(loadHist,60000); setInterval(loadEvents,30000);
setInterval(loadNose,20000); setInterval(loadClips,30000);
</script></body></html>)HTML";

static void handleRoot() { server.send_P(200, "text/html", PAGE); }

// ------------------------------------------------------------------ setup ---
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\n=== %s fw %s ===\n", NODE_NAME, FW_VERSION);

  g_aiMutex = xSemaphoreCreateMutex();
  aiPrefs.begin("apiary-ai", false);
  g_aiProvider = aiPrefs.getString("provider", "");
  g_aiKey      = aiPrefs.getString("key", "");
  g_aiModel    = aiPrefs.getString("model", "");
  if (g_aiProvider == "openrouter" && !g_aiModel.length()) g_aiModel = AI_OPENROUTER_MODEL;
  if (g_aiProvider == "openai" && !g_aiModel.length()) g_aiModel = AI_OPENAI_MODEL;
  Serial.printf("[ai] %s%s\n", aiConfigured() ? "configured: " : "not configured", aiConfigured() ? aiProviderLabel(g_aiProvider).c_str() : "");

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(NODE_NAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[wifi] connecting");
  for (int i = 0; i < 60 && WiFi.status() != WL_CONNECTED; i++) { delay(500); Serial.print('.'); }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("\n[wifi] %s\n[open] http://%s/\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
  else
    Serial.println("\n[wifi] not connected - check WIFI_SSID / WIFI_PASS.");

  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  if (!LittleFS.begin(true)) {
    Serial.println("[fs] LittleFS would not mount - recordings will not be kept.");
    Serial.println("     In the IDE: Tools -> Partition Scheme -> one with a SPIFFS/LittleFS area.");
  } else {
    if (!LittleFS.exists("/clips")) LittleFS.mkdir("/clips");
    Serial.printf("[fs] %u KB total, %u KB free, %d recording(s) kept (max %d)\n",
                  (unsigned)(fsTotal() / 1024), (unsigned)(fsFree() / 1024), clipCount(), CLIP_KEEP);
  }

  audioBegin();
  noseBegin();

  g_clipCap = (size_t)SAMPLE_RATE * CLIP_SECONDS;
  g_clipBuf = (int16_t*)ps_malloc(g_clipCap * sizeof(int16_t));
  if (!g_clipBuf) {
    g_clipCap = (size_t)SAMPLE_RATE * 2;
    g_clipBuf = (int16_t*)malloc(g_clipCap * sizeof(int16_t));
    Serial.println("[clip] no PSRAM - clips limited to 2 s");
  }
  g_minStartMs = millis();
  g_dayStartMs = millis();

  server.on("/", handleRoot);
  server.on("/api/now", handleNow);
  server.on("/api/history", handleHistory);
  server.on("/api/events", handleEvents);
  server.on("/api/note", handleNote);
  server.on("/api/record", handleRecord);
  server.on("/api/export", handleExport);
  server.on("/clip.wav", handleClipWav);
  server.on("/listen.wav", handleListen);
  server.on("/api/nose/history", handleNoseHistory);
  server.on("/api/clips", handleClipList);
  server.on("/api/clip", handleClipFile);
  server.on("/api/clip/delete", handleClipDelete);
  server.on("/api/node", handleNode);
  server.on("/api/selfcheck", handleSelfCheck);
  server.on("/api/restart", handleRestart);
  server.on("/api/ai/status", HTTP_GET, handleAIStatus);
  server.on("/api/ai/config", HTTP_POST, handleAIConfig);
  server.on("/api/ai/delete", HTTP_POST, handleAIDelete);
  server.on("/api/ai/test", HTTP_POST, handleAITest);
  server.on("/api/ai/analyze", HTTP_POST, handleAIAnalyze);
  server.begin();
  Serial.println("[http] listening on :80");
  Serial.println("[note] the score needs a minute to learn this hive's baseline before it means anything.");
}

void loop() {
  server.handleClient();
  if (g_noseOk) nose.run();          // BSEC decides when to talk to the sensor
  if (g_clipRecording) { clipService(); return; }   // recording takes priority
  scoreWindow();
  eventService();
  minuteRoll();
}
