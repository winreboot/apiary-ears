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
 *  v1.1.0
 * =============================================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <bsec2.h>
#include <math.h>
#include <time.h>

// ---------------------------------------------------------------- settings --
#define WIFI_SSID       "YOUR_WIFI"
#define WIFI_PASS       "YOUR_PASSWORD"
#define NODE_NAME       "apiary-ears"
#define FW_VERSION      "1.1.0"

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
#define BURST_GAP_MS    30000UL    // a longer pause means a new burst began

#define EVENT_ON        55         // score at or above this opens an event
#define EVENT_OFF       35         // and it closes when the score falls below
#define EVENT_ON_WINS   3
#define EVENT_COOLDOWN_S 300

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

struct MinuteRow {
  uint32_t epoch;
  uint8_t  band[BAND_COUNT];
  uint8_t  score;
  uint16_t noseMean;                   // kOhm, clipped to 65535
  int8_t   tempC;
  uint8_t  rh;
};
MinuteRow g_min[MINUTE_RING];
uint16_t  g_minHead = 0, g_minCount = 0;
uint32_t  g_minAccN = 0;
float     g_minAcc[BAND_COUNT], g_minScorePeak = 0;
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
  r.noseMean = isnan(g_noseMean) ? 0 : (uint16_t)fminf(g_noseMean, 65535.0f);
  r.tempC = isnan(g_tempC) ? -99 : (int8_t)roundf(g_tempC);
  r.rh = isnan(g_rh) ? 0 : (uint8_t)roundf(g_rh);
  g_minHead = (g_minHead + 1) % MINUTE_RING;
  if (g_minCount < MINUTE_RING) g_minCount++;
  for (int b = 0; b < BAND_COUNT; b++) g_minAcc[b] = 0;
  g_minAccN = 0; g_minScorePeak = 0;
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

      float sum = 0; int n = 0;
      for (int k = 0; k < NOSE_STEPS; k++) if (!isnan(g_spec[k]) && g_spec[k] > 0) { sum += g_spec[k]; n++; }
      float mean = n ? sum / n : NAN;

      // Position 1 is the first scan after the sensor rested: the surface
      // recovered, so it reads high for reasons that have nothing to do with
      // the air. Keep it in the record, keep it out of the baseline.
      if (!isnan(mean) && g_nosePos >= 2) {
        g_noseMean = mean;
        if (isnan(g_noseBase)) g_noseBase = mean;
        else g_noseBase = 0.995f * g_noseBase + 0.005f * mean;   // ~hours
        g_noseDev = (g_noseBase > 1.0f) ? (g_noseBase - mean) / g_noseBase : 0.0f;
      } else if (!isnan(mean)) {
        g_noseMean = mean;
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
  }
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
         ",\"nose\":" + String(r.noseMean) + ",\"t\":" + String(r.tempC) +
         ",\"rh\":" + String(r.rh) + ",\"b\":[";
    for (int b = 0; b < BAND_COUNT; b++) { if (b) j += ','; j += String(r.band[b]); }
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
         ", \"temp_c\": " + String(r.tempC) + ", \"rh\": " + String(r.rh) + ", \"band_db\": [";
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
:root{--bg:#14110c;--pan:#1b1712;--edge:rgba(232,185,35,.22);--honey:#e8b923;--cream:#e9dcc3;--mut:#9a8a6a;--crit:#e04e3a}
*{box-sizing:border-box}
body{background:var(--bg);color:var(--cream);font:14px/1.5 system-ui,sans-serif;margin:auto;padding:18px;max-width:880px}
h1{color:var(--honey);font-size:22px;margin:6px 0 2px}
h2{color:var(--honey);font-size:12px;letter-spacing:.08em;text-transform:uppercase;margin:0 0 8px}
.sub{color:var(--mut);font-size:12px}
.card{background:var(--pan);border:1px solid var(--edge);border-radius:12px;padding:13px 15px;margin-top:12px}
button,input{background:transparent;color:var(--honey);border:1px solid var(--edge);border-radius:999px;padding:6px 13px;margin:0 6px 6px 0;font-size:12px;font-family:inherit;cursor:pointer}
input{border-radius:8px;color:var(--cream);cursor:text}
button.on{background:var(--honey);color:var(--bg);font-weight:600}
.big{font-size:46px;font-weight:700;line-height:1}
.row{display:flex;gap:18px;flex-wrap:wrap;align-items:flex-end}
.meter{height:9px;background:rgba(255,255,255,.08);border-radius:5px;overflow:hidden;margin-top:5px}
.meter i{display:block;height:100%;background:var(--honey);transition:width .3s}
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
<h1>Apiary Ears</h1>
<div class="sub" id="hdr">connecting…</div>

<div class="card">
  <div class="row">
    <div><div class="sub">Score now</div><div class="big" id="score">–</div></div>
    <div style="flex:1;min-width:230px">
      <div class="sub">fanning · agitation · piping · environment</div>
      <div id="comps"></div>
    </div>
    <div><div class="sub">level</div><div id="dbfs" class="sub">–</div><div class="sub" id="evstate"></div></div>
  </div>
  <div class="note" id="baseNote"></div>
</div>

<div class="card">
  <h2>Sound — twelve bands</h2>
  <div class="bars" id="bars"></div>
  <div class="lab" id="bandLab"></div>
  <div class="note">Bars turn honey then red as a band rises above this hive's own baseline. The absolute height means little; the colour means something.</div>
</div>

<div class="card">
  <h2>Smell — the ten heater steps</h2>
  <div class="bars" id="nose" style="height:60px"></div>
  <div class="lab" id="noseLab"></div>
  <div class="note" id="noseMsg"></div>
</div>

<div class="card">
  <h2>Score, smell and temperature</h2>
  <canvas id="hist" width="840" height="190"></canvas>
  <div class="lab"><span id="histFrom"></span><span>honey = score · green = smell (lower resistance is more gas) · grey = temperature</span><span>now</span></div>
  <div id="win" style="margin-top:8px"></div>
</div>

<div class="card">
  <h2>Listen and record</h2>
  <button onclick="listen()">▶ Live listen</button>
  <input id="recLabel" placeholder="what is this? e.g. inspection, calm evening" maxlength="39" style="width:42%">
  <button onclick="record()">● Record 10 s</button>
  <div class="note" id="recMsg">Live listen streams the microphone straight to your browser. Recording captures ten seconds into memory — download it before the board reboots.</div>
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

<div class="sub" style="margin:18px 0 30px">
  The score is a hypothesis, not a prediction. Sound moves in minutes and smell over hours, so a
  spike with no change in the fingerprint is probably weather or machinery, while both moving together
  is a colony doing something. Whether any of it anticipates a swarm is the open question.
</div>

<script>
let MIN=180, N=null, H=null;
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
  el('comps').innerHTML=N.comp.map((v,i)=>
    `<div style="font-size:11px;color:var(--mut)">${names[i]} <b style="color:var(--cream)">${v}</b>
     <div class="meter"><i style="width:${v}%"></i></div></div>`).join('');
  el('dbfs').textContent=N.dbfs.toFixed(0)+' dBFS'+(N.clip?' · CLIPPING':'');
  el('evstate').innerHTML=N.in_event?'<span class="tag">event in progress</span>':(N.recording?'<span class="tag">recording</span>':'');
  el('baseNote').textContent=N.baseline_ready?'':'Learning this hive\u2019s baseline — scores settle after a minute or so.';
  const mx=Math.max(...N.band_db,1);
  el('bars').innerHTML=N.band_db.map((v,i)=>{
    const d=N.band_dev[i];
    return `<i style="height:${Math.max(2,Math.round(90*v/mx))}px;background:${d>3?'#e04e3a':d>1?'#e8b923':'#7a6a46'}"
      title="${N.bands_hz[i]} Hz: ${v.toFixed(1)} dB (${d>=0?'+':''}${d.toFixed(1)} vs baseline)"></i>`;}).join('');
  el('bandLab').innerHTML=N.bands_hz.map(h=>`<span>${h}</span>`).join('');
  const sp=N.nose.spec.map(v=>v==null?0:v), smx=Math.max(...sp,1);
  el('nose').innerHTML=sp.map((v,i)=>
    `<i style="height:${Math.max(2,Math.round(60*v/smx))}px;background:#e8b923;opacity:.85"
      title="step ${i} (${N.steps_c[i]}°): ${v.toFixed(0)} kΩ"></i>`).join('');
  el('noseLab').innerHTML=N.steps_c.map(c=>`<span>${c}°</span>`).join('');
  el('noseMsg').textContent = N.nose.ok
    ? `burst position ${N.nose.burst_pos}${N.nose.burst_pos===1?' (first after a rest — reads high, kept out of the baseline)':''}`
      + (N.nose.mean_kohm!=null?` · mean ${N.nose.mean_kohm.toFixed(0)} kΩ`:'')
      + (N.nose.baseline_kohm!=null?` · baseline ${N.nose.baseline_kohm.toFixed(0)} kΩ`:'')
      + ` · ${(N.nose.dev*100).toFixed(0)}% below baseline`
    : 'No BME688 found — sound still works, the environment component sits at zero.';
}
async function loadHist(){
  el('win').innerHTML=WINS.map(w=>`<button class="${MIN===w[0]?'on':''}" onclick="MIN=${w[0]};loadHist()">${w[1]}</button>`).join('');
  try{ H=await (await fetch('/api/history?minutes='+MIN)).json(); }catch(e){ return; }
  drawHist();
}
function drawHist(){
  const cv=el('hist'), c=cv.getContext('2d'), W=cv.width, Ht=cv.height, R=H.rows, n=R.length;
  c.fillStyle='#0d0b07'; c.fillRect(0,0,W,Ht);
  if(!n){ c.fillStyle='#9a8a6a'; c.font='12px system-ui'; c.fillText('no history yet — a row is written every minute',16,Ht/2); return; }
  c.strokeStyle='rgba(255,255,255,.10)'; c.setLineDash([4,4]);
  [45,70].forEach(v=>{const y=Ht-10-(Ht-26)*v/100; c.beginPath(); c.moveTo(30,y); c.lineTo(W-6,y); c.stroke();
    c.fillStyle='#9a8a6a'; c.font='10px system-ui'; c.fillText(v,8,y+3);});
  c.setLineDash([]);
  const nz=R.map(r=>r.nose>0?r.nose:null).filter(v=>v!=null);
  const nLo=nz.length?Math.min(...nz):0, nHi=nz.length?Math.max(...nz):1;
  const tv=R.map(r=>r.t>-99?r.t:null).filter(v=>v!=null);
  const tLo=tv.length?Math.min(...tv):0, tHi=tv.length?Math.max(...tv):1;
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
function listen(){ window.open('/listen.wav','_blank'); }
async function record(){
  const l=el('recLabel').value.trim()||'manual';
  const r=await (await fetch('/api/record?label='+encodeURIComponent(l),{method:'POST'})).json();
  el('recMsg').textContent = r.ok ? `Recording ${r.seconds} s…` : (r.error||'could not start');
  setTimeout(()=>{ el('recMsg').innerHTML='Done — <a href="/clip.wav">download the clip</a>.'; }, 11000);
}
function exportJson(){
  const l=el('xpLabel').value.trim(); if(!l){ alert('Give the export a label first.'); return; }
  window.location=`/api/export?label=${encodeURIComponent(l)}&minutes=${el('xpMins').value}`;
}
tick(); loadHist(); loadEvents();
setInterval(tick,2000); setInterval(loadHist,60000); setInterval(loadEvents,30000);
</script></body></html>)HTML";

static void handleRoot() { server.send_P(200, "text/html", PAGE); }

// ------------------------------------------------------------------ setup ---
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\n=== %s fw %s ===\n", NODE_NAME, FW_VERSION);

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

  server.on("/", handleRoot);
  server.on("/api/now", handleNow);
  server.on("/api/history", handleHistory);
  server.on("/api/events", handleEvents);
  server.on("/api/note", handleNote);
  server.on("/api/record", handleRecord);
  server.on("/api/export", handleExport);
  server.on("/clip.wav", handleClipWav);
  server.on("/listen.wav", handleListen);
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
