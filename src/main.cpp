/*
 * ============================================================================
 *  Ensō (円相) — Ambient Sound Simulator for M5Stack Cardputer ADV
 *  Version: v1.0
 *
 *  Everything you hear is synthesized in real time from noise, filters,
 *  resonators and oscillators — no samples are used. The firmware is built
 *  around fifteen sound "layers" grouped by genre (Water / Weather / Life /
 *  Fire / Meditative / Mechanical / Noise). Each layer has a volume, a color
 *  and three parameters. A "scene" is a combination of layer settings.
 *
 *  Structure of this file:
 *    1. Constants, palette, layer definitions, preset scenes
 *    2. Audio engine (control-rate ticks + per-sample generators)
 *    3. Persistence (SD card, JSON settings and scene files)
 *    4. Scene registry and scene fading
 *    5. UI (canvas rendering for every screen)
 *    6. Input handling
 *    7. setup() / loop()
 *
 *  DSP rule of thumb used throughout: expensive math (sinf/expf/div) is done
 *  once per audio buffer in the *tick* functions; the per-sample functions
 *  only use multiply/add and table lookups.
 * ============================================================================
 */

#include <M5Cardputer.h>
#include <SD.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <string.h>
#include <vector>
#include "esp_sleep.h"

// ---------------------------------------------------------------------------
// 1. Constants
// ---------------------------------------------------------------------------
static const char* FW_NAME    = "Enso";   // displayed with a macron drawn over the 'o'
static const char* FW_VERSION = "v1.0";   // shown on the boot splash and About screen -- bump this with every release

static const int   SAMPLE_RATE = 22050;                 // Hz — plenty for ambient textures, halves the DSP load
static const int   BUF         = 512;                   // samples per audio buffer (~23 ms)
static const float DT          = (float)BUF / SAMPLE_RATE;
static const int   SCREEN_W    = 240;
static const int   SCREEN_H    = 135;

// SD card pins (Cardputer / Cardputer ADV TF slot)
static const int SD_SCK = 40, SD_MISO = 39, SD_MOSI = 14, SD_CS = 12;
static const char* DIR_ROOT      = "/ENSO";
static const char* DIR_SCENE     = "/ENSO/scene";
static const char* SETTINGS_PATH = "/ENSO/settings.json";

// Timing
static const float BOOT_FADE_IN_SEC   = 1.8f;   // silence -> first scene, at startup only
static const float SCENE_XFADE_SEC    = 2.4f;   // crossfade time when switching scenes
static const uint32_t SAVE_DEBOUNCE_MS = 1000;   // settle time before writing to SD after a change
static const uint32_t KEY_REPEAT_DELAY_MS = 420;
static const uint32_t KEY_REPEAT_RATE_MS  = 60;

// ---------------------------------------------------------------------------
// Color palette (selectable per layer)
// ---------------------------------------------------------------------------
struct PaletteColor { const char* name; uint8_t r, g, b; };
static const PaletteColor PALETTE[] = {
  {"Sky",      110, 170, 230},
  {"Aqua",      90, 210, 220},
  {"Teal",      60, 165, 150},
  {"Mint",     140, 220, 170},
  {"Lime",     180, 220, 110},
  {"Sand",     225, 200, 140},
  {"Amber",    240, 170,  80},
  {"Coral",    240, 130, 110},
  {"Rose",     230, 120, 170},
  {"Lavender", 170, 140, 230},
  {"Indigo",   100, 110, 210},
  {"Fog",      170, 180, 190},
};
static const int NUM_PALETTE = sizeof(PALETTE) / sizeof(PALETTE[0]);

// Background / UI colors (RGB)
static const uint8_t BG_R = 10, BG_G = 12, BG_B = 16;

// On-board RGB LED: a single WS2812 on GPIO21. If a future board revision
// moves it, this is the only line that needs changing.
static const int LED_PIN = 21;
static Adafruit_NeoPixel ledStrip(1, LED_PIN, NEO_GRB + NEO_KHZ800);

// ---------------------------------------------------------------------------
// Layer definitions
// ---------------------------------------------------------------------------
// Layers are grouped by genre and ordered accordingly. The order below is the
// order shown in the EDIT list.
//
// v0.6 reordered this list and dropped the Train layer. That breaks scene
// files written by v0.1-v0.51, because scenes store layers as a positional
// JSON array -- an old file's layer 4 is not this build's layer 4. That was an
// accepted one-time cost while the project is still unreleased; from here on,
// new layers should be appended at the END OF THEIR GROUP and nothing should
// be reordered or removed again.
enum LayerId {
  // Water
  L_RAIN = 0, L_STREAM, L_WAVES,
  // Weather
  L_WIND, L_THUNDER,
  // Life
  L_LEAVES, L_BIRDS, L_CRICKETS,
  // Fire
  L_FIRE,
  // Meditative
  L_BOWL, L_CHIME,
  // Mechanical
  L_CLOCK, L_FAN,
  // Noise
  L_WHITE, L_PINK,
  NUM_LAYERS
};
#define NPARAM 3

enum LayerGroup { G_WATER = 0, G_WEATHER, G_LIFE, G_FIRE, G_MEDITATIVE, G_MECHANICAL, G_NOISE, NUM_GROUPS };
static const char* GROUP_NAMES[NUM_GROUPS] = {"Water", "Weather", "Life", "Fire", "Meditative", "Mechanical", "Noise"};
static const uint8_t LAYER_GROUP[NUM_LAYERS] = {
  G_WATER, G_WATER, G_WATER,
  G_WEATHER, G_WEATHER,
  G_LIFE, G_LIFE, G_LIFE,
  G_FIRE,
  G_MEDITATIVE, G_MEDITATIVE,
  G_MECHANICAL, G_MECHANICAL,
  G_NOISE, G_NOISE,
};

// Layers excluded from Random Scene generation: the mechanical/man-made ones
// and the plain noise generators. Random scenes are meant to sound like a
// place, and a clock or a noise floor dropped in at random rarely does.
static inline bool layerIsRandomizable(int i) {
  uint8_t g = LAYER_GROUP[i];
  return g != G_MECHANICAL && g != G_NOISE;
}
// Mechanical layers run on an exact sample-domain clock with no timing jitter.
static inline bool layerIsMechanical(int i) { return LAYER_GROUP[i] == G_MECHANICAL; }

struct LayerDef {
  const char* name;
  const char* pname[NPARAM];
  uint8_t     defP[NPARAM];
  uint8_t     defColor;
};
static const LayerDef LAYER_DEFS[NUM_LAYERS] = {
  {"Rain",     {"Intensity", "Drops",   "Tone"    }, {50, 50, 50}, 0 },
  {"Stream",   {"Flow",      "Bubbles", "Pitch"   }, {60, 50, 50}, 2 },
  {"Waves",    {"Size",      "Period",  "Foam"    }, {60, 50, 40}, 1 },
  {"Wind",     {"Strength",  "Gusts",   "Pitch"   }, {50, 50, 40}, 11},
  {"Thunder",  {"Frequency", "Distance","Power"   }, {40, 60, 55}, 10},
  {"Leaves",   {"Amount",    "Gusts",   "Size"    }, {45, 50, 45}, 3 },
  {"Birds",    {"Activity",  "Pitch",   "Variety" }, {50, 50, 50}, 4 },
  {"Crickets", {"Density",   "Pitch",   "Tempo"   }, {50, 50, 50}, 9 },
  {"Fire",     {"Intensity", "Crackle", "Tone"    }, {35, 55, 50}, 6 },
  {"Bowl",     {"Interval",  "Pitch",   "Decay"   }, {35, 45, 65}, 7 },
  {"Chime",    {"Density",   "Pitch",   "Decay"   }, {40, 50, 60}, 6 },
  {"Clock",    {"Interval",  "Tone",    "Room"    }, {50, 50, 35}, 5 },
  {"Fan",      {"Speed",     "Tone",    "Swish"   }, {45, 40, 45}, 11},
  {"White",    {"Tone",      "Drift",   "Depth"   }, {50, 30, 30}, 5 },
  {"Pink",     {"Tone",      "Drift",   "Depth"   }, {60, 30, 30}, 8 },
};

// Persisted per-layer settings (inside a scene)
struct LayerSettings {
  uint8_t vol;        // 0..100
  bool    mute;
  uint8_t color;      // palette index
  uint8_t p[NPARAM];  // 0..100 each
};

struct SceneData {
  char          name[24];
  LayerSettings L[NUM_LAYERS];
};

static void sceneSetDefaults(SceneData& s, const char* name) {
  memset(&s, 0, sizeof(s));
  strncpy(s.name, name, sizeof(s.name) - 1);
  for (int i = 0; i < NUM_LAYERS; i++) {
    s.L[i].vol = 0;
    s.L[i].mute = false;
    s.L[i].color = LAYER_DEFS[i].defColor;
    for (int k = 0; k < NPARAM; k++) s.L[i].p[k] = LAYER_DEFS[i].defP[k];
  }
}

// ---------------------------------------------------------------------------
// Preset scenes. Layers not listed stay at volume 0 with default parameters.
// ---------------------------------------------------------------------------
struct PresetLayer { int8_t layer; uint8_t vol; uint8_t p0, p1, p2; };
struct PresetDef   { const char* name; PresetLayer L[4]; };
static const PresetDef PRESETS[] = {
  {"Forest",      {{L_WIND, 28, 25, 45, 35}, {L_LEAVES, 45, 45, 50, 45}, {L_BIRDS, 55, 55, 50, 65}, {-1, 0, 0, 0, 0}}},
  {"Riverside",   {{L_STREAM, 70, 70, 60, 50}, {L_BIRDS, 35, 35, 55, 50}, {L_LEAVES, 25, 30, 45, 50}, {-1, 0, 0, 0, 0}}},
  {"Ocean",       {{L_WAVES, 75, 70, 50, 50}, {L_WIND, 35, 35, 30, 30}, {-1, 0, 0, 0, 0}, {-1, 0, 0, 0, 0}}},
  {"Rainy Day",   {{L_RAIN, 65, 60, 55, 45}, {L_WIND, 20, 30, 35, 30}, {-1, 0, 0, 0, 0}, {-1, 0, 0, 0, 0}}},
  {"Thunderstorm",{{L_RAIN, 70, 75, 60, 40}, {L_THUNDER, 60, 55, 45, 65}, {L_WIND, 30, 40, 45, 30}, {-1, 0, 0, 0, 0}}},
  {"Campfire",    {{L_FIRE, 70, 38, 60, 50}, {L_CRICKETS, 30, 35, 50, 45}, {L_LEAVES, 18, 25, 35, 45}, {-1, 0, 0, 0, 0}}},
  {"Night",       {{L_CRICKETS, 55, 50, 50, 50}, {L_WIND, 15, 20, 30, 25}, {-1, 0, 0, 0, 0}, {-1, 0, 0, 0, 0}}},
  {"Zen Garden",  {{L_CHIME, 45, 40, 50, 65}, {L_WIND, 25, 20, 30, 45}, {L_STREAM, 20, 30, 15, 60}, {-1, 0, 0, 0, 0}}},
  {"Meditation",  {{L_BOWL, 55, 30, 45, 75}, {L_WIND, 18, 18, 25, 45}, {-1, 0, 0, 0, 0}, {-1, 0, 0, 0, 0}}},
  {"Study Room",  {{L_CLOCK, 45, 50, 45, 40}, {L_RAIN, 35, 35, 45, 35}, {-1, 0, 0, 0, 0}, {-1, 0, 0, 0, 0}}},
  {"Cool Room",   {{L_FAN, 60, 45, 40, 45}, {L_PINK, 15, 40, 25, 20}, {-1, 0, 0, 0, 0}, {-1, 0, 0, 0, 0}}},
  {"Pink Noise",  {{L_PINK, 60, 60, 30, 30}, {-1, 0, 0, 0, 0}, {-1, 0, 0, 0, 0}, {-1, 0, 0, 0, 0}}},
  {"White Noise", {{L_WHITE, 60, 50, 30, 30}, {-1, 0, 0, 0, 0}, {-1, 0, 0, 0, 0}, {-1, 0, 0, 0, 0}}},
};
static const int NUM_PRESETS = sizeof(PRESETS) / sizeof(PRESETS[0]);

static void presetToScene(int idx, SceneData& s) {
  sceneSetDefaults(s, PRESETS[idx].name);
  for (int k = 0; k < 4; k++) {
    const PresetLayer& pl = PRESETS[idx].L[k];
    if (pl.layer < 0) break;
    LayerSettings& L = s.L[pl.layer];
    L.vol = pl.vol; L.p[0] = pl.p0; L.p[1] = pl.p1; L.p[2] = pl.p2;
  }
}

// ---------------------------------------------------------------------------
// Global app state shared between UI and audio
// ---------------------------------------------------------------------------
static SceneData live;                       // the scene currently playing (editable)
static bool      playing = true;
static uint16_t  soloMask = 0;               // transient (not saved)

// Values written by the UI task, read by the audio task (32-bit float writes are atomic on ESP32)
static volatile float gLayerTarget[NUM_LAYERS];
static volatile float gMasterTarget = 0.5f;
static float gAmbientScale = 1.f;   // scene level multiplier (dimmed on the beats screen)

// ---------------------------------------------------------------------------
// 2. Audio engine
// ---------------------------------------------------------------------------
#define VOICES 6
#define SINE_N 512
static float sineTab[SINE_N];
static inline float tsin(float ph) { return sineTab[((int)(ph * SINE_N)) & (SINE_N - 1)]; }
// Linearly interpolated variant. The plain lookup above truncates to one of 512
// table entries, and that phase quantisation is audible as a fine buzz on any
// long, sustained, otherwise-pure tone -- percussive and noisy layers mask it
// completely, which is why only the singing bowl needs this. Two extra
// multiply-adds, used by one layer with very few voices.
static inline float tsinL(float ph) {
  float x = ph * SINE_N;
  int i = (int)x;
  float f = x - (float)i;
  int a = i & (SINE_N - 1), b = (i + 1) & (SINE_N - 1);
  return sineTab[a] + (sineTab[b] - sineTab[a]) * f;
}

// Fast xorshift RNG used only inside the audio task
static uint32_t rngState = 0x9E3779B9u;
static inline float frand() {
  rngState ^= rngState << 13; rngState ^= rngState >> 17; rngState ^= rngState << 5;
  return (rngState & 0xFFFFFF) * (1.0f / 16777216.0f);
}
static inline float noise() { return frand() * 2.0f - 1.0f; }

static inline float onePoleCoef(float hz) {
  float c = 1.0f - expf(-6.2831853f * hz / SAMPLE_RATE);
  return c > 0.999f ? 0.999f : c;
}

// A general-purpose voice used by the event-based generators
// (rain drops, bubbles, bird syllables, chime strikes, crickets)
struct Voice {
  bool  active;
  int   pos, len;          // pos<0 acts as a start delay inside the buffer (avoids buffer-grid rhythm)
  float invLen;
  float phase, inc, incSlope;
  float amp, decay;
  float p2, p3, i2, i3, a2, a3;   // extra partials / scratch fields (meaning depends on layer)
};

struct LayerDSP {
  float gain;                       // current linear gain (smoothed toward gLayerTarget)
  float amp, ampStep, ampNext, ampTarget;   // slow amplitude modulation (gusts, swells, drift)
  float lp1, lp2, lpCoef, lpCoef2;  // one-pole low-pass states/coefs
  float svLow, svBand, svF, svF1, svQ;      // Chamberlin state-variable filter
  float pk0, pk1, pk2;              // pink noise filter states
  float timer;                      // seconds until next scheduled event
  float phase, period, peak;        // slow LFO for waves
  float acc;                        // scratch (event accumulator / foam amount)
  // Mechanical layers only: an exact sample-domain clock. Counting in whole
  // samples (rather than accumulating float seconds) is what keeps the tick
  // rock-steady with no drift and no jitter.
  uint32_t sampPos;                 // samples elapsed since this layer started
  uint32_t nextEvent;               // sampPos at which the next event fires
  uint32_t evPeriod;                // current period in samples
  int      evIndex;                 // which event within a cycle (tick/tock, clack pair)
  Voice v[VOICES];
  float level;                      // 0..1 activity indicator for the UI
};
static LayerDSP dsp[NUM_LAYERS];

static inline void slewAmp(LayerDSP& d, float k) {
  d.ampNext = d.amp + (d.ampTarget - d.amp) * k;
  d.ampStep = (d.ampNext - d.amp) / BUF;
}
static inline void svfSet(LayerDSP& d, float hz, float q) {
  float f = 2.0f * sinf(3.14159265f * hz / SAMPLE_RATE);
  if (f > 0.9f) f = 0.9f;
  d.svF1 = f; d.svQ = 1.0f / q;
}
static inline float pinkStep(LayerDSP& d) {
  float w = noise();
  d.pk0 = 0.99765f * d.pk0 + w * 0.0990460f;
  d.pk1 = 0.96300f * d.pk1 + w * 0.2965164f;
  d.pk2 = 0.57000f * d.pk2 + w * 1.0526913f;
  return (d.pk0 + d.pk1 + d.pk2 + w * 0.1848f) * 0.18f;
}
static Voice* freeVoice(LayerDSP& d) {
  Voice* best = &d.v[0];
  for (int k = 0; k < VOICES; k++) {
    if (!d.v[k].active) return &d.v[k];
    if (d.v[k].amp < best->amp) best = &d.v[k];
  }
  return best;   // steal the quietest one
}
static inline void voiceStart(Voice& v) {
  v.active = true;
  v.pos = -(int)(frand() * BUF);   // random start delay inside this buffer
  v.phase = 0; v.p2 = 0; v.p3 = 0;
  v.incSlope = 0; v.i2 = 0; v.i3 = 0; v.a2 = 0; v.a3 = 0;
}

// ----- Rain ---------------------------------------------------------------
static void trigDrop(LayerDSP& d, float tone) {
  Voice& v = *freeVoice(d); voiceStart(v);
  float f = (900.f + 2200.f * frand()) * (0.6f + 0.8f * tone);
  v.inc = f / SAMPLE_RATE;
  v.amp = 0.06f + 0.18f * frand();
  v.decay = expf(-1.0f / (SAMPLE_RATE * (0.003f + 0.012f * frand())));
}
static void tickRain(LayerDSP& d, const LayerSettings& s) {
  float inten = s.p[0] * 0.01f, dens = s.p[1] * 0.01f, tone = s.p[2] * 0.01f;
  d.timer -= DT;
  if (d.timer <= 0) { d.timer = 2.0f + 4.0f * frand(); d.ampTarget = inten * (0.5f + 0.5f * frand()); }
  slewAmp(d, 0.08f);
  d.lpCoef = onePoleCoef(500.f + 3500.f * tone);
  float rate = dens * dens * 60.f * (0.3f + 0.7f * inten);   // drops per second
  d.acc += rate * DT;
  while (d.acc >= 1.f) { d.acc -= 1.f; if (frand() < 0.85f) trigDrop(d, tone); }
  if (frand() < rate * DT * 0.3f) trigDrop(d, tone);
}
static inline float smpRain(LayerDSP& d) {
  float n = noise();
  d.lp1 += d.lpCoef * (n - d.lp1);
  float o = d.lp1 * d.amp * 1.6f;
  d.amp += d.ampStep;
  for (int k = 0; k < VOICES; k++) {
    Voice& v = d.v[k];
    if (!v.active) continue;
    if (v.pos < 0) { v.pos++; continue; }
    o += tsin(v.phase) * v.amp;
    v.phase += v.inc; if (v.phase >= 1.f) v.phase -= 1.f;
    v.amp *= v.decay; if (v.amp < 0.0005f) v.active = false;
  }
  return o;
}

// ----- Wind ---------------------------------------------------------------
static void tickWind(LayerDSP& d, const LayerSettings& s) {
  float str = s.p[0] * 0.01f, gust = s.p[1] * 0.01f, pitch = s.p[2] * 0.01f;
  d.timer -= DT;
  if (d.timer <= 0) {
    d.timer = (1.0f + 7.0f * (1.f - gust)) * (0.4f + 1.2f * frand());
    float r = frand();
    d.ampTarget = str * (0.12f + 0.88f * r * r);
  }
  slewAmp(d, 0.02f + 0.03f * gust);
  float norm = str > 0.01f ? d.amp / str : 0.f;
  float fc = (120.f + 520.f * pitch) * (0.75f + 0.7f * norm);
  d.svF += (fc - d.svF) * 0.15f;
  svfSet(d, d.svF, 2.5f);
  d.lpCoef = onePoleCoef(150.f);
}
static inline float smpWind(LayerDSP& d) {
  float p = pinkStep(d);
  d.svLow += d.svF1 * d.svBand;
  float hi = p - d.svLow - d.svQ * d.svBand;
  d.svBand += d.svF1 * hi;
  d.lp1 += d.lpCoef * (p - d.lp1);
  float o = (d.svBand * 1.8f + d.lp1 * 1.2f) * d.amp * 1.4f;
  d.amp += d.ampStep;
  return o;
}

// ----- Waves --------------------------------------------------------------
static void tickWaves(LayerDSP& d, const LayerSettings& s) {
  float size = s.p[0] * 0.01f, period = s.p[1] * 0.01f, foam = s.p[2] * 0.01f;
  if (d.period <= 0.f) { d.period = 5.f + 12.f * period; d.peak = size; }
  d.phase += DT / d.period;
  if (d.phase >= 1.f) {
    d.phase -= 1.f;
    d.period = (5.f + 12.f * period) * (0.75f + 0.5f * frand());
    d.peak = size * (0.55f + 0.45f * frand());
  }
  float ph = d.phase;
  float w = ph < 0.4f ? ph / 0.4f * 0.5f : 0.5f + (ph - 0.4f) / 0.6f * 0.5f;   // faster rise, slower fall
  float sn = sinf(3.14159265f * w);
  float env = d.peak * (0.12f + 0.88f * sn * sn);
  d.ampTarget = env;
  slewAmp(d, 0.5f);
  d.lpCoef  = onePoleCoef(250.f + 900.f * sn * sn);
  d.lpCoef2 = onePoleCoef(2500.f);
  d.acc = foam * sn * sn * sn * sn * d.peak;   // foam amount (hiss at the crest)
}
static inline float smpWaves(LayerDSP& d) {
  float n = noise();
  d.lp1 += d.lpCoef * (n - d.lp1);
  d.lp2 += d.lpCoef2 * (n - d.lp2);
  float hp = n - d.lp2;
  float o = d.lp1 * d.amp * 2.2f + hp * d.acc * 0.25f;
  d.amp += d.ampStep;
  return o;
}

// ----- Stream -------------------------------------------------------------
static void trigBubble(LayerDSP& d, float pitch) {
  Voice& v = *freeVoice(d); voiceStart(v);
  v.len = (int)(SAMPLE_RATE * (0.025f + 0.07f * frand())); v.invLen = 1.0f / v.len;
  float f0 = (250.f + 450.f * pitch) * (0.7f + 0.6f * frand());
  float f1 = f0 * (1.5f + 1.0f * frand());
  v.inc = f0 / SAMPLE_RATE; v.incSlope = (f1 - f0) / SAMPLE_RATE / v.len;
  v.amp = 0.06f + 0.12f * frand();
}
static void tickStream(LayerDSP& d, const LayerSettings& s) {
  float flow = s.p[0] * 0.01f, bub = s.p[1] * 0.01f, pitch = s.p[2] * 0.01f;
  d.ampTarget = flow * (0.65f + 0.35f * frand());
  slewAmp(d, 0.6f);
  float fc = (700.f + 1800.f * pitch) * (0.9f + 0.2f * frand());
  svfSet(d, fc, 1.3f);
  d.lpCoef = onePoleCoef(400.f);
  float rate = bub * bub * 14.f + bub * 2.f;
  d.acc += rate * DT;
  while (d.acc >= 1.f) { d.acc -= 1.f; if (frand() < 0.8f) trigBubble(d, pitch); }
}
static inline float sweepVoices(LayerDSP& d) {
  float o = 0;
  for (int k = 0; k < VOICES; k++) {
    Voice& v = d.v[k];
    if (!v.active) continue;
    if (v.pos < 0) { v.pos++; continue; }
    float t = v.pos * v.invLen;
    float env = 4.f * t * (1.f - t);
    o += tsin(v.phase) * env * v.amp;
    v.phase += v.inc; if (v.phase >= 1.f) v.phase -= 1.f;
    v.inc += v.incSlope;
    if (++v.pos >= v.len) v.active = false;
  }
  return o;
}
static inline float smpStream(LayerDSP& d) {
  float n = noise();
  d.svLow += d.svF1 * d.svBand;
  float hi = n - d.svLow - d.svQ * d.svBand;
  d.svBand += d.svF1 * hi;
  d.lp1 += d.lpCoef * (n - d.lp1);
  float o = (d.svBand * 1.5f + d.lp1 * 0.5f) * d.amp;
  d.amp += d.ampStep;
  return o + sweepVoices(d);
}

// ----- Birds --------------------------------------------------------------
// Voice scratch fields: p2 = syllables left in phrase, p3 = seconds until next syllable,
// a2 = phrase base frequency, a3 = phrase style (0..1)
static void startSyllable(Voice& v, float var) {
  float baseF = v.a2, style = v.a3;
  voiceStart(v);
  v.a2 = baseF; v.a3 = style;
  v.pos = -(int)(frand() * BUF * 0.5f);
  v.len = (int)(SAMPLE_RATE * (0.03f + 0.10f * frand() * (0.5f + var))); v.invLen = 1.0f / v.len;
  float f0 = baseF * (0.85f + 0.3f * frand());
  float r = frand(), f1;
  if      (r < 0.4f + 0.3f * style) f1 = f0 * (1.2f + 0.8f * frand());   // upward sweep
  else if (r < 0.8f)                f1 = f0 * (0.55f + 0.3f * frand());  // downward sweep
  else                              f1 = f0 * (0.97f + 0.06f * frand()); // flat tone
  v.inc = f0 / SAMPLE_RATE; v.incSlope = (f1 - f0) / SAMPLE_RATE / v.len;
  v.amp = 0.08f + 0.12f * frand();
  v.p2 -= 1;
  v.p3 = (float)v.len / SAMPLE_RATE + 0.03f + 0.15f * frand();
}
static void tickBirds(LayerDSP& d, const LayerSettings& s) {
  float act = s.p[0] * 0.01f, pitch = s.p[1] * 0.01f, var = s.p[2] * 0.01f;
  d.timer -= DT;
  if (d.timer <= 0) {
    d.timer = (0.6f + 6.f * (1.f - act)) * (0.3f + 1.4f * frand());
    if (act > 0.01f) {
      for (int k = 0; k < VOICES; k++) {
        Voice& v = d.v[k];
        if (!v.active && v.p2 <= 0) {
          v.p2 = 1 + (int)(frand() * (1.5f + 5.f * var));
          v.a2 = (1600.f + 3000.f * pitch) * (0.75f + 0.5f * frand());
          v.a3 = frand();
          v.p3 = 0;
          break;
        }
      }
    }
  }
  for (int k = 0; k < VOICES; k++) {
    Voice& v = d.v[k];
    if (!v.active && v.p2 > 0) { v.p3 -= DT; if (v.p3 <= 0) startSyllable(v, var); }
  }
}
static inline float smpBirds(LayerDSP& d) { return sweepVoices(d); }

// ----- Wind chime (furin) -------------------------------------------------
static void trigChime(LayerDSP& d, float pitch, float dec) {
  static const float RATIOS[6] = {1.0f, 1.125f, 1.25f, 1.5f, 1.6875f, 2.0f};   // pentatonic
  Voice& v = *freeVoice(d); voiceStart(v);
  float root = 1400.f + 1600.f * pitch;
  float f = root * RATIOS[(int)(frand() * 6) % 6] * (0.995f + 0.01f * frand());
  v.inc = f / SAMPLE_RATE; v.i2 = v.inc * 2.71f; v.i3 = v.inc * 5.15f;   // inharmonic glass partials
  v.a2 = 0.45f; v.a3 = 0.2f;
  v.amp = 0.10f + 0.10f * frand();
  float T = 0.4f + 2.6f * dec;
  v.decay = expf(-1.0f / (SAMPLE_RATE * T));
}
static void tickChime(LayerDSP& d, const LayerSettings& s) {
  float dens = s.p[0] * 0.01f, pitch = s.p[1] * 0.01f, dec = s.p[2] * 0.01f;
  // Strikes become more frequent while the Wind layer is gusting
  float wl = 0.5f;
  if (dsp[L_WIND].gain > 0.01f) {
    float st = live.L[L_WIND].p[0] * 0.01f;
    wl = st > 0.01f ? dsp[L_WIND].amp / st : 0.f;
  }
  float rate = dens * dens * 3.0f * (0.25f + 1.5f * wl);
  if (frand() < rate * DT) {
    trigChime(d, pitch, dec);
    if (frand() < 0.35f) trigChime(d, pitch, dec);   // occasional double strike
  }
}
static inline float smpChime(LayerDSP& d) {
  float o = 0;
  for (int k = 0; k < VOICES; k++) {
    Voice& v = d.v[k];
    if (!v.active) continue;
    if (v.pos < 0) { v.pos++; continue; }
    o += (tsin(v.phase) + tsin(v.p2) * v.a2 + tsin(v.p3) * v.a3) * v.amp;
    v.phase += v.inc; if (v.phase >= 1.f) v.phase -= 1.f;
    v.p2 += v.i2; if (v.p2 >= 1.f) v.p2 -= 1.f;
    v.p3 += v.i3; if (v.p3 >= 1.f) v.p3 -= 1.f;
    v.amp *= v.decay; if (v.amp < 0.0003f) v.active = false;
  }
  return o;
}

// ----- Crickets -----------------------------------------------------------
// Two crickets (v[0], v[1]). active = currently chirping burst, p3 = burst/pause timer,
// inc = carrier increment, i2 = pulse increment, a2 = amplitude at end of buffer, a3 = per-sample step
static void tickCrickets(LayerDSP& d, const LayerSettings& s) {
  float dens = s.p[0] * 0.01f, pitch = s.p[1] * 0.01f, tempo = s.p[2] * 0.01f;
  int nCr = dens < 0.02f ? 0 : (dens < 0.35f ? 1 : 2);
  for (int k = 0; k < 2; k++) {
    Voice& v = d.v[k];
    v.amp = v.a2;
    v.p3 -= DT;
    if (v.p3 <= 0) {
      if (v.active) {
        v.active = false;
        v.p3 = (0.8f + 2.5f * (1.f - tempo) * (0.5f + frand())) * (1.2f - dens * 0.6f);
      } else {
        v.active = true;
        v.p3 = 0.25f + 0.5f * frand();
        v.inc = (3600.f + 2400.f * pitch) * (1.f + (k ? 0.03f : -0.01f)) / SAMPLE_RATE;
        v.i2  = (22.f + 28.f * tempo) * (k ? 1.1f : 1.f) / SAMPLE_RATE;
      }
    }
    float target = (k < nCr && v.active) ? dens * 0.14f + 0.04f : 0.f;
    float next = v.amp + (target - v.amp) * 0.5f;
    v.a3 = (next - v.amp) / BUF;
    v.a2 = next;
  }
}
static inline float smpCrickets(LayerDSP& d) {
  float o = 0;
  for (int k = 0; k < 2; k++) {
    Voice& v = d.v[k];
    float pulse = tsin(v.p2);
    pulse = pulse > 0.3f ? (pulse - 0.3f) * 1.43f : 0.f;
    o += tsin(v.phase) * pulse * v.amp;
    v.phase += v.inc; if (v.phase >= 1.f) v.phase -= 1.f;
    v.p2 += v.i2;     if (v.p2 >= 1.f) v.p2 -= 1.f;
    v.amp += v.a3;
  }
  return o;
}

// ----- White / Pink noise -------------------------------------------------
static void tickNoise(LayerDSP& d, const LayerSettings& s) {
  float tone = s.p[0] * 0.01f, drift = s.p[1] * 0.01f, depth = s.p[2] * 0.01f;
  d.timer -= DT;
  if (d.timer <= 0) { d.timer = (0.5f + 6.f * (1.f - drift)) * (0.5f + frand()); d.ampTarget = 1.f - depth * 0.6f * frand(); }
  slewAmp(d, 0.05f);
  d.lpCoef = onePoleCoef(300.f + 6000.f * tone);
  d.acc = 0.7f + (1.f - tone) * 0.6f;   // gain compensation for a darker tone
}
static inline float smpWhite(LayerDSP& d) {
  float n = noise();
  d.lp1 += d.lpCoef * (n - d.lp1);
  float o = d.lp1 * d.amp * d.acc;
  d.amp += d.ampStep;
  return o;
}
static inline float smpPink(LayerDSP& d) {
  float p = pinkStep(d);
  d.lp1 += d.lpCoef * (p - d.lp1);
  float o = d.lp1 * d.amp * d.acc * 1.3f;
  d.amp += d.ampStep;
  return o;
}

// ----- Shared percussive engine for the mechanical layers ------------------
// A real tick or rail-joint clack is a broadband impact that excites a couple
// of resonances -- it is NOT a tone. An earlier attempt built these from sine
// partials and came out sounding like a relay / car indicator, which is
// exactly what summed sines do. So these voices are instead a short noise
// burst feeding two parallel 2-pole resonators, plus some of the raw burst
// left in for the attack transient.
//
// Field reuse inside Voice (no struct changes needed):
//   p2, p3         resonator A state (y[n-1], y[n-2])
//   i2, i3         resonator A coefficients (2*r*cos w, r^2)
//   amp            resonator A output gain
//   phase, inc     resonator B state
//   incSlope, invLen  resonator B coefficients
//   decay          resonator B output gain
//   a2, a3         noise burst level and its per-sample decay
//   len            hard cut-off length in samples (safety net)
// Returns the gain that normalises this resonator for an impulse-like hit.
//
// Getting this wrong twice is what made both mechanical sounds misbehave, so
// it is worth stating plainly. The impulse response of this 2-pole resonator
// is r^n * sin((n+1)w) / sin(w), so its PEAK is about 1/sin(w) -- it depends on
// the frequency, not on the decay time. Two earlier versions got this wrong in
// opposite directions:
//   * dividing by (1-r) normalises for a *sustained* input, which these are
//     not. It made long-decay resonators almost silent -- the train's 0.17 s
//     body came out ~30x quieter than the clock's 6 ms click, so the clacks
//     were inaudible and only the noise bed was left.
//   * not normalising at all let low resonators explode instead: at 72 Hz,
//     1/sin(w) is about 44, so a modest hit slammed into the limiter.
// Scaling by sin(w) is the correct answer for both, and makes the per-hit
// gains below mean what they look like.
static inline float resSet(float freq, float decaySec, float& coef, float& r2) {
  float r = expf(-1.0f / (SAMPLE_RATE * decaySec));
  if (r > 0.9995f) r = 0.9995f;
  float w = 6.2831853f * freq / SAMPLE_RATE;
  if (w > 3.0f) w = 3.0f;                 // keep well below Nyquist
  coef = 2.0f * r * cosf(w);
  r2 = r * r;
  return sinf(w);
}
static Voice* freeResVoice(LayerDSP& d) {
  Voice* oldest = &d.v[0];
  for (int k = 0; k < VOICES; k++) {
    if (!d.v[k].active) return &d.v[k];
    if (d.v[k].pos > oldest->pos) oldest = &d.v[k];
  }
  return oldest;   // steal the one that has been ringing longest
}
static void trigRes(LayerDSP& d, int offsetInBuf,
                    float fA, float decA, float gA,
                    float fB, float decB, float gB,
                    float burst, float burstSec, float maxSec) {
  Voice& v = *freeResVoice(d);
  v.active = true;
  v.pos = -offsetInBuf;                   // exact sample offset -- no randomness
  v.p2 = v.p3 = 0.f;
  v.phase = v.inc = 0.f;
  v.amp   = gA * resSet(fA, decA, v.i2, v.i3);
  v.decay = gB * resSet(fB, decB, v.incSlope, v.invLen);
  v.a2 = burst;
  v.a3 = expf(-1.0f / (SAMPLE_RATE * burstSec));
  v.len = (int)(SAMPLE_RATE * maxSec);
}
static inline float smpRes(LayerDSP& d, float rawMix) {
  float o = 0;
  for (int k = 0; k < VOICES; k++) {
    Voice& v = d.v[k];
    if (!v.active) continue;
    if (v.pos < 0) { v.pos++; continue; }
    float x = noise() * v.a2;
    v.a2 *= v.a3;
    float yA = x + v.i2 * v.p2 - v.i3 * v.p3;              v.p3 = v.p2; v.p2 = yA;
    float yB = x + v.incSlope * v.phase - v.invLen * v.inc; v.inc = v.phase; v.phase = yB;
    o += yA * v.amp + yB * v.decay + x * rawMix;
    if (++v.pos >= v.len) v.active = false;
  }
  return o;
}

// ----- Clock (mechanical, exact timing) -----------------------------------
static void tickClock(LayerDSP& d, const LayerSettings& s) {
  float interval = s.p[0] * 0.01f, tone = s.p[1] * 0.01f, room = s.p[2] * 0.01f;
  // 50 lands on exactly 1.000 s -- a real second hand.
  float sec = interval <= 0.5f ? 2.5f + (1.0f - 2.5f) * (interval / 0.5f)
                               : 1.0f + (0.3f - 1.0f) * ((interval - 0.5f) / 0.5f);
  d.evPeriod = (uint32_t)(sec * SAMPLE_RATE + 0.5f);
  if (d.evPeriod < 64) d.evPeriod = 64;
  if (d.nextEvent < d.sampPos) d.nextEvent = d.sampPos;   // first run / after a reset
  uint32_t bufEnd = d.sampPos + BUF;
  while (d.nextEvent < bufEnd) {
    bool tock = (d.evIndex & 1) != 0;
    // Tick and tock differ slightly in pitch, the way a real escapement does.
    float m = tock ? 0.84f : 1.0f;
    // Click resonance + a lower wooden/case body. "Room" lengthens the body's
    // ring, suggesting a larger, emptier space.
    // v0.51: pitched down and shortened from the first resonator version --
    // "harder and lower" reads as a lower centre frequency plus a FASTER decay,
    // since a long ring is what makes an impact sound soft rather than hard.
    float fA = (1050.f + 1400.f * tone) * m;
    float fB = (300.f + 380.f * tone) * m;
    trigRes(d, (int)(d.nextEvent - d.sampPos),
            fA, 0.0035f + 0.0060f * room, 0.30f,
            fB, 0.009f + 0.050f * room, 0.45f,
            1.0f, 0.0007f, 0.08f + 0.20f * room);
    d.evIndex++;
    d.nextEvent += d.evPeriod;   // scheduled from the previous event, so error never accumulates
  }
  d.sampPos = bufEnd;
}
static inline float smpClock(LayerDSP& d) {
  return smpRes(d, 0.14f) * 0.55f;
}

// ----- Fire -----------------------------------------------------------------
// Deliberately built WITHOUT resonators. The train layer failed because a
// pitched resonator makes any impact sound like a drum; a fire crackle has no
// pitch at all, so it is pure filtered noise. What makes it read as a fire is
// the *distribution*: a great many tiny ticks, a few larger pops, and both
// arriving in clusters rather than evenly.
//
// Voice fields here: amp/decay = level and ring-down, phase = one-pole
// low-pass state, inc = its coefficient (each pop gets its own brightness).
static void trigCrackle(LayerDSP& d, float tone, bool big) {
  Voice& v = *freeResVoice(d);
  v.active = true;
  v.pos = -(int)(frand() * BUF);      // scatter within the buffer
  v.len = (int)(SAMPLE_RATE * (big ? 0.12f : 0.02f));
  v.phase = 0.f;
  if (big) {
    v.amp = 0.30f + 0.40f * frand();
    v.decay = expf(-1.0f / (SAMPLE_RATE * (0.008f + 0.020f * frand())));
    v.inc = onePoleCoef((900.f + 1800.f * frand()) * (0.5f + tone));
  } else {
    v.amp = 0.07f + 0.15f * frand();
    v.decay = expf(-1.0f / (SAMPLE_RATE * (0.0008f + 0.0025f * frand())));
    v.inc = onePoleCoef((2800.f + 5000.f * frand()) * (0.5f + tone));
  }
}
static void tickFire(LayerDSP& d, const LayerSettings& s) {
  float inten = s.p[0] * 0.01f, crackle = s.p[1] * 0.01f, tone = s.p[2] * 0.01f;
  // Body of the flame: a low roar that breathes
  d.timer -= DT;
  if (d.timer <= 0) {
    d.timer = 0.25f + 1.4f * frand();
    // Squared, and much quieter overall. In v0.6 the flame bed was usable only
    // around Intensity 2-4 out of 100 -- the rest of the dial was a wall of
    // hiss. Squaring the control and cutting the gain puts the useful range
    // back in the middle of the knob, with the top of the dial a genuine roar
    // rather than something unusable.
    d.ampTarget = inten * inten * (0.5f + 0.5f * frand());
  }
  slewAmp(d, 0.09f);
  d.lpCoef  = onePoleCoef(110.f + 220.f * tone);          // deep roar
  d.lpCoef2 = onePoleCoef(700.f + 1500.f * tone);         // upper breath
  // Crackle clustering: the rate itself wanders, so pops bunch up and then
  // thin out instead of arriving at a steady average.
  d.period -= DT;
  if (d.period <= 0.f) {
    d.period = 0.35f + 1.8f * frand();
    float r = frand();
    d.peak = 0.15f + 2.6f * r * r;                        // burst multiplier
  }
  float rate = crackle * 46.f * d.peak * (0.75f + 0.25f * inten);
  d.acc += rate * DT;
  while (d.acc >= 1.f) {
    d.acc -= 1.f;
    trigCrackle(d, tone, frand() < 0.055f);               // roughly 1 in 18 is a big pop
  }
}
static inline float smpFire(LayerDSP& d) {
  float n = noise();
  d.lp1 += d.lpCoef * (n - d.lp1);
  d.lp2 += d.lpCoef2 * (n - d.lp2);
  float o = (d.lp1 * 0.52f + d.lp2 * 0.10f) * d.amp;
  d.amp += d.ampStep;
  for (int k = 0; k < VOICES; k++) {
    Voice& v = d.v[k];
    if (!v.active) continue;
    if (v.pos < 0) { v.pos++; continue; }
    float x = noise() * v.amp;
    v.phase += v.inc * (x - v.phase);                     // per-pop low-pass
    o += v.phase;
    v.amp *= v.decay;
    if (++v.pos >= v.len || v.amp < 0.0004f) v.active = false;
  }
  return o;
}

// ----- Thunder ---------------------------------------------------------------
// Rare, long events rather than a continuous texture, so this runs off the
// layer's own state instead of the voice pool. Two cascaded one-poles give a
// steeper roll-off than one, and the cutoff falls as the peal rolls away --
// which is what actually makes thunder read as distant.
static void tickThunder(LayerDSP& d, const LayerSettings& s) {
  float freq = s.p[0] * 0.01f, dist = s.p[1] * 0.01f, power = s.p[2] * 0.01f;
  if (d.period <= 0.f) {
    // Waiting for the next strike
    d.timer -= DT;
    if (d.timer <= 0.f) {
      d.period = 2.2f + 4.5f * dist + 2.0f * frand();     // farther away -> longer roll
      d.acc = 0.f;                                        // elapsed time within the peal
      d.peak = power * (0.45f + 0.55f * frand());
      d.phase = 0.f;
    }
    d.ampTarget = 0.f;
    slewAmp(d, 0.2f);
    d.lpCoef = onePoleCoef(400.f);
    d.lpCoef2 = onePoleCoef(400.f);
    return;
  }
  d.acc += DT;
  float t = d.acc / d.period;                             // 0..1 through the peal
  if (t >= 1.f) {
    d.period = 0.f;
    // Next strike: frequent setting -> every few seconds, sparse -> a minute+
    d.timer = (4.f + 70.f * (1.f - freq)) * (0.35f + 1.3f * frand());
    d.ampTarget = 0.f;
    slewAmp(d, 0.2f);
    return;
  }
  // A near strike cracks open quickly; a distant one swells slowly.
  float attack = 0.012f + 0.42f * dist;
  float env;
  if (t < attack) env = t / attack;
  else {
    float u = (t - attack) / (1.f - attack);
    env = (1.f - u) * (1.f - u);                          // long tapering roll
  }
  // Rumble: a slow wander layered on the decay, so the peal surges and sags
  d.phase += DT * (0.7f + 1.6f * frand());
  env *= 0.55f + 0.45f * (0.5f + 0.5f * sinf(d.phase * 3.1f));
  d.ampTarget = d.peak * env;
  slewAmp(d, 0.35f);
  // Brightest at the strike, then progressively darker as it rolls away
  float fc = (900.f - 700.f * dist) * (1.f - 0.62f * t) + 70.f;
  d.lpCoef  = onePoleCoef(fc);
  d.lpCoef2 = onePoleCoef(fc * 1.6f);
}
static inline float smpThunder(LayerDSP& d) {
  float p = pinkStep(d);
  d.lp1 += d.lpCoef2 * (p - d.lp1);
  d.lp2 += d.lpCoef * (d.lp1 - d.lp2);                    // two poles in series
  float o = d.lp2 * d.amp * 5.4f;
  d.amp += d.ampStep;
  return o;
}

// ----- Leaves ----------------------------------------------------------------
// Rustling foliage: band-passed noise whose level is driven by the same kind of
// gust envelope the Wind layer uses, plus a sprinkle of individual leaf ticks.
// Like Chime, this follows the Wind layer when one is playing, so a scene with
// both moves as one thing rather than two.
static void tickLeaves(LayerDSP& d, const LayerSettings& s) {
  float amount = s.p[0] * 0.01f, gust = s.p[1] * 0.01f, size = s.p[2] * 0.01f;
  float windLvl = -1.f;
  if (dsp[L_WIND].gain > 0.01f) {
    float st = live.L[L_WIND].p[0] * 0.01f;
    windLvl = st > 0.01f ? dsp[L_WIND].amp / st : 0.f;
  }
  d.timer -= DT;
  if (d.timer <= 0.f) {
    d.timer = (0.5f + 3.2f * (1.f - gust)) * (0.4f + 1.2f * frand());
    float r = frand();
    d.ampTarget = amount * (0.10f + 0.90f * r * r);
  }
  // When Wind is present its gusts lead; the layer's own envelope just adds
  // a little independent life on top.
  if (windLvl >= 0.f) d.ampTarget = amount * (0.12f + 0.88f * windLvl) * (0.75f + 0.35f * frand());
  slewAmp(d, 0.05f + 0.06f * gust);
  // Bigger leaves rustle lower; the band is wide because foliage is broadband
  svfSet(d, 1500.f + 3200.f * (1.f - size), 0.85f);
  d.lpCoef = onePoleCoef(700.f);
  float drive = windLvl >= 0.f ? windLvl : (amount > 0.01f ? d.amp / amount : 0.f);
  d.acc += (4.f + 46.f * drive) * amount * DT;            // individual leaf ticks
  while (d.acc >= 1.f) {
    d.acc -= 1.f;
    Voice& v = *freeResVoice(d);
    v.active = true;
    v.pos = -(int)(frand() * BUF);
    v.len = (int)(SAMPLE_RATE * 0.02f);
    v.phase = 0.f;
    v.amp = 0.02f + 0.05f * frand();
    v.decay = expf(-1.0f / (SAMPLE_RATE * (0.0012f + 0.0035f * frand())));
    v.inc = onePoleCoef(2200.f + 5200.f * frand() * (1.2f - size));
  }
}
static inline float smpLeaves(LayerDSP& d) {
  float n = noise();
  d.svLow += d.svF1 * d.svBand;
  float hi = n - d.svLow - d.svQ * d.svBand;
  d.svBand += d.svF1 * hi;
  d.lp1 += d.lpCoef * (n - d.lp1);
  float o = (d.svBand * 1.05f + (n - d.lp1) * 0.16f) * d.amp;
  d.amp += d.ampStep;
  for (int k = 0; k < VOICES; k++) {
    Voice& v = d.v[k];
    if (!v.active) continue;
    if (v.pos < 0) { v.pos++; continue; }
    float x = noise() * v.amp;
    v.phase += v.inc * (x - v.phase);
    o += v.phase;
    v.amp *= v.decay;
    if (++v.pos >= v.len || v.amp < 0.0003f) v.active = false;
  }
  return o;
}

// ----- Singing bowl ----------------------------------------------------------
// Here a tonal treatment is right rather than wrong: a struck bowl IS a pitch,
// and its character comes from two partials a couple of Hz apart beating
// against each other, which is why the tone seems to breathe. Built from sines
// (not resonators) because the ring lasts many seconds, far longer than a
// stable 2-pole resonator will hold at this sample rate.
//
// Voice fields: phase/inc + p2/i2 = the two beating fundamentals, p3/i3 = an
// upper inharmonic partial, amp/decay = main envelope, a2/a3 = upper partial
// level and its faster decay.
static void trigBowl(LayerDSP& d, float pitch, float dec) {
  Voice& v = *freeResVoice(d);
  v.active = true;
  v.pos = -(int)(frand() * BUF);
  v.len = 0x7FFFFFFF;                        // ends when the envelope dies away
  // All three partials start at phase 0. v0.6 started the second one at 0.25,
  // i.e. at sin = 1.0 -- so the waveform jumped from silence to full scale on
  // the first sample, which is exactly the "click" on each strike.
  v.phase = 0.f; v.p2 = 0.f; v.p3 = 0.f;
  float f = 170.f + 330.f * pitch;
  float beat = 0.7f + 2.6f * frand();        // Hz between the two partials
  v.inc = f / SAMPLE_RATE;
  v.i2  = (f + beat) / SAMPLE_RATE;
  v.i3  = f * (2.71f + 0.16f * frand()) / SAMPLE_RATE;
  float T = 2.5f + 14.f * dec;
  // Lower than v0.6: two partials at full amplitude plus the shimmer could sum
  // past 1.0 on the strike and run into the soft limiter, which is harshness
  // of its own on top of the click.
  v.amp = 0.17f + 0.07f * frand();
  v.decay = expf(-1.0f / (SAMPLE_RATE * T));
  v.a2 = v.amp * 0.38f;
  v.a3 = expf(-1.0f / (SAMPLE_RATE * T * 0.22f));   // the shimmer fades first
  // Short attack ramp (invLen = current gain, incSlope = its step). Even with
  // the phases fixed, an instant 0 -> full envelope is itself a transient.
  v.invLen = 0.f;
  v.incSlope = 1.0f / (SAMPLE_RATE * 0.012f);
}
static void tickBowl(LayerDSP& d, const LayerSettings& s) {
  float interval = s.p[0] * 0.01f, pitch = s.p[1] * 0.01f, dec = s.p[2] * 0.01f;
  d.timer -= DT;
  if (d.timer <= 0.f) {
    // 8 s at the busy end out to a couple of minutes at the sparse end
    d.timer = (8.f + 105.f * (1.f - interval)) * (0.55f + 0.9f * frand());
    trigBowl(d, pitch, dec);
  }
}
static inline float smpBowl(LayerDSP& d) {
  float o = 0;
  for (int k = 0; k < VOICES; k++) {
    Voice& v = d.v[k];
    if (!v.active) continue;
    if (v.pos < 0) { v.pos++; continue; }
    float g = v.invLen;
    if (g < 1.f) { v.invLen += v.incSlope; if (v.invLen > 1.f) v.invLen = 1.f; }
    // The second partial sits lower than the first on purpose. With both at
    // equal level the beat nulls were TOTAL cancellation, dropping the signal
    // to nothing a few times a second -- musically a bit dead, and it parked
    // the waveform right in the worst region for quantisation. At 0.72 the
    // null only falls to about a quarter of peak, so the tone still breathes
    // but never collapses.
    o += ((tsinL(v.phase) + tsinL(v.p2) * 0.72f) * v.amp + tsinL(v.p3) * v.a2) * g;
    // A touch of mallet contact. v0.6 ran raw white noise for 900 samples
    // (41 ms) at 0.05, which reads as a spitty "tss" rather than a soft strike;
    // now it is a fifth of the length and much quieter, shaped by the same
    // attack ramp.
    if (v.pos < 180) o += noise() * 0.014f * (1.f - v.pos * (1.f / 180.f));
    v.phase += v.inc; if (v.phase >= 1.f) v.phase -= 1.f;
    v.p2 += v.i2;     if (v.p2 >= 1.f) v.p2 -= 1.f;
    v.p3 += v.i3;     if (v.p3 >= 1.f) v.p3 -= 1.f;
    v.amp *= v.decay;
    v.a2 *= v.a3;
    v.pos++;
    if (v.amp < 0.0002f) v.active = false;
  }
  return o;
}

// ----- Fan / air conditioner (mechanical, exact timing) -------------------
// Steady broadband air noise amplitude-modulated at the blade-pass rate, plus
// a faint tonal hum at that rate. The modulation phase advances by a fixed
// per-sample increment, so it is exactly periodic by construction.
static void tickFan(LayerDSP& d, const LayerSettings& s) {
  float speed = s.p[0] * 0.01f, tone = s.p[1] * 0.01f, depth = s.p[2] * 0.01f;
  float rotHz = 5.f + 16.f * speed;
  float bladeHz = rotHz * 5.f;                             // five blades
  d.period = bladeHz / SAMPLE_RATE;                        // per-sample phase increment
  d.peak = depth;
  d.acc = 0.10f + 0.30f * depth;                           // tonal hum level
  d.lpCoef  = onePoleCoef(700.f + 4200.f * tone);
  d.lpCoef2 = onePoleCoef(110.f + 180.f * speed);
  d.ampTarget = 1.f;
  slewAmp(d, 0.2f);
}
static inline float smpFan(LayerDSP& d) {
  float n = noise();
  d.lp1 += d.lpCoef * (n - d.lp1);
  d.lp2 += d.lpCoef2 * (n - d.lp2);
  float m = tsin(d.phase);
  d.phase += d.period; if (d.phase >= 1.f) d.phase -= 1.f;
  float mod = 1.f - d.peak * 0.45f * (1.f - m);            // blade-pass swish
  float o = (d.lp1 * 0.90f + d.lp2 * 1.30f) * mod + m * d.acc * 0.17f;
  o *= d.amp;
  d.amp += d.ampStep;
  return o;
}

// ----- Dispatch -----------------------------------------------------------
static void layerTick(int i) {
  LayerDSP& d = dsp[i]; const LayerSettings& s = live.L[i];
  switch (i) {
    case L_RAIN:     tickRain(d, s); break;
    case L_WIND:     tickWind(d, s); break;
    case L_WAVES:    tickWaves(d, s); break;
    case L_STREAM:   tickStream(d, s); break;
    case L_BIRDS:    tickBirds(d, s); break;
    case L_CHIME:    tickChime(d, s); break;
    case L_CRICKETS: tickCrickets(d, s); break;
    case L_WHITE:    tickNoise(d, s); break;
    case L_PINK:     tickNoise(d, s); break;
    case L_CLOCK:    tickClock(d, s); break;
    case L_FAN:      tickFan(d, s); break;
    case L_THUNDER:  tickThunder(d, s); break;
    case L_LEAVES:   tickLeaves(d, s); break;
    case L_FIRE:     tickFire(d, s); break;
    case L_BOWL:     tickBowl(d, s); break;
  }
}
static inline float layerSample(int i, LayerDSP& d) {
  switch (i) {
    case L_RAIN:     return smpRain(d);
    case L_WIND:     return smpWind(d);
    case L_WAVES:    return smpWaves(d);
    case L_STREAM:   return smpStream(d);
    case L_BIRDS:    return smpBirds(d);
    case L_CHIME:    return smpChime(d);
    case L_CRICKETS: return smpCrickets(d);
    case L_WHITE:    return smpWhite(d);
    case L_CLOCK:    return smpClock(d);
    case L_FAN:      return smpFan(d);
    case L_THUNDER:  return smpThunder(d);
    case L_LEAVES:   return smpLeaves(d);
    case L_FIRE:     return smpFire(d);
    case L_BOWL:     return smpBowl(d);
    default:         return smpPink(d);
  }
}
static void layerReset(LayerDSP& d) {
  memset(&d, 0, sizeof(d));
}

static float mixBuf[BUF];
static float masterGain = 0.f;

// ---------------------------------------------------------------------------
// Space (reverb)
// ---------------------------------------------------------------------------
// A small Schroeder reverb: four damped comb filters in parallel feeding two
// allpass diffusers. Chosen over anything fancier because it is entirely
// multiply-add and fixed-length delay reads -- no per-sample transcendentals,
// which is the same rule the rest of the engine follows. The comb lengths are
// mutually prime so their echo patterns never line up into a ringing tone.
#define NCOMB 4
#define NAP   2
static const int COMB_LEN[NCOMB] = {557, 593, 641, 677};   // samples @22050 Hz
static const int AP_LEN[NAP]     = {113, 277};
static float combBuf[NCOMB][680];
static float apBuf[NAP][280];
static int   combIdx[NCOMB], apIdx[NAP];
static float combLP[NCOMB];
// Written by the UI task, read by the audio task
static volatile float gSpaceMix = 0.f;        // 0..1
static volatile float gSpaceFeedback = 0.78f; // room size
static volatile float gSpaceDamp = 0.35f;     // high-frequency absorption

static void spaceClear() {
  memset(combBuf, 0, sizeof(combBuf));
  memset(apBuf, 0, sizeof(apBuf));
  memset(combLP, 0, sizeof(combLP));
  memset(combIdx, 0, sizeof(combIdx));
  memset(apIdx, 0, sizeof(apIdx));
}
static inline float applySpace(float x, float mix, float fb, float damp) {
  float in = x * 0.25f;
  float acc = 0.f;
  for (int i = 0; i < NCOMB; i++) {
    float y = combBuf[i][combIdx[i]];
    combLP[i] = y * (1.f - damp) + combLP[i] * damp;
    combBuf[i][combIdx[i]] = in + combLP[i] * fb;
    if (++combIdx[i] >= COMB_LEN[i]) combIdx[i] = 0;
    acc += y;
  }
  float w = acc * 0.25f;
  for (int i = 0; i < NAP; i++) {
    float b = apBuf[i][apIdx[i]];
    float t = w + b * 0.5f;
    apBuf[i][apIdx[i]] = t;
    if (++apIdx[i] >= AP_LEN[i]) apIdx[i] = 0;
    w = b - t * 0.5f;
  }
  return x * (1.f - 0.28f * mix) + w * mix * 1.5f;
}

// ---------------------------------------------------------------------------
// Session bell
// ---------------------------------------------------------------------------
// The meditation timer's bell is deliberately NOT the Bowl layer: the bell has
// to be heard at the start and end of a session whatever the current scene
// happens to be, including scenes where Bowl's volume is zero. So it is its own
// tiny generator, mixed in after the scene.
static float bellPh[3], bellIncr[3], bellAmp = 0.f, bellDec = 1.f, bellUp = 0.f, bellUpDec = 1.f;
static volatile bool  gBellRequest = false;
static volatile float gBellPitch = 0.5f;

static void bellTrigger(float pitch) {
  float f = 400.f + 340.f * pitch;
  bellPh[0] = bellPh[1] = bellPh[2] = 0.f;
  bellIncr[0] = f / SAMPLE_RATE;
  bellIncr[1] = (f * 1.004f) / SAMPLE_RATE;      // slight detune, gives it life
  bellIncr[2] = (f * 2.76f) / SAMPLE_RATE;       // inharmonic shimmer
  bellAmp = 0.22f;
  bellDec = expf(-1.0f / (SAMPLE_RATE * 4.5f));
  bellUp = 0.10f;
  bellUpDec = expf(-1.0f / (SAMPLE_RATE * 0.9f));
}
static inline float bellSample() {
  if (bellAmp < 0.00004f) return 0.f;
  float o = (tsinL(bellPh[0]) + tsinL(bellPh[1]) * 0.7f) * bellAmp + tsinL(bellPh[2]) * bellUp;
  for (int i = 0; i < 3; i++) { bellPh[i] += bellIncr[i]; if (bellPh[i] >= 1.f) bellPh[i] -= 1.f; }
  bellAmp *= bellDec;
  bellUp  *= bellUpDec;
  return o;
}

// ---------------------------------------------------------------------------
// Beat tones (hidden mode)
// ---------------------------------------------------------------------------
// v0.81: true binaural (a different tone per ear) and the Balance control are
// both GONE. Testing on real hardware showed panning broken the same way in
// every mode -- Balance stuck at max from center leftward and cutting to
// silence rightward, with no gradient in between. That is not a software
// balance-curve bug; it is what you get when only one side of an interleaved
// stereo buffer is actually reaching the DAC. The 3.5 mm jack likely carries
// the same mono signal on both sides rather than two independent channels, or
// M5Unified's "stereo" playRaw path doesn't do what its name suggests on this
// board -- either way, genuine per-channel output is not available here, so
// the whole engine is back to a single mono stream. This also undoes the
// v0.7 stereo conversion of the main audio path.
//
// What's left is the two delivery modes that never actually needed stereo:
//   MONAURAL   two tones summed into one signal. The beat is physically
//              present in the waveform, so a single speaker reproduces it.
//   ISOCHRONIC one tone switched on and off at the beat rate. Also mono-safe.
enum BeatMode { BEAT_MONAURAL = 0, BEAT_ISOCHRONIC, NUM_BEAT_MODES };
static const char* BEAT_MODE_NAMES[NUM_BEAT_MODES] = {"Monaural", "Isochronic"};

// v0.83: the individually-racy 'volatile float' globals below (gBinBaseHz,
// gBinBeatHz, gBinVol, one write per field) are gone. Raising Beat with the
// repeat-key held fires many writes per second to those fields from the UI
// thread while the audio thread (a separate core) reads them every buffer
// with no lock between the two sides; each field was defended individually
// (see the NaN/Inf self-heal further down), which fixed the Preset case
// (one write) but not this one (many rapid writes -> a much wider window for
// the audio thread to read a torn or inconsistent COMBINATION of fields).
// Guarding each field's own value can't catch that, since a torn read can
// land on a value that looks perfectly plausible on its own.
//
// The actual fix is to stop the audio thread from reading several
// independently-updated fields in the first place. All five parameters are
// now written together into one BinParams struct held in one of two buffers;
// the UI thread finishes writing to the buffer that ISN'T currently in use,
// then flips a single volatile int to publish it. The audio thread reads
// that one index once per buffer and then reads only from the struct copy it
// points to -- a copy the UI thread will never touch again until the index
// has moved on to the other slot. So the audio side always sees either the
// complete old set or the complete new set, never a mix of the two.
struct BinParams { float baseHz, beatHz, vol; uint8_t mode; bool on; };
static BinParams binParamBuf[2] = {
  {200.f, 6.f, 0.f, BEAT_MONAURAL, false},
  {200.f, 6.f, 0.f, BEAT_MONAURAL, false},
};
static volatile int binParamActive = 0;

static void binPublishParams(float baseHz, float beatHz, float vol, uint8_t mode, bool on) {
  int next = 1 - binParamActive;
  binParamBuf[next].baseHz = baseHz;
  binParamBuf[next].beatHz = beatHz;
  binParamBuf[next].vol    = vol;
  binParamBuf[next].mode   = mode;
  binParamBuf[next].on     = on;
  binParamActive = next;   // single-word publish -- audio task picks it up on its next buffer
}

static float binPhA = 0.f, binPhB = 0.f, binPhGate = 0.f;
static float binGain = 0.f;   // smoothed, so entering/leaving the mode never clicks

static void renderBuffer(int16_t* out) {
  memset(mixBuf, 0, sizeof(mixBuf));
  for (int i = 0; i < NUM_LAYERS; i++) {
    LayerDSP& d = dsp[i];
    float target = gLayerTarget[i];
    float gNext = d.gain + (target - d.gain) * 0.25f;
    if (fabsf(gNext - target) < 0.0005f) gNext = target;
    if (d.gain <= 0.0005f && gNext <= 0.0005f) {
      // Layer is silent: skip all DSP work. Reset so it starts clean next time.
      if (d.gain != 0.f || d.level > 0.f) layerReset(d);
      continue;
    }
    layerTick(i);
    float g = d.gain, gs = (gNext - g) / BUF, pk = 0.f;
    for (int n = 0; n < BUF; n++) {
      float s = layerSample(i, d);
      float a = fabsf(s); if (a > pk) pk = a;
      mixBuf[n] += s * g;
      g += gs;
    }
    d.gain = gNext;
    d.amp  = d.ampNext;
    float lv = pk * 2.0f; if (lv > 1.f) lv = 1.f;
    d.level += (lv - d.level) * (lv > d.level ? 0.5f : 0.12f);
  }
  // One index read publishes the whole parameter set for this buffer -- see
  // the comment above BinParams for why this replaced separate volatile
  // fields for baseHz/beatHz/vol/mode/on.
  const BinParams& bp = binParamBuf[binParamActive];
  if (gBellRequest) { gBellRequest = false; bellTrigger(bp.on ? 0.5f : gBellPitch); }

  float mNext = masterGain + (gMasterTarget - masterGain) * 0.2f;
  float m = masterGain, ms = (mNext - m) / BUF;
  // Reverb controls are resolved once per buffer, never per sample
  float spMix = gSpaceMix, spFb = gSpaceFeedback, spDamp = gSpaceDamp;
  bool  spOn  = spMix > 0.002f;
  float binTarget = bp.on ? bp.vol : 0.f;
  float binStep = (binTarget - binGain) / BUF;
  // Extra belt-and-braces: even with the double-buffer above making a torn
  // CROSS-FIELD read impossible, still sanity-check the values themselves
  // before they reach a division, and self-heal the phases below too, in
  // case anything ever manages to publish a bad struct (e.g. a future edit
  // that stores an intermediate value before it's fully computed).
  float baseHz = bp.baseHz, beatHz = bp.beatHz;
  if (!(baseHz > 0.f && baseHz < 5000.f)) baseHz = 200.f;
  if (!(beatHz >= 0.f && beatHz < 200.f)) beatHz = 6.f;
  float incA = baseHz / SAMPLE_RATE;
  float incB = (baseHz + beatHz) / SAMPLE_RATE;
  float incGate = beatHz / SAMPLE_RATE;
  uint8_t mode = bp.mode;

  for (int n = 0; n < BUF; n++) {
    float x = mixBuf[n] * m + bellSample();
    if (spOn) x = applySpace(x, spMix, spFb, spDamp);
    if (binGain > 0.0001f || binStep > 0.f) {
      float a = tsinL(binPhA), b = tsinL(binPhB);
      float t;
      if (mode == BEAT_ISOCHRONIC) {
        float gate = tsinL(binPhGate) * 0.5f + 0.5f;     // 0..1 at the beat rate
        gate *= gate;                                    // softer edges than a square gate
        t = a * gate;
      } else {
        t = (a + b) * 0.5f;                               // monaural: the beat is in the waveform itself
      }
      x += t * binGain;
    }
    // "< 2.f" is false for NaN and for any runaway magnitude, not just for
    // values past the wrap point, so this single comparison both performs the
    // ordinary phase wrap AND self-heals a corrupted phase within one sample.
    binPhA += incA;    if (!(binPhA < 2.f))    binPhA = 0.f;    else if (binPhA >= 1.f)    binPhA -= 1.f;
    binPhB += incB;    if (!(binPhB < 2.f))    binPhB = 0.f;    else if (binPhB >= 1.f)    binPhB -= 1.f;
    binPhGate += incGate; if (!(binPhGate < 2.f)) binPhGate = 0.f; else if (binPhGate >= 1.f) binPhGate -= 1.f;
    binGain += binStep;

    // Soft clip, then dither + round-to-nearest on the way into 16 bits.
    //
    // The old plain cast truncated, which both doubles the quantisation error
    // and leaves a dead zone: everything within +-1 LSB collapses to zero and
    // the error is correlated with the signal, so a quiet sustained tone turns
    // into a coarse staircase that buzzes. Noise-based layers mask this
    // completely, which is why it only ever showed up on the singing bowl's
    // long tail -- and worst at each beat null, where the two partials cancel
    // and the signal passes straight through the dead zone. Measured error on
    // a decaying bowl tail was about -44 dB by the end of the tail; rounding
    // recovers 6 dB of that and the TPDF dither decorrelates the remainder
    // into a flat -91 dBFS hiss, which is far below anything audible here.
    if (x > 0.7f)       x = 0.7f + (x - 0.7f) / (1.f + (x - 0.7f) * 3.f);
    else if (x < -0.7f) x = -0.7f + (x + 0.7f) / (1.f - (x + 0.7f) * 3.f);
    if (x > 0.98f) x = 0.98f; else if (x < -0.98f) x = -0.98f;
    if (x == 0.f) { out[n] = 0; m += ms; continue; }     // true silence when paused
    float q = x * 32000.f + (frand() - frand());        // TPDF, +-1 LSB
    out[n] = (int16_t)(q >= 0.f ? q + 0.5f : q - 0.5f);
    m += ms;
  }
  masterGain = mNext;
  binGain = binTarget;
}

static void audioTask(void*) {
  static int16_t buf[2][BUF];
  int b = 0, cnt = 0;
  for (;;) {
    renderBuffer(buf[b]);
    M5Cardputer.Speaker.playRaw(buf[b], BUF, SAMPLE_RATE, false, 1, 0);   // blocks while the queue is full
    b ^= 1;
    if (++cnt >= 8) { cnt = 0; vTaskDelay(1); }
  }
}

// ---------------------------------------------------------------------------
// 3. Settings and persistence
// ---------------------------------------------------------------------------
struct AppSettings {
  uint8_t  masterVol;     // 0..100
  uint8_t  brightness;    // 10..100 (%)
  uint16_t dimAfterSec;   // 0 = never
  uint8_t  dimLevel;      // 5..100 (%)
  uint16_t offAfterSec;   // 0 = never
  bool     sleepEnabled;
  uint16_t sleepMinutes;  // 1..180
  bool     sleepFade;     // fade out over the last minute
  // LED
  bool     ledEnabled;
  uint8_t  ledSource;     // 0..NUM_LAYERS-1 = follow that layer, NUM_LAYERS = free-running period
  uint8_t  ledColor;      // 0..NUM_PALETTE-1, or NUM_PALETTE = "follow the source layer's colour"
  uint8_t  ledBright;     // 0..100
  uint8_t  ledSpeed;      // 0..100 (period mode rate / how fast it tracks in layer mode)
  uint8_t  ledDepth;      // 0..100 fluctuation intensity
  // Shake (IMU)
  bool     shakeEnabled;
  uint8_t  shakeSens;     // 0..100
  uint8_t  shakeAction;   // ShakeAction
  // Space (reverb)
  uint8_t  spaceMix;      // 0..100, 0 = off
  uint8_t  spaceSize;     // 0..100
  uint8_t  spaceDamp;     // 0..100
  // Meditation session
  uint8_t  sessionMin;    // 1..120
  uint8_t  sessionInterval; // 0 = no interval bell, else minutes
  uint8_t  sessionPitch;  // 0..100
  // Main screen visualisation / breathing guide
  uint8_t  mainViz;
  // Hidden Beats screen -- persisted so it doesn't reset to defaults every
  // time it's opened (it used to, which meant re-dialing Ambient every visit).
  uint8_t  binPreset;
  uint8_t  binMode;
  float    binBase;
  float    binBeat;
  uint8_t  binVolume;
  uint8_t  binAmbient;
};
enum ShakeAction { SHAKE_RANDOM = 0, SHAKE_SLEEP, SHAKE_DISPLAY, SHAKE_LED, NUM_SHAKE_ACTIONS };
static const char* SHAKE_ACTION_NAMES[NUM_SHAKE_ACTIONS] = {"Random Scene", "Sleep On", "Display Off", "LED On/Off"};
static AppSettings settings;

static void settingsDefaults() {
  settings.masterVol = 70;
  settings.brightness = 80;
  settings.dimAfterSec = 30;
  settings.dimLevel = 20;
  settings.offAfterSec = 120;
  settings.sleepEnabled = false;
  settings.sleepMinutes = 30;
  settings.sleepFade = true;
  settings.ledEnabled = false;
  settings.ledSource = NUM_LAYERS;     // free-running period by default
  settings.ledColor = NUM_PALETTE;     // follow the source layer's colour
  settings.ledBright = 40;
  settings.ledSpeed = 30;
  settings.ledDepth = 55;
  settings.shakeEnabled = false;
  settings.shakeSens = 50;
  settings.shakeAction = SHAKE_RANDOM;
  settings.spaceMix = 0;
  settings.spaceSize = 55;
  settings.spaceDamp = 40;
  settings.sessionMin = 15;
  settings.sessionInterval = 0;
  settings.sessionPitch = 50;
  settings.mainViz = 0;
  settings.binPreset = 1;
  settings.binMode = 0;
  settings.binBase = 200.f;
  settings.binBeat = 6.f;
  settings.binVolume = 30;
  settings.binAmbient = 25;
}

// Scene registry: the ordered list shown to the user (presets + custom scenes)
// isDrift entries are the evolving scene. They set isPreset = true as well, so
// every existing "presets cannot be renamed / deleted / written to" guard
// applies to them unchanged; presetIdx is -1 because there is no PRESETS[] row.
struct SceneEntry { String name; bool isPreset; bool isDrift; int presetIdx; bool visible; };
static const char* DRIFT_NAME = "Drift";

// Drift state lives up here because loadSceneIntoLive() (further down in the
// persistence section) has to be able to arm it on boot.
static bool driftActive = false;
static void generateRandomScene(SceneData& s, const char* name);
static void driftBeginTimer();
static std::vector<SceneEntry> sceneList;
static int  currentScene = 0;       // index into sceneList
static bool liveEdited = false;     // live differs from its source (only meaningful for presets)

static bool sdReady = false;
static bool settingsDirty = false, sceneDirty = false;
static uint32_t dirtyAtMs = 0;

static void markSettingsDirty() { settingsDirty = true; dirtyAtMs = millis(); }
static void markSceneDirty()    { sceneDirty = true; liveEdited = true; dirtyAtMs = millis(); }

static String sceneFileName(const String& name) {
  String f = String(DIR_SCENE) + "/";
  for (size_t i = 0; i < name.length(); i++) {
    char c = name[i];
    if (isalnum((unsigned char)c) || c == '-' || c == '_') f += c;
    else if (c == ' ') f += '_';
  }
  return f + ".json";
}

static void layersToJson(JsonArray arr, const SceneData& s) {
  for (int i = 0; i < NUM_LAYERS; i++) {
    JsonObject o = arr.add<JsonObject>();
    o["vol"] = s.L[i].vol;
    o["mute"] = s.L[i].mute;
    o["color"] = s.L[i].color;
    JsonArray p = o["p"].to<JsonArray>();
    for (int k = 0; k < NPARAM; k++) p.add(s.L[i].p[k]);
  }
}
static void layersFromJson(JsonArray arr, SceneData& s) {
  int i = 0;
  for (JsonObject o : arr) {
    if (i >= NUM_LAYERS) break;
    s.L[i].vol = constrain((int)(o["vol"] | 0), 0, 100);
    s.L[i].mute = o["mute"] | false;
    s.L[i].color = constrain((int)(o["color"] | LAYER_DEFS[i].defColor), 0, NUM_PALETTE - 1);
    JsonArray p = o["p"].as<JsonArray>();
    int k = 0;
    for (JsonVariant v : p) { if (k >= NPARAM) break; s.L[i].p[k] = constrain((int)v.as<int>(), 0, 100); k++; }
    i++;
  }
}

static bool saveSceneFile(const SceneData& s) {
  if (!sdReady) return false;
  JsonDocument doc;
  doc["name"] = s.name;
  layersToJson(doc["layers"].to<JsonArray>(), s);
  String path = sceneFileName(s.name);
  File f = SD.open(path, FILE_WRITE);
  if (!f) return false;
  serializeJson(doc, f);
  f.close();
  return true;
}
static bool loadSceneFile(const String& name, SceneData& s) {
  sceneSetDefaults(s, name.c_str());
  if (!sdReady) return false;
  File f = SD.open(sceneFileName(name), FILE_READ);
  if (!f) return false;
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return false;
  layersFromJson(doc["layers"].as<JsonArray>(), s);
  return true;
}
static void deleteSceneFile(const String& name) {
  if (sdReady) SD.remove(sceneFileName(name));
}

static void saveSettings() {
  if (!sdReady) return;
  JsonDocument doc;
  doc["vol"] = settings.masterVol;
  doc["playing"] = playing;
  doc["scene"] = sceneList.empty() ? "" : sceneList[currentScene].name;
  doc["bright"] = settings.brightness;
  doc["dimAfter"] = settings.dimAfterSec;
  doc["dimLevel"] = settings.dimLevel;
  doc["offAfter"] = settings.offAfterSec;
  doc["sleepOn"] = settings.sleepEnabled;
  doc["sleepMin"] = settings.sleepMinutes;
  doc["sleepFade"] = settings.sleepFade;
  doc["ledOn"] = settings.ledEnabled;
  doc["ledSrc"] = settings.ledSource;
  doc["ledCol"] = settings.ledColor;
  doc["ledBri"] = settings.ledBright;
  doc["ledSpd"] = settings.ledSpeed;
  doc["ledDep"] = settings.ledDepth;
  doc["shakeOn"] = settings.shakeEnabled;
  doc["shakeSens"] = settings.shakeSens;
  doc["shakeAct"] = settings.shakeAction;
  doc["spMix"] = settings.spaceMix;
  doc["spSize"] = settings.spaceSize;
  doc["spDamp"] = settings.spaceDamp;
  doc["sesMin"] = settings.sessionMin;
  doc["sesInt"] = settings.sessionInterval;
  doc["sesPit"] = settings.sessionPitch;
  doc["viz"] = settings.mainViz;
  doc["binPre"] = settings.binPreset;
  doc["binMode"] = settings.binMode;
  doc["binBase"] = settings.binBase;
  doc["binBeat"] = settings.binBeat;
  doc["binVol"] = settings.binVolume;
  doc["binAmb"] = settings.binAmbient;
  JsonArray order = doc["order"].to<JsonArray>();
  for (auto& e : sceneList) {
    JsonObject o = order.add<JsonObject>();
    o["n"] = e.name; o["c"] = !e.isPreset; o["v"] = e.visible; o["d"] = e.isDrift;
  }
  // Unsaved edits made on top of a preset are kept here so they survive a reboot
  if (liveEdited && !sceneList.empty() && sceneList[currentScene].isPreset
      && !sceneList[currentScene].isDrift) {
    layersToJson(doc["live"].to<JsonArray>(), live);
  }
  File f = SD.open(SETTINGS_PATH, FILE_WRITE);
  if (!f) return;
  serializeJson(doc, f);
  f.close();
}

static int findPreset(const String& name) {
  for (int i = 0; i < NUM_PRESETS; i++) if (name == PRESETS[i].name) return i;
  return -1;
}
static int findSceneEntry(const String& name) {
  for (size_t i = 0; i < sceneList.size(); i++) if (sceneList[i].name == name) return (int)i;
  return -1;
}

// Build the scene list: saved order first, then any presets / files not yet listed
static void buildSceneList(JsonArray savedOrder) {
  sceneList.clear();
  if (!savedOrder.isNull()) {
    for (JsonObject o : savedOrder) {
      SceneEntry e;
      e.name = (const char*)(o["n"] | "");
      e.isPreset = !(bool)(o["c"] | false);
      e.isDrift = o["d"] | false;
      e.visible = o["v"] | true;
      e.presetIdx = (e.isPreset && !e.isDrift) ? findPreset(e.name) : -1;
      if (e.isPreset && !e.isDrift && e.presetIdx < 0) continue;   // preset no longer exists
      if (!e.isPreset && !sdReady) continue;
      if (!e.isPreset && !SD.exists(sceneFileName(e.name))) continue;
      if (e.name.length() && findSceneEntry(e.name) < 0) sceneList.push_back(e);
    }
  }
  for (int i = 0; i < NUM_PRESETS; i++) {
    if (findSceneEntry(PRESETS[i].name) < 0) {
      SceneEntry e; e.name = PRESETS[i].name; e.isPreset = true; e.isDrift = false;
      e.presetIdx = i; e.visible = true;
      sceneList.push_back(e);
    }
  }
  if (findSceneEntry(DRIFT_NAME) < 0) {
    SceneEntry e; e.name = DRIFT_NAME; e.isPreset = true; e.isDrift = true;
    e.presetIdx = -1; e.visible = true;
    sceneList.push_back(e);
  }
  if (sdReady) {
    File dir = SD.open(DIR_SCENE);
    if (dir) {
      File f;
      while ((f = dir.openNextFile())) {
        if (!f.isDirectory()) {
          JsonDocument doc;
          if (!deserializeJson(doc, f)) {
            String n = (const char*)(doc["name"] | "");
            if (n.length() && findSceneEntry(n) < 0 && findPreset(n) < 0) {
              SceneEntry e; e.name = n; e.isPreset = false; e.isDrift = false;
              e.presetIdx = -1; e.visible = true;
              sceneList.push_back(e);
            }
          }
        }
        f.close();
      }
      dir.close();
    }
  }
}

static void loadSceneIntoLive(int idx) {
  if (sceneList.empty()) { sceneSetDefaults(live, "Empty"); return; }
  const SceneEntry& e = sceneList[idx];
  driftActive = e.isDrift;
  if (e.isDrift) { generateRandomScene(live, DRIFT_NAME); driftBeginTimer(); }
  else if (e.isPreset) presetToScene(e.presetIdx, live);
  else loadSceneFile(e.name, live);
  liveEdited = false;
}

static void loadSettings() {
  settingsDefaults();
  JsonDocument doc;
  bool ok = false;
  if (sdReady) {
    File f = SD.open(SETTINGS_PATH, FILE_READ);
    if (f) { ok = !deserializeJson(doc, f); f.close(); }
  }
  if (ok) {
    settings.masterVol   = constrain((int)(doc["vol"] | 70), 0, 100);
    playing              = doc["playing"] | true;
    settings.brightness  = constrain((int)(doc["bright"] | 80), 10, 100);
    settings.dimAfterSec = constrain((int)(doc["dimAfter"] | 30), 0, 600);
    settings.dimLevel    = constrain((int)(doc["dimLevel"] | 20), 5, 100);
    settings.offAfterSec = constrain((int)(doc["offAfter"] | 120), 0, 1800);
    settings.sleepEnabled = doc["sleepOn"] | false;
    settings.sleepMinutes = constrain((int)(doc["sleepMin"] | 30), 1, 180);
    settings.sleepFade    = doc["sleepFade"] | true;
    settings.ledEnabled   = doc["ledOn"] | false;
    settings.ledSource    = constrain((int)(doc["ledSrc"] | NUM_LAYERS), 0, NUM_LAYERS);
    settings.ledColor     = constrain((int)(doc["ledCol"] | NUM_PALETTE), 0, NUM_PALETTE);
    settings.ledBright    = constrain((int)(doc["ledBri"] | 40), 0, 100);
    settings.ledSpeed     = constrain((int)(doc["ledSpd"] | 30), 0, 100);
    settings.ledDepth     = constrain((int)(doc["ledDep"] | 55), 0, 100);
    settings.shakeEnabled = doc["shakeOn"] | false;
    settings.shakeSens    = constrain((int)(doc["shakeSens"] | 50), 0, 100);
    settings.shakeAction  = constrain((int)(doc["shakeAct"] | 0), 0, NUM_SHAKE_ACTIONS - 1);
    settings.spaceMix     = constrain((int)(doc["spMix"] | 0), 0, 100);
    settings.spaceSize    = constrain((int)(doc["spSize"] | 55), 0, 100);
    settings.spaceDamp    = constrain((int)(doc["spDamp"] | 40), 0, 100);
    settings.sessionMin   = constrain((int)(doc["sesMin"] | 15), 1, 120);
    settings.sessionInterval = constrain((int)(doc["sesInt"] | 0), 0, 30);
    settings.sessionPitch = constrain((int)(doc["sesPit"] | 50), 0, 100);
    settings.mainViz      = constrain((int)(doc["viz"] | 0), 0, 3);
    settings.binPreset    = constrain((int)(doc["binPre"] | 1), 0, 6);
    settings.binMode      = constrain((int)(doc["binMode"] | 0), 0, 1);
    settings.binBase      = doc["binBase"] | 200.f;
    settings.binBeat      = doc["binBeat"] | 6.f;
    settings.binVolume    = constrain((int)(doc["binVol"] | 30), 0, 100);
    settings.binAmbient   = constrain((int)(doc["binAmb"] | 25), 0, 100);
  }
  buildSceneList(ok ? doc["order"].as<JsonArray>() : JsonArray());
  currentScene = 0;
  if (ok) {
    int idx = findSceneEntry(String((const char*)(doc["scene"] | "")));
    if (idx >= 0) currentScene = idx;
  }
  loadSceneIntoLive(currentScene);
  if (ok && !doc["live"].isNull() && sceneList[currentScene].isPreset
      && !sceneList[currentScene].isDrift) {
    layersFromJson(doc["live"].as<JsonArray>(), live);
    liveEdited = true;
  }
}

static void initStorage() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  sdReady = SD.begin(SD_CS, SPI, 25000000);
  if (sdReady) {
    if (!SD.exists(DIR_ROOT)) SD.mkdir(DIR_ROOT);
    if (!SD.exists(DIR_SCENE)) SD.mkdir(DIR_SCENE);
  }
}

// Called every loop: flushes pending changes once things have settled
static void flushDirty() {
  if (!(settingsDirty || sceneDirty)) return;
  if (millis() - dirtyAtMs < SAVE_DEBOUNCE_MS) return;
  if (sceneDirty && !sceneList.empty() && !sceneList[currentScene].isPreset) {
    saveSceneFile(live);   // custom scene: edits go straight into its file
  }
  saveSettings();          // settings (and preset edits, stored under "live")
  settingsDirty = sceneDirty = false;
}

// ---------------------------------------------------------------------------
// 4. Scene switching (immediate crossfade), gain targets, sleep timer
// ---------------------------------------------------------------------------
// Boot-time fade-in only (silence -> first scene). Scene-to-scene switches no
// longer mute anything; they crossfade instead (see below).
static float bootFadeMul = 0.f;
static bool  bootFading  = true;

static float sleepMul = 1.f;
static uint32_t lastFrameMs = 0;
static uint32_t nameShowUntil = 0;   // scene-name toast shown on main screen after a switch

// Moving to a new scene no longer requires a separate confirm step: as soon as
// the highlighted scene changes, `live` starts morphing from where it
// currently is toward the newly selected scene's per-layer volumes and
// parameters, over SCENE_XFADE_SEC. Since the same persistent layer DSP state
// (dsp[]) keeps running underneath the whole time, this is a true crossfade,
// not a mute/reload — and because `live` itself (not a separate scene index)
// is what gets sampled each frame, re-triggering mid-transition (fast
// browsing) just retargets smoothly from the current in-between values with
// no click or jump.
static SceneData xfFrom, xfTo;
static float xfT = 1.f;          // 0..1, 1 = transition finished
static bool  xfActive = false;

static float xfDur = SCENE_XFADE_SEC;   // this transition's length in seconds

// How a crossfade target relates to the scene list.
enum XfKind {
  XF_LISTED = 0,   // a real entry: becomes currentScene
  XF_LOOSE,        // generated on the fly (Random): currentScene untouched, counts as an edit
  XF_DRIFT_STEP    // one nudge of the evolving scene: silent, no name banner, no save
};

static void startCrossfadeToData(const SceneData& to, XfKind kind, int idx, float dur) {
  if (sceneDirty && !sceneList.empty()) {
    if (!sceneList[currentScene].isPreset) saveSceneFile(live);
    sceneDirty = false;
  }
  xfFrom = live;   // current (possibly still mid-transition) values become the new starting point
  xfTo = to;
  xfT = 0.f;
  xfDur = dur;
  xfActive = true;
  if (kind != XF_DRIFT_STEP) soloMask = 0;
  if (kind == XF_LISTED) { currentScene = idx; liveEdited = false; }
  else if (kind == XF_LOOSE) liveEdited = true;
  strncpy(live.name, xfTo.name, sizeof(live.name) - 1); live.name[sizeof(live.name) - 1] = 0;
  // A drift step is meant to go unnoticed: no banner, and nothing written to
  // SD (otherwise the card would be rewritten every time the scene breathes).
  if (kind != XF_DRIFT_STEP) {
    nameShowUntil = millis() + 1600;
    markSettingsDirty();
  }
}

// Random Scene: pick a few natural layers and give them plausible settings.
// Mechanical layers (Clock, Fan) and the plain noise generators are excluded
// -- see layerIsRandomizable().
static void generateRandomScene(SceneData& s, const char* name) {
  sceneSetDefaults(s, name);
  int pool[NUM_LAYERS], nPool = 0;
  for (int i = 0; i < NUM_LAYERS; i++) if (layerIsRandomizable(i)) pool[nPool++] = i;
  // Shuffle, then take the first few
  for (int i = nPool - 1; i > 0; i--) {
    int j = (int)(esp_random() % (uint32_t)(i + 1));
    int t = pool[i]; pool[i] = pool[j]; pool[j] = t;
  }
  int pick = 2 + (int)(esp_random() % 3);      // 2-4 layers
  if (pick > nPool) pick = nPool;
  for (int k = 0; k < pick; k++) {
    int i = pool[k];
    LayerSettings& L = s.L[i];
    // The first pick is the bed and sits loudest; later picks are accents.
    int lo = k == 0 ? 45 : 15, hi = k == 0 ? 80 : 55;
    L.vol = lo + (int)(esp_random() % (uint32_t)(hi - lo + 1));
    for (int p = 0; p < NPARAM; p++) L.p[p] = 20 + (int)(esp_random() % 61);   // 20-80, avoids extremes
    L.color = (uint8_t)(esp_random() % NUM_PALETTE);
  }
}
// "Elsewhere" rather than "Random": the screen is showing the name of a place
// you have arrived at, and a poetic label sits better with the rest of the
// firmware than a technical one.
static const char* RANDOM_SCENE_NAME = "Elsewhere";

static void randomScene() {
  driftActive = false;
  SceneData s;
  generateRandomScene(s, RANDOM_SCENE_NAME);
  startCrossfadeToData(s, XF_LOOSE, -1, SCENE_XFADE_SEC);
}

// ---------------------------------------------------------------------------
// Drift: a scene that keeps changing
// ---------------------------------------------------------------------------
// Selecting the Drift entry rolls a fresh random scene, and from then on the
// mix keeps evolving for as long as it stays selected: every so often one
// thing changes -- a parameter nudged, a layer's level moved, a new sound
// easing in, or one fading away -- each over a long, slow crossfade so no
// single change is ever a moment you can point at.
static float driftTimer = 0.f;

static const float DRIFT_MIN_WAIT = 18.f;   // seconds between changes
static const float DRIFT_MAX_WAIT = 45.f;
static const float DRIFT_XFADE    = 14.f;   // seconds each change takes to arrive

static inline int randRange(int lo, int hi) {   // inclusive
  if (hi <= lo) return lo;
  return lo + (int)(esp_random() % (uint32_t)(hi - lo + 1));
}

static void driftBeginTimer() { driftTimer = DRIFT_MIN_WAIT + (esp_random() % 1000) * 0.001f * (DRIFT_MAX_WAIT - DRIFT_MIN_WAIT); }

static void driftMutate(SceneData& s) {
  int active[NUM_LAYERS], nActive = 0, idle[NUM_LAYERS], nIdle = 0;
  for (int i = 0; i < NUM_LAYERS; i++) {
    if (!layerIsRandomizable(i)) continue;     // drift stays in the natural palette
    if (s.L[i].vol > 3) active[nActive++] = i; else idle[nIdle++] = i;
  }
  int roll = randRange(0, 99);
  // Bring a new sound in when the mix is thin, let one go when it is crowded
  if (nActive < 2 && nIdle > 0) roll = 95;
  if (nActive >= 5) roll = 88;

  if (roll < 45 && nActive > 0) {
    // Nudge one parameter of one active layer
    int i = active[randRange(0, nActive - 1)];
    int k = randRange(0, NPARAM - 1);
    int delta = randRange(8, 22) * (randRange(0, 1) ? 1 : -1);
    s.L[i].p[k] = (uint8_t)constrain((int)s.L[i].p[k] + delta, 10, 90);
  } else if (roll < 80 && nActive > 0) {
    // Move one active layer's level
    int i = active[randRange(0, nActive - 1)];
    int delta = randRange(8, 20) * (randRange(0, 1) ? 1 : -1);
    s.L[i].vol = (uint8_t)constrain((int)s.L[i].vol + delta, 8, 85);
  } else if (roll < 92 && nActive > 2) {
    // Let one fade away entirely
    int i = active[randRange(0, nActive - 1)];
    s.L[i].vol = 0;
  } else if (nIdle > 0) {
    // Ease a new one in, quietly
    int i = idle[randRange(0, nIdle - 1)];
    s.L[i].vol = (uint8_t)randRange(15, 40);
    for (int k = 0; k < NPARAM; k++) s.L[i].p[k] = (uint8_t)randRange(25, 75);
  }
}

static void updateDrift(float dt) {
  if (!driftActive) return;
  if (xfActive) return;                 // let the previous change finish arriving first
  driftTimer -= dt;
  if (driftTimer > 0.f) return;
  driftBeginTimer();
  SceneData to = live;
  driftMutate(to);
  startCrossfadeToData(to, XF_DRIFT_STEP, -1, DRIFT_XFADE);
}

static void startCrossfade(int idx) {
  if (idx < 0 || idx >= (int)sceneList.size()) return;
  if (idx == currentScene && !xfActive) return;
  SceneData to;
  const SceneEntry& e = sceneList[idx];
  driftActive = e.isDrift;
  if (e.isDrift) { generateRandomScene(to, DRIFT_NAME); driftBeginTimer(); }
  else if (e.isPreset) presetToScene(e.presetIdx, to);
  else loadSceneFile(e.name, to);
  startCrossfadeToData(to, XF_LISTED, idx, SCENE_XFADE_SEC);
}

static void updateCrossfade(float dt) {
  if (!xfActive) return;
  xfT += dt / (xfDur > 0.01f ? xfDur : SCENE_XFADE_SEC);
  bool done = xfT >= 1.f;
  float t = done ? 1.f : xfT;
  float e = t * t * (3.f - 2.f * t);   // smoothstep easing
  for (int i = 0; i < NUM_LAYERS; i++) {
    const LayerSettings& Lf = xfFrom.L[i]; const LayerSettings& Lt = xfTo.L[i];
    LayerSettings& Lv = live.L[i];
    Lv.vol = (uint8_t)(Lf.vol + (Lt.vol - Lf.vol) * e + 0.5f);
    for (int k = 0; k < NPARAM; k++) Lv.p[k] = (uint8_t)(Lf.p[k] + (Lt.p[k] - Lf.p[k]) * e + 0.5f);
    // Mute and color are step changes rather than blends (a "half muted" or
    // "half colored" state has no natural meaning) — switched at the midpoint.
    Lv.mute  = e < 0.5f ? Lf.mute  : Lt.mute;
    Lv.color = e < 0.5f ? Lf.color : Lt.color;
  }
  if (done) { xfActive = false; live = xfTo; }
}

static float effectiveLayerVol(int i) {
  const LayerSettings& L = live.L[i];
  if (L.mute) return 0.f;
  if (soloMask && !(soloMask & (1 << i))) return 0.f;
  float v = L.vol * 0.01f;
  return v * v;   // perceptual curve
}

static void updateGainTargets() {
  float playMul = playing ? 1.f : 0.f;
  // The beats screen dims the scene rather than muting it, so the tones can sit
  // on a bed of rain or wind without fighting it.
  float amb = gAmbientScale;
  for (int i = 0; i < NUM_LAYERS; i++) gLayerTarget[i] = effectiveLayerVol(i) * bootFadeMul * playMul * amb;
  float mv = settings.masterVol * 0.01f;
  gMasterTarget = mv * sqrtf(mv) * sleepMul;
}
static void updateBootFade(float dt) {
  if (!bootFading) return;
  bootFadeMul += dt / BOOT_FADE_IN_SEC;
  if (bootFadeMul >= 1.f) { bootFadeMul = 1.f; bootFading = false; }
}

// Sleep timer
static uint32_t sleepStartMs = 0;
static bool sleepArmed = false;
static void armSleep() { sleepStartMs = millis(); sleepArmed = settings.sleepEnabled && playing; sleepMul = 1.f; }
static int  sleepRemainingSec() {
  if (!sleepArmed) return -1;
  int total = settings.sleepMinutes * 60;
  int el = (int)((millis() - sleepStartMs) / 1000);
  return total - el;
}
static void powerDown() {
  ledStrip.clear();
  ledStrip.show();
  M5Cardputer.Display.sleep();
  M5Cardputer.Power.powerOff();
  esp_deep_sleep_start();   // fallback if powerOff is not supported
}
static void updateSleep() {
  if (!sleepArmed) { sleepMul = 1.f; return; }
  int rem = sleepRemainingSec();
  if (rem <= 0) { saveSettings(); powerDown(); }
  sleepMul = (settings.sleepFade && rem < 60) ? rem / 60.f : 1.f;
}

// ---------------------------------------------------------------------------
// 4b. Breathing guide
// ---------------------------------------------------------------------------
// The main screen's ring is already the centre of attention, so the breathing
// guide reuses it rather than adding a separate widget: the circle simply
// becomes the thing you breathe with. Phase types are 0 = inhale,
// 1 = hold (full), 2 = exhale, 3 = hold (empty).
enum MainViz { VIZ_RINGS = 0, VIZ_COHERENT, VIZ_BOX, VIZ_478, NUM_VIZ };
struct BreathPattern { const char* name; uint8_t n; float dur[4]; uint8_t type[4]; };
static const BreathPattern BREATH[NUM_VIZ - 1] = {
  {"Coherent 5.5", 2, {5.5f, 5.5f, 0.f, 0.f}, {0, 2, 0, 0}},
  {"Box 4-4-4-4",  4, {4.f, 4.f, 4.f, 4.f},   {0, 1, 2, 3}},
  {"Relax 4-7-8",  3, {4.f, 7.f, 8.f, 0.f},   {0, 1, 2, 0}},
};
static const char* VIZ_NAMES[NUM_VIZ] = {"Rings", "Coherent 5.5", "Box 4-4-4-4", "Relax 4-7-8"};
static const char* BREATH_LABEL[4] = {"Breathe in", "Hold", "Breathe out", "Hold"};

static int   breathPhase = 0;
static float breathT = 0.f;
static float breathOpen = 0.f;    // 0 = fully exhaled, 1 = fully inhaled
static uint32_t vizNameUntil = 0;

static inline bool breathActive() { return settings.mainViz != VIZ_RINGS; }

static void breathReset() { breathPhase = 0; breathT = 0.f; breathOpen = 0.f; }

static void updateBreath(float dt) {
  if (!breathActive()) { breathOpen = 0.f; return; }
  const BreathPattern& b = BREATH[settings.mainViz - 1];
  if (breathPhase >= b.n) breathPhase = 0;
  breathT += dt;
  while (breathT >= b.dur[breathPhase]) {
    breathT -= b.dur[breathPhase];
    breathPhase = (breathPhase + 1) % b.n;
  }
  float u = b.dur[breathPhase] > 0.f ? breathT / b.dur[breathPhase] : 0.f;
  float e = u * u * (3.f - 2.f * u);        // ease in/out, so the turns are soft
  switch (b.type[breathPhase]) {
    case 0: breathOpen = e; break;          // inhale
    case 1: breathOpen = 1.f; break;        // hold full
    case 2: breathOpen = 1.f - e; break;    // exhale
    default: breathOpen = 0.f; break;       // hold empty
  }
}
static float breathRemaining() {
  if (!breathActive()) return 0.f;
  const BreathPattern& b = BREATH[settings.mainViz - 1];
  return b.dur[breathPhase] - breathT;
}
static const char* breathLabel() {
  if (!breathActive()) return "";
  const BreathPattern& b = BREATH[settings.mainViz - 1];
  return BREATH_LABEL[b.type[breathPhase]];
}

// ---------------------------------------------------------------------------
// 4c. LED
// ---------------------------------------------------------------------------
static float ledPhase = 0.f;     // free-running phase for Period mode
static float ledLevel = 0.f;     // smoothed output level
static float ledFlick = 1.f;     // slow random flicker multiplier
static float ledFlickTarget = 1.f, ledFlickTimer = 0.f;

static void ledShow(float level) {
  // Pick the colour: either an explicit palette entry, or the colour of the
  // layer the LED is following (so the LED matches the ring on screen).
  int ci = settings.ledColor;
  if (ci >= NUM_PALETTE) {
    ci = settings.ledSource < NUM_LAYERS ? live.L[settings.ledSource].color : live.L[0].color;
  }
  const PaletteColor& c = PALETTE[ci];
  float g = level * (settings.ledBright * 0.01f);
  if (g < 0.f) g = 0.f; if (g > 1.f) g = 1.f;
  g = g * g;   // perceptual curve -- LEDs are far brighter at the low end than they look linear
  ledStrip.setPixelColor(0, ledStrip.Color((uint8_t)(c.r * g), (uint8_t)(c.g * g), (uint8_t)(c.b * g)));
  ledStrip.show();
}

static void updateLed(float dt) {
  static bool wasOn = false;
  if (!settings.ledEnabled || !playing) {
    if (wasOn) { ledShow(0.f); wasOn = false; }
    return;
  }
  wasOn = true;
  float speed = settings.ledSpeed * 0.01f;
  float depth = settings.ledDepth * 0.01f;

  // Slow random flicker, shared by both modes -- this is the "intensity sway"
  ledFlickTimer -= dt;
  if (ledFlickTimer <= 0.f) {
    ledFlickTimer = 0.15f + 1.2f * (1.f - speed) * (0.4f + 1.2f * (esp_random() % 1000) * 0.001f);
    ledFlickTarget = 1.f - depth * ((esp_random() % 1000) * 0.001f);
  }
  ledFlick += (ledFlickTarget - ledFlick) * (dt * (2.f + 10.f * speed));

  float target;
  if (breathActive()) {
    // While a breathing guide is running it takes the LED over: matching the
    // light to the breath is the whole point, and it works with eyes closed.
    target = 0.10f + 0.90f * breathOpen;
    ledLevel += (target * ledFlick - ledLevel) * (dt * 6.f > 1.f ? 1.f : dt * 6.f);
    ledShow(ledLevel);
    return;
  }
  if (settings.ledSource < NUM_LAYERS) {
    // Follow a layer: brightness tracks how loud and how active that layer is
    const LayerDSP& d = dsp[settings.ledSource];
    target = sqrtf(d.gain) * (0.25f + 0.75f * d.level);
  } else {
    // Free-running breathing pulse
    ledPhase += dt * (0.05f + 0.55f * speed);
    if (ledPhase >= 1.f) ledPhase -= 1.f;
    float s = sinf(6.2831853f * ledPhase) * 0.5f + 0.5f;
    target = 0.15f + 0.85f * s * s;
  }
  target *= ledFlick;
  // Rise quickly, fall gently -- reads as a pulse rather than a flash
  float k = target > ledLevel ? dt * (6.f + 18.f * speed) : dt * (1.5f + 6.f * speed);
  if (k > 1.f) k = 1.f;
  ledLevel += (target - ledLevel) * k;
  ledShow(ledLevel);
}

// ---------------------------------------------------------------------------
// 4d. Meditation session timer
// ---------------------------------------------------------------------------
static bool     sessionRunning = false;
static uint32_t sessionStartMs = 0;
static uint32_t sessionNextBellMs = 0;
static int      sessionEndBells = 0;
static uint32_t sessionEndBellAt = 0;

static void ringBell() { gBellPitch = settings.sessionPitch * 0.01f; gBellRequest = true; }

static void sessionStart() {
  sessionRunning = true;
  sessionStartMs = millis();
  sessionEndBells = 0;
  sessionNextBellMs = settings.sessionInterval > 0
      ? sessionStartMs + (uint32_t)settings.sessionInterval * 60000u : 0;
  ringBell();
}
static void sessionStop() { sessionRunning = false; sessionEndBells = 0; }

static int sessionRemainingSec() {
  if (!sessionRunning) return -1;
  int total = (int)settings.sessionMin * 60;
  int el = (int)((millis() - sessionStartMs) / 1000);
  int rem = total - el;
  return rem > 0 ? rem : 0;
}
static void updateSession() {
  if (sessionEndBells > 0 && millis() >= sessionEndBellAt) {
    ringBell();
    sessionEndBells--;
    sessionEndBellAt = millis() + 2600;
  }
  if (!sessionRunning) return;
  uint32_t now = millis();
  if (sessionNextBellMs && now >= sessionNextBellMs) {
    ringBell();
    sessionNextBellMs = now + (uint32_t)settings.sessionInterval * 60000u;
  }
  if (sessionRemainingSec() <= 0) {
    sessionRunning = false;
    sessionEndBells = 2;                 // one now, two more to close the session
    sessionEndBellAt = now;
  }
}

// ---------------------------------------------------------------------------
// 4e. Shake detection (IMU)
// ---------------------------------------------------------------------------
// Forward declarations: the shake handler triggers actions defined further down.
static void displayOff();
static void displayWake();
static bool displayIsOn();
static void toast(const char* m);

static uint32_t lastShakeMs = 0;
static float shakeEnergy = 0.f;
static bool  imuReady = false;   // set in setup() once the IMU reports itself present
static bool  shakeArmed = true;  // cleared on trigger, re-armed once the device settles

static void doShakeAction() {
  switch (settings.shakeAction) {
    case SHAKE_RANDOM:
      randomScene();
      break;
    case SHAKE_SLEEP:
      settings.sleepEnabled = true;
      armSleep();
      markSettingsDirty();
      toast("Sleep timer on");
      break;
    case SHAKE_DISPLAY:
      // Off only, deliberately. Waking by shake never worked reliably on
      // hardware and I was not able to pin down why; since any keypress
      // already brings the display back, a one-way "shake to dim the room"
      // is both more useful and more predictable than a toggle that could
      // leave you unsure which state you had landed in.
      if (displayIsOn()) displayOff();
      break;
    case SHAKE_LED:
      settings.ledEnabled = !settings.ledEnabled;
      markSettingsDirty();
      toast(settings.ledEnabled ? "LED on" : "LED off");
      break;
  }
}

static void updateShake(float dt) {
  if (!settings.shakeEnabled) { shakeEnergy = 0.f; return; }
  float ax = 0, ay = 0, az = 0;
  // NOTE: the IMU is NOT aliased onto the M5_CARDPUTER class the way Display,
  // Speaker, Power and BtnA are -- it only exists on M5Unified's global M5
  // object, so it must be reached as M5.Imu rather than M5Cardputer.Imu.
  if (!imuReady) return;
  M5.Imu.update();
  if (!M5.Imu.getAccel(&ax, &ay, &az)) return;
  // Deviation from whatever orientation the device is resting in: at rest the
  // magnitude is ~1g regardless of how it is held, so this ignores tilt and
  // only responds to actual movement.
  float mag = sqrtf(ax * ax + ay * ay + az * az);
  float jolt = fabsf(mag - 1.0f);
  // Leaky integrator: a deliberate shake builds energy over a few samples,
  // while a single bump or a tap on the desk decays before reaching threshold.
  shakeEnergy += jolt * dt * 12.f;
  shakeEnergy -= shakeEnergy * dt * 4.f;
  float thresh = 1.6f - 1.2f * (settings.shakeSens * 0.01f);   // 1.6 (insensitive) .. 0.4 (sensitive)
  uint32_t now = millis();
  // One shake must mean one action. A real shake lasts a second or two, which
  // a plain time-based debounce would happily read as several triggers -- that
  // is why Display On/Off appeared not to work: it was toggling two or three
  // times per shake and landing back where it started. So after firing, the
  // trigger stays disarmed until the device has actually settled again.
  if (shakeArmed && shakeEnergy > thresh) {
    lastShakeMs = now;
    shakeArmed = false;
    shakeEnergy = 0.f;
    doShakeAction();
    return;
  }
  if (!shakeArmed && shakeEnergy < thresh * 0.2f && now - lastShakeMs > 1000) shakeArmed = true;
}

// ---------------------------------------------------------------------------
// 5. UI
// ---------------------------------------------------------------------------
static M5Canvas canvas(&M5Cardputer.Display);

enum Screen { SCR_MAIN = 0, SCR_EDIT, SCR_SETTINGS, NUM_SCREENS };
enum SubWindow { SUB_NONE = 0, SUB_LAYER_PARAMS, SUB_DISPLAY, SUB_SLEEP, SUB_LED, SUB_SHAKE, SUB_SPACE, SUB_SESSION, SUB_SCENES, SUB_ABOUT };

static Screen    curScreen = SCR_MAIN;
static SubWindow curSub = SUB_NONE;
static bool helpVisible = false;
static bool displayOn = true;static uint32_t lastInputMs = 0;
// Main screen

// Edit screen
static int editCursor = 0, editScroll = 0;
static int paramCursor = 0;              // 0..2 params, 3 = color

// Settings screen
static int settingsCursor = 0;
static int subCursor = 0;
static int scenesCursor = 0, scenesScroll = 0;

// Text input modal (scene name)
enum TextPurpose { TXT_NONE = 0, TXT_SAVE_AS, TXT_RENAME };
static bool textActive = false;
static TextPurpose textPurpose = TXT_NONE;
static char textBuf[24];
static int  textLen = 0;
static int  textTarget = -1;

// Confirm modal
static bool confirmActive = false;
static int  confirmTarget = -1;

// Toast (short message at the bottom)
static String toastMsg; static uint32_t toastUntil = 0;
static void toast(const char* m) { toastMsg = m; toastUntil = millis() + 1600; }

// --- color helpers ---
static inline uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) { return canvas.color565(r, g, b); }
static inline uint16_t bgColor() { return rgb(BG_R, BG_G, BG_B); }
// Blend a palette color toward the background. t=1 full color, t=0 background.
static uint16_t palColor(int idx, float t) {
  if (idx < 0 || idx >= NUM_PALETTE) idx = 0;
  if (t < 0) t = 0; if (t > 1) t = 1;
  const PaletteColor& c = PALETTE[idx];
  return rgb((uint8_t)(BG_R + (c.r - BG_R) * t), (uint8_t)(BG_G + (c.g - BG_G) * t), (uint8_t)(BG_B + (c.b - BG_B) * t));
}
static uint16_t grey(uint8_t v) { return rgb(v, v, v + 4); }
static const uint16_t INK = 0xFFFF;

// Draws "Ensō" with a macron over the 'o' (the built-in fonts have no 'ō' glyph)
static void drawEnsoWord(int x, int y, int size, uint16_t color) {
  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(size);
  canvas.setTextDatum(textdatum_t::top_left);
  canvas.setTextColor(color, bgColor());
  canvas.drawString(FW_NAME, x, y);
  int cw = 6 * size;                    // glyph advance of Font0
  int ox = x + cw * 3 + size;           // 'o' is the 4th character
  int w = cw - 2 * size;
  canvas.fillRect(ox, y - size - 1, w, size > 1 ? size - 1 : 1, color);
  canvas.setTextSize(1);
}

static void drawText(const char* s, int x, int y, uint16_t color, int size = 1, textdatum_t datum = textdatum_t::top_left) {
  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(size);
  canvas.setTextDatum(datum);
  canvas.setTextColor(color, bgColor());
  canvas.drawString(s, x, y);
  canvas.setTextSize(1);
}
static void drawTextT(const char* s, int x, int y, uint16_t color, int size = 1, textdatum_t datum = textdatum_t::top_left) {
  // transparent variant (no background fill)
  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(size);
  canvas.setTextDatum(datum);
  canvas.setTextColor(color);
  canvas.drawString(s, x, y);
  canvas.setTextSize(1);
}

static void drawHeader(const char* title, uint16_t color) {
  drawText(title, 4, 2, color);
  canvas.drawFastHLine(0, 11, SCREEN_W, grey(40));
}
static void drawNav(const char* text) {
  canvas.drawFastHLine(0, 124, SCREEN_W, grey(40));
  drawText(text, 3, 127, grey(110));
}
static void drawBar(int x, int y, int w, int h, float frac, uint16_t fill, uint16_t frame) {
  canvas.drawRect(x, y, w, h, frame);
  int fw = (int)((w - 2) * frac + 0.5f);
  if (fw > 0) canvas.fillRect(x + 1, y + 1, fw, h - 2, fill);
}

// --- Boot animation: an ink brush drawing an enso ------------------------
static void bootSplash() {
  const float cx = 120, cy = 58, R = 44;
  const int STEPS = 90;
  canvas.fillScreen(bgColor());
  canvas.pushSprite(0, 0);
  delay(200);
  uint16_t ink = rgb(228, 222, 206);
  float prevX = 0, prevY = 0;
  for (int i = 0; i <= STEPS; i++) {
    float t = (float)i / STEPS;                            // 0..1 along the stroke
    float ang = -1.9f + t * 5.4f;                          // ~310 degrees, gap at the end
    float jitter = ((esp_random() & 0xFF) / 255.f - 0.5f) * 1.6f;
    float r = R + jitter + sinf(t * 9.f) * 0.8f;
    float x = cx + cosf(ang) * r, y = cy + sinf(ang) * r;
    float w = 3.6f - 2.8f * t + (t < 0.08f ? 1.2f : 0.f); // thick start, tapering tail
    if (i > 0) {
      canvas.drawLine((int)prevX, (int)prevY, (int)x, (int)y, ink);
      canvas.fillCircle((int)x, (int)y, (int)(w + 0.5f), ink);
    } else canvas.fillCircle((int)x, (int)y, (int)w + 1, ink);
    prevX = x; prevY = y;
    canvas.pushSprite(0, 0);
    delay(12);
  }
  // Fade in the logo and version at the bottom
  for (int f = 1; f <= 16; f++) {
    float t = f / 16.f;
    uint16_t c = rgb((uint8_t)(BG_R + (228 - BG_R) * t), (uint8_t)(BG_G + (222 - BG_G) * t), (uint8_t)(BG_B + (206 - BG_B) * t));
    canvas.fillRect(0, 110, SCREEN_W, 25, bgColor());
    drawEnsoWord(96, 112, 2, c);
    drawText(FW_VERSION, 150, 118, c);
    canvas.pushSprite(0, 0);
    delay(35);
  }
  delay(900);
}

// --- Main curScreen -----------------------------------------------------------
static const int RING_SEG = 72;
static float ringCos[RING_SEG + 1], ringSin[RING_SEG + 1];

static void drawRings() {
  const float cx = 120.f, cy = 64.f;
  float t = millis() * 0.001f;
  int drawn = 0;
  for (int i = 0; i < NUM_LAYERS; i++) {
    LayerDSP& d = dsp[i];
    float g = d.gain;
    float ev = effectiveLayerVol(i) * 0.6f;   // keeps a dimmed ring visible while paused
    float vis = g > ev ? g : ev;
    if (vis < 0.005f) continue;
    drawn++;
    float bright = 0.2f + 0.8f * sqrtf(vis);  // volume -> color intensity
    float lvl = d.level;                     // sound activity -> wobble amplitude
    float base = 33.f + i * 1.6f;
    float A = 1.2f + 8.f * lvl;
    float ph1 = t * (0.35f + 0.05f * i) + i * 1.7f;
    float ph2 = -t * (0.21f + 0.04f * i) + i * 0.9f;
    float ph3 = t * (0.5f + 0.07f * i) + i * 2.3f;
    uint16_t col = palColor(live.L[i].color, bright);
    uint16_t glow = palColor(live.L[i].color, bright * 0.35f);
    float px = 0, py = 0, gx = 0, gy = 0;
    for (int k = 0; k <= RING_SEG; k++) {
      float ang = k * (6.2831853f / RING_SEG);
      float w = 0.5f * sinf(3.f * ang + ph1) + 0.3f * sinf(5.f * ang + ph2) + 0.6f * sinf(2.f * ang + ph3);
      float r = base + A * w;
      float x = cx + ringCos[k] * r, y = cy + ringSin[k] * r;
      float x2 = cx + ringCos[k] * (r + 1.5f), y2 = cy + ringSin[k] * (r + 1.5f);
      if (k > 0) {
        canvas.drawLine((int)gx, (int)gy, (int)x2, (int)y2, glow);
        canvas.drawLine((int)px, (int)py, (int)x, (int)y, col);
      }
      px = x; py = y; gx = x2; gy = y2;
    }
  }
  if (drawn == 0) canvas.drawCircle((int)cx, (int)cy, 36, grey(38));
}

// The breathing guide: one large circle that grows and shrinks with the
// pattern, a fixed outline showing the full extent, and the phase name with a
// countdown. The reactive layer rings stay behind it, dimmed, so you can still
// see what is playing.
static void drawBreathGuide() {
  const float cx = 120.f, cy = 64.f;
  const float rMin = 12.f, rMax = 46.f;
  float r = rMin + (rMax - rMin) * breathOpen;
  uint16_t c = rgb(150, 205, 225);
  canvas.drawCircle((int)cx, (int)cy, (int)rMax, grey(38));
  // Filled disc, drawn as a soft stack of circles so the edge is not harsh
  canvas.fillCircle((int)cx, (int)cy, (int)r, palColor(0, 0.28f));
  canvas.drawCircle((int)cx, (int)cy, (int)r, c);
  canvas.drawCircle((int)cx, (int)cy, (int)r - 1, palColor(0, 0.55f));
  char b[12];
  int rem = (int)(breathRemaining() + 0.999f);
  snprintf(b, sizeof(b), "%d", rem < 1 ? 1 : rem);
  drawTextT(b, (int)cx, (int)cy - 4, rgb(228, 222, 206), 2, textdatum_t::top_center);
  drawTextT(breathLabel(), (int)cx, 96, grey(170), 1, textdatum_t::top_center);
}

static void drawMain() {
  if (breathActive()) {
    drawRings();
    drawBreathGuide();
  } else drawRings();
  // Play state and master volume, top-left
  uint16_t c = grey(120);
  if (playing) canvas.fillTriangle(5, 3, 5, 11, 12, 7, c);
  else { canvas.fillRect(4, 3, 3, 8, c); canvas.fillRect(9, 3, 3, 8, c); }
  char buf[24];
  snprintf(buf, sizeof(buf), "Vol %d", settings.masterVol);
  drawText(buf, 18, 3, c);
  drawBar(58, 4, 50, 7, settings.masterVol / 100.f, grey(110), grey(50));
  // Top-right cluster: battery, then session timer, then sleep timer
  int rx = SCREEN_W - 4;
  int bat = M5Cardputer.Power.getBatteryLevel();
  if (bat >= 0) {
    snprintf(buf, sizeof(buf), "%d%%", bat);
    drawText(buf, rx, 3, bat <= 15 ? rgb(230, 120, 110) : grey(130), 1, textdatum_t::top_right);
    rx -= 26;
  }
  int sRem = sessionRemainingSec();
  if (sRem >= 0) {
    snprintf(buf, sizeof(buf), "* %d:%02d", sRem / 60, sRem % 60);
    drawText(buf, rx, 3, rgb(200, 190, 150), 1, textdatum_t::top_right);
    rx -= 44;
  }
  int rem = sleepRemainingSec();
  if (rem >= 0) {
    snprintf(buf, sizeof(buf), "zz %d:%02d", rem / 60, rem % 60);
    drawText(buf, rx, 3, grey(120), 1, textdatum_t::top_right);
  }
  // Visualisation mode name, shown briefly after switching with Shift + ; / .
  if (millis() < vizNameUntil)
    drawTextT(VIZ_NAMES[settings.mainViz], SCREEN_W / 2, 16, grey(150), 1, textdatum_t::top_center);
  // Scene name, shown briefly right after switching (or shortly after boot).
  // This reads live.name, not the scene-list entry: a generated scene has no
  // entry, and showing the list entry made Random display the previous
  // scene's name.
  if (millis() < nameShowUntil && !sceneList.empty()) {
    String n = live.name;
    const SceneEntry& e = sceneList[currentScene];
    if (!e.isPreset && n == e.name) n += " *";   // custom scene, unmodified
    drawTextT(n.c_str(), SCREEN_W / 2, 111, rgb(228, 222, 206), 2, textdatum_t::top_center);
    drawTextT("<", 6, 115, grey(120), 1, textdatum_t::top_left);
    drawTextT(">", SCREEN_W - 6, 115, grey(120), 1, textdatum_t::top_right);
  }
  // While Drift is running, a small mark shows the mix is still moving
  if (driftActive) drawTextT("~", SCREEN_W - 6, 15, grey(110), 1, textdatum_t::top_right);
  drawNav("[,][/] Scene [;][.] Vol [Sh+;.] Breathe [R]nd [L]ED [Tab][H]");
}

// --- Edit curScreen -----------------------------------------------------------
static const int EDIT_ROWS = 8;
static void drawEdit() {
  drawHeader("EDIT", grey(200));
  // The group of the highlighted layer, so the list stays orientating even
  // when scrolled past the first row of that group.
  drawText(GROUP_NAMES[LAYER_GROUP[editCursor]], SCREEN_W - 4, 2, grey(130), 1, textdatum_t::top_right);
  if (editCursor < editScroll) editScroll = editCursor;
  if (editCursor >= editScroll + EDIT_ROWS) editScroll = editCursor - EDIT_ROWS + 1;
  for (int r = 0; r < EDIT_ROWS; r++) {
    int i = editScroll + r;
    if (i >= NUM_LAYERS) break;
    int y = 14 + r * 14;
    const LayerSettings& L = live.L[i];
    bool sel = (i == editCursor);
    if (sel) canvas.fillRect(0, y - 1, SCREEN_W, 13, grey(28));
    // Faint rule wherever a new group starts (never above the first visible
    // row, where it would read as part of the header)
    if (r > 0 && LAYER_GROUP[i] != LAYER_GROUP[i - 1]) canvas.drawFastHLine(6, y - 2, SCREEN_W - 12, grey(34));
    float active = effectiveLayerVol(i) > 0 ? 1.f : 0.55f;
    uint16_t nc = palColor(L.color, active);
    if (sel) drawTextT(">", 2, y + 2, grey(200));
    drawTextT(LAYER_DEFS[i].name, 10, y + 2, nc);
    // Mute / Solo indicators
    bool solo = soloMask & (1 << i);
    canvas.drawRect(64, y + 1, 9, 11, L.mute ? rgb(230, 120, 110) : grey(45));
    if (L.mute) drawTextT("M", 66, y + 2, rgb(230, 120, 110));
    canvas.drawRect(75, y + 1, 9, 11, solo ? rgb(240, 200, 90) : grey(45));
    if (solo) drawTextT("S", 77, y + 2, rgb(240, 200, 90));
    // Volume bar
    drawBar(90, y + 2, 110, 9, L.vol / 100.f, palColor(L.color, 0.25f + 0.75f * active), grey(50));
    // Activity dot inside the bar (shows the layer is actually sounding)
    if (dsp[i].level > 0.02f) {
      int ax = 91 + (int)(108 * (L.vol / 100.f));
      canvas.fillRect(ax - 1, y + 3, 2, 7, palColor(L.color, 1.f));
    }
    char v[8]; snprintf(v, sizeof(v), "%3d", L.vol);
    drawTextT(v, 204, y + 2, sel ? grey(220) : grey(130));
  }
  drawNav("[;][.] Sel [,][/] Vol [Ent] Edit [M]ute [S]olo [R]nd [L]ED [Tab]");
}

static void drawLayerParams() {
  int i = editCursor;
  const LayerSettings& L = live.L[i];
  uint16_t nc = palColor(L.color, 1.f);
  drawHeader(LAYER_DEFS[i].name, nc);
  drawText("PARAMETERS", SCREEN_W - 4, 2, grey(110), 1, textdatum_t::top_right);
  for (int r = 0; r < NPARAM + 1; r++) {
    int y = 22 + r * 22;
    bool sel = (r == paramCursor);
    if (sel) canvas.fillRect(0, y - 3, SCREEN_W, 19, grey(28));
    if (sel) drawTextT(">", 2, y + 2, grey(200));
    if (r < NPARAM) {
      drawTextT(LAYER_DEFS[i].pname[r], 12, y + 2, sel ? grey(230) : grey(160));
      drawBar(90, y, 110, 13, L.p[r] / 100.f, palColor(L.color, sel ? 0.9f : 0.5f), grey(55));
      char v[8]; snprintf(v, sizeof(v), "%3d", L.p[r]);
      drawTextT(v, 206, y + 2, sel ? grey(220) : grey(130));
    } else {
      drawTextT("Color", 12, y + 2, sel ? grey(230) : grey(160));
      canvas.fillRect(90, y, 13, 13, nc);
      canvas.drawRect(90, y, 13, 13, grey(90));
      drawTextT(PALETTE[L.color].name, 110, y + 2, nc);
      // palette strip
      for (int k = 0; k < NUM_PALETTE; k++) {
        int px = 110 + k * 9;
        canvas.fillRect(px, y + 14, 7, 3, palColor(k, k == L.color ? 1.f : 0.45f));
      }
    }
  }
  drawNav("[;][.] Select  [,][/] Adjust  [Tab] / [`] Back");
}

// --- Settings screens ------------------------------------------------------
static const char* SETTINGS_ITEMS[] = {"Display", "LED", "Space", "Session", "Sleep Timer", "Shake", "Scenes", "About"};
static const int NUM_SETTINGS_ITEMS = 8;

static void drawSettings() {
  drawHeader("SETTINGS", grey(200));
  for (int r = 0; r < NUM_SETTINGS_ITEMS; r++) {
    int y = 14 + r * 13;
    bool sel = (r == settingsCursor);
    if (sel) canvas.fillRect(0, y - 2, SCREEN_W, 12, grey(28));
    if (sel) drawTextT(">", 2, y, grey(200));
    drawTextT(SETTINGS_ITEMS[r], 12, y, sel ? grey(230) : grey(160));
    const char* hint = "";
    char b[32];
    if (r == 0) { snprintf(b, sizeof(b), "%d%%", settings.brightness); hint = b; }
    if (r == 1) { hint = settings.ledEnabled ? "On" : "Off"; }
    if (r == 2) { if (settings.spaceMix == 0) hint = "Off"; else { snprintf(b, sizeof(b), "%d%%", settings.spaceMix); hint = b; } }
    if (r == 3) { if (sessionRunning) { int q = sessionRemainingSec(); snprintf(b, sizeof(b), "%d:%02d left", q / 60, q % 60); }
                  else snprintf(b, sizeof(b), "%d min", settings.sessionMin); hint = b; }
    if (r == 4) { snprintf(b, sizeof(b), "%s %dm", settings.sleepEnabled ? "On" : "Off", settings.sleepMinutes); hint = b; }
    if (r == 5) { snprintf(b, sizeof(b), "%s", settings.shakeEnabled ? SHAKE_ACTION_NAMES[settings.shakeAction] : "Off"); hint = b; }
    if (r == 6) { snprintf(b, sizeof(b), "%d scenes", (int)sceneList.size()); hint = b; }
    if (r == 7) { hint = FW_VERSION; }
    drawTextT(hint, SCREEN_W - 8, y, grey(120), 1, textdatum_t::top_right);
  }
  drawNav("[;][.] Select  [Enter] Open  [Tab] Next  [H] Help");
}

// Row height shrinks for longer lists so 6 rows still clear the nav bar.
static void drawValueRows(const char* title, const char* labels[], const char* values[], int n) {
  drawHeader(title, grey(200));
  int rowH = n <= 4 ? 20 : 17;
  int y0 = n <= 4 ? 22 : 18;
  for (int r = 0; r < n; r++) {
    int y = y0 + r * rowH;
    bool sel = (r == subCursor);
    if (sel) canvas.fillRect(0, y - 3, SCREEN_W, rowH - 3, grey(28));
    if (sel) drawTextT(">", 2, y + 2, grey(200));
    drawTextT(labels[r], 12, y + 2, sel ? grey(230) : grey(160));
    drawTextT(values[r], SCREEN_W - 8, y + 2, sel ? grey(230) : grey(140), 1, textdatum_t::top_right);
  }
  drawNav("[;][.] Select  [,][/] Adjust  [Tab] / [`] Back");
}
static void secText(char* b, size_t n, int sec) {
  if (sec == 0) snprintf(b, n, "Never");
  else if (sec < 60) snprintf(b, n, "%ds", sec);
  else snprintf(b, n, "%dm%02ds", sec / 60, sec % 60);
}
static void drawDisplaySettings() {
  static char b0[16], b1[16], b2[16], b3[16];
  snprintf(b0, sizeof(b0), "%d%%", settings.brightness);
  secText(b1, sizeof(b1), settings.dimAfterSec);
  snprintf(b2, sizeof(b2), "%d%%", settings.dimLevel);
  secText(b3, sizeof(b3), settings.offAfterSec);
  const char* labels[] = {"Brightness", "Dim after", "Dim level", "Screen off after"};
  const char* values[] = {b0, b1, b2, b3};
  drawValueRows("DISPLAY", labels, values, 4);
}
static void drawSleepSettings() {
  static char b1[16];
  snprintf(b1, sizeof(b1), "%d min", settings.sleepMinutes);
  const char* labels[] = {"Sleep timer", "Duration", "Fade out (last 1 min)"};
  const char* values[] = {settings.sleepEnabled ? "On" : "Off", b1, settings.sleepFade ? "On" : "Off"};
  drawValueRows("SLEEP TIMER", labels, values, 3);
  int rem = sleepRemainingSec();
  if (rem >= 0) {
    char b[32]; snprintf(b, sizeof(b), "Remaining %d:%02d", rem / 60, rem % 60);
    drawTextT(b, 12, 90, grey(120));
  }
}
static void drawLedSettings() {
  static char b1[24], b2[24], b3[12], b4[12], b5[12];
  snprintf(b1, sizeof(b1), "%s", settings.ledSource < NUM_LAYERS ? LAYER_DEFS[settings.ledSource].name : "Period");
  snprintf(b2, sizeof(b2), "%s", settings.ledColor < NUM_PALETTE ? PALETTE[settings.ledColor].name : "Follow source");
  snprintf(b3, sizeof(b3), "%d%%", settings.ledBright);
  snprintf(b4, sizeof(b4), "%d%%", settings.ledSpeed);
  snprintf(b5, sizeof(b5), "%d%%", settings.ledDepth);
  const char* labels[] = {"LED", "Source", "Color", "Brightness", "Speed", "Sway"};
  const char* values[] = {settings.ledEnabled ? "On" : "Off", b1, b2, b3, b4, b5};
  drawValueRows("LED", labels, values, 6);
  // Colour swatch next to the Color row so the choice is visible at a glance
  int ci = settings.ledColor;
  if (ci >= NUM_PALETTE) ci = settings.ledSource < NUM_LAYERS ? live.L[settings.ledSource].color : live.L[0].color;
  canvas.fillRect(96, 54, 10, 10, palColor(ci, 1.f));
  canvas.drawRect(96, 54, 10, 10, grey(90));
}

static void drawShakeSettings() {
  static char b1[12];
  snprintf(b1, sizeof(b1), "%d%%", settings.shakeSens);
  const char* labels[] = {"Shake", "Sensitivity", "Action"};
  const char* values[] = {settings.shakeEnabled ? "On" : "Off", b1, SHAKE_ACTION_NAMES[settings.shakeAction]};
  drawValueRows("SHAKE", labels, values, 3);
  if (imuReady) {
    drawTextT("Shake to trigger the action.", 12, 92, grey(110));
    drawTextT("Any key wakes the display.", 12, 103, grey(110));
  } else {
    drawTextT("No IMU detected -- shake is", 12, 92, rgb(230, 120, 110));
    drawTextT("unavailable on this board.", 12, 103, rgb(230, 120, 110));
  }
}

static void drawSpaceSettings() {
  static char b0[12], b1[12], b2[12];
  if (settings.spaceMix == 0) snprintf(b0, sizeof(b0), "Off");
  else snprintf(b0, sizeof(b0), "%d%%", settings.spaceMix);
  snprintf(b1, sizeof(b1), "%d%%", settings.spaceSize);
  snprintf(b2, sizeof(b2), "%d%%", settings.spaceDamp);
  const char* labels[] = {"Amount", "Size", "Damping"};
  const char* values[] = {b0, b1, b2};
  drawValueRows("SPACE", labels, values, 3);
  drawTextT("Reverb over the whole mix.", 12, 90, grey(110));
  drawTextT("Amount 0 turns it off entirely.", 12, 101, grey(110));
}

static void drawSessionSettings() {
  static char b0[16], b1[16], b2[16];
  snprintf(b0, sizeof(b0), "%d min", settings.sessionMin);
  if (settings.sessionInterval == 0) snprintf(b1, sizeof(b1), "Off");
  else snprintf(b1, sizeof(b1), "every %dm", settings.sessionInterval);
  snprintf(b2, sizeof(b2), "%d%%", settings.sessionPitch);
  const char* labels[] = {"Duration", "Interval bell", "Bell pitch", sessionRunning ? "Stop session" : "Start session"};
  const char* values[] = {b0, b1, b2, sessionRunning ? "[Enter]" : "[Enter]"};
  drawValueRows("SESSION", labels, values, 4);
  if (sessionRunning) {
    int q = sessionRemainingSec();
    char b[32]; snprintf(b, sizeof(b), "Running - %d:%02d remaining", q / 60, q % 60);
    drawTextT(b, 12, 106, rgb(200, 190, 150));
  } else {
    drawTextT("A bell marks start and end.", 12, 106, grey(110));
  }
}

static const int SCENE_ROWS = 7;
static void drawSceneManager() {
  drawHeader("SCENES", grey(200));
  drawText("[N] New [R] Rename [D] Del", SCREEN_W - 4, 2, grey(110), 1, textdatum_t::top_right);
  if (scenesCursor < scenesScroll) scenesScroll = scenesCursor;
  if (scenesCursor >= scenesScroll + SCENE_ROWS) scenesScroll = scenesCursor - SCENE_ROWS + 1;
  for (int r = 0; r < SCENE_ROWS; r++) {
    int i = scenesScroll + r;
    if (i >= (int)sceneList.size()) break;
    int y = 15 + r * 15;
    const SceneEntry& e = sceneList[i];
    bool sel = (i == scenesCursor);
    if (sel) canvas.fillRect(0, y - 2, SCREEN_W, 14, grey(28));
    if (sel) drawTextT(">", 2, y + 2, grey(200));
    canvas.drawRect(12, y + 1, 9, 9, grey(90));
    if (e.visible) canvas.fillRect(14, y + 3, 5, 5, grey(200));
    uint16_t nc = e.visible ? grey(220) : grey(100);
    if (i == currentScene) nc = rgb(228, 222, 206);
    String n = e.name; if (!e.isPreset) n += " *";
    drawTextT(n.c_str(), 26, y + 2, nc);
    if (i == currentScene) drawTextT("playing", SCREEN_W - 8, y + 2, grey(110), 1, textdatum_t::top_right);
  }
  drawNav("[;][.] Move [Enter] Show/Hide [,][/] Reorder [Tab] Back");
}
static void drawAbout() {
  drawHeader("ABOUT", grey(200));
  drawEnsoWord(12, 24, 2, rgb(228, 222, 206));
  drawText(FW_VERSION, 70, 30, grey(150));
  drawText("Ambient Sound Simulator", 12, 48, grey(170));
  drawText("for M5Stack Cardputer ADV", 12, 58, grey(170));
  drawText("All sounds are synthesized", 12, 74, grey(120));
  drawText("in real time - no samples.", 12, 84, grey(120));
  char b[48];
  int bat = M5Cardputer.Power.getBatteryLevel();
  snprintf(b, sizeof(b), "SD %s   IMU %s", sdReady ? "OK" : "--", imuReady ? "OK" : "--");
  drawText(b, 12, 100, grey(120));
  if (bat >= 0) { snprintf(b, sizeof(b), "Battery %d%%", bat); drawText(b, 12, 110, grey(120)); }
  drawNav("[Tab] / [`] Back");
}

// --- Modals ----------------------------------------------------------------
static void drawTextInput() {
  canvas.fillRect(16, 34, 208, 66, bgColor());
  canvas.drawRect(16, 34, 208, 66, grey(120));
  drawTextT(textPurpose == TXT_RENAME ? "Rename scene" : "Save scene as", 24, 40, grey(200));
  canvas.fillRect(24, 56, 192, 16, grey(22));
  canvas.drawRect(24, 56, 192, 16, grey(70));
  char b[26]; memcpy(b, textBuf, textLen); b[textLen] = 0;
  drawTextT(b, 28, 60, rgb(228, 222, 206));
  if ((millis() / 400) & 1) canvas.fillRect(28 + textLen * 6, 59, 5, 10, grey(200));
  drawTextT("[Enter] OK   [`] Cancel   [Del] Backspace", 24, 82, grey(110));
}
static void drawConfirm() {
  canvas.fillRect(16, 40, 208, 54, bgColor());
  canvas.drawRect(16, 40, 208, 54, rgb(230, 120, 110));
  drawTextT("Delete this scene?", 24, 48, grey(220));
  if (confirmTarget >= 0 && confirmTarget < (int)sceneList.size())
    drawTextT(sceneList[confirmTarget].name.c_str(), 24, 60, rgb(228, 222, 206));
  drawTextT("[Y] Yes     [N] / [`] No", 24, 78, grey(120));
}
static void drawHelp() {
  canvas.fillRect(8, 14, 224, 108, bgColor());
  canvas.drawRect(8, 14, 224, 108, grey(120));
  drawTextT("HELP", 16, 19, grey(220));
  drawTextT("[H] Close", 224, 19, grey(110), 1, textdatum_t::top_right);
  const char* lines[10]; int n = 0;
  if (curScreen == SCR_MAIN) {
    lines[n++] = ", /   Switch scene (crossfades in)";
    lines[n++] = "; .   Master volume";
    lines[n++] = "Space Play / pause";
    lines[n++] = "R     Random scene   L  LED on/off";
    lines[n++] = "Shift + ; / .   Breathing guide";
    lines[n++] = "Shift + S   Run the Shake action";
    lines[n++] = "Drift scene: keeps evolving on";
    lines[n++] = "its own while it is selected.";
    lines[n++] = "Tab   Next menu (Main>Edit>Settings)";
    lines[n++] = "G0    Display off / on";
  } else if (curScreen == SCR_EDIT) {
    lines[n++] = "; .   Select layer   , /  Layer volume";
    lines[n++] = "Enter Open parameters of the layer";
    lines[n++] = "M     Mute   S  Solo (transient)";
    lines[n++] = "R     Random scene   L  LED on/off";
    lines[n++] = "Layers are grouped by genre.";
    lines[n++] = "Clock and Fan keep exact timing;";
    lines[n++] = "Preset edits: save as new scene in";
    lines[n++] = "Settings > Scenes > [N].";
  } else {
    lines[n++] = "; .   Select   Enter  Open";
    lines[n++] = "In sub windows: , / adjust values";
    lines[n++] = "Scenes: N new, R rename, D delete,";
    lines[n++] = "Enter show/hide, , / reorder.";
    lines[n++] = "Space = reverb. Session = timed";
    lines[n++] = "sitting with a bell at each end.";
    lines[n++] = "Files: SD:/ENSO/settings.json";
    lines[n++] = "       SD:/ENSO/scene/*.json";
  }
  for (int i = 0; i < n; i++) drawTextT(lines[i], 16, 31 + i * 10, grey(170));
}

// ---------------------------------------------------------------------------
// Hidden beats mode
// ---------------------------------------------------------------------------
// A base tone plus a beat offset, summed or gated depending on Mode above.
// Not reachable from any menu and not mentioned in the help or the nav bar --
// opened with Fn + B from the main screen.
struct BinPreset { const char* name; float base; float beat; const char* d1; const char* d2; };
static const BinPreset BIN_PRESETS[] = {
  {"Delta 2 Hz",     180.f,  2.00f, "Deep, dreamless sleep band.",   "Lying down, eyes closed."},
  {"Theta 6 Hz",     200.f,  6.00f, "Drowsy, meditative states.",    "Often used for deep meditation."},
  {"Schumann 7.83",  200.f,  7.83f, "Earth's background resonance.", "Popular for grounding."},
  {"Alpha 10 Hz",    210.f, 10.00f, "Relaxed but awake.",            "Calm focus, light rest."},
  {"SMR 14 Hz",      220.f, 14.00f, "Quiet, settled alertness.",     "Between relaxed and active."},
  {"Beta 18 Hz",     240.f, 18.00f, "Alert, active thinking.",       "Daytime concentration."},
  {"Gamma 40 Hz",    250.f, 40.00f, "High-frequency focus band.",    "Keep sessions short."},
};
static const int NUM_BIN_PRESETS = sizeof(BIN_PRESETS) / sizeof(BIN_PRESETS[0]);

static bool  binScreen = false;
static int   binCursor = 0;
static int   binPreset = 1;
static float binBase = 200.f, binBeat = 6.f;
static int   binVolume = 30, binAmbient = 25;
static int   binMode = BEAT_MONAURAL;
static bool  binPlaying = false;
#define BIN_ROWS 6

static void binApply() {
  binPublishParams(binBase, binBeat, binPlaying ? binVolume * 0.01f * 0.5f : 0.f,
                    (uint8_t)binMode, binPlaying);
}
// Persisted so re-opening this screen doesn't forget last time's tuning --
// previously only the in-session static variables held this, so it reset to
// defaults (Ambient included) on every visit.
static void binSaveToSettings() {
  settings.binPreset  = (uint8_t)binPreset;
  settings.binMode    = (uint8_t)binMode;
  settings.binBase    = binBase;
  settings.binBeat    = binBeat;
  settings.binVolume  = (uint8_t)binVolume;
  settings.binAmbient = (uint8_t)binAmbient;
  markSettingsDirty();
}
static void binLoadPreset(int i) {
  if (i < 0 || i >= NUM_BIN_PRESETS) return;
  binPreset = i;
  binBase = BIN_PRESETS[i].base;
  binBeat = BIN_PRESETS[i].beat;
  binApply();
  binSaveToSettings();
}

static void drawBinaural() {
  drawHeader("BEATS", rgb(170, 140, 230));
  drawText(binPlaying ? "PLAYING" : "stopped", SCREEN_W - 4, 2,
           binPlaying ? rgb(150, 220, 170) : grey(110), 1, textdatum_t::top_right);
  char b[44];
  snprintf(b, sizeof(b), "%.2f Hz beat", binBeat);
  drawTextT(b, 8, 13, rgb(200, 180, 240), 2, textdatum_t::top_left);

  static char v[BIN_ROWS][24];
  snprintf(v[0], 24, "%s", BIN_PRESETS[binPreset].name);
  snprintf(v[1], 24, "%s", BEAT_MODE_NAMES[binMode]);
  snprintf(v[2], 24, "%.2f Hz", binBase);
  snprintf(v[3], 24, "%.2f Hz", binBeat);
  snprintf(v[4], 24, "%d%%", binVolume);
  snprintf(v[5], 24, "%d%%", binAmbient);
  const char* labels[BIN_ROWS] = {"Preset", "Mode", "Base", "Beat", "Volume", "Ambient"};
  // Rows were 13px apart with the note line starting at y=118 -- on real
  // hardware that put the note's bottom half past the nav bar at y=124 (and
  // partly off the 135px canvas), which is what was reported as "half the
  // description text is hidden". Rows are now 11px apart and start higher, so
  // the note line lands at y=99 with real clearance before the nav bar.
  for (int r = 0; r < BIN_ROWS; r++) {
    int y = 30 + r * 11;
    bool sel = (r == binCursor);
    if (sel) canvas.fillRect(0, y - 1, SCREEN_W, 10, grey(28));
    if (sel) drawTextT(">", 2, y, grey(200));
    drawTextT(labels[r], 12, y, sel ? grey(230) : grey(150));
    drawTextT(v[r], SCREEN_W - 8, y, sel ? grey(230) : grey(130), 1, textdatum_t::top_right);
  }
  // Context line: whatever the cursor is on gets explained here
  const char* note;
  if (binCursor == 0)      note = BIN_PRESETS[binPreset].d1;
  else if (binCursor == 1) note = binMode == BEAT_ISOCHRONIC ? "One tone pulsed at the beat rate."
                                                              : "Two tones summed into one.";
  else if (binCursor == 5) note = "Level of the scene behind the tones.";
  else                     note = BIN_PRESETS[binPreset].d2;
  drawTextT(note, 8, 99, grey(135));
  drawNav("[;][.] Sel [,][/] Adj [Enter] Play [Tab] Exit");
}

static void render() {
  canvas.fillScreen(bgColor());
  if (binScreen) {
    drawBinaural();
    if (toastUntil > millis()) {
      canvas.fillRect(0, 124, SCREEN_W, 11, bgColor());
      drawTextT(toastMsg.c_str(), SCREEN_W / 2, 127, rgb(228, 222, 206), 1, textdatum_t::top_center);
    }
    canvas.pushSprite(0, 0);
    return;
  }
  if (curScreen == SCR_MAIN) drawMain();
  else if (curScreen == SCR_EDIT) { if (curSub == SUB_LAYER_PARAMS) drawLayerParams(); else drawEdit(); }
  else {
    switch (curSub) {
      case SUB_DISPLAY: drawDisplaySettings(); break;
      case SUB_SLEEP:   drawSleepSettings(); break;
      case SUB_LED:     drawLedSettings(); break;
      case SUB_SHAKE:   drawShakeSettings(); break;
      case SUB_SPACE:   drawSpaceSettings(); break;
      case SUB_SESSION: drawSessionSettings(); break;
      case SUB_SCENES:  drawSceneManager(); break;
      case SUB_ABOUT:   drawAbout(); break;
      default:          drawSettings(); break;
    }
  }
  if (toastUntil > millis()) {
    canvas.fillRect(0, 124, SCREEN_W, 11, bgColor());
    drawTextT(toastMsg.c_str(), SCREEN_W / 2, 127, rgb(228, 222, 206), 1, textdatum_t::top_center);
  }
  if (textActive) drawTextInput();
  else if (confirmActive) drawConfirm();
  else if (helpVisible) drawHelp();
  canvas.pushSprite(0, 0);
}

// ---------------------------------------------------------------------------
// 6. Input handling
// ---------------------------------------------------------------------------
struct KeyEv { char c; bool enter, tab, space, del, shift, fn; uint32_t heldMs; };

// Returns true when a key event should be processed this frame (fresh press, or auto-repeat).
// ev.heldMs is 0 on the initial press and the actual hold duration on every repeat firing,
// so callers can accelerate their own step size the longer a key is held (see sleepStepFor()).
static bool readKey(KeyEv& ev) {
  static int lastSig = 0;
  static uint32_t heldSince = 0, lastRepeat = 0;
  ev.heldMs = 0;
  if (!M5Cardputer.Keyboard.isPressed()) { lastSig = 0; return false; }
  Keyboard_Class::KeysState st = M5Cardputer.Keyboard.keysState();
  ev.c = st.word.empty() ? 0 : st.word[0];
  ev.enter = st.enter; ev.tab = st.tab; ev.space = st.space; ev.del = st.del;
  ev.shift = st.shift; ev.fn = st.fn;
  int sig = (unsigned char)ev.c | (ev.enter << 8) | (ev.tab << 9) | (ev.space << 10) | (ev.del << 11)
          | (ev.shift << 12) | (ev.fn << 13);
  if (sig == 0) { lastSig = 0; return false; }
  uint32_t now = millis();
  if (sig != lastSig) { lastSig = sig; heldSince = now; lastRepeat = now; return true; }
  ev.heldMs = now - heldSince;
  bool repeatable = (ev.c == ',' || ev.c == '/' || ev.c == ';' || ev.c == '.' || ev.del);
  if (repeatable && now - heldSince > KEY_REPEAT_DELAY_MS && now - lastRepeat > KEY_REPEAT_RATE_MS) {
    lastRepeat = now; return true;
  }
  return false;
}

static int nextVisibleScene(int from, int dir) {
  int n = sceneList.size();
  if (n == 0) return -1;
  int i = from;
  for (int k = 0; k < n; k++) {
    i = (i + dir + n) % n;
    if (sceneList[i].visible || i == currentScene) return i;
  }
  return from;
}

static void applyBrightness(int percent) {
  M5Cardputer.Display.setBrightness((uint8_t)(percent * 255 / 100));
}
static int lastBrightLevel = -1;   // last value pushed to setBrightness()
// A multi-key combo (Shift+S) doesn't press and release as one atomic event --
// each key's own edge lands a few ms apart, and readKey() treats every one of
// those edges as its own "a key was pressed" signal. Turning the display off
// from such a combo (Shift+S -> Shake -> Display Off) meant the very next
// edge -- Shift or S being released a moment later -- was read as a fresh
// keypress and woke the display right back up, since the wake-on-any-key
// logic didn't distinguish that from an intentional new press. This flag
// closes that gap: once set, the display stays asleep until the keyboard has
// been seen fully released at least once, and only a genuine new press after
// that is allowed to wake it.
static bool displayWakeGuard = false;
static void displayOff() {
  displayOn = false;
  lastBrightLevel = -1;
  displayWakeGuard = true;
  M5Cardputer.Display.setBrightness(0);
  M5Cardputer.Display.sleep();
}
static void displayWake() {
  displayOn = true;
  displayWakeGuard = false;
  M5Cardputer.Display.wakeup();
  applyBrightness(settings.brightness);
  lastBrightLevel = settings.brightness;
  lastInputMs = millis();
}
static bool displayIsOn() { return displayOn; }

static void openTextInput(TextPurpose p, int target, const char* initial) {
  textActive = true; textPurpose = p; textTarget = target;
  textLen = 0;
  if (initial) { textLen = strlen(initial); if (textLen > 22) textLen = 22; memcpy(textBuf, initial, textLen); }
  textBuf[textLen] = 0;
}

static void commitSaveAs(const String& name) {
  if (findSceneEntry(name) >= 0 || findPreset(name) >= 0) { toast("Name already exists"); return; }
  SceneData s = live;
  strncpy(s.name, name.c_str(), sizeof(s.name) - 1); s.name[sizeof(s.name) - 1] = 0;
  if (!saveSceneFile(s)) { toast("Save failed (SD?)"); return; }
  SceneEntry e; e.name = name; e.isPreset = false; e.isDrift = false; e.presetIdx = -1; e.visible = true;
  sceneList.push_back(e);
  currentScene = sceneList.size() - 1;
  scenesCursor = currentScene;
  live = s; liveEdited = false; sceneDirty = false;
  markSettingsDirty();
  toast("Scene saved");
}
static void commitRename(int idx, const String& name) {
  if (idx < 0 || idx >= (int)sceneList.size() || sceneList[idx].isPreset) return;
  if (name == sceneList[idx].name) return;
  if (findSceneEntry(name) >= 0 || findPreset(name) >= 0) { toast("Name already exists"); return; }
  String old = sceneList[idx].name;
  SceneData s;
  if (idx == currentScene) s = live; else loadSceneFile(old, s);
  strncpy(s.name, name.c_str(), sizeof(s.name) - 1); s.name[sizeof(s.name) - 1] = 0;
  if (!saveSceneFile(s)) { toast("Rename failed (SD?)"); return; }
  deleteSceneFile(old);
  sceneList[idx].name = name;
  if (idx == currentScene) live = s;
  markSettingsDirty();
  toast("Renamed");
}
static void commitDelete(int idx) {
  if (idx < 0 || idx >= (int)sceneList.size() || sceneList[idx].isPreset) return;
  deleteSceneFile(sceneList[idx].name);
  sceneList.erase(sceneList.begin() + idx);
  if (sceneList.empty()) { currentScene = 0; sceneSetDefaults(live, "Empty"); }
  else if (idx == currentScene) {
    currentScene = idx < (int)sceneList.size() ? idx : (int)sceneList.size() - 1;
    loadSceneIntoLive(currentScene);
    xfActive = false;   // the scene we were crossfading from/to may no longer exist
  } else if (idx < currentScene) currentScene--;
  if (scenesCursor >= (int)sceneList.size()) scenesCursor = (int)sceneList.size() - 1;
  if (scenesCursor < 0) scenesCursor = 0;
  sceneDirty = false;
  markSettingsDirty();
  toast("Deleted");
}

static void handleTextInput(const KeyEv& ev) {
  if (ev.c == '`') { textActive = false; return; }
  if (ev.del) { if (textLen > 0) textBuf[--textLen] = 0; return; }
  if (ev.enter) {
    String name(textBuf); name.trim();
    textActive = false;
    if (!name.length()) return;
    if (textPurpose == TXT_SAVE_AS) commitSaveAs(name);
    else if (textPurpose == TXT_RENAME) commitRename(textTarget, name);
    return;
  }
  char c = ev.space ? ' ' : ev.c;
  if (c >= 32 && c < 127 && textLen < 22) { textBuf[textLen++] = c; textBuf[textLen] = 0; }
}

static void handleConfirm(const KeyEv& ev) {
  if (ev.c == 'y' || ev.c == 'Y') { confirmActive = false; commitDelete(confirmTarget); }
  else if (ev.c == 'n' || ev.c == 'N' || ev.c == '`') confirmActive = false;
}

static void adjustLayerVol(int i, int delta) {
  int v = constrain((int)live.L[i].vol + delta, 0, 100);
  if (v != live.L[i].vol) { live.L[i].vol = v; markSceneDirty(); }
}

// Shared by the Main and Edit screens. R and L are free on both (Edit already
// uses M and S); the Scenes manager's own R/N/D live on the Settings screen,
// so there is no collision.
static bool handleRandomAndLed(const KeyEv& ev) {
  if (ev.c == 'r' || ev.c == 'R') { randomScene(); return true; }
  if (ev.c == 'l' || ev.c == 'L') {
    settings.ledEnabled = !settings.ledEnabled;
    markSettingsDirty();
    toast(settings.ledEnabled ? "LED on" : "LED off");
    return true;
  }
  return false;
}

static void handleMain(const KeyEv& ev) {
  // Hidden: Fn + B. Nothing else on this screen uses Fn, and B is unused, so
  // there is no way to reach it by accident.
  if (ev.fn && (ev.c == 'b' || ev.c == 'B')) {
    binScreen = true;
    binCursor = 0;
    binPlaying = false;
    // Restore last time's tuning rather than resetting to defaults
    binPreset  = settings.binPreset;
    binMode    = settings.binMode;
    binBase    = settings.binBase;
    binBeat    = settings.binBeat;
    binVolume  = settings.binVolume;
    binAmbient = settings.binAmbient;
    gAmbientScale = binAmbient * 0.01f;
    binApply();
    return;
  }
  // Shift + S: keyboard equivalent of a physical shake, for the original
  // Cardputer (no IMU). Gated on the same "Shake" enable as the physical
  // gesture, so turning Shake off in Settings also turns this off, and it
  // doubles as a way to discover the feature if it's off.
  if (ev.shift && (ev.c == 's' || ev.c == 'S')) {
    if (settings.shakeEnabled) doShakeAction();
    else toast("Enable Shake in Settings first");
    return;
  }
  // Shift + ; / . cycles the breathing guide (Rings -> the three patterns)
  if (ev.shift && (ev.c == ';' || ev.c == ':' || ev.c == '.' || ev.c == '>')) {
    int d = (ev.c == ';' || ev.c == ':') ? 1 : -1;
    settings.mainViz = (uint8_t)((settings.mainViz + d + NUM_VIZ) % NUM_VIZ);
    breathReset();
    vizNameUntil = millis() + 2000;
    markSettingsDirty();
    return;
  }
  if (handleRandomAndLed(ev)) return;
  if (ev.c == ',' || ev.c == '/') {
    int idx = nextVisibleScene(currentScene, ev.c == '/' ? 1 : -1);
    startCrossfade(idx);
  } else if (ev.c == ';' || ev.c == '.') {
    int v = constrain((int)settings.masterVol + (ev.c == ';' ? 2 : -2), 0, 100);
    if (v != settings.masterVol) { settings.masterVol = v; markSettingsDirty(); }
  }
}

static void handleEdit(const KeyEv& ev) {
  if (curSub == SUB_LAYER_PARAMS) {
    if (ev.c == ';') paramCursor = (paramCursor + NPARAM) % (NPARAM + 1);
    else if (ev.c == '.') paramCursor = (paramCursor + 1) % (NPARAM + 1);
    else if (ev.c == ',' || ev.c == '/') {
      int d = ev.c == '/' ? 1 : -1;
      LayerSettings& L = live.L[editCursor];
      if (paramCursor < NPARAM) L.p[paramCursor] = constrain((int)L.p[paramCursor] + d * 2, 0, 100);
      else L.color = (L.color + d + NUM_PALETTE) % NUM_PALETTE;
      markSceneDirty();
    } else if (ev.c == '`') curSub = SUB_NONE;
    return;
  }
  if (handleRandomAndLed(ev)) return;
  if (ev.c == ';') editCursor = (editCursor + NUM_LAYERS - 1) % NUM_LAYERS;
  else if (ev.c == '.') editCursor = (editCursor + 1) % NUM_LAYERS;
  else if (ev.c == ',') adjustLayerVol(editCursor, -2);
  else if (ev.c == '/') adjustLayerVol(editCursor, +2);
  else if (ev.enter) { curSub = SUB_LAYER_PARAMS; paramCursor = 0; }
  else if (ev.c == 'm' || ev.c == 'M') { live.L[editCursor].mute = !live.L[editCursor].mute; markSceneDirty(); }
  else if (ev.c == 's' || ev.c == 'S') soloMask ^= (1 << editCursor);
}

// Sleep timer's Duration row: single press = 1 minute; holding the key
// accelerates the step so the full 1-180 range doesn't take forever to reach.
static int sleepStepFor(uint32_t heldMs) {
  if (heldMs < 900)  return 1;
  if (heldMs < 2200) return 5;
  return 15;
}

static void handleSettings(const KeyEv& ev) {
  if (curSub == SUB_NONE) {
    if (ev.c == ';') settingsCursor = (settingsCursor + NUM_SETTINGS_ITEMS - 1) % NUM_SETTINGS_ITEMS;
    else if (ev.c == '.') settingsCursor = (settingsCursor + 1) % NUM_SETTINGS_ITEMS;
    else if (ev.enter) {
      subCursor = 0;
      switch (settingsCursor) {
        case 0: curSub = SUB_DISPLAY; break;
        case 1: curSub = SUB_LED; break;
        case 2: curSub = SUB_SPACE; break;
        case 3: curSub = SUB_SESSION; break;
        case 4: curSub = SUB_SLEEP; break;
        case 5: curSub = SUB_SHAKE; break;
        case 6: curSub = SUB_SCENES; scenesCursor = currentScene; break;
        default: curSub = SUB_ABOUT; break;
      }
    }
    return;
  }
  if (ev.c == '`') { curSub = SUB_NONE; return; }
  int d = ev.c == '/' ? 1 : (ev.c == ',' ? -1 : 0);

  if (curSub == SUB_DISPLAY) {
    if (ev.c == ';') subCursor = (subCursor + 3) % 4;
    else if (ev.c == '.') subCursor = (subCursor + 1) % 4;
    else if (d) {
      switch (subCursor) {
        case 0: settings.brightness = constrain(settings.brightness + d * 5, 10, 100); applyBrightness(settings.brightness); break;
        case 1: settings.dimAfterSec = constrain(settings.dimAfterSec + d * 10, 0, 600); break;
        case 2: settings.dimLevel = constrain(settings.dimLevel + d * 5, 5, 100); break;
        case 3: settings.offAfterSec = constrain(settings.offAfterSec + d * 30, 0, 1800); break;
      }
      markSettingsDirty();
    }
  } else if (curSub == SUB_SLEEP) {
    if (ev.c == ';') subCursor = (subCursor + 2) % 3;
    else if (ev.c == '.') subCursor = (subCursor + 1) % 3;
    else if (d || ev.enter) {
      switch (subCursor) {
        case 0: settings.sleepEnabled = !settings.sleepEnabled; armSleep(); break;
        case 1: settings.sleepMinutes = constrain((int)settings.sleepMinutes + d * sleepStepFor(ev.heldMs), 1, 180); armSleep(); break;
        case 2: settings.sleepFade = !settings.sleepFade; break;
      }
      markSettingsDirty();
    }
  } else if (curSub == SUB_LED) {
    if (ev.c == ';') subCursor = (subCursor + 5) % 6;
    else if (ev.c == '.') subCursor = (subCursor + 1) % 6;
    else if (d || (ev.enter && subCursor == 0)) {
      switch (subCursor) {
        case 0: settings.ledEnabled = !settings.ledEnabled; break;
        // Source cycles through every layer plus one extra slot for "Period"
        case 1: settings.ledSource = (settings.ledSource + d + NUM_LAYERS + 1) % (NUM_LAYERS + 1); break;
        // Colour cycles through the palette plus one extra slot for "Follow source"
        case 2: settings.ledColor = (settings.ledColor + d + NUM_PALETTE + 1) % (NUM_PALETTE + 1); break;
        case 3: settings.ledBright = constrain((int)settings.ledBright + d * 5, 0, 100); break;
        case 4: settings.ledSpeed = constrain((int)settings.ledSpeed + d * 5, 0, 100); break;
        case 5: settings.ledDepth = constrain((int)settings.ledDepth + d * 5, 0, 100); break;
      }
      markSettingsDirty();
    }
  } else if (curSub == SUB_SHAKE) {
    if (ev.c == ';') subCursor = (subCursor + 2) % 3;
    else if (ev.c == '.') subCursor = (subCursor + 1) % 3;
    else if (d || (ev.enter && subCursor == 0)) {
      switch (subCursor) {
        case 0: settings.shakeEnabled = !settings.shakeEnabled; break;
        case 1: settings.shakeSens = constrain((int)settings.shakeSens + d * 5, 0, 100); break;
        case 2: settings.shakeAction = (settings.shakeAction + d + NUM_SHAKE_ACTIONS) % NUM_SHAKE_ACTIONS; break;
      }
      markSettingsDirty();
    }
  } else if (curSub == SUB_SPACE) {
    if (ev.c == ';') subCursor = (subCursor + 2) % 3;
    else if (ev.c == '.') subCursor = (subCursor + 1) % 3;
    else if (d) {
      switch (subCursor) {
        case 0: settings.spaceMix = constrain((int)settings.spaceMix + d * 5, 0, 100); break;
        case 1: settings.spaceSize = constrain((int)settings.spaceSize + d * 5, 0, 100); break;
        case 2: settings.spaceDamp = constrain((int)settings.spaceDamp + d * 5, 0, 100); break;
      }
      markSettingsDirty();
    }
  } else if (curSub == SUB_SESSION) {
    if (ev.c == ';') subCursor = (subCursor + 3) % 4;
    else if (ev.c == '.') subCursor = (subCursor + 1) % 4;
    else if (subCursor == 3 && ev.enter) {
      if (sessionRunning) sessionStop(); else sessionStart();
    } else if (d) {
      switch (subCursor) {
        case 0: settings.sessionMin = constrain((int)settings.sessionMin + d * sleepStepFor(ev.heldMs), 1, 120); break;
        case 1: settings.sessionInterval = constrain((int)settings.sessionInterval + d, 0, 30); break;
        case 2: settings.sessionPitch = constrain((int)settings.sessionPitch + d * 5, 0, 100); break;
      }
      markSettingsDirty();
    }
  } else if (curSub == SUB_SCENES) {
    int n = sceneList.size();
    if (n == 0) { if (ev.c == 'n' || ev.c == 'N') openTextInput(TXT_SAVE_AS, -1, ""); return; }
    if (ev.c == ';') scenesCursor = (scenesCursor + n - 1) % n;
    else if (ev.c == '.') scenesCursor = (scenesCursor + 1) % n;
    else if (ev.enter) { sceneList[scenesCursor].visible = !sceneList[scenesCursor].visible; markSettingsDirty(); }
    else if (d) {
      int j = scenesCursor + d;
      if (j >= 0 && j < n) {
        std::swap(sceneList[scenesCursor], sceneList[j]);
        if (currentScene == scenesCursor) currentScene = j; else if (currentScene == j) currentScene = scenesCursor;
        scenesCursor = j;
        markSettingsDirty();
      }
    }
    else if (ev.c == 'n' || ev.c == 'N') openTextInput(TXT_SAVE_AS, -1, "");
    else if (ev.c == 'r' || ev.c == 'R') {
      if (sceneList[scenesCursor].isPreset) toast("Presets cannot be renamed");
      else openTextInput(TXT_RENAME, scenesCursor, sceneList[scenesCursor].name.c_str());
    }
    else if (ev.c == 'd' || ev.c == 'D') {
      if (sceneList[scenesCursor].isPreset) toast("Presets cannot be deleted");
      else { confirmActive = true; confirmTarget = scenesCursor; }
    }
  }
}

static void handleBinaural(const KeyEv& ev) {
  if (ev.tab || ev.c == '`') {
    binPlaying = false;
    binApply();
    binScreen = false;
    gAmbientScale = 1.f;
    return;
  }
  if (ev.enter) { binPlaying = !binPlaying; binApply(); return; }
  if (ev.c == ';') { binCursor = (binCursor + BIN_ROWS - 1) % BIN_ROWS; return; }
  if (ev.c == '.') { binCursor = (binCursor + 1) % BIN_ROWS; return; }
  int d = ev.c == '/' ? 1 : (ev.c == ',' ? -1 : 0);
  if (!d) return;
  // Frequency steps accelerate on hold: 0.1 Hz for fine work, faster when
  // travelling across the range.
  float step = ev.heldMs < 900 ? 0.1f : (ev.heldMs < 2200 ? 1.0f : 5.0f);
  switch (binCursor) {
    case 0: binLoadPreset((binPreset + d + NUM_BIN_PRESETS) % NUM_BIN_PRESETS); return;
    case 1: binMode = (binMode + d + NUM_BEAT_MODES) % NUM_BEAT_MODES; break;
    case 2: binBase += d * step; break;
    case 3: binBeat += d * (step * 0.5f); break;   // beat offsets are small; finer steps than the base
    case 4: binVolume = constrain(binVolume + d * 5, 0, 100); break;
    case 5: binAmbient = constrain(binAmbient + d * 5, 0, 100); gAmbientScale = binAmbient * 0.01f; binSaveToSettings(); return;
  }
  if (binBase < 20.f) binBase = 20.f; if (binBase > 1500.f) binBase = 1500.f;
  if (binBeat < 0.f)  binBeat = 0.f;  if (binBeat > 60.f)   binBeat = 60.f;
  binApply();
  binSaveToSettings();
}

static void handleKey(const KeyEv& ev) {
  if (binScreen)     { handleBinaural(ev); return; }
  if (textActive)    { handleTextInput(ev); return; }
  if (confirmActive) { handleConfirm(ev); return; }
  if (ev.c == 'h' || ev.c == 'H') { helpVisible = !helpVisible; return; }
  if (helpVisible) return;
  if (ev.space) {
    playing = !playing;
    armSleep();
    markSettingsDirty();
    return;
  }
  if (ev.tab) {
    if (curSub != SUB_NONE) curSub = SUB_NONE;
    else curScreen = (Screen)((curScreen + 1) % NUM_SCREENS);
    return;
  }
  switch (curScreen) {
    case SCR_MAIN:     handleMain(ev); break;
    case SCR_EDIT:     handleEdit(ev); break;
    case SCR_SETTINGS: handleSettings(ev); break;
    default: break;
  }
}

// ---------------------------------------------------------------------------
// 7. setup() / loop()
// ---------------------------------------------------------------------------
void setup() {
  auto cfg = M5.config();
  cfg.internal_spk = true;
  cfg.internal_imu = true;
  M5Cardputer.begin(cfg, true);
  // The Shake feature needs the IMU; remember whether one was actually found
  // so the Settings screen can say so instead of silently doing nothing.
  imuReady = (M5.Imu.getType() != m5::imu_none);
  M5Cardputer.Display.setRotation(1);
  M5Cardputer.Display.setBrightness(200);

  // Speaker: pinned feed task, generous DMA buffering
  M5Cardputer.Speaker.end();
  auto sc = M5Cardputer.Speaker.config();
  sc.sample_rate = SAMPLE_RATE;
  sc.stereo = false;   // v0.81: reverted -- this hardware does not deliver true per-channel stereo (see the notes above renderBuffer)
  sc.dma_buf_count = 8;
  sc.dma_buf_len = 512;
  sc.task_pinned_core = APP_CPU_NUM;
  M5Cardputer.Speaker.config(sc);
  M5Cardputer.Speaker.begin();
  M5Cardputer.Speaker.setVolume(180);

  canvas.setColorDepth(16);
  canvas.createSprite(SCREEN_W, SCREEN_H);

  spaceClear();

  ledStrip.begin();
  ledStrip.setBrightness(255);   // actual dimming is done in ledShow()
  ledStrip.clear();
  ledStrip.show();

  for (int i = 0; i < SINE_N; i++) sineTab[i] = sinf(6.2831853f * i / SINE_N);
  for (int k = 0; k <= RING_SEG; k++) {
    float a = k * (6.2831853f / RING_SEG);
    ringCos[k] = cosf(a); ringSin[k] = sinf(a);
  }
  memset(dsp, 0, sizeof(dsp));

  bootSplash();

  initStorage();
  loadSettings();
  applyBrightness(settings.brightness);
  nameShowUntil = millis() + 2500;

  bootFadeMul = 0.f; bootFading = true;
  updateGainTargets();
  armSleep();
  lastInputMs = millis();
  lastFrameMs = millis();

  xTaskCreatePinnedToCore(audioTask, "audio", 8192, NULL, 3, NULL, PRO_CPU_NUM);
}

void loop() {
  M5Cardputer.update();
  uint32_t now = millis();
  float dt = (now - lastFrameMs) * 0.001f;
  if (dt > 0.1f) dt = 0.1f;
  lastFrameMs = now;

  KeyEv ev; ev.c = 0; ev.enter = ev.tab = ev.space = ev.del = ev.shift = ev.fn = false; ev.heldMs = 0;
  bool got = readKey(ev);
  bool g0 = M5Cardputer.BtnA.wasPressed();

  if (!displayOn) {
    // Once the guard is armed (see displayOff()), wait for a moment with
    // nothing held at all before a keyboard press is allowed to wake the
    // display -- that's what tells apart a genuine new keypress from a
    // trailing release-edge of the combo that just turned the display off.
    // G0 is a separate physical button, not part of any keyboard combo, so
    // it isn't gated by this.
    if (displayWakeGuard && !M5Cardputer.Keyboard.isPressed()) displayWakeGuard = false;
    if (g0 || (got && !displayWakeGuard)) displayWake();
  } else {
    if (g0) displayOff();
    else if (got) { lastInputMs = now; handleKey(ev); }
  }

  updateBootFade(dt);
  updateCrossfade(dt);
  updateDrift(dt);
  updateBreath(dt);
  updateSession();
  updateShake(dt);
  // Reverb parameters are plain floats read by the audio task once per buffer
  gSpaceMix      = settings.spaceMix * 0.01f;
  gSpaceFeedback = 0.62f + 0.34f * (settings.spaceSize * 0.01f);
  gSpaceDamp     = 0.12f + 0.70f * (settings.spaceDamp * 0.01f);
  updateSleep();
  updateGainTargets();
  updateLed(dt);
  flushDirty();

  // Dim / auto-off
  if (displayOn) {
    uint32_t idle = (now - lastInputMs) / 1000;
    int level = settings.brightness;
    if (settings.dimAfterSec > 0 && idle >= settings.dimAfterSec) level = settings.brightness * settings.dimLevel / 100;
    if (level < 3) level = 3;
    if (level != lastBrightLevel) { applyBrightness(level); lastBrightLevel = level; }
    if (settings.offAfterSec > 0 && idle >= settings.offAfterSec) displayOff();
  }

  static uint32_t lastRender = 0;
  if (displayOn && now - lastRender >= 33) { lastRender = now; render(); }
  delay(4);
}
