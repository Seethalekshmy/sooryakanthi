#include <Arduino.h>
#include <WiFi.h>
#include <base64.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// One firmware file. No extra libraries (no ESP32Servo, no WebSockets).
// Servos: ESP32 LEDC. Phone UI: web/index.html on the host, ws://IP:81.
//
//   controlTask  core 1  prio 5   50 ms tick: LDR → engines → servos
//   commsTask    core 0  prio 3   WebSocket + serial
//
// After the first bench test, edit INVERT_* and DARK/DZ below.

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

enum ErrAxis
{
  AXIS_YAW,
  AXIS_PITCH
};

static const char *WIFI_AP_SSID = "Sooryaganthi";
static const char *WIFI_AP_PASS = "soorya123";
static const char *WIFI_STA_SSID = "";
static const char *WIFI_STA_PASS = "";

static const int PIN_SERVO_YAW = 25;
static const int PIN_SERVO_PITCH = 26;
static const int PIN_LDR_L = 34;
static const int PIN_LDR_R = 35;
static const int SERVO_MIN_US = 500;
static const int SERVO_MAX_US = 2400;
static const int LEDC_CH_YAW = 0;
static const int LEDC_CH_PITCH = 1;
static const int LEDC_BITS = 16;
static const uint32_t LEDC_MAX = (1u << LEDC_BITS) - 1;

static const uint32_t TICK_MS = 50;
static const uint32_t TELEMETRY_EVERY = 2;
static const uint16_t WS_PORT = 81;

static const int ADC_SAMPLES = 8;
static const int ADC_BITS = 12;
static const int ADC_MAX = 4095;
// Divider: 3V3 — LDR — tap — 1 kΩ — GND. Tap is GPIO 34/35.
// 1 kΩ to GND pulls the ADC down vs a 10 kΩ divider (covered ~0–80).
static const bool INVERT_LIGHT = false;
static const bool INVERT_YAW = false;
static const bool INVERT_PITCH = false;
static const ErrAxis ERR_AXIS = AXIS_YAW;

static const float YAW_MIN = 0.0f;
static const float YAW_MAX = 180.0f;
static const float PITCH_MIN = 20.0f;
static const float PITCH_MAX = 160.0f;
static const float BOOT_YAW = 90.0f;
static const float BOOT_PITCH = 90.0f;

// 12-bit counts after invert. Covered ≈ 0–80; dim room ≈ 200+; torch ≈ 2000+.
static const uint16_t DARK = 80;
static const uint16_t DARK_TICKS = 20;
static const uint16_t PARK_HYST = 40;
static const uint16_t WAKE_TICKS = 20;
static const float PARK_PITCH = PITCH_MIN;

static const int16_t DZ = 50;
static const float STEP = 1.0f;
static const float SCAN_STEP = 5.0f;
static const uint16_t DITHER_TICKS = 20;
static const float DITHER_DEG = 2.0f;
static const float RELOCK_RATIO = 0.6f;
static const uint16_t RELOCK_TICKS = 20;
static const float MANUAL_RATE = 3.0f;
static const float SERVO_RATE_AUTO = SCAN_STEP;
static const float SETTLED_DEG = 1.5f;

static float clampf(float v, float lo, float hi)
{
  if (v < lo)
  {
    return lo;
  }
  if (v > hi)
  {
    return hi;
  }
  return v;
}

static float clampYaw(float v)
{
  return clampf(v, YAW_MIN, YAW_MAX);
}

static float clampPitch(float v)
{
  return clampf(v, PITCH_MIN, PITCH_MAX);
}

// ---------------------------------------------------------------------------
// Runtime
// ---------------------------------------------------------------------------

enum Mode : uint8_t
{
  MODE_INIT,
  MODE_AUTO,
  MODE_MANUAL,
  MODE_PARK
};
enum AutoSub : uint8_t
{
  SUB_SEARCH_YAW,
  SUB_SEARCH_PITCH,
  SUB_LOCK,
  SUB_NONE
};

struct Runtime
{
  Mode mode;
  AutoSub autoSub;
  float cmdYaw;
  float cmdPitch;
  float yaw;
  float pitch;
  float bestYaw;
  float bestPitch;
  float bestAvg;
  uint16_t iL;
  uint16_t iR;
  uint16_t iAvg;
  int16_t iErr;
  bool dark;
  uint32_t tick;
  uint32_t tickMs;
  bool forceTelemetry;
};

static Runtime rt;
static SemaphoreHandle_t gRtLock = nullptr;

static const BaseType_t CONTROL_CORE = 1;
static const BaseType_t COMMS_CORE = 0;
static const UBaseType_t CONTROL_PRIO = 5;
static const UBaseType_t COMMS_PRIO = 3;
static const uint32_t CONTROL_STACK = 4096;
static const uint32_t COMMS_STACK = 4096;

static void rtLock()
{
  xSemaphoreTake(gRtLock, portMAX_DELAY);
}

static void rtUnlock()
{
  xSemaphoreGive(gRtLock);
}

static const char *modeName(Mode m)
{
  switch (m)
  {
  case MODE_AUTO:
    return "AUTO";
  case MODE_MANUAL:
    return "MANUAL";
  case MODE_PARK:
    return "PARK";
  default:
    return "INIT";
  }
}

static const char *subName(AutoSub s)
{
  switch (s)
  {
  case SUB_SEARCH_YAW:
    return "search_yaw";
  case SUB_SEARCH_PITCH:
    return "search_pitch";
  case SUB_LOCK:
    return "lock";
  default:
    return "none";
  }
}

static const char *modeWire(Mode m)
{
  switch (m)
  {
  case MODE_AUTO:
    return "auto";
  case MODE_MANUAL:
    return "manual";
  case MODE_PARK:
    return "park";
  default:
    return "init";
  }
}

static const char *subWire(AutoSub s)
{
  switch (s)
  {
  case SUB_SEARCH_YAW:
    return "search_yaw";
  case SUB_SEARCH_PITCH:
    return "search_pitch";
  case SUB_LOCK:
    return "lock";
  default:
    return "none";
  }
}

static void setMode(Mode next);
static void requestRescan();

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------

static void sensorsBegin()
{
  analogReadResolution(ADC_BITS);
  analogSetPinAttenuation(PIN_LDR_L, ADC_11db);
  analogSetPinAttenuation(PIN_LDR_R, ADC_11db);
  pinMode(PIN_LDR_L, INPUT);
  pinMode(PIN_LDR_R, INPUT);
}

static uint16_t readLdr(int pin)
{
  uint32_t acc = 0;
  for (int i = 0; i < ADC_SAMPLES; i++)
  {
    acc += (uint32_t)analogRead(pin);
  }
  uint16_t raw = (uint16_t)(acc / (uint32_t)ADC_SAMPLES);
  if (INVERT_LIGHT)
  {
    return (uint16_t)(ADC_MAX - raw);
  }
  return raw;
}

static void sensorsRead()
{
  rt.iL = readLdr(PIN_LDR_L);
  rt.iR = readLdr(PIN_LDR_R);
  rt.iAvg = (uint16_t)(((uint32_t)rt.iL + (uint32_t)rt.iR) / 2);
  rt.iErr = (int16_t)((int32_t)rt.iL - (int32_t)rt.iR);
  rt.dark = rt.iAvg < DARK;
}

// ---------------------------------------------------------------------------
// Servos
// ---------------------------------------------------------------------------

static float rateToward(float current, float target, float maxDelta)
{
  float d = target - current;
  if (d > maxDelta)
  {
    return current + maxDelta;
  }
  if (d < -maxDelta)
  {
    return current - maxDelta;
  }
  return target;
}

static void servoWriteAngle(int channel, float angle, float lo, float hi, bool invert)
{
  float clamped = clampf(angle, lo, hi);
  if (invert)
  {
    clamped = lo + (hi - clamped);
  }
  clamped = clampf(clamped, 0.0f, 180.0f);
  const int us =
      (int)lroundf(SERVO_MIN_US + (clamped / 180.0f) * (float)(SERVO_MAX_US - SERVO_MIN_US));
  const uint32_t duty = (uint32_t)((uint64_t)us * LEDC_MAX / 20000ULL);
  ledcWrite((uint8_t)channel, duty);
}

static void servosBegin()
{
  ledcSetup(LEDC_CH_YAW, 50, LEDC_BITS);
  ledcSetup(LEDC_CH_PITCH, 50, LEDC_BITS);
  ledcAttachPin(PIN_SERVO_YAW, LEDC_CH_YAW);
  ledcAttachPin(PIN_SERVO_PITCH, LEDC_CH_PITCH);
  servoWriteAngle(LEDC_CH_YAW, BOOT_YAW, YAW_MIN, YAW_MAX, INVERT_YAW);
  servoWriteAngle(LEDC_CH_PITCH, BOOT_PITCH, PITCH_MIN, PITCH_MAX, INVERT_PITCH);
}

static void servosWrite()
{
  rt.cmdYaw = clampYaw(rt.cmdYaw);
  rt.cmdPitch = clampPitch(rt.cmdPitch);

  const float maxDelta = (rt.mode == MODE_MANUAL) ? MANUAL_RATE : SERVO_RATE_AUTO;
  rt.yaw = rateToward(rt.yaw, rt.cmdYaw, maxDelta);
  rt.pitch = rateToward(rt.pitch, rt.cmdPitch, maxDelta);
  rt.yaw = clampYaw(rt.yaw);
  rt.pitch = clampPitch(rt.pitch);

  servoWriteAngle(LEDC_CH_YAW, rt.yaw, YAW_MIN, YAW_MAX, INVERT_YAW);
  servoWriteAngle(LEDC_CH_PITCH, rt.pitch, PITCH_MIN, PITCH_MAX, INVERT_PITCH);
}

// ---------------------------------------------------------------------------
// MANUAL engine
// ---------------------------------------------------------------------------

enum ManualKind : uint8_t
{
  MK_NONE,
  MK_SET,
  MK_NUDGE
};

static ManualKind manualPending = MK_NONE;
static float manualQYaw = 0;
static float manualQPitch = 0;

static void manualClear()
{
  manualPending = MK_NONE;
}

static void manualOnEnter()
{
  manualClear();
  rt.autoSub = SUB_NONE;
  rt.forceTelemetry = true;
}

static void manualQueueSet(float yaw, float pitch)
{
  manualPending = MK_SET;
  manualQYaw = yaw;
  manualQPitch = pitch;
}

static void manualQueueNudge(float dyaw, float dpitch)
{
  manualPending = MK_NUDGE;
  manualQYaw = dyaw;
  manualQPitch = dpitch;
}

static void manualStep()
{
  if (manualPending == MK_NONE)
  {
    return;
  }
  if (manualPending == MK_SET)
  {
    rt.cmdYaw = clampYaw(manualQYaw);
    rt.cmdPitch = clampPitch(manualQPitch);
  }
  else if (manualPending == MK_NUDGE)
  {
    rt.cmdYaw = clampYaw(rt.cmdYaw + manualQYaw);
    rt.cmdPitch = clampPitch(rt.cmdPitch + manualQPitch);
  }
  manualPending = MK_NONE;
}

// ---------------------------------------------------------------------------
// AUTO engine
// ---------------------------------------------------------------------------

static uint16_t darkTicks = 0;
static uint16_t relockTicks = 0;
static uint16_t wakeTicks = 0;
static uint16_t ditherWait = 0;

enum DitherPhase : uint8_t
{
  DITHER_IDLE,
  DITHER_PLUS,
  DITHER_MINUS
};
static DitherPhase ditherPhase = DITHER_IDLE;
static float ditherCenter = 0;
static float ditherBest = 0;
static float ditherBestAvg = 0;

static bool settled(float applied, float cmd)
{
  return fabsf(applied - cmd) <= SETTLED_DEG;
}

static void resetLockCounters()
{
  darkTicks = 0;
  relockTicks = 0;
  ditherWait = 0;
  ditherPhase = DITHER_IDLE;
}

static void rememberBest()
{
  if ((float)rt.iAvg > rt.bestAvg)
  {
    rt.bestAvg = (float)rt.iAvg;
    rt.bestYaw = rt.yaw;
    rt.bestPitch = rt.pitch;
  }
}

static void logSub()
{
  Serial.printf("%s/%s\n", modeName(rt.mode), subName(rt.autoSub));
}

static void autoEnterSearchYaw()
{
  rt.mode = MODE_AUTO;
  rt.autoSub = SUB_SEARCH_YAW;
  rt.bestAvg = -1.0f;
  rt.bestYaw = YAW_MIN;
  rt.bestPitch = clampPitch(rt.cmdPitch);
  rt.cmdYaw = YAW_MIN;
  resetLockCounters();
  logSub();
}

static void enterSearchPitch()
{
  rt.autoSub = SUB_SEARCH_PITCH;
  rt.cmdYaw = clampYaw(rt.bestYaw);
  rt.cmdPitch = PITCH_MIN;
  logSub();
}

static void enterLock()
{
  rt.autoSub = SUB_LOCK;
  rt.cmdYaw = clampYaw(rt.bestYaw);
  rt.cmdPitch = clampPitch(rt.bestPitch);
  resetLockCounters();
  logSub();
}

static void parkEnter()
{
  rt.autoSub = SUB_NONE;
  rt.cmdPitch = PARK_PITCH;
  wakeTicks = 0;
  Serial.printf("%s/%s\n", modeName(MODE_PARK), subName(SUB_NONE));
}

static void stepErrorAxis()
{
  if (rt.iErr > DZ)
  {
    if (ERR_AXIS == AXIS_YAW)
    {
      rt.cmdYaw = clampYaw(rt.cmdYaw - STEP);
    }
    else
    {
      rt.cmdPitch = clampPitch(rt.cmdPitch - STEP);
    }
  }
  else if (rt.iErr < -DZ)
  {
    if (ERR_AXIS == AXIS_YAW)
    {
      rt.cmdYaw = clampYaw(rt.cmdYaw + STEP);
    }
    else
    {
      rt.cmdPitch = clampPitch(rt.cmdPitch + STEP);
    }
  }
}

static float *ditherCmd()
{
  return (ERR_AXIS == AXIS_YAW) ? &rt.cmdPitch : &rt.cmdYaw;
}

static float ditherClamp(float v)
{
  return (ERR_AXIS == AXIS_YAW) ? clampPitch(v) : clampYaw(v);
}

static void stepDither()
{
  float *cmd = ditherCmd();
  if (ditherPhase == DITHER_IDLE)
  {
    ditherWait++;
    if (ditherWait < DITHER_TICKS)
    {
      return;
    }
    ditherWait = 0;
    ditherCenter = *cmd;
    ditherBest = ditherCenter;
    ditherBestAvg = (float)rt.iAvg;
    *cmd = ditherClamp(ditherCenter + DITHER_DEG);
    ditherPhase = DITHER_PLUS;
    return;
  }
  if (ditherPhase == DITHER_PLUS)
  {
    if ((float)rt.iAvg > ditherBestAvg)
    {
      ditherBestAvg = (float)rt.iAvg;
      ditherBest = *cmd;
    }
    *cmd = ditherClamp(ditherCenter - DITHER_DEG);
    ditherPhase = DITHER_MINUS;
    return;
  }
  if ((float)rt.iAvg > ditherBestAvg)
  {
    ditherBest = *cmd;
  }
  *cmd = ditherClamp(ditherBest);
  ditherPhase = DITHER_IDLE;
}

static void stepSearchYaw()
{
  if (!settled(rt.yaw, rt.cmdYaw))
  {
    return;
  }
  rememberBest();
  if (rt.cmdYaw >= YAW_MAX)
  {
    rt.cmdYaw = clampYaw(rt.bestYaw);
    enterSearchPitch();
    return;
  }
  rt.cmdYaw = clampYaw(rt.cmdYaw + SCAN_STEP);
}

static void stepSearchPitch()
{
  if (!settled(rt.yaw, rt.cmdYaw) || !settled(rt.pitch, rt.cmdPitch))
  {
    return;
  }
  rememberBest();
  if (rt.cmdPitch >= PITCH_MAX)
  {
    rt.cmdYaw = clampYaw(rt.bestYaw);
    rt.cmdPitch = clampPitch(rt.bestPitch);
    enterLock();
    return;
  }
  rt.cmdPitch = clampPitch(rt.cmdPitch + SCAN_STEP);
}

static void stepLock()
{
  if (rt.iAvg < DARK)
  {
    darkTicks++;
    if (darkTicks >= DARK_TICKS)
    {
      setMode(MODE_PARK);
      return;
    }
  }
  else
  {
    darkTicks = 0;
  }

  if (rt.bestAvg > 0.0f && (float)rt.iAvg < rt.bestAvg * RELOCK_RATIO)
  {
    relockTicks++;
    if (relockTicks >= RELOCK_TICKS)
    {
      autoEnterSearchYaw();
      return;
    }
  }
  else
  {
    relockTicks = 0;
  }

  stepErrorAxis();
  stepDither();
}

static void autoTrackerStep()
{
  switch (rt.autoSub)
  {
  case SUB_SEARCH_YAW:
    stepSearchYaw();
    break;
  case SUB_SEARCH_PITCH:
    stepSearchPitch();
    break;
  case SUB_LOCK:
    stepLock();
    break;
  default:
    autoEnterSearchYaw();
    break;
  }
}

static void parkStep()
{
  rt.cmdPitch = PARK_PITCH;
  if (rt.iAvg > (uint16_t)(DARK + PARK_HYST))
  {
    wakeTicks++;
    if (wakeTicks >= WAKE_TICKS)
    {
      setMode(MODE_AUTO);
    }
  }
  else
  {
    wakeTicks = 0;
  }
}

// ---------------------------------------------------------------------------
// Mode arbiter (callers already hold gRtLock)
// ---------------------------------------------------------------------------

static void setMode(Mode next)
{
  if (next == rt.mode)
  {
    return;
  }
  rt.mode = next;
  if (next == MODE_AUTO)
  {
    autoEnterSearchYaw();
    return;
  }
  if (next == MODE_MANUAL)
  {
    manualOnEnter();
    Serial.printf("%s/%s\n", modeName(rt.mode), subName(rt.autoSub));
    return;
  }
  if (next == MODE_PARK)
  {
    parkEnter();
  }
}

static void requestRescan()
{
  rt.mode = MODE_AUTO;
  autoEnterSearchYaw();
}

// ---------------------------------------------------------------------------
// Comms — minimal WebSocket (single client). Stays on commsTask (core 0).
// ---------------------------------------------------------------------------

static WiFiServer wsServer(WS_PORT);
static WiFiClient wsClient;
static bool wsLive = false;
static uint32_t lastTelemetryMs = 0;
static uint8_t wsRx[256];
static size_t wsRxLen = 0;

static uint32_t rol32(uint32_t v, int b)
{
  return (v << b) | (v >> (32 - b));
}

static void sha1(const uint8_t *msg, size_t len, uint8_t digest[20])
{
  uint32_t h0 = 0x67452301UL, h1 = 0xEFCDAB89UL, h2 = 0x98BADCFEUL, h3 = 0x10325476UL,
           h4 = 0xC3D2E1F0UL;
  uint8_t block[64];
  const uint64_t bitlen = (uint64_t)len * 8ULL;
  size_t off = 0;

  auto process = [&](const uint8_t *blk)
  {
    uint32_t w[80];
    for (int t = 0; t < 16; t++)
    {
      w[t] = ((uint32_t)blk[t * 4] << 24) | ((uint32_t)blk[t * 4 + 1] << 16) |
             ((uint32_t)blk[t * 4 + 2] << 8) | (uint32_t)blk[t * 4 + 3];
    }
    for (int t = 16; t < 80; t++)
    {
      w[t] = rol32(w[t - 3] ^ w[t - 8] ^ w[t - 14] ^ w[t - 16], 1);
    }
    uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;
    for (int t = 0; t < 80; t++)
    {
      uint32_t f, k;
      if (t < 20)
      {
        f = (b & c) | ((~b) & d);
        k = 0x5A827999UL;
      }
      else if (t < 40)
      {
        f = b ^ c ^ d;
        k = 0x6ED9EBA1UL;
      }
      else if (t < 60)
      {
        f = (b & c) | (b & d) | (c & d);
        k = 0x8F1BBCDCUL;
      }
      else
      {
        f = b ^ c ^ d;
        k = 0xCA62C1D6UL;
      }
      const uint32_t temp = rol32(a, 5) + f + e + k + w[t];
      e = d;
      d = c;
      c = rol32(b, 30);
      b = a;
      a = temp;
    }
    h0 += a;
    h1 += b;
    h2 += c;
    h3 += d;
    h4 += e;
  };

  while (len - off >= 64)
  {
    process(msg + off);
    off += 64;
  }
  const size_t rem = len - off;
  memset(block, 0, sizeof(block));
  memcpy(block, msg + off, rem);
  block[rem] = 0x80;
  if (rem >= 56)
  {
    process(block);
    memset(block, 0, sizeof(block));
  }
  for (int t = 0; t < 8; t++)
  {
    block[63 - t] = (uint8_t)(bitlen >> (8 * t));
  }
  process(block);
  const uint32_t hs[5] = {h0, h1, h2, h3, h4};
  for (int t = 0; t < 5; t++)
  {
    digest[t * 4] = (uint8_t)(hs[t] >> 24);
    digest[t * 4 + 1] = (uint8_t)(hs[t] >> 16);
    digest[t * 4 + 2] = (uint8_t)(hs[t] >> 8);
    digest[t * 4 + 3] = (uint8_t)hs[t];
  }
}

static void wsClose()
{
  if (wsLive)
  {
    wsClient.stop();
  }
  wsLive = false;
  wsRxLen = 0;
}

static bool wsSendFrame(uint8_t opcode, const uint8_t *data, size_t len)
{
  if (!wsLive || !wsClient.connected())
  {
    return false;
  }
  uint8_t hdr[4];
  size_t hdrn = 2;
  hdr[0] = (uint8_t)(0x80 | (opcode & 0x0F));
  if (len < 126)
  {
    hdr[1] = (uint8_t)len;
  }
  else if (len <= 0xFFFF)
  {
    hdr[1] = 126;
    hdr[2] = (uint8_t)(len >> 8);
    hdr[3] = (uint8_t)len;
    hdrn = 4;
  }
  else
  {
    return false;
  }
  if (wsClient.write(hdr, hdrn) != hdrn)
  {
    wsClose();
    return false;
  }
  if (len > 0 && wsClient.write(data, len) != len)
  {
    wsClose();
    return false;
  }
  return true;
}

static bool wsSendText(const char *text)
{
  return wsSendFrame(0x01, (const uint8_t *)text, strlen(text));
}

static bool jsonString(const char *json, const char *key, char *out, size_t outN)
{
  char pat[24];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char *p = strstr(json, pat);
  if (!p)
  {
    return false;
  }
  p = strchr(p + strlen(pat), ':');
  if (!p)
  {
    return false;
  }
  p++;
  while (*p == ' ' || *p == '\t')
  {
    p++;
  }
  if (*p != '\"')
  {
    return false;
  }
  p++;
  size_t i = 0;
  while (*p && *p != '\"' && i + 1 < outN)
  {
    out[i++] = *p++;
  }
  out[i] = '\0';
  return *p == '\"';
}

static bool jsonNumber(const char *json, const char *key, float *out)
{
  char pat[24];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char *p = strstr(json, pat);
  if (!p)
  {
    return false;
  }
  p = strchr(p + strlen(pat), ':');
  if (!p)
  {
    return false;
  }
  p++;
  while (*p == ' ' || *p == '\t')
  {
    p++;
  }
  if (!(*p == '-' || *p == '.' || (*p >= '0' && *p <= '9')))
  {
    return false;
  }
  char *end = nullptr;
  float v = strtof(p, &end);
  if (end == p || !isfinite(v))
  {
    return false;
  }
  *out = v;
  return true;
}

static void handleInbound(const char *msg)
{
  char type[16] = {0};
  if (!jsonString(msg, "type", type, sizeof(type)))
  {
    return;
  }

  rtLock();
  if (strcmp(type, "setMode") == 0)
  {
    char mode[12] = {0};
    if (jsonString(msg, "mode", mode, sizeof(mode)))
    {
      if (strcmp(mode, "auto") == 0)
      {
        setMode(MODE_AUTO);
      }
      else if (strcmp(mode, "manual") == 0)
      {
        setMode(MODE_MANUAL);
      }
    }
  }
  else if (strcmp(type, "set") == 0)
  {
    float yaw = 0;
    float pitch = 0;
    if (jsonNumber(msg, "yaw", &yaw) && jsonNumber(msg, "pitch", &pitch))
    {
      manualQueueSet(yaw, pitch);
    }
  }
  else if (strcmp(type, "nudge") == 0)
  {
    float dyaw = 0;
    float dpitch = 0;
    if (jsonNumber(msg, "dyaw", &dyaw) && jsonNumber(msg, "dpitch", &dpitch))
    {
      manualQueueNudge(dyaw, dpitch);
    }
  }
  else if (strcmp(type, "rescan") == 0)
  {
    requestRescan();
  }
  rtUnlock();
}

static void sendState(const Runtime &snap)
{
  char json[256];
  snprintf(json, sizeof(json),
           "{\"type\":\"state\",\"mode\":\"%s\",\"sub\":\"%s\","
           "\"yaw\":%.1f,\"pitch\":%.1f,\"iL\":%u,\"iR\":%u,\"iAvg\":%u,"
           "\"iErr\":%d,\"dark\":%s}",
           modeWire(snap.mode),
           snap.mode == MODE_AUTO ? subWire(snap.autoSub) : "none",
           snap.yaw, snap.pitch, snap.iL, snap.iR, snap.iAvg, (int)snap.iErr,
           snap.dark ? "true" : "false");
  wsSendText(json);
}

static Runtime snapshotRt()
{
  rtLock();
  Runtime snap = rt;
  rt.forceTelemetry = false;
  rtUnlock();
  return snap;
}

static void maybeBroadcast()
{
  if (!wsLive)
  {
    return;
  }
  rtLock();
  const bool force = rt.forceTelemetry;
  const uint32_t now = millis();
  const bool due = (now - lastTelemetryMs) >= (TICK_MS * TELEMETRY_EVERY);
  if (!force && !due)
  {
    rtUnlock();
    return;
  }
  Runtime snap = rt;
  rt.forceTelemetry = false;
  rtUnlock();
  lastTelemetryMs = now;
  sendState(snap);
}

static bool wsReadHeaders(WiFiClient &c, char *buf, size_t buflen)
{
  size_t n = 0;
  const uint32_t start = millis();
  while (n + 1 < buflen && (millis() - start) < 800)
  {
    if (!c.connected())
    {
      return false;
    }
    if (!c.available())
    {
      delay(1);
      continue;
    }
    buf[n++] = (char)c.read();
    if (n >= 4 && buf[n - 4] == '\r' && buf[n - 3] == '\n' && buf[n - 2] == '\r' &&
        buf[n - 1] == '\n')
    {
      buf[n] = '\0';
      return true;
    }
  }
  return false;
}

static bool wsHandshake(WiFiClient &c)
{
  char hdrs[768];
  if (!wsReadHeaders(c, hdrs, sizeof(hdrs)))
  {
    return false;
  }
  const char *p = strstr(hdrs, "Sec-WebSocket-Key:");
  if (!p)
  {
    p = strstr(hdrs, "sec-websocket-key:");
  }
  if (!p)
  {
    return false;
  }
  p += 18;
  while (*p == ' ' || *p == '\t')
  {
    p++;
  }
  char key[32] = {0};
  size_t k = 0;
  while (*p && *p != '\r' && *p != '\n' && k + 1 < sizeof(key))
  {
    key[k++] = *p++;
  }

  char concat[64];
  snprintf(concat, sizeof(concat), "%s258EAFA5-E914-47DA-95CA-C5AB0DC85B11", key);
  uint8_t hash[20];
  sha1((const uint8_t *)concat, strlen(concat), hash);
  const String accept = base64::encode(hash, 20);

  char resp[192];
  snprintf(resp, sizeof(resp),
           "HTTP/1.1 101 Switching Protocols\r\n"
           "Upgrade: websocket\r\n"
           "Connection: Upgrade\r\n"
           "Sec-WebSocket-Accept: %s\r\n"
           "\r\n",
           accept.c_str());
  return c.print(resp) > 0;
}

static void wsAcceptNew()
{
  if (!wsServer.hasClient())
  {
    return;
  }
  WiFiClient incoming = wsServer.available();
  incoming.setNoDelay(true);
  if (wsLive)
  {
    Serial.println("ws replace");
    wsClose();
  }
  if (!wsHandshake(incoming))
  {
    incoming.stop();
    return;
  }
  wsClient = incoming;
  wsLive = true;
  wsRxLen = 0;
  Serial.printf("ws %s connected\n", wsClient.remoteIP().toString().c_str());
  sendState(snapshotRt());
}

static void wsConsumeFrames()
{
  while (wsLive && wsClient.available() && wsRxLen < sizeof(wsRx))
  {
    wsRx[wsRxLen++] = (uint8_t)wsClient.read();
  }
  if (!wsLive)
  {
    return;
  }
  if (!wsClient.connected())
  {
    Serial.println("ws disconnected");
    wsClose();
    return;
  }

  size_t off = 0;
  while (wsRxLen - off >= 2)
  {
    const uint8_t *f = wsRx + off;
    const uint8_t opcode = f[0] & 0x0F;
    const bool masked = (f[1] & 0x80) != 0;
    uint64_t pay = (uint64_t)(f[1] & 0x7F);
    size_t hdr = 2;
    if (pay == 126)
    {
      if (wsRxLen - off < 4)
      {
        break;
      }
      pay = ((uint16_t)f[2] << 8) | f[3];
      hdr = 4;
    }
    else if (pay == 127)
    {
      wsClose();
      return;
    }
    if (!masked)
    {
      wsClose();
      return;
    }
    hdr += 4;
    if (wsRxLen - off < hdr + (size_t)pay)
    {
      break;
    }
    const uint8_t *mask = f + hdr - 4;
    const uint8_t *payload = f + hdr;
    char msg[193];
    const size_t n = pay < sizeof(msg) - 1 ? (size_t)pay : sizeof(msg) - 1;
    for (size_t i = 0; i < n; i++)
    {
      msg[i] = (char)(payload[i] ^ mask[i & 3]);
    }
    msg[n] = '\0';

    if (opcode == 0x8)
    {
      Serial.println("ws disconnected");
      wsClose();
      return;
    }
    if (opcode == 0x9)
    {
      wsSendFrame(0x0A, (const uint8_t *)msg, n);
    }
    else if (opcode == 0x1 || opcode == 0x0)
    {
      handleInbound(msg);
    }

    off += hdr + (size_t)pay;
  }
  if (off > 0)
  {
    memmove(wsRx, wsRx + off, wsRxLen - off);
    wsRxLen -= off;
  }
}

static void wsPoll()
{
  wsAcceptNew();
  if (wsLive)
  {
    wsConsumeFrames();
  }
}

static void commsPrintWifi()
{
  Serial.println("WiFi:");
  Serial.printf("  AP SSID=%s  pass=%s\n", WIFI_AP_SSID, WIFI_AP_PASS);
  Serial.printf("  ws    ws://%s:%u\n", WiFi.softAPIP().toString().c_str(), WS_PORT);
  Serial.println("  UI    open web/index.html on the phone/PC (not hosted here)");
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.printf("  STA  ws://%s:%u\n", WiFi.localIP().toString().c_str(), WS_PORT);
  }
}

static void startWifi()
{
  const bool useSta = WIFI_STA_SSID[0] != '\0';
  WiFi.mode(useSta ? WIFI_AP_STA : WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1),
                    IPAddress(255, 255, 255, 0));
  if (!WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS))
  {
    Serial.println("softAP failed");
  }

  if (useSta)
  {
    Serial.printf("joining %s", WIFI_STA_SSID);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < 8000)
    {
      delay(250);
      Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("station join timed out; AP still available");
    }
  }
}

static void commsBegin()
{
  startWifi();
  wsServer.begin();
  wsServer.setNoDelay(true);
  commsPrintWifi();
}

// ---------------------------------------------------------------------------
// Serial (runs on commsTask)
// ---------------------------------------------------------------------------

static bool verboseSensors = false;
static char lineBuf[96];
static size_t lineLen = 0;

static void printStatus(const Runtime &snap)
{
  Serial.printf("%s/%s yaw=%.1f pitch=%.1f cmd=%.1f/%.1f iL=%u iR=%u iAvg=%u iErr=%d dark=%d best=%.0f\n",
                modeName(snap.mode), subName(snap.autoSub), snap.yaw, snap.pitch, snap.cmdYaw,
                snap.cmdPitch, snap.iL, snap.iR, snap.iAvg, (int)snap.iErr, snap.dark ? 1 : 0,
                snap.bestAvg);
}

static void printHelp()
{
  Serial.println("SLR tracker @ 115200");
  Serial.println("  auto | manual | park | rescan | status | verbose | wifi | ?");
  Serial.println("  set <yaw> <pitch>     MANUAL only");
  Serial.println("  nudge <dy> <dp>       MANUAL only");
  Serial.println("Yaw GPIO 25, pitch GPIO 26, LDR L GPIO 34, LDR R GPIO 35");
  Serial.println("FreeRTOS: control core 1 @ 50 ms, comms core 0");
  commsPrintWifi();
}

static void handleLine(char *line)
{
  while (*line == ' ' || *line == '\t')
  {
    line++;
  }
  if (*line == '\0')
  {
    return;
  }
  if (strcmp(line, "?") == 0 || strcasecmp(line, "help") == 0)
  {
    printHelp();
    return;
  }
  if (strcasecmp(line, "wifi") == 0)
  {
    commsPrintWifi();
    return;
  }
  if (strcasecmp(line, "verbose") == 0)
  {
    verboseSensors = !verboseSensors;
    Serial.printf("verbose=%d\n", verboseSensors ? 1 : 0);
    return;
  }
  if (strcasecmp(line, "status") == 0)
  {
    printStatus(snapshotRt());
    return;
  }

  rtLock();
  if (strcasecmp(line, "auto") == 0)
  {
    setMode(MODE_AUTO);
  }
  else if (strcasecmp(line, "manual") == 0)
  {
    setMode(MODE_MANUAL);
  }
  else if (strcasecmp(line, "park") == 0)
  {
    setMode(MODE_PARK);
  }
  else if (strcasecmp(line, "rescan") == 0)
  {
    requestRescan();
  }
  else
  {
    float a = 0;
    float b = 0;
    char tag[12] = {0};
    if (sscanf(line, "%11s %f %f", tag, &a, &b) == 3 && strcasecmp(tag, "set") == 0)
    {
      manualQueueSet(a, b);
    }
    else if (sscanf(line, "%11s %f %f", tag, &a, &b) == 3 && strcasecmp(tag, "nudge") == 0)
    {
      manualQueueNudge(a, b);
    }
    else
    {
      rtUnlock();
      Serial.printf("unknown: %s\n", line);
      return;
    }
  }
  rtUnlock();
}

static void pollSerial()
{
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();
    if (c == '\r')
    {
      continue;
    }
    if (c == '\n')
    {
      lineBuf[lineLen] = '\0';
      handleLine(lineBuf);
      lineLen = 0;
      continue;
    }
    if (lineLen < sizeof(lineBuf) - 1)
    {
      lineBuf[lineLen++] = c;
    }
  }
}

// ---------------------------------------------------------------------------
// Control tick + FreeRTOS tasks
// ---------------------------------------------------------------------------

static void onTick()
{
  sensorsRead();

  switch (rt.mode)
  {
  case MODE_AUTO:
    autoTrackerStep();
    break;
  case MODE_MANUAL:
    manualStep();
    break;
  case MODE_PARK:
    parkStep();
    break;
  default:
    setMode(MODE_AUTO);
    break;
  }

  servosWrite();
  rt.tick++;
  rt.tickMs = millis();
  if ((rt.tick % TELEMETRY_EVERY) == 0)
  {
    rt.forceTelemetry = true;
  }
}

static void controlTask(void * /*arg*/)
{
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(TICK_MS);
  for (;;)
  {
    Runtime verboseSnap;
    bool doVerbose = false;

    rtLock();
    onTick();
    if (verboseSensors)
    {
      verboseSnap = rt;
      doVerbose = true;
    }
    rtUnlock();

    if (doVerbose)
    {
      printStatus(verboseSnap);
    }
    vTaskDelayUntil(&last, period);
  }
}

static void commsTask(void * /*arg*/)
{
  for (;;)
  {
    wsPoll();
    pollSerial();
    maybeBroadcast();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup()
{
  Serial.begin(115200);
  delay(200);

  gRtLock = xSemaphoreCreateMutex();
  if (gRtLock == nullptr)
  {
    Serial.println("rt mutex failed");
    while (true)
    {
      delay(1000);
    }
  }

  memset(&rt, 0, sizeof(rt));
  rt.mode = MODE_INIT;
  rt.autoSub = SUB_NONE;
  rt.cmdYaw = BOOT_YAW;
  rt.cmdPitch = BOOT_PITCH;
  rt.yaw = BOOT_YAW;
  rt.pitch = BOOT_PITCH;
  rt.bestAvg = -1.0f;
  rt.bestYaw = BOOT_YAW;
  rt.bestPitch = BOOT_PITCH;

  sensorsBegin();
  servosBegin();
  commsBegin();

  rtLock();
  setMode(MODE_AUTO);
  rtUnlock();
  printHelp();

  BaseType_t okCtl = xTaskCreatePinnedToCore(controlTask, "control", CONTROL_STACK, nullptr,
                                             CONTROL_PRIO, nullptr, CONTROL_CORE);
  BaseType_t okCom = xTaskCreatePinnedToCore(commsTask, "comms", COMMS_STACK, nullptr, COMMS_PRIO,
                                             nullptr, COMMS_CORE);
  if (okCtl != pdPASS || okCom != pdPASS)
  {
    Serial.println("task create failed");
  }
  else
  {
    Serial.printf("tasks: control core %d prio %u, comms core %d prio %u\n", (int)CONTROL_CORE,
                  (unsigned)CONTROL_PRIO, (int)COMMS_CORE, (unsigned)COMMS_PRIO);
  }
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}
