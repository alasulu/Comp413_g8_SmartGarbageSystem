/*******************************************************
 * Smart Garbage IoT (ESP32-WROVER)
 * Sensors: KY-026 (Flame D0 + A0), KY-032 (Level DO), KY-015 (DHT11), KY-020 (Tilt DO)
 * Outputs: Buzzer + Traffic Light LEDs (traffic lights + buzzer driven ONLY by FIRE)
 *
 * FIXES APPLIED (per your last screenshot):
 *  - KY-026 D0 logic is FIXED and "flipped" permanently: ACTIVE-HIGH
 *      (Your UI showed D0 raw = 0 while it was "detected" => means your board idles LOW)
 *  - A0 backup detection DISABLED (your A0 raw was stuck at 0, which makes strength ~4095 => always fire)
 *  - No fire polarity flipping in Web/Blynk (fixed always)
 *  - WiFi connect TIMEOUT
 *  - HTML in FLASH (PROGMEM) + send_P
 *  - Events JS fixed
 *  - Blynk connect non-blocking
 *******************************************************/

#define BLYNK_TEMPLATE_ID   "TMPL4gCFNeg6O"
#define BLYNK_TEMPLATE_NAME "Absolute Trash"
#include "secrets.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>
#include <BlynkSimpleEsp32.h>

// =====================================================
//                 WIFI
// =====================================================
// =====================================================
//                 PINS (ESP32-WROVER safe)
// =====================================================
// KY-026 Flame
static const int PIN_FLAME_DO = 26;    // D0 -> GPIO26
static const int PIN_FLAME_AO = 35;    // A0 -> GPIO35 (ADC1, input-only)

// KY-032 Level (IR obstacle)
static const int PIN_KY032_DO = 27;    // DO -> GPIO27

// KY-020 Tilt
static const int PIN_KY020_DO = 33;    // DO -> GPIO33

// KY-015 DHT11
static const int PIN_DHT = 25;         // signal pin
static const int DHT_TYPE = DHT11;

// Outputs
static const int PIN_BUZZER = 18;
static const int PIN_LED_R  = 19;
static const int PIN_LED_Y  = 21;
static const int PIN_LED_G  = 22;

// =====================================================
//                 BLYNK VPINS
// =====================================================
#define VPIN_FIRE_DETECTED        V0
#define VPIN_FLAME_STRENGTH_FILT  V1
#define VPIN_FLAME_A0_RAW         V2
#define VPIN_BIN_FILL_PCT         V3
#define VPIN_TEMP_C               V4
#define VPIN_HUM_PCT              V5
#define VPIN_TILT_DETECTED        V6

#define VPIN_FLAME_THRESHOLD      V10
#define VPIN_MUTE_BUZZER          V11
#define VPIN_SILENCE_10MIN        V12

#define VPIN_KY032_ACTIVELOW      V13
#define VPIN_FLAME_D0_ACTIVELOW   V14  // kept for display only (fixed)
#define VPIN_KY026_ACTIVELOW      V15  // kept for display only (fixed)
#define VPIN_KY020_ACTIVELOW      V16

// =====================================================
//                 ALARM THRESHOLDS
// =====================================================
static const float TEMP_ALARM_C = 35.0f;
static const int   BIN_FULL_PCT = 100;

// =====================================================
//                 FLAME (KY-026)  *** FIXED HERE ***
// =====================================================
// Your module idles LOW => "fire" should be ACTIVE-HIGH.
static const bool FLAME_D0_ACTIVE_LOW = false;

// Only use D0 for detection (A0 backup disabled because A0 was stuck at 0)
static const bool USE_FLAME_D0 = true;
static const bool USE_FLAME_A0_BACKUP = false;

// strength = 4095 - A0 (flame => higher) (still computed for UI only)
static int FLAME_STRENGTH_THRESHOLD = 900;
static const float FLAME_EMA_ALPHA = 0.85f;

// =====================================================
//                 KY-032 (responsive + tunable)
// =====================================================
static bool KY032_ACTIVE_LOW = true;
static bool KY032_USE_PULLUP = true;

// Majority filter
static const uint8_t KY_MAJ_SAMPLES = 15;
static uint8_t kyMajRequire = 12;

// Debounce stable majority output (ms)
static uint32_t kyStableMs = 120;

// Latch timing
static const uint32_t KY_ON_CONFIRM_MS  = 500;
static const uint32_t KY_OFF_CONFIRM_MS = 1200;

// If top sensor => 100% when detected, else 50%
static const bool KY032_IS_TOP_SENSOR = true;

// =====================================================
//                 KY-020 TILT (debounced)
// =====================================================
static bool KY020_ACTIVE_LOW = true;
static bool KY020_USE_PULLUP = true;

// Stable debounce (ms) for tilt
static uint32_t tiltStableMs = 180;

// Confirm times (optional “anti-spike”)
static const uint32_t TILT_ON_CONFIRM_MS  = 300;
static const uint32_t TILT_OFF_CONFIRM_MS = 600;

// =====================================================
//                 BUZZER (LEDC core 3.x)
// =====================================================
static const uint32_t BUZ_FREQ = 2000;
static const uint8_t  BUZ_RES  = 8;
static const uint32_t BUZ_DUTY = 140;
static bool buzzerPwmOK = false;

static const uint32_t FIRE_BEEP_ON_MS  = 200;
static const uint32_t FIRE_BEEP_OFF_MS = 200;

static const uint32_t SILENCE_DEFAULT_MS = 10UL * 60UL * 1000UL;

// =====================================================
//                 GENERAL TIMING
// =====================================================
static const uint32_t SAMPLE_MS = 120;
static const uint32_t DHT_MS    = 1500;

// =====================================================
//                 WEB + EVENTS
// =====================================================
WebServer server(80);

static const int EV_MAX = 35;
struct Event { uint32_t ms; String msg; };
Event ev[EV_MAX];
int evHead = 0, evCount = 0;

static void addEvent(const String& s) {
  ev[evHead] = { millis(), s };
  evHead = (evHead + 1) % EV_MAX;
  if (evCount < EV_MAX) evCount++;
}

// =====================================================
//                 STATE
// =====================================================
struct State {
  // Flame
  int  flame_aoRaw = 0;
  int  flame_strength = 0;
  int  flame_strengthFilt = 0;
  int  flame_doRaw = 0;
  bool flame_d0Detected = false;
  bool flame_a0Detected = false;
  bool fireDetected = false;

  // KY-032
  int  ky_rawDO = 0;
  bool ky_rawActive = false;
  bool ky_majActive = false;
  bool ky_latched = false;
  uint32_t ky_lastEdgeMs = 0;
  int fillPct = 0;

  // KY-020 Tilt
  int  tilt_rawDO = 0;
  bool tilt_rawActive = false;
  bool tilt_active = false;
  bool tilt_latched = false;

  // DHT
  float tempC = 0.0f;
  float humPct = 0.0f;
  bool dhtOK = false;

  // Buzzer
  bool buzzerMuted = false;
  uint32_t silenceUntilMs = 0;
  bool buzzerSilenced = false;

  // System
  int rssi = 0;
  uint32_t uptimeS = 0;
};
State st;

// =====================================================
//                 INTERNALS
// =====================================================
static float flameStrengthEMA = 0.0f;

static uint32_t tSample = 0;
static uint32_t tDht = 0;

static uint32_t tBeep = 0;
static bool beepOn = false;

static bool lastFire = false;

// KY-032 debounce internals
static bool kyMajCandidate = false;
static uint32_t kyMajCandidateSince = 0;
static bool kyMajStable = false;

// KY-020 debounce internals
static bool tiltCandidate = false;
static uint32_t tiltCandidateSince = 0;
static bool tiltStable = false;
static uint32_t tiltLastEdgeMs = 0;
static bool lastTiltLatched = false;

// DHT
DHT dht(PIN_DHT, DHT_TYPE);

// Blynk
BlynkTimer blynkTimer;

// =====================================================
//                 HELPERS
// =====================================================
static inline bool readActive(int pin, bool activeLow) {
  int v = digitalRead(pin);
  return activeLow ? (v == LOW) : (v == HIGH);
}

// Traffic lights ONLY based on flame detection
static void updateTrafficLightsFromFlame(bool fire) {
  digitalWrite(PIN_LED_R, fire ? HIGH : LOW);
  digitalWrite(PIN_LED_Y, LOW);
  digitalWrite(PIN_LED_G, fire ? LOW : HIGH);
}

// LEDC (core 3.x): ledcWrite(pin, duty)
static void buzzerSet(bool on) {
  if (buzzerPwmOK) ledcWrite((uint8_t)PIN_BUZZER, on ? BUZ_DUTY : 0);
  else digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
}

static void stopBuzzer() {
  beepOn = false;
  buzzerSet(false);
}

static void updateBuzzerFireOnly(bool fireAlarm) {
  uint32_t now = millis();
  st.buzzerSilenced = (st.silenceUntilMs != 0 && (int32_t)(st.silenceUntilMs - now) > 0);

  if (!fireAlarm || st.buzzerMuted || st.buzzerSilenced) {
    stopBuzzer();
    return;
  }

  if (!beepOn) {
    if (now - tBeep >= FIRE_BEEP_OFF_MS) {
      tBeep = now;
      beepOn = true;
      buzzerSet(true);
    }
  } else {
    if (now - tBeep >= FIRE_BEEP_ON_MS) {
      tBeep = now;
      beepOn = false;
      buzzerSet(false);
    }
  }
}

// =====================================================
//                 FLAME UPDATE (KY-026)  *** FIXED ***
// =====================================================
static void updateFlame() {
  // Read A0 (for UI only)
  st.flame_aoRaw = analogRead(PIN_FLAME_AO);
  st.flame_strength = 4095 - st.flame_aoRaw;

  flameStrengthEMA = FLAME_EMA_ALPHA * flameStrengthEMA +
                     (1.0f - FLAME_EMA_ALPHA) * (float)st.flame_strength;
  st.flame_strengthFilt = (int)flameStrengthEMA;

  // D0
  st.flame_doRaw = digitalRead(PIN_FLAME_DO);

  // ACTIVE-HIGH detection (no flipping UI)
  st.flame_d0Detected = USE_FLAME_D0 ? readActive(PIN_FLAME_DO, FLAME_D0_ACTIVE_LOW) : false;

  // A0 backup disabled (prevents always-fire when A0 is stuck)
  st.flame_a0Detected = false;

  st.fireDetected = st.flame_d0Detected;

  if (st.fireDetected && !lastFire) addEvent("FIRE DETECTED");
  if (!st.fireDetected && lastFire) addEvent("Fire cleared");
  lastFire = st.fireDetected;
}

// =====================================================
//                 KY-032 update
// =====================================================
static bool kyMajorityActiveRaw() {
  uint8_t c = 0;
  for (uint8_t i = 0; i < KY_MAJ_SAMPLES; i++) {
    if (readActive(PIN_KY032_DO, KY032_ACTIVE_LOW)) c++;
    delayMicroseconds(220);
  }
  return (c >= kyMajRequire);
}

static void updateKY032() {
  uint32_t now = millis();

  st.ky_rawDO = digitalRead(PIN_KY032_DO);
  st.ky_rawActive = readActive(PIN_KY032_DO, KY032_ACTIVE_LOW);

  bool majRaw = kyMajorityActiveRaw();

  if (majRaw != kyMajCandidate) {
    kyMajCandidate = majRaw;
    kyMajCandidateSince = now;
  }

  if ((now - kyMajCandidateSince) >= kyStableMs) {
    if (kyMajStable != kyMajCandidate) {
      kyMajStable = kyMajCandidate;
      st.ky_lastEdgeMs = now;
      addEvent(String("KY032 stable -> ") + (kyMajStable ? "ACTIVE" : "CLEAR"));
    }
  }

  st.ky_majActive = kyMajStable;

  if (!st.ky_latched && st.ky_majActive && (now - st.ky_lastEdgeMs) >= KY_ON_CONFIRM_MS) {
    st.ky_latched = true;
    addEvent("KY032 latched ON");
  }
  if (st.ky_latched && !st.ky_majActive && (now - st.ky_lastEdgeMs) >= KY_OFF_CONFIRM_MS) {
    st.ky_latched = false;
    addEvent("KY032 latched OFF");
  }

  st.fillPct = KY032_IS_TOP_SENSOR ? (st.ky_majActive ? 100 : 0)
                                   : (st.ky_majActive ? 50 : 0);
}

// =====================================================
//                 KY-020 Tilt update
// =====================================================
static void updateKY020Tilt() {
  uint32_t now = millis();

  st.tilt_rawDO = digitalRead(PIN_KY020_DO);
  st.tilt_rawActive = readActive(PIN_KY020_DO, KY020_ACTIVE_LOW);

  bool raw = st.tilt_rawActive;

  if (raw != tiltCandidate) {
    tiltCandidate = raw;
    tiltCandidateSince = now;
  }
  if ((now - tiltCandidateSince) >= tiltStableMs) {
    if (tiltStable != tiltCandidate) {
      tiltStable = tiltCandidate;
      tiltLastEdgeMs = now;
      addEvent(String("KY020 stable -> ") + (tiltStable ? "TILTED" : "OK"));
    }
  }

  st.tilt_active = tiltStable;

  if (!st.tilt_latched && st.tilt_active && (now - tiltLastEdgeMs) >= TILT_ON_CONFIRM_MS) {
    st.tilt_latched = true;
  }
  if (st.tilt_latched && !st.tilt_active && (now - tiltLastEdgeMs) >= TILT_OFF_CONFIRM_MS) {
    st.tilt_latched = false;
  }

  if (st.tilt_latched != lastTiltLatched) {
    lastTiltLatched = st.tilt_latched;
    addEvent(String("TILT -> ") + (st.tilt_latched ? "DETECTED" : "CLEARED"));
  }
}

// =====================================================
//                 DHT11
// =====================================================
static void updateDHT11() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) { st.dhtOK = false; return; }
  st.dhtOK = true;
  st.humPct = h;
  st.tempC = t;
}

// =====================================================
//                 POLARITY setters (KY032 + KY020 only)
// =====================================================
static void setKY032Polarity(bool activeLow, const char* who) {
  KY032_ACTIVE_LOW = activeLow;
  addEvent(String("KY032 activeLow -> ") + (activeLow ? "true" : "false") + " (" + who + ")");
  if (Blynk.connected()) Blynk.virtualWrite(VPIN_KY032_ACTIVELOW, activeLow ? 1 : 0);
}

static void setKY020Polarity(bool activeLow, const char* who) {
  KY020_ACTIVE_LOW = activeLow;
  addEvent(String("KY020 activeLow -> ") + (activeLow ? "true" : "false") + " (" + who + ")");
  if (Blynk.connected()) Blynk.virtualWrite(VPIN_KY020_ACTIVELOW, activeLow ? 1 : 0);
}

// =====================================================
//                 BLYNK WRITE handlers
// =====================================================
BLYNK_WRITE(VPIN_FLAME_THRESHOLD) {
  int v = constrain(param.asInt(), 0, 4095);
  FLAME_STRENGTH_THRESHOLD = v;
  addEvent(String("Flame threshold -> ") + v + " (Blynk)");
  Blynk.virtualWrite(VPIN_FLAME_THRESHOLD, v);
}

BLYNK_WRITE(VPIN_MUTE_BUZZER) {
  st.buzzerMuted = (param.asInt() != 0);
  addEvent(String("Buzzer muted -> ") + (st.buzzerMuted ? "true" : "false") + " (Blynk)");
}

BLYNK_WRITE(VPIN_SILENCE_10MIN) {
  if (param.asInt() != 0) {
    st.silenceUntilMs = millis() + SILENCE_DEFAULT_MS;
    addEvent("Buzzer silenced 10 min (Blynk)");
    Blynk.virtualWrite(VPIN_SILENCE_10MIN, 0);
  }
}

BLYNK_WRITE(VPIN_KY032_ACTIVELOW) { setKY032Polarity(param.asInt() != 0, "Blynk V13"); }
BLYNK_WRITE(VPIN_KY020_ACTIVELOW) { setKY020Polarity(param.asInt() != 0, "Blynk V16"); }

// Fire polarity streams are DISPLAY ONLY now (ignore writes)
BLYNK_WRITE(VPIN_FLAME_D0_ACTIVELOW) { Blynk.virtualWrite(VPIN_FLAME_D0_ACTIVELOW, FLAME_D0_ACTIVE_LOW ? 1 : 0); }
BLYNK_WRITE(VPIN_KY026_ACTIVELOW)     { Blynk.virtualWrite(VPIN_KY026_ACTIVELOW,     FLAME_D0_ACTIVE_LOW ? 1 : 0); }

// =====================================================
//                 PUSH sensor values to Blynk
// =====================================================
static void pushToBlynk() {
  if (!Blynk.connected()) return;

  Blynk.virtualWrite(VPIN_FIRE_DETECTED, st.fireDetected ? 1 : 0);
  Blynk.virtualWrite(VPIN_FLAME_STRENGTH_FILT, st.flame_strengthFilt);
  Blynk.virtualWrite(VPIN_FLAME_A0_RAW, st.flame_aoRaw);
  Blynk.virtualWrite(VPIN_BIN_FILL_PCT, st.fillPct);
  Blynk.virtualWrite(VPIN_TILT_DETECTED, st.tilt_latched ? 1 : 0);

  if (st.dhtOK) {
    Blynk.virtualWrite(VPIN_TEMP_C, st.tempC);
    Blynk.virtualWrite(VPIN_HUM_PCT, st.humPct);
  }

  Blynk.virtualWrite(VPIN_FLAME_THRESHOLD, FLAME_STRENGTH_THRESHOLD);
  Blynk.virtualWrite(VPIN_MUTE_BUZZER, st.buzzerMuted ? 1 : 0);
  Blynk.virtualWrite(VPIN_KY032_ACTIVELOW, KY032_ACTIVE_LOW ? 1 : 0);

  // Fire polarity fixed display
  Blynk.virtualWrite(VPIN_FLAME_D0_ACTIVELOW, FLAME_D0_ACTIVE_LOW ? 1 : 0);
  Blynk.virtualWrite(VPIN_KY026_ACTIVELOW,     FLAME_D0_ACTIVE_LOW ? 1 : 0);

  Blynk.virtualWrite(VPIN_KY020_ACTIVELOW, KY020_ACTIVE_LOW ? 1 : 0);
}

// =====================================================
//                 WEB API
// =====================================================
static void apiStatus() {
  uint32_t now = millis();
  int32_t remain = (st.silenceUntilMs != 0) ? (int32_t)(st.silenceUntilMs - now) : 0;
  if (remain < 0) remain = 0;

  String j = "{";
  j += "\"fireDetected\":" + String(st.fireDetected ? "true":"false") + ",";

  j += "\"tempAlarmC\":" + String(TEMP_ALARM_C, 1) + ",";
  j += "\"binFullPct\":" + String(BIN_FULL_PCT) + ",";

  j += "\"flame_aoRaw\":" + String(st.flame_aoRaw) + ",";
  j += "\"flame_strength\":" + String(st.flame_strength) + ",";
  j += "\"flame_strengthFilt\":" + String(st.flame_strengthFilt) + ",";
  j += "\"flame_strengthThreshold\":" + String(FLAME_STRENGTH_THRESHOLD) + ",";
  j += "\"flame_doRaw\":" + String(st.flame_doRaw) + ",";
  j += "\"flame_d0Detected\":" + String(st.flame_d0Detected ? "true":"false") + ",";
  j += "\"flame_a0Detected\":false,";
  j += "\"flameD0ActiveLow\":" + String(FLAME_D0_ACTIVE_LOW ? "true":"false") + ",";

  j += "\"fillPct\":" + String(st.fillPct) + ",";
  j += "\"ky_rawDO\":" + String(st.ky_rawDO) + ",";
  j += "\"ky_rawActive\":" + String(st.ky_rawActive ? "true":"false") + ",";
  j += "\"ky_majActive\":" + String(st.ky_majActive ? "true":"false") + ",";
  j += "\"ky_latched\":" + String(st.ky_latched ? "true":"false") + ",";
  j += "\"ky_activeLow\":" + String(KY032_ACTIVE_LOW ? "true":"false") + ",";
  j += "\"ky_req\":" + String(kyMajRequire) + ",";
  j += "\"ky_samples\":" + String(KY_MAJ_SAMPLES) + ",";
  j += "\"ky_stableMs\":" + String(kyStableMs) + ",";

  j += "\"tilt_rawDO\":" + String(st.tilt_rawDO) + ",";
  j += "\"tilt_rawActive\":" + String(st.tilt_rawActive ? "true":"false") + ",";
  j += "\"tilt_active\":" + String(st.tilt_active ? "true":"false") + ",";
  j += "\"tilt_latched\":" + String(st.tilt_latched ? "true":"false") + ",";
  j += "\"tilt_activeLow\":" + String(KY020_ACTIVE_LOW ? "true":"false") + ",";
  j += "\"tilt_stableMs\":" + String(tiltStableMs) + ",";

  j += "\"dhtOK\":" + String(st.dhtOK ? "true":"false") + ",";
  j += "\"tempC\":" + String(st.dhtOK ? String(st.tempC, 1) : "null") + ",";
  j += "\"humPct\":" + String(st.dhtOK ? String(st.humPct, 1) : "null") + ",";

  j += "\"buzzerMuted\":" + String(st.buzzerMuted ? "true":"false") + ",";
  j += "\"buzzerPwmOK\":" + String(buzzerPwmOK ? "true":"false") + ",";
  j += "\"buzzerSilenced\":" + String(st.buzzerSilenced ? "true":"false") + ",";
  j += "\"silenceRemainMs\":" + String((uint32_t)remain) + ",";

  j += "\"rssi\":" + String(st.rssi) + ",";
  j += "\"uptimeS\":" + String(st.uptimeS);
  j += "}";

  server.send(200, "application/json", j);
}

static void apiEvents() {
  String j = "{\"count\":" + String(evCount) + ",\"events\":[";
  int start = (evHead - evCount + EV_MAX) % EV_MAX;
  for (int i = 0; i < evCount; i++) {
    int idx = (start + i) % EV_MAX;
    if (i) j += ",";
    j += "{\"ms\":" + String(ev[idx].ms) + ",\"msg\":\"";
    String m = ev[idx].msg; m.replace("\"", "'");
    j += m + "\"}";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

static void apiCmd() {
  String cmd = server.arg("cmd");

  if (cmd == "mute") { st.buzzerMuted = true; addEvent("Buzzer muted (Web)"); server.send(200,"text/plain","OK"); return; }
  if (cmd == "unmute") { st.buzzerMuted = false; addEvent("Buzzer unmuted (Web)"); server.send(200,"text/plain","OK"); return; }

  if (cmd == "silence") {
    uint32_t ms = server.hasArg("ms") ? (uint32_t)server.arg("ms").toInt() : SILENCE_DEFAULT_MS;
    st.silenceUntilMs = millis() + ms;
    addEvent("Buzzer silenced (timed) (Web)");
    server.send(200,"text/plain","OK");
    return;
  }
  if (cmd == "unsilence") { st.silenceUntilMs = 0; addEvent("Buzzer unsilenced (Web)"); server.send(200,"text/plain","OK"); return; }

  if (cmd == "setFlameTh") {
    if (!server.hasArg("v")) { server.send(400,"text/plain","ERR"); return; }
    int v = constrain(server.arg("v").toInt(), 0, 4095);
    FLAME_STRENGTH_THRESHOLD = v;
    addEvent("Flame threshold -> " + String(v) + " (Web)");
    server.send(200,"text/plain", String("OK ") + v);
    return;
  }

  if (cmd == "flipKy032") { setKY032Polarity(!KY032_ACTIVE_LOW, "Web"); server.send(200,"text/plain","OK"); return; }
  if (cmd == "setKyReq") {
    if (!server.hasArg("v")) { server.send(400,"text/plain","ERR"); return; }
    int v = constrain(server.arg("v").toInt(), 1, (int)KY_MAJ_SAMPLES);
    kyMajRequire = (uint8_t)v;
    addEvent("KY032 kyMajRequire -> " + String(kyMajRequire) + " (Web)");
    server.send(200,"text/plain","OK");
    return;
  }

  if (cmd == "flipTilt") { setKY020Polarity(!KY020_ACTIVE_LOW, "Web"); server.send(200,"text/plain","OK"); return; }

  if (cmd == "clearEvents") { evHead = 0; evCount = 0; server.send(200,"text/plain","OK"); return; }
  server.send(400, "text/plain", "ERR");
}

// =====================================================
//                 MODERN UI (HTML in FLASH)
// =====================================================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Smart Garbage IoT</title>
<style>
:root{
  --bg0:#070a14; --bg1:#0b1220;
  --panel: rgba(20, 26, 45, .62);
  --panel2: rgba(18, 24, 40, .55);
  --stroke: rgba(255,255,255,.10);
  --stroke2: rgba(255,255,255,.06);
  --txt: #eaf0ff;
  --muted: rgba(234,240,255,.70);
  --good:#36e2a0; --warn:#ffd166; --bad:#ff5c7a;
  --brand:#3aa0ff; --brand2:#7b61ff;
  --shadow: 0 18px 60px rgba(0,0,0,.45);
  --radius: 18px;
}
*{box-sizing:border-box}
html,body{height:100%}
body{
  margin:0; color:var(--txt);
  font-family: ui-sans-serif, -apple-system, system-ui, Segoe UI, Roboto, Arial;
  background: radial-gradient(900px 600px at 15% 20%, rgba(58,160,255,.22), transparent 60%),
              radial-gradient(800px 520px at 85% 25%, rgba(123,97,255,.18), transparent 60%),
              radial-gradient(900px 700px at 40% 90%, rgba(255,92,122,.10), transparent 65%),
              linear-gradient(180deg, var(--bg0), var(--bg1));
  overflow-x:hidden;
}
.bgblur{position:fixed; inset:-30px; filter: blur(14px); opacity:.65; pointer-events:none;}
.wrap{min-height:100%; display:flex; align-items:center; justify-content:center; padding:26px 14px;}
.panel{
  width:min(1040px, 100%);
  background: var(--panel);
  border: 1px solid var(--stroke);
  border-radius: 22px;
  box-shadow: var(--shadow);
  backdrop-filter: blur(18px);
  -webkit-backdrop-filter: blur(18px);
  padding: 18px;
}
.topbar{
  display:flex; gap:14px; align-items:center; justify-content:space-between;
  padding:10px 10px 16px; border-bottom:1px solid var(--stroke2);
}
.brand{display:flex; gap:10px; align-items:center;}
.logo{width:42px;height:42px;border-radius:12px;background:linear-gradient(135deg, rgba(58,160,255,.95), rgba(123,97,255,.95));}
.hgroup h1{font-size:18px;margin:0}
.hgroup p{margin:2px 0 0;font-size:12px;color:var(--muted)}
.pills{display:flex;gap:10px;flex-wrap:wrap;justify-content:flex-end}
.pill{
  background:rgba(0,0,0,.22); border:1px solid var(--stroke2);
  padding:10px 12px; border-radius:999px; font-size:12px; color:var(--muted);
}
.pill b{color:var(--txt);font-weight:700}
.grid{display:grid;grid-template-columns:1.15fr .85fr;gap:14px;padding:14px 4px 6px}
@media (max-width:980px){.grid{grid-template-columns:1fr}}
.card{background:var(--panel2); border:1px solid var(--stroke2); border-radius:18px; padding:14px;}
.card h3{margin:0 0 10px;font-size:14px;letter-spacing:.2px;display:flex;align-items:center;gap:8px}
.kv{
  display:flex;justify-content:space-between;gap:10px;
  padding:8px 0;border-bottom:1px dashed rgba(255,255,255,.08)
}
.kv:last-child{border-bottom:none}
.kv .k{color:rgba(234,240,255,.70);font-size:12px}
.kv .v{font-weight:800;font-size:13px;text-align:right}
.bad{color:#ff5c7a} .ok{color:#36e2a0} .warn{color:#ffd166}
.log{height:340px;overflow:auto;background:rgba(0,0,0,.25); border:1px solid rgba(255,255,255,.08);
  border-radius:14px;padding:10px;font-family:ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;font-size:12px;}
.controls{display:flex;gap:10px;flex-wrap:wrap;margin-top:10px}
.btn{
  appearance:none;border:none;border-radius:14px;padding:10px 12px;font-weight:800;color:#eaf0ff;
  background:rgba(255,255,255,.08); border:1px solid rgba(255,255,255,.10);
  cursor:pointer;
}
.btn.primary{background:linear-gradient(135deg, rgba(58,160,255,.95), rgba(123,97,255,.90)); border:1px solid rgba(255,255,255,.18)}
.btn.danger{background:linear-gradient(135deg, rgba(255,92,122,.92), rgba(255,153,102,.45)); border:1px solid rgba(255,255,255,.18)}
.inp{width:160px;max-width:100%;padding:10px 12px;border-radius:14px;outline:none;color:#eaf0ff;
  background:rgba(0,0,0,.28); border:1px solid rgba(255,255,255,.10)}
hr.sep{border:0;border-top:1px solid rgba(255,255,255,.08);margin:12px 0}
.small{font-size:12px;color:rgba(234,240,255,.70)}
</style>
</head>
<body>
<div class="bgblur"></div>

<div class="wrap">
  <div class="panel">
    <div class="topbar">
      <div class="brand">
        <div class="logo"></div>
        <div class="hgroup">
          <h1>Smart Garbage IoT</h1>
          <p>KY-026 Flame • KY-032 Level • KY-020 Tilt • KY-015 DHT11</p>
        </div>
      </div>
      <div class="pills">
        <div class="pill">Uptime: <b id="up">--</b></div>
        <div class="pill">RSSI: <b id="rssi">--</b></div>
        <div class="pill">Fire: <b id="fire">--</b></div>
      </div>
    </div>

    <div class="grid">
      <div class="card">
        <h3>🔥 Fire (KY-026) — drives traffic lights + buzzer</h3>

        <div class="kv"><div class="k">FIRE DETECTED</div><div class="v" id="fire2">--</div></div>
        <div class="kv"><div class="k">D0 Raw</div><div class="v" id="fdo">--</div></div>
        <div class="kv"><div class="k">D0 Detected</div><div class="v" id="fdod">--</div></div>
        <div class="kv"><div class="k">D0 activeLow (fixed)</div><div class="v" id="fd0pol">--</div></div>
        <div class="kv"><div class="k">A0 Raw</div><div class="v" id="fao">--</div></div>
        <div class="kv"><div class="k">Strength (Filtered)</div><div class="v" id="fstf">--</div></div>

        <hr class="sep">
        <div class="kv"><div class="k">Current Threshold</div><div class="v" id="fth">--</div></div>
        <div class="controls">
          <input class="inp" id="flTh" type="number" min="0" max="4095" step="1" placeholder="0..4095">
          <button class="btn primary" onclick="setFlTh()">Set Threshold</button>
        </div>
        <div class="small">Input will not be overwritten while you type.</div>

        <hr class="sep">
        <div class="kv"><div class="k">Buzzer Muted</div><div class="v" id="muted">--</div></div>
        <div class="kv"><div class="k">Timed Silenced</div><div class="v" id="sil">--</div></div>
        <div class="controls">
          <button class="btn" onclick="cmd('mute')">Mute</button>
          <button class="btn" onclick="cmd('unmute')">Unmute</button>
          <button class="btn" onclick="cmd('silence')">Silence 10 min</button>
          <button class="btn" onclick="cmd('unsilence')">Unsilence</button>
        </div>
      </div>

      <div>
        <div class="card" style="margin-bottom:14px">
          <h3>🧭 Tilt (KY-020)</h3>
          <div class="kv"><div class="k">Tilt (latched)</div><div class="v" id="tiltLat">--</div></div>
          <div class="kv"><div class="k">Stable</div><div class="v" id="tiltSt">--</div></div>
          <div class="kv"><div class="k">Raw DO</div><div class="v" id="tiltRaw">--</div></div>
          <div class="kv"><div class="k">activeLow</div><div class="v" id="tiltPol">--</div></div>
          <div class="controls">
            <button class="btn" onclick="cmd('flipTilt')">Flip Polarity</button>
          </div>
          <div class="small">No buzzer/LED action here — only detection + reporting.</div>
        </div>

        <div class="card" style="margin-bottom:14px">
          <h3>🗑️ Bin Level (KY-032)</h3>
          <div class="kv"><div class="k">Fill %</div><div class="v" id="fill">--</div></div>
          <div class="kv"><div class="k">Stable Detected</div><div class="v" id="kyMaj">--</div></div>
          <div class="kv"><div class="k">Latched</div><div class="v" id="kyLat">--</div></div>
          <div class="kv"><div class="k">activeLow</div><div class="v" id="kyPol">--</div></div>
          <div class="kv"><div class="k">Majority require</div><div class="v" id="kyReqLbl">--</div></div>

          <div class="controls">
            <input class="inp" id="kyReq" type="number" min="1" max="15" step="1" placeholder="1..15">
            <button class="btn primary" onclick="setKyReq()">Set Req</button>
            <button class="btn" onclick="cmd('flipKy032')">Flip Polarity</button>
          </div>

          <hr class="sep">
          <div class="small">Debug rawDO=<span id="kyRaw">--</span>, rawActive=<span id="kyRawAct">--</span></div>
        </div>

        <div class="card" style="margin-bottom:14px">
          <h3>🌡️ Temperature / Humidity (DHT11 KY-015)</h3>
          <div class="kv"><div class="k">Status</div><div class="v" id="dhtok">--</div></div>
          <div class="kv"><div class="k">Temperature</div><div class="v" id="tc">--</div></div>
          <div class="kv"><div class="k">Humidity</div><div class="v" id="hp">--</div></div>
        </div>

        <div class="card">
          <h3>🧾 Events</h3>
          <div class="log" id="events">Loading...</div>
          <div class="controls" style="margin-top:10px">
            <button class="btn danger" onclick="cmd('clearEvents')">Clear Log</button>
          </div>
        </div>
      </div>
    </div>

  </div>
</div>

<script>
async function cmd(c){ await fetch('/api/cmd?cmd='+c); }

async function setFlTh(){
  const inp = document.getElementById('flTh');
  await fetch('/api/cmd?cmd=setFlameTh&v='+encodeURIComponent(inp.value));
  tick().catch(()=>{});
}
async function setKyReq(){
  const inp = document.getElementById('kyReq');
  await fetch('/api/cmd?cmd=setKyReq&v='+encodeURIComponent(inp.value));
  tick().catch(()=>{});
}

function setBool(el, v){
  el.textContent = v ? "TRUE" : "FALSE";
  el.className = "v " + (v ? "bad" : "ok");
}

async function tick(){
  const s = await (await fetch('/api/status',{cache:'no-store'})).json();

  document.getElementById('up').textContent = s.uptimeS + " s";
  document.getElementById('rssi').textContent = s.rssi + " dBm";
  setBool(document.getElementById('fire'), s.fireDetected);

  setBool(document.getElementById('fire2'), s.fireDetected);
  document.getElementById('fdo').textContent = s.flame_doRaw;
  setBool(document.getElementById('fdod'), s.flame_d0Detected);
  setBool(document.getElementById('fd0pol'), s.flameD0ActiveLow);
  document.getElementById('fao').textContent = s.flame_aoRaw;
  document.getElementById('fstf').textContent = s.flame_strengthFilt;
  document.getElementById('fth').textContent = s.flame_strengthThreshold;

  const flInp = document.getElementById('flTh');
  if (document.activeElement !== flInp) flInp.value = s.flame_strengthThreshold;

  const muted = document.getElementById('muted');
  muted.textContent = s.buzzerMuted ? "YES" : "NO";
  muted.className = "v " + (s.buzzerMuted ? "warn" : "ok");

  const sil = document.getElementById('sil');
  sil.textContent = s.buzzerSilenced ? ("YES ("+Math.round(s.silenceRemainMs/1000)+"s)") : "NO";
  sil.className = "v " + (s.buzzerSilenced ? "warn" : "ok");

  // Tilt
  setBool(document.getElementById('tiltLat'), s.tilt_latched);
  setBool(document.getElementById('tiltSt'), s.tilt_active);
  document.getElementById('tiltRaw').textContent = s.tilt_rawDO;
  setBool(document.getElementById('tiltPol'), s.tilt_activeLow);

  // KY-032
  document.getElementById('fill').textContent = s.fillPct + "%";
  setBool(document.getElementById('kyMaj'), s.ky_majActive);
  setBool(document.getElementById('kyLat'), s.ky_latched);
  setBool(document.getElementById('kyPol'), s.ky_activeLow);
  document.getElementById('kyReqLbl').textContent = s.ky_req + " / " + s.ky_samples;

  const kyInp = document.getElementById('kyReq');
  if (document.activeElement !== kyInp) kyInp.value = s.ky_req;

  document.getElementById('kyRaw').textContent = s.ky_rawDO;
  setBool(document.getElementById('kyRawAct'), s.ky_rawActive);

  // DHT
  setBool(document.getElementById('dhtok'), s.dhtOK);
  document.getElementById('tc').textContent = (s.tempC === null) ? "--" : (Number(s.tempC).toFixed(1) + " °C");
  document.getElementById('hp').textContent = (s.humPct === null) ? "--" : (Number(s.humPct).toFixed(1) + " %");
}

async function tickEvents(){
  const e = await (await fetch('/api/events',{cache:'no-store'})).json();
  const lines = e.events.map(x => `[${Math.round(x.ms/1000)}s] ${x.msg}`);
  document.getElementById('events').textContent = lines.length ? lines.join("\n") : "(no events yet)";
}

setInterval(()=>{ tick().catch(()=>{}); tickEvents().catch(()=>{}); }, 800);
tick(); tickEvents();
</script>
</body>
</html>
)rawliteral";

static void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

// =====================================================
//                 SETUP / LOOP
// =====================================================
void setup() {
  Serial.begin(115200);
  Serial.println("BOOT: start");

  // Inputs
  // NOTE: DO is active-high in logic, but we can still use INPUT_PULLUP safely.
  pinMode(PIN_FLAME_DO, INPUT_PULLUP);
  pinMode(PIN_KY032_DO, KY032_USE_PULLUP ? INPUT_PULLUP : INPUT);
  pinMode(PIN_KY020_DO, KY020_USE_PULLUP ? INPUT_PULLUP : INPUT);

  // Outputs
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_Y, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);

  // ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(PIN_FLAME_AO, ADC_11db);

  // LEDC attach (core 3.x)
  buzzerPwmOK = ledcAttach((uint8_t)PIN_BUZZER, BUZ_FREQ, BUZ_RES);
  buzzerSet(false);

  // DHT
  dht.begin();

  // WiFi (TIMEOUT, no infinite stuck)
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting WiFi");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 12000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    addEvent("WiFi OK");
  } else {
    Serial.println("WiFi FAILED (timeout) -> continuing");
    addEvent("WiFi FAILED");
  }

  addEvent("Boot OK");

  // Web routes
  server.on("/", handleRoot);
  server.on("/api/status", apiStatus);
  server.on("/api/events", apiEvents);
  server.on("/api/cmd", apiCmd);
  server.begin();
  Serial.println("BOOT: web up");

  // Blynk (non-blocking)
  Blynk.config(BLYNK_AUTH_TOKEN);
  // no Blynk.connect(...) here; Blynk.run() will keep trying

  // Push values to Blynk periodically
  blynkTimer.setInterval(500L, pushToBlynk);

  Serial.println("BOOT: done");
}

void loop() {
  server.handleClient();

  Blynk.run();
  blynkTimer.run();

  uint32_t now = millis();

  if ((int32_t)(now - tSample) >= (int32_t)SAMPLE_MS) {
    tSample = now;

    updateFlame();
    updateKY032();
    updateKY020Tilt();

    st.rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
    st.uptimeS = now / 1000UL;
  }

  if ((int32_t)(now - tDht) >= (int32_t)DHT_MS) {
    tDht = now;
    updateDHT11();
  }

  updateTrafficLightsFromFlame(st.fireDetected);
  updateBuzzerFireOnly(st.fireDetected);
}
