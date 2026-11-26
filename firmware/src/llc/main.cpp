/*
  llc - rover1 low-level controller (ESP32, emakefun Maker-ESP32-Pro)

  Closed-loop wheel speed control for ROS. The Jetson sends wheel speeds; the ESP32 holds them with a
  PI controller per wheel at 100 Hz and reports encoder counts at 50 Hz. If commands stop arriving
  the motors stop (watchdog).

  Serial: 115200 baud, text lines ending in newline. Fields are separated by commas or spaces.
  A line may end in *HH, the XOR of every character before the '*', in hex (NMEA style). If present it
  is checked and a bad line is ignored; if absent the line is accepted (so commands can be typed by
  hand). The ROS driver always sends it.

  Commands (Jetson -> ESP32):
    V <left> <right>      wheel speed targets, counts/s (forward +). Closed loop. Resets the watchdog.
    P <left> <right>      open-loop power, -100..100 %. Bypasses the controller. Resets the watchdog.
    S                     stop (coast) and reset the controllers
    Z                     zero the encoder counts
    K <kp> <ki> <kf> <ks> controller gains (see below); replies with the gains in use
    W <ms>                watchdog timeout, 100..5000 ms (default 300). Raise it for typing by hand.
    T <ms>                telemetry period, 0 = off, else 10..1000 ms (default 20 = 50 Hz)
    I                     info: name, version, build time, settings

  Replies (ESP32 -> Jetson):
    T,<ms>,<left count>,<right count>,<left c/s>,<right c/s>,<left %>,<right %>,<mode>*HH
        ms = ESP32 uptime; counts are cumulative (forward +); c/s = measured speed over the last 40 ms;
        % = power applied; mode: I idle, V speed control, P open loop, W stopped by the watchdog.
    I,rover1 llc,<version>,<build>,...    A,<what was set>    E,<error>

  Units: 3960 counts per wheel turn (rover_hw.h). rad/s = counts/s x 2 pi / 3960. Full speed is about
  6800 counts/s (about 103 rpm, 0.46 m/s) with the wheels off the ground at 12 V.

  Controller, per wheel, every 10 ms:
    power % = kf x target + ks x sign(target) + kp x error + integral(ki x error)
  kf and ks come from the motor_test sweep (brake PWM, 20 kHz): about 68 counts/s per % above a 2.4 %
  offset. kp and ki correct the rest (load, battery voltage, the motors' 3 % difference). Target 0 =
  coast, integral cleared. The target changes at most ACCEL_CPS2 per second.

  Version 0.1.0 - created 2026-11-25, last modified 2026-11-25
  History:
    0.1.0  2026-11-25  First version: V/P/S/Z/K/W/T/I commands, PI + feedforward speed control at
                       100 Hz, 300 ms watchdog, 50 Hz telemetry, optional checksums. Bench test only.
*/
#include <Arduino.h>
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include <rover_hw.h>

#define FW_NAME    "rover1 llc"
#define FW_VERSION "0.1.0"

using namespace rover;

// ---- settings ---------------------------------------------------------------------------------
const uint32_t CONTROL_US = 10000;        // 100 Hz control loop
const int      VEL_WINDOW = 4;            // speed measured over 4 control ticks = 40 ms
const float    ACCEL_CPS2 = 20000.0f;     // target slew limit, counts/s per second (0 to full in ~0.35 s)
const float    MAX_TARGET = 7000.0f;      // counts/s
uint32_t watchdog_ms  = 300;
uint32_t telemetry_ms = 20;

float kp = 0.008f, ki = 0.05f, kf = 1.0f / 68.0f, ks = 2.4f;

// ---- hardware ---------------------------------------------------------------------------------
const uint32_t PWM_FULL = 1u << PWM_BITS;   // ledcWrite(pin, PWM_FULL) = steady HIGH

void motorPinsSafe() {                      // first thing at boot: all motor inputs LOW (GPIO 12!)
  for (int p : {L_FI, L_BI, R_FI, R_BI}) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
}

void setupPwm() {
  for (int p : {L_FI, L_BI, R_FI, R_BI}) {
    if (!ledcAttach(p, PWM_FREQ_HZ, PWM_BITS)) Serial.printf("E,PWM attach failed on GPIO %d\n", p);
    ledcWrite(p, 0);
  }
}

// Brake-mode PWM: forward = FI held HIGH, BI pulsed with the inverse duty (drive / brake).
// pct -100..100; 0 = coast (both LOW).
void drive(int fi, int bi, bool invert, float pct) {
  if (invert) pct = -pct;
  pct = constrain(pct, -100.0f, 100.0f);
  uint32_t duty = (uint32_t)(fabsf(pct) * PWM_FULL / 100.0f + 0.5f);
  if (duty == 0) { ledcWrite(fi, 0); ledcWrite(bi, 0); return; }
  int on = pct > 0 ? fi : bi, other = pct > 0 ? bi : fi;
  ledcWrite(on, PWM_FULL);
  ledcWrite(other, PWM_FULL - duty);
}

pcnt_unit_handle_t makeEncoder(int pin_a, int pin_b) {
  pcnt_unit_config_t uc = {};
  uc.low_limit = -30000;
  uc.high_limit = 30000;
  uc.flags.accum_count = 1;                       // keep counting past +-30000
  pcnt_unit_handle_t unit = nullptr;
  ESP_ERROR_CHECK(pcnt_new_unit(&uc, &unit));
  pcnt_glitch_filter_config_t fc = {};
  fc.max_glitch_ns = 1000;
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &fc));
  pcnt_chan_config_t ca = {}; ca.edge_gpio_num = pin_a; ca.level_gpio_num = pin_b;
  pcnt_chan_config_t cb = {}; cb.edge_gpio_num = pin_b; cb.level_gpio_num = pin_a;
  pcnt_channel_handle_t ch_a = nullptr, ch_b = nullptr;
  ESP_ERROR_CHECK(pcnt_new_channel(unit, &ca, &ch_a));
  ESP_ERROR_CHECK(pcnt_new_channel(unit, &cb, &ch_b));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, uc.high_limit));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, uc.low_limit));
  ESP_ERROR_CHECK(pcnt_unit_enable(unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
  ESP_ERROR_CHECK(pcnt_unit_start(unit));
  return unit;
}

// ---- one wheel --------------------------------------------------------------------------------
struct Wheel {
  int fi, bi;
  bool motor_invert, enc_invert;
  pcnt_unit_handle_t unit = nullptr;
  long hist[VEL_WINDOW + 1] = {};   // counts at the last ticks, for the speed window
  int  hist_i = 0;
  long count = 0;                   // cumulative, forward +
  float speed = 0;                  // measured counts/s
  float target = 0;                 // requested counts/s
  float ramped = 0;                 // target after the slew limit
  float integ = 0;                  // integral term, %
  float out = 0;                    // applied power, %

  long read() {
    int c = 0;
    pcnt_unit_get_count(unit, &c);
    return enc_invert ? -c : c;
  }
  void zero() {
    pcnt_unit_clear_count(unit);
    count = 0;
    for (long &h : hist) h = 0;
  }
  void apply(float pct) { out = pct; drive(fi, bi, motor_invert, pct); }
  void reset() { target = ramped = integ = 0; apply(0); }

  void measure() {                   // every control tick
    count = read();
    hist_i = (hist_i + 1) % (VEL_WINDOW + 1);
    hist[hist_i] = count;
    long oldest = hist[(hist_i + 1) % (VEL_WINDOW + 1)];
    speed = (count - oldest) * (1e6f / (CONTROL_US * VEL_WINDOW));
  }

  void control(float dt) {           // closed loop, every control tick
    float step = ACCEL_CPS2 * dt;
    ramped += constrain(target - ramped, -step, step);
    if (ramped == 0.0f) { integ = 0; apply(0); return; }
    float err = ramped - speed;
    float base = kf * ramped + (ramped > 0 ? ks : -ks) + kp * err;
    float next_integ = integ + ki * err * dt;
    float u = base + next_integ;
    if (u > 100.0f || u < -100.0f) {               // saturated: keep the integral unless it unwinds
      if ((u > 100.0f && err < 0) || (u < -100.0f && err > 0)) integ = next_integ;
    } else {
      integ = next_integ;
    }
    integ = constrain(integ, -50.0f, 50.0f);
    apply(constrain(base + integ, -100.0f, 100.0f));
  }
};

Wheel left  {L_FI, L_BI, LEFT_MOTOR_INVERT,  LEFT_ENC_INVERT};
Wheel right {R_FI, R_BI, RIGHT_MOTOR_INVERT, RIGHT_ENC_INVERT};

// ---- state ------------------------------------------------------------------------------------
enum Mode : char { IDLE = 'I', SPEED = 'V', OPEN = 'P', TRIPPED = 'W' };
Mode mode = IDLE;
uint32_t last_cmd_ms = 0;

void stopAll(Mode m) { left.reset(); right.reset(); mode = m; }

// ---- serial out (with checksum) ---------------------------------------------------------------
void sendLine(const char *body) {
  uint8_t cs = 0;
  for (const char *p = body; *p; p++) cs ^= (uint8_t)*p;
  Serial.printf("%s*%02X\n", body, cs);
}

void sendInfo() {
  char b[200];
  snprintf(b, sizeof b, "I,%s,%s,%s %s,cpr=%.0f,wd=%lu,tel=%lu,kp=%.4f,ki=%.4f,kf=%.5f,ks=%.2f",
           FW_NAME, FW_VERSION, __DATE__, __TIME__, COUNTS_PER_REV, (unsigned long)watchdog_ms,
           (unsigned long)telemetry_ms, kp, ki, kf, ks);
  sendLine(b);
}

void sendTelemetry() {
  char b[120];
  snprintf(b, sizeof b, "T,%lu,%ld,%ld,%d,%d,%d,%d,%c", (unsigned long)millis(), left.count, right.count,
           (int)lroundf(left.speed), (int)lroundf(right.speed), (int)lroundf(left.out),
           (int)lroundf(right.out), (char)mode);
  sendLine(b);
}

void sendError(const char *what) {
  char b[100];
  snprintf(b, sizeof b, "E,%s", what);
  sendLine(b);
}

// ---- serial in --------------------------------------------------------------------------------
char line[96];
size_t line_len = 0;
bool line_overflow = false;

void handleLine(char *s) {
  // checksum, if present
  char *star = strchr(s, '*');
  if (star) {
    uint8_t cs = 0;
    for (char *p = s; p < star; p++) cs ^= (uint8_t)*p;
    if (strtoul(star + 1, nullptr, 16) != cs || strlen(star + 1) != 2) { sendError("checksum"); return; }
    *star = 0;
  }
  // split into fields
  char *f[6]; int n = 0;
  for (char *tok = strtok(s, ", \t"); tok && n < 6; tok = strtok(nullptr, ", \t")) f[n++] = tok;
  if (n == 0) return;
  char cmd = toupper(f[0][0]);
  if (f[0][1] != 0) { sendError("unknown command"); return; }

  switch (cmd) {
    case 'V':
      if (n != 3) { sendError("V needs 2 values"); return; }
      left.target  = constrain(strtof(f[1], nullptr), -MAX_TARGET, MAX_TARGET);
      right.target = constrain(strtof(f[2], nullptr), -MAX_TARGET, MAX_TARGET);
      if (mode != SPEED) { left.integ = right.integ = 0; left.ramped = left.speed; right.ramped = right.speed; }
      mode = SPEED;
      last_cmd_ms = millis();
      break;
    case 'P':
      if (n != 3) { sendError("P needs 2 values"); return; }
      mode = OPEN;
      left.target = right.target = left.ramped = right.ramped = left.integ = right.integ = 0;
      left.apply(constrain(strtof(f[1], nullptr), -100.0f, 100.0f));
      right.apply(constrain(strtof(f[2], nullptr), -100.0f, 100.0f));
      last_cmd_ms = millis();
      break;
    case 'S':
      stopAll(IDLE);
      sendLine("A,stopped");
      break;
    case 'Z':
      left.zero(); right.zero();
      sendLine("A,counts zeroed");
      break;
    case 'K': {
      if (n != 5) { sendError("K needs kp ki kf ks"); return; }
      kp = strtof(f[1], nullptr); ki = strtof(f[2], nullptr);
      kf = strtof(f[3], nullptr); ks = strtof(f[4], nullptr);
      left.integ = right.integ = 0;
      char b[100];
      snprintf(b, sizeof b, "A,kp=%.4f,ki=%.4f,kf=%.5f,ks=%.2f", kp, ki, kf, ks);
      sendLine(b);
      break;
    }
    case 'W': {
      if (n != 2) { sendError("W needs ms"); return; }
      watchdog_ms = constrain(strtoul(f[1], nullptr, 10), 100UL, 5000UL);
      char b[40]; snprintf(b, sizeof b, "A,wd=%lu", (unsigned long)watchdog_ms); sendLine(b);
      break;
    }
    case 'T': {
      if (n != 2) { sendError("T needs ms"); return; }
      unsigned long t = strtoul(f[1], nullptr, 10);
      telemetry_ms = (t == 0) ? 0 : constrain(t, 10UL, 1000UL);
      char b[40]; snprintf(b, sizeof b, "A,tel=%lu", (unsigned long)telemetry_ms); sendLine(b);
      break;
    }
    case 'I':
      sendInfo();
      break;
    default:
      sendError("unknown command");
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (!line_overflow && line_len > 0) { line[line_len] = 0; handleLine(line); }
      else if (line_overflow) sendError("line too long");
      line_len = 0; line_overflow = false;
    } else if (line_len < sizeof(line) - 1) {
      line[line_len++] = c;
    } else {
      line_overflow = true;
    }
  }
}

// ---- main -------------------------------------------------------------------------------------
uint32_t next_control_us = 0, last_tel_ms = 0;

void setup() {
  motorPinsSafe();                                  // first: GPIO 12 low before anything else
  Serial.setRxBufferSize(512);
  Serial.setTxBufferSize(1024);                     // telemetry must never block the control loop
  Serial.begin(115200);
  setupPwm();
  for (int p : {L_ENC_A, L_ENC_B, R_ENC_A, R_ENC_B}) pinMode(p, INPUT);
  esp_log_level_set("gpio", ESP_LOG_NONE);          // PCNT tries pull-ups on input-only pins: harmless
  left.unit = makeEncoder(L_ENC_A, L_ENC_B);
  right.unit = makeEncoder(R_ENC_A, R_ENC_B);
  esp_log_level_set("gpio", ESP_LOG_ERROR);
  delay(200);
  sendInfo();
  next_control_us = micros();
}

void loop() {
  readSerial();

  if ((mode == SPEED || mode == OPEN) && millis() - last_cmd_ms > watchdog_ms) stopAll(TRIPPED);

  uint32_t now = micros();
  if ((int32_t)(now - next_control_us) >= 0) {
    next_control_us += CONTROL_US;
    if ((int32_t)(now - next_control_us) > (int32_t)CONTROL_US) next_control_us = now + CONTROL_US;  // fell behind
    left.measure(); right.measure();
    if (mode == SPEED) { left.control(CONTROL_US / 1e6f); right.control(CONTROL_US / 1e6f); }
  }

  if (telemetry_ms && millis() - last_tel_ms >= telemetry_ms) {
    last_tel_ms += telemetry_ms;
    if (millis() - last_tel_ms > telemetry_ms) last_tel_ms = millis();
    sendTelemetry();
  }
}
