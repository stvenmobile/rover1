/*
  Rover1_MotorTest - spin each wheel on command and check its encoder.

  Board: emakefun Maker-ESP32-Pro (ESP32-WROOM-32, RZ7889 H-bridges), Arduino ESP32 core 3.x.
  PlatformIO: rover1/firmware, environment motor_test:
      pio run -e motor_test -t upload --upload-port COM3
      pio device monitor --port COM3   (115200; each key is sent at once - type a command letter)
  Commands:

    l / L    left wheel forward / backward for RUN_MS
    r / R    right wheel forward / backward for RUN_MS
    b / B    both wheels forward / backward for RUN_MS
    + / -    more / less power (PWM duty, 10 % steps)
    c        watch the encoder counts (turn the wheels by hand); any key stops
    z        zero the encoder counts
    s        stop now
    ?        help and current settings

  SAFETY: put the rover on a box with both wheels off the ground before running a motor.
  Every run stops by itself after RUN_MS, and nothing runs at power-up.

  What to check (and write down):
    1. 'l': does the LEFT wheel turn FORWARD (the way the rover would drive)? Do its counts go UP?
    2. 'r': the same for the RIGHT wheel.
    3. 'c' + one full wheel turn by hand: how many counts? (expected about COUNTS_PER_REV)
    If a wheel turns backwards, flip its *_MOTOR_INVERT; if its counts go down when it turns
    forwards, flip its *_ENC_INVERT. The final firmware uses the same four settings.

  Encoders are counted by the ESP32's hardware pulse counter (PCNT) on both channels and both
  edges (4x), so no pulse is missed and the CPU does nothing for it.

  Version 1.0.11 - created 2026-11-24, last modified 2026-11-24
  History:
    1.0.11 2026-11-24  Invert flags and PWM settings now start from lib/rover_hw 1.1.0 (confirmed).
    1.0.10 2026-11-24  RIGHT_ENC_INVERT = true (right encoder counts up going backwards). PWM back to
                       20 kHz: in brake mode it drives as well as any frequency and is silent.
    1.0.9  2026-11-24  'w' sweeps the left wheel, then the right wheel, for side-by-side comparison.
    1.0.8  2026-11-24  Brake-mode PWM by default (drive/brake instead of drive/coast): the coast-mode
                       sweep was erratic below 70 %. 'm' toggles the mode. Sweep drops the "real Hz"
                       column (read-back gave 0). Silenced the harmless PCNT pull-up errors at boot.
    1.0.7  2026-11-24  'w' sweep: left motor at every PWM frequency x duty, 1 s each, prints a count
                       table plus the frequency the LEDC actually set.
    1.0.6  2026-11-24  LEFT_MOTOR_INVERT = true (FI high turned the left wheel backwards; its
                       encoder already counts down when going backwards, so no encoder flip).
    1.0.5  2026-11-24  PWM starts at 1 kHz (motor stayed still at 20 kHz but ran on a steady HIGH);
                       'f' steps the PWM frequency 1k/2k/5k/10k/20k without reflashing.
    1.0.4  2026-11-24  Version and build time printed at startup ('v' repeats it). 'h' holds the
                       left motor's FI pin at a plain digital HIGH (no PWM) for meter checks.
    1.0.3  2026-11-24  A stray Enter no longer stops a run early.
    1.0.2  2026-11-24  Part of the rover1/firmware PlatformIO project; pins from lib/rover_hw.
    1.0.1  2026-11-24  PlatformIO project (src/main.cpp) instead of an Arduino sketch.
    1.0.0  2026-11-24  First version: per-wheel runs, PCNT quadrature counts, hand-turn watch mode.
*/
#include <Arduino.h>
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include <rover_hw.h>

#define FW_NAME    "rover1 motor_test"
#define FW_VERSION "1.0.11"                // bump with every change, matches the header above

using namespace rover;                   // pins and drive-train constants (lib/rover_hw/rover_hw.h)

// ---- settings to adjust after the first test
// Directions and PWM start from lib/rover_hw (confirmed values); the test can change them at run time.
bool leftMotorInvert  = LEFT_MOTOR_INVERT;
bool rightMotorInvert = RIGHT_MOTOR_INVERT;
bool leftEncInvert    = LEFT_ENC_INVERT;
bool rightEncInvert   = RIGHT_ENC_INVERT;

uint32_t pwmFreqHz = PWM_FREQ_HZ;        // 'f' steps through PWM_STEPS
const uint32_t PWM_STEPS[] = {1000, 2000, 5000, 10000, 20000};
const uint8_t pwmBits = PWM_BITS;        // duty 0..1023
const uint32_t RUN_MS = 2000;

int power_pct = 40;                      // start gently
pcnt_unit_handle_t left_unit = nullptr, right_unit = nullptr;

// ---- motors -----------------------------------------------------------------------------------
void motorPinsSafe() {                   // before anything else: all motor inputs low (coast)
  for (int p : {L_FI, L_BI, R_FI, R_BI}) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
}

void setupPwm() {
  for (int p : {L_FI, L_BI, R_FI, R_BI}) {
    if (!ledcAttach(p, pwmFreqHz, pwmBits)) Serial.printf("PWM attach failed on GPIO %d\n", p);
    ledcWrite(p, 0);
  }
}

// power -100..100 %; 0 = coast (both inputs low).
// Brake mode (default): forward = FI held HIGH, BI pulsed with the inverse duty, so the motor
//   alternates drive / brake (both high). Speed follows power evenly and low-speed torque is good.
// Coast mode: forward = FI pulsed, BI low, so the motor alternates drive / coast. Erratic below ~70 %.
bool BRAKE_MODE = true;
const uint32_t PWM_FULL = 1u << pwmBits;  // ledcWrite(pin, PWM_FULL) = steady HIGH

void drive(int fi, int bi, bool invert, int pct) {
  if (invert) pct = -pct;
  pct = constrain(pct, -100, 100);
  uint32_t duty = (uint32_t)(abs(pct) * PWM_FULL / 100);
  if (pct == 0) { ledcWrite(fi, 0); ledcWrite(bi, 0); return; }
  int on = pct > 0 ? fi : bi, other = pct > 0 ? bi : fi;
  if (BRAKE_MODE) { ledcWrite(on, PWM_FULL); ledcWrite(other, PWM_FULL - duty); }
  else            { ledcWrite(other, 0);     ledcWrite(on, duty); }
}
void driveLeft(int pct)  { drive(L_FI, L_BI, leftMotorInvert, pct); }
void driveRight(int pct) { drive(R_FI, R_BI, rightMotorInvert, pct); }
void stopAll() { driveLeft(0); driveRight(0); }

// ---- encoders (PCNT, x4 quadrature, accumulating past the hardware limit) ----------------------
pcnt_unit_handle_t makeEncoder(int pin_a, int pin_b) {
  pcnt_unit_config_t uc = {};
  uc.low_limit = -30000;
  uc.high_limit = 30000;
  uc.flags.accum_count = 1;                       // keep counting past +-30000
  pcnt_unit_handle_t unit = nullptr;
  ESP_ERROR_CHECK(pcnt_new_unit(&uc, &unit));
  pcnt_glitch_filter_config_t fc = {};
  fc.max_glitch_ns = 1000;                        // ignore spikes shorter than 1 us
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &fc));

  pcnt_chan_config_t ca = {}; ca.edge_gpio_num = pin_a; ca.level_gpio_num = pin_b;
  pcnt_chan_config_t cb = {}; cb.edge_gpio_num = pin_b; cb.level_gpio_num = pin_a;
  pcnt_channel_handle_t ch_a = nullptr, ch_b = nullptr;
  ESP_ERROR_CHECK(pcnt_new_channel(unit, &ca, &ch_a));
  ESP_ERROR_CHECK(pcnt_new_channel(unit, &cb, &ch_b));
  // the standard quadrature decoding (ESP-IDF rotary encoder example)
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, uc.high_limit));   // needed for accum_count
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, uc.low_limit));
  ESP_ERROR_CHECK(pcnt_unit_enable(unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
  ESP_ERROR_CHECK(pcnt_unit_start(unit));
  return unit;
}

long countOf(pcnt_unit_handle_t u, bool invert) {
  int c = 0;
  pcnt_unit_get_count(u, &c);
  return invert ? -c : c;
}
long leftCount()  { return countOf(left_unit, leftEncInvert); }
long rightCount() { return countOf(right_unit, rightEncInvert); }
void zeroCounts() { pcnt_unit_clear_count(left_unit); pcnt_unit_clear_count(right_unit); }

// ---- tests ------------------------------------------------------------------------------------
void report(const char *what, long dl, long dr, uint32_t ms) {
  float s = ms / 1000.0f;
  auto line = [&](const char *side, long d) {
    float revs = d / COUNTS_PER_REV;
    Serial.printf("  %-5s %+7ld counts  %+6.2f wheel turns  %+6.1f rpm  %+5.2f m/s\n", side, d, revs,
                  revs / s * 60.0f, revs * PI * WHEEL_DIAMETER_M / s);
  };
  Serial.printf("%s at %d %% power for %.1f s:\n", what, power_pct, s);
  line("left", dl);
  line("right", dr);
}

void run(const char *what, int left_pct, int right_pct) {
  long l0 = leftCount(), r0 = rightCount();
  uint32_t t0 = millis();
  driveLeft(left_pct);
  driveRight(right_pct);
  while (millis() - t0 < RUN_MS) {
    if (Serial.available()) {                       // any key stops (but not a stray Enter)
      char k = Serial.read();
      if (k != '\n' && k != '\r') { Serial.println("(stopped early)"); break; }
    }
    delay(5);
  }
  uint32_t ms = millis() - t0;
  stopAll();
  delay(300);                                       // let it coast to a stop before reading
  report(what, leftCount() - l0, rightCount() - r0, ms);
}

void watchCounts() {
  Serial.println("Watching counts - turn the wheels by hand (forward should count UP). Any key stops.");
  while (Serial.available()) Serial.read();
  while (!Serial.available()) {
    Serial.printf("  left %+8ld   right %+8ld\n", leftCount(), rightCount());
    delay(250);
  }
  while (Serial.available()) Serial.read();
}

void banner() {                          // build time changes on every compile: proof of a fresh flash
  Serial.printf("\n=== %s v%s  (built %s %s) ===\n", FW_NAME, FW_VERSION, __DATE__, __TIME__);
}

// Diagnostic: left FI pin driven as a plain digital HIGH, no PWM, until a key is pressed.
// The motor should run flat out. Measure motor connector pins 1 and 6 (expect ~12 V).
void holdLeftHigh() {
  Serial.printf("HOLD: GPIO %d HIGH, GPIO %d LOW (left forward, full voltage, no PWM). Any key stops.\n",
                L_FI, L_BI);
  while (Serial.available()) Serial.read();
  ledcDetach(L_FI); ledcDetach(L_BI);
  pinMode(L_FI, OUTPUT); pinMode(L_BI, OUTPUT);
  digitalWrite(L_BI, LOW); digitalWrite(L_FI, HIGH);
  long c0 = leftCount();
  while (!Serial.available()) {
    Serial.printf("  left counts %+8ld\n", leftCount() - c0);
    delay(500);
  }
  while (Serial.available()) Serial.read();
  digitalWrite(L_FI, LOW);
  ledcAttach(L_FI, pwmFreqHz, pwmBits); ledcWrite(L_FI, 0);
  ledcAttach(L_BI, pwmFreqHz, pwmBits); ledcWrite(L_BI, 0);
  Serial.println("hold released");
}

void nextPwmFreq() {
  size_t n = sizeof(PWM_STEPS) / sizeof(PWM_STEPS[0]), i = 0;
  while (i < n && PWM_STEPS[i] != pwmFreqHz) i++;
  pwmFreqHz = PWM_STEPS[(i + 1) % n];
  stopAll();
  for (int p : {L_FI, L_BI, R_FI, R_BI}) {
    if (!ledcChangeFrequency(p, pwmFreqHz, pwmBits)) Serial.printf("frequency change failed on GPIO %d\n", p);
  }
  Serial.printf("PWM %lu Hz\n", (unsigned long)pwmFreqHz);
}

// Sweep one wheel at each frequency x duty for 1 s, counts per second in a table.
// Returns false if a key aborted it.
bool sweepWheel(const char *name, int fi, int bi, void (*driveFn)(int), long (*countFn)()) {
  const int duties[] = {30, 50, 70, 100};
  Serial.printf("\n%s wheel (%s PWM), counts/s, forward should be +\n", name,
                BRAKE_MODE ? "brake" : "coast");
  Serial.println("      Hz      30%      50%      70%     100%");
  for (uint32_t f : PWM_STEPS) {
    for (int p : {fi, bi}) ledcChangeFrequency(p, f, pwmBits);
    Serial.printf("  %6lu", (unsigned long)f);
    for (int d : duties) {
      long c0 = countFn();
      driveFn(d);
      delay(1000);
      long dc = countFn() - c0;
      driveFn(0);
      Serial.printf("  %+7ld", dc);
      delay(400);
      if (Serial.available()) { Serial.println("\n(aborted)"); return false; }
    }
    Serial.println();
  }
  return true;
}

// Sweep: left wheel, then right wheel, same table each, so the two motors can be compared.
void sweep() {
  uint32_t keep = pwmFreqHz;
  while (Serial.available()) Serial.read();
  Serial.println("SWEEP left then right, 1 s per cell (about 1 minute). Any key aborts.");
  if (sweepWheel("LEFT", L_FI, L_BI, driveLeft, leftCount))
    sweepWheel("RIGHT", R_FI, R_BI, driveRight, rightCount);
  while (Serial.available()) Serial.read();
  stopAll();
  for (int p : {L_FI, L_BI, R_FI, R_BI}) ledcChangeFrequency(p, keep, pwmBits);
  Serial.printf("back to PWM %lu Hz. Full speed is about 6600 counts/s.\n", (unsigned long)keep);
}

void help() {
  Serial.println("\nRover1 motor test - wheels OFF THE GROUND first!");
  Serial.println("  l/L left fwd/back   r/R right fwd/back   b/B both fwd/back");
  Serial.println("  +/- power   c watch counts   z zero counts   s stop   ? help");
  Serial.println("  h hold left FI pin HIGH (no PWM, meter check)   f next PWM frequency   v version");
  Serial.println("  w sweep both wheels over all frequencies and powers   m toggle brake/coast PWM");
  Serial.printf("  power %d %%, run %lu ms, PWM %lu Hz, counts/turn %.0f\n", power_pct, (unsigned long)RUN_MS,
                (unsigned long)pwmFreqHz, COUNTS_PER_REV);
  Serial.printf("  PWM mode: %s\n", BRAKE_MODE ? "brake" : "coast");
  Serial.printf("  invert: motor L %d R %d, encoder L %d R %d\n", leftMotorInvert, rightMotorInvert,
                leftEncInvert, rightEncInvert);
}

void setup() {
  motorPinsSafe();                                  // first: GPIO 12 low before anything else
  Serial.begin(115200);
  delay(500);
  banner();
  setupPwm();
  pinMode(L_ENC_A, INPUT); pinMode(L_ENC_B, INPUT);
  pinMode(R_ENC_A, INPUT); pinMode(R_ENC_B, INPUT);
  esp_log_level_set("gpio", ESP_LOG_NONE);          // PCNT tries pull-ups on input-only pins: harmless
  left_unit = makeEncoder(L_ENC_A, L_ENC_B);
  right_unit = makeEncoder(R_ENC_A, R_ENC_B);
  esp_log_level_set("gpio", ESP_LOG_ERROR);
  help();
}

void loop() {
  if (!Serial.available()) { delay(10); return; }
  char c = Serial.read();
  if (c == '\n' || c == '\r' || c == ' ') return;
  int p = power_pct;
  switch (c) {
    case 'l': run("LEFT forward", p, 0); break;
    case 'L': run("LEFT backward", -p, 0); break;
    case 'r': run("RIGHT forward", 0, p); break;
    case 'R': run("RIGHT backward", 0, -p); break;
    case 'b': run("BOTH forward", p, p); break;
    case 'B': run("BOTH backward", -p, -p); break;
    case '+': power_pct = min(100, power_pct + 10); Serial.printf("power %d %%\n", power_pct); break;
    case '-': power_pct = max(10, power_pct - 10); Serial.printf("power %d %%\n", power_pct); break;
    case 'c': watchCounts(); break;
    case 'z': zeroCounts(); Serial.println("counts zeroed"); break;
    case 's': stopAll(); Serial.println("stopped"); break;
    case 'h': holdLeftHigh(); break;
    case 'v': banner(); break;
    case 'f': nextPwmFreq(); break;
    case 'w': sweep(); break;
    case 'm': stopAll(); BRAKE_MODE = !BRAKE_MODE;
              Serial.printf("PWM mode: %s\n", BRAKE_MODE ? "brake" : "coast"); break;
    case '?': banner(); help(); break;
    default: Serial.printf("unknown command '%c' (? for help)\n", c);
  }
}
