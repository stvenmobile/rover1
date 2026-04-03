#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- OLED CONFIGURATION ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64 // Works for 32 or 64px screens
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- PIN MAPPING (Schematic Verified) ---
const int L_FI = 14; const int L_BI = 15; // Left Motor (M3)
const int R_FI = 17; const int R_BI = 12; // Right Motor (M2)

const int L_ENC_A = 34; const int L_ENC_B = 39; // Left Encoder
const int R_ENC_A = 35; const int R_ENC_B = 36; // Right Encoder

// --- GLOBALS ---
volatile long left_ticks = 0;
volatile long right_ticks = 0;
int last_l_pwm = 0;
int last_r_pwm = 0;

bool first_cmd_received = false; // Prevents immediate timeout on boot
unsigned long last_cmd_time = 0;
const unsigned long WATCHDOG_TIMEOUT = 2000; 

// --- ENCODER ISRs ---
void IRAM_ATTR isr_L() {
  if (digitalRead(L_ENC_A) == digitalRead(L_ENC_B)) left_ticks++;
  else left_ticks--;
}

void IRAM_ATTR isr_R() {
  if (digitalRead(R_ENC_A) != digitalRead(R_ENC_B)) right_ticks++;
  else right_ticks--;
}

// --- MOTOR CONTROL HELPER ---
void setMotor(int speed, int fwdPin, int revPin) {
  int pwm = constrain(abs(speed), 0, 255);
  if (speed > 0) {
    analogWrite(revPin, 0);
    analogWrite(fwdPin, pwm);
  } else if (speed < 0) {
    analogWrite(fwdPin, 0);
    analogWrite(revPin, pwm);
  } else {
    analogWrite(fwdPin, 0);
    analogWrite(revPin, 0);
  }
}

void setup() {
  // 1. SAFETY LOCKDOWN
  pinMode(L_FI, OUTPUT); digitalWrite(L_FI, LOW);
  pinMode(L_BI, OUTPUT); digitalWrite(L_BI, LOW);
  pinMode(R_FI, OUTPUT); digitalWrite(R_FI, LOW);
  pinMode(R_BI, OUTPUT); digitalWrite(R_BI, LOW);

  Serial.begin(115200);

  // 2. I2C & OLED 
  Wire.begin(21, 22);
  Wire.setClock(100000); 
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED 0x3C failed"));
  }
  
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 10);
  display.print("STANDBY");
  display.display();

  // 3. ENCODERS
  pinMode(L_ENC_A, INPUT); pinMode(L_ENC_B, INPUT);
  pinMode(R_ENC_A, INPUT); pinMode(R_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), isr_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), isr_R, CHANGE);

  last_cmd_time = millis();
}

void loop() {
  // --- A. SERIAL PARSER ---
  if (Serial.available() > 0) {
    char startChar = Serial.read();
    if (startChar == 'S') {
      int l_in = Serial.parseInt();
      if (Serial.read() == ',') {
        int r_in = Serial.parseInt();
        last_l_pwm = l_in;
        last_r_pwm = r_in;
        setMotor(last_l_pwm, L_FI, L_BI);
        setMotor(last_r_pwm, R_FI, R_BI);
        
        last_cmd_time = millis();
        first_cmd_received = true; 
      }
    }
  }

  // --- B. SAFETY WATCHDOG ---
  if (first_cmd_received && (millis() - last_cmd_time > WATCHDOG_TIMEOUT)) {
    setMotor(0, L_FI, L_BI);
    setMotor(0, R_FI, R_BI);
    last_l_pwm = 0; last_r_pwm = 0;
  }

  // --- C. SERIAL TELEMETRY (10Hz) ---
  static unsigned long last_tele = 0;
  if (millis() - last_tele > 100) {
    Serial.printf("E%ld,%ld\n", left_ticks, right_ticks);
    last_tele = millis();
  }

  // --- D. DASHBOARD REFRESH (5Hz) ---
  static unsigned long last_oled = 0;
  if (millis() - last_oled > 200) {
    updateOLED();
    last_oled = millis();
  }
}

void updateOLED() {
  display.clearDisplay();

  // Line 1: Header & Status
  display.setTextSize(2);
  display.setCursor(0, 0);

  if (millis() - last_cmd_time < WATCHDOG_TIMEOUT) display.print("CONNECTED");
  else display.print("TIMEOUT!");
  display.drawFastHLine(0, 10, 128, SSD1306_WHITE);

  // Line 2: PWM Speeds
  display.setTextSize(2);
  display.setCursor(0, 24);
  display.print("PWM-L:"); display.print(last_l_pwm);
  display.setCursor(0, 44);
  display.print("PWM-R:"); display.print(last_r_pwm);

  display.display();
}