#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <SparkFun_VL53L1X.h>

#define MUX_ADDR 0x70
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;
SFEVL53L1X distanceSensors[6]; 
bool sensorOnline[6] = {false, false, false, false, false, false};

// Pin Mappings
const int L_FI = 14; const int L_BI = 15;
const int R_FI = 17; const int R_BI = 12;
const int L_ENC_A = 34; const int L_ENC_B = 39;
const int R_ENC_A = 35; const int R_ENC_B = 36;

volatile long left_ticks = 0;
volatile long right_ticks = 0;

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
  delay(1); 
}

void IRAM_ATTR isr_L() {
  if (digitalRead(L_ENC_A) == digitalRead(L_ENC_B)) left_ticks++;
  else left_ticks--;
}
void IRAM_ATTR isr_R() {
  if (digitalRead(R_ENC_A) != digitalRead(R_ENC_B)) right_ticks++;
  else right_ticks--;
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to connect
  
  Wire.begin(21, 22);
  Wire.setClock(100000); // 100kHz for maximum reliability during testing

  Serial.println("\n--- BOOTING ROVER1 ---");

  // 1. Initialize IMU (CH0)
  tcaselect(0);
  if (!bno08x.begin_I2C(0x4B)) {
    Serial.println("CH0: IMU FAIL");
  } else {
    bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 50000);
    sensorOnline[0] = true;
    Serial.println("CH0: IMU OK");
  }

  // 2. Initialize TOFs (CH1-5)
  for (uint8_t i = 1; i <= 5; i++) {
    tcaselect(i);
    // Check if sensor is even on the bus first
    Wire.beginTransmission(0x29);
    if (Wire.endTransmission() == 0) {
      if (distanceSensors[i].begin() == 0) {
        distanceSensors[i].setDistanceModeShort();
        distanceSensors[i].setTimingBudgetInMs(50);
        distanceSensors[i].startRanging();
        sensorOnline[i] = true;
        Serial.printf("CH%d: TOF OK  ", i);
      } else {
        Serial.printf("CH%d: TOF INIT_ERR  ", i);
      }
    } else {
      Serial.printf("CH%d: TOF NOT_FOUND  ", i);
    }
  }
  Serial.println("\n----------------------");

  pinMode(L_ENC_A, INPUT); pinMode(L_ENC_B, INPUT);
  pinMode(R_ENC_A, INPUT); pinMode(R_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), isr_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), isr_R, CHANGE);
}

void loop() {
  static unsigned long last_tele = 0;
  if (millis() - last_tele > 200) { // 5Hz update rate for clearer debugging
    last_tele = millis();

    // 1. IMU Read
    float r=0, i=0, j=0, k=0;
    if (sensorOnline[0]) {
      tcaselect(0);
      if (bno08x.getSensorEvent(&sensorValue)) {
        r = sensorValue.un.arvrStabilizedRV.real;
        i = sensorValue.un.arvrStabilizedRV.i;
        j = sensorValue.un.arvrStabilizedRV.j;
        k = sensorValue.un.arvrStabilizedRV.k;
      }
    }

    // 2. TOF Read
    int dists[6] = {0, 0, 0, 0, 0, 0};
    for (int ch = 1; ch <= 5; ch++) {
      if (sensorOnline[ch]) {
        tcaselect(ch);
        // INCREASED TIMEOUT to 70ms to allow for the 50ms budget
        unsigned long startWait = millis();
        while(!distanceSensors[ch].checkForDataReady() && millis() - startWait < 70);

        if (distanceSensors[ch].checkForDataReady()) {
          dists[ch] = distanceSensors[ch].getDistance();
          distanceSensors[ch].clearInterrupt();
        } else {
          dists[ch] = -999; // Explicit Timeout Marker
        }
      } else {
        dists[ch] = -404; // Explicit "Not Online" Marker
      }
    }

    // 3. Serial Packet
    Serial.printf("T%ld,%ld|%.2f,%.2f,%.2f,%.2f|%d,%d,%d,%d,%d\n", 
      left_ticks, right_ticks, r, i, j, k,
      dists[1], dists[2], dists[3], dists[4], dists[5]);
  }
}