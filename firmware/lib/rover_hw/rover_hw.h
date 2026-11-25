/*
  rover_hw.h - rover1's wiring and drive-train constants, shared by every firmware program.

  Board: emakefun Maker-ESP32-Pro, schematic V1.6 (docs/maker-esp32-pro.pdf). Motor ports use RZ7889
  H-bridges: two inputs per motor, PWM on one with the other low; both low = coast, both high = brake.
  DIP switch SW1 must connect GPIO 12/14/15/17 to the motor drivers.

  Motor ports (6-pin connectors: motor -, encoder VCC/B/A/GND, motor +), from schematic sheet 4:
      port  driver  FI   BI   enc A  enc B
      E3    U7      15   14   34     39      <- rover1 LEFT
      E2    U8      17   12   35     36      <- rover1 RIGHT
      E0    U4      27   13   18     19      (unused; 13/27 free)
      E1    U11      4    2    5     23      (unused; GPIO 2 and 5 are boot strapping pins)

  Version 1.1.1 - created 2026-11-24, last modified 2026-11-24
  History:
    1.1.1  2026-11-24  COUNTS_PER_REV confirmed: 10 hand turns = 39586 counts (3958.6 per turn).
    1.1.0  2026-11-24  Directions and PWM confirmed by motor_test 1.0.10: invert flags, brake-mode PWM
                       at 20 kHz, measured top speed. (Coast-mode PWM was erratic below ~70 %.)
    1.0.1  2026-11-24  Left FI/BI corrected from the schematic (FI = 15, BI = 14); port table.
    1.0.0  2026-11-24  Pins and constants from the old Rover1_LLC sketch and the schematic.
*/
#pragma once

namespace rover {

// motors (FI = forward input, BI = backward input)
constexpr int L_FI = 15, L_BI = 14;      // port E3
constexpr int R_FI = 17, R_BI = 12;      // port E2. GPIO 12 is a boot strapping pin: drive it LOW first thing

// encoders: GPIO 34-39 are input-only, with no internal pull-ups
constexpr int L_ENC_A = 34, L_ENC_B = 39;
constexpr int R_ENC_A = 35, R_ENC_B = 36;

// I2C (TCA9548A multiplexer: CH0 BNO085 IMU, CH1-5 VL53L1X distance sensors)
constexpr int I2C_SDA = 21, I2C_SCL = 22;
constexpr int MUX_ADDR = 0x70;

// directions (motor_test 1.0.10): forward = caster at the back; forward counts are positive
constexpr bool LEFT_MOTOR_INVERT  = true;   // FI high turns the left wheel backwards
constexpr bool RIGHT_MOTOR_INVERT = false;
constexpr bool LEFT_ENC_INVERT    = false;
constexpr bool RIGHT_ENC_INVERT   = true;   // right encoder counts up going backwards

// PWM: brake mode (one input held HIGH, the other pulsed = drive/brake), 20 kHz, 10-bit.
// Measured speed is smooth and even from 30 % to 100 %; the two motors match within ~3 %.
constexpr unsigned PWM_FREQ_HZ = 20000;
constexpr unsigned PWM_BITS = 10;

// drive train
constexpr float COUNTS_PER_REV = 3960.0f;  // 11 x 90:1 x 4 edges; confirmed 10 turns = 39586
constexpr float WHEEL_DIAMETER_M = 0.085f;
constexpr float MAX_COUNTS_PER_S = 6800.0f; // measured at 100 %, 12 V, wheels off the ground (~103 rpm)

}  // namespace rover
