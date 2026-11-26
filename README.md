# rover1

A two-wheel differential-drive robot with a Jetson Orin Nano and an ESP32 motor controller.

**Current goals:**
1. Drive it from ROS 2 as a standard differential-drive robot (`cmd_vel` in, odometry out).
2. Build towards precise navigation: an RPLidar A1 for mapping and localisation, then
   navigation to known places.
3. Publish camera images to a web page.

Each item is described below as **working**, **in progress** or **planned**. Only working items
have been tested.

---

## Hardware

| Part | Details | Status |
| :--- | :--- | :--- |
| Main computer | NVIDIA Jetson Orin Nano 8 GB, JetPack 6.2.1, Ubuntu 22.04 (ROS 2 Humble) | on board |
| Motor controller | emakefun Maker-ESP32-Pro (ESP32-D0WD-V3, 4 MB flash, CH340 USB serial), schematic in `docs/` | working |
| Motor drivers | RZ7889 H-bridges on the board (3 A continuous, 18 V max) | working |
| Motors | 2x JGB37-520, 12 V, 110 rpm, 90:1 gearbox, Hall encoders (11 pulses/rev) | working |
| Wheels | 2x rubber drive wheels, measured 87.6 mm diameter x 31.6 mm wide (sold as 85 mm); 221 mm apart, tyre centre to centre; rear caster | working |
| Lidar | Slamtec RPLidar A1 | planned (deck 3, already in the chassis file) |
| Camera | Raspberry Pi Camera v2.1 (IMX219) | on board, not yet used by the new software |
| Power | 12 V for the motors; 19 V buck-boost regulator for the Orin | bench supply used for testing |

### Chassis

The chassis is 3D printed. It comes from **Weddell 2: Differential drive robot powered by ROS**
by Thingiverse user *pokpong* ([thing:3787601](https://www.thingiverse.com/thing:3787601)),
in the remix **Modified Weddell 2 robot for ROS/RPi4** by *jbowling* (Jason Bowling,
[thing:4697812](https://www.thingiverse.com/thing:4697812)). The remix changes the motor plate
for low-cost gear motors with encoders; Jason Bowling's
[build write-up](https://medium.com/swlh/building-an-inexpensive-3-d-printed-ros-robot-625ac7766f4a)
has assembly notes.

The print file is `hardware/cad/rover1.3mf` (Bambu Studio, 2024-07-29). It holds the parts as meshes,
which can also serve as visual meshes in the URDF.

Each deck is two half-plates (98 x 196 mm each), joined where their straight-edge tabs overlap,
giving a deck about 196 mm square. The original design spaces the decks with 50 mm PCB standoffs.
The front and back "masks" (148 x 54.5 mm) fill the gap between two decks and carry sensors.

| Layer | Parts | Carries | Status |
| :--- | :--- | :--- | :--- |
| 1 (bottom) | `motorPlate-dual` x2 | Motors and batteries underneath; buck converter and ESP32 on top | built |
| 1 to 2 | `mask_pi_camera_modified` (front), `mask_template_modified` (back) | Camera at the front | built |
| 2 | `base_RaspberryPi_B` x2 | Jetson Orin Nano | built |
| 3 (top) | `base_RplidarA1_A` + `base_RplidarA1_B` | RPLidar A1 (`lidar_serial_housing` holds its USB adapter) | planned |

Before printing layer 3: measure the height of the Orin Nano's heatsink and fan above deck 2 (the
original 50 mm spacing was set for a Jetson Nano), and check the lidar holes against the A1.

### Motor wiring (confirmed)

| Side | Board port | Driver | FI / BI pins | Encoder A / B |
| :--- | :--- | :--- | :--- | :--- |
| Left | E3 | U7 | GPIO 15 / 14 | GPIO 34 / 39 |
| Right | E2 | U8 | GPIO 17 / 12 | GPIO 35 / 36 |

- DIP switch SW1 must be ON: it connects GPIO 12/14/15/17 to the motor drivers.
- GPIO 12 is a boot strapping pin, so the firmware drives it LOW first thing at startup.
- GPIO 34–39 are input-only with no internal pull-ups; the encoders provide their own.
- All pins and drive-train constants live in one place: `firmware/lib/rover_hw/rover_hw.h`.

### Measured motor behaviour (motor_test, 2026-11-24, wheels off the ground, 12 V)

- **Directions:** forward means the caster is at the back. The left motor is wired reversed
  (inverted in software). The right encoder counts reversed (also inverted in software). Forward
  counts are positive on both sides.
- **PWM:** brake mode at 20 kHz, 10-bit. One driver input is held HIGH and the other is pulsed,
  so the motor alternates between drive and brake. Speed rises evenly from 30 % to 100 %.
  Coast mode (pulsing with the other input LOW) was erratic below about 70 %.
- **Encoders:** 3,960 counts per wheel turn (11 x 90 x 4 edges, using the ESP32's hardware pulse
  counter). Confirmed by hand: 10 turns = 39,586 counts. That is about 0.067 mm of travel per count.
- **Speed:** about 6,800 counts/s at 100 % (about 103 rpm, 0.46 m/s). The two motors match
  within about 3 %.

---

## Software

### ESP32 firmware (`firmware/`, PlatformIO)

One PlatformIO project, with one environment per program:

| Environment | Source | Purpose | Status |
| :--- | :--- | :--- | :--- |
| `motor_test` | `src/motor_test/` | Bench test: run each wheel, sweep PWM settings, read encoders | working (v1.0.11) |
| `llc` | `src/llc/` | Low-level controller used with ROS (see below) | in progress (v0.1.0, not yet tested) |

Shared code is in `lib/rover_hw/` (pins, directions, PWM settings, drive-train constants).
Platform: pioarduino 55.03.39 (Arduino-ESP32 3.3.9 / ESP-IDF 5.5.4).

Build and upload from `firmware/` (or use the PlatformIO buttons in VS Code):

```
pio run -e motor_test -t upload
pio device monitor
```

The motor test prints its version and build time at startup. Commands: `l`/`r`/`b` run the left,
right or both wheels forward (capitals for backward), `+`/`-` change power, `w` sweeps both
wheels over all PWM frequencies and powers, `c` shows live encoder counts, and `?` lists the rest.

A full backup of the board's original firmware is in `rover1_esp32_backup.bin`. To restore it:
`esptool write-flash 0 rover1_esp32_backup.bin`.

### Low-level controller firmware (in progress)

`src/llc/main.cpp` v0.1.0, written 2026-11-25, not yet tested on the robot. The full command
list is in the file's header.

- Per-wheel speed control at 100 Hz: PI plus feedforward taken from the motor_test sweep.
- Watchdog: the motors stop if no `V` or `P` command arrives for 300 ms (adjustable with `W`).
- Telemetry at 50 Hz: `T,<ms>,<left count>,<right count>,<left c/s>,<right c/s>,<left %>,<right %>,<mode>*HH`.
- Text commands with an optional NMEA-style checksum (`*HH`), so they can also be typed by hand:
  `V <left> <right>` (counts/s), `P` (open-loop %), `S` stop, `Z` zero counts, `K` gains,
  `W` watchdog, `T` telemetry rate, `I` info.

### ROS 2 on the Jetson (`ros2/`, in progress)

Two colcon packages for ROS 2 Humble; setup, first run and calibration steps are in `ros2/README.md`.

| Package | Purpose | Status |
| :--- | :--- | :--- |
| `rover1_hardware` | ros2_control hardware interface for the llc firmware over USB serial | v0.1.0, tested against a simulated ESP32 only |
| `rover1_bringup` | URDF, `diff_drive_controller` config, launch files, gamepad teleop, udev rule | v0.1.0, as above |

`ros2/tools/fake_llc.py` pretends to be the ESP32, so the ROS side can be tested without the robot.

Plan, in order:
1. **Differential drive.** `cmd_vel` drives the robot; wheel odometry is published. (in progress)
2. **Lidar.** Print the lidar deck (layer 3); `rplidar_ros` publishing `/scan`.
3. **Mapping.** `slam_toolbox` to build a map of the house.
4. **Navigation.** Nav2 localising on the saved map and driving to named places.

ROS networking will use a Fast DDS discovery server, so rover1 can later talk to other robots
(such as Piper) without relying on multicast discovery.

### Camera web page (planned)

Publish camera images to a web page served from the Jetson. `src/rover_stream.py` (on the
`archive/pre-ros` branch) streams camera frames with Flask and is a possible starting point.

---

## Earlier design (archived)

The pre-ROS code is no longer on `main`. It is kept, with its full history, on the branch
`archive/pre-ros` and the tag `pre-ros-archive`:

- `firmware/Rover1_LLC/`, `firmware/Calibrate_Encoder/`: Arduino sketches for the old controller
  (encoders, VL53L1X distance sensors through the TCA9548A multiplexer, BNO085 IMU).
- `src/`, `start_rover.sh`: Python serial broker, Flask camera stream, joystick teleop and voice
  commands. Their constants (1980 counts/rev, 65 mm wheels) are out of date.

The IMU may come back later to improve odometry for navigation.

## Change history

| Version | Date | Change |
| :--- | :--- | :--- |
| 2.3.0 | 2026-11-26 | Repo restarted: pre-ROS code moved to the `archive/pre-ros` branch. ROS 2 packages added in `ros2/`. Measured wheel size and separation. |
| 2.2.0 | 2026-11-25 | Low-level controller firmware v0.1.0 described (in progress). |
| 2.1.0 | 2026-11-24 | Chassis section: credit to Weddell 2 (pokpong) and the jbowling remix, layer layout from `hardware/cad/rover1.3mf`, lidar deck noted as planned. |
| 2.0.0 | 2026-11-24 | Rewritten for the current goals (ROS differential drive, lidar navigation, camera web page). Speech and voice commands removed. Confirmed wiring, motor measurements and the new PlatformIO firmware added; the earlier design is marked for archiving. |
| 1.x | 2026-04 | Original README (Vosk/Piper voice, Flask stream, safety loop). |
