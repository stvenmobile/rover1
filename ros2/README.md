# rover1 ROS 2 (Humble) - differential drive

Version 0.1.0, 2026-11-26. First draft: built and tested against a simulated ESP32 (`tools/fake_llc.py`),
**not yet run on the robot**.

Two packages:

| Package | What |
| :--- | :--- |
| `rover1_hardware` | ros2_control `SystemInterface` plugin (`rover1_hardware/Rover1System`) that talks to the llc firmware (`firmware/src/llc/main.cpp` v0.1.0) over USB serial |
| `rover1_bringup` | URDF (xacro), `diff_drive_controller` config, launch files, gamepad teleop config, udev rule |

`cmd_vel` (geometry_msgs/Twist) in; `/odom`, TF `odom -> base_footprint`, `/joint_states` out.

## How the driver uses the firmware

- **Port open.** Opening the CH340 port can reset the ESP32. The driver ignores anything without a valid
  `*HH` checksum (boot noise) and waits up to 5 s for a valid telemetry line before configuring.
  It then sends `S`, `W 300`, `T 20` and `I`.
- **Every control cycle (50 Hz).** `V <left> <right>` in counts/s, with checksum. Sent even when stopped,
  so the ESP32's 300 ms watchdog only trips if ROS stops or the cable is pulled.
  rad/s x 3960 / 2pi = counts/s. Wheel targets under 30 counts/s (about 2 mm/s) are sent as 0 (coast),
  because very low speeds are untested in the firmware and may just hum the motors (`deadband_cps`).
- **Telemetry (50 Hz).** Cumulative counts become wheel positions, the ESP32's measured counts/s become
  wheel velocities. `Z` is never sent: positions are counted from an offset taken at activation.
- **ESP32 reboot while running.** An unexpected `I,...` line means the board restarted. The driver
  resends `W`/`T` and re-bases the offsets, so odometry continues without a jump (tested with the fake).
- **No telemetry for 0.5 s.** Wheel velocities go to 0 and a warning is logged once a second.
  A port error (USB pulled) makes `read()` return an error.

## Values

| Value | Where | Source |
| :--- | :--- | :--- |
| Wheel radius 0.0438 m | URDF + controllers yaml | Steve measured 87.6 mm diameter (rover_hw.h still says 85 mm) |
| Wheel separation 0.221 m | URDF + controllers yaml | Steve measured, tyre centre to centre |
| Wheel width 0.0316 m | URDF | measured |
| 3960 counts/rev | URDF `<hardware>` params | rover_hw.h, confirmed by hand |
| Max 0.35 m/s, 2.5 rad/s | controllers yaml | measured 0.46 m/s unloaded; headroom left. **Guess** |
| Accelerations 1.0 m/s², 6 rad/s² | controllers yaml | **guess** (firmware limits wheels to about 1.3 m/s²) |
| Body box, caster position, masses | URDF | **guesses**, visual only for now |
| Odometry covariances | controllers yaml | **guesses**, tune when an EKF / Nav2 uses them |
| Gamepad buttons/axes | `config/teleop_joy.yaml` | **guess** for an Xbox-style pad (hold LB to drive) |

## Set up on the Jetson

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-teleop-twist-keyboard ros-humble-teleop-twist-joy ros-humble-joy

mkdir -p ~/rover1_ws/src && cd ~/rover1_ws/src
ln -s ~/rover1/ros2/src/rover1_hardware ~/rover1/ros2/src/rover1_bringup .   # wherever the repo lives
cd ~/rover1_ws && rosdep install --from-paths src -iy && colcon build --symlink-install
source install/setup.bash

# stable port name /dev/rover1_llc, and serial access
sudo cp src/rover1_bringup/udev/99-rover1.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
sudo usermod -aG dialout $USER   # then log out and in
```

## First run

1. **Without the robot**: `ros2 launch rover1_bringup rover1.launch.py use_mock_hardware:=true`
2. **With the fake ESP32** (exercises the real serial code):
   `python3 tools/fake_llc.py /tmp/rover1_fake` and in another terminal
   `ros2 launch rover1_bringup rover1.launch.py device:=/tmp/rover1_fake`. `tools/sim_test.sh` runs a scripted check.
3. **On the robot, wheels off the ground first.** Flash the llc firmware (`pio run -e llc -t upload`), then
   `ros2 launch rover1_bringup rover1.launch.py`
   and in another terminal `ros2 run teleop_twist_keyboard teleop_twist_keyboard`.
   Check: forward key turns both wheels forward; `ros2 topic echo /odom` x increases.
   Gamepad instead: add `teleop:=joy`.

## Calibration (on the floor)

- **Wheel radius:** drive 2 m straight (tape measure), compare with `/odom` x. Multiply `wheel_radius`
  by real / odom distance.
- **Wheel separation:** spin in place 5 full turns (e.g. `angular.z` 0.5 rad/s), compare odom yaw with
  reality. If odom says more than 5 turns, increase `wheel_separation` by that ratio, and vice versa.
- Update both `urdf/rover1.urdf.xacro` and `config/rover1_controllers.yaml`.

## Next

- Feed `/odom` + the BNO085 into `robot_localization` when the IMU comes back; then set `enable_odom_tf: false`.
- Add a `laser` link for the RPLidar A1 once deck 3 is printed and its height is known.
- The controller manager warns that passing robot_description as a parameter is deprecated; that is normal
  on recent Humble releases and still works.

## Tested here

- Built with colcon on ROS 2 Humble (Docker image), no warnings with `-Wall -Wextra -Wpedantic`.
- 7 unit tests of the protocol code (checksums, framing, telemetry parsing) pass.
- End to end against `fake_llc.py`: hardware activates, both controllers load, `cmd_vel` drives the wheels,
  `/odom` matches the commanded motion, and a simulated ESP32 reboot mid-run leaves odometry continuous.
- Mock hardware mode launches and drives.
