// rover1_system.hpp - ros2_control hardware interface for rover1's ESP32 low-level controller
#pragma once

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rover1_hardware/serial_port.hpp"

namespace rover1_hardware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Rover1System : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Wheel
  {
    std::string joint;
    double command = 0.0;   // rad/s, from diff_drive_controller
    double position = 0.0;  // rad, since activation
    double velocity = 0.0;  // rad/s, measured by the ESP32 over 40 ms
    int64_t offset = 0;     // encoder count that is position 0
    int64_t relative = 0;   // last count - offset
  };

  // Reads whatever bytes are waiting and handles every complete line. Returns false on a port error.
  bool poll_serial();
  void handle_line(const std::string & line);
  bool send(const std::string & framed);
  void send_config();
  // Waits up to timeout_s for a telemetry line (the board may reset when the port opens).
  bool wait_for_telemetry(double timeout_s);

  // parameters (from the <hardware> block in the URDF)
  std::string device_;
  int baud_ = 115200;
  double counts_per_rev_ = 3960.0;
  int watchdog_ms_ = 300;
  int telemetry_period_ms_ = 20;
  double startup_timeout_s_ = 5.0;
  double telemetry_timeout_s_ = 0.5;
  double deadband_cps_ = 30.0;

  SerialPort port_;
  std::string rx_buffer_;
  Wheel left_, right_;
  bool zero_pending_ = true;    // take the next telemetry counts as zero
  bool rebase_pending_ = false; // ESP32 rebooted: keep positions continuous across its count reset
  bool config_pending_ = false; // (re)send watchdog and telemetry settings
  bool got_telemetry_ = false;
  char llc_mode_ = '?';
  rclcpp::Time last_telemetry_time_{0, 0, RCL_STEADY_TIME};
  uint64_t bad_lines_ = 0;

  rclcpp::Logger logger_ = rclcpp::get_logger("Rover1System");
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};

}  // namespace rover1_hardware
