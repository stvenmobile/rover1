// rover1_system.cpp - ros2_control hardware interface for rover1's ESP32 low-level controller
//
// Talks to firmware/src/llc/main.cpp (v0.1.0) over USB serial:
//   write(): "V <left c/s> <right c/s>*HH" every control cycle. This also feeds the ESP32's watchdog
//            (300 ms by default), so if this node stops or hangs the motors stop.
//   read():  parses the 50 Hz "T,..." telemetry. Cumulative counts -> wheel position (rad),
//            measured counts/s -> wheel velocity (rad/s).
#include "rover1_hardware/rover1_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rover1_hardware/llc_protocol.hpp"

namespace rover1_hardware
{

namespace
{
std::string param_or(
  const hardware_interface::HardwareInfo & info, const std::string & name, const std::string & fallback)
{
  const auto it = info.hardware_parameters.find(name);
  return it == info.hardware_parameters.end() ? fallback : it->second;
}
}  // namespace

CallbackReturn Rover1System::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    device_ = param_or(info_, "device", "/dev/rover1_llc");
    baud_ = std::stoi(param_or(info_, "baud", "115200"));
    counts_per_rev_ = std::stod(param_or(info_, "counts_per_rev", "3960"));
    watchdog_ms_ = std::stoi(param_or(info_, "watchdog_ms", "300"));
    telemetry_period_ms_ = std::stoi(param_or(info_, "telemetry_period_ms", "20"));
    startup_timeout_s_ = std::stod(param_or(info_, "startup_timeout_s", "5.0"));
    telemetry_timeout_s_ = std::stod(param_or(info_, "telemetry_timeout_s", "0.5"));
    deadband_cps_ = std::stod(param_or(info_, "deadband_cps", "30"));
    left_.joint = param_or(info_, "left_joint", "left_wheel_joint");
    right_.joint = param_or(info_, "right_joint", "right_wheel_joint");
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger_, "Bad hardware parameter: %s", e.what());
    return CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 2) {
    RCLCPP_FATAL(logger_, "Expected 2 joints, got %zu", info_.joints.size());
    return CallbackReturn::ERROR;
  }
  bool have_left = false, have_right = false;
  for (const auto & joint : info_.joints) {
    have_left |= joint.name == left_.joint;
    have_right |= joint.name == right_.joint;
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(logger_, "Joint '%s' needs exactly one 'velocity' command interface", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2 ||
      joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
      joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(logger_, "Joint '%s' needs 'position' then 'velocity' state interfaces", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }
  if (!have_left || !have_right) {
    RCLCPP_FATAL(
      logger_, "Joints must be named '%s' and '%s' (hardware parameters left_joint / right_joint)",
      left_.joint.c_str(), right_.joint.c_str());
    return CallbackReturn::ERROR;
  }
  if (counts_per_rev_ <= 0) {
    RCLCPP_FATAL(logger_, "counts_per_rev must be positive");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Rover1System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> s;
  for (Wheel * w : {&left_, &right_}) {
    s.emplace_back(w->joint, hardware_interface::HW_IF_POSITION, &w->position);
    s.emplace_back(w->joint, hardware_interface::HW_IF_VELOCITY, &w->velocity);
  }
  return s;
}

std::vector<hardware_interface::CommandInterface> Rover1System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> c;
  for (Wheel * w : {&left_, &right_}) {
    c.emplace_back(w->joint, hardware_interface::HW_IF_VELOCITY, &w->command);
  }
  return c;
}

CallbackReturn Rover1System::on_configure(const rclcpp_lifecycle::State &)
{
  if (!port_.open(device_, baud_)) {
    RCLCPP_FATAL(logger_, "Cannot open %s: %s", device_.c_str(), port_.error().c_str());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "Opened %s at %d baud, waiting for the ESP32...", device_.c_str(), baud_);
  rx_buffer_.clear();
  got_telemetry_ = false;

  // Opening the port can reset the ESP32 (CH340 auto-reset), so give it time to boot and start
  // sending telemetry before we configure it.
  if (!wait_for_telemetry(startup_timeout_s_)) {
    RCLCPP_FATAL(
      logger_, "No telemetry from the ESP32 on %s within %.1f s. Is the llc firmware flashed?",
      device_.c_str(), startup_timeout_s_);
    port_.close();
    return CallbackReturn::ERROR;
  }
  send(frame("S"));
  send_config();
  send(frame("I"));
  return CallbackReturn::SUCCESS;
}

void Rover1System::send_config()
{
  send(frame("W " + std::to_string(watchdog_ms_)));
  send(frame("T " + std::to_string(telemetry_period_ms_)));
  config_pending_ = false;
}

CallbackReturn Rover1System::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (port_.is_open()) {
    send(frame("S"));
  }
  port_.close();
  return CallbackReturn::SUCCESS;
}

CallbackReturn Rover1System::on_activate(const rclcpp_lifecycle::State &)
{
  left_.command = right_.command = 0.0;
  left_.position = right_.position = 0.0;
  left_.velocity = right_.velocity = 0.0;
  zero_pending_ = true;  // the next telemetry counts become position 0
  if (!wait_for_telemetry(1.0)) {
    RCLCPP_ERROR(logger_, "No telemetry from the ESP32 while activating");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "rover1 hardware active");
  return CallbackReturn::SUCCESS;
}

CallbackReturn Rover1System::on_deactivate(const rclcpp_lifecycle::State &)
{
  left_.command = right_.command = 0.0;
  if (port_.is_open()) {
    send(frame("S"));
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn Rover1System::on_shutdown(const rclcpp_lifecycle::State &)
{
  if (port_.is_open()) {
    send(frame("S"));
  }
  port_.close();
  return CallbackReturn::SUCCESS;
}

bool Rover1System::send(const std::string & framed)
{
  if (!port_.write_all(framed)) {
    RCLCPP_ERROR_THROTTLE(logger_, steady_clock_, 1000, "Serial write failed: %s", port_.error().c_str());
    return false;
  }
  return true;
}

bool Rover1System::wait_for_telemetry(double timeout_s)
{
  got_telemetry_ = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline) {
    if (!poll_serial()) {
      return false;
    }
    if (got_telemetry_) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

bool Rover1System::poll_serial()
{
  if (!port_.read_available(rx_buffer_)) {
    return false;
  }
  size_t start = 0;
  for (size_t nl = rx_buffer_.find('\n'); nl != std::string::npos; nl = rx_buffer_.find('\n', start)) {
    handle_line(rx_buffer_.substr(start, nl - start));
    start = nl + 1;
  }
  rx_buffer_.erase(0, start);
  if (rx_buffer_.size() > 4096) {  // no newline in a long time: garbage (boot messages at 74880 baud?)
    rx_buffer_.clear();
  }
  return true;
}

void Rover1System::handle_line(const std::string & line)
{
  const auto body = unframe(line);
  if (!body) {
    // Expected while the ESP32 boots (ROM messages); only worth a debug message.
    ++bad_lines_;
    RCLCPP_DEBUG(logger_, "Ignoring line without a valid checksum: '%s'", line.c_str());
    return;
  }
  if (body->empty()) {
    return;
  }
  switch ((*body)[0]) {
    case 'T': {
        const auto t = parse_telemetry(*body);
        if (!t) {
          ++bad_lines_;
          return;
        }
        // Never send Z: positions are differences from an offset, so odometry never jumps.
        if (zero_pending_) {
          left_.offset = t->left_count;
          right_.offset = t->right_count;
          left_.relative = right_.relative = 0;
          zero_pending_ = false;
        } else if (rebase_pending_) {
          // the ESP32 restarted counting from 0: carry on from the last position
          left_.offset = t->left_count - left_.relative;
          right_.offset = t->right_count - right_.relative;
        }
        rebase_pending_ = false;
        left_.relative = t->left_count - left_.offset;
        right_.relative = t->right_count - right_.offset;
        const double rad_per_count = 2.0 * M_PI / counts_per_rev_;
        left_.position = static_cast<double>(left_.relative) * rad_per_count;
        right_.position = static_cast<double>(right_.relative) * rad_per_count;
        left_.velocity = t->left_cps * rad_per_count;
        right_.velocity = t->right_cps * rad_per_count;
        llc_mode_ = t->mode;
        got_telemetry_ = true;
        last_telemetry_time_ = steady_clock_.now();
        break;
      }
    case 'I':
      // Sent at boot (and in reply to our "I"). After a boot the ESP32 has default settings and
      // counts from 0 again, so resend the settings and keep positions continuous.
      RCLCPP_INFO(logger_, "ESP32: %s", body->c_str());
      rebase_pending_ = !zero_pending_;
      config_pending_ = true;
      break;
    case 'A':
      RCLCPP_DEBUG(logger_, "ESP32: %s", body->c_str());
      break;
    case 'E':
      RCLCPP_WARN(logger_, "ESP32 error: %s", body->c_str());
      break;
    default:
      RCLCPP_DEBUG(logger_, "ESP32 (unknown line): %s", body->c_str());
  }
}

hardware_interface::return_type Rover1System::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!poll_serial()) {
    RCLCPP_ERROR_THROTTLE(logger_, steady_clock_, 1000, "Serial read failed: %s", port_.error().c_str());
    return hardware_interface::return_type::ERROR;
  }
  const double age = (steady_clock_.now() - last_telemetry_time_).seconds();
  if (age > telemetry_timeout_s_) {
    // Odometry freezes while this lasts; the ESP32's watchdog stops the motors if our commands
    // aren't getting through either.
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 1000, "No telemetry from the ESP32 for %.2f s", age);
    left_.velocity = right_.velocity = 0.0;
  }
  if (llc_mode_ == 'W') {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 2000,
      "ESP32 watchdog tripped (no command for %d ms). Next command resumes driving.", watchdog_ms_);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Rover1System::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  const double counts_per_rad = counts_per_rev_ / (2.0 * M_PI);
  auto to_cps = [&](double rad_s) {
      const double cps = std::isfinite(rad_s) ? rad_s * counts_per_rad : 0.0;
      // Tiny targets just hum the motors (the firmware's feedforward is only checked down to about
      // 1900 c/s), so send 0 = coast instead.
      return std::fabs(cps) < deadband_cps_ ? 0.0 : cps;
    };
  if (config_pending_) {
    send_config();
  }
  // Sent every cycle, even when stopped: it keeps the ESP32 watchdog fed. The firmware clamps to
  // +-7000 c/s and limits acceleration itself.
  // A failed write is logged (throttled) but not fatal: a lost command just lets the watchdog stop
  // the motors, and a pulled cable is reported by read().
  send(speed_command(to_cps(left_.command), to_cps(right_.command)));
  return hardware_interface::return_type::OK;
}

}  // namespace rover1_hardware

PLUGINLIB_EXPORT_CLASS(rover1_hardware::Rover1System, hardware_interface::SystemInterface)
