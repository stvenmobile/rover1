// serial_port.hpp - minimal non-blocking POSIX serial port (8N1, raw), no extra dependencies
#pragma once

#include <string>

namespace rover1_hardware
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // Opens the device raw at the given baud rate. On failure returns false and sets error().
  bool open(const std::string & device, int baud);
  void close();
  bool is_open() const {return fd_ >= 0;}

  // Non-blocking read: appends whatever is waiting to out. Returns false on a port error
  // (for example the USB cable was pulled).
  bool read_available(std::string & out);
  // Writes all of data (waits briefly if the kernel buffer is full). Returns false on error.
  bool write_all(const std::string & data);

  const std::string & error() const {return error_;}

private:
  int fd_ = -1;
  std::string error_;
};

}  // namespace rover1_hardware
