#include "rover1_hardware/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace rover1_hardware
{

namespace
{
speed_t to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return 0;
  }
}
}  // namespace

SerialPort::~SerialPort() {close();}

bool SerialPort::open(const std::string & device, int baud)
{
  close();
  const speed_t speed = to_speed(baud);
  if (speed == 0) {
    error_ = "unsupported baud rate " + std::to_string(baud);
    return false;
  }
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    error_ = device + ": " + std::strerror(errno);
    return false;
  }
  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    error_ = device + ": tcgetattr: " + std::strerror(errno);
    close();
    return false;
  }
  cfmakeraw(&tio);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= ~(CSTOPB | CRTSCTS);
  // Don't drop DTR/RTS when the port closes. (Opening the port can still reset an ESP32 with the
  // usual auto-reset circuit; the driver waits for telemetry after opening to cope with that.)
  tio.c_cflag &= ~HUPCL;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);
  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    error_ = device + ": tcsetattr: " + std::strerror(errno);
    close();
    return false;
  }
  tcflush(fd_, TCIOFLUSH);
  error_.clear();
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::read_available(std::string & out)
{
  if (fd_ < 0) {
    error_ = "port not open";
    return false;
  }
  char buf[512];
  while (true) {
    const ssize_t n = ::read(fd_, buf, sizeof buf);
    if (n > 0) {
      out.append(buf, static_cast<size_t>(n));
      continue;
    }
    if (n == 0) {
      return true;  // nothing waiting (VMIN = 0)
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return true;
    }
    if (errno == EINTR) {
      continue;
    }
    error_ = std::string("read: ") + std::strerror(errno);
    return false;
  }
}

bool SerialPort::write_all(const std::string & data)
{
  if (fd_ < 0) {
    error_ = "port not open";
    return false;
  }
  size_t done = 0;
  while (done < data.size()) {
    const ssize_t n = ::write(fd_, data.data() + done, data.size() - done);
    if (n > 0) {
      done += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      pollfd p{fd_, POLLOUT, 0};
      if (::poll(&p, 1, 20) <= 0) {   // don't stall the control loop for long
        error_ = "write timed out";
        return false;
      }
      continue;
    }
    error_ = std::string("write: ") + std::strerror(errno);
    return false;
  }
  return true;
}

}  // namespace rover1_hardware
