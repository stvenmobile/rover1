// llc_protocol.hpp - text protocol of the rover1 low-level controller (firmware/src/llc/main.cpp v0.1.0)
//
// Lines end in '\n'. The ESP32 always appends "*HH" (XOR of every character before the '*', in hex).
// This header has no ROS dependencies so it can be unit tested on its own.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

namespace rover1_hardware
{

// XOR of every byte, as in the firmware's sendLine() / handleLine().
inline uint8_t checksum(const std::string & body)
{
  uint8_t cs = 0;
  for (unsigned char c : body) {
    cs ^= c;
  }
  return cs;
}

// "V 1200 -1200" -> "V 1200 -1200*4B\n"
inline std::string frame(const std::string & body)
{
  char tail[8];
  std::snprintf(tail, sizeof tail, "*%02X\n", checksum(body));
  return body + tail;
}

// Wheel speed command in counts/s (forward +). The firmware parses floats, but whole counts/s are
// plenty (1 count/s = 0.07 mm/s) and keep the line short.
inline std::string speed_command(double left_cps, double right_cps)
{
  char b[48];
  std::snprintf(b, sizeof b, "V %ld %ld", std::lround(left_cps), std::lround(right_cps));
  return frame(b);
}

// Strips and checks a trailing "*HH". Returns the body, or nothing if the checksum is missing or wrong.
// (The firmware accepts lines without a checksum, but it always sends one, so we require it.)
inline std::optional<std::string> unframe(std::string line)
{
  while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
    line.pop_back();
  }
  const auto star = line.rfind('*');
  if (star == std::string::npos || line.size() != star + 3) {
    return std::nullopt;
  }
  char * end = nullptr;
  const std::string hex = line.substr(star + 1);
  const unsigned long cs = std::strtoul(hex.c_str(), &end, 16);
  if (end != hex.c_str() + 2) {
    return std::nullopt;
  }
  std::string body = line.substr(0, star);
  if (checksum(body) != cs) {
    return std::nullopt;
  }
  return body;
}

inline std::vector<std::string> split_commas(const std::string & s)
{
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == ',') {
      out.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  out.push_back(cur);
  return out;
}

// T,<ms>,<left count>,<right count>,<left c/s>,<right c/s>,<left %>,<right %>,<mode>
struct Telemetry
{
  uint32_t ms = 0;
  int64_t left_count = 0;
  int64_t right_count = 0;
  double left_cps = 0;
  double right_cps = 0;
  double left_pct = 0;
  double right_pct = 0;
  char mode = '?';  // I idle, V speed control, P open loop, W stopped by the watchdog
};

// Parses an unframed body. Returns nothing if it is not a well-formed T line.
inline std::optional<Telemetry> parse_telemetry(const std::string & body)
{
  const auto f = split_commas(body);
  if (f.size() != 9 || f[0] != "T" || f[8].size() != 1) {
    return std::nullopt;
  }
  Telemetry t;
  try {
    t.ms = static_cast<uint32_t>(std::stoul(f[1]));
    t.left_count = std::stoll(f[2]);
    t.right_count = std::stoll(f[3]);
    t.left_cps = std::stod(f[4]);
    t.right_cps = std::stod(f[5]);
    t.left_pct = std::stod(f[6]);
    t.right_pct = std::stod(f[7]);
  } catch (...) {
    return std::nullopt;
  }
  t.mode = f[8][0];
  return t;
}

}  // namespace rover1_hardware
