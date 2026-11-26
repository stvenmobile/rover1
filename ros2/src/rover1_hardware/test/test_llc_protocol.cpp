#include <gtest/gtest.h>

#include "rover1_hardware/llc_protocol.hpp"

using namespace rover1_hardware;

TEST(LlcProtocol, FrameAndUnframeRoundTrip)
{
  const std::string framed = frame("V 1200 -1200");
  EXPECT_EQ(framed.back(), '\n');
  const auto body = unframe(framed);
  ASSERT_TRUE(body.has_value());
  EXPECT_EQ(*body, "V 1200 -1200");
}

TEST(LlcProtocol, ChecksumMatchesFirmware)
{
  // "S" = 0x53, so the firmware expects "S*53"
  EXPECT_EQ(frame("S"), "S*53\n");
}

TEST(LlcProtocol, SpeedCommandRounds)
{
  EXPECT_EQ(unframe(speed_command(630.4, -630.6)).value(), "V 630 -631");
}

TEST(LlcProtocol, RejectsBadChecksum)
{
  EXPECT_FALSE(unframe("T,1,2,3,4,5,6,7,V*00\n").has_value());
  EXPECT_FALSE(unframe("T,1,2,3,4,5,6,7,V\n").has_value());  // ESP32 always sends one
  EXPECT_FALSE(unframe("garbage*Z\n").has_value());
}

TEST(LlcProtocol, ParsesTelemetry)
{
  const std::string line = frame("T,123456,-39586,40000,-6800,6790,-100,99,V");
  const auto body = unframe(line);
  ASSERT_TRUE(body.has_value());
  const auto t = parse_telemetry(*body);
  ASSERT_TRUE(t.has_value());
  EXPECT_EQ(t->ms, 123456u);
  EXPECT_EQ(t->left_count, -39586);
  EXPECT_EQ(t->right_count, 40000);
  EXPECT_DOUBLE_EQ(t->left_cps, -6800);
  EXPECT_DOUBLE_EQ(t->right_cps, 6790);
  EXPECT_DOUBLE_EQ(t->left_pct, -100);
  EXPECT_DOUBLE_EQ(t->right_pct, 99);
  EXPECT_EQ(t->mode, 'V');
}

TEST(LlcProtocol, RejectsMalformedTelemetry)
{
  EXPECT_FALSE(parse_telemetry("T,1,2,3").has_value());
  EXPECT_FALSE(parse_telemetry("T,1,x,3,4,5,6,7,V").has_value());
  EXPECT_FALSE(parse_telemetry("I,rover1 llc,0.1.0,a,b,c,d,e,f").has_value());
}
