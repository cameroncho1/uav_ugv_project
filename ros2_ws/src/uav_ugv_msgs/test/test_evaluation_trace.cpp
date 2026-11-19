#include <gtest/gtest.h>
#include <limits>
#include <uav_ugv_msgs/evaluation_trace.hpp>

TEST(EvaluationTrace, PreservesNanosecondsAndEscapesStrings)
{
  uav_ugv_msgs::EvaluationJson value;
  value.integer("time", 1800000000000000001LL).text("reason", "line\n\"quoted\\")
  .boolean("valid", false).number("missing", std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(
    value.str(),
    "{\"time\":1800000000000000001,\"reason\":\"line\\u000a\\\"quoted\\\\\","
    "\"valid\":false,\"missing\":null}");
}
