#include <gtest/gtest.h>
#include <uav_control/follow_setpoint.hpp>

TEST(FollowSetpoint, MapEnuBecomesPx4NedAroundTheMapOrigin)
{
  // UGV at map (-4, 1); PX4's local origin sits at map (0.5, -0.5).
  const auto ned = uav_control::map_to_ned(-4.0, 1.0, 0.5, -0.5);
  EXPECT_DOUBLE_EQ(ned.north, 1.5);
  EXPECT_DOUBLE_EQ(ned.east, -4.5);
}

TEST(FollowSetpoint, StepIsRateLimitedAndIgnoresJitter)
{
  const uav_control::NorthEast here{0.0, 0.0};
  auto next = uav_control::step_toward(here, {0.0, -4.0}, 0.1, 0.15);
  EXPECT_DOUBLE_EQ(next.north, 0.0);
  EXPECT_DOUBLE_EQ(next.east, -0.1);
  next = uav_control::step_toward(here, {0.1, 0.05}, 0.1, 0.15);
  EXPECT_DOUBLE_EQ(next.north, 0.0);
  EXPECT_DOUBLE_EQ(next.east, 0.0);
  next = uav_control::step_toward(here, {0.12, 0.16}, 0.5, 0.15);  // Within one step.
  EXPECT_DOUBLE_EQ(next.north, 0.12);
  EXPECT_DOUBLE_EQ(next.east, 0.16);
}
