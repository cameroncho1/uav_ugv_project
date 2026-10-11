#include <limits>
#include <gtest/gtest.h>
#include <uav_ugv_msgs/pose_guard.hpp>

using uav_ugv_msgs::PoseGuard;

geometry_msgs::msg::PoseWithCovarianceStamped reading()
{
  geometry_msgs::msg::PoseWithCovarianceStamped m;
  m.header.frame_id = "map";
  m.header.stamp.sec = 10;
  m.pose.pose.orientation.w = 1;
  m.pose.covariance[0] = 0.04;
  m.pose.covariance[7] = 0.04;
  m.pose.covariance[35] = 0.01;
  return m;
}

TEST(PoseGuard, RejectsAcquisitionAndReceiptOutages)
{
  PoseGuard guard;
  guard.update(reading(), 10000000000, 1000000000);
  EXPECT_TRUE(guard.check(10000000000, 1000000000));
  EXPECT_FALSE(guard.check(10600000000, 1000000000));
  guard.update(reading(), 10000000000, 1000000000);
  EXPECT_FALSE(guard.check(10000000000, 1600000000));
}

TEST(PoseGuard, RejectsInvalidFramesTimesAndValues)
{
  for (int fault = 0; fault < 9; ++fault) {
    PoseGuard guard;
    auto m = reading();
    switch (fault) {
      case 0: m.header.frame_id = "odom"; break;
      case 1: m.header.stamp.sec = 11; break;
      case 2: m.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN(); break;
      case 3: m.pose.pose.orientation.w = 0; break;
      case 4: m.pose.covariance[0] = 0; break;
      case 5: m.pose.covariance[1] = 0.01; break;
      case 6: m.pose.covariance[0] = 1; break;
      case 7: m.pose.covariance[35] = 1; break;
      case 8: m.pose.covariance[5] = m.pose.covariance[30] = 1; break;
    }
    guard.update(m, 10000000000, 1000000000);
    EXPECT_FALSE(guard.check(10000000000, 1000000000)) << fault;
  }
}

TEST(PoseGuard, BoundsCorrelatedPositionUncertainty)
{
  PoseGuard guard;
  auto m = reading();
  m.pose.covariance[0] = m.pose.covariance[7] = 0.2;
  m.pose.covariance[1] = m.pose.covariance[6] = 0.15;
  guard.update(m, 10000000000, 1000000000);
  EXPECT_FALSE(guard.check(10000000000, 1000000000));
}

TEST(PoseGuard, RequiresNewSamplesAndRecoversAfterClockReset)
{
  PoseGuard guard;
  auto m = reading();
  guard.update(m, 10000000000, 1000000000);
  guard.update(m, 10000000000, 1100000000);
  EXPECT_FALSE(guard.check(10000000000, 1100000000));
  m.header.stamp.nanosec = 100000000;
  guard.update(m, 10100000000, 1200000000);
  EXPECT_TRUE(guard.check(10100000000, 1200000000));
  EXPECT_FALSE(guard.check(2000000000, 1300000000));
  m.header.stamp.sec = 2;
  m.header.stamp.nanosec = 0;
  guard.update(m, 2000000000, 1400000000);
  EXPECT_TRUE(guard.check(2000000000, 1400000000));
}

TEST(PoseGuard, RejectsInvalidLimits)
{
  EXPECT_THROW(PoseGuard({0, 0.5, 0.2}), std::invalid_argument);
}
