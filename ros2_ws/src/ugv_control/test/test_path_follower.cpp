#include <gtest/gtest.h>
#include <ugv_control/path_follower.hpp>

TEST(PathFollower, StopsWithoutAPath)
{
  const geometry_msgs::msg::Pose pose;
  const nav_msgs::msg::Path path;
  const auto cmd = ugv_control::follow_path(pose, path, {});
  EXPECT_EQ(cmd.linear.x, 0);
  EXPECT_EQ(cmd.angular.z, 0);
}

TEST(PathFollower, BoundsSpeedAndTurnsBeforeReversing)
{
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1;
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 10;
  path.poses.push_back(p);
  auto cmd = ugv_control::follow_path(pose, path, {});
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.5);
  EXPECT_DOUBLE_EQ(cmd.angular.z, 0);
  path.poses[0].pose.position.x = -10;
  cmd = ugv_control::follow_path(pose, path, {});
  EXPECT_EQ(cmd.linear.x, 0);
  EXPECT_LE(std::abs(cmd.angular.z), 1);
}

TEST(PathFollower, UsesTheForwardPartOfThePath)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 5;
  pose.orientation.w = 1;
  nav_msgs::msg::Path path;
  for (int i = 0; i < 11; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = i;
    path.poses.push_back(p);
  }
  auto cmd = ugv_control::follow_path(pose, path, {});
  EXPECT_GT(cmd.linear.x, 0);
  EXPECT_DOUBLE_EQ(cmd.angular.z, 0);
  pose.position.x = 10;
  cmd = ugv_control::follow_path(pose, path, {});
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0);
}
