#include <gtest/gtest.h>
#include <ugv_control/path_follower.hpp>

nav_msgs::msg::OccupancyGrid free_grid()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.width = grid.info.height = 100;
  grid.info.resolution = 0.25;
  grid.info.origin.position.x = grid.info.origin.position.y = -10;
  grid.info.origin.orientation.w = 1;
  grid.data.assign(10000, 0);
  return grid;
}

TEST(PathFollower, StopsWithoutAPath)
{
  const geometry_msgs::msg::Pose pose;
  const nav_msgs::msg::Path path;
  const auto cmd = ugv_control::follow_path(pose, path, {}, free_grid());
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
  auto cmd = ugv_control::follow_path(pose, path, {}, free_grid());
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.5);
  EXPECT_DOUBLE_EQ(cmd.angular.z, 0);
  path.poses[0].pose.position.x = -10;
  cmd = ugv_control::follow_path(pose, path, {}, free_grid());
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
  auto cmd = ugv_control::follow_path(pose, path, {}, free_grid());
  EXPECT_GT(cmd.linear.x, 0);
  EXPECT_DOUBLE_EQ(cmd.angular.z, 0);
  pose.position.x = 10;
  cmd = ugv_control::follow_path(pose, path, {}, free_grid());
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0);
}

TEST(PathFollower, DoesNotDriveAcrossABlockedBend)
{
  auto grid = free_grid();
  grid.info.resolution = 0.1;
  grid.info.origin.position.x = grid.info.origin.position.y = 0;
  grid.data[12 * 100 + 12] = 100;
  geometry_msgs::msg::Pose pose;
  pose.position.x = pose.position.y = 1.05;
  pose.orientation.w = 1;
  nav_msgs::msg::Path path;
  for (const auto & xy : {std::pair<double, double>{1.05, 1.05},
      {1.05, 1.25}, {1.05, 1.45}, {1.25, 1.45}, {1.45, 1.45}})
  {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = xy.first; p.pose.position.y = xy.second;
    path.poses.push_back(p);
  }
  EXPECT_FALSE(uav_ugv_msgs::clear_segment(grid, 1.05, 1.05, 1.45, 1.45));
  const auto command = ugv_control::follow_path(pose, path, {}, grid);
  EXPECT_DOUBLE_EQ(command.linear.x, 0);
  EXPECT_GT(command.angular.z, 0);
}
