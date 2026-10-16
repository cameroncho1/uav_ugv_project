#include <gtest/gtest.h>
#include <uav_ugv_msgs/grid_geometry.hpp>
#include <uav_ugv_msgs/route_contract.hpp>

nav_msgs::msg::OccupancyGrid grid()
{
  nav_msgs::msg::OccupancyGrid g;
  g.header.frame_id = "map";
  g.header.stamp.sec = 10;
  g.info.resolution = .25;
  g.info.width = g.info.height = 80;
  g.info.origin.position.x = g.info.origin.position.y = -5;
  g.info.origin.orientation.w = 1;
  g.data.assign(6400, 0);
  return g;
}

TEST(RouteSafety, RejectsOldGoalEvenWhenEndpointCoordinatesMatch)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp.sec = 11;
  goal.pose.position.x = 3;
  uav_ugv_msgs::msg::PlannedRoute route;
  route.header.frame_id = "map";
  route.header.stamp.sec = 12;
  route.goal = goal;
  route.path.poses.push_back(goal);
  EXPECT_TRUE(uav_ugv_msgs::route_matches_goal(route, goal));
  route.goal.header.stamp.sec = 10;
  EXPECT_FALSE(uav_ugv_msgs::route_matches_goal(route, goal));
  route.goal = goal;
  route.path.poses.back().pose.position.x = 2;
  EXPECT_FALSE(uav_ugv_msgs::route_matches_goal(route, goal));
}

TEST(RouteSafety, InflationBlocksOccupiedStartAndNarrowClearance)
{
  auto g = grid();
  g.data[20 * 80 + 20] = 100;
  EXPECT_TRUE(uav_ugv_msgs::free_cell(g, 21, 20));
  const auto inflated = uav_ugv_msgs::inflate_grid(g, .45);
  EXPECT_FALSE(uav_ugv_msgs::free_cell(inflated, 21, 20));
  EXPECT_FALSE(uav_ugv_msgs::clear_segment(inflated, 0, 0, 3, 0));
  EXPECT_FALSE(uav_ugv_msgs::free_cell(inflated, 0, 40));
}

TEST(RouteSafety, SegmentTraversalChecksDiagonalNeighbors)
{
  auto g = grid();
  g.data[20 * 80 + 21] = 100;
  EXPECT_FALSE(uav_ugv_msgs::clear_segment(g, .125, .125, .375, .375));
  EXPECT_TRUE(uav_ugv_msgs::clear_segment(g, .125, .125, .125, .375));
}
