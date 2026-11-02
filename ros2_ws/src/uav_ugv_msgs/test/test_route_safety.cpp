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

TEST(RouteSafety, PaddingIsRoundAndMatchesTheFootprintOnFineGrids)
{
  auto g = grid();
  g.info.resolution = .1;
  g.data[40 * 80 + 40] = 100;
  const auto inflated = uav_ugv_msgs::inflate_grid(g, .55);
  // Along an axis the gap to the obstacle square is (cells - 1) * 0.1 m.
  EXPECT_FALSE(uav_ugv_msgs::free_cell(inflated, 46, 40));
  EXPECT_TRUE(uav_ugv_msgs::free_cell(inflated, 47, 40));
  EXPECT_FALSE(uav_ugv_msgs::free_cell(inflated, 40, 34));
  EXPECT_TRUE(uav_ugv_msgs::free_cell(inflated, 40, 33));
  // A diagonal gap of hypot(0.4, 0.4) = 0.57 m is outside the disk; a square would block it.
  EXPECT_TRUE(uav_ugv_msgs::free_cell(inflated, 45, 45));
  EXPECT_FALSE(uav_ugv_msgs::free_cell(inflated, 44, 45));
}

TEST(RouteSafety, ShortcutsKeepEndpointsAndOnlyClearSegments)
{
  auto g = grid();
  // Wall at x = 0 for y in [-5, 0.5); the route must pass above it.
  for (int y = 0; y < 22; ++y) {
    g.data[y * 80 + 20] = 100;
  }
  const std::vector<std::pair<double, double>> points{
    {-1, -1}, {-1, 0}, {-1, 1}, {0, 1}, {1, 1}, {1, 0}, {1, -1}};
  const auto kept = uav_ugv_msgs::shortcut_indices(g, points);
  ASSERT_GE(kept.size(), 3u);
  EXPECT_EQ(kept.front(), 0u);
  EXPECT_EQ(kept.back(), points.size() - 1);
  EXPECT_LT(kept.size(), points.size());
  for (std::size_t i = 1; i < kept.size(); ++i) {
    const auto & a = points[kept[i - 1]];
    const auto & b = points[kept[i]];
    EXPECT_TRUE(uav_ugv_msgs::clear_segment(g, a.first, a.second, b.first, b.second));
  }
  // Straight free line collapses to its endpoints.
  const std::vector<std::pair<double, double>> line{{-3, 2}, {-2, 2}, {-1, 2}, {0, 2}};
  EXPECT_EQ(uav_ugv_msgs::shortcut_indices(g, line), (std::vector<std::size_t>{0, 3}));
}
