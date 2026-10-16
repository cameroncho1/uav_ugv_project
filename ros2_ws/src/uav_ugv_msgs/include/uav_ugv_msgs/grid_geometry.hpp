#ifndef UAV_UGV_MSGS__GRID_GEOMETRY_HPP_
#define UAV_UGV_MSGS__GRID_GEOMETRY_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>
#include <utility>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace uav_ugv_msgs
{
inline bool valid_grid_geometry(const nav_msgs::msg::OccupancyGrid & grid)
{
  const auto & p = grid.info.origin.position;
  const auto & q = grid.info.origin.orientation;
  const auto cells = static_cast<uint64_t>(grid.info.width) * grid.info.height;
  return grid.header.frame_id == "map" && cells > 0 && cells <= 1000000 &&
         grid.data.size() == cells && std::isfinite(grid.info.resolution) &&
         grid.info.resolution > 0 && std::isfinite(p.x) && std::isfinite(p.y) &&
         std::abs(q.x) < 1e-6 && std::abs(q.y) < 1e-6 && std::abs(q.z) < 1e-6 &&
         std::abs(std::abs(q.w) - 1) < 1e-6;
}

inline bool free_cell(const nav_msgs::msg::OccupancyGrid & grid, int x, int y)
{
  if (x < 0 || y < 0 || x >= static_cast<int>(grid.info.width) ||
    y >= static_cast<int>(grid.info.height)) {return false;}
  const auto value = grid.data[static_cast<std::size_t>(y) * grid.info.width + x];
  return value >= 0 && value < 50;
}

// Supercover traversal visits both adjacent cells at a diagonal corner.
inline bool clear_segment(
  const nav_msgs::msg::OccupancyGrid & grid, double ax, double ay, double bx, double by)
{
  if (!valid_grid_geometry(grid)) {return false;}
  const auto r = grid.info.resolution;
  ax = (ax - grid.info.origin.position.x) / r;
  ay = (ay - grid.info.origin.position.y) / r;
  bx = (bx - grid.info.origin.position.x) / r;
  by = (by - grid.info.origin.position.y) / r;
  for (const auto v : {ax, ay, bx, by}) {
    if (!std::isfinite(v) || std::abs(v) > 1000000) {return false;}
  }
  int x = static_cast<int>(std::floor(ax)), y = static_cast<int>(std::floor(ay));
  const int end_x = static_cast<int>(std::floor(bx)), end_y = static_cast<int>(std::floor(by));
  const auto dx = bx - ax, dy = by - ay;
  const int sx = dx > 0 ? 1 : -1, sy = dy > 0 ? 1 : -1;
  const auto infinity = std::numeric_limits<double>::infinity();
  const auto tx = dx == 0 ? infinity : std::abs(1 / dx);
  const auto ty = dy == 0 ? infinity : std::abs(1 / dy);
  auto next_x = dx == 0 ? infinity : (dx > 0 ? x + 1 - ax : ax - x) * tx;
  auto next_y = dy == 0 ? infinity : (dy > 0 ? y + 1 - ay : ay - y) * ty;
  for (uint64_t steps = 0; steps <= grid.data.size() * 2; ++steps) {
    if (!free_cell(grid, x, y)) {return false;}
    if (x == end_x && y == end_y) {return true;}
    if (std::abs(next_x - next_y) < 1e-12) {
      if (!free_cell(grid, x + sx, y) || !free_cell(grid, x, y + sy)) {return false;}
      x += sx; y += sy; next_x += tx; next_y += ty;
    } else if (next_x < next_y) {
      x += sx; next_x += tx;
    } else {
      y += sy; next_y += ty;
    }
  }
  return false;
}

inline nav_msgs::msg::OccupancyGrid inflate_grid(
  const nav_msgs::msg::OccupancyGrid & grid, double radius)
{
  if (!valid_grid_geometry(grid) || !std::isfinite(radius) || radius < 0 || radius > 10) {
    throw std::invalid_argument("Invalid grid or footprint radius");
  }
  auto result = grid;
  const auto r = grid.info.resolution;
  const auto edge = radius + r * std::sqrt(0.5);
  // A square contains the circular footprint. Two distance-transform passes
  // conservatively inflate in linear time, even for dense or fine grids.
  const double cells = std::ceil(edge / r);
  const int width = static_cast<int>(grid.info.width), height = static_cast<int>(grid.info.height);
  std::vector<int> distance(grid.data.size(), 1000001);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const auto i = static_cast<std::size_t>(y) * width + x;
      if (!free_cell(grid, x, y)) {distance[i] = 0;}
      for (const auto & offset : {std::pair<int, int>{-1, 0}, {-1, -1}, {0, -1}, {1, -1}}) {
        const int nx = x + offset.first, ny = y + offset.second;
        if (nx >= 0 && nx < width && ny >= 0) {
          distance[i] = std::min(
            distance[i],
            distance[static_cast<std::size_t>(ny) * width + nx] + 1);
        }
      }
    }
  }
  for (int y = height - 1; y >= 0; --y) {
    for (int x = width - 1; x >= 0; --x) {
      const auto i = static_cast<std::size_t>(y) * width + x;
      for (const auto & offset : {std::pair<int, int>{1, 0}, {-1, 1}, {0, 1}, {1, 1}}) {
        const int nx = x + offset.first, ny = y + offset.second;
        if (nx >= 0 && nx < width && ny < height) {
          distance[i] = std::min(
            distance[i],
            distance[static_cast<std::size_t>(ny) * width + nx] + 1);
        }
      }
      if (distance[i] <= cells || std::min(
          {x + 0.5, y + 0.5,
            width - x - 0.5, height - y - 0.5}) * r <= edge)
      {
        result.data[i] = 100;
      }
    }
  }
  return result;
}
}  // namespace uav_ugv_msgs
#endif
