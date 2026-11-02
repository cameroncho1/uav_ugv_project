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

// Block every cell from which a robot disk of `radius`, centered anywhere in that cell,
// could touch a non-free cell or leave the grid. The test uses the exact gap between
// two cell squares, so padding is round rather than a square that contains the disk.
// Only non-free cells that border free space stamp the disk: the nearest non-free cell
// to any free cell is always such a border cell.
inline nav_msgs::msg::OccupancyGrid inflate_grid(
  const nav_msgs::msg::OccupancyGrid & grid, double radius)
{
  if (!valid_grid_geometry(grid) || !std::isfinite(radius) || radius < 0 || radius > 10) {
    throw std::invalid_argument("Invalid grid or footprint radius");
  }
  auto result = grid;
  const auto r = grid.info.resolution;
  const int width = static_cast<int>(grid.info.width), height = static_cast<int>(grid.info.height);
  const int reach = static_cast<int>(std::floor(radius / r)) + 1;
  std::vector<std::pair<int, int>> disk;
  for (int dy = -reach; dy <= reach; ++dy) {
    for (int dx = -reach; dx <= reach; ++dx) {
      const auto gap_x = std::max(std::abs(dx) - 1, 0) * r;
      const auto gap_y = std::max(std::abs(dy) - 1, 0) * r;
      if (std::hypot(gap_x, gap_y) <= radius) {disk.emplace_back(dx, dy);}
    }
  }
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const auto i = static_cast<std::size_t>(y) * width + x;
      // Cells whose square lies within the radius of the grid boundary.
      if (std::min({x, y, width - 1 - x, height - 1 - y}) * r <= radius) {
        result.data[i] = 100;
      }
      if (free_cell(grid, x, y)) {continue;}
      bool borders_free = false;
      for (int dy = -1; dy <= 1 && !borders_free; ++dy) {
        for (int dx = -1; dx <= 1 && !borders_free; ++dx) {
          borders_free = free_cell(grid, x + dx, y + dy);
        }
      }
      if (!borders_free) {continue;}
      for (const auto & [dx, dy] : disk) {
        const int nx = x + dx, ny = y + dy;
        if (nx >= 0 && ny >= 0 && nx < width && ny < height) {
          result.data[static_cast<std::size_t>(ny) * width + nx] = 100;
        }
      }
    }
  }
  return result;
}

// Indices of a polyline that keep every shortcut clear: from each kept point, extend
// to the last point reachable by one clear segment. The first and last points stay.
inline std::vector<std::size_t> shortcut_indices(
  const nav_msgs::msg::OccupancyGrid & grid,
  const std::vector<std::pair<double, double>> & points)
{
  std::vector<std::size_t> kept;
  if (points.empty()) {return kept;}
  kept.push_back(0);
  std::size_t anchor = 0;
  while (anchor + 1 < points.size()) {
    std::size_t next = anchor + 1;
    while (next + 1 < points.size() && clear_segment(
        grid, points[anchor].first, points[anchor].second,
        points[next + 1].first, points[next + 1].second))
    {
      ++next;
    }
    kept.push_back(next);
    anchor = next;
  }
  return kept;
}
}  // namespace uav_ugv_msgs
#endif
