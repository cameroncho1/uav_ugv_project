#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class LocalAstarPlanner : public rclcpp::Node {
public:
  LocalAstarPlanner() : Node("local_astar_planner") {
    max_goal_adjust_cells_ = declare_parameter("max_goal_adjust_cells", 5);

    auto latched_qos = rclcpp::QoS(1).reliable().transient_local();

    //subscribe to local grid published by traversability mapper
    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_grid", latched_qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        grid_ = *msg;
        have_grid_ = true;
      });

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/global_goal_pose", latched_qos,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_ = *msg;
        have_goal_ = true;
      });
    ugv_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ugv_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        ugv_pose_ = *msg;
        have_ugv_pose_ = true;
      });

    //publish local path for UGV controller to use
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/local_path", latched_qos);
    timer_ = create_wall_timer(200ms, [this]() { plan(); });
  }

private:
  struct Cell { int x; int y; };

  bool cell_in_bounds(const Cell &cell) const {
    return cell.x >= 0 && cell.y >= 0 &&
      cell.x < static_cast<int>(grid_.info.width) &&
      cell.y < static_cast<int>(grid_.info.height);
  }

  std::size_t index_of(const Cell &cell) const {
    return static_cast<std::size_t>(cell.y) * grid_.info.width +
      static_cast<std::size_t>(cell.x);
  }

  bool traversable(const Cell &cell) const {
    return cell_in_bounds(cell) && grid_.data[index_of(cell)] >= 0 &&
      grid_.data[index_of(cell)] < 50;
  }

  Cell world_to_cell(double x, double y) const {
    return {
      static_cast<int>(std::floor((x - grid_.info.origin.position.x) / grid_.info.resolution)),
      static_cast<int>(std::floor((y - grid_.info.origin.position.y) / grid_.info.resolution))
    };
  }

  geometry_msgs::msg::Point cell_center(const Cell &cell) const {
    geometry_msgs::msg::Point point;
    point.x = grid_.info.origin.position.x + (cell.x + 0.5) * grid_.info.resolution;
    point.y = grid_.info.origin.position.y + (cell.y + 0.5) * grid_.info.resolution;
    point.z = 0.0;
    return point;
  }

  bool nearest_free(Cell &cell) const {
    if (traversable(cell)) {
      return true;
    }
    const Cell original = cell;
    double best_distance = std::numeric_limits<double>::infinity();
    bool found = false;
    for (int radius = 1; radius <= max_goal_adjust_cells_; ++radius) {
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          Cell candidate{original.x + dx, original.y + dy};
          if (!traversable(candidate)) {
            continue;
          }
          const double distance = static_cast<double>(dx * dx + dy * dy);
          if (distance < best_distance) {
            best_distance = distance;
            cell = candidate;
            found = true;
          }
        }
      }
      if (found) {
        return true;
      }
    }
    return false;
  }

  double heuristic(const Cell &from, const Cell &to) const {
    const double dx = std::abs(from.x - to.x);
    const double dy = std::abs(from.y - to.y);
    const double diagonal = std::min(dx, dy);
    return std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * diagonal;
  }

  std::vector<Cell> find_path(Cell start, Cell goal) const {
    if (!nearest_free(start) || !nearest_free(goal)) {
      return {};
    }

    const std::size_t cell_count = static_cast<std::size_t>(grid_.info.width) * grid_.info.height;
    const std::size_t start_index = index_of(start);
    const std::size_t goal_index = index_of(goal);
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> costs(cell_count, infinity);
    std::vector<std::size_t> parents(cell_count, cell_count);
    std::vector<bool> closed(cell_count, false);
    using QueueItem = std::pair<double, std::size_t>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;

    costs[start_index] = 0.0;
    open.emplace(heuristic(start, goal), start_index);
    constexpr std::array<int, 8> dx{{-1, 0, 1, -1, 1, -1, 0, 1}};
    constexpr std::array<int, 8> dy{{-1, -1, -1, 0, 0, 1, 1, 1}};

    while (!open.empty()) {
      const auto [estimated_total, current_index] = open.top();
      (void)estimated_total;
      open.pop();
      if (closed[current_index]) {
        continue;
      }
      if (current_index == goal_index) {
        std::vector<Cell> path;
        for (std::size_t cursor = goal_index; cursor != cell_count; cursor = parents[cursor]) {
          path.push_back({static_cast<int>(cursor % grid_.info.width),
            static_cast<int>(cursor / grid_.info.width)});
          if (cursor == start_index) {
            break;
          }
        }
        std::reverse(path.begin(), path.end());
        return path;
      }
      closed[current_index] = true;
      const Cell current{static_cast<int>(current_index % grid_.info.width),
        static_cast<int>(current_index / grid_.info.width)};

      for (std::size_t direction = 0; direction < dx.size(); ++direction) {
        const Cell next{current.x + dx[direction], current.y + dy[direction]};
        if (!traversable(next)) {
          continue;
        }
        const bool diagonal = dx[direction] != 0 && dy[direction] != 0;
        if (diagonal && (!traversable({current.x + dx[direction], current.y}) ||
          !traversable({current.x, current.y + dy[direction]}))) {
          continue;
        }

        const std::size_t next_index = index_of(next);
        if (closed[next_index]) {
          continue;
        }
        const double next_cost = costs[current_index] + (diagonal ? std::sqrt(2.0) : 1.0);
        if (next_cost < costs[next_index]) {
          costs[next_index] = next_cost;
          parents[next_index] = current_index;
          open.emplace(next_cost + heuristic(next, goal), next_index);
        }
      }
    }
    return {};
  }

  void plan() {
    if (!have_grid_ || !have_goal_ || !have_ugv_pose_) {
      return;
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();
    const Cell start = world_to_cell(ugv_pose_.pose.position.x, ugv_pose_.pose.position.y);
    const Cell goal = world_to_cell(goal_.pose.position.x, goal_.pose.position.y);
    const auto cells = find_path(start, goal);
    for (const auto &cell : cells) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = cell_center(cell);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    if (cells.empty()) {
      const double min_x = grid_.info.origin.position.x;
      const double min_y = grid_.info.origin.position.y;
      const double max_x = min_x + grid_.info.width * grid_.info.resolution;
      const double max_y = min_y + grid_.info.height * grid_.info.resolution;
      const char *start_state = !cell_in_bounds(start) ? "outside" :
        (traversable(start) ? "free" : "blocked");
      const char *goal_state = !cell_in_bounds(goal) ? "outside" :
        (traversable(goal) ? "free" : "blocked");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "No local A-star path: UGV (%.2f, %.2f) cell (%d, %d) %s; goal (%.2f, %.2f) "
        "cell (%d, %d) %s; grid x=[%.2f, %.2f), y=[%.2f, %.2f)",
        ugv_pose_.pose.position.x, ugv_pose_.pose.position.y, start.x, start.y, start_state,
        goal_.pose.position.x, goal_.pose.position.y, goal.x, goal.y, goal_state,
        min_x, max_x, min_y, max_y);
    }
    path_pub_->publish(path);
  }

  int max_goal_adjust_cells_{5};
  bool have_grid_{false};
  bool have_goal_{false};
  bool have_ugv_pose_{false};
  nav_msgs::msg::OccupancyGrid grid_;
  geometry_msgs::msg::PoseStamped goal_;
  geometry_msgs::msg::PoseStamped ugv_pose_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ugv_pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalAstarPlanner>());
  rclcpp::shutdown();
  return 0;
}