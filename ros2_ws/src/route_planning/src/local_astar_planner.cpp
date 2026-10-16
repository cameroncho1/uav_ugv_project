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
#include <uav_ugv_msgs/pose_guard.hpp>
#include <uav_ugv_msgs/grid_geometry.hpp>
#include <uav_ugv_msgs/route_contract.hpp>

using namespace std::chrono_literals;

class LocalAstarPlanner : public rclcpp::Node
{
public:
  LocalAstarPlanner()
  : Node("local_astar_planner"), pose_guard_(pose_limits())
  {
    max_grid_age_ = declare_parameter("max_grid_age", 2.0);
    robot_radius_ = declare_parameter("robot_radius", 0.5);
    clearance_ = declare_parameter("obstacle_clearance", 0.05);
    if (!std::isfinite(max_grid_age_) || max_grid_age_ <= 0 ||
      !std::isfinite(robot_radius_) || robot_radius_ <= 0 || robot_radius_ > 5 ||
      !std::isfinite(clearance_) || clearance_ < 0 || clearance_ > 5)
    {
      throw std::invalid_argument("Invalid planner limits");
    }

    auto latched_qos = rclcpp::QoS(1).reliable().transient_local();

    //subscribe to local grid published by traversability mapper
    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_grid", latched_qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        grid_ = *msg;
        have_grid_ = true;
        grid_receipt_ = uav_ugv_msgs::steady_time_ns();
      });

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/global_goal_pose", latched_qos,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_ = *msg;
        have_goal_ = uav_ugv_msgs::valid_goal(*msg);
      });
    ugv_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/ugv/localization/pose", 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        pose_guard_.update(*msg, now().nanoseconds(), uav_ugv_msgs::steady_time_ns());
      });

    //publish local path for UGV controller to use
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/local_path", latched_qos);
    route_pub_ = create_publisher<uav_ugv_msgs::msg::PlannedRoute>(
      "/ugv/navigation/route", latched_qos);
    timer_ = create_wall_timer(200ms, [this]() {plan();});
  }

private:
  uav_ugv_msgs::PoseLimits pose_limits()
  {
    return {declare_parameter("max_pose_age", 0.5),
      declare_parameter("max_position_stddev", 0.5),
      declare_parameter("max_heading_stddev", 0.2)};
  }

  bool valid_grid() const
  {
    const auto stamp = uav_ugv_msgs::stamp_ns(grid_.header.stamp);
    const auto age = (now().nanoseconds() - stamp) * 1e-9;
    const auto & origin = grid_.info.origin;
    const auto & q = origin.orientation;
    const auto cells = static_cast<uint64_t>(grid_.info.width) * grid_.info.height;
    return have_grid_ && grid_.header.frame_id == "map" && stamp > 0 &&
           age >= -0.05 && age <= max_grid_age_ &&
           (uav_ugv_msgs::steady_time_ns() - grid_receipt_) * 1e-9 <= max_grid_age_ &&
           cells > 0 && cells <= 1000000 && grid_.data.size() == cells &&
           std::isfinite(grid_.info.resolution) && grid_.info.resolution > 0 &&
           std::isfinite(origin.position.x) && std::isfinite(origin.position.y) &&
           std::abs(q.x) < 1e-6 && std::abs(q.y) < 1e-6 && std::abs(q.z) < 1e-6 &&
           std::abs(std::abs(q.w) - 1) < 1e-6;
  }

  struct Cell { int x; int y; };

  bool cell_in_bounds(const Cell & cell) const
  {
    return cell.x >= 0 && cell.y >= 0 &&
           cell.x < static_cast<int>(grid_.info.width) &&
           cell.y < static_cast<int>(grid_.info.height);
  }

  std::size_t index_of(const Cell & cell) const
  {
    return static_cast<std::size_t>(cell.y) * grid_.info.width +
           static_cast<std::size_t>(cell.x);
  }

  bool traversable(const Cell & cell) const
  {
    return cell_in_bounds(cell) && safety_grid_.data[index_of(cell)] >= 0 &&
           safety_grid_.data[index_of(cell)] < 50;
  }

  Cell world_to_cell(double x, double y) const
  {
    return {
      static_cast<int>(std::floor((x - grid_.info.origin.position.x) / grid_.info.resolution)),
      static_cast<int>(std::floor((y - grid_.info.origin.position.y) / grid_.info.resolution))
    };
  }

  geometry_msgs::msg::Point cell_center(const Cell & cell) const
  {
    geometry_msgs::msg::Point point;
    point.x = grid_.info.origin.position.x + (cell.x + 0.5) * grid_.info.resolution;
    point.y = grid_.info.origin.position.y + (cell.y + 0.5) * grid_.info.resolution;
    point.z = 0.0;
    return point;
  }

  double heuristic(const Cell & from, const Cell & to) const
  {
    const double dx = std::abs(from.x - to.x);
    const double dy = std::abs(from.y - to.y);
    const double diagonal = std::min(dx, dy);
    return std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * diagonal;
  }

  std::vector<Cell> find_path(Cell start, Cell goal) const
  {
    if (!traversable(start) || !traversable(goal)) {
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
          path.push_back(
            {static_cast<int>(cursor % grid_.info.width),
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
          !traversable({current.x, current.y + dy[direction]})))
        {
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

  void plan()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();
    if (!pose_guard_.check(now().nanoseconds(), uav_ugv_msgs::steady_time_ns()) ||
      !have_goal_ || !valid_grid())
    {
      publish_route(path);
      return;
    }
    safety_grid_ = uav_ugv_msgs::inflate_grid(grid_, robot_radius_ + clearance_);
    const auto & ugv_pose_ = pose_guard_.pose();
    // Bound coordinates before converting floating-point cells to integers.
    for (const auto & p : {ugv_pose_.position, goal_.pose.position}) {
      if (std::abs((p.x - grid_.info.origin.position.x) / grid_.info.resolution) > 1000000 ||
        std::abs((p.y - grid_.info.origin.position.y) / grid_.info.resolution) > 1000000)
      {
        publish_route(path);
        return;
      }
    }
    const Cell start = world_to_cell(ugv_pose_.position.x, ugv_pose_.position.y);
    const Cell goal = world_to_cell(goal_.pose.position.x, goal_.pose.position.y);
    const auto cells = find_path(start, goal);
    for (const auto & cell : cells) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = cell_center(cell);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    if (!cells.empty()) {
      geometry_msgs::msg::PoseStamped start_pose, goal_pose;
      start_pose.header = goal_pose.header = path.header;
      start_pose.pose = ugv_pose_;
      goal_pose.pose = goal_.pose;
      path.poses.insert(path.poses.begin(), start_pose);
      path.poses.push_back(goal_pose);
      for (std::size_t i = 1; i < path.poses.size(); ++i) {
        const auto & a = path.poses[i - 1].pose.position;
        const auto & b = path.poses[i].pose.position;
        if (!uav_ugv_msgs::clear_segment(safety_grid_, a.x, a.y, b.x, b.y)) {
          path.poses.clear();
          break;
        }
      }
    }
    if (cells.empty()) {
      const double min_x = grid_.info.origin.position.x;
      const double min_y = grid_.info.origin.position.y;
      const double max_x = min_x + grid_.info.width * grid_.info.resolution;
      const double max_y = min_y + grid_.info.height * grid_.info.resolution;
      const char * start_state = !cell_in_bounds(start) ? "outside" :
        (traversable(start) ? "free" : "blocked");
      const char * goal_state = !cell_in_bounds(goal) ? "outside" :
        (traversable(goal) ? "free" : "blocked");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No local A-star path: UGV (%.2f, %.2f) cell (%d, %d) %s; goal (%.2f, %.2f) "
        "cell (%d, %d) %s; grid x=[%.2f, %.2f), y=[%.2f, %.2f)",
        ugv_pose_.position.x, ugv_pose_.position.y, start.x, start.y, start_state,
        goal_.pose.position.x, goal_.pose.position.y, goal.x, goal.y, goal_state,
        min_x, max_x, min_y, max_y);
    }
    publish_route(path);
  }

  void publish_route(const nav_msgs::msg::Path & path)
  {
    uav_ugv_msgs::msg::PlannedRoute route;
    route.header = path.header;
    route.goal = goal_;
    route.path = path;
    route.clearance_grid = safety_grid_;
    path_pub_->publish(path);
    route_pub_->publish(route);
  }

  double robot_radius_{0.5}, clearance_{0.05};
  bool have_grid_{false};
  bool have_goal_{false};
  uav_ugv_msgs::PoseGuard pose_guard_;
  double max_grid_age_{2.0};
  int64_t grid_receipt_{0};
  nav_msgs::msg::OccupancyGrid grid_, safety_grid_;
  geometry_msgs::msg::PoseStamped goal_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ugv_pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<uav_ugv_msgs::msg::PlannedRoute>::SharedPtr route_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalAstarPlanner>());
  rclcpp::shutdown();
  return 0;
}
