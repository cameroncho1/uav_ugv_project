#ifndef UGV_CONTROL__PATH_FOLLOWER_HPP_
#define UGV_CONTROL__PATH_FOLLOWER_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <uav_ugv_msgs/pose_guard.hpp>
#include <uav_ugv_msgs/grid_geometry.hpp>

namespace ugv_control
{
struct FollowerLimits
{
  double lookahead{0.6}, speed{0.5}, turn_rate{1.0};
};

// The caller validates frames, timestamps, and finite coordinates.
inline geometry_msgs::msg::Twist follow_path(
  const geometry_msgs::msg::Pose & pose, const nav_msgs::msg::Path & path,
  const FollowerLimits & limits, const nav_msgs::msg::OccupancyGrid & clearance)
{
  geometry_msgs::msg::Twist command;
  if (path.poses.empty()) {return command;}
  std::size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const auto & p = path.poses[i].pose.position;
    const auto distance = std::hypot(p.x - pose.position.x, p.y - pose.position.y);
    if (distance < best) {best = distance; nearest = i;}
  }
  std::size_t target = nearest;
  double arc = 0.0;
  for (std::size_t i = nearest; i < path.poses.size(); ++i) {
    const auto & p = path.poses[i].pose.position;
    if (!uav_ugv_msgs::clear_segment(clearance, pose.position.x, pose.position.y, p.x, p.y)) {
      break;
    }
    target = i;
    if (i > nearest) {
      const auto & prev = path.poses[i - 1].pose.position;
      arc += std::hypot(p.x - prev.x, p.y - prev.y);
    }
    if (arc >= limits.lookahead) {break;}
  }
  const auto & p = path.poses[target].pose.position;
  const auto dx = p.x - pose.position.x, dy = p.y - pose.position.y;
  const auto distance = std::hypot(dx, dy);
  if (distance < 0.05) {return command;}
  const auto error = std::atan2(dy, dx) - uav_ugv_msgs::heading(pose.orientation);
  const auto wrapped = std::atan2(std::sin(error), std::cos(error));
  // Turn before translating through a bend; never steer across blocked cells.
  command.linear.x = std::abs(wrapped) <= 0.15 ? std::min(limits.speed, 0.5 * distance) : 0.0;
  command.angular.z = std::clamp(1.5 * wrapped, -limits.turn_rate, limits.turn_rate);
  double previous_x = pose.position.x, previous_y = pose.position.y;
  const auto yaw = uav_ugv_msgs::heading(pose.orientation);
  for (int step = 1; step <= 8; ++step) {
    const auto time = step * 0.05;
    const auto angle = yaw + command.angular.z * time;
    const auto x = pose.position.x + (std::abs(command.angular.z) < 1e-9 ?
      command.linear.x * time * std::cos(yaw) : command.linear.x / command.angular.z *
      (std::sin(angle) - std::sin(yaw)));
    const auto y = pose.position.y + (std::abs(command.angular.z) < 1e-9 ?
      command.linear.x * time * std::sin(yaw) : command.linear.x / command.angular.z *
      (std::cos(yaw) - std::cos(angle)));
    if (!uav_ugv_msgs::clear_segment(clearance, previous_x, previous_y, x, y)) {
      command.linear.x = 0.0;
      break;
    }
    previous_x = x; previous_y = y;
  }
  return command;
}
}  // namespace ugv_control
#endif  // UGV_CONTROL__PATH_FOLLOWER_HPP_
