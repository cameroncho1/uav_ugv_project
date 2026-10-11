#ifndef UGV_CONTROL__PATH_FOLLOWER_HPP_
#define UGV_CONTROL__PATH_FOLLOWER_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <uav_ugv_msgs/pose_guard.hpp>

namespace ugv_control
{
struct FollowerLimits
{
  double lookahead{0.6}, speed{0.5}, turn_rate{1.0};
};

// The caller validates frames, timestamps, and finite coordinates.
inline geometry_msgs::msg::Twist follow_path(
  const geometry_msgs::msg::Pose & pose, const nav_msgs::msg::Path & path,
  const FollowerLimits & limits)
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
  std::size_t target = path.poses.size() - 1;
  for (std::size_t i = nearest; i < path.poses.size(); ++i) {
    const auto & p = path.poses[i].pose.position;
    if (std::hypot(p.x - pose.position.x, p.y - pose.position.y) >= limits.lookahead) {
      target = i;
      break;
    }
  }
  const auto & p = path.poses[target].pose.position;
  const auto dx = p.x - pose.position.x, dy = p.y - pose.position.y;
  const auto distance = std::hypot(dx, dy);
  if (distance < 0.05) {return command;}
  const auto error = std::atan2(dy, dx) - uav_ugv_msgs::heading(pose.orientation);
  const auto wrapped = std::atan2(std::sin(error), std::cos(error));
  command.linear.x = std::min(limits.speed, 0.5 * distance) * std::max(0.0, std::cos(wrapped));
  command.angular.z = std::clamp(1.5 * wrapped, -limits.turn_rate, limits.turn_rate);
  return command;
}
}  // namespace ugv_control
#endif  // UGV_CONTROL__PATH_FOLLOWER_HPP_
