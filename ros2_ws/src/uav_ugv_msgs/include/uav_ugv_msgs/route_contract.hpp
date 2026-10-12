#ifndef UAV_UGV_MSGS__ROUTE_CONTRACT_HPP_
#define UAV_UGV_MSGS__ROUTE_CONTRACT_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <uav_ugv_msgs/pose_guard.hpp>
#include <uav_ugv_msgs/msg/planned_route.hpp>

namespace uav_ugv_msgs
{
inline bool valid_goal(const geometry_msgs::msg::PoseStamped & goal)
{
  return goal.header.frame_id == "map" && stamp_ns(goal.header.stamp) > 0 &&
         std::isfinite(goal.pose.position.x) && std::isfinite(goal.pose.position.y);
}

inline bool same_goal(
  const geometry_msgs::msg::PoseStamped & a, const geometry_msgs::msg::PoseStamped & b)
{
  return valid_goal(a) && valid_goal(b) && a.header.stamp == b.header.stamp &&
         a.pose.position.x == b.pose.position.x && a.pose.position.y == b.pose.position.y;
}

inline bool route_matches_goal(
  const uav_ugv_msgs::msg::PlannedRoute & route,
  const geometry_msgs::msg::PoseStamped & goal, double endpoint_tolerance = 1e-6)
{
  if (!same_goal(route.goal, goal) || route.path.poses.empty() ||
    stamp_ns(route.header.stamp) < stamp_ns(goal.header.stamp)) {return false;}
  const auto & end = route.path.poses.back().pose.position;
  return std::hypot(end.x - goal.pose.position.x, end.y - goal.pose.position.y) <=
         endpoint_tolerance;
}
}  // namespace uav_ugv_msgs
#endif
