#ifndef UAV_UGV_MSGS__POSE_GUARD_HPP_
#define UAV_UGV_MSGS__POSE_GUARD_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace uav_ugv_msgs
{

struct PoseLimits
{
  double max_age{0.5};
  double max_position_stddev{0.5};
  double max_heading_stddev{0.2};
};

inline int64_t steady_time_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000 + stamp.nanosec;
}

inline double heading(const geometry_msgs::msg::Quaternion & q)
{
  const auto norm = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  return std::atan2(2 * (q.w * q.z + q.x * q.y), norm - 2 * (q.y * q.y + q.z * q.z));
}

// Call update and check from the same ROS callback group.
// Acquisition age uses ROS time; receipt age uses a clock that cannot stop with /clock.
class PoseGuard
{
public:
  explicit PoseGuard(PoseLimits limits = {})
  : limits_(limits)
  {
    for (const auto value : {limits.max_age, limits.max_position_stddev,
        limits.max_heading_stddev})
    {
      if (!std::isfinite(value) || value <= 0) {
        throw std::invalid_argument("Pose limits must be finite and positive");
      }
    }
  }

  void update(
    const geometry_msgs::msg::PoseWithCovarianceStamped & message,
    int64_t ros_now, int64_t steady_now)
  {
    observe_clock(ros_now);
    const auto stamp = stamp_ns(message.header.stamp);
    if (stamp <= last_stamp_) {
      reject("Nonmonotonic fused pose");
      return;
    }
    value_ = message;
    receipt_time_ = steady_now;
    reason_ = "Ready";
    if (check(ros_now, steady_now)) {last_stamp_ = stamp;}
  }

  bool check(int64_t ros_now, int64_t steady_now)
  {
    observe_clock(ros_now);
    if (!value_) {return false;}
    const auto & message = *value_;
    const auto & p = message.pose.pose.position;
    const auto & q = message.pose.pose.orientation;
    const auto & c = message.pose.covariance;
    const auto age = (ros_now - stamp_ns(message.header.stamp)) * 1e-9;
    const auto receipt_age = (steady_now - receipt_time_) * 1e-9;
    if (message.header.frame_id != "map") {return reject("Fused pose must use map");}
    if (stamp_ns(message.header.stamp) <= 0 || age < -0.05) {
      return reject("Invalid fused pose timestamp");
    }
    if (age > limits_.max_age || receipt_age > limits_.max_age || receipt_age < 0) {
      return reject("Fused pose is stale");
    }
    const auto norm = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(norm) || std::abs(norm - 1.0) > 0.01)
    {
      return reject("Invalid fused pose coordinates or orientation");
    }
    if (!std::all_of(c.begin(), c.end(), [](double v) {return std::isfinite(v);}) ||
      c[0] <= 0 || c[7] <= 0 || c[35] <= 0 || std::abs(c[1] - c[6]) > 1e-8 ||
      std::abs(c[5] - c[30]) > 1e-8 || std::abs(c[11] - c[31]) > 1e-8)
    {
      return reject("Invalid fused pose covariance");
    }
    const auto trace = c[0] + c[7];
    const auto spread = std::hypot(c[0] - c[7], 2 * c[1]);
    if (trace - spread <= 0) {return reject("Invalid planar covariance");}
    const auto determinant = c[0] * c[7] * c[35] + 2 * c[1] * c[5] * c[11] -
      c[0] * c[11] * c[11] - c[7] * c[5] * c[5] - c[35] * c[1] * c[1];
    if (determinant <= 0) {return reject("Invalid position-heading covariance");}
    if (std::sqrt((trace + spread) / 2) > limits_.max_position_stddev) {
      return reject("Position uncertainty exceeds the navigation limit");
    }
    if (std::sqrt(c[35]) > limits_.max_heading_stddev) {
      return reject("Heading uncertainty exceeds the navigation limit");
    }
    return true;
  }

  const geometry_msgs::msg::Pose & pose() const
  {
    if (!value_) {throw std::logic_error("Fused pose is unavailable");}
    return value_->pose.pose;
  }

  const char * reason() const {return reason_;}

private:
  bool reject(const char * reason)
  {
    value_.reset();
    reason_ = reason;
    return false;
  }

  void observe_clock(int64_t ros_now)
  {
    if (ros_now < last_clock_) {
      reject("ROS clock reset; waiting for a new fused pose");
      last_stamp_ = 0;
    }
    last_clock_ = ros_now;
  }

  PoseLimits limits_;
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> value_;
  int64_t last_clock_{0}, last_stamp_{0}, receipt_time_{0};
  const char * reason_{"Waiting for a fused pose"};
};

}  // namespace uav_ugv_msgs

#endif  // UAV_UGV_MSGS__POSE_GUARD_HPP_
