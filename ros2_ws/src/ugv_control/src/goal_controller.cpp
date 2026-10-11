#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ugv_control/path_follower.hpp>
#include <uav_ugv_msgs/pose_guard.hpp>

using namespace std::chrono_literals;

class UgvGoalController : public rclcpp::Node
{
public:
  UgvGoalController()
  : Node("ugv_goal_controller"), pose_guard_(pose_limits())
  {
    follower_.lookahead = positive_parameter("lookahead_distance", 0.6);
    follower_.speed = positive_parameter("max_linear_speed", 0.5);
    follower_.turn_rate = positive_parameter("max_angular_speed", 1.0);
    goal_tolerance_ = positive_parameter("goal_tolerance", 0.5);
    max_path_age_ = positive_parameter("max_path_age", 1.0);
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/ugv/localization/pose", 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        pose_guard_.update(*msg, now().nanoseconds(), uav_ugv_msgs::steady_time_ns());
      });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/local_path", latched, [this](nav_msgs::msg::Path::SharedPtr msg) {
        path_ = *msg;
        path_receipt_ = uav_ugv_msgs::steady_time_ns();
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/global_goal_pose", latched, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        const auto changed = msg->pose.position.x != goal_.pose.position.x ||
        msg->pose.position.y != goal_.pose.position.y ||
        msg->header.frame_id != goal_.header.frame_id;
        goal_ = *msg;
        have_goal_ = msg->header.frame_id == "map" &&
        std::isfinite(msg->pose.position.x) && std::isfinite(msg->pose.position.y);
        // Require a route published after this goal change.
        if (changed) {path_ = nav_msgs::msg::Path();}
      });
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>("/ugv/cmd_vel", 10);
    timer_ = create_wall_timer(100ms, [this]() {control_tick();});
  }

private:
  double positive_parameter(const std::string & name, double fallback)
  {
    const auto value = declare_parameter(name, fallback);
    if (!std::isfinite(value) || value <= 0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return value;
  }

  uav_ugv_msgs::PoseLimits pose_limits()
  {
    return {positive_parameter("max_pose_age", 0.5),
      positive_parameter("max_position_stddev", 0.5),
      positive_parameter("max_heading_stddev", 0.2)};
  }

  bool valid_path(int64_t ros_now, int64_t steady_now) const
  {
    const auto stamp = uav_ugv_msgs::stamp_ns(path_.header.stamp);
    const auto age = (ros_now - stamp) * 1e-9;
    if (path_.header.frame_id != "map" || path_.poses.empty() || stamp <= 0 ||
      age < -0.05 || age > max_path_age_ ||
      (steady_now - path_receipt_) * 1e-9 > max_path_age_)
    {
      return false;
    }
    for (const auto & point : path_.poses) {
      if (point.header.frame_id != "map" || !std::isfinite(point.pose.position.x) ||
        !std::isfinite(point.pose.position.y)) {return false;}
    }
    return true;
  }

  void control_tick()
  {
    geometry_msgs::msg::Twist command;
    const auto ros_now = now().nanoseconds(), steady_now = uav_ugv_msgs::steady_time_ns();
    std::string status;
    if (!pose_guard_.check(ros_now, steady_now)) {
      status = pose_guard_.reason();
    } else if (!have_goal_) {
      status = "Waiting for a map goal";
    } else {
      const auto & pose = pose_guard_.pose();
      const auto distance = std::hypot(
        goal_.pose.position.x - pose.position.x,
        goal_.pose.position.y - pose.position.y);
      if (distance <= goal_tolerance_) {
        status = "UGV reached global goal tolerance";
      } else if (!valid_path(ros_now, steady_now)) {
        status = "Waiting for a fresh map path";
      } else {
        status = "Following path using the fused pose";
        command = ugv_control::follow_path(pose, path_, follower_);
      }
    }
    // Publish zero every tick during a hold, including when /clock stops.
    command_pub_->publish(command);
    if (status != status_) {
      RCLCPP_INFO(get_logger(), "%s", status.c_str());
      status_ = status;
    }
  }

  uav_ugv_msgs::PoseGuard pose_guard_;
  ugv_control::FollowerLimits follower_;
  double goal_tolerance_{0.5}, max_path_age_{1.0};
  int64_t path_receipt_{0};
  bool have_goal_{false};
  std::string status_;
  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped goal_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UgvGoalController>());
  rclcpp::shutdown();
  return 0;
}
