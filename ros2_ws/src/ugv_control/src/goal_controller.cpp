#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <ugv_control/path_follower.hpp>
#include <ugv_control/supervisor_inputs.hpp>
#include <uav_ugv_msgs/msg/fault_status.hpp>
#include <uav_ugv_msgs/pose_guard.hpp>
#include <uav_ugv_msgs/grid_geometry.hpp>
#include <uav_ugv_msgs/route_contract.hpp>
#include <uav_ugv_msgs/evaluation_trace.hpp>
#include <std_msgs/msg/string.hpp>

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
    rcl_interfaces::msg::ParameterDescriptor startup_only;
    startup_only.read_only = true;
    startup_only.description = "Supervisor input configuration; restart to change.";
    supervisor_ = ugv_control::initial_supervisor(
      declare_parameter("detector_enabled", false, startup_only));
    const auto fault_topic = declare_parameter<std::string>(
      "fault_status_topic", "/fault_detection/status", startup_only);
    if (fault_topic.empty()) {throw std::invalid_argument("fault_status_topic must be nonempty");}
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/ugv/localization/pose", 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        observe_clock(now().nanoseconds());
        pose_receipt_ = uav_ugv_msgs::steady_time_ns();
        pose_stamp_ = uav_ugv_msgs::stamp_ns(msg->header.stamp);
        pose_guard_.update(*msg, now().nanoseconds(), pose_receipt_);
      });
    path_sub_ = create_subscription<uav_ugv_msgs::msg::PlannedRoute>(
      "/ugv/navigation/route", latched, [this](uav_ugv_msgs::msg::PlannedRoute::SharedPtr msg) {
        observe_clock(now().nanoseconds());
        route_ = *msg;
        path_ = msg->path;
        path_receipt_ = uav_ugv_msgs::steady_time_ns();
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/global_goal_pose", latched, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        const auto ros_now = now().nanoseconds();
        observe_clock(ros_now);
        const auto accepted = goals_.receive(uav_ugv_msgs::valid_goal(*msg),
          uav_ugv_msgs::stamp_ns(msg->header.stamp), msg->pose.position.x,
          msg->pose.position.y, ros_now);
        if (accepted) {goal_ = *msg;}
        if (!accepted || goals_.new_goal_pending()) {invalidate_route();}
      });
    if (supervisor_.detector_enabled) {
      fault_sub_ = create_subscription<uav_ugv_msgs::msg::FaultStatus>(
        fault_topic,
        rclcpp::SensorDataQoS().keep_last(20),
        [this](uav_ugv_msgs::msg::FaultStatus::SharedPtr msg) {
          const auto ros_now = now().nanoseconds();
          const auto steady_now = uav_ugv_msgs::steady_time_ns();
          observe_clock(ros_now);
          const auto pose_valid = pose_guard_.check(ros_now, steady_now);
          auto input = supervisor_input(ros_now, steady_now, pose_valid);
          const ugv_control::FaultDecision decision{msg->schema_version, msg->source_id,
            msg->sequence, uav_ugv_msgs::stamp_ns(msg->header.stamp), msg->window_valid,
            msg->gps_outage, msg->odometry_scale, msg->imu_drift, msg->uav_loss};
          input.fault = fault_input_.receive(decision, ros_now, steady_now, pose_valid);
          input.recovery_mature = supervisor_.fault_latched && supervisor_.clear_count > 0 &&
            fault_input_.recovery_mature(ros_now);
          // Process every received decision, including alarm/clear bursts between ticks.
          supervisor_ = ugv_control::supervise(supervisor_, input);
        });
    }
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>("/ugv/cmd_vel", 10);
    if (declare_parameter<bool>("evaluation_trace", false)) {
      trace_pub_ = create_publisher<std_msgs::msg::String>("/evaluation/control", 50);
    }
    timer_ = create_wall_timer(100ms, [this]() {control_tick();});
  }

private:
  void invalidate_route()
  {
    path_ = nav_msgs::msg::Path();
    route_ = uav_ugv_msgs::msg::PlannedRoute();
  }

  void observe_clock(int64_t ros_now)
  {
    if (ros_now < last_ros_clock_) {
      goals_.reset_epoch();
      fault_input_.reset();
      invalidate_route();
      ugv_control::SupervisorInput reset;
      reset.clock_reset = true;
      supervisor_ = ugv_control::supervise(supervisor_, reset);
    }
    last_ros_clock_ = ros_now;
  }

  ugv_control::SupervisorInput supervisor_input(int64_t ros_now, int64_t steady_now,
    bool pose_valid)
  {
    ugv_control::SupervisorInput input;
    input.have_goal = goals_.available();
    // The first decision for a revision consumes it, whether callback or timer.
    input.new_goal = input.have_goal && goals_.take_new_goal();
    input.pose_valid = pose_valid;
    input.route_valid = input.have_goal && valid_path(ros_now, steady_now);
    if (input.have_goal && pose_valid) {
      const auto & pose = pose_guard_.pose();
      input.goal_reached = std::hypot(goal_.pose.position.x - pose.position.x,
          goal_.pose.position.y - pose.position.y) <= goal_tolerance_;
    }
    return input;
  }

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
    if (!uav_ugv_msgs::route_matches_goal(route_, goal_) ||
      !uav_ugv_msgs::valid_grid_geometry(route_.clearance_grid)) {return false;}
    const auto grid_age = (ros_now - uav_ugv_msgs::stamp_ns(route_.clearance_grid.header.stamp)) *
      1e-9;
    if (grid_age < -0.05 || grid_age > 2.0) {return false;}
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
    for (std::size_t i = 1; i < path_.poses.size(); ++i) {
      const auto & a = path_.poses[i - 1].pose.position;
      const auto & b = path_.poses[i].pose.position;
      if (!uav_ugv_msgs::clear_segment(route_.clearance_grid, a.x, a.y, b.x, b.y)) {return false;}
    }
    return true;
  }

  void control_tick()
  {
    geometry_msgs::msg::Twist command;
    const auto ros_now = now().nanoseconds(), steady_now = uav_ugv_msgs::steady_time_ns();
    observe_clock(ros_now);
    std::string status;
    const bool guard_valid = pose_guard_.check(ros_now, steady_now);
    const std::string guard_reason = pose_guard_.reason();
    auto input = supervisor_input(ros_now, steady_now, guard_valid);
    if (supervisor_.detector_enabled) {
      input.fault = fault_input_.poll(ros_now, steady_now, guard_valid);
      input.recovery_mature = supervisor_.fault_latched && supervisor_.clear_count > 0 &&
            fault_input_.recovery_mature(ros_now);
    }
    supervisor_ = ugv_control::supervise(supervisor_, input);
    switch (supervisor_.mode) {
      case ugv_control::Mode::Init: status = "Waiting for a map goal"; break;
      case ugv_control::Mode::HoldFault:
        status = "Fault hold: waiting for fresh clear detector decisions"; break;
      case ugv_control::Mode::HoldGuard:
        status = guard_valid ? "Waiting for a fresh map path" : guard_reason; break;
      case ugv_control::Mode::Goal: status = "UGV reached global goal tolerance"; break;
      case ugv_control::Mode::Navigate:
        status = "Following path using the fused pose";
        command = ugv_control::follow_path(pose_guard_.pose(), path_, follower_, route_.clearance_grid);
        break;
    }
    // Publish zero every tick during a hold, including when /clock stops.
    command_pub_->publish(command);
    const auto command_steady_ns = uav_ugv_msgs::trace_steady_ns();
    if (trace_pub_) {
      if (ros_now < last_trace_sim_) {++trace_epoch_;}
      last_trace_sim_ = ros_now;
      uav_ugv_msgs::EvaluationJson trace;
      trace.integer("schema_version", 1).text("kind", "control_tick")
        .text("host_id", uav_ugv_msgs::trace_host_id()).integer("sequence", trace_sequence_++)
        .integer("steady_ns", steady_now).integer("sim_ns", ros_now).integer("epoch", trace_epoch_)
        .integer("command_steady_ns", command_steady_ns)
        .integer("pose_stamp_ns", pose_stamp_).integer("pose_receipt_ns", pose_receipt_)
        .boolean("guard_valid", guard_valid).text("guard_reason", guard_reason)
        .text("hold_reason", status).integer("goal_revision", uav_ugv_msgs::stamp_ns(goal_.header.stamp))
        .text("supervisor_state", ugv_control::mode_name(supervisor_.mode))
        .text("supervisor_policy", ugv_control::kSupervisorPolicySha256)
        .boolean("detector_enabled", supervisor_.detector_enabled)
        .boolean("fault_latched", supervisor_.fault_latched)
        .integer("clear_windows", supervisor_.clear_count)
        .integer("recovery_age", static_cast<int>(supervisor_.recovery_age))
        .boolean("goal_completed", supervisor_.goal_completed)
        .boolean("have_goal", input.have_goal).boolean("new_goal", input.new_goal)
        .boolean("route_valid", input.route_valid).boolean("goal_reached", input.goal_reached)
        .boolean("recovery_mature", input.recovery_mature)
        .integer("fault_event", static_cast<int>(input.fault))
        .integer("fault_stamp_ns", fault_input_.latest_stamp())
        .text("fault_sequence", std::to_string(fault_input_.latest_sequence()))
        .text("fault_source_id", fault_input_.latest_source())
        .boolean("fault_decision_valid", fault_input_.decision_valid())
        .integer("route_stamp_ns", uav_ugv_msgs::stamp_ns(route_.header.stamp))
        .integer("route_goal_revision", uav_ugv_msgs::stamp_ns(route_.goal.header.stamp))
        .integer("route_receipt_ns", path_receipt_)
        .number("linear", command.linear.x).number("angular", command.angular.z);
      std_msgs::msg::String message;
      message.data = trace.str();
      trace_pub_->publish(message);
    }
    if (status != status_) {
      RCLCPP_INFO(get_logger(), "%s", status.c_str());
      status_ = status;
    }
  }

  uav_ugv_msgs::PoseGuard pose_guard_;
  ugv_control::FollowerLimits follower_;
  double goal_tolerance_{0.5}, max_path_age_{1.0};
  int64_t path_receipt_{0};
  int64_t pose_receipt_{0}, pose_stamp_{0}, last_trace_sim_{0}, trace_epoch_{0}, trace_sequence_{0};
  int64_t last_ros_clock_{0};
  ugv_control::SupervisorState supervisor_;
  ugv_control::FaultInput fault_input_;
  ugv_control::GoalHistory goals_;
  std::string status_;
  nav_msgs::msg::Path path_;
  uav_ugv_msgs::msg::PlannedRoute route_;
  geometry_msgs::msg::PoseStamped goal_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<uav_ugv_msgs::msg::PlannedRoute>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<uav_ugv_msgs::msg::FaultStatus>::SharedPtr fault_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trace_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UgvGoalController>());
  rclcpp::shutdown();
  return 0;
}
