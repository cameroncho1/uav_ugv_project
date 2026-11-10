#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uav_control/follow_setpoint.hpp>
#include <uav_ugv_msgs/pose_guard.hpp>

using namespace std::chrono_literals;

class OffboardGoalController : public rclcpp::Node
{
public:
  OffboardGoalController()
  : Node("uav_goal_controller")
  {
    uav_altitude_ = declare_parameter("uav_altitude", 4.0);
    // Follow mode flies over the UGV's fused map pose. It reads the estimate only to
    // choose where to fly; the camera detector and PX4 anchor stay independent of it.
    follow_ugv_ = declare_parameter("follow_ugv", false);
    const auto origin = declare_parameter("map_origin", std::vector<double>{0.0, 0.0, 0.0});
    follow_speed_ = declare_parameter("follow_max_speed", 1.0);
    follow_deadband_ = declare_parameter("follow_deadband", 0.15);
    const auto follow_age = declare_parameter("follow_max_pose_age", 0.5);
    const auto follow_stddev = declare_parameter("follow_max_position_stddev", 0.5);
    if (origin.size() != 3 || !std::isfinite(origin[0]) || !std::isfinite(origin[1]) ||
      !std::isfinite(follow_speed_) || follow_speed_ <= 0 || follow_speed_ > 5 ||
      !std::isfinite(follow_deadband_) || follow_deadband_ < 0 || follow_deadband_ > 2)
    {
      throw std::invalid_argument("Invalid UAV follow parameters");
    }
    origin_x_ = origin[0];
    origin_y_ = origin[1];
    // Heading uncertainty does not matter for following, so only position is gated.
    ugv_guard_ = uav_ugv_msgs::PoseGuard({follow_age, follow_stddev, 1000.0});

    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    //offboard controller sends position set points
    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", px4_qos);

    //contains the desired UAV position and yaw
    trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", px4_qos);

    //requests UAV offboard mode or arming
    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", px4_qos);


    //receives the state of the preflight checks, offboard mode, and armed state  from px4 msg
    const auto status_version = px4_msgs::msg::VehicleStatus::MESSAGE_VERSION;
    const auto status_topic = std::string("/fmu/out/vehicle_status") +
      (status_version ? "_v" + std::to_string(status_version) : "");
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      status_topic, px4_qos,
      [this](
        px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        preflight_passed_ = msg->pre_flight_checks_pass;
        offboard_active_ = msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
        armed_ = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        have_vehicle_status_ = true; //just confirms that the px4 object has been created
      });

    // The first odometry sample fixes the takeoff hold; later samples track altitude.
    odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", px4_qos,
      [this](px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        current_down_ = msg->position[2];
        if (!have_position_) {
          hold_north_ = msg->position[0];
          hold_east_ = msg->position[1];
          command_ = {hold_north_, hold_east_};
          have_position_ = true;
          RCLCPP_INFO(
            get_logger(), "Holding initial horizontal position (N=%.2f, E=%.2f)",
            hold_north_, hold_east_);
        }
      });

    if (follow_ugv_) {
      ugv_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/ugv/localization/pose", 10,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          ugv_guard_.update(*msg, now().nanoseconds(), uav_ugv_msgs::steady_time_ns());
        });
    }

    timer_ = create_wall_timer(100ms, [this]() {control_tick();});
    RCLCPP_INFO(get_logger(), "Waiting for PX4 odometry and vehicle status; UAV will hold XY");
  }

private:
  void publish_vehicle_command(uint16_t command, float param1, float param2)
  {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = static_cast<uint64_t>(now().nanoseconds() / 1000);
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }

  void control_tick()
  {
    if (!have_position_) {
      return;
    }

    px4_msgs::msg::OffboardControlMode mode;
    mode.timestamp = static_cast<uint64_t>(now().nanoseconds() / 1000);
    mode.position = true; //controller sets position setpoints rather than velocity or acc
    offboard_mode_pub_->publish(mode);

    px4_msgs::msg::TrajectorySetpoint setpoint;
    setpoint.timestamp = mode.timestamp;
    update_follow_command();
    // ROS map is ENU; PX4 trajectory setpoints use NED.
    setpoint.position = {static_cast<float>(command_.north), static_cast<float>(command_.east),
      static_cast<float>(-uav_altitude_)};
    setpoint.yaw = 0.0F;
    trajectory_pub_->publish(setpoint);

    if (setpoint_count_ < 10) {
      ++setpoint_count_;
      return;
    }

    if (!have_vehicle_status_ || !preflight_passed_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Streaming setpoints; waiting for PX4 preflight checks to pass before Offboard/arm");
      return;
    }

    if (offboard_active_ && armed_) {
      if (!flight_ready_reported_) {
        RCLCPP_INFO(get_logger(), "PX4 confirmed Offboard mode and armed state");
        flight_ready_reported_ = true;
      }
      return;
    }

    if (++retry_counter_ >= 10) {
      retry_counter_ = 0;
      if (!offboard_active_) {
        publish_vehicle_command(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
        RCLCPP_INFO(get_logger(), "Requested PX4 Offboard mode; waiting for confirmation");
      } else if (!armed_) {
        publish_vehicle_command(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F, 0.0F);
        RCLCPP_INFO(get_logger(), "Requested PX4 arm; waiting for confirmation");
      }
    }
  }

  // Follow only once PX4 flies the setpoints near cruise height; otherwise hold.
  void update_follow_command()
  {
    if (!follow_ugv_) {return;}
    std::string status;
    if (!offboard_active_ || !armed_ || -current_down_ < 0.8 * uav_altitude_) {
      status = "Climbing before following the UGV";
    } else if (!ugv_guard_.check(now().nanoseconds(), uav_ugv_msgs::steady_time_ns())) {
      status = std::string("Holding position: ") + ugv_guard_.reason();
    } else {
      const auto & p = ugv_guard_.pose().position;
      command_ = uav_control::step_toward(
        command_, uav_control::map_to_ned(p.x, p.y, origin_x_, origin_y_),
        follow_speed_ * 0.1, follow_deadband_);
      status = "Following the UGV";
    }
    if (status != follow_status_) {
      RCLCPP_INFO(get_logger(), "%s", status.c_str());
      follow_status_ = status;
    }
  }

  double uav_altitude_{2.0};
  bool follow_ugv_{false};
  double follow_speed_{1.0}, follow_deadband_{0.15}, origin_x_{0.0}, origin_y_{0.0};
  float current_down_{0.0F};
  uav_control::NorthEast command_;
  uav_ugv_msgs::PoseGuard ugv_guard_;
  std::string follow_status_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ugv_pose_sub_;
  float hold_north_{0.0F};
  float hold_east_{0.0F};
  uint32_t setpoint_count_{0};
  uint32_t retry_counter_{0};
  bool have_position_{false};
  bool have_vehicle_status_{false};
  bool preflight_passed_{false};
  bool offboard_active_{false};
  bool armed_{false};
  bool flight_ready_reported_{false};
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardGoalController>());
  rclcpp::shutdown();
  return 0;
}
