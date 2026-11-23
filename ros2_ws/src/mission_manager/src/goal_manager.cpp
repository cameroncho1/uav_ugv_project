#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

class GoalManager : public rclcpp::Node
{
public:
  GoalManager()
  : Node("goal_manager")
  {
    //Takes in a goal_x and goal_y but defaults to (4,0)
    goal_x_ = declare_parameter("goal_x", 4.0);
    goal_y_ = declare_parameter("goal_y", 0.0);

    //publishes goal pose data which is used by A* and UGV control
    auto goal_qos = rclcpp::QoS(1).reliable().transient_local();
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/global_goal_pose", goal_qos);

    //publishes marker visualization in the global grid for Rviz2
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/global_goal", 1);
    timer_ = create_wall_timer(1s, [this]() {publish_goal();});

    RCLCPP_INFO(get_logger(), "Global goal in map frame: (%.1f, %.1f)", goal_x_, goal_y_);

    // A new goal is a new revision: routes and completion belong to the revision that
    // produced them. Both coordinates must be set in one request.
    goal_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        double x = goal_x_, y = goal_y_;
        bool changed = false;
        for (const auto & parameter : parameters) {
          if (parameter.get_name() != "goal_x" && parameter.get_name() != "goal_y") {continue;}
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
            !std::isfinite(parameter.as_double()) || std::abs(parameter.as_double()) > 100.0)
          {
            result.successful = false;
            result.reason = parameter.get_name() + " must be a finite number within 100 m";
            return result;
          }
          (parameter.get_name() == "goal_x" ? x : y) = parameter.as_double();
          changed = true;
        }
        if (changed && (x != goal_x_ || y != goal_y_)) {
          goal_x_ = x;
          goal_y_ = y;
          revision_ = rclcpp::Time(0, 0, RCL_ROS_TIME);  // restamped by the next publish
          RCLCPP_INFO(get_logger(), "New global goal in map frame: (%.1f, %.1f)", goal_x_, goal_y_);
          publish_goal();
        }
        return result;
      });
  }

private:
  void publish_goal()
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    const auto current = now();
    if (current.nanoseconds() <= 0) {return;}
    if (revision_.nanoseconds() == 0 || current < last_clock_) {revision_ = current;}
    last_clock_ = current;
    goal.header.stamp = revision_;
    goal.pose.position.x = goal_x_;
    goal.pose.position.y = goal_y_;
    goal.pose.orientation.w = 1.0;
    goal_pub_->publish(goal);

    visualization_msgs::msg::Marker marker;
    marker.header = goal.header;
    marker.ns = "global_goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = goal_x_;
    marker.pose.position.y = goal_y_;
    marker.pose.position.z = 0.15;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.3;
    marker.color.r = 0.1F;
    marker.color.g = 0.85F;
    marker.color.b = 0.25F;
    marker.color.a = 1.0F;
    marker_pub_->publish(marker);
  }

  double goal_x_{0.0};
  double goal_y_{0.0};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr goal_callback_;
  rclcpp::Time revision_{0, 0, RCL_ROS_TIME}, last_clock_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalManager>());
  rclcpp::shutdown();
  return 0;
}
