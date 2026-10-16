#include <memory>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

// Simulator boundary: pair true vehicle poses at one simulation timestamp.
// Only the noisy sensor simulator consumes this topic.
class SimVehiclePoses : public rclcpp::Node
{
public:
  SimVehiclePoses()
  : Node("sim_vehicle_poses")
  {
    const auto world = declare_parameter<std::string>("world", "uav_ugv");
    ugv_ = declare_parameter<std::string>("ugv_model", "simple_ugv");
    uav_ = declare_parameter<std::string>("uav_model", "");
    const auto position =
      declare_parameter<std::vector<double>>("fixed_uav_position", {2., -2., 4.});
    if (position.size() != 3) {throw std::invalid_argument("fixed_uav_position needs xyz");}
    for (const auto value : position) {
      if (!std::isfinite(value)) {throw std::invalid_argument("fixed_uav_position must be finite");}
    }
    fixed_.position.x = position[0];
    fixed_.position.y = position[1];
    fixed_.position.z = position[2];
    fixed_.orientation.w = 1.;
    const auto frequency = declare_parameter<double>("publish_frequency", 20.);
    if (!std::isfinite(frequency) || frequency <= 0) {
      throw std::invalid_argument("publish_frequency must be positive");
    }
    period_ = 1. / frequency;
    publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/simulation/vehicle_poses", rclcpp::SensorDataQoS());
    const auto topic = "/world/" + world + "/dynamic_pose/info";
    if (!transport_.Subscribe(topic, &SimVehiclePoses::on_poses, this)) {
      throw std::runtime_error("Cannot subscribe to " + topic);
    }
  }

private:
  static geometry_msgs::msg::Pose convert(const gz::msgs::Pose & pose)
  {
    geometry_msgs::msg::Pose output;
    output.position.x = pose.position().x();
    output.position.y = pose.position().y();
    output.position.z = pose.position().z();
    output.orientation.w = pose.orientation().w();
    output.orientation.x = pose.orientation().x();
    output.orientation.y = pose.orientation().y();
    output.orientation.z = pose.orientation().z();
    return output;
  }

  void on_poses(const gz::msgs::Pose_V & poses)
  {
    const double stamp = poses.header().stamp().sec() + poses.header().stamp().nsec() * 1e-9;
    if (stamp >= last_stamp_ && stamp - last_stamp_ < period_) {return;}
    geometry_msgs::msg::PoseArray output;
    output.poses.resize(2);
    output.poses[0] = fixed_;
    bool have_uav = uav_.empty();
    bool have_ugv = false;
    for (const auto & pose : poses.pose()) {
      if (pose.name() == ugv_) {output.poses[1] = convert(pose); have_ugv = true;}
      if (pose.name() == uav_) {output.poses[0] = convert(pose); have_uav = true;}
    }
    if (!have_uav || !have_ugv) {return;}
    last_stamp_ = stamp;
    output.header.frame_id = "map";
    output.header.stamp.sec = poses.header().stamp().sec();
    output.header.stamp.nanosec = poses.header().stamp().nsec();
    publisher_->publish(output);
  }

  std::string uav_, ugv_;
  double period_, last_stamp_{-1.};
  geometry_msgs::msg::Pose fixed_;
  gz::transport::Node transport_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimVehiclePoses>());
  rclcpp::shutdown();
  return 0;
}
