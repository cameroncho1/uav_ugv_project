#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

// Evaluation only: publish Gazebo's true model pose without exposing it to the EKF.
class UgvTruth : public rclcpp::Node
{
public:
  UgvTruth()
  : Node("ugv_truth")
  {
    const auto world = declare_parameter<std::string>("world", "uav_ugv");
    const auto model = declare_parameter<std::string>("model", "simple_ugv");
    model_ = model;
    publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/evaluation/ugv_truth", rclcpp::SensorDataQoS());
    const auto topic = "/world/" + world + "/dynamic_pose/info";
    if (!transport_.Subscribe(topic, &UgvTruth::on_pose, this)) {
      throw std::runtime_error("Cannot subscribe to " + topic);
    }
  }

private:
  void on_pose(const gz::msgs::Pose_V & poses)
  {
    for (const auto & pose : poses.pose()) {
      if (pose.name() != model_) {continue;}
      geometry_msgs::msg::PoseStamped output;
      output.header.frame_id = "map";
      output.header.stamp.sec = poses.header().stamp().sec();
      output.header.stamp.nanosec = poses.header().stamp().nsec();
      output.pose.position.x = pose.position().x();
      output.pose.position.y = pose.position().y();
      output.pose.position.z = pose.position().z();
      output.pose.orientation.x = pose.orientation().x();
      output.pose.orientation.y = pose.orientation().y();
      output.pose.orientation.z = pose.orientation().z();
      output.pose.orientation.w = pose.orientation().w();
      publisher_->publish(output);
      return;
    }
  }

  std::string model_;
  gz::transport::Node transport_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UgvTruth>());
  rclcpp::shutdown();
  return 0;
}
