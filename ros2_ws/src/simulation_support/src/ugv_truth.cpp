#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <uav_ugv_msgs/evaluation_trace.hpp>
#include <cmath>

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
    if (declare_parameter<bool>("evaluation_trace", false)) {
      trace_pub_ = create_publisher<std_msgs::msg::String>("/evaluation/truth", 50);
    }
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
      if (trace_pub_) {
        const int64_t sim_ns = int64_t(output.header.stamp.sec) * 1000000000 +
          output.header.stamp.nanosec;
        const int64_t steady_ns = uav_ugv_msgs::trace_steady_ns();
        if (sim_ns < previous_sim_ns_) {++epoch_;}
        const auto & p = output.pose.position;
        const auto & q = output.pose.orientation;
        const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y),
          1 - 2 * (q.y * q.y + q.z * q.z));
        const bool advancing = previous_sim_ns_ > 0 && sim_ns > previous_sim_ns_;
        const double dt = advancing ? (sim_ns - previous_sim_ns_) * 1e-9 : 1.0;
        const double speed = advancing ? std::hypot(p.x - previous_x_, p.y - previous_y_) / dt : 0.0;
        const double yaw_rate = advancing ? std::atan2(std::sin(yaw - previous_yaw_),
          std::cos(yaw - previous_yaw_)) / dt : 0.0;
        uav_ugv_msgs::EvaluationJson trace;
        trace.integer("schema_version", 1).text("kind", "truth")
          .text("host_id", uav_ugv_msgs::trace_host_id()).integer("sequence", sequence_++)
          .integer("sim_ns", sim_ns).integer("steady_ns", steady_ns).integer("epoch", epoch_)
          .number("x", p.x).number("y", p.y).number("z", p.z).number("yaw", yaw)
          .number("linear", speed).number("angular", yaw_rate)
          .boolean("simulation_advancing", advancing)
          .integer("uncertainty_ns", previous_steady_ns_ ? steady_ns - previous_steady_ns_ : 0);
        std_msgs::msg::String message;
        message.data = trace.str();
        trace_pub_->publish(message);
        previous_sim_ns_ = sim_ns;
        previous_steady_ns_ = steady_ns;
        previous_x_ = p.x; previous_y_ = p.y; previous_yaw_ = yaw;
      }
      return;
    }
  }

  std::string model_;
  gz::transport::Node transport_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trace_pub_;
  int64_t previous_sim_ns_{0}, previous_steady_ns_{0}, epoch_{0}, sequence_{0};
  double previous_x_{0.0}, previous_y_{0.0}, previous_yaw_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UgvTruth>());
  rclcpp::shutdown();
  return 0;
}
