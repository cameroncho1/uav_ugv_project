#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/transport/Node.hh>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class UgvGoalController : public rclcpp::Node {
public:
	UgvGoalController() : Node("ugv_goal_controller") {
		lookahead_distance_ = declare_parameter("lookahead_distance", 0.6);
		goal_tolerance_ = declare_parameter("goal_tolerance", 0.5);
		//translates the ugv position on the grid
		odom_to_map_x_ = declare_parameter("odom_to_map_x", -4.0);
		odom_to_map_y_ = declare_parameter("odom_to_map_y", 0.0);


		//subscribes to local_path which is the path generated from the A* planner
		auto path_qos = rclcpp::QoS(1).reliable().transient_local();
		path_sub_ = create_subscription<nav_msgs::msg::Path>(
			"/local_path", path_qos,
			[this](nav_msgs::msg::Path::SharedPtr msg) {
				path_ = *msg;
				have_path_ = !msg->poses.empty();
			});

		//stores the global pose (not really used yet)
		goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
			"/global_goal_pose", path_qos,
			[this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
				goal_x_ = msg->pose.position.x;
				goal_y_ = msg->pose.position.y;
				have_goal_ = true;
			});

		//publishes pose for other ros nodes to use (e.g. mapper, planner, visualizer)
		pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/ugv_pose", 10);

		//publishes velocity commands to Gazebo to simulate the UGV movement
		gz_cmd_pub_ = gz_node_.Advertise<gz::msgs::Twist>("/simple_ugv/cmd_vel");

		//receives UGV odometry from Gazebo
		gz_odom_subscribed_ = gz_node_.Subscribe(
			"/simple_ugv/odom", &UgvGoalController::ugv_odom_cb, this);
		if (!gz_odom_subscribed_) {
			RCLCPP_ERROR(get_logger(), "Could not subscribe to Gazebo UGV odometry");
		}

		timer_ = create_wall_timer(100ms, [this]() { control_tick(); });
		RCLCPP_INFO(get_logger(), "Waiting for a local A-star path and Gazebo UGV odometry");
	}

private:
	void ugv_odom_cb(const gz::msgs::Odometry &msg) {
		const auto &position = msg.pose().position();
		const auto &orientation = msg.pose().orientation(); //quaternion: [qx,qy,qz,qw]

		const double sin_yaw = 2.0 * (orientation.w() * orientation.z() +
			orientation.x() * orientation.y());
		const double cos_yaw = 1.0 - 2.0 * (orientation.y() * orientation.y() +
			orientation.z() * orientation.z());

		//need a lock because we might read while Gazebo updates the state
		std::lock_guard<std::mutex> lock(state_mutex_);
		ugv_x_ = position.x() + odom_to_map_x_;
		ugv_y_ = position.y() + odom_to_map_y_;
		ugv_yaw_ = std::atan2(sin_yaw, cos_yaw);
		have_odom_ = true;

		//publish a standard ros2 pose message
		geometry_msgs::msg::PoseStamped pose;
		pose.header.stamp = now();
		pose.header.frame_id = "map";
		pose.pose.position.x = ugv_x_;
		pose.pose.position.y = ugv_y_;
		pose.pose.position.z = position.z();
		pose.pose.orientation.x = orientation.x();
		pose.pose.orientation.y = orientation.y();
		pose.pose.orientation.z = orientation.z();
		pose.pose.orientation.w = orientation.w();
		pose_pub_->publish(pose);
	}

	void control_tick() {
		double ugv_x;
		double ugv_y;
		double ugv_yaw;
		bool have_odom;
		bool have_goal = have_goal_;
		double goal_x = goal_x_;
		double goal_y = goal_y_;

		//use lock to guarantee a consistent snapshot state
		{
			std::lock_guard<std::mutex> lock(state_mutex_);
			ugv_x = ugv_x_;
			ugv_y = ugv_y_;
			ugv_yaw = ugv_yaw_;
			have_odom = have_odom_;
		}
		if (have_odom && have_goal && std::hypot(goal_x - ugv_x, goal_y - ugv_y) <= goal_tolerance_) {
			if (!goal_reported_) {
				RCLCPP_INFO(get_logger(), "UGV reached global goal tolerance");
				goal_reported_ = true;
			}
			gz_cmd_pub_.Publish(gz::msgs::Twist());
			return;
		}
		goal_reported_ = false;

		if (!have_path_ || !have_odom) {
			if (!waiting_reported_) {
				if (!have_path_ && !have_odom) {
					RCLCPP_WARN(get_logger(), "Waiting for local path and Gazebo UGV odometry");
				} else if (!have_path_) {
					RCLCPP_WARN(get_logger(), "Waiting for local A-star path");
				} else {
					RCLCPP_WARN(get_logger(), "Waiting for Gazebo UGV odometry");
				}
				waiting_reported_ = true;
				gz_cmd_pub_.Publish(gz::msgs::Twist());
			}
			return;
		}
		waiting_reported_ = false;


		//following code is generic path finding that we will probably change later

		//find nearest point in the path generated from A*
		std::size_t nearest_index = 0;
		double nearest_distance = std::numeric_limits<double>::infinity();
		for (std::size_t index = 0; index < path_.poses.size(); ++index) {
			const auto &position = path_.poses[index].pose.position;
			const double distance = std::hypot(position.x - ugv_x, position.y - ugv_y);
			if (distance < nearest_distance) {
				nearest_distance = distance;
				nearest_index = index;
			}
		}

		//select a lookahead point and interpolate for a smoother path
		std::size_t target_index = path_.poses.size() - 1;
		for (std::size_t index = nearest_index; index < path_.poses.size(); ++index) {
			const auto &position = path_.poses[index].pose.position;
			if (std::hypot(position.x - ugv_x, position.y - ugv_y) >= lookahead_distance_) {
				target_index = index;
				break;
			}
		}

		//calculate target distance relative to UGV
		const auto &target = path_.poses[target_index].pose.position;
		const double dx = target.x - ugv_x;
		const double dy = target.y - ugv_y;
		const double distance = std::hypot(dx, dy);
		gz::msgs::Twist command;

		//calculate linear and angular velocity (again some generic controller that we can implement later)
		if (nearest_index + 1 < path_.poses.size() || nearest_distance > lookahead_distance_) {
			const double desired_yaw = std::atan2(dy, dx);
			const double yaw_error = std::atan2(std::sin(desired_yaw - ugv_yaw),std::cos(desired_yaw - ugv_yaw));
			command.mutable_linear()->set_x(std::min(0.7, 0.5 * distance) * std::max(0.0, std::cos(yaw_error)));
			command.mutable_angular()->set_z(std::clamp(1.5 * yaw_error, -1.0, 1.0));
		}

		gz_cmd_pub_.Publish(command);
	}

	double lookahead_distance_{0.6};
	double goal_tolerance_{0.5};
	double odom_to_map_x_{-4.0};
	double odom_to_map_y_{0.0};
	double ugv_x_{0.0};
	double ugv_y_{0.0};
	double ugv_yaw_{0.0};
	double goal_x_{0.0};
	double goal_y_{0.0};
	bool have_path_{false};
	bool have_goal_{false};
	bool have_odom_{false};
	bool waiting_reported_{false};
	bool goal_reported_{false};
	bool gz_odom_subscribed_{false};
	std::mutex state_mutex_;
	gz::transport::Node gz_node_;
	gz::transport::Node::Publisher gz_cmd_pub_;
	nav_msgs::msg::Path path_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UgvGoalController>());
	rclcpp::shutdown();
	return 0;
}