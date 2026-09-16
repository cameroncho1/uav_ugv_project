#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

class OdometryListener : public rclcpp::Node
{
public:
    OdometryListener()
        : Node("odometry_listener")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.best_effort();

        subscription_ =
            this->create_subscription<px4_msgs::msg::VehicleOdometry>(
                "/fmu/out/vehicle_odometry",
                qos,
                [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
                {
                    message_count_++;

                    // Avoid flooding the terminal.
                    if (message_count_ % 50 == 0)
                    {
                        RCLCPP_INFO(
                            this->get_logger(),
                            "PX4 position (NED): x=%.3f, y=%.3f, z=%.3f m",
                            msg->position[0],
                            msg->position[1],
                            msg->position[2]);
                    }
                });
                
        RCLCPP_INFO(
            this->get_logger(),
            "Listening to /fmu/out/vehicle_odometry");
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
    std::size_t message_count_{0};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<OdometryListener>());

    rclcpp::shutdown();
    return 0;
}
