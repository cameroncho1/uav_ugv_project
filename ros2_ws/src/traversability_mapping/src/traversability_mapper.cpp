#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class TraversabilityMapper : public rclcpp::Node {
public:
  TraversabilityMapper() : Node("traversability_mapper") {
    camera_topic_ = declare_parameter<std::string>(
      "camera_topic",
      "/world/simple_goal_demo/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image");
    camera_hfov_ = declare_parameter("camera_horizontal_fov", 1.74);
    grid_resolution_ = declare_parameter("grid_resolution", 0.25);
    obstacle_luminance_threshold_ = declare_parameter("obstacle_luminance_threshold", 32.0);

    //publishes a 2D occupancy local grid that is used by the local A* path planner
    auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/local_grid", map_qos);

    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", px4_qos,
      [this](px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        camera_east_ = msg->position[1];
        camera_north_ = msg->position[0];
        camera_altitude_ = std::max(0.0, -static_cast<double>(msg->position[2]));
        have_pose_ = true;
      });

    if (!gz_node_.Subscribe(camera_topic_, &TraversabilityMapper::image_cb, this)) {
      RCLCPP_ERROR(get_logger(), "Could not subscribe to Gazebo camera topic: %s", camera_topic_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Mapping Gazebo camera topic: %s", camera_topic_.c_str());
    }
  }

private:
  static bool image_format_channels(gz::msgs::PixelFormatType format, uint32_t &channels,
    bool &bgr) {
    bgr = false;
    switch (format) {
      case gz::msgs::RGB_INT8:
        channels = 3;
        return true;
      case gz::msgs::BGR_INT8:
        channels = 3;
        bgr = true;
        return true;
      case gz::msgs::RGBA_INT8:
      case gz::msgs::BGRA_INT8:
        channels = 4;
        bgr = format == gz::msgs::BGRA_INT8;
        return true;
      case gz::msgs::L_INT8:
        channels = 1;
        return true;
      default:
        return false;
    }
  }

  //calculates the brightness of the pixels which is used to determine whether an obstacle or not (refine later)
  double luminance_at(const std::string &data, std::size_t index, uint32_t channels, bool bgr) const {
    const auto channel = [&data](std::size_t i) {
      return static_cast<double>(static_cast<uint8_t>(data[i]));
    };
    if (channels == 1) {
      return channel(index);
    }
    const double red = channel(index + (bgr ? 2 : 0));
    const double green = channel(index + 1);
    const double blue = channel(index + (bgr ? 0 : 2));
    return 0.299 * red + 0.587 * green + 0.114 * blue;
  }

  //runs whenever Gazebo publishes an image from the UAV's camera
  //converts image to a 2D occupancy local grid
  void image_cb(const gz::msgs::Image &image) {
    if (image.width() == 0 || image.height() == 0) {
      return;
    }

    uint32_t channels = 0;
    bool bgr = false;
    if (!image_format_channels(image.pixel_format_type(), channels, bgr)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Unsupported Gazebo camera pixel format: %d",
        static_cast<int>(image.pixel_format_type()));
      return;
    }

    const uint32_t width = image.width();
    const uint32_t height = image.height();
    const uint32_t step = image.step() == 0 ? width * channels : image.step();
    const std::string &data = image.data();
    if (data.size() < static_cast<std::size_t>(step) * height) {
      return;
    }

    double east;
    double north;
    double altitude;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      if (!have_pose_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for PX4 camera pose");
        return;
      }
      east = camera_east_;
      north = camera_north_;
      altitude = camera_altitude_;
    }
    if (altitude < 0.5) {
      return;
    }

    // calculate camera footprint for a level, downward-facing camera using the pinhole-camera model.
    // the demo UAV holds yaw at zero
    const double half_width = altitude * std::tan(camera_hfov_ / 2.0);
    const double half_height = half_width * static_cast<double>(height) / width;
    const uint32_t cells_x = std::max<uint32_t>(1, static_cast<uint32_t>(
      std::ceil((2.0 * half_width) / grid_resolution_)));
    const uint32_t cells_y = std::max<uint32_t>(1, static_cast<uint32_t>(
      std::ceil((2.0 * half_height) / grid_resolution_)));

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.header.stamp = now();
    grid.info.resolution = static_cast<float>(grid_resolution_);
    grid.info.width = cells_x;
    grid.info.height = cells_y;
    grid.info.origin.position.x = east - half_width;
    grid.info.origin.position.y = north - half_height;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(static_cast<std::size_t>(cells_x) * cells_y, 0);

    for (uint32_t grid_y = 0; grid_y < cells_y; ++grid_y) {
      const uint32_t image_y_begin = height - ((grid_y + 1) * height / cells_y);
      const uint32_t image_y_end = height - (grid_y * height / cells_y);
      for (uint32_t grid_x = 0; grid_x < cells_x; ++grid_x) {
        const uint32_t image_x_begin = grid_x * width / cells_x;
        const uint32_t image_x_end = (grid_x + 1) * width / cells_x;
        double luminance_sum = 0.0;
        std::size_t pixel_count = 0;

        for (uint32_t py = image_y_begin; py < image_y_end; ++py) {
          for (uint32_t px = image_x_begin; px < image_x_end; ++px) {
            const std::size_t offset = static_cast<std::size_t>(py) * step + px * channels;
            luminance_sum += luminance_at(data, offset, channels, bgr);
            ++pixel_count;
          }
        }

        if (pixel_count > 0 && luminance_sum / pixel_count < obstacle_luminance_threshold_) {
          grid.data[static_cast<std::size_t>(grid_y) * cells_x + grid_x] = 100;
        }
      }
    }

    grid_pub_->publish(grid);
  }

  std::string camera_topic_;
  double camera_hfov_{1.74};
  double grid_resolution_{0.25};
  double obstacle_luminance_threshold_{32.0};
  double camera_east_{0.0};
  double camera_north_{0.0};
  double camera_altitude_{0.0};
  bool have_pose_{false};
  std::mutex pose_mutex_;
  gz::transport::Node gz_node_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TraversabilityMapper>());
  rclcpp::shutdown();
  return 0;
}
