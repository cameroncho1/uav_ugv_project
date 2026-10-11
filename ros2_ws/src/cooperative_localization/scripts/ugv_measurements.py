#!/usr/bin/env python3
"""Prepare simulated UGV measurements for a map-frame localization filter."""

import math
import random
from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_srvs.srv import SetBool


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def geodetic_to_ecef(latitude_deg, longitude_deg, altitude):
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude = math.sin(latitude)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_latitude**2)
    return (
        (radius + altitude) * math.cos(latitude) * math.cos(longitude),
        (radius + altitude) * math.cos(latitude) * math.sin(longitude),
        (radius * (1.0 - WGS84_E2) + altitude) * sin_latitude,
    )


def geodetic_to_enu(latitude, longitude, altitude, origin):
    x, y, z = geodetic_to_ecef(latitude, longitude, altitude)
    east_x, east_y, east_z = origin[3]
    north_x, north_y, north_z = origin[4]
    up_x, up_y, up_z = origin[5]
    dx, dy, dz = x - origin[0], y - origin[1], z - origin[2]
    return (
        east_x * dx + east_y * dy + east_z * dz,
        north_x * dx + north_y * dy + north_z * dz,
        up_x * dx + up_y * dy + up_z * dz,
    )


def make_enu_origin(latitude_deg, longitude_deg, altitude):
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude, cos_latitude = math.sin(latitude), math.cos(latitude)
    sin_longitude, cos_longitude = math.sin(longitude), math.cos(longitude)
    return (*geodetic_to_ecef(latitude_deg, longitude_deg, altitude),
            (-sin_longitude, cos_longitude, 0.0),
            (-sin_latitude * cos_longitude, -sin_latitude * sin_longitude, cos_latitude),
            (cos_latitude * cos_longitude, cos_latitude * sin_longitude, sin_latitude))


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
    )


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_quaternion(yaw):
    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def diagonal_covariance(values):
    covariance = [0.0] * 36
    for index, value in enumerate(values):
        covariance[index * 6 + index] = value
    return covariance


class UgvMeasurements(Node):
    def __init__(self):
        super().__init__("ugv_measurements")
        self.declare_parameter("origin_latitude", 47.397971057728974)
        self.declare_parameter("origin_longitude", 8.546163739800146)
        self.declare_parameter("origin_altitude", 0.0)
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("spawn_yaw", 0.0)
        self.declare_parameter("gps_horizontal_stddev", 1.5)
        self.declare_parameter("gps_vertical_stddev", 3.0)
        self.declare_parameter("odom_xy_stddev_per_sqrt_meter", 0.05)
        self.declare_parameter("odom_yaw_stddev_per_sqrt_meter", 0.02)
        self.declare_parameter("odom_linear_velocity_stddev", 0.05)
        self.declare_parameter("odom_angular_velocity_stddev", 0.02)
        self.declare_parameter("imu_orientation_stddev", 0.02)
        self.declare_parameter("imu_angular_velocity_stddev", 0.01)
        self.declare_parameter("imu_linear_acceleration_stddev", 0.1)
        self.declare_parameter("random_seed", 513)
        self.declare_parameter("initial_gps_enabled", True)
        self.declare_parameter("wheel_speed_scale", 1.0)
        self.declare_parameter("imu_yaw_bias_rate", 0.0)

        def parameter(name):
            return self.get_parameter(name).value

        self.spawn_x = parameter("spawn_x")
        self.spawn_y = parameter("spawn_y")
        self.spawn_yaw = parameter("spawn_yaw")
        self.origin = make_enu_origin(
            parameter("origin_latitude"), parameter("origin_longitude"),
            parameter("origin_altitude"))
        self.gps_xy_stddev = parameter("gps_horizontal_stddev")
        self.gps_z_stddev = parameter("gps_vertical_stddev")
        self.odom_xy_stddev = parameter("odom_xy_stddev_per_sqrt_meter")
        self.odom_yaw_stddev = parameter("odom_yaw_stddev_per_sqrt_meter")
        self.odom_linear_stddev = parameter("odom_linear_velocity_stddev")
        self.odom_angular_stddev = parameter("odom_angular_velocity_stddev")
        self.imu_orientation_stddev = parameter("imu_orientation_stddev")
        self.imu_angular_stddev = parameter("imu_angular_velocity_stddev")
        self.imu_accel_stddev = parameter("imu_linear_acceleration_stddev")
        seed = parameter("random_seed")
        self.odom_random = random.Random(seed + 1)
        self.imu_random = random.Random(seed + 2)
        self.gps_random = random.Random(seed + 3)
        self.gps_enabled = parameter("initial_gps_enabled")
        self.wheel_speed_scale = parameter("wheel_speed_scale")
        self.imu_yaw_bias_rate = parameter("imu_yaw_bias_rate")
        self.imu_start = None
        self.last_odom = None
        self.dead_reckoned_x = self.spawn_x
        self.dead_reckoned_y = self.spawn_y
        self.dead_reckoned_yaw = self.spawn_yaw
        self.position_variance = 0.01
        self.yaw_variance = 0.0004

        for name in (
            "gps_xy_stddev", "gps_z_stddev", "odom_xy_stddev", "odom_yaw_stddev",
            "odom_linear_stddev", "odom_angular_stddev", "imu_orientation_stddev",
            "imu_angular_stddev", "imu_accel_stddev",
        ):
            if getattr(self, name) < 0.0:
                raise ValueError(f"{name} must be nonnegative")

        output_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.odom_pub = self.create_publisher(
            Odometry, "/ugv/measurements/wheel_odom", output_qos)
        self.imu_pub = self.create_publisher(
            Imu, "/ugv/measurements/imu", output_qos)
        self.gps_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/ugv/measurements/gps_pose", output_qos)
        self.create_subscription(
            Odometry, "/ugv/sensors/wheel_odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(
            Imu, "/ugv/sensors/imu", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(
            NavSatFix, "/ugv/sensors/gps", self.on_gps, qos_profile_sensor_data)
        self.create_service(SetBool, "/ugv/localization/set_gps_enabled", self.set_gps_enabled)

    def set_gps_enabled(self, request, response):
        self.gps_enabled = request.data
        response.success = True
        response.message = "GPS enabled" if self.gps_enabled else "GPS disabled"
        return response

    @staticmethod
    def noise(standard_deviation, generator):
        return generator.gauss(0.0, standard_deviation) if standard_deviation else 0.0

    def on_odom(self, message):
        raw_x = message.pose.pose.position.x
        raw_y = message.pose.pose.position.y
        raw_yaw = yaw_from_quaternion(message.pose.pose.orientation)
        cos_spawn, sin_spawn = math.cos(self.spawn_yaw), math.sin(self.spawn_yaw)
        if self.last_odom is None:
            self.dead_reckoned_x = self.spawn_x + cos_spawn * raw_x - sin_spawn * raw_y
            self.dead_reckoned_y = self.spawn_y + sin_spawn * raw_x + cos_spawn * raw_y
            self.dead_reckoned_yaw = wrap_angle(self.spawn_yaw + raw_yaw)
            distance = math.hypot(raw_x, raw_y)
            self.position_variance += self.odom_xy_stddev**2 * distance
            self.yaw_variance += self.odom_yaw_stddev**2 * distance
        else:
            last_x, last_y, last_yaw = self.last_odom
            dx, dy = raw_x - last_x, raw_y - last_y
            distance = math.hypot(dx, dy)
            step_xy_stddev = self.odom_xy_stddev * math.sqrt(distance)
            step_yaw_stddev = self.odom_yaw_stddev * math.sqrt(distance)
            self.dead_reckoned_x += (
                cos_spawn * dx - sin_spawn * dy + self.noise(step_xy_stddev, self.odom_random))
            self.dead_reckoned_y += (
                sin_spawn * dx + cos_spawn * dy + self.noise(step_xy_stddev, self.odom_random))
            self.dead_reckoned_yaw = wrap_angle(
                self.dead_reckoned_yaw + wrap_angle(raw_yaw - last_yaw)
                + self.noise(step_yaw_stddev, self.odom_random))
            self.position_variance += step_xy_stddev**2
            self.yaw_variance += step_yaw_stddev**2
        self.last_odom = (raw_x, raw_y, raw_yaw)

        output = Odometry()
        output.header = deepcopy(message.header)
        output.header.frame_id = "map"
        output.child_frame_id = message.child_frame_id
        output.pose.pose.position.x = self.dead_reckoned_x
        output.pose.pose.position.y = self.dead_reckoned_y
        output.pose.pose.position.z = message.pose.pose.position.z
        output.pose.pose.orientation = yaw_quaternion(self.dead_reckoned_yaw)
        output.pose.covariance = diagonal_covariance((
            self.position_variance, self.position_variance, 1e6,
            1e6, 1e6, self.yaw_variance))
        output.twist.twist = deepcopy(message.twist.twist)
        output.twist.twist.linear.x *= self.wheel_speed_scale
        output.twist.twist.linear.x += self.noise(self.odom_linear_stddev, self.odom_random)
        # A differential-drive UGV has no lateral wheel velocity. This is a useful
        # zero-speed measurement when GPS is unavailable.
        output.twist.twist.linear.y = 0.0
        output.twist.twist.angular.z += self.noise(self.odom_angular_stddev, self.odom_random)
        output.twist.covariance = diagonal_covariance((
            self.odom_linear_stddev**2, self.odom_linear_stddev**2, 1e6, 1e6, 1e6,
            self.odom_angular_stddev**2))
        self.odom_pub.publish(output)

    def on_imu(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        if self.imu_start is None:
            self.imu_start = stamp
        bias = self.imu_yaw_bias_rate * (stamp-self.imu_start)
        output = Imu()
        output.header = deepcopy(message.header)
        output.orientation = yaw_quaternion(
            yaw_from_quaternion(message.orientation)
            + bias
            + self.noise(self.imu_orientation_stddev, self.imu_random))
        output.orientation_covariance = [
            1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            self.imu_orientation_stddev**2,
        ]
        output.angular_velocity = deepcopy(message.angular_velocity)
        output.angular_velocity.x += self.noise(self.imu_angular_stddev, self.imu_random)
        output.angular_velocity.y += self.noise(self.imu_angular_stddev, self.imu_random)
        output.angular_velocity.z += self.noise(self.imu_angular_stddev, self.imu_random)
        output.angular_velocity.z += self.imu_yaw_bias_rate
        output.angular_velocity_covariance = [
            self.imu_angular_stddev**2, 0.0, 0.0,
            0.0, self.imu_angular_stddev**2, 0.0,
            0.0, 0.0, self.imu_angular_stddev**2,
        ]
        output.linear_acceleration = deepcopy(message.linear_acceleration)
        output.linear_acceleration.x += self.noise(self.imu_accel_stddev, self.imu_random)
        output.linear_acceleration.y += self.noise(self.imu_accel_stddev, self.imu_random)
        output.linear_acceleration.z += self.noise(self.imu_accel_stddev, self.imu_random)
        output.linear_acceleration_covariance = [
            self.imu_accel_stddev**2, 0.0, 0.0,
            0.0, self.imu_accel_stddev**2, 0.0,
            0.0, 0.0, self.imu_accel_stddev**2,
        ]
        self.imu_pub.publish(output)

    def on_gps(self, message):
        if not self.gps_enabled:
            return
        has_fix = message.status.status != NavSatStatus.STATUS_NO_FIX
        coordinates = (message.latitude, message.longitude, message.altitude)
        if not has_fix or not all(math.isfinite(value) for value in coordinates):
            return
        east, north, up = geodetic_to_enu(
            message.latitude, message.longitude, message.altitude, self.origin)
        output = PoseWithCovarianceStamped()
        output.header = deepcopy(message.header)
        output.header.frame_id = "map"
        output.pose.pose.position.x = east + self.noise(self.gps_xy_stddev, self.gps_random)
        output.pose.pose.position.y = north + self.noise(self.gps_xy_stddev, self.gps_random)
        output.pose.pose.position.z = up + self.noise(self.gps_z_stddev, self.gps_random)
        output.pose.pose.orientation.w = 1.0
        output.pose.covariance = diagonal_covariance((
            self.gps_xy_stddev**2, self.gps_xy_stddev**2,
            self.gps_z_stddev**2, 1e6, 1e6, 1e6))
        self.gps_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = UgvMeasurements()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
