#!/usr/bin/env python3
"""Validate sensor data and expose measurements in the estimator's frames."""

import copy
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from localization_common.frames import covariance, rotation
from localization_common.geodesy import geodetic_to_enu, make_enu_origin
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus


def sensor_covariance(values, fallback):
    """Unknown (all-zero) covariance uses a stated prior; known data is retained."""
    matrix = covariance(values)
    return np.diag(np.square(fallback)) if not np.any(matrix) else matrix


class UgvMeasurements(Node):
    def __init__(self):
        super().__init__("ugv_measurements")
        latitude = self.declare_parameter("origin_latitude", 47.397971057728974).value
        longitude = self.declare_parameter("origin_longitude", 8.546163739800146).value
        altitude = self.declare_parameter("origin_altitude", 0.0).value
        self.origin = make_enu_origin(latitude, longitude, altitude)
        self.basis = np.array(self.origin[3:])
        self.max_age = self.declare_parameter("max_age", .5).value
        self.gps_fallback = np.array(self.declare_parameter(
            "unknown_gps_stddev", [1.5, 1.5, 3.0]).value)
        self.wheel_fallback = np.array(self.declare_parameter(
            "unknown_wheel_velocity_stddev", [.05, .05, .02]).value)
        self.imu_orientation_fallback = self.declare_parameter(
            "unknown_imu_orientation_stddev", .02).value
        self.imu_rate_fallback = self.declare_parameter("unknown_imu_rate_stddev", .01).value
        self.imu_acceleration_fallback = self.declare_parameter(
            "unknown_imu_acceleration_stddev", .1).value
        values = [self.max_age, self.imu_orientation_fallback,
                  self.imu_rate_fallback, self.imu_acceleration_fallback]
        if (any(not math.isfinite(v) or v <= 0 for v in values)
                or any(v.shape != (3,) or not np.isfinite(v).all() or np.any(v <= 0)
                       for v in (self.gps_fallback, self.wheel_fallback))):
            raise ValueError("Measurement limits and unknown-covariance priors must be positive")
        self.last_clock = None
        self.last_stamps = {}
        self.odom_pub = self.create_publisher(Odometry, "/ugv/measurements/wheel_odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/ugv/measurements/imu", 10)
        self.gps_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/ugv/measurements/gps_pose", 10)
        for cls, topic, callback in ((Odometry, "wheel_odom", self.on_odom),
                                     (Imu, "imu", self.on_imu),
                                     (NavSatFix, "gps", self.on_gps)):
            self.create_subscription(cls, "/ugv/sensors/" + topic, callback,
                                     qos_profile_sensor_data)

    def valid_time(self, message, sensor):
        now = self.get_clock().now().nanoseconds
        if self.last_clock is not None and now < self.last_clock:
            self.last_stamps.clear()
        self.last_clock = now
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if (stamp <= self.last_stamps.get(sensor, 0)
                or not -.05e9 <= now - stamp <= self.max_age * 1e9):
            return False
        self.last_stamps[sensor] = stamp
        return True

    def on_odom(self, message):
        if (message.header.frame_id not in ("map", "simple_ugv/odom", "ugv/odom")
                or message.child_frame_id != "simple_ugv/chassis"
                or not self.valid_time(message, "wheel")):
            return
        twist = message.twist.twist
        if not all(math.isfinite(v) for v in (twist.linear.x, twist.linear.y, twist.angular.z)):
            return
        try:
            matrix = covariance(message.twist.covariance, 6).copy()
        except ValueError:
            return
        for i, fallback in zip((0, 1, 5), self.wheel_fallback):
            if matrix[i, i] == 0:
                matrix[i, i] = fallback**2
        output = copy.deepcopy(message)
        # The EKF consumes body velocities only; an integrated pose is not fused.
        output.twist.covariance = matrix.ravel().tolist()
        self.odom_pub.publish(output)

    def on_imu(self, message):
        if (message.header.frame_id != "simple_ugv/chassis/imu"
                or not self.valid_time(message, "imu")):
            return
        output = copy.deepcopy(message)
        q = message.orientation
        try:
            if message.orientation_covariance[0] == -1:
                raise ValueError("Orientation unavailable")
            rotation((q.w, q.x, q.y, q.z))
            norm = math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)
            output.orientation.w, output.orientation.x, output.orientation.y, output.orientation.z = (
                float(v / norm) for v in (q.w, q.x, q.y, q.z))
            c = sensor_covariance(message.orientation_covariance,
                                  [self.imu_orientation_fallback] * 3)
            output.orientation_covariance = c.ravel().tolist()
        except ValueError:
            output.orientation_covariance = [-1.] + [0.] * 8
        for field, prior in (("angular_velocity", self.imu_rate_fallback),
                             ("linear_acceleration", self.imu_acceleration_fallback)):
            vector = getattr(message, field)
            values = getattr(message, field + "_covariance")
            try:
                if values[0] == -1 or not all(math.isfinite(v) for v in
                                             (vector.x, vector.y, vector.z)):
                    raise ValueError("IMU component unavailable")
                c = sensor_covariance(values, [prior] * 3)
                setattr(output, field + "_covariance", c.ravel().tolist())
            except ValueError:
                setattr(output, field + "_covariance", [-1.] + [0.] * 8)
        self.imu_pub.publish(output)

    def on_gps(self, message):
        if (message.header.frame_id != "simple_ugv/chassis/gps"
                or message.status.status < NavSatStatus.STATUS_FIX
                or not self.valid_time(message, "gps")):
            return
        try:
            position = geodetic_to_enu(message.latitude, message.longitude,
                                       message.altitude, self.origin)
            if message.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
                c = np.diag(self.gps_fallback**2)
            else:
                c = covariance(message.position_covariance)
                if np.linalg.eigvalsh(c[:2, :2]).min() <= 0:
                    raise ValueError("GPS horizontal covariance must be positive")
                fix_basis = np.array(make_enu_origin(
                    message.latitude, message.longitude, message.altitude)[3:])
                transform = self.basis @ fix_basis.T
                c = transform @ c @ transform.T
        except ValueError:
            return
        output = PoseWithCovarianceStamped()
        output.header = copy.deepcopy(message.header)
        output.header.frame_id = "map"
        p = output.pose.pose.position
        p.x, p.y, p.z = (float(v) for v in position)
        output.pose.pose.orientation.w = 1.
        full = np.zeros((6, 6))
        full[:3, :3] = c
        full[3:, 3:] = np.eye(3) * 1e6
        output.pose.covariance = full.ravel().tolist()
        self.gps_pub.publish(output)


def main():
    rclpy.init()
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
