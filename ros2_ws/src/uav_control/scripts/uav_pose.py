#!/usr/bin/env python3
"""Convert synchronized PX4 NED/FRD odometry into a map ENU/FLU anchor."""

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_msgs.msg import VehicleLandDetected, VehicleOdometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from localization_common.frames import px4_pose


class UavPose(Node):
    def __init__(self):
        super().__init__("uav_pose")
        self.origin = np.asarray(self.declare_parameter("map_origin", [0., 0., 0.]).value)
        self.max_age = self.declare_parameter("max_age", .5).value
        self.require_airborne = self.declare_parameter("require_airborne", False).value
        self.settle_time = self.declare_parameter("initial_settle_time", 0.).value
        if (self.origin.shape != (3,) or not np.all(np.isfinite(self.origin))
                or not np.isfinite(self.max_age) or self.max_age <= 0
                or not np.isfinite(self.settle_time) or self.settle_time < 0):
            raise ValueError("map_origin must be finite xyz and max_age must be positive")
        self.publisher = self.create_publisher(PoseWithCovarianceStamped,
                                               "/uav/localization/pose", 20)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry",
                                 self.on_odometry, qos_profile_sensor_data)
        self.last_clock = None
        self.last_stamp = None
        self.reset_counter = None
        self.candidate_counter = None
        self.candidate_since = None
        self.airborne_since = None
        if self.require_airborne:
            self.create_subscription(VehicleLandDetected, "/fmu/out/vehicle_land_detected",
                                     self.on_landing, qos_profile_sensor_data)

    def observe_clock(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock is not None and now < self.last_clock:
            self.last_stamp = self.reset_counter = self.candidate_counter = None
            self.candidate_since = self.airborne_since = None
        self.last_clock = now
        return now

    def on_landing(self, message):
        stamp = int(message.timestamp) * 1000
        now = self.observe_clock()
        if stamp <= 0 or not -.05e9 <= now-stamp <= self.max_age*1e9:
            return
        if message.landed:
            self.airborne_since = None
        elif self.airborne_since is None:
            self.airborne_since = stamp

    def on_odometry(self, message):
        stamp = int(message.timestamp_sample) * 1000
        now = self.observe_clock()
        if message.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            return
        if stamp <= 0 or not -.05e9 <= now-stamp <= self.max_age*1e9:
            self.get_logger().warning("PX4 sample clock differs from ROS clock; check "
                                      "UXRCE_DDS_SYNCT and use_sim_time", throttle_duration_sec=5.)
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        if self.reset_counter is None:
            if self.require_airborne and self.airborne_since is None:
                return
            if self.candidate_counter != message.reset_counter:
                self.candidate_counter = message.reset_counter
                self.candidate_since = stamp
            ready_since = max(self.candidate_since, self.airborne_since or 0)
            if stamp-ready_since < self.settle_time*1e9:
                return
        # A reset changes the meaning of the local origin. Require explicit realignment.
        if self.reset_counter is not None and message.reset_counter != self.reset_counter:
            self.get_logger().error("PX4 local reference reset; realign map_origin and restart",
                                    throttle_duration_sec=5.)
            return
        try:
            position, attitude, c = px4_pose(message.position, message.q,
                                             message.position_variance,
                                             message.orientation_variance, self.origin)
        except ValueError:
            return
        output = PoseWithCovarianceStamped()
        output.header.frame_id = "map"
        output.header.stamp.sec, output.header.stamp.nanosec = divmod(stamp, 1000000000)
        output.pose.pose.position.x, output.pose.pose.position.y, output.pose.pose.position.z = (
            float(value) for value in position)
        q = output.pose.pose.orientation
        q.w, q.x, q.y, q.z = (float(value) for value in attitude)
        output.pose.covariance = c.flatten().tolist()
        self.last_stamp = stamp
        self.reset_counter = message.reset_counter
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = UavPose()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # ROS can close the context while the executor creates its wait set.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
