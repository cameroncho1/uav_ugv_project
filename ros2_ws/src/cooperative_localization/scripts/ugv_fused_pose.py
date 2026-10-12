#!/usr/bin/env python3
"""Expose the UGV EKF output through the localization pose contract."""

import rclpy
import copy
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def pose_from_odometry(odometry, systematic_position_stddev=0.0):
    if odometry.header.frame_id != "map":
        raise ValueError("Filtered odometry must use the map frame")
    if not math.isfinite(systematic_position_stddev) or systematic_position_stddev < 0:
        raise ValueError("Systematic position deviation must be finite and nonnegative")
    pose = PoseWithCovarianceStamped()
    pose.header = odometry.header
    pose.pose = copy.deepcopy(odometry.pose)
    # A shared global offset is not reduced by repeated independent EKF updates.
    # This stated prior is added only to the public covariance, never fed back.
    pose.pose.covariance[0] += systematic_position_stddev**2
    pose.pose.covariance[7] += systematic_position_stddev**2
    return pose


class UgvFusedPose(Node):
    def __init__(self):
        super().__init__("ugv_fused_pose")
        self.systematic_stddev = self.declare_parameter("systematic_position_stddev", 0.0).value
        if not math.isfinite(self.systematic_stddev) or self.systematic_stddev < 0:
            raise ValueError("Systematic position deviation must be finite and nonnegative")
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/ugv/localization/pose", 10)
        self.create_subscription(
            Odometry, "/ugv/localization/odometry", self.on_odometry, 10)

    def on_odometry(self, odometry):
        try:
            pose = pose_from_odometry(odometry, self.systematic_stddev)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        self.publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = UgvFusedPose()
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
