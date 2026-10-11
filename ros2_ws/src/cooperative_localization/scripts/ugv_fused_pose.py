#!/usr/bin/env python3
"""Expose the UGV EKF output through the localization pose contract."""

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def pose_from_odometry(odometry):
    if odometry.header.frame_id != "map":
        raise ValueError("Filtered odometry must use the map frame")
    pose = PoseWithCovarianceStamped()
    pose.header = odometry.header
    pose.pose = odometry.pose
    return pose


class UgvFusedPose(Node):
    def __init__(self):
        super().__init__("ugv_fused_pose")
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/ugv/localization/pose", 10)
        self.create_subscription(
            Odometry, "/ugv/localization/odometry", self.on_odometry, 10)

    def on_odometry(self, odometry):
        try:
            pose = pose_from_odometry(odometry)
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
