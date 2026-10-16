#!/usr/bin/env python3
"""Exercise startup anchoring, known GNSS uncertainty, and EKF rejection."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from localization_common.geodesy import enu_to_geodetic, make_enu_origin
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus


class Measurements(Node):
    def __init__(self):
        super().__init__("gnss_contract_fixture")
        self.origin = make_enu_origin(47.397971057728974, 8.546163739800146, 0.)
        self.gps = self.create_publisher(NavSatFix, "/ugv/sensors/gps", 10)
        self.wheel = self.create_publisher(Odometry, "/ugv/sensors/wheel_odom", 10)
        self.imu = self.create_publisher(Imu, "/ugv/sensors/imu", 10)
        self.positions = []
        self.covariances = []
        self.create_subscription(PoseWithCovarianceStamped, "/ugv/localization/pose",
                                 self.on_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/ugv/measurements/gps_pose",
                                 lambda m: self.covariances.append(m.pose.covariance[0]), 10)

    def on_pose(self, message):
        p = message.pose.pose.position
        self.positions.append([p.x, p.y])

    def publish(self, east=None, variance=4.):
        stamp = self.get_clock().now().to_msg()
        wheel = Odometry()
        wheel.header.stamp = stamp
        wheel.header.frame_id = "simple_ugv/odom"
        wheel.child_frame_id = "simple_ugv/chassis"
        wheel.twist.covariance[0] = wheel.twist.covariance[7] = .0025
        wheel.twist.covariance[35] = .0004
        self.wheel.publish(wheel)
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "simple_ugv/chassis/imu"
        imu.orientation.w = 1.
        for i in (0, 4, 8):
            imu.orientation_covariance[i] = .0004
            imu.angular_velocity_covariance[i] = .0001
        imu.linear_acceleration_covariance[0] = -1.
        self.imu.publish(imu)
        if east is not None:
            gps = NavSatFix()
            gps.header.stamp = stamp
            gps.header.frame_id = "simple_ugv/chassis/gps"
            gps.status.status = NavSatStatus.STATUS_FIX
            gps.latitude, gps.longitude, gps.altitude = enu_to_geodetic(
                east, -400., 0., self.origin)
            gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
            for i in (0, 4, 8):
                gps.position_covariance[i] = variance
            self.gps.publish(gps)

    def phase(self, seconds, east=None, variance=4.):
        self.positions.clear()
        self.covariances.clear()
        deadline = time.monotonic() + seconds
        next_publish = time.monotonic()
        while time.monotonic() < deadline:
            if time.monotonic() >= next_publish:
                self.publish(east, variance)
                next_publish = time.monotonic() + .05
            rclpy.spin_once(self, timeout_sec=.005)
        return list(self.positions), list(self.covariances)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.with_suffix(".log").open("w") as log:
        process = subprocess.Popen(
            ["ros2", "launch", "cooperative_localization", "ugv_localization.launch.py"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Measurements()
        try:
            node.phase(3.)
            anchored, _ = node.phase(3., 1200.)
            degraded, covariances = node.phase(2., 1200., 100.)
            outlier, _ = node.phase(2., 2200.)
            recovered, _ = node.phase(2., 1200.)
            distance = lambda p: math.hypot(p[0] - 1200., p[1] + 400.)
            result = {
                "clock": "wall", "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
                "initial_absolute_position_map_xy": [1200., -400.],
                "outlier_east_m": 2200., "innovation_threshold_sigma": 5.,
                "startup_anchor_error_m": distance(anchored[-1]) if anchored else None,
                "degraded_published_x_variance_m2": covariances[-1] if covariances else None,
                "maximum_outlier_phase_error_m": max(map(distance, outlier)) if outlier else None,
                "recovered_anchor_error_m": distance(recovered[-1]) if recovered else None,
                "sample_counts": list(map(len, (anchored, degraded, outlier, recovered))),
            }
            result["passed"] = bool(
                anchored and outlier and recovered and covariances
                and distance(anchored[-1]) < 2.
                and abs(covariances[-1] - 100.) < 1e-3
                and max(map(distance, outlier)) < 2.
                and distance(recovered[-1]) < 2.)
            args.output.write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result), flush=True)
            assert result["passed"], "GNSS startup/uncertainty/innovation contract failed"
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=2.)
                    break
                except subprocess.TimeoutExpired:
                    continue


if __name__ == "__main__":
    main()
