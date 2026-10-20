#!/usr/bin/env python3
"""Compare real EKFs on identical declared measurements, with/without camera yaw."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import yaml


def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))


class HeadingFixture(Node):
    def __init__(self, imu_stddev):
        super().__init__('camera_heading_fixture')
        self.imu_variance = imu_stddev**2
        self.wheel = self.create_publisher(Odometry, '/ugv/sensors/wheel_odom', 10)
        self.imu = self.create_publisher(Imu, '/ugv/sensors/imu', 10)
        self.anchor = self.create_publisher(
            PoseWithCovarianceStamped, '/uav/localization/pose', 10)
        self.relative = self.create_publisher(
            PoseWithCovarianceStamped, '/ugv/sensors/uav_relative_pose', 10)
        self.assisted, self.position_only = [], []
        self.joint_stamps, self.position_stamps = set(), set()
        self.create_subscription(PoseWithCovarianceStamped, '/ugv/localization/pose',
                                 lambda m: self.assisted.append(yaw(m.pose.pose.orientation)), 10)
        self.create_subscription(Odometry, '/heading_fixture/position_only',
                                 lambda m: self.position_only.append(yaw(m.pose.pose.orientation)), 10)
        self.create_subscription(PoseWithCovarianceStamped,
                                 '/ugv/measurements/uav_pose_with_heading',
                                 lambda m: self.joint_stamps.add(self.stamp(m)), 10)
        self.create_subscription(PoseWithCovarianceStamped, '/ugv/measurements/uav_pose',
                                 lambda m: self.position_stamps.add(self.stamp(m)), 10)
        self.ticks = 0

    @staticmethod
    def stamp(message):
        return message.header.stamp.sec*1000000000 + message.header.stamp.nanosec

    def publish(self, imu_yaw, camera=True, camera_yaw=0.):
        stamp = self.get_clock().now().to_msg()
        wheel = Odometry()
        wheel.header.stamp, wheel.header.frame_id = stamp, 'simple_ugv/odom'
        wheel.child_frame_id = 'simple_ugv/chassis'
        wheel.twist.covariance[0] = wheel.twist.covariance[7] = .0025
        self.wheel.publish(wheel)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'simple_ugv/chassis/imu'
        imu.orientation.w, imu.orientation.z = math.cos(imu_yaw/2), math.sin(imu_yaw/2)
        imu.orientation_covariance = np.diag([self.imu_variance]*3).ravel().tolist()
        imu.angular_velocity_covariance = np.diag([.0001]*3).ravel().tolist()
        imu.linear_acceleration_covariance[0] = -1.
        self.imu.publish(imu)
        self.ticks += 1
        if self.ticks % 4:
            return
        anchor = PoseWithCovarianceStamped()
        anchor.header.stamp, anchor.header.frame_id = stamp, 'map'
        anchor.pose.pose.position.z = 4.
        anchor.pose.pose.orientation.w = math.cos(.7/2)
        anchor.pose.pose.orientation.z = math.sin(.7/2)
        anchor.pose.covariance = np.diag([.01]*3 + [.0004]*3).ravel().tolist()
        self.anchor.publish(anchor)
        if not camera:
            return
        relative = PoseWithCovarianceStamped()
        relative.header.stamp, relative.header.frame_id = stamp, 'uav/base_link'
        relative.pose.pose.position.z = -4.
        relative.pose.pose.orientation.w = math.cos((camera_yaw-.7)/2)
        relative.pose.pose.orientation.z = math.sin((camera_yaw-.7)/2)
        relative.pose.covariance = np.diag([.01]*3 + [.0004]*3).ravel().tolist()
        self.relative.publish(relative)

    def phase(self, duration, start_bias, end_bias=None, camera=True, camera_yaw=0.):
        self.assisted.clear()
        self.position_only.clear()
        start = time.monotonic()
        next_publish = start
        while time.monotonic()-start < duration:
            now = time.monotonic()
            if now >= next_publish:
                fraction = min(1., (now-start)/duration)
                bias = start_bias if end_bias is None else start_bias+(end_bias-start_bias)*fraction
                self.publish(bias, camera, camera_yaw)
                next_publish = now+.05
            rclpy.spin_once(self, timeout_sec=.005)
        assert len(self.assisted) > 10 and len(self.position_only) > 10, (
            f'Missing EKF output: assisted={len(self.assisted)}, '
            f'position_only={len(self.position_only)}, joint_updates={len(self.joint_stamps)}')
        tail = lambda values: values[-min(30, len(values)):]
        return {'assisted_mean_abs_heading_rad': float(np.mean(np.abs(tail(self.assisted)))),
                'position_only_mean_abs_heading_rad': float(np.mean(np.abs(tail(self.position_only)))),
                'assisted_max_abs_heading_rad': float(np.max(np.abs(self.assisted))),
                'sample_counts': [len(self.assisted), len(self.position_only)]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--imu-bias', type=float, default=.18,
                        help='Final IMU heading bias in radians (default: 0.18).')
    parser.add_argument('--imu-stddev', type=float, default=.1,
                        help='Declared IMU orientation standard deviation in radians (default: 0.1).')
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    share = Path(get_package_share_directory('cooperative_localization'))
    baseline = yaml.safe_load((share/'config/ugv_ekf.yaml').read_text())['ugv_ekf']['ros__parameters']
    baseline['pose0'] = '/ugv/measurements/uav_pose_with_heading'
    baseline['pose1'] = '/ugv/measurements/uav_pose'
    baseline['pose1_config'] = list(baseline['pose0_config'])
    baseline['pose1_rejection_threshold'] = 5.
    processes = []
    if not math.isfinite(args.imu_bias) or not 0 < args.imu_bias < math.pi:
        parser.error('--imu-bias must be finite and between zero and pi')
    if not math.isfinite(args.imu_stddev) or args.imu_stddev <= 0:
        parser.error('--imu-stddev must be finite and positive')
    result = {'passed': False, 'clock': 'wall', 'expected_map_heading_rad': 0.,
              'imu_bias_rad': args.imu_bias,
              'uav_map_heading_rad': .7, 'imu_orientation_stddev_rad': args.imu_stddev,
              'relative_orientation_stddev_rad': .02, 'phases': {}}
    with tempfile.TemporaryDirectory() as temporary, args.output.with_suffix('.log').open('w') as log:
        config = Path(temporary)/'baseline.yaml'
        config.write_text(yaml.safe_dump({'ugv_ekf_position_only': {'ros__parameters': baseline}}))
        commands = [
            ['ros2', 'launch', 'cooperative_localization', 'ugv_localization.launch.py',
             'uav_assistance:=true', 'uav_heading:=true'],
            ['ros2', 'run', 'robot_localization', 'ekf_node', '--ros-args',
             '-r', '__node:=ugv_ekf_position_only', '--params-file', str(config),
             '-r', 'odometry/filtered:=/heading_fixture/position_only']]
        rclpy.init()
        node = HeadingFixture(args.imu_stddev)
        try:
            for command in commands:
                processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                                  start_new_session=True))
            # Publish while DDS discovery and estimator initialization complete.
            deadline = time.monotonic()+20.
            while len(node.assisted) < 10 or len(node.position_only) < 10:
                assert time.monotonic() < deadline, (
                    f'EKF discovery timed out: assisted={len(node.assisted)}, '
                    f'position_only={len(node.position_only)}, joint={len(node.joint_stamps)}')
                node.publish(0.)
                rclpy.spin_once(node, timeout_sec=.05)
            phases = result['phases']
            phases['initial'] = node.phase(3., 0.)
            phases['drift'] = node.phase(12., 0., args.imu_bias)
            phases['outage'] = node.phase(2., args.imu_bias, camera=False)
            phases['recovery'] = node.phase(3., args.imu_bias)
            phases['outlier'] = node.phase(1., args.imu_bias, camera_yaw=math.radians(170))
            phases['final'] = node.phase(3., args.imu_bias)
            result['joint_updates'] = len(node.joint_stamps)
            result['duplicate_position_updates'] = len(node.joint_stamps & node.position_stamps)
            recovered = phases['recovery']
            result['passed'] = bool(
                result['joint_updates'] >= 50 and result['duplicate_position_updates'] == 0
                and recovered['assisted_mean_abs_heading_rad'] < .12
                and recovered['assisted_mean_abs_heading_rad'] <
                recovered['position_only_mean_abs_heading_rad']*.7
                and phases['outlier']['assisted_max_abs_heading_rad'] < .2
                and phases['final']['assisted_mean_abs_heading_rad'] < .12)
            assert result['passed'], 'Heading fusion, recovery, or outlier rejection failed'
        finally:
            args.output.write_text(json.dumps(result, indent=2)+'\n')
            print(json.dumps(result), flush=True)
            node.destroy_node()
            rclpy.try_shutdown()
            for process in processes:
                for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                    try:
                        if sig == signal.SIGINT:
                            process.send_signal(sig)
                        else:
                            os.killpg(process.pid, sig)
                    except ProcessLookupError:
                        pass
                    try:
                        process.wait(timeout=2.)
                        break
                    except subprocess.TimeoutExpired:
                        continue


if __name__ == '__main__':
    main()
