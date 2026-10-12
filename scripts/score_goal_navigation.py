#!/usr/bin/env python3
"""Score closed-loop goal arrival; simulator truth is used only here."""

import argparse
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class GoalScore(Node):
    def __init__(self):
        super().__init__('goal_navigation_score')
        self.truth = None
        self.estimate = None
        self.commands = []
        self.pose_count = 0
        self.correction_count = 0
        self.relative_count = 0
        self.wheel_count = 0
        self.gps_count = 0
        self.min_position_stddev = float('inf')
        self.truth_receipt = self.pose_receipt = self.command_receipt = 0.0
        self.create_subscription(PoseStamped, '/evaluation/ugv_truth', self.on_truth,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/ugv/localization/pose',
                                 self.on_pose, 10)
        self.create_subscription(Twist, '/ugv/cmd_vel', self.on_command, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/ugv/measurements/uav_pose',
                                 self.on_correction, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/ugv/sensors/uav_relative_pose',
                                 self.on_relative, 10)
        self.create_subscription(Odometry, '/ugv/measurements/wheel_odom', self.on_wheel, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/ugv/measurements/gps_pose',
                                 self.on_gps, 10)

    def on_truth(self, message):
        self.truth = message
        self.truth_receipt = time.monotonic()

    def on_pose(self, message):
        self.estimate = message
        self.pose_receipt = time.monotonic()
        self.pose_count += 1
        c = message.pose.covariance
        largest = (c[0] + c[7] + math.hypot(c[0] - c[7], 2 * c[1])) / 2
        self.min_position_stddev = min(self.min_position_stddev, math.sqrt(largest))

    def on_command(self, message):
        self.commands.append(message)
        self.command_receipt = time.monotonic()

    def on_correction(self, _):
        self.correction_count += 1

    def on_relative(self, _):
        self.relative_count += 1

    def on_gps(self, _):
        self.gps_count += 1

    def on_wheel(self, message):
        if (message.header.frame_id != 'map' or
                message.child_frame_id != 'simple_ugv/chassis'):
            raise RuntimeError('Wheel measurements require map and simple_ugv/chassis')
        self.wheel_count += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--goal-x', type=float, default=4.0)
    parser.add_argument('--goal-y', type=float, default=0.0)
    parser.add_argument('--timeout', type=float, default=600.0)
    parser.add_argument('--seed', type=int, default=513)
    parser.add_argument('--shared-position-stddev', type=float, default=0.0)
    parser.add_argument('--systematic-position-stddev', type=float, default=0.0)
    parser.add_argument('--uav-altitude', type=float, default=4.5)
    parser.add_argument('--initial-gps-enabled', action='store_true')
    parser.add_argument('--expect-hold', action='store_true')
    parser.add_argument('--true-goal-tolerance', type=float, default=0.85)
    args = parser.parse_args()
    limits = [args.goal_x, args.goal_y, args.timeout, args.true_goal_tolerance,
              args.shared_position_stddev, args.systematic_position_stddev, args.uav_altitude]
    if (not all(math.isfinite(v) for v in limits) or
            args.timeout <= 0 or args.true_goal_tolerance <= 0 or args.uav_altitude <= 0 or
            args.shared_position_stddev < 0 or args.systematic_position_stddev < 0):
        parser.error('Require finite coordinates, positive timing/altitude, '
                     'and nonnegative deviations')
    rclpy.init()
    node = GoalScore()
    started = time.monotonic()
    first_motion, stopped_at = None, None
    max_speed = 0.0
    truth_distance, estimated_distance = None, None
    try:
        while time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = time.monotonic() - started
            if node.commands:
                command = node.commands[-1]
                max_speed = max(max_speed, abs(command.linear.x))
                moving = abs(command.linear.x) + abs(command.angular.z) > 0.001
                if moving and first_motion is None:
                    first_motion = elapsed
            else:
                moving = False
            if node.truth and node.estimate:
                truth = node.truth.pose.position
                estimate = node.estimate.pose.pose.position
                truth_distance = math.hypot(truth.x - args.goal_x, truth.y - args.goal_y)
                estimated_distance = math.hypot(
                    estimate.x - args.goal_x, estimate.y - args.goal_y)
                fresh = all(time.monotonic() - receipt < 0.5 for receipt in
                            [node.truth_receipt, node.pose_receipt, node.command_receipt])
                hold_ready = (args.expect_hold and first_motion is None and
                              len(node.commands) > 100 and node.pose_count > 100 and
                              node.wheel_count > 100 and node.correction_count > 20 and
                              node.relative_count > 20 and node.min_position_stddev > 0.5 and
                              (node.gps_count > 10 if args.initial_gps_enabled else
                               node.gps_count == 0))
                # Capture a qualifying fresh sample within the bounded window.
                # A deadline can fall in a rendering pause despite a valid hold.
                if hold_ready and fresh:
                    break
                if first_motion is not None and fresh and not moving and estimated_distance <= 0.5:
                    stopped_at = stopped_at or elapsed
                    if elapsed - stopped_at >= 1.0:
                        break
                else:
                    stopped_at = None
        fresh = all(time.monotonic() - receipt < 0.5 for receipt in
                    [node.truth_receipt, node.pose_receipt, node.command_receipt])
        passed = (first_motion is None and len(node.commands) > 100 and fresh and
                  node.pose_count > 100 and node.correction_count > 20 and
                  node.relative_count > 20 and node.min_position_stddev > 0.5
                  if args.expect_hold else
                  stopped_at is not None and truth_distance < args.true_goal_tolerance)
        passed = (passed and node.wheel_count > 100 and node.relative_count > 20 and
                  node.correction_count > 20 and
                  (node.gps_count > 10 if args.initial_gps_enabled else node.gps_count == 0))
        result = {'seed': args.seed, 'expected_hold': args.expect_hold,
                  'initial_gps_enabled': args.initial_gps_enabled,
                  'shared_position_stddev_m': args.shared_position_stddev,
                  'systematic_position_stddev_m': args.systematic_position_stddev,
                  'uav_altitude_setpoint_m': args.uav_altitude,
                  'relative_source': 'camera', 'pose_count': node.pose_count,
                  'uav_correction_count': node.correction_count,
                  'relative_observation_count': node.relative_count,
                  'wheel_measurement_count': node.wheel_count,
                  'gps_measurement_count': node.gps_count,
                  'min_position_stddev_m': (node.min_position_stddev
                                            if math.isfinite(node.min_position_stddev) else None),
                  'final_receipt_ages_wall_s': {
                      'truth': time.monotonic() - node.truth_receipt,
                      'pose': time.monotonic() - node.pose_receipt,
                      'command': time.monotonic() - node.command_receipt},
                  'first_motion_wall_s': first_motion, 'duration_wall_s': elapsed,
                  'true_goal_distance_m': truth_distance,
                  'estimated_goal_distance_m': estimated_distance,
                  'true_goal_tolerance_m': args.true_goal_tolerance,
                  'max_linear_speed_m_s': max_speed,
                  'command_count': len(node.commands),
                  'passed': passed}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result))
        assert result['passed'], 'Goal arrival check failed; inspect the recorded conditions'
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
