#!/usr/bin/env python3
"""Score closed-loop goal arrival; simulator truth is used only here."""

import argparse
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class GoalScore(Node):
    def __init__(self):
        super().__init__('goal_navigation_score')
        self.truth = None
        self.estimate = None
        self.commands = []
        self.create_subscription(PoseStamped, '/evaluation/ugv_truth', self.on_truth,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/ugv/localization/pose',
                                 self.on_pose, 10)
        self.create_subscription(Twist, '/ugv/cmd_vel', self.commands.append, 10)

    def on_truth(self, message):
        self.truth = message

    def on_pose(self, message):
        self.estimate = message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--goal-x', type=float, default=4.0)
    parser.add_argument('--goal-y', type=float, default=0.0)
    parser.add_argument('--timeout', type=float, default=150.0)
    args = parser.parse_args()
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
                if first_motion is not None and not moving and estimated_distance <= 0.5:
                    stopped_at = stopped_at or elapsed
                    if elapsed - stopped_at >= 1.0:
                        break
                else:
                    stopped_at = None
        result = {'first_motion_wall_s': first_motion, 'duration_wall_s': elapsed,
                  'true_goal_distance_m': truth_distance,
                  'estimated_goal_distance_m': estimated_distance,
                  'max_linear_speed_m_s': max_speed,
                  'command_count': len(node.commands),
                  'passed': stopped_at is not None and truth_distance < 0.85}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result))
        assert result['passed'], 'Goal arrival check failed; inspect the recorded conditions'
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
