#!/usr/bin/env python3
"""Exercise the real planner and controller with controlled ROS inputs."""

import argparse
import copy
import json
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from uav_ugv_msgs.msg import PlannedRoute
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock


class GuardTrial(Node):
    def __init__(self):
        super().__init__('navigation_guard_trial')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose = self.create_publisher(PoseWithCovarianceStamped, '/ugv/localization/pose', 10)
        self.raw = self.create_publisher(PoseStamped, '/ugv_pose', 10)
        self.grid = self.create_publisher(OccupancyGrid, '/local_grid', latched)
        self.route = self.create_publisher(PlannedRoute, "/ugv/navigation/route", latched)
        self.goal = self.create_publisher(PoseStamped, '/global_goal_pose', latched)
        self.clock = self.create_publisher(Clock, '/clock', 10)
        self.commands = []
        self.paths = []
        self.create_subscription(Twist, '/ugv/cmd_vel', self.commands.append, 10)
        self.create_subscription(RosPath, '/local_path', self.paths.append, latched)
        self.started = time.monotonic()
        self.clock_time = 10.0

    def phase(self, name, moving, duration=2.0):
        self.commands.clear()
        self.paths.clear()
        goal_revision = None
        start = time.monotonic()
        while time.monotonic() - start < duration:
            elapsed = time.monotonic() - start
            if name != 'frozen_clock':
                self.clock_time = 10 + time.monotonic() - self.started
            clock = Clock()
            clock.clock.sec = int(self.clock_time)
            clock.clock.nanosec = int((self.clock_time % 1) * 1e9)
            self.clock.publish(clock)
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            if goal_revision is None:
                goal_revision = copy.deepcopy(clock.clock)
            goal.header.stamp = goal_revision
            goal.pose.position.x = 0.0 if name == "obsolete_route" else 3.0
            goal.pose.position.y = 3.0 if name == "obsolete_route" else 0.0
            goal.pose.orientation.w = 1.0
            self.goal.publish(goal)
            raw = PoseStamped()
            raw.header = goal.header
            raw.pose.orientation.w = 1.0
            self.raw.publish(raw)
            pose = PoseWithCovarianceStamped()
            pose.header = copy.deepcopy(goal.header)
            pose.header.stamp = clock.clock
            pose.pose.pose.orientation.w = 1.0
            pose.pose.covariance[0] = pose.pose.covariance[7] = 0.04
            pose.pose.covariance[35] = 0.01
            if name == 'wrong_frame':
                pose.header.frame_id = 'odom'
            elif name == 'uncertain_position':
                pose.pose.covariance[0] = 4.0
            elif name == 'uncertain_heading':
                pose.pose.covariance[35] = 1.0
            elif name == 'correlated_xy':
                pose.pose.covariance[0] = pose.pose.covariance[7] = 0.2
                pose.pose.covariance[1] = pose.pose.covariance[6] = 0.15
            elif name == 'stale_pose':
                pose.header.stamp.sec -= 2
            elif name == 'future_pose':
                pose.header.stamp.sec += 2
            elif name == 'goal_reached':
                pose.pose.pose.position.x = 3.0
            if name not in ('raw_only', 'missing_pose', 'frozen_clock'):
                self.pose.publish(pose)
            if name != 'missing_grid':
                grid = OccupancyGrid()
                grid.header = copy.deepcopy(goal.header)
                grid.header.stamp = clock.clock
                grid.info.resolution = 0.25
                grid.info.width, grid.info.height = 80, 40
                grid.info.origin.position.x = grid.info.origin.position.y = -5.0
                grid.info.origin.orientation.w = 1.0
                grid.data = [100 if name == 'blocked_grid' else 0] * 3200
                if name == 'blocked_start':
                    grid.data[20 * 80 + 20] = 100
                if name == 'invalid_grid':
                    grid.data = []
                self.grid.publish(grid)
                if name == "obsolete_route":
                    route = PlannedRoute()
                    route.header = copy.deepcopy(grid.header)
                    route.goal = copy.deepcopy(goal)
                    route.goal.pose.position.x, route.goal.pose.position.y = 3.0, 0.0
                    route.goal.header.stamp.sec -= 1
                    route.path.header = copy.deepcopy(route.header)
                    for x in (0.0, 0.6, 3.0):
                        point = PoseStamped()
                        point.header = copy.deepcopy(route.header)
                        point.pose.position.x = x
                        point.pose.orientation.w = 1.0
                        route.path.poses.append(point)
                    route.clearance_grid = copy.deepcopy(grid)
                    self.route.publish(route)
            rclpy.spin_once(self, timeout_sec=0.05)
            if elapsed < duration - 0.5:
                self.commands.clear()
                self.paths.clear()
        assert self.commands, f'{name}: controller did not publish commands'
        speeds = [abs(c.linear.x) + abs(c.angular.z) for c in self.commands]
        if moving:
            assert max(speeds) > 0.05, f'{name}: expected motion'
        else:
            assert max(speeds) < 1e-9, f'{name}: expected a continuous zero command'
        return {'case': name, 'expected_motion': moving, 'passed': True,
                'commands_checked': len(speeds), 'max_command_magnitude': max(speeds),
                'path_received': bool(self.paths),
                'path_points': len(self.paths[-1].poses) if self.paths else 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    processes, logs, results = [], [], []
    rclpy.init()
    node = GuardTrial()
    try:
        for package, executable in [('route_planning', 'local_astar_planner'),
                                    ('ugv_control', 'goal_controller')]:
            log = args.output.with_name(f'{package}.log').open('w')
            logs.append(log)
            binary = Path(get_package_prefix(package)) / 'lib' / package / executable
            processes.append(subprocess.Popen(
                [str(binary), '--ros-args', '-p', 'use_sim_time:=true'],
                stdout=log, stderr=subprocess.STDOUT))
        deadline = time.monotonic() + 20
        while node.pose.get_subscription_count() < 2:
            assert time.monotonic() < deadline, 'Planner/controller discovery timed out'
            rclpy.spin_once(node, timeout_sec=0.1)
        cases = [('raw_only', False), ('valid', True), ('uncertain_position', False),
                 ('recovery', True), ('uncertain_heading', False), ('correlated_xy', False),
                 ('wrong_frame', False), ('stale_pose', False), ('future_pose', False),
                 ('recovery', True), ('missing_pose', False), ('recovery', True),
                 ('frozen_clock', False), ('recovery', True), ('missing_grid', False),
                 ('invalid_grid', False), ('blocked_grid', False), ('blocked_start', False), ('recovery', True),
                 ('goal_reached', False)]
        for name, moving in cases:
            results.append(node.phase(name, moving, 3.0 if name == 'missing_grid' else 2.0))
        processes[0].send_signal(signal.SIGSTOP)
        try:
            results.append(node.phase("obsolete_route", False))
        finally:
            processes[0].send_signal(signal.SIGCONT)
        results.append(node.phase("recovery", True))
        args.output.write_text(json.dumps({'cases': results, 'passed': True}, indent=2) + '\n')
        print(f'{len(results)} navigation cases passed')
    finally:
        for process in processes:
            process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
