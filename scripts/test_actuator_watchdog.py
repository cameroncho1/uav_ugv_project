#!/usr/bin/env python3
"""Score physical stopping after killing the actual controller or ROS command bridge."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from uav_ugv_msgs.msg import PlannedRoute

ROOT = Path(__file__).resolve().parents[1]


class Fixture(Node):
    def __init__(self):
        super().__init__("actuator_watchdog_fixture")
        self.clock = None
        self.truth = []
        self.goal_revision = None
        self.last_fixture_stamp = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/ugv/localization/pose", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/global_goal_pose", latched)
        self.route_pub = self.create_publisher(PlannedRoute, "/ugv/navigation/route", latched)
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/evaluation/ugv_truth", self.on_truth,
                                 qos_profile_sensor_data)

    def on_clock(self, message):
        self.clock = message.clock

    def on_truth(self, message):
        self.truth.append((time.monotonic(), message.pose.position.x, message.pose.position.y))

    def tick(self):
        stamp = ((self.clock.sec, self.clock.nanosec) if self.clock is not None else None)
        if stamp is not None and stamp != self.last_fixture_stamp:
            self.last_fixture_stamp = stamp
            if self.goal_revision is None:
                self.goal_revision = self.clock
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.clock
            # Declared test input, independent of measured truth. This fixture
            # tests the actuator boundary, not estimation or physical accuracy.
            pose.pose.pose.position.x = 1.0
            pose.pose.pose.orientation.w = 1.0
            pose.pose.covariance[0] = pose.pose.covariance[7] = .04
            pose.pose.covariance[35] = .01
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = self.goal_revision
            goal.pose.position.x = 4.0
            goal.pose.orientation.w = 1.0
            grid = OccupancyGrid()
            grid.header = pose.header
            grid.info.width = grid.info.height = 80
            grid.info.resolution = .25
            grid.info.origin.position.x = grid.info.origin.position.y = -5.0
            grid.info.origin.orientation.w = 1.0
            grid.data = [0] * 6400
            route = PlannedRoute()
            route.header = pose.header
            route.goal = goal
            route.path.header = pose.header
            route.clearance_grid = grid
            for x in (1.0, 1.6, 4.0):
                point = PoseStamped()
                point.header = pose.header
                point.pose.position.x = x
                point.pose.orientation.w = 1.0
                route.path.poses.append(point)
            self.goal_pub.publish(goal)
            self.pose_pub.publish(pose)
            self.route_pub.publish(route)
        rclpy.spin_once(self, timeout_sec=.05)


def stop(processes):
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        for p in processes:
            try: os.killpg(p.pid, sig)
            except ProcessLookupError: pass
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and any(p.poll() is None for p in processes):
            time.sleep(.05)
    for p in processes:
        if p.poll() is not None: p.wait()


def case(output, mode):
    directory = output / mode
    directory.mkdir(parents=True, exist_ok=True)
    os.environ["GZ_PARTITION"] = "watchdog_" + mode + "_" + str(os.getpid())
    os.environ["GZ_IP"] = "127.0.0.1"
    processes, logs = [], []

    def launch(label, command):
        log = (directory / (label + ".log")).open("w")
        logs.append(log)
        process = subprocess.Popen(command, env=os.environ.copy(), cwd=ROOT,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    rclpy.init()
    node = Fixture()
    try:
        launch("world", ["bash", str(ROOT / "scripts/world.sh")])
        launch("sensors", ["ros2", "launch", "simulation_support", "ugv_sensors.launch.py"])
        launch("truth", ["ros2", "run", "simulation_support", "ugv_truth", "--ros-args",
                         "-p", "world:=uav_ugv", "-p", "use_sim_time:=true"])
        bridge = launch("command_bridge", ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "--ros-args", "-p", "config_file:=" + str(ROOT / "ros2_ws/src/mission_manager/config/ugv_command_bridge.yaml")])
        controller = launch("controller", ["ros2", "run", "ugv_control", "goal_controller",
                                           "--ros-args", "-p", "use_sim_time:=true"])
        deadline = time.monotonic() + 35
        initial = None
        while time.monotonic() < deadline:
            node.tick()
            if node.truth:
                initial = initial or node.truth[0]
                if node.truth[-1][1] - initial[1] >= .15:
                    break
        assert initial is not None and node.truth[-1][1] - initial[1] >= .15, "No physical pre-fault motion"
        killed = controller if mode == "controller_death" else bridge
        lost_at = time.monotonic()
        os.killpg(killed.pid, signal.SIGKILL)
        while time.monotonic() - lost_at < 1.6:
            node.tick()
        late = [row for row in node.truth if row[0] >= lost_at + .75]
        assert len(late) >= 5, "Missing physical truth after command loss"
        distance = max(((row[1] - late[0][1])**2 + (row[2] - late[0][2])**2)**.5 for row in late)
        result = {"case": mode, "pre_fault_motion_m": node.truth[-1][1] - initial[1],
                  "watchdog_timeout_wall_s": .35, "maximum_additional_delivery_period_wall_s": .05,
                  "settled_check_after_wall_s": .75, "late_truth_samples": len(late),
                  "maximum_late_displacement_m": distance, "passed": distance < .02}
        (directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        assert result["passed"], "Actuator kept moving after controller/transport loss"
        return result
    finally:
        stop(processes)
        node.destroy_node()
        rclpy.try_shutdown()
        for log in logs: log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = [case(args.output, mode) for mode in ("controller_death", "command_transport_loss")]
    (args.output / "summary.json").write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps(results))


if __name__ == "__main__":
    main()
