#!/usr/bin/env python3
"""Drive seeded open-loop UGV maneuvers and record clean raw sensors with truth labels."""

import argparse
import csv
import json
import math
import random
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, NavSatFix

MAX_LINEAR = 0.8
MAX_ANGULAR = 1.2
LINEAR_ACCEL = 0.8
ANGULAR_ACCEL = 2.0
COMMAND_PERIOD = 0.05

COLUMNS = {
    "imu": ["stamp", "qx", "qy", "qz", "qw", "wx", "wy", "wz", "ax", "ay", "az"],
    "wheel_odom": ["stamp", "x", "y", "z", "qx", "qy", "qz", "qw",
                   "vx", "vy", "vz", "wx", "wy", "wz"],
    "gps": ["stamp", "status", "latitude", "longitude", "altitude"],
    "truth": ["stamp", "x", "y", "z", "qx", "qy", "qz", "qw"],
    "cmd": ["stamp", "linear", "angular", "segment"],
}


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def maneuver_schedule(seed, duration, settle):
    """Return seeded (start, end, kind, parameters) segments covering ``duration``."""
    generator = random.Random(seed)
    kinds = ["stop", "straight", "arc", "spin", "weave", "ramp"]
    weights = [1, 3, 3, 1, 2, 2]
    segments = [(0.0, settle, "stop", {})]
    start = settle
    while start < duration:
        kind = generator.choices(kinds, weights)[0]
        length = generator.uniform(2.0, 4.0) if kind == "stop" else generator.uniform(3.0, 9.0)
        speed = generator.choice([-1, 1, 1, 1]) * generator.uniform(0.15, MAX_LINEAR)
        parameters = {
            "stop": {},
            "straight": {"linear": speed},
            "arc": {"linear": speed, "angular": generator.uniform(-1, 1) * MAX_ANGULAR * 0.7},
            "spin": {"angular": generator.choice([-1, 1]) * generator.uniform(0.3, MAX_ANGULAR)},
            "weave": {"linear": abs(speed), "amplitude": generator.uniform(0.3, MAX_ANGULAR),
                      "period": generator.uniform(2.0, 6.0)},
            "ramp": {"from": generator.uniform(0.0, MAX_LINEAR),
                     "to": generator.uniform(0.0, MAX_LINEAR)},
        }[kind]
        end = min(start + length, duration)
        segments.append((start, end, kind, parameters))
        start = end
    return segments


def target_command(segment, elapsed):
    """Return the unconstrained (linear, angular) command for a segment at ``elapsed``."""
    start, end, kind, parameters = segment
    t = elapsed - start
    if kind == "weave":
        return (parameters["linear"],
                parameters["amplitude"] * math.sin(2 * math.pi * t / parameters["period"]))
    if kind == "ramp":
        fraction = t / max(end - start, 1e-6)
        return parameters["from"] + fraction * (parameters["to"] - parameters["from"]), 0.0
    return parameters.get("linear", 0.0), parameters.get("angular", 0.0)


def step_toward(current, target, limit):
    return current + max(-limit, min(limit, target - current))


class UgvDatasetRecorder(Node):
    def __init__(self, args):
        super().__init__("ugv_dataset_recorder")
        self.args = args
        self.output = Path(args.output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.segments = maneuver_schedule(args.seed, args.duration, args.settle)
        self.files = {}
        self.writers = {}
        for name, columns in COLUMNS.items():
            self.files[name] = (self.output / f"{name}.csv").open("w", newline="")
            self.writers[name] = csv.writer(self.files[name])
            self.writers[name].writerow(columns)
        self.counts = dict.fromkeys(COLUMNS, 0)
        self.seen = set()
        self.sim_time = None
        self.start = None
        self.segment_index = 0
        self.linear = 0.0
        self.angular = 0.0
        self.done = False
        self.cmd_pub = self.create_publisher(Twist, "/evaluation/ugv_cmd_vel", 10)
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.create_subscription(
            Imu, "/simulation/ugv/imu", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, "/simulation/ugv/wheel_odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(
            NavSatFix, "/simulation/ugv/gps", self.on_gps, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, "/evaluation/ugv_truth", self.on_truth, qos_profile_sensor_data)
        self.create_timer(COMMAND_PERIOD, self.on_tick)

    def write(self, name, row):
        self.seen.add(name)
        if self.start is None or self.done:
            return
        self.writers[name].writerow(row)
        self.counts[name] += 1

    def on_clock(self, message):
        self.sim_time = seconds(message.clock)

    def on_imu(self, message):
        q, w, a = message.orientation, message.angular_velocity, message.linear_acceleration
        self.write("imu", [seconds(message.header.stamp), q.x, q.y, q.z, q.w,
                           w.x, w.y, w.z, a.x, a.y, a.z])

    def on_odom(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        v, w = message.twist.twist.linear, message.twist.twist.angular
        self.write("wheel_odom", [seconds(message.header.stamp), p.x, p.y, p.z,
                                  q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z])

    def on_gps(self, message):
        self.write("gps", [seconds(message.header.stamp), message.status.status,
                           message.latitude, message.longitude, message.altitude])

    def on_truth(self, message):
        p, q = message.pose.position, message.pose.orientation
        self.write("truth", [seconds(message.header.stamp), p.x, p.y, p.z,
                             q.x, q.y, q.z, q.w])

    def on_tick(self):
        if self.sim_time is None or self.done:
            return
        if self.start is None:
            # Start once every raw stream and truth is flowing, so each run has
            # the same stationary prefix.
            if not {"imu", "wheel_odom", "gps", "truth"} <= self.seen:
                return
            self.start = self.sim_time
            self.get_logger().info(f"Recording {self.args.duration:.0f} s from sim time {self.start:.2f}")
        elapsed = self.sim_time - self.start
        if elapsed >= self.args.duration:
            self.finish()
            return
        while self.segments[self.segment_index][1] <= elapsed:
            self.segment_index += 1
        linear, angular = target_command(self.segments[self.segment_index], elapsed)
        self.linear = step_toward(self.linear, linear, LINEAR_ACCEL * COMMAND_PERIOD)
        self.angular = step_toward(self.angular, angular, ANGULAR_ACCEL * COMMAND_PERIOD)
        command = Twist()
        command.linear.x = self.linear
        command.angular.z = self.angular
        self.cmd_pub.publish(command)
        self.write("cmd", [self.sim_time, self.linear, self.angular, self.segment_index])

    def finish(self):
        self.cmd_pub.publish(Twist())
        self.done = True
        for file in self.files.values():
            file.close()
        metadata = {
            "seed": self.args.seed, "duration_s": self.args.duration, "settle_s": self.args.settle,
            "world": self.args.world, "start_sim_time": self.start, "counts": self.counts,
            "frames": {"truth": "ENU map, UGV model origin",
                       "wheel_odom": "DiffDrive odom frame at spawn pose, FLU chassis twist",
                       "imu": "FLU chassis, orientation relative to ENU",
                       "gps": "WGS84 geodetic"},
            "segments": [{"start": s, "end": e, "kind": k, **p} for s, e, k, p in self.segments],
        }
        (self.output / "metadata.json").write_text(json.dumps(metadata, indent=2))
        self.get_logger().info(f"Wrote {self.output}: {self.counts}")
        missing = [name for name, count in self.counts.items() if not count]
        if missing:
            raise RuntimeError(f"No samples recorded for {missing}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--settle", type=float, default=3.0)
    parser.add_argument("--world", default="ugv_dataset")
    parser.add_argument("--output", required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = UgvDatasetRecorder(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
