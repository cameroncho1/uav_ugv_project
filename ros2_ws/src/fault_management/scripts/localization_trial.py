#!/usr/bin/env python3
"""Drive a repeatable route and score any map-frame pose estimate against Gazebo truth."""

import argparse
from bisect import bisect_left
import csv
import json
import math
from collections import deque
from pathlib import Path

import rclpy
import numpy as np
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_srvs.srv import SetBool


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw(quaternion):
    return math.atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1 - 2 * (quaternion.y**2 + quaternion.z**2))


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def metrics(rows):
    if not rows:
        return {"samples": 0}
    errors = sorted(row["position_error_m"] for row in rows)
    headings = [row["heading_error_deg"] for row in rows]
    return {
        "samples": len(rows),
        "mean_error_m": sum(errors) / len(errors),
        "rmse_m": math.sqrt(sum(error * error for error in errors) / len(errors)),
        "p95_error_m": errors[min(len(errors) - 1, math.ceil(0.95 * len(errors)) - 1)],
        "max_error_m": errors[-1],
        "mean_heading_error_deg": sum(headings) / len(headings),
        **({"mean_planar_nees": sum(row["planar_nees"] for row in rows)/len(rows),
            "coverage_95": sum(row["planar_nees"] <= 5.991 for row in rows)/len(rows)}
           if "planar_nees" in rows[0] else {}),
    }


class LocalizationTrial(Node):
    def __init__(self, args):
        super().__init__("localization_trial")
        self.args = args
        self.start = None
        self.sim_time = None
        self.last_tick_time = None
        self.truth = deque(maxlen=20000)
        self.pending = deque()
        self.baseline_pending = deque()
        self.baseline_rows = []
        self.uav_count = 0
        self.uav_heading_count = 0
        self.uav_diagnostics = {}
        self.estimate_count = 0
        self.truth_count = 0
        self.skip_counts = {"before": 0, "after": 0, "gap": 0}
        self.rows = []
        self.done = False
        self.gps_request = None
        self.gps_disabled_at = None
        self.gps_after_outage = 0
        self.gps_count = 0
        self.wheel_count = 0
        self.wheel_after_outage = 0
        self.cmd_pub = self.create_publisher(Twist, "/evaluation/ugv_cmd_vel", 10)
        self.start_pub = self.create_publisher(
            Clock, "/evaluation/trial_start",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.gps_client = self.create_client(SetBool, "/evaluation/set_gps_enabled")
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/evaluation/ugv_truth", self.on_truth,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, args.estimate_topic,
                                 self.on_estimate, 10)
        if args.comparison_topic:
            self.create_subscription(PoseWithCovarianceStamped, args.comparison_topic,
                                     self.on_baseline, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/ugv/measurements/uav_pose",
                                 self.on_uav, 10)
        self.create_subscription(PoseWithCovarianceStamped,
                                 "/ugv/measurements/uav_pose_with_heading", self.on_uav_heading, 10)
        self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diagnostics, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/ugv/measurements/gps_pose",
                                 self.on_gps, 10)
        self.create_subscription(Odometry, "/ugv/measurements/wheel_odom", self.on_wheel, 10)

    def on_clock(self, message):
        self.sim_time = seconds(message.clock)
        if self.start is None and self.gps_client.service_is_ready():
            self.start = self.sim_time
            self.start_pub.publish(message)
        if self.start is not None and (self.last_tick_time is None or
                                       self.sim_time - self.last_tick_time >= 0.05):
            self.last_tick_time = self.sim_time
            self.tick()

    def on_truth(self, message):
        self.truth_count += 1
        self.truth.append((seconds(message.header.stamp), message.pose))

    def on_estimate(self, message):
        self.estimate_count += 1
        if message.header.frame_id == "map":
            self.pending.append(message)

    def on_gps(self, message):
        self.gps_count += 1
        stamp = seconds(message.header.stamp)
        if self.gps_disabled_at is not None and stamp > self.gps_disabled_at + 0.3:
            self.gps_after_outage += 1

    def on_baseline(self, message):
        if message.header.frame_id == "map":
            self.baseline_pending.append(message)

    def on_wheel(self, message):
        if (message.header.frame_id != "map" or
                message.child_frame_id != "simple_ugv/chassis"):
            raise RuntimeError("Wheel measurements must use map and simple_ugv/chassis; "
                               "check the Gazebo canonical link and DiffDrive frames")
        self.wheel_count += 1
        if (self.gps_disabled_at is not None and
                seconds(message.header.stamp) > self.gps_disabled_at + 0.3):
            self.wheel_after_outage += 1

    def on_uav(self, message):
        self.uav_count += 1

    def on_uav_heading(self, message):
        self.uav_heading_count += 1
        self.on_uav(message)

    def on_diagnostics(self, message):
        for status in message.status:
            if status.name == "cooperative_localization/uav_correction":
                self.uav_diagnostics = {value.key: int(value.value) for value in status.values}

    def score_pending(self):
        self.score_messages(self.pending, self.rows)
        self.score_messages(self.baseline_pending, self.baseline_rows)

    def score_messages(self, pending, rows):
        times = [sample[0] for sample in self.truth]
        while (pending and self.truth and
               seconds(pending[0].header.stamp) <= self.truth[-1][0]):
            estimate = pending.popleft()
            stamp = seconds(estimate.header.stamp)
            if stamp <= 0 or self.start is None:
                continue
            index = bisect_left(times, stamp)
            after = self.truth[index] if index < len(self.truth) else None
            before = self.truth[index-1] if index > 0 else None
            if after is not None and after[0] == stamp:
                before = after
            if before is None:
                self.skip_counts["before"] += 1
                continue
            if after is None:
                self.skip_counts["after"] += 1
                continue
            if after[0] - before[0] > 0.1 + 1e-9:
                self.skip_counts["gap"] += 1
                continue
            alpha = ((stamp - before[0]) / (after[0] - before[0])
                     if after[0] > before[0] else 0.0)
            x = before[1].position.x + alpha * (after[1].position.x - before[1].position.x)
            y = before[1].position.y + alpha * (after[1].position.y - before[1].position.y)
            true_yaw = yaw(before[1].orientation)
            true_yaw += alpha * angle_difference(yaw(after[1].orientation), true_yaw)
            pose = estimate.pose.pose
            delta = np.array([pose.position.x-x, pose.position.y-y])
            c = np.asarray(estimate.pose.covariance).reshape(6, 6)[:2, :2]
            if not np.all(np.isfinite(c)) or np.linalg.eigvalsh(c).min() <= 0:
                self.skip_counts["covariance"] = self.skip_counts.get("covariance", 0) + 1
                continue
            nees = float(delta @ np.linalg.solve(c, delta))
            rows.append({
                "elapsed_s": round(stamp - self.start, 4),
                "true_x_m": x, "true_y_m": y,
                "estimate_x_m": pose.position.x, "estimate_y_m": pose.position.y,
                "position_error_m": math.hypot(pose.position.x - x, pose.position.y - y),
                "heading_error_deg": math.degrees(abs(angle_difference(
                    yaw(pose.orientation), true_yaw))),
                "covariance_xx": c[0, 0], "covariance_xy": c[0, 1], "covariance_yy": c[1, 1],
                "planar_nees": nees,
            })

    def tick(self):
        if self.start is None or self.sim_time is None or self.done:
            return
        elapsed = self.sim_time - self.start
        command = Twist()
        route_time = elapsed if self.args.route == "short" else 4 + (elapsed-4) % 27
        forward = (4 <= route_time < 10 or 14 <= route_time < 22 or 25 <= route_time < 30)
        if elapsed >= 4 and forward:
            command.linear.x = 0.35
        elif elapsed >= 4 and (10 <= route_time < 14 or 22 <= route_time < 25):
            command.angular.z = 0.4
        self.cmd_pub.publish(command)
        if self.args.gps == "off" and elapsed >= 10 and self.gps_request is None:
            self.gps_request = self.gps_client.call_async(SetBool.Request(data=False))
            self.gps_disabled_at = self.sim_time
        self.score_pending()
        if elapsed >= self.args.duration:
            self.cmd_pub.publish(Twist())
            self.done = True

    def save(self):
        if self.args.gps == "off" and (self.gps_request is None or
                                       not self.gps_request.done() or
                                       not self.gps_request.result().success):
            raise RuntimeError("GPS disable request failed")
        if len(self.rows) < 100:
            raise RuntimeError(f"Only {len(self.rows)} matched truth samples; "
                               f"truth={self.truth_count}, estimates={self.estimate_count}, "
                               f"pending={len(self.pending)}, start={self.start}, "
                               f"skipped={self.skip_counts}, truth_span="
                               f"{(self.truth[0][0], self.truth[-1][0]) if self.truth else None}")
        if self.gps_after_outage:
            raise RuntimeError("Prepared GPS messages continued after GPS was disabled")
        lost_wheels = self.gps_disabled_at is not None and self.wheel_after_outage < 100
        if self.wheel_count < 100 or lost_wheels:
            raise RuntimeError("Missing wheel measurements during the localization trial")
        if self.args.gps == "startup_off" and self.gps_count:
            raise RuntimeError("Prepared GPS messages appeared in startup-off trial")
        if self.args.gps != "startup_off" and self.gps_count < 10:
            raise RuntimeError("Too few prepared GPS messages for a GPS-on interval")
        before_rows = [row for row in self.rows if 4 <= row["elapsed_s"] < 10]
        after_rows = [row for row in self.rows if 10.5 <= row["elapsed_s"] < self.args.duration]
        if len(before_rows) < 100 or len(after_rows) < 500:
            raise RuntimeError(
                f"Too few matched estimates: before={len(before_rows)}, after={len(after_rows)}")
        output = Path(self.args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.with_suffix(".csv").open("w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=self.rows[0].keys())
            writer.writeheader()
            writer.writerows(self.rows)
        summary = {
            "scenario": self.args.gps,
            "seed": self.args.seed,
            "estimate_topic": self.args.estimate_topic,
            "route": self.args.route,
            "duration_s": self.args.duration,
            "start_simulation_time_s": self.start,
            "measurement_profile": self.args.measurement_profile,
            "anchor_source": self.args.anchor_source,
            "relative_source": getattr(self.args, "relative_source", "simulated"),
            "uav_heading_enabled": self.args.uav_heading == "true",
            "uav_altitude_setpoint_m": getattr(self.args, "uav_altitude", None),
            "shared_position_stddev_m": getattr(self.args, "shared_position_stddev", 0.0),
            "systematic_position_stddev_m": getattr(self.args, "systematic_position_stddev", 0.0),
            "observer_model": self.args.observer_model or "fixed",
            "relative_outlier_every": self.args.outlier_every,
            "relative_dropout_interval_s": [self.args.dropout_start, self.args.dropout_end],
            "uav_messages": self.uav_count,
            "uav_heading_messages": self.uav_heading_count,
            "uav_diagnostics": self.uav_diagnostics,
            "gps_messages_after_disable": self.gps_after_outage,
            "gps_messages_total": self.gps_count,
            "wheel_messages_total": self.wheel_count,
            "wheel_messages_after_gps_disable": self.wheel_after_outage,
            "before_10s": metrics(before_rows),
            "after_10s": metrics(after_rows),
        }
        if self.args.comparison_topic:
            baseline_after = [row for row in self.baseline_rows
                              if 10.5 <= row["elapsed_s"] < self.args.duration]
            if len(baseline_after) < 500 or self.uav_count < 50:
                raise RuntimeError("Missing baseline estimates or UAV corrections")
            summary["comparison_topic"] = self.args.comparison_topic
            summary["baseline_after_10s"] = metrics(baseline_after)
            baseline_file = output.with_name(output.name + "_baseline").with_suffix(".csv")
            with baseline_file.open("w", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=self.baseline_rows[0].keys())
                writer.writeheader()
                writer.writerows(self.baseline_rows)
        output.with_suffix(".json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gps", choices=["on", "off", "startup_off"], required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--estimate-topic", default="/ugv/localization/pose")
    parser.add_argument("--comparison-topic", default="")
    parser.add_argument("--route", choices=["short", "long"], default="short")
    parser.add_argument("--duration", type=float, default=31.)
    parser.add_argument("--measurement-profile", choices=["nominal", "drift"], default="nominal")
    parser.add_argument("--anchor-source", default="unspecified")
    parser.add_argument("--relative-source", choices=["simulated", "camera"], default="simulated")
    parser.add_argument("--uav-heading", choices=["true", "false"], default="false")
    parser.add_argument("--uav-altitude", type=float, default=None)
    parser.add_argument("--shared-position-stddev", type=float, default=0.0)
    parser.add_argument("--systematic-position-stddev", type=float, default=0.0)
    parser.add_argument("--observer-model", default="")
    parser.add_argument("--outlier-every", type=int, default=0)
    parser.add_argument("--dropout-start", type=float, default=-1.)
    parser.add_argument("--dropout-end", type=float, default=-1.)
    parser.add_argument("--output", required=True)
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.shared_position_stddev) or args.shared_position_stddev < 0:
        parser.error("Shared position deviation must be finite and nonnegative")
    if args.duration < 31:
        parser.error("Duration must be at least 31 seconds")
    rclpy.init(args=ros_args)
    node = LocalizationTrial(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.score_pending()
        node.save()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
