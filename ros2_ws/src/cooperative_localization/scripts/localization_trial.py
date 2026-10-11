#!/usr/bin/env python3
"""Drive a repeatable route and score any map-frame pose estimate against Gazebo truth."""

import argparse
import csv
import json
import math
from collections import deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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
    }


class LocalizationTrial(Node):
    def __init__(self, args):
        super().__init__("localization_trial")
        self.args = args
        self.start = None
        self.sim_time = None
        self.last_tick_time = None
        self.truth = deque(maxlen=3000)
        self.pending = deque()
        self.estimate_count = 0
        self.truth_count = 0
        self.skip_counts = {"before": 0, "after": 0, "gap": 0}
        self.rows = []
        self.done = False
        self.gps_request = None
        self.gps_disabled_at = None
        self.gps_after_outage = 0
        self.gps_count = 0
        self.cmd_pub = self.create_publisher(Twist, "/evaluation/ugv_cmd_vel", 10)
        self.gps_client = self.create_client(SetBool, "/ugv/localization/set_gps_enabled")
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/evaluation/ugv_truth", self.on_truth,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, args.estimate_topic,
                                 self.on_estimate, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/ugv/measurements/gps_pose",
                                 self.on_gps, 10)

    def on_clock(self, message):
        self.sim_time = seconds(message.clock)
        if self.start is None and self.gps_client.service_is_ready():
            self.start = self.sim_time
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

    def score_pending(self):
        while (self.pending and self.truth and
               seconds(self.pending[0].header.stamp) <= self.truth[-1][0]):
            estimate = self.pending.popleft()
            stamp = seconds(estimate.header.stamp)
            if stamp <= 0 or self.start is None:
                continue
            # Estimates arrive in stamp order. Keep only the two truth samples
            # around the next estimate instead of searching the full history.
            while len(self.truth) > 1 and self.truth[1][0] <= stamp:
                self.truth.popleft()
            before = self.truth[0] if self.truth[0][0] <= stamp else None
            after = self.truth[1] if len(self.truth) > 1 else None
            if before is None:
                self.skip_counts["before"] += 1
                continue
            if before[0] == stamp:
                after = before
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
            self.rows.append({
                "elapsed_s": round(stamp - self.start, 4),
                "true_x_m": x, "true_y_m": y,
                "estimate_x_m": pose.position.x, "estimate_y_m": pose.position.y,
                "position_error_m": math.hypot(pose.position.x - x, pose.position.y - y),
                "heading_error_deg": math.degrees(abs(angle_difference(
                    yaw(pose.orientation), true_yaw))),
            })

    def tick(self):
        if self.start is None or self.sim_time is None or self.done:
            return
        elapsed = self.sim_time - self.start
        command = Twist()
        if 4 <= elapsed < 10 or 14 <= elapsed < 22 or 25 <= elapsed < 30:
            command.linear.x = 0.35
        elif 10 <= elapsed < 14 or 22 <= elapsed < 25:
            command.angular.z = 0.4
        self.cmd_pub.publish(command)
        if self.args.gps == "off" and elapsed >= 10 and self.gps_request is None:
            self.gps_request = self.gps_client.call_async(SetBool.Request(data=False))
            self.gps_disabled_at = self.sim_time
        self.score_pending()
        if elapsed >= 31:
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
        if self.args.gps == "startup_off" and self.gps_count:
            raise RuntimeError("Prepared GPS messages appeared in startup-off trial")
        if self.args.gps != "startup_off" and self.gps_count < 10:
            raise RuntimeError("Too few prepared GPS messages for a GPS-on interval")
        before_rows = [row for row in self.rows if 4 <= row["elapsed_s"] < 10]
        after_rows = [row for row in self.rows if 10.5 <= row["elapsed_s"] < 31]
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
            "gps_messages_after_disable": self.gps_after_outage,
            "gps_messages_total": self.gps_count,
            "before_10s": metrics(before_rows),
            "after_10s": metrics(after_rows),
        }
        output.with_suffix(".json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gps", choices=["on", "off", "startup_off"], required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--estimate-topic", default="/ugv/localization/pose")
    parser.add_argument("--output", required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = LocalizationTrial(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.score_pending()
        node.save()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
