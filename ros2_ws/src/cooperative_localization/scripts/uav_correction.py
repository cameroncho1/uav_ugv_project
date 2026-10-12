#!/usr/bin/env python3
"""Combine independently localized UAV poses and body-frame UGV observations."""

from collections import Counter, deque

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool

from localization_common.frames import covariance, interpolate, rotation, ugv_position


def stamp_ns(message):
    return message.header.stamp.sec*1000000000 + message.header.stamp.nanosec


def pose_state(message):
    p, q = message.pose.pose.position, message.pose.pose.orientation
    position = np.array([p.x, p.y, p.z])
    attitude = np.array([q.w, q.x, q.y, q.z])
    rotation(attitude)
    if not np.all(np.isfinite(position)):
        raise ValueError("Nonfinite position")
    return position, attitude / np.linalg.norm(attitude), covariance(message.pose.covariance, 6)


class UavCorrection(Node):
    def __init__(self):
        super().__init__("uav_correction")
        self.max_age = self.declare_parameter("max_age", .5).value
        self.max_gap = self.declare_parameter("max_anchor_gap", .25).value
        self.max_range = self.declare_parameter("max_range", 30.).value
        self.max_speed = self.declare_parameter("max_ugv_speed", 2.).value
        self.enabled = self.declare_parameter("enabled", True).value
        if not all(np.isfinite(value) and value > 0 for value in
                   (self.max_age, self.max_gap, self.max_range, self.max_speed)):
            raise ValueError("Correction limits must be finite and positive")
        self.anchors = deque(maxlen=200)
        self.pending = deque(maxlen=100)
        self.counters = Counter()
        self.previous = None
        self.last_clock = None
        self.publisher = self.create_publisher(PoseWithCovarianceStamped,
                                               "/ugv/measurements/uav_pose", 20)
        self.diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(PoseWithCovarianceStamped, "/uav/localization/pose",
                                 self.on_anchor, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "/ugv/sensors/uav_relative_pose",
                                 self.on_relative, qos_profile_sensor_data)
        self.create_service(SetBool, "/ugv/localization/set_uav_enabled", self.set_enabled)
        self.create_timer(.1, self.process)
        self.create_timer(1., self.report)

    def set_enabled(self, request, response):
        self.enabled = request.data
        self.pending.clear()
        self.previous = None
        response.success = True
        response.message = "UAV corrections enabled" if self.enabled else "UAV corrections paused"
        return response

    def valid_time(self, stamp):
        now = self.get_clock().now().nanoseconds
        if self.last_clock is not None and now < self.last_clock:
            self.anchors.clear()
            self.pending.clear()
            self.previous = None
        self.last_clock = now
        return stamp > 0 and -.05e9 <= now-stamp <= self.max_age*1e9

    def on_anchor(self, message):
        stamp = stamp_ns(message)
        try:
            if message.header.frame_id != "map" or not self.valid_time(stamp):
                raise ValueError("Wrong frame or stale anchor")
            state = pose_state(message)
            if self.anchors and stamp <= self.anchors[-1][0]:
                raise ValueError("Nonmonotonic anchor")
        except ValueError:
            self.counters["invalid_anchor"] += 1
            return
        self.anchors.append((stamp, state))
        self.process()

    def on_relative(self, message):
        stamp = stamp_ns(message)
        try:
            if message.header.frame_id != "uav/base_link" or not self.valid_time(stamp):
                raise ValueError("Wrong frame or stale relative reading")
            p = message.pose.pose.position
            relative = np.array([p.x, p.y, p.z])
            c = covariance(message.pose.covariance, 6)[:3, :3]
            if (not np.all(np.isfinite(relative)) or np.linalg.norm(relative) > self.max_range
                    or np.linalg.eigvalsh(c).min() <= 0):
                raise ValueError("Invalid relative measurement")
        except ValueError:
            self.counters["invalid_relative"] += 1
            return
        if self.enabled:
            self.pending.append((stamp, relative, c, message.header.stamp))
            self.process()

    def anchor_at(self, stamp):
        for i, (time, state) in enumerate(self.anchors):
            if time == stamp:
                return state
            if time > stamp:
                if i == 0:
                    return None
                before, first = self.anchors[i-1]
                if time-before > self.max_gap*1e9:
                    return None
                return interpolate(first, state, (stamp-before)/(time-before))
        return None

    def process(self):
        # Observe rewinds before taking a batch. valid_time() may clear queues.
        self.valid_time(0)
        batch = list(self.pending)
        self.pending.clear()
        for stamp, relative, c, ros_stamp in batch:
            if not self.valid_time(stamp):
                self.counters["expired"] += 1
                continue
            anchor = self.anchor_at(stamp)
            if anchor is None:
                self.pending.append((stamp, relative, c, ros_stamp))
                continue
            position, result_cov = ugv_position(anchor, relative, c)
            if self.previous is not None:
                previous_stamp, previous_position, previous_cov = self.previous
                if stamp <= previous_stamp:
                    self.counters["duplicate"] += 1
                    continue
                elapsed = (stamp-previous_stamp)*1e-9
                uncertainty = np.linalg.eigvalsh(result_cov[:2, :2] + previous_cov[:2, :2]).max()
                limit = self.max_speed*elapsed + 5*np.sqrt(uncertainty)
                if np.linalg.norm(position[:2]-previous_position[:2]) > limit:
                    self.counters["jump_rejected"] += 1
                    continue
            output = PoseWithCovarianceStamped()
            output.header.frame_id = "map"
            output.header.stamp = ros_stamp
            p = output.pose.pose.position
            p.x, p.y, p.z = (float(value) for value in position)
            output.pose.pose.orientation.w = 1.
            output_cov = np.zeros((6, 6))
            output_cov[:3, :3] = result_cov
            output_cov[3:, 3:] = np.eye(3)*1e6
            output.pose.covariance = output_cov.flatten().tolist()
            self.previous = (stamp, position, result_cov)
            self.counters["accepted"] += 1
            self.publisher.publish(output)

    def report(self):
        status = DiagnosticStatus(name="cooperative_localization/uav_correction",
                                  hardware_id="uav_relative_position")
        recent = (self.previous is not None and
                  self.get_clock().now().nanoseconds-self.previous[0] <= self.max_age*1e9)
        status.level = DiagnosticStatus.OK if self.enabled and recent else DiagnosticStatus.WARN
        status.message = "Receiving corrections" if recent else "Waiting for valid measurements"
        if not self.enabled:
            status.message = "Disabled"
        status.values = [KeyValue(key=key, value=str(value))
                         for key, value in self.counters.items()]
        report = DiagnosticArray(status=[status])
        report.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics.publish(report)


def main():
    rclpy.init()
    node = UavCorrection()
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
