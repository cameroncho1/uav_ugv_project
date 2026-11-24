#!/usr/bin/env python3
"""Explicit UAV-measurement outage control (fault class F4, ``uav_loss``).

Republishes the camera producer's UGV relative observations unchanged from
``input_topic`` to ``/ugv/sensors/uav_relative_pose`` while enabled. While
disabled, it drops them. ``/evaluation/set_uav_enabled`` (``std_srvs/SetBool``)
toggles the gate, and with ``evaluation_trace`` each change is published as a
``fault_event`` on ``/evaluation/faults`` in the same form as the GPS gate.
"""

import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool


class UavMeasurementGate(Node):
    def __init__(self):
        super().__init__("uav_measurement_gate")
        input_topic = self.declare_parameter(
            "input_topic", "/ugv/sensors/uav_relative_pose_raw").value
        output_topic = self.declare_parameter(
            "output_topic", "/ugv/sensors/uav_relative_pose").value
        self.enabled = self.declare_parameter("initial_uav_enabled", True).value
        trace = self.declare_parameter("evaluation_trace", False).value
        if input_topic == output_topic:
            raise ValueError("input_topic and output_topic must differ")

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, output_topic, 10)
        self.create_subscription(PoseWithCovarianceStamped, input_topic, self.on_measurement, 10)
        self.create_service(SetBool, "/evaluation/set_uav_enabled", self.set_enabled)
        self.dropped = 0

        self.trace_sequence = 0
        self.active_event = None if self.enabled else "uav-initial"
        trace_qos = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.trace_pub = (self.create_publisher(String, "/evaluation/faults", trace_qos)
                          if trace else None)
        self.trace_host = (Path("/proc/sys/kernel/random/boot_id").read_text().strip()
                           if self.trace_pub else "")
        self.trace_initial = self.trace_pub is not None
        if self.trace_pub:
            self.create_timer(0.1, self.publish_initial_state)
        self.get_logger().info(
            f"UAV measurements {input_topic} -> {output_topic} "
            + ("enabled" if self.enabled else "disabled at start"))

    def on_measurement(self, message):
        if self.enabled:
            self.publisher.publish(message)
        else:
            self.dropped += 1

    def set_enabled(self, request, response):
        self.publish_initial_state()
        changed = self.enabled != request.data
        self.enabled = request.data
        if changed:
            if not request.data:
                self.active_event = f"uav-{self.trace_sequence}"
            self.publish_event("fault_end" if request.data else "fault_start")
            if request.data:
                self.active_event = None
            self.get_logger().info(
                "UAV measurements enabled" if request.data
                else "UAV measurements disabled (fault F4)")
        response.success = True
        response.message = "UAV enabled" if self.enabled else "UAV disabled"
        return response

    def publish_initial_state(self):
        # Wait for /clock so the first event carries a simulation stamp.
        if self.trace_initial and self.get_clock().now().nanoseconds > 0:
            self.trace_initial = False
            self.publish_event("state" if self.enabled else "fault_start")

    def publish_event(self, kind):
        if self.trace_pub is None:
            return
        message = String()
        message.data = json.dumps({"schema_version": 1, "kind": "fault_event",
            "sequence": self.trace_sequence, "host_id": self.trace_host,
            "sim_ns": self.get_clock().now().nanoseconds, "steady_ns": time.monotonic_ns(),
            "event": {"kind": kind, "event_id": self.active_event or "uav-nominal",
                      "sensor": "uav", "channel": "relative_pose", "fault_class": "uav_loss",
                      "enabled": self.enabled}}, allow_nan=False)
        self.trace_sequence += 1
        self.trace_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = UavMeasurementGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
