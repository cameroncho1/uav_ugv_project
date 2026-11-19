#!/usr/bin/env python3
"""Flag UGV sensor-channel faults live with the exported BiLSTM / autoencoder.

Buffers the prepared UGV measurements (``/ugv/sensors/*``) and the drive
command, rebuilds each channel's last ``window_len`` samples with its training
reference (``sensor_fault_model.reference``), and publishes one
``DiagnosticStatus`` per channel on ``/ugv/sensor_faults``. Each evaluation
also publishes one ``uav_ugv_msgs/FaultStatus`` class decision on
``/fault_detection/status`` for the navigation supervisor
(``sensor_fault_model.fault_decision``). Inputs are the stated measurement
interfaces only; simulator truth is not used.
"""

import math
import uuid
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from uav_ugv_msgs.msg import FaultStatus

from sensor_fault_model import (CHANNELS, SensorFaultModel, channel_name, fault_decision,
                                reference)

# Channels whose training windows had a reference; they wait until it exists.
REFERENCED = {("imu", "wz"), ("imu", "ax"), ("imu", "ay"), ("wheel_odom", "vx"),
              ("wheel_odom", "wz"), ("gps", "latitude"), ("gps", "longitude")}
FIELDS = {
    "imu": ("wx", "wy", "wz", "ax", "ay", "az"),
    "wheel_odom": ("x", "y", "vx", "wz"),
    "gps": ("latitude", "longitude", "altitude"),
    "cmd": ("linear", "angular"),
}


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class StreamBuffer:
    """Time-ordered samples of one stream, pruned to ``retention`` seconds."""

    def __init__(self, fields, retention):
        self.fields = fields
        self.retention = retention
        self.samples = deque()

    def add(self, stamp, values):
        if self.samples and stamp <= self.samples[-1][0]:
            return False
        self.samples.append((stamp, values))
        while self.samples and self.samples[0][0] < stamp - self.retention:
            self.samples.popleft()
        return True

    def clear(self):
        self.samples.clear()

    def latest(self):
        return self.samples[-1][0] if self.samples else None

    def arrays(self):
        if not self.samples:
            return None
        data = np.array([(stamp, *values) for stamp, values in self.samples], dtype=np.float64)
        arrays = {"stamp": data[:, 0]}
        arrays.update({field: data[:, i + 1] for i, field in enumerate(self.fields)})
        return arrays


class SensorFaultDetector(Node):
    def __init__(self):
        super().__init__("sensor_fault_detector")
        default_model = (Path(get_package_share_directory("fault_management"))
                         / "models/ugv_sensor_fault_detector.npz")
        self.declare_parameter("model_path", str(default_model))
        self.declare_parameter("command_topic", "/ugv/cmd_vel")
        self.declare_parameter("output_topic", "/ugv/sensor_faults")
        self.declare_parameter("status_topic", "/fault_detection/status")
        self.declare_parameter("gps_timeout", 1.0)
        self.declare_parameter("command_rate", 20.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("evaluation_period", 0.5)
        self.declare_parameter("stale_after", 3.0)
        self.declare_parameter("use_autoencoder", False)
        self.declare_parameter("origin_latitude", float("nan"))
        self.declare_parameter("origin_longitude", float("nan"))

        def parameter(name):
            return self.get_parameter(name).value

        self.model = SensorFaultModel.load(parameter("model_path"))
        self.use_autoencoder = parameter("use_autoencoder")
        if self.use_autoencoder and not self.model.has_autoencoder:
            raise ValueError("model has no autoencoder weights")
        origin = (parameter("origin_latitude"), parameter("origin_longitude"))
        self.origin = tuple(self.model.metadata["origin"][i] if math.isnan(value) else value
                            for i, value in enumerate(origin))
        self.command_timeout = parameter("command_timeout")
        self.stale_after = parameter("stale_after")
        self.gps_timeout = parameter("gps_timeout")
        # GPS windows span about 20 s; their odometry reference must cover them.
        self.streams = {name: StreamBuffer(fields, 45.0) for name, fields in FIELDS.items()}
        self.command = (0.0, 0.0)
        self.command_time = None
        self.last_now = None
        self.evaluated = {}
        self.state = {}
        self.faulted = set()
        self.decisions = {}   # channel -> (fault, window_end) of its latest evaluation
        self.start_session()

        self.publisher = self.create_publisher(DiagnosticArray, parameter("output_topic"), 10)
        self.status_publisher = self.create_publisher(FaultStatus, parameter("status_topic"), 20)
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Imu, "/ugv/sensors/imu", self.on_imu, qos)
        self.create_subscription(Odometry, "/ugv/sensors/wheel_odom", self.on_odom, qos)
        self.create_subscription(NavSatFix, "/ugv/sensors/gps", self.on_gps, qos)
        self.create_subscription(Twist, parameter("command_topic"), self.on_command, 10)
        self.create_timer(1.0 / parameter("command_rate"), self.sample_command)
        self.create_timer(parameter("evaluation_period"), self.evaluate)
        meta = self.model.metadata
        self.get_logger().info(
            f"Loaded {meta['bilstm']}"
            + (f" + {meta['autoencoder']}" if self.use_autoencoder else "")
            + f" ({self.model.window_len}-sample windows); commands from "
            f"{parameter('command_topic')}")

    def start_session(self):
        # A new producer identity after a clock reset; sequence and stamps restart.
        self.source_id = f"sensor_fault_detector/{uuid.uuid4().hex[:12]}"
        self.sequence = 0
        self.last_decision_stamp = 0.0
        self.decision_valid = None

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_imu(self, message):
        w, a = message.angular_velocity, message.linear_acceleration
        if message.angular_velocity_covariance[0] == -1 or \
                message.linear_acceleration_covariance[0] == -1:
            return
        self.streams["imu"].add(stamp_seconds(message.header.stamp),
                                (w.x, w.y, w.z, a.x, a.y, a.z))

    def on_odom(self, message):
        p, twist = message.pose.pose.position, message.twist.twist
        self.streams["wheel_odom"].add(stamp_seconds(message.header.stamp),
                                       (p.x, p.y, twist.linear.x, twist.angular.z))

    def on_gps(self, message):
        values = (message.latitude, message.longitude, message.altitude)
        if message.status.status == NavSatStatus.STATUS_NO_FIX or \
                not all(math.isfinite(v) for v in values):
            return
        self.streams["gps"].add(stamp_seconds(message.header.stamp), values)

    def on_command(self, message):
        self.command = (message.linear.x, message.angular.z)
        self.command_time = self.now_seconds()

    def sample_command(self):
        # Training logged the drive command at 20 Hz on sim time; resample the
        # latest command the same way. A silent command source means zero.
        now = self.now_seconds()
        if self.last_now is not None and now < self.last_now:
            self.get_logger().warn("Clock moved backwards; clearing buffers")
            for stream in self.streams.values():
                stream.clear()
            self.evaluated.clear()
            self.decisions.clear()
            self.command_time = None
            self.start_session()
        self.last_now = now
        if now <= 0.0:
            return
        fresh = self.command_time is not None and now - self.command_time <= self.command_timeout
        self.streams["cmd"].add(now, self.command if fresh else (0.0, 0.0))

    def evaluate(self):
        now = self.now_seconds()
        arrays = {name: stream.arrays() for name, stream in self.streams.items()}
        arrays = {name: value for name, value in arrays.items() if value is not None}
        windows, info, statuses = {}, {}, {}
        n = self.model.window_len
        for channel in CHANNELS:
            sensor, column = channel
            source = arrays.get(sensor)
            if source is None or len(source["stamp"]) < n:
                statuses[channel] = (DiagnosticStatus.STALE, "warming up", [])
                continue
            end = float(source["stamp"][-1])
            if now - end > self.stale_after:
                statuses[channel] = (DiagnosticStatus.STALE, "no recent samples", [])
                continue
            stamps = source["stamp"][-n:]
            ref = reference(channel, stamps, arrays, self.origin)
            if ref is None and channel in REFERENCED:
                statuses[channel] = (DiagnosticStatus.STALE, "waiting for reference", [])
                continue
            if self.evaluated.get(channel) == end and channel in self.state:
                continue   # no new samples; keep the previous result
            windows[channel] = np.stack(
                [source[column][-n:], np.full(n, np.nan) if ref is None else ref], axis=1)
            info[channel] = (float(stamps[0]), end)
        for channel, result in self.model.evaluate(windows).items():
            self.evaluated[channel] = info[channel][1]
            fault = result["bilstm"] or (self.use_autoencoder and result.get("autoencoder", False))
            self.decisions[channel] = (bool(fault), info[channel][1])
            values = [("fault_probability", f"{result['probability']:.4f}"),
                      ("window_start", f"{info[channel][0]:.3f}"),
                      ("window_end", f"{info[channel][1]:.3f}")]
            if self.use_autoencoder:
                values += [("ae_error", f"{result['ae_error']:.5f}"),
                           ("ae_threshold", f"{result['ae_threshold']:.5f}")]
            level = DiagnosticStatus.ERROR if fault else DiagnosticStatus.OK
            self.state[channel] = (level, "fault" if fault else "ok", values)
            self.report_transition(channel, fault, result, info[channel])
        for channel in CHANNELS:
            if channel not in statuses:
                statuses[channel] = self.state.get(
                    channel, (DiagnosticStatus.STALE, "warming up", []))
        self.publish(statuses)
        self.publish_decision(now, statuses)

    def publish_decision(self, now, statuses):
        if now <= 0.0:
            return
        current = {channel: self.decisions[channel] for channel in CHANNELS
                   if channel in self.decisions and statuses[channel][0] != DiagnosticStatus.STALE}
        gps_latest = self.streams["gps"].latest()
        gps_available = gps_latest is not None and now - gps_latest <= self.gps_timeout
        valid, stamp, flags = fault_decision(current, gps_available)
        if not valid:
            stamp = now
        # Stamps strictly increase within a session; without newer evidence the
        # supervisor's decision-age limit turns the silence into a hold.
        if stamp <= self.last_decision_stamp:
            return
        if valid != self.decision_valid:
            self.get_logger().info("Fault decisions " + ("valid" if valid else
                                   "invalid: yaw-rate or wheel-odometry windows missing or stale"))
            self.decision_valid = valid
        self.sequence += 1
        self.last_decision_stamp = stamp
        message = FaultStatus(schema_version=1, source_id=self.source_id,
                              sequence=self.sequence, window_valid=valid, **flags)
        message.header.stamp = Time(nanoseconds=round(stamp * 1e9)).to_msg()
        self.status_publisher.publish(message)

    def report_transition(self, channel, fault, result, window):
        name = channel_name(channel)
        if fault and name not in self.faulted:
            self.faulted.add(name)
            self.get_logger().warn(
                f"Fault on {name}: p={result['probability']:.2f}"
                + (f", ae {result['ae_error']:.3f}/{result['ae_threshold']:.3f}"
                   if self.use_autoencoder else "")
                + f", window {window[0]:.1f}-{window[1]:.1f} s")
        elif not fault and name in self.faulted:
            self.faulted.discard(name)
            self.get_logger().info(f"{name} back to normal (window ends {window[1]:.1f} s)")

    def publish(self, statuses):
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        for channel in CHANNELS:
            level, text, values = statuses[channel]
            status = DiagnosticStatus(level=level, name=f"ugv_sensor_fault/{channel_name(channel)}",
                                      message=text, hardware_id="ugv")
            status.values = [KeyValue(key=k, value=v) for k, v in values]
            message.status.append(status)
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SensorFaultDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        # A timer can publish after Ctrl-C has invalidated the context.
        if node.context.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
