#!/usr/bin/env python3
"""Produce body-relative UGV measurements from calibrated camera pixels."""

from collections import Counter
from array import array

import cv2
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster

from fiducial_math import Calibration, annotate_heading, detect_marker, solve_marker_pose
from localization_common.frames import quaternion


class FiducialSensor(Node):
    def __init__(self):
        super().__init__('uav_fiducial_sensor')
        cv2.setNumThreads(1)
        defaults = vars(Calibration())
        self.calibration = Calibration(**{
            name: self.declare_parameter(name, value).value for name, value in defaults.items()})
        self.marker_id = self.declare_parameter('marker_id', 7).value
        rotation = self.declare_parameter(
            'body_from_optical_rotation', [0., -1., 0., -1., 0., 0., 0., 0., -1.]).value
        self.rotation = np.array(rotation).reshape(3, 3)
        self.translation = self.declare_parameter(
            'body_from_optical_translation', [0., 0., -0.18]).value
        self.offset = self.declare_parameter('marker_to_chassis', [0., 0., -0.12]).value
        self.marker_rotation = np.array(self.declare_parameter(
            'marker_from_chassis_rotation', np.eye(3).ravel().tolist()).value).reshape(3, 3)
        if (not np.isfinite(self.rotation).all() or
                not np.allclose(self.rotation.T @ self.rotation, np.eye(3), atol=1e-6) or
                not np.isclose(np.linalg.det(self.rotation), 1.0) or
                np.shape(self.translation) != (3,) or np.shape(self.offset) != (3,) or
                not np.isfinite(self.translation).all() or not np.isfinite(self.offset).all() or
                not np.isfinite(self.marker_rotation).all() or
                not np.allclose(self.marker_rotation.T @ self.marker_rotation, np.eye(3), atol=1e-6) or
                not np.isclose(np.linalg.det(self.marker_rotation), 1.0)):
            raise ValueError('Invalid camera mount or marker offset')
        self.input_frame = self.declare_parameter('input_frame', 'uav/camera_optical_frame').value
        if not self.input_frame or self.input_frame == 'uav/base_link':
            raise ValueError('Camera input requires a distinct optical frame')
        self.mount_broadcaster = StaticTransformBroadcaster(self)
        mount = TransformStamped()
        mount.header.stamp = self.get_clock().now().to_msg()
        mount.header.frame_id = 'uav/base_link'
        mount.child_frame_id = self.input_frame
        position = mount.transform.translation
        position.x, position.y, position.z = (float(value) for value in self.translation)
        w, x, y, z = quaternion(self.rotation)
        mount.transform.rotation.w = float(w)
        mount.transform.rotation.x = float(x)
        mount.transform.rotation.y = float(y)
        mount.transform.rotation.z = float(z)
        self.mount_broadcaster.sendTransform(mount)
        self.max_age = self.declare_parameter('max_image_age', 0.5).value
        self.rate = self.declare_parameter('max_processing_rate', 5.0).value
        if not (np.isfinite(self.max_age) and self.max_age > 0 and
                np.isfinite(self.rate) and self.rate > 0 and 0 <= self.marker_id < 587):
            raise ValueError('Invalid fiducial timing or marker ID')
        self.info = None
        self.last_stamp = None
        self.last_clock = None
        self.counters = Counter()
        self.last_error = 0.0
        self.last_accepted = None
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/ugv/sensors/uav_relative_pose', 10)
        self.annotated_publisher = self.create_publisher(
            Image, '/uav/camera/heading_image', qos_profile_sensor_data)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(CameraInfo, '/uav/camera/camera_info',
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/uav/camera/image', self.on_image,
                                 qos_profile_sensor_data)
        self.create_timer(1.0, self.report)

    def on_info(self, message):
        self.info = message

    def on_image(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_clock is not None and now < self.last_clock:
            self.last_stamp = None
            self.last_accepted = None
        self.last_clock = now
        if stamp <= 0 or not -0.05 <= now - stamp <= self.max_age:
            self.counters['stale'] += 1
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.counters['nonmonotonic'] += 1
            return
        if self.last_stamp is not None and stamp - self.last_stamp < 1 / self.rate - 1e-6:
            return
        self.last_stamp = stamp
        self.counters['processed'] += 1
        info = self.info
        if (info is None or info.header.frame_id != message.header.frame_id or
                message.header.frame_id != self.input_frame or
                (info.width, info.height) != (message.width, message.height) or
                info.distortion_model not in ('plumb_bob', '')):
            self.counters['missing_calibration'] += 1
            return
        channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}
        count = channels.get(message.encoding)
        if (count is None or message.width == 0 or message.height == 0 or
                message.step < message.width * count or
                len(message.data) != message.step * message.height):
            self.counters['invalid_image'] += 1
            return
        pixels = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
        image = pixels[:, :message.width * count].reshape(message.height, message.width, count)
        if count == 1:
            image = image[:, :, 0]
        else:
            code = cv2.COLOR_RGB2GRAY if message.encoding.startswith('rgb') else cv2.COLOR_BGR2GRAY
            image = cv2.cvtColor(image, code)
        corners = detect_marker(image, self.marker_id)
        if corners is None:
            self.counters['not_visible'] += 1
            self.publish_annotation(image, message)
            return
        try:
            observation = solve_marker_pose(
                corners, info.k, info.d, self.rotation, self.translation,
                self.offset, self.marker_rotation, self.calibration)
        except (ValueError, cv2.error, np.linalg.LinAlgError):
            self.counters['rejected_geometry'] += 1
            self.publish_annotation(image, message)
            return
        if self.get_clock().now().nanoseconds * 1e-9 - stamp > self.max_age:
            self.counters['stale'] += 1
            return
        output = PoseWithCovarianceStamped()
        output.header.stamp = message.header.stamp
        output.header.frame_id = 'uav/base_link'
        output.pose.pose.position.x, output.pose.pose.position.y, output.pose.pose.position.z = (
            float(value) for value in observation.position)
        q = output.pose.pose.orientation
        q.w, q.x, q.y, q.z = (float(value) for value in observation.attitude)
        output.pose.covariance = observation.covariance.ravel().tolist()
        self.publisher.publish(output)
        self.publish_annotation(image, message, observation)
        self.counters['accepted'] += 1
        self.last_accepted = stamp
        self.last_error = observation.error

    def publish_annotation(self, image, source, observation=None):
        if not self.annotated_publisher.get_subscription_count():
            return
        pixels = annotate_heading(image, observation)
        output = Image()
        output.header = source.header
        output.height, output.width = pixels.shape[:2]
        output.encoding = 'bgr8'
        output.step = output.width * 3
        # Typed storage avoids Python's per-byte ROS sequence validation on a
        # multi-megabyte frame, keeping visualization off the sensing deadline.
        output.data = array('B', pixels.tobytes())
        self.annotated_publisher.publish(output)

    def report(self):
        status = DiagnosticStatus(name='uav_fiducial_sensor',
                                  message='Camera-derived relative pose; counts are cumulative')
        age = (self.get_clock().now().nanoseconds * 1e-9 - self.last_accepted
               if self.last_accepted is not None else float('inf'))
        fresh = 0 <= age <= self.max_age
        status.level = DiagnosticStatus.OK if fresh else DiagnosticStatus.WARN
        status.message = ('Target visible' if fresh else
                          'Target unavailable; no relative observation')
        status.values = [KeyValue(key=k, value=str(v)) for k, v in self.counters.items()]
        status.values.append(KeyValue(key='reprojection_error_pixels', value=str(self.last_error)))
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostics.publish(message)


def main():
    rclpy.init()
    node = FiducialSensor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
