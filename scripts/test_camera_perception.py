#!/usr/bin/env python3
"""Score a rendered metric target and real visual occlusion in the camera fixture."""

import argparse
import json
import math
from pathlib import Path
import subprocess
import time

import cv2
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticArray


class CameraTrial(Node):
    def __init__(self):
        super().__init__('camera_perception_trial')
        self.observations = []
        self.image = None
        self.image_count = 0
        self.heading_image = None
        self.results = []
        self.diagnostics = {}
        self.create_subscription(PoseWithCovarianceStamped, '/ugv/sensors/uav_relative_pose',
                                 self.observations.append, 10)
        self.create_subscription(Image, '/uav/camera/image', self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '/uav/camera/heading_image', self.on_heading_image,
                                 qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, 10)

    def on_image(self, message):
        self.image = message
        self.image_count += 1

    def on_heading_image(self, message):
        self.heading_image = message

    def on_diagnostics(self, message):
        for status in message.status:
            if status.name == 'uav_fiducial_sensor':
                self.diagnostics = {value.key: value.value for value in status.values}

    def save_image(self, path):
        if self.image is None:
            return
        image = self.image
        channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}[image.encoding]
        pixels = np.frombuffer(image.data, np.uint8).reshape(image.height, image.step)
        pixels = pixels[:, :image.width * channels].reshape(image.height, image.width, channels)
        if image.encoding.startswith('rgb'):
            code = cv2.COLOR_RGBA2BGR if channels == 4 else cv2.COLOR_RGB2BGR
            pixels = cv2.cvtColor(pixels, code)
        cv2.imwrite(str(path), pixels)
        if self.heading_image is not None:
            annotated = np.frombuffer(self.heading_image.data, np.uint8).reshape(
                self.heading_image.height, self.heading_image.width, 3)
            cv2.imwrite(str(path.with_name(path.stem + '_heading.png')), annotated)

    def collect(self, seconds):
        start = time.monotonic()
        while time.monotonic() - start < seconds:
            rclpy.spin_once(self, timeout_sec=0.1)

    def collect_frames(self, count):
        start = self.image_count
        deadline = time.monotonic() + 30
        while self.image_count - start < count:
            assert time.monotonic() < deadline, 'Camera frames stopped arriving'
            rclpy.spin_once(self, timeout_sec=0.1)

    @staticmethod
    def move(name, position, yaw=0.):
        x, y, z = position
        request = (f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
                   f'orientation: {{w: {math.cos(yaw/2)}, x: 0, y: 0, z: {math.sin(yaw/2)}}}')
        result = subprocess.run(
            ['gz', 'service', '-s', '/world/fiducial_fixture/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000', '--req', request], capture_output=True, text=True, timeout=5)
        assert result.returncode == 0 and 'data: true' in result.stdout, result.stderr

    def phase(self, name, visible, expected, expected_yaw=0.):
        self.collect_frames(3)
        self.collect(0.2)
        self.observations.clear()
        missing_before = int(self.diagnostics.get('not_visible', 0))
        self.collect_frames(12)
        self.collect(0.5)
        assert self.image is not None, 'Camera frames are absent'
        result = {'case': name, 'observations': len(self.observations), 'visible': visible}
        self.results.append(result)
        result['diagnostics'] = self.diagnostics.copy()
        if visible:
            assert len(self.observations) >= 5, f'{name}: missing detections'
            points = np.array([[m.pose.pose.position.x, m.pose.pose.position.y,
                                m.pose.pose.position.z] for m in self.observations])
            errors = np.linalg.norm(points - expected, axis=1)
            result.update(rmse_3d_m=float(np.sqrt(np.mean(errors**2))),
                          max_error_3d_m=float(errors.max()))
            assert errors.max() < 0.15, f'{name}: incorrect camera geometry'
            for m in self.observations:
                assert m.header.frame_id == 'uav/base_link'
                covariance = np.array(m.pose.covariance).reshape(6, 6)
                assert np.linalg.eigvalsh(covariance).min() > 0
            headings = []
            for m in self.observations:
                q = m.pose.pose.orientation
                heading = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                headings.append(math.atan2(math.sin(heading-expected_yaw),
                                            math.cos(heading-expected_yaw)))
            result['max_heading_error_deg'] = math.degrees(max(map(abs, headings)))
            assert result['max_heading_error_deg'] < 5., f'{name}: incorrect chassis heading'
            assert self.heading_image is not None, 'Heading overlay is absent'
            assert self.heading_image.header.frame_id == 'uav/camera_optical_frame'
        else:
            assert not self.observations, f'{name}: occluded target produced observations'
            missing = int(self.diagnostics.get('not_visible', 0)) - missing_before
            assert missing >= 5, f'{name}: detector did not process enough occluded frames'
        result['passed'] = True
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = CameraTrial()
    try:
        deadline = time.monotonic() + 30
        while node.image is None:
            assert time.monotonic() < deadline, 'Camera discovery timed out'
            node.collect(0.1)
        # The image bridge can become ready before Python detector startup.
        deadline = time.monotonic() + 30
        while len(node.observations) < 3:
            assert time.monotonic() < deadline, 'Detector startup timed out'
            node.collect(0.1)
        results = [node.phase('visible', True, [0, 0, -3.75])]
        args.output.parent.mkdir(parents=True, exist_ok=True)
        node.save_image(args.output.with_suffix('.png'))
        node.move('occluder', [0.28, 0, 1])
        results.append(node.phase('partial_occlusion', False, None))
        node.move('occluder', [0, 0, 1])
        results.append(node.phase('full_occlusion', False, None))
        node.move('occluder', [10, 10, 1])
        results.append(node.phase('recovery', True, [0, 0, -3.75]))
        node.move('target', [1, -0.5, 0.37])
        results.append(node.phase('off_axis', True, [1, -0.5, -3.75]))
        for name, yaw in [('left', math.pi/2), ('reverse', math.pi), ('right', -math.pi/2)]:
            node.move('target', [0, 0, 0.37], yaw)
            results.append(node.phase(name, True, [0, 0, -3.75], yaw))
        args.output.write_text(json.dumps({'cases': results, 'passed': True}, indent=2) + '\n')
        print(json.dumps(results))
    except Exception as error:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        node.save_image(args.output.with_name('failure_frame.png'))
        args.output.write_text(json.dumps({'cases': node.results, 'passed': False,
                                          'error': str(error)}, indent=2) + '\n')
        raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
