"""Check the map-frame contracts used by the localization input adapter."""

import importlib.util
import math
import unittest
from pathlib import Path
from types import SimpleNamespace

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "ugv_measurements.py"
SPEC = importlib.util.spec_from_file_location("ugv_measurements", MODULE_PATH)
measurements = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(measurements)


class UgvMeasurementsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = measurements.UgvMeasurements()
        self.odom_messages = []
        self.gps_messages = []
        self.node.odom_pub = SimpleNamespace(publish=self.odom_messages.append)
        self.node.gps_pub = SimpleNamespace(publish=self.gps_messages.append)

    def tearDown(self):
        self.node.destroy_node()

    def test_wheel_odom_starts_at_spawn_and_uncertainty_grows_with_motion(self):
        self.node.spawn_x = -4.0
        self.node.dead_reckoned_x = -4.0
        self.node.odom_xy_stddev = 0.0
        self.node.odom_yaw_stddev = 0.0

        first = Odometry()
        first.pose.pose.orientation.w = 1.0
        self.node.on_odom(first)
        self.assertAlmostEqual(self.odom_messages[-1].pose.pose.position.x, -4.0)
        self.assertEqual(self.odom_messages[-1].header.frame_id, "map")

        self.node.odom_xy_stddev = 0.05
        moved = Odometry()
        moved.pose.pose.position.x = 2.0
        moved.pose.pose.orientation.w = 1.0
        self.node.on_odom(moved)
        result = self.odom_messages[-1]
        self.assertLess(abs(result.pose.pose.position.x - (-2.0)), 0.5)
        self.assertAlmostEqual(result.pose.covariance[0], 0.015)

    def test_gps_origin_projects_to_map_origin_and_no_fix_is_ignored(self):
        self.node.gps_xy_stddev = 0.0
        self.node.gps_z_stddev = 0.0
        fix = NavSatFix()
        fix.latitude = 47.397971057728974
        fix.longitude = 8.546163739800146
        self.node.on_gps(fix)
        position = self.gps_messages[-1].pose.pose.position
        self.assertLess(math.hypot(position.x, position.y), 1e-6)
        self.assertEqual(self.gps_messages[-1].header.frame_id, "map")

        fix.longitude += 0.0000133  # About one meter east at this latitude.
        self.node.on_gps(fix)
        self.assertAlmostEqual(self.gps_messages[-1].pose.pose.position.x, 1.0, delta=0.02)

        fix.status.status = NavSatStatus.STATUS_NO_FIX
        self.node.on_gps(fix)
        self.assertEqual(len(self.gps_messages), 2)

    def test_first_odom_sample_keeps_motion_before_adapter_starts(self):
        self.node.spawn_x = -4.0
        first = Odometry()
        first.pose.pose.position.x = 2.0
        first.pose.pose.orientation.w = 1.0
        self.node.on_odom(first)
        result = self.odom_messages[-1]
        self.assertAlmostEqual(result.pose.pose.position.x, -2.0)
        self.assertAlmostEqual(result.pose.covariance[0], 0.015)


if __name__ == "__main__":
    unittest.main()
