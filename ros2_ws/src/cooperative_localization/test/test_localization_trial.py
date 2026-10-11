"""Check timestamp and error calculations used by live localization trials."""

import importlib.util
import math
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "localization_trial.py"
SPEC = importlib.util.spec_from_file_location("localization_trial", MODULE_PATH)
trial = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(trial)


class LocalizationTrialTest(unittest.TestCase):
    def test_metrics_use_each_sample_and_report_tail_error(self):
        rows = [{"position_error_m": error, "heading_error_deg": 2.0}
                for error in [0.0, 1.0, 2.0, 10.0]]
        result = trial.metrics(rows)
        self.assertEqual(result["samples"], 4)
        self.assertAlmostEqual(result["mean_error_m"], 3.25)
        self.assertAlmostEqual(result["rmse_m"], math.sqrt(105 / 4))
        self.assertEqual(result["p95_error_m"], 10.0)
        self.assertEqual(result["mean_heading_error_deg"], 2.0)

    def test_heading_error_wraps_at_half_turn(self):
        self.assertAlmostEqual(math.degrees(abs(trial.angle_difference(
            math.radians(179), math.radians(-179)))), 2.0)

    def test_estimate_is_compared_with_interpolated_truth_at_its_stamp(self):
        first = Pose()
        first.orientation.w = 1.0
        second = Pose()
        second.position.x = 2.0
        second.orientation.w = 1.0
        estimate = PoseWithCovarianceStamped()
        estimate.header.stamp.sec = 1
        estimate.header.stamp.nanosec = 50000000
        estimate.pose.pose.position.x = 1.5
        estimate.pose.pose.orientation.w = 1.0
        estimate.pose.covariance[0] = 1.0
        estimate.pose.covariance[7] = 1.0
        state = SimpleNamespace(
            truth=deque([(1.0, first), (1.1, second)]),
            pending=deque([estimate]), rows=[], start=0.0,
            skip_counts={"before": 0, "after": 0, "gap": 0},
        )
        trial.LocalizationTrial.score_messages(state, state.pending, state.rows)
        self.assertEqual(len(state.rows), 1)
        self.assertAlmostEqual(state.rows[0]["true_x_m"], 1.0)
        self.assertAlmostEqual(state.rows[0]["position_error_m"], 0.5)


if __name__ == "__main__":
    unittest.main()
