"""Numerical evaluation and arrival checks independent of ROS or simulator APIs."""

import math
import numpy as np


class ArrivalDwell:
    def __init__(self, seconds=1.0):
        if not math.isfinite(seconds) or seconds <= 0:
            raise ValueError("Arrival dwell must be positive")
        self.seconds = seconds
        self.since = None

    def update(self, elapsed, eligible):
        if not eligible:
            self.since = None
            return False
        if self.since is None or elapsed < self.since:
            self.since = elapsed
        return elapsed - self.since >= self.seconds


def matched_heading(truth_times, truth_heading, estimate_times, estimate_heading, max_gap=.1):
    truth_times = np.asarray(truth_times, float)
    estimate_times = np.asarray(estimate_times, float)
    truth_heading = np.asarray(truth_heading, float)
    estimate_heading = np.asarray(estimate_heading, float)
    if (truth_times.ndim != 1 or estimate_times.ndim != 1 or len(estimate_times) < 2
            or truth_heading.shape != truth_times.shape
            or estimate_heading.shape != estimate_times.shape
            or not all(np.isfinite(a).all() for a in
                       (truth_times, estimate_times, truth_heading, estimate_heading))
            or np.any(np.diff(estimate_times) <= 0) or np.any(np.diff(truth_times) <= 0)):
        raise ValueError("Heading logs require finite, strictly increasing acquisition times")
    indexes = np.searchsorted(estimate_times, truth_times)
    safe = np.clip(indexes, 0, len(estimate_times) - 1)
    exact = (indexes < len(estimate_times)) & (estimate_times[safe] == truth_times)
    before = np.maximum(0, safe - 1)
    covered = (truth_times >= estimate_times[0]) & (truth_times <= estimate_times[-1])
    gap_ok = exact | ((estimate_times[safe] - estimate_times[before]) <= max_gap + 1e-9)
    selected = covered & gap_ok
    matched = np.interp(truth_times[selected], estimate_times, estimate_heading)
    errors = (matched - truth_heading[selected] + np.pi) % (2 * np.pi) - np.pi
    return selected, matched, errors, {"outside_estimate_coverage": int((~covered).sum()),
                                      "estimate_gap": int((covered & ~gap_ok).sum())}


def cross_track(point, path):
    points = np.asarray(path, float)
    p = np.asarray(point, float)
    if len(points) < 2 or points.shape[1:] != (2,) or not np.isfinite(points).all():
        raise ValueError("Tracking needs a finite planar polyline")
    a, delta = points[:-1], np.diff(points, axis=0)
    lengths2 = (delta * delta).sum(axis=1)
    alpha = np.clip(((p - a) * delta).sum(axis=1) / np.maximum(lengths2, 1e-15), 0., 1.)
    projected = a + alpha[:, None] * delta
    return float(np.sqrt(((projected - p)**2).sum(axis=1)).min())


def tracking_metrics(rows):
    if not rows:
        return {"samples": 0}
    values = np.array([r["cross_track_m"] for r in rows])
    return {"samples": len(rows), "cross_track_rmse_m": float(np.sqrt(np.mean(values**2))),
            "mean_cross_track_m": float(values.mean()), "max_cross_track_m": float(values.max()),
            "p95_cross_track_m": float(np.percentile(values, 95))}
