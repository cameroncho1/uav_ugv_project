#!/usr/bin/env python3
"""Compare logged PX4 heading with simulator heading at acquisition timestamps."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from evaluation_math import matched_heading


def heading(data):
    w, x, y, z = (data[f"q[{index}]"] for index in range(4))
    return np.unwrap(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def sample_times(data):
    # Gazebo truth records use timestamp; timestamp_sample can be unset.
    samples = data.get("timestamp_sample", data["timestamp"])
    return np.where(samples > 0, samples, data["timestamp"])*1e-6


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", type=Path)
    parser.add_argument("trial", type=Path, help="Localization trial JSON defines the time window")
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    trial = json.loads(args.trial.read_text())
    start = trial["start_simulation_time_s"] + 10.5
    end = trial["start_simulation_time_s"] + trial["duration_s"]
    from pyulog import ULog
    data = {dataset.name: dataset.data for dataset in ULog(str(args.ulog)).data_list}
    truth = data["vehicle_attitude_groundtruth"]
    estimate = data["vehicle_attitude"]
    times = sample_times(truth)
    estimate_times = sample_times(estimate)
    selected = (times >= start) & (times < end)
    times = times[selected]
    if len(times) < 100:
        raise ValueError("Insufficient logged heading samples in the trial scoring window")
    true_heading = heading(truth)[selected]
    valid, estimated_heading, errors, skipped = matched_heading(
        times, true_heading, estimate_times, heading(estimate))
    times, true_heading = times[valid], true_heading[valid]
    if len(times) < 100:
        raise ValueError("Insufficient covered, gap-checked estimate/truth matches")
    errors_deg = np.degrees(errors)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.with_suffix(".csv").open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["sample_time_s", "estimated_ned_yaw_deg",
                         "true_ned_yaw_deg", "error_deg"])
        writer.writerows(zip(times, np.degrees(estimated_heading),
                             np.degrees(true_heading), errors_deg))
    result = {
        "samples": len(times), "skipped": skipped, "maximum_estimate_gap_s": 0.1, "window_s": [start, end],
        "source_ulog": args.ulog.name,
        "signed_heading_bias_deg": float(errors_deg.mean()),
        "rmse_heading_deg": float(np.sqrt(np.mean(errors_deg**2))),
        "max_abs_heading_error_deg": float(np.abs(errors_deg).max()),
    }
    args.output.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
