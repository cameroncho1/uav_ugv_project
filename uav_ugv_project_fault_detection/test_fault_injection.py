# Checks for fault_injection.py on a small synthetic run.
# Run with ``./bin/python test_fault_injection.py`` (also pytest-compatible).

import json
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd

from fault_injection import (FaultSpec, apply_fault, channel_scale, label_frames,
                             make_variant, random_fault)


def synthetic_frames(duration=60.0):
    def frame(rate, columns):
        stamp = 100.0 + np.arange(0, duration, 1.0 / rate)
        return pd.DataFrame({"stamp": stamp, **{c: np.zeros(len(stamp)) for c in columns}})

    frames = {"imu": frame(50, ["wx", "wy", "wz", "ax", "ay", "az"]),
              "wheel_odom": frame(20, ["vx", "wz"]),
              "gps": frame(5, ["latitude", "longitude", "altitude"])}
    frames["gps"]["latitude"] = 47.4
    return frames


def test_constant_fault_offsets_from_onset_to_end():
    frames = synthetic_frames()
    spec = FaultSpec("imu", "az", "constant", 20.0, 60.0, 5.0, sign=-1)
    mask = apply_fault(frames, spec, np.random.default_rng(0))
    t = frames["imu"]["stamp"] - 100.0
    assert mask.tolist() == (t >= 20.0).tolist()
    assert np.allclose(frames["imu"]["az"][mask], -0.5)
    assert np.allclose(frames["imu"]["az"][~mask], 0.0)


def test_noise_fault_is_confined_and_scaled():
    frames = synthetic_frames(400.0)
    spec = FaultSpec("wheel_odom", "vx", "noise", 50.0, 350.0, 4.0)
    mask = apply_fault(frames, spec, np.random.default_rng(1))
    values = frames["wheel_odom"]["vx"]
    assert np.allclose(values[~mask], 0.0)
    assert abs(values[mask].std() - 4.0 * 0.05) < 0.01


def test_short_fault_labels_only_spike_samples():
    frames = synthetic_frames()
    spec = FaultSpec("imu", "wz", "short", 10.0, 20.0, 8.0,
                     spikes=[(12.0, 2, 8.0), (15.0, 1, -6.0)])
    mask = apply_fault(frames, spec, np.random.default_rng(2))
    assert mask.sum() == 3
    assert np.allclose(sorted(frames["imu"]["wz"][mask]), [-0.06, 0.08, 0.08])


def test_gps_offsets_are_meters():
    meters_per_degree = 1.0 / channel_scale("gps", "latitude", 47.4) * 1.5
    assert abs(meters_per_degree - 111_319.5) < 1.0
    assert channel_scale("gps", "longitude", 47.4) > channel_scale("gps", "latitude", 47.4)


def test_random_faults_fit_inside_run():
    rng = np.random.default_rng(3)
    for kind in ("short", "noise", "constant"):
        for _ in range(200):
            spec = random_fault(kind, 120.0, rng)
            assert 3.0 <= spec.start < spec.end <= 120.0
            assert all(spec.start <= s[0] <= spec.end for s in spec.spikes)


def test_noise_window_is_configurable_and_clipped():
    rng = np.random.default_rng(6)
    for _ in range(200):
        spec = random_fault("noise", 300.0, rng, noise_window=(30.0, 120.0))
        assert 30.0 <= spec.end - spec.start <= 120.0 and spec.end <= 300.0
        short_run = random_fault("noise", 20.0, rng, noise_window=(30.0, 120.0))
        assert 3.0 <= short_run.start < short_run.end <= 20.0


def test_labels_and_variant_files():
    frames = synthetic_frames()
    spec = FaultSpec("gps", "altitude", "constant", 30.0, 60.0, 3.0)
    label_frames(frames, [(spec, apply_fault(frames, spec, np.random.default_rng(4)))])
    assert set(frames["gps"]["fault_type"]) == {"none", "constant"}
    assert frames["imu"]["fault"].sum() == 0
    with tempfile.TemporaryDirectory() as directory:
        run = Path(directory) / "run_1"
        run.mkdir()
        for name, frame in synthetic_frames().items():
            frame.to_csv(run / f"{name}.csv", index=False)
        for name in ("truth.csv", "cmd.csv"):
            (run / name).write_text("stamp\n0\n")
        (run / "metadata.json").write_text("{}")
        make_variant(run, Path(directory) / "out", "noise", seed=5)
        faults = json.loads((Path(directory) / "out/faults.json").read_text())
        assert faults["source_run"] == "run_1" and faults["faults"][0]["kind"] == "noise"
        sensor = faults["faults"][0]["sensor"]
        labels = pd.read_csv(Path(directory) / f"out/{sensor}.csv")["fault"]
        assert labels.sum() == faults["faults"][0]["labelled_samples"] > 0


if __name__ == "__main__":
    for name, test in list(globals().items()):
        if name.startswith("test_"):
            test()
            print("ok", name)
