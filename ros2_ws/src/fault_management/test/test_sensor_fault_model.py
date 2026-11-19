"""Exported UGV sensor-fault model: numpy forward passes and live features."""

import math
from pathlib import Path

import numpy as np
import pytest
from sensor_fault_model import (CHANNELS, DECISION_CHANNELS, EARTH_RADIUS, SensorFaultModel,
                                fault_decision, reference)

MODEL = Path(__file__).resolve().parents[1] / "models/ugv_sensor_fault_detector.npz"


@pytest.fixture(scope="module")
def model():
    return SensorFaultModel.load(MODEL)


def test_numpy_forward_matches_exported_torch_outputs(model):
    with np.load(MODEL) as arrays:
        windows = arrays["check.windows"]
        probability = arrays["check.probability"]
        error = arrays["check.ae_error"]
    np.testing.assert_allclose(model.classify(windows), probability, atol=1e-5)
    np.testing.assert_allclose(model.reconstruction_error(windows), error, rtol=1e-4)
    assert model.channels == list(CHANNELS)
    assert np.all(model.std > 0)


def test_missing_reference_standardizes_to_zero(model):
    values = np.column_stack([np.full(5, model.mean[0]), np.full(5, np.nan)])
    np.testing.assert_array_equal(model.standardize(("imu", "wx"), values), np.zeros((5, 2)))


def test_references_follow_training_definitions():
    stamps = np.array([0.0, 0.5, 1.0])
    streams = {
        "imu": {"stamp": np.array([0.0, 1.0]), "wz": np.array([0.0, 0.2])},
        "wheel_odom": {"stamp": np.array([0.0, 1.0]), "wz": np.array([0.1, 0.3]),
                       "x": np.array([-4.0, -2.0]), "y": np.array([0.0, 10.0])},
        "cmd": {"stamp": np.array([0.0, 1.0]), "linear": np.array([0.0, 0.5]),
                "angular": np.array([1.0, 1.0])},
    }
    origin = (47.4, 8.5)
    np.testing.assert_allclose(reference(("imu", "wz"), stamps, streams, origin), [0.1, 0.2, 0.3])
    np.testing.assert_allclose(reference(("imu", "ax"), stamps, streams, origin), [0.5] * 3)
    np.testing.assert_allclose(reference(("imu", "ay"), stamps, streams, origin), [0, .25, .5])
    np.testing.assert_allclose(reference(("wheel_odom", "vx"), stamps, streams, origin),
                               [0, .25, .5])
    np.testing.assert_allclose(reference(("wheel_odom", "wz"), stamps, streams, origin),
                               [0, .1, .2])
    latitude = reference(("gps", "latitude"), stamps, streams, origin)
    assert latitude[-1] == pytest.approx(47.4 + math.degrees(10.0 / EARTH_RADIUS))
    longitude = reference(("gps", "longitude"), stamps, streams, origin)
    assert longitude[0] == pytest.approx(
        8.5 + math.degrees(-4.0 / (EARTH_RADIUS * math.cos(math.radians(47.4)))))
    for channel in (("imu", "wx"), ("imu", "az"), ("gps", "altitude")):
        assert reference(channel, stamps, streams, origin) is None


def test_constant_offsets_are_flagged(model):
    # Stationary yaw rate and a 0.4 m/s straight drive with nominal noise, then
    # constant offsets of 10 sigma (yaw rate) and 6 sigma (wheel speed).
    rng = np.random.default_rng(1)
    n = model.window_len
    yaw = np.column_stack([rng.normal(0, 0.01, n), rng.normal(0, 0.02, n)])
    speed = np.column_stack([0.4 + rng.normal(0, 0.05, n), np.full(n, 0.4)])
    clean = model.evaluate({("imu", "wz"): yaw, ("wheel_odom", "vx"): speed})
    assert not any(result["bilstm"] for result in clean.values())
    faulty = model.evaluate({("imu", "wz"): yaw + [0.1, 0.0],
                             ("wheel_odom", "vx"): speed + [0.3, 0.0]})
    assert all(result["bilstm"] for result in faulty.values())


def test_fault_decision_maps_channels_to_classes():
    clear = {channel: (False, 10.0 + i) for i, channel in enumerate(CHANNELS)}
    valid, stamp, flags = fault_decision(clear, gps_available=True)
    assert valid and not any(flags.values())
    # Oldest window end among the hold channels (imu/wz is index 2).
    assert stamp == 12.0

    for channel, flag in ((("wheel_odom", "vx"), "odometry_scale"),
                          (("wheel_odom", "wz"), "odometry_scale"),
                          (("imu", "wz"), "imu_drift"),
                          (("gps", "longitude"), "gps_outage")):
        _, _, flags = fault_decision({**clear, channel: (True, 20.0)}, True)
        assert [name for name, value in flags.items() if value] == [flag]
    for channel in (("imu", "ay"), ("imu", "wx"), ("gps", "altitude")):
        assert not any(fault_decision({**clear, channel: (True, 20.0)}, True)[2].values())


def test_fault_decision_validity_and_gps_outage():
    clear = {channel: (False, 10.0) for channel in DECISION_CHANNELS}
    valid, _, flags = fault_decision(clear, gps_available=False)
    assert valid and flags["gps_outage"] and not flags["uav_loss"]
    for missing in DECISION_CHANNELS:
        partial = {c: v for c, v in clear.items() if c != missing}
        valid, stamp, flags = fault_decision(partial, gps_available=True)
        assert not valid and stamp is None and not any(flags.values())
