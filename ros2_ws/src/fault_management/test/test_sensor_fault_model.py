"""Exported UGV sensor-fault model: numpy forward passes and live features."""

import math
from pathlib import Path

import numpy as np
import pytest
from sensor_fault_model import (CHANNELS, DECISION_CHANNELS, EARTH_RADIUS, FlagPersistence,
                                SensorFaultModel,
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
    valid, stamp, flags = fault_decision(clear, gps_available=True, uav_available=True)
    assert valid and not any(flags.values())
    # Oldest window end among the hold channels (imu/wz is index 2).
    assert stamp == 12.0

    for channel, flag in ((("wheel_odom", "vx"), "odometry_scale"),
                          (("imu", "wz"), "imu_drift"),
                          (("gps", "longitude"), "gps_outage")):
        _, _, flags = fault_decision({**clear, channel: (True, 20.0)}, True, True)
        assert [name for name, value in flags.items() if value] == [flag]
    for channel in (("imu", "ay"), ("imu", "wx"), ("wheel_odom", "wz"), ("gps", "altitude")):
        assert not any(fault_decision({**clear, channel: (True, 20.0)}, True, True)[2].values())


def test_fault_decision_validity_and_gps_outage():
    clear = {channel: (False, 10.0) for channel in DECISION_CHANNELS}
    valid, _, flags = fault_decision(clear, gps_available=False, uav_available=True)
    assert valid and flags["gps_outage"] and not flags["uav_loss"]
    valid, _, flags = fault_decision(clear, gps_available=True, uav_available=False)
    assert valid and flags["uav_loss"] and not flags["gps_outage"]
    for missing in DECISION_CHANNELS:
        partial = {c: v for c, v in clear.items() if c != missing}
        valid, stamp, flags = fault_decision(partial, gps_available=True, uav_available=False)
        assert not valid and stamp is None and not any(flags.values())


def test_flag_persistence_suppresses_isolated_flags():
    persistence = FlagPersistence(3)
    clear = {channel: (False, 0.0) for channel in DECISION_CHANNELS}

    def decide(stamp, faulty=(), gps=True):
        current = {c: (c in faulty, stamp) for c in clear}
        return fault_decision(current, gps, True, persistence)[2]

    yaw = (("imu", "wz"),)
    assert not decide(1.0, yaw)["imu_drift"]          # isolated flag
    assert not decide(1.5)["imu_drift"]
    assert not decide(2.0, yaw)["imu_drift"]
    assert not decide(2.5, yaw)["imu_drift"]
    assert decide(3.0, yaw)["imu_drift"]               # third consecutive window
    assert decide(3.0, yaw)["imu_drift"]               # same window: unchanged, not recounted
    assert not decide(3.5)["imu_drift"]                # clears on the first clear window
    # Loss of a sensor is not delayed by persistence.
    assert decide(4.0, gps=False)["gps_outage"]


def test_flag_persistence_resets_on_invalid_decision():
    persistence = FlagPersistence(2)
    wheel = {c: (c == ("wheel_odom", "vx"), 0.0) for c in DECISION_CHANNELS}
    fault_decision({c: (v[0], 1.0) for c, v in wheel.items()}, True, True, persistence)
    fault_decision({}, True, True, persistence)        # invalid: counts reset
    flags = fault_decision({c: (v[0], 2.0) for c, v in wheel.items()}, True, True, persistence)[2]
    assert not flags["odometry_scale"]
    with pytest.raises(ValueError):
        FlagPersistence(0)
