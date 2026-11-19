# Multivariate window dataset for the simulated UGV fault variants.
#
# ``ugv_dataset.py`` windows each sensor channel separately, and one model
# scores every channel without knowing which one it sees. Here every time step
# carries all sensor channels at once, and the model outputs one fault
# probability per faultable channel, so it can check each sensor against the
# others.
#
# Time base: the IMU stamps (50 Hz). Wheel odometry (~19 Hz), GPS (5 Hz), and
# the drive command (20 Hz) are sample-and-hold: each step carries the latest
# sample at or before it. This is causal and keeps spike amplitudes intact.
#
# Features per step (FEATURES): the 11 faultable channels, wheel-odometry
# position, and the drive command. GPS latitude / longitude become east / north
# meters about the world datum. Within each window, GPS east / north and odometry
# east / north are taken relative to the odometry position at the window's first
# step, so windows do not depend on where the drive is; GPS altitude stays absolute.
# Simulator truth is never used.
#
# Labels: an ``(N, 11)`` 0/1 array in ``TARGETS`` order. A window is faulty for a
# channel when any held sample of that channel inside the window carries its
# fault label, so the label matches what the model sees.
#
# Features are standardized with statistics from fault-free training windows;
# the frequency domain keeps the seq_len // 2 + 1 one-sided FFT magnitudes per
# feature, as in ugv_dataset.py.

import json
import math

import numpy as np
import pandas as pd

from fault_injection import EARTH_RADIUS
from ugv_dataset import (CHANNELS, ORIGIN_LATITUDE, ORIGIN_LONGITUDE, SPAWN_EAST,
                         variant_splits)

FEATURES = ["imu_wx", "imu_wy", "imu_wz", "imu_ax", "imu_ay", "imu_az",
            "wheel_vx", "wheel_wz", "gps_east", "gps_north", "gps_up",
            "odom_east", "odom_north", "cmd_linear", "cmd_angular"]
TARGETS = [f"{sensor}/{column}" for sensor, column in CHANNELS]
# Feature column that carries each target channel (latitude is north).
TARGET_FEATURES = [FEATURES.index(name) for name in (
    "imu_wx", "imu_wy", "imu_wz", "imu_ax", "imu_ay", "imu_az",
    "wheel_vx", "wheel_wz", "gps_north", "gps_east", "gps_up")]
RELATIVE = [FEATURES.index(n) for n in ("gps_east", "gps_north", "odom_east", "odom_north")]
ODOM = [FEATURES.index("odom_east"), FEATURES.index("odom_north")]
RATE_HZ = 50.0


def variant_series(variant):
    """Return stamps ``(T,)``, features ``(T, F)``, and labels ``(T, 11)`` of a variant."""
    frames = {name: pd.read_csv(variant / f"{name}.csv")
              for name in ("imu", "wheel_odom", "gps", "cmd")}
    imu, odom, gps, cmd = (frames[n] for n in ("imu", "wheel_odom", "gps", "cmd"))
    stamps = imu["stamp"].to_numpy()
    first = max(frames[n]["stamp"].iloc[0] for n in ("wheel_odom", "gps", "cmd"))
    rows = {"imu": np.flatnonzero(stamps >= first)}
    stamps = stamps[rows["imu"]]
    for name in ("wheel_odom", "gps", "cmd"):   # latest sample at or before each step
        rows[name] = np.searchsorted(frames[name]["stamp"].to_numpy(), stamps, "right") - 1

    north = np.radians(gps["latitude"].to_numpy() - ORIGIN_LATITUDE) * EARTH_RADIUS
    east = (np.radians(gps["longitude"].to_numpy() - ORIGIN_LONGITUDE) * EARTH_RADIUS
            * math.cos(math.radians(ORIGIN_LATITUDE)))
    held = lambda name, values: np.asarray(values)[rows[name]]
    features = np.column_stack(
        [held("imu", imu[c]) for c in ("wx", "wy", "wz", "ax", "ay", "az")]
        + [held("wheel_odom", odom["vx"]), held("wheel_odom", odom["wz"]),
           held("gps", east), held("gps", north), held("gps", gps["altitude"]),
           held("wheel_odom", odom["x"] + SPAWN_EAST), held("wheel_odom", odom["y"]),
           held("cmd", cmd["linear"]), held("cmd", cmd["angular"])])
    labels = np.column_stack([
        held(sensor, (frames[sensor]["fault"] == 1) & (frames[sensor]["fault_column"] == column))
        for sensor, column in CHANNELS])
    return stamps, features, labels.astype(bool)


def variant_windows(variant, seq_len, stride):
    """Return raw ``(N, seq_len, F)`` windows, ``(N, 11)`` labels, and info rows."""
    stamps, features, labels = variant_series(variant)
    starts = np.arange(0, len(stamps) - seq_len + 1, stride)
    index = starts[:, None] + np.arange(seq_len)
    windows = features[index]
    windows[:, :, RELATIVE] -= np.tile(windows[:, :1, ODOM], 2)   # about the first odom fix
    faults = json.loads((variant / "faults.json").read_text())
    kind = faults["faults"][0]["kind"] if faults["faults"] else "none"
    info = pd.DataFrame({"variant": variant.name, "source_run": faults["source_run"],
                         "kind": kind, "t_first": stamps[starts],
                         "t_last": stamps[starts + seq_len - 1]})
    return windows, labels[index].any(axis=1).astype(np.float32), info


def multivariate_windows(variants, seq_len, stride):
    parts = [variant_windows(v, seq_len, stride) for v in variants]
    return (np.concatenate([p[0] for p in parts]), np.concatenate([p[1] for p in parts]),
            pd.concat([p[2] for p in parts], ignore_index=True))


def feature_stats(windows, labels):
    """Per-feature mean / std over every step of fault-free training windows."""
    clean = windows[~labels.any(axis=1)]
    std = clean.std(axis=(0, 1))
    return clean.mean(axis=(0, 1)), np.where(std > 0, std, 1.0)


def standardize(windows, stats, seq_len, domain):
    scaled = (windows - stats[0]) / stats[1]
    if domain == "frequency":   # one-sided orthonormal FFT magnitude per feature
        scaled = np.abs(np.fft.rfft(scaled, axis=1)) / math.sqrt(seq_len)
    return scaled.astype(np.float32)


def build_mv_splits(base_dir, seq_len=100, train_stride=None, val_runs=5, test_runs=5,
                    seed=42, domain="time", extra_train_dirs=(), extra_test_dirs=(),
                    extra_val_dirs=()):
    """Return ``{split: (windows (N, L, F), labels (N, 11), info)}``.

    Training windows advance by ``train_stride`` steps (default seq_len // 4);
    validation and test windows do not overlap.
    """
    splits = variant_splits(base_dir, val_runs, test_runs, seed, extra_train_dirs,
                            extra_test_dirs, extra_val_dirs)
    raw = {name: multivariate_windows(
               group, seq_len, (train_stride or max(seq_len // 4, 1)) if name == "train"
               else seq_len)
           for name, group in splits.items()}
    stats = feature_stats(*raw["train"][:2])
    return {name: (standardize(w, stats, seq_len, domain), y, info)
            for name, (w, y, info) in raw.items()}
