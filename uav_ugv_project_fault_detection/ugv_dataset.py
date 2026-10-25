# Window dataset for the simulated UGV fault variants.
#
# Each folder in ``datasets/ugv_faulty`` is one variant written by
# ``fault_injection.py``. Every faultable channel (IMU wx..az, wheel vx/wz, GPS
# latitude/longitude/altitude) is cut into fixed-length sample windows. A window
# is faulty (1) when any of its samples carries a fault label for that channel.
# Variants are split into train / validation / test by ``source_run``, so no
# clean drive contributes to two sets.
#
# With ``paired=True`` each window has a second feature: a reference for the
# same quantity derived from another sensor or the drive command (never truth),
# e.g. IMU yaw rate next to wheel yaw rate. A constant offset then breaks the
# agreement between the two features even when the channel alone looks normal.
# Channels without a reference get zeros.
#
# Windows are standardized with per-channel statistics from the training
# fault-free windows, not per window: a per-window z-score would remove a
# constant offset entirely. The reference uses its channel's statistics.

import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

import dataset as D
from fault_injection import EARTH_RADIUS, NOMINAL_SIGMA

CHANNELS = [(sensor, column) for sensor, columns in NOMINAL_SIGMA.items() for column in columns]

# ugv_dataset world datum and spawn: map ENU = wheel odom position + (-4, 0).
ORIGIN_LATITUDE = 47.397971057728974
ORIGIN_LONGITUDE = 8.546163739800146
SPAWN_EAST = -4.0


def split_runs(variants, val_runs, test_runs, seed):
    """Split variant folders into train / validation / test lists by source run."""
    source = {v: json.loads((v / "faults.json").read_text())["source_run"] for v in variants}
    runs = sorted(set(source.values()))
    if val_runs + test_runs >= len(runs):
        raise ValueError(f"{len(runs)} source runs cannot hold {val_runs} val + {test_runs} test")
    order = np.random.default_rng(seed).permutation(runs).tolist()
    val, test = set(order[:val_runs]), set(order[val_runs:val_runs + test_runs])
    splits = {"train": [], "val": [], "test": []}
    for variant in variants:
        name = "val" if source[variant] in val else "test" if source[variant] in test else "train"
        splits[name].append(variant)
    return splits


def references(frames, cmd):
    """Return reference series (physical units, on each sensor's stamps) per channel."""
    imu, odom, gps = frames["imu"], frames["wheel_odom"], frames["gps"]
    at = lambda stamps, source, values: np.interp(stamps, source["stamp"], values)
    command_accel = np.gradient(cmd["linear"].to_numpy(), cmd["stamp"].to_numpy())
    north, east = odom["y"].to_numpy(), odom["x"].to_numpy() + SPAWN_EAST
    return {
        ("imu", "wz"): at(imu["stamp"], odom, odom["wz"]),
        ("imu", "ax"): at(imu["stamp"], cmd, command_accel),
        ("imu", "ay"): at(imu["stamp"], cmd, cmd["linear"] * cmd["angular"]),
        ("wheel_odom", "vx"): at(odom["stamp"], cmd, cmd["linear"]),
        ("wheel_odom", "wz"): at(odom["stamp"], imu, imu["wz"]),
        ("gps", "latitude"): at(gps["stamp"], odom, ORIGIN_LATITUDE
                                + np.degrees(north / EARTH_RADIUS)),
        ("gps", "longitude"): at(gps["stamp"], odom, ORIGIN_LONGITUDE + np.degrees(
            east / (EARTH_RADIUS * math.cos(math.radians(ORIGIN_LATITUDE))))),
    }


def channel_windows(variants, seq_len, stride, paired):
    """Return raw ``(N, seq_len, F)`` windows (NaN = no reference), labels, and info."""
    windows, labels, info = [], [], []
    for variant in variants:
        frames = {sensor: pd.read_csv(variant / f"{sensor}.csv") for sensor in NOMINAL_SIGMA}
        refs = references(frames, pd.read_csv(variant / "cmd.csv")) if paired else {}
        kind = variant.name.split("_")[2]
        for sensor, column in CHANNELS:
            frame = frames[sensor]
            features = [frame[column].to_numpy(np.float64)]
            if paired:
                features.append(refs.get((sensor, column), np.full(len(frame), np.nan)))
            values = np.stack(features, axis=1)
            faulty = ((frame["fault"] == 1) & (frame["fault_column"] == column)).to_numpy()
            for start in range(0, len(frame) - seq_len + 1, stride):
                rows = slice(start, start + seq_len)
                label = int(faulty[rows].any())
                windows.append(values[rows])
                labels.append(label)
                info.append((variant.name, sensor, column, kind if label else "none"))
    info = pd.DataFrame(info, columns=["variant", "sensor", "column", "fault_type"])
    return np.stack(windows), np.asarray(labels, dtype=np.int64), info


def channel_stats(windows, labels, info):
    """Per-channel mean / std of the measured feature over fault-free training windows."""
    stats = {}
    for (sensor, column), group in info[labels == 0].groupby(["sensor", "column"]):
        values = windows[group.index.to_numpy(), :, 0]
        stats[(sensor, column)] = (float(values.mean()), float(values.std()) or 1.0)
    return stats


def standardize(windows, info, stats, seq_len, domain):
    keys = list(zip(info["sensor"], info["column"]))
    mean = np.array([stats[key][0] for key in keys])[:, None, None]
    std = np.array([stats[key][1] for key in keys])[:, None, None]
    scaled = np.nan_to_num((windows - mean) / std)   # missing reference -> 0
    if domain == "frequency":
        # Orthonormal FFT magnitude (divide by sqrt(seq_len)): unit white noise stays
        # near 1 per bin instead of growing with the window length.
        scaled = np.stack([
            np.concatenate([D._to_window(w[:, f], seq_len, "frequency")
                            for f in range(w.shape[1])], axis=1)
            for w in scaled]) / math.sqrt(seq_len)
    return scaled.astype(np.float32)


def build_ugv_splits(base_dir, seq_len=100, stride=None, val_runs=5, test_runs=5, seed=42,
                     domain="time", paired=False):
    """Return ``{"train"|"val"|"test": (windows, labels, info)}`` standardized windows."""
    variants = sorted(p for p in Path(base_dir).iterdir() if (p / "faults.json").exists())
    if not variants:
        raise RuntimeError(f'No fault_injection.py variants under "{base_dir}"')
    raw = {name: channel_windows(group, seq_len, stride or seq_len, paired)
           for name, group in split_runs(variants, val_runs, test_runs, seed).items()}
    stats = channel_stats(*raw["train"])
    return {name: (standardize(w, info, stats, seq_len, domain), y, info)
            for name, (w, y, info) in raw.items()}
