# Window dataset for the simulated UGV fault variants.
#
# Each folder in ``datasets/ugv_faulty`` is one variant written by
# ``fault_injection.py``. Every faultable channel (IMU wx..az, wheel vx/wz, GPS
# latitude/longitude/altitude) is cut into fixed-length sample windows, so each
# window is a single-channel ``(seq_len, 1)`` sequence like the ALFA pipeline.
# A window is faulty (1) when any of its samples carries a fault label for that
# channel. Variants are split into train / validation by ``source_run``, so no
# clean drive contributes to both sets.
#
# Windows are standardized with per-channel statistics from the training
# fault-free variants, not per window: a per-window z-score would remove a
# constant offset entirely.

import json
from pathlib import Path

import numpy as np
import pandas as pd

import dataset as D
from fault_injection import NOMINAL_SIGMA

CHANNELS = [(sensor, column) for sensor, columns in NOMINAL_SIGMA.items() for column in columns]


def split_runs(variants, val_fraction, seed):
    """Split variant folders into train / validation lists by source run."""
    sources = sorted({json.loads((v / "faults.json").read_text())["source_run"]
                      for v in variants})
    rng = np.random.default_rng(seed)
    val_count = max(1, int(round(len(sources) * val_fraction)))
    val_sources = set(rng.choice(sources, size=val_count, replace=False).tolist())
    train, val = [], []
    for variant in variants:
        source = json.loads((variant / "faults.json").read_text())["source_run"]
        (val if source in val_sources else train).append(variant)
    return train, val


def channel_windows(variants, seq_len, stride):
    """Return raw windows, labels, and per-window info for every channel."""
    windows, labels, info = [], [], []
    for variant in variants:
        frames = {sensor: pd.read_csv(variant / f"{sensor}.csv") for sensor in NOMINAL_SIGMA}
        kind = variant.name.split("_")[2]
        for sensor, column in CHANNELS:
            frame = frames[sensor]
            values = frame[column].to_numpy(np.float64)
            faulty = ((frame["fault"] == 1) & (frame["fault_column"] == column)).to_numpy()
            for start in range(0, len(values) - seq_len + 1, stride):
                rows = slice(start, start + seq_len)
                label = int(faulty[rows].any())
                windows.append(values[rows])
                labels.append(label)
                info.append((variant.name, sensor, column, kind if label else "none"))
    info = pd.DataFrame(info, columns=["variant", "sensor", "column", "fault_type"])
    return np.stack(windows), np.asarray(labels, dtype=np.int64), info


def channel_stats(windows, labels, info):
    """Per-channel mean / std over fault-free training windows."""
    stats = {}
    for (sensor, column), group in info[labels == 0].groupby(["sensor", "column"]):
        values = windows[group.index.to_numpy()]
        stats[(sensor, column)] = (float(values.mean()), float(values.std()) or 1.0)
    return stats


def standardize(windows, info, stats, seq_len, domain):
    keys = list(zip(info["sensor"], info["column"]))
    mean = np.array([stats[key][0] for key in keys])[:, None]
    std = np.array([stats[key][1] for key in keys])[:, None]
    scaled = (windows - mean) / std
    if domain == "frequency":
        scaled = np.stack([D._to_window(w, seq_len, "frequency")[:, 0] for w in scaled])
    return scaled[:, :, None].astype(np.float32)


def build_ugv_train_val(base_dir, seq_len=100, stride=None, val_fraction=0.2, seed=42,
                        domain="time"):
    """Return ``((train_w, train_y, train_info), (val_w, val_y, val_info))``."""
    variants = sorted(p for p in Path(base_dir).iterdir() if (p / "faults.json").exists())
    if not variants:
        raise RuntimeError(f'No fault_injection.py variants under "{base_dir}"')
    train, val = split_runs(variants, val_fraction, seed)
    train_w, train_y, train_info = channel_windows(train, seq_len, stride or seq_len)
    val_w, val_y, val_info = channel_windows(val, seq_len, stride or seq_len)
    stats = channel_stats(train_w, train_y, train_info)
    return ((standardize(train_w, train_info, stats, seq_len, domain), train_y, train_info),
            (standardize(val_w, val_info, stats, seq_len, domain), val_y, val_info))
