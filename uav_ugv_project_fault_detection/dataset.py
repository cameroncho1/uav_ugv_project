# Single-sensor-channel fault-detection data pipeline.
#
# Each folder in ``datasets/processed`` is one flight scenario from the ALFA
# dataset. Every scenario contains many per-topic CSV exports (IMU, navigation,
# control outputs, ...). This pipeline isolates ONE sensor channel (a single
# numeric column from a single topic) and turns the time series recorded during
# that flight into fixed-length windows that the RNN / BiLSTM / Autoencoder can
# consume. A scenario is labelled *faulty* (1) when any of its
# ``*-failure_status-*.csv`` files is present and non-zero, and *normal* (0)
# otherwise. Scenarios without a ground truth (``*no_ground_truth*``) are
# skipped.
#
# The result is a pair of PyTorch ``Dataset`` objects (train / validation) so
# that every model type can be trained with the same split.

import os
import glob

import numpy as np
import pandas as pd
import torch
from torch.utils.data import Dataset, random_split

# Reuse the project's frequency-domain helper for the FFT path. It lives under
# models/utils, so make it importable regardless of the current working dir.
import sys

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "models", "utils")
)


# Convert time series data to frequency domain using FFT
def time_to_frequency_domain(data):
    # Perform FFT on the data
    freq_data = np.fft.fft(data)
    
    # Get the frequency bins
    freq_bins = np.fft.fftfreq(len(data))
    
    return freq_data, freq_bins
    

# A scenario folder that has no ground-truth labels cannot be used for
# supervised training and is ignored.
_SKIP_SUBSTRINGS = ("no_ground_truth",)

# Supported analysis domains. Both produce an identical window shape
# ``(num_scenarios, seq_len, 1)`` so every model accepts either one:
#   - "time":      the raw (resampled) sensor channel samples.
#   - "frequency": the FFT magnitude spectrum of the channel window.
DOMAINS = ("time", "frequency")

# Default single channel: measured yaw from MAVROS. It is present in every
# scenario and is a natural indicator of control-surface / engine faults.
DEFAULT_TOPIC = "mavros-nav_info-yaw"
DEFAULT_COLUMN = "field.measured"

# Columns that carry no signal for a single-channel fault detector.
_NON_SIGNAL_PREFIXES = (
    "%time",
    "field.header.seq",
    "field.header.stamp",
    "field.header.frame_id",
    "field.coordinate_frame",
)


def _find_topic_csv(folder, topic):
    """Return the path to the CSV for ``topic`` inside ``folder`` (or None)."""
    matches = glob.glob(os.path.join(folder, f"*-{topic}.csv"))
    return matches[0] if matches else None


def read_channel(folder, topic=DEFAULT_TOPIC, column=DEFAULT_COLUMN):
    """Read a single sensor channel from a scenario folder.

    Returns a 1-D ``np.ndarray`` of the sampled channel, forward/backward filled
    so that gaps between message topics do not create NaNs. Returns ``None`` if
    the topic or column is unavailable for this scenario.
    """
    path = _find_topic_csv(folder, topic)
    if path is None:
        return None
    df = pd.read_csv(path)
    if column not in df.columns:
        return None
    series = df[column].apply(pd.to_numeric, errors="coerce")
    # Fill gaps (topics are not all sampled at the same rate) then any edges.
    series = series.ffill().bfill().fillna(0.0)
    return series.to_numpy(dtype=np.float64)


def get_scenario_label(folder):
    """Return 1 if the scenario contains a fault, 0 if it is a clean flight.

    Returns ``None`` when the scenario has no ground-truth labels and therefore
    should be skipped.
    """
    name = os.path.basename(folder)
    if any(skip in name for skip in _SKIP_SUBSTRINGS):
        return None
    label_files = glob.glob(os.path.join(folder, "*-failure_status-*.csv"))
    if not label_files:
        # No failure-status export: treat as a normal flight.
        return 0
    for label_file in label_files:
        values = pd.read_csv(label_file).iloc[:, 1].dropna()
        if len(values) and values.max() >= 1:
            return 1
    return 0


def _resample(series, seq_len):
    """Evenly resample a series to exactly ``seq_len`` points."""
    series = np.asarray(series, dtype=np.float64)
    if series.size == 0:
        return np.zeros(seq_len, dtype=np.float64)
    if series.size == seq_len:
        return series
    indices = np.linspace(0, series.size - 1, seq_len).astype(int)
    return series[indices]


def _standardize(window):
    """Per-window z-score so the channel is scale invariant across scenarios."""
    mean = window.mean()
    std = window.std()
    if std < 1e-8:
        return window - mean
    return (window - mean) / std


def _to_window(window, seq_len, domain):
    """Shape a single-channel window into ``(seq_len, 1)`` for the given domain.

    The time domain keeps the raw samples; the frequency domain uses the
    magnitude of the FFT (via ``FrequencyDomain.time_to_frequency_domain``) as
    the spectral feature. Both are padded/truncated to ``seq_len`` bins so the
    two domains produce identically shaped inputs for the models.
    """
    window = np.asarray(window, dtype=np.float64)
    if domain == "frequency":
        if time_to_frequency_domain is None:
            raise RuntimeError(
                "Frequency-domain analysis is unavailable: "
                "models/utils/FrequencyDomain.py could not be imported."
            )
        # Magnitude spectrum of the window's FFT.
        freq_data, _ = time_to_frequency_domain(window)
        feats = np.abs(freq_data)
    else:
        feats = window
    # Pad or truncate to exactly seq_len so every window matches the config.
    if feats.size < seq_len:
        feats = np.pad(feats, (0, seq_len - feats.size), mode="constant")
    elif feats.size > seq_len:
        feats = feats[:seq_len]
    return feats.reshape(seq_len, 1)


def build_dataset(
    base_dir,
    topic=DEFAULT_TOPIC,
    column=DEFAULT_COLUMN,
    seq_len=100,
    features_per_step=1,
    min_fault_windows=1,
    max_windows_per_scenario=None,
    domain="time",
):
    """Build per-scenario feature windows from a single sensor channel.

    Parameters
    ----------
    base_dir : str
        Directory containing the per-scenario folders (``datasets/processed``).
    topic, column : str
        The single sensor channel to use.
    seq_len : int
        Number of timesteps in every window (all windows are the same length).
    features_per_step : int
        Number of features per timestep. ``1`` means the raw channel value; a
        larger value appends simple rolling statistics so the channel can be
        turned into a multi-dimensional feature vector.
    min_fault_windows : int
        Guard so that scenarios producing no usable windows are dropped safely.
    max_windows_per_scenario : int or None
        Cap the number of windows taken per scenario to keep the dataset balanced.
    domain : str
        ``'time'`` (raw samples) or ``'frequency'`` (FFT magnitude spectrum).
    Both return identically shaped ``(num_scenarios, seq_len, 1)`` windows.

    Returns
    -------
    (windows, labels, scenario_names) :
        ``windows`` is a list of ``(seq_len, features_per_step)`` arrays,
        ``labels`` the matching fault labels, ``scenario_names`` provenance.
    """
    if domain not in DOMAINS:
        raise ValueError(f"Unknown domain '{domain}'. Expected one of {DOMAINS}.")
    folders = sorted(
        d for d in glob.glob(os.path.join(base_dir, "*")) if os.path.isdir(d)
    )
    windows, labels, scenario_names = [], [], []

    for folder in folders:
        label = get_scenario_label(folder)
        if label is None:
            continue  # no ground truth: cannot be used for training

        channel = read_channel(folder, topic, column)
        if channel is None or channel.size < 2:
            continue  # channel unavailable for this scenario

        window = _resample(channel, seq_len)
        window = _standardize(window)

        if domain == "frequency":
            # Spectral features; identical shape to the time-domain window.
            feats = _to_window(window, seq_len, domain="frequency")
        elif features_per_step > 1:
            # Expand a single channel into a small feature vector per timestep
            # when requested, otherwise keep it as one column.
            rolling = pd.Series(window)
            mean = rolling.rolling(features_per_step, center=True, min_periods=1).mean()
            std = (
                rolling.rolling(features_per_step, center=True, min_periods=1)
                .std()
                .fillna(0.0)
            )
            feats = np.stack([window, mean, std], axis=1)[:, :features_per_step]
        else:
            feats = window.reshape(seq_len, 1)

        windows.append(feats.astype(np.float32))
        labels.append(label)
        scenario_names.append(os.path.basename(folder))

    windows = np.stack(windows, axis=0)  # (num_scenarios, seq_len, features)
    labels = np.array(labels, dtype=np.int64)

    if max_windows_per_scenario is not None:
        windows = windows[:max_windows_per_scenario]
        labels = labels[:max_windows_per_scenario]
        scenario_names = scenario_names[:max_windows_per_scenario]

    if windows.size == 0:
        raise RuntimeError(
            f'No usable windows produced from "{base_dir}" with '
            f'topic="{topic}", column="{column}". '
            "Check that the dataset folder contains the expected CSVs."
        )

    return windows, labels, scenario_names


class FaultDataset(Dataset):
    """A single-sensor-channel fault dataset.

    Each item is a ``(seq_len, features_per_step)`` window paired with its
    fault label (0 = normal, 1 = fault).
    """

    def __init__(self, windows, labels):
        self.windows = torch.as_tensor(
            np.ascontiguousarray(windows), dtype=torch.float32
        )
        self.labels = torch.as_tensor(labels, dtype=torch.long)

    def __len__(self):
        return self.windows.shape[0]

    def __getitem__(self, idx):
        return self.windows[idx], self.labels[idx]


def make_train_val(windows, labels, val_fraction=0.2, seed=42):
    """Split scenarios into training and validation windows/labels.

    Returns ``((train_windows, train_labels), (val_windows, val_labels))`` where
    each element is a NumPy array. The validation tuple is ``(None, None)`` when
    there are too few scenarios to justify a split.
    """
    total = len(windows)
    if total < 2:
        return (windows, labels), (None, None)
    val_size = max(1, int(round(total * val_fraction)))
    if val_size >= total:
        val_size = total - 1
    train_size = total - val_size
    generator = torch.Generator().manual_seed(seed)
    dataset = FaultDataset(windows, labels)
    train_ds, val_ds = random_split(
        dataset, [train_size, val_size], generator=generator
    )
    train_idx = torch.as_tensor(sorted(train_ds.indices))
    val_idx = torch.as_tensor(sorted(val_ds.indices))
    train_windows = windows[train_idx]
    train_labels = labels[train_idx]
    val_windows = windows[val_idx]
    val_labels = labels[val_idx]
    return (train_windows, train_labels), (val_windows, val_labels)
