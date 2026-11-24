"""Numpy inference for the UGV sensor-fault BiLSTM and autoencoder.

The models are trained offline in ``uav_ugv_project_fault_detection`` (see its
MODELS.md) and exported with ``export_live_detector.py`` to one ``.npz``. This
module rebuilds the paired training features from live measurement buffers and
runs both models without torch, ROS, or simulator imports.

Each channel window is the last ``window_len`` samples of one sensor channel,
paired with a reference for the same quantity from another sensor or the drive
command (``ugv_dataset.references``). Windows are standardized with per-channel
statistics of the clean training windows; a missing reference becomes 0.
"""

import json
import math

import numpy as np

EARTH_RADIUS = 6378137.0

CHANNELS = (
    ("imu", "wx"), ("imu", "wy"), ("imu", "wz"), ("imu", "ax"), ("imu", "ay"), ("imu", "az"),
    ("wheel_odom", "vx"), ("wheel_odom", "wz"),
    ("gps", "latitude"), ("gps", "longitude"), ("gps", "altitude"),
)


# FaultStatus classes (completion plan section 4.3) and the channels whose
# BiLSTM flags set them. The simulated wheel-speed scale fault changes only the
# forward speed, so wheel_odom/wz sets no class; neither do the other IMU
# channels or GPS altitude.
FAULT_CLASSES = {
    "odometry_scale": (("wheel_odom", "vx"),),
    "imu_drift": (("imu", "wz"),),
    "gps_outage": (("gps", "latitude"), ("gps", "longitude")),
}
# Channels behind the hold-requesting classes. A decision is valid only when
# each has a current evaluation; the oldest of their window ends stamps it.
DECISION_CHANNELS = FAULT_CLASSES["odometry_scale"] + FAULT_CLASSES["imu_drift"]
FAULT_FLAGS = ("gps_outage", "odometry_scale", "imu_drift", "uav_loss")


class FlagPersistence:
    """Raise a model-derived class flag only after ``required`` consecutive flags.

    Counts distinct decision windows (strictly increasing stamps), so a repeated
    evaluation of the same window does not count twice. An invalid decision
    resets every count. Isolated single-window flags then never reach the
    supervisor; a persistent fault is reported ``required - 1`` windows later.
    """

    def __init__(self, required):
        if int(required) != required or required < 1:
            raise ValueError("flag persistence must be a positive integer")
        self.required = int(required)
        self.reset()

    def reset(self):
        self.counts = dict.fromkeys(FAULT_CLASSES, 0)
        self.last_stamp = None
        self.output = dict.fromkeys(FAULT_CLASSES, False)

    def update(self, stamp, flags):
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return dict(self.output)
        self.last_stamp = stamp
        for name in FAULT_CLASSES:
            self.counts[name] = self.counts[name] + 1 if flags[name] else 0
            self.output[name] = self.counts[name] >= self.required
        return dict(self.output)


def channel_name(channel):
    return f"{channel[0]}/{channel[1]}"


def fault_decision(current, gps_available, uav_available, persistence=None):
    """Combine per-channel results into one class decision.

    ``current`` maps a channel to ``(fault, window_end)`` for channels with a
    current (not stale) evaluation. ``gps_available`` and ``uav_available`` are
    whether a GPS fix or a UAV relative measurement arrived recently; absence
    sets ``gps_outage`` or ``uav_loss`` immediately. With a ``FlagPersistence``,
    the model-derived part of each class must persist before it is raised.
    Returns ``(valid, stamp, flags)``; an invalid decision has no stamp and all
    flags false.
    """
    flags = dict.fromkeys(FAULT_FLAGS, False)
    if any(channel not in current for channel in DECISION_CHANNELS):
        if persistence is not None:
            persistence.reset()
        return False, None, flags
    stamp = min(current[channel][1] for channel in DECISION_CHANNELS)
    model = {name: any(current[channel][0] for channel in channels if channel in current)
             for name, channels in FAULT_CLASSES.items()}
    if persistence is not None:
        model = persistence.update(stamp, model)
    flags.update(model)
    flags["gps_outage"] = flags["gps_outage"] or not gps_available
    flags["uav_loss"] = not uav_available
    return True, stamp, flags


def command_acceleration(cmd):
    """Time derivative of the commanded linear speed on the command stamps."""
    if len(cmd["stamp"]) < 2:
        return np.zeros(len(cmd["stamp"]))
    return np.gradient(cmd["linear"], cmd["stamp"])


def reference(channel, stamps, streams, origin):
    """Reference series for ``channel`` on ``stamps``, or ``None`` without one.

    ``streams`` maps ``imu``, ``wheel_odom``, ``gps``, and ``cmd`` to dicts of
    equal-length numpy arrays with a ``stamp`` key. ``wheel_odom`` positions
    ``x`` (east) and ``y`` (north) are map ENU meters. ``origin`` is the
    (latitude, longitude) of the map origin in degrees. Sources are linearly
    interpolated and held at their ends, as in training.
    """
    imu, odom, cmd = streams.get("imu"), streams.get("wheel_odom"), streams.get("cmd")

    def at(source, values):
        if source is None or len(source["stamp"]) == 0:
            return None
        return np.interp(stamps, source["stamp"], values)

    if channel == ("imu", "wz"):
        return at(odom, odom["wz"]) if odom else None
    if channel == ("imu", "ax"):
        return at(cmd, command_acceleration(cmd)) if cmd else None
    if channel == ("imu", "ay"):
        return at(cmd, cmd["linear"] * cmd["angular"]) if cmd else None
    if channel == ("wheel_odom", "vx"):
        return at(cmd, cmd["linear"]) if cmd else None
    if channel == ("wheel_odom", "wz"):
        return at(imu, imu["wz"]) if imu else None
    if channel == ("gps", "latitude") and odom:
        return at(odom, origin[0] + np.degrees(odom["y"] / EARTH_RADIUS))
    if channel == ("gps", "longitude") and odom:
        scale = EARTH_RADIUS * math.cos(math.radians(origin[0]))
        return at(odom, origin[1] + np.degrees(odom["x"] / scale))
    return None


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def _lstm_direction(x, w_ih, w_hh, b_ih, b_hh, reverse):
    """One LSTM direction with PyTorch gate order (i, f, g, o); x is (B, T, In)."""
    batch, steps, _ = x.shape
    hidden = w_hh.shape[1]
    inputs = x @ w_ih.T + (b_ih + b_hh)
    h = np.zeros((batch, hidden))
    c = np.zeros((batch, hidden))
    out = np.empty((batch, steps, hidden))
    for t in (range(steps - 1, -1, -1) if reverse else range(steps)):
        gates = inputs[:, t] + h @ w_hh.T
        i, f, g, o = np.split(gates, 4, axis=1)
        c = _sigmoid(f) * c + _sigmoid(i) * np.tanh(g)
        h = _sigmoid(o) * np.tanh(c)
        out[:, t] = h
    return out


class SensorFaultModel:
    """Exported BiLSTM classifier and autoencoder with their channel statistics."""

    def __init__(self, arrays):
        self.p = {key: np.asarray(arrays[key], dtype=np.float64)
                  for key in arrays if key.startswith(("bilstm.", "ae."))}
        names = [str(name) for name in arrays["channels"]]
        self.channels = [tuple(name.split("/")) for name in names]
        self.index = {channel: i for i, channel in enumerate(self.channels)}
        self.mean = np.asarray(arrays["mean"], dtype=np.float64)
        self.std = np.asarray(arrays["std"], dtype=np.float64)
        self.ae_threshold = np.asarray(arrays["ae_threshold"], dtype=np.float64)
        self.window_len = int(arrays["window_len"])
        self.num_layers = int(arrays["num_layers"])
        self.metadata = json.loads(str(arrays["metadata"]))
        self.has_autoencoder = "ae.encoder.weight" in self.p

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as arrays:
            return cls({key: arrays[key] for key in arrays.files})

    def standardize(self, channel, values):
        """(T, 2) raw [channel, reference] -> standardized; NaN reference -> 0."""
        i = self.index[channel]
        return np.nan_to_num((np.asarray(values, dtype=np.float64) - self.mean[i]) / self.std[i])

    def classify(self, windows):
        """Fault probability (softmax) for standardized (B, T, 2) windows."""
        x = np.asarray(windows, dtype=np.float64)
        for layer in range(self.num_layers):
            parts = []
            for suffix, reverse in (("", False), ("_reverse", True)):
                key = f"bilstm.lstm.{{}}_l{layer}{suffix}"
                parts.append(_lstm_direction(
                    x, self.p[key.format("weight_ih")], self.p[key.format("weight_hh")],
                    self.p[key.format("bias_ih")], self.p[key.format("bias_hh")], reverse))
            x = np.concatenate(parts, axis=2)
        hidden = x.shape[2] // 2
        features = np.concatenate((x[:, -1, :hidden], x[:, 0, hidden:]), axis=1)
        logits = features @ self.p["bilstm.fc.weight"].T + self.p["bilstm.fc.bias"]
        logits -= logits.max(axis=1, keepdims=True)
        probability = np.exp(logits)
        return probability[:, 1] / probability.sum(axis=1)

    def reconstruction_error(self, windows):
        """Mean squared autoencoder reconstruction error per standardized window."""
        x = np.asarray(windows, dtype=np.float64).reshape(len(windows), -1)
        encoded = np.maximum(x @ self.p["ae.encoder.weight"].T + self.p["ae.encoder.bias"], 0.0)
        decoded = encoded @ self.p["ae.decoder.weight"].T + self.p["ae.decoder.bias"]
        return ((decoded - x) ** 2).mean(axis=1)

    def evaluate(self, raw_windows):
        """Score ``{channel: (T, 2) raw window}``; returns ``{channel: result dict}``."""
        channels = list(raw_windows)
        if not channels:
            return {}
        x = np.stack([self.standardize(c, raw_windows[c]) for c in channels])
        probability = self.classify(x)
        error = self.reconstruction_error(x) if self.has_autoencoder else None
        results = {}
        for k, channel in enumerate(channels):
            result = {"probability": float(probability[k]), "bilstm": bool(probability[k] > 0.5)}
            if error is not None:
                threshold = self.ae_threshold[self.index[channel]]
                result.update(ae_error=float(error[k]), ae_threshold=float(threshold),
                              autoencoder=bool(error[k] > threshold))
            results[channel] = result
        return results
