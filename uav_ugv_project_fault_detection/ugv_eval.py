# Layout-independent scoring for the UGV fault detectors.
#
# Both layouts reduce to one decision per (window, channel) pair:
#   - channel layout (ugv_dataset.py): one window of one channel = one pair;
#   - multivariate layout (ugv_multivariate.py): one window = 11 pairs.
# Window lengths differ between the layouts (channel windows last 2 s for IMU,
# ~5.2 s for wheel odometry, and 20 s for GPS; multivariate windows last 2 s),
# so window-level F1 is not comparable across them. Two measures are:
#   - event isolation recall: a fault event (one injected fault, on one channel
#     of one variant) counts as found when any window of that channel that holds
#     faulty samples is flagged on that channel;
#   - clean time flagged: the duration of falsely flagged clean pairs over the
#     duration of all clean pairs, averaged over channels.

import numpy as np
import pandas as pd

import metrics

KINDS = ("short", "noise", "constant")


def _durations(info, seq_len):
    # Window duration from its first and last sample stamps.
    return (info["t_last"] - info["t_first"]).to_numpy() * seq_len / (seq_len - 1)


def channel_pairs(info, labels, preds, seq_len):
    """Pair table of the channel layout: one row per window."""
    return pd.DataFrame({
        "variant": info["variant"].to_numpy(),
        "channel": (info["sensor"] + "/" + info["column"]).to_numpy(),
        "label": np.asarray(labels, dtype=int), "pred": np.asarray(preds, dtype=int),
        "fault_type": info["fault_type"].to_numpy(), "duration": _durations(info, seq_len)})


def multivariate_pairs(info, labels, preds, targets, seq_len):
    """Pair table of the multivariate layout: one row per (window, channel)."""
    labels, preds = np.asarray(labels, dtype=int), np.asarray(preds, dtype=int)
    n, k = labels.shape
    kind = np.repeat(info["kind"].to_numpy(), k)
    return pd.DataFrame({
        "variant": np.repeat(info["variant"].to_numpy(), k),
        "channel": np.tile(np.asarray(targets), n),
        "label": labels.ravel(), "pred": preds.ravel(),
        "fault_type": np.where(labels.ravel() == 1, kind, "none"),
        "duration": np.repeat(_durations(info, seq_len), k)})


def evaluate(pairs):
    """Return window-level, event-level, time-flagged, and per-channel scores."""
    _, overall = metrics.summary(pairs["label"], pairs["pred"])
    clean = pairs[pairs["label"] == 0]
    faulty = pairs[pairs["label"] == 1]
    events = faulty.groupby(["variant", "channel"]).agg(
        found=("pred", "max"), fault_type=("fault_type", "first"))
    flagged_time = (clean["pred"] * clean["duration"]).groupby(clean["channel"]).sum()
    clean_time = clean["duration"].groupby(clean["channel"]).sum()
    result = {
        "windows": overall,
        "window_recall": {k: _mean(faulty["pred"][faulty["fault_type"] == k]) for k in KINDS},
        "clean_fpr": _mean(clean["pred"]),
        "events": int(len(events)),
        "event_recall": _mean(events["found"]),
        "event_recall_by_type": {k: _mean(events["found"][events["fault_type"] == k])
                                 for k in KINDS},
        "events_by_type": {k: int((events["fault_type"] == k).sum()) for k in KINDS},
        "clean_time_flagged": float((flagged_time / clean_time).mean()),
        "per_channel": {},
    }
    for channel, group in pairs.groupby("channel"):
        _, scores = metrics.summary(group["label"], group["pred"])
        found = events["found"].xs(channel, level="channel") if channel in \
            events.index.get_level_values("channel") else pd.Series(dtype=float)
        result["per_channel"][channel] = {
            "f1": scores["f1"], "precision": scores["precision"], "recall": scores["recall"],
            "events": int(len(found)), "event_recall": _mean(found),
            "clean_time_flagged": float(flagged_time[channel] / clean_time[channel]),
        }
    return result


def _mean(values):
    values = np.asarray(values, dtype=float)
    return float(values.mean()) if len(values) else float("nan")


def format_result(result):
    w = result["windows"]
    lines = [
        f"   windows: TP={w['tp']} FP={w['fp']} FN={w['fn']} TN={w['tn']}  "
        f"P={w['precision']:.3f} R={w['recall']:.3f} F1={w['f1']:.3f}  "
        f"clean FPR={result['clean_fpr']:.4f}",
        "   window recall: " + ", ".join(f"{k} {v:.2f}" for k, v in result["window_recall"].items()),
        f"   event isolation recall: {result['event_recall']:.3f} of {result['events']} "
        "(" + ", ".join(f"{k} {result['event_recall_by_type'][k]:.2f}/"
                        f"{result['events_by_type'][k]}" for k in KINDS) + ")",
        f"   clean time flagged: {result['clean_time_flagged']:.4f}",
        "   per channel (F1 | event recall | clean time flagged):",
    ]
    for channel, c in result["per_channel"].items():
        lines.append(f"      {channel:22s} {c['f1']:.3f} | {c['event_recall']:.2f} of "
                     f"{c['events']:3d} | {c['clean_time_flagged']:.4f}")
    return "\n".join(lines)
