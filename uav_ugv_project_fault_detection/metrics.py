# Binary classification / anomaly metrics for fault detection.
#
# These helpers are dependency-free (numpy only) so they work without scikit-learn.
# They report the positive class as the *fault* (or *anomaly*) class, which is the
# class of interest for this dataset (faults are the minority).

import numpy as np


def confusion_counts(y_true, y_pred):
    """Return (tp, fp, tn, fn) for the positive (fault) class."""
    y_true = np.asarray(y_true).ravel()
    y_pred = np.asarray(y_pred).ravel()
    tp = int(((y_pred == 1) & (y_true == 1)).sum())
    fp = int(((y_pred == 1) & (y_true == 0)).sum())
    tn = int(((y_pred == 0) & (y_true == 0)).sum())
    fn = int(((y_pred == 0) & (y_true == 1)).sum())
    return tp, fp, tn, fn


def precision_recall_f1(tp, fp, tn, fn):
    """Compute precision, recall, F1, accuracy from confusion counts.

    All ratios are guarded against division by zero.
    """
    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    f1 = (
        2 * precision * recall / (precision + recall)
        if (precision + recall) > 0
        else 0.0
    )
    total = tp + fp + tn + fn
    accuracy = (tp + tn) / total if total > 0 else 0.0
    return precision, recall, f1, accuracy


def summary(y_true, y_pred, label="fault"):
    """Return a human-readable P/R/F1 string and a metrics dict.

    ``label`` is the name of the positive class (e.g. 'fault' or 'anomaly').
    """
    tp, fp, tn, fn = confusion_counts(y_true, y_pred)
    precision, recall, f1, accuracy = precision_recall_f1(tp, fp, tn, fn)
    lines = [
        f"{label} class (positive = {label}): " f"TP={tp} FP={fp} TN={tn} FN={fn}",
        f"Precision = {precision:.3f}   Recall = {recall:.3f}     "
        f"F1 = {f1:.3f}   Accuracy = {accuracy:.3f}",
    ]
    result = {
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "accuracy": accuracy,
        "tp": tp,
        "fp": fp,
        "tn": tn,
        "fn": fn,
    }
    return "\n".join(lines), result
