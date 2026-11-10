# main.py -- training + inference entry point for single-sensor-channel
# fault detection on the ALFA UAV dataset.
#
# Usage (run from this project folder with the project venv):
#     ./bin/python main.py                                # train ALL models
#     ./bin/python main.py --model rnn                    # train a single model
#     ./bin/python main.py --mode run --model autoencoder # load + run saved
#     ./bin/python main.py --domain frequency --model all # frequency domain
#
# Pipeline:
#     1. read ONE sensor channel from every flight scenario (dataset.build_dataset)
#     2. split the scenarios into a training and a validation set
#     3. train the requested model type(s), checkpointing to models/<name>Weights
#     4. optionally load the saved weights and run an inference over the data
#
# Model shapes:
#     - RNN / BiLSTM ingest a (seq_len, 1) window of the single channel and
#       output a fault probability (binary classification).
#     - Autoencoder ingests the flattened (seq_len,) window -- 100 features, which
#       matches input_dim in the config -- and reconstructs it; the reconstruction
#       error is the anomaly score.

import os
import json
import time
import argparse

import numpy as np
import torch
from torch import nn
from torch import optim
from torch.utils.data import Dataset
from torch.utils.tensorboard import SummaryWriter

import dataset as D
import metrics
import ugv_dataset as U
from models.Trainer import Trainer
from models.Autoencoder import Autoencoder
from models.BiLstm import BiLSTM
from models.Rnn import RNN


# --- Configuration -----------------------------------------------------------
# Loaded relative to this file so the script works from any working directory.
CONFIG_PATH = os.path.join(
     os.path.dirname(os.path.abspath(__file__)),
     "models",
     "utils",
     "ModelParameters.json",
)
with open(CONFIG_PATH, "r", encoding="utf-8") as f:
    config = json.load(f)
f.close()

ROOT = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(ROOT, "datasets", "processed")
UGV_DIR = os.path.join(ROOT, "datasets", "ugv_faulty")
RUNS_DIR = os.path.join(ROOT, "runs")
MODELS_DIR = os.path.join(ROOT, "models")

# Weight subfolder for each model type (created if missing).
WEIGHT_DIRS = {
     "Autoencoder": os.path.join(MODELS_DIR, "autoencoderWeights"),
     "RNN": os.path.join(MODELS_DIR, "rnnWeights"),
     "BiLSTM": os.path.join(MODELS_DIR, "bilstmWeights"),
}


def weight_folder(model_name, domain, dataset="alfa", paired=False, tag=""):
     # Return the checkpoint folder for a model and analysis domain. The time
     # domain keeps its historical subfolder; the frequency domain uses a
     # sibling ``<name>Weights_freq`` folder so the two never overwrite each other.
     # UGV runs add ``_ugv`` (and ``_paired`` for reference features) so they
     # never overwrite the ALFA or single-channel weights.
    base = WEIGHT_DIRS[model_name]
    if dataset == "ugv":
        base += "_ugv_paired" if paired else "_ugv"
    if tag:   # --tag keeps separate experiments apart, e.g. *_ugv_paired_long
        base += f"_{tag}"
    if domain == "time":
        return base
    return base + "_freq"


# Optimizer selected from the config (Adam / SGD).
if config["Trainer"]["optimizer"] == "Adam":
    OptimizerClass = optim.Adam
elif config["Trainer"]["optimizer"] == "SGD":
    OptimizerClass = optim.SGD
else:
    raise ValueError(f"Unsupported optimizer: {config['Trainer']['optimizer']}")


# --- Datasets ----------------------------------------------------------------
class WindowDataset(Dataset):
     # Yields (window, target) pairs for the Trainer.
     # - classification models: target is the integer fault label (0/1).
     # - autoencoder (labels is None): target is the window itself (reconstruct).
     # When ``flatten`` is True the window is flattened to a 1-D vector so the
     # autoencoder can treat the whole window as its input feature vector.

    def __init__(self, windows, labels=None, flatten=False):
        self.windows = torch.as_tensor(
             np.ascontiguousarray(windows), dtype=torch.float32
        )
        self.labels = (
             torch.as_tensor(labels, dtype=torch.long)
            if labels is not None
            else None
        )
        self.flatten = flatten

    def __len__(self):
        return self.windows.shape[0]

    def __getitem__(self, idx):
        x = self.windows[idx].flatten() if self.flatten else self.windows[idx]
        target = x if self.labels is None else self.labels[idx]
        return x, target


# --- Model factory -----------------------------------------------------------
def build_model(name, input_dim, num_classes=2, readout="full"):
     # Instantiate a model by name for the given per-step input dimension.
     # The RNN/BiLSTM are binary fault classifiers and therefore always use
     # ``num_classes`` output units (fault vs normal), independent of the
     # config's ``shared.output_dim``.
    if name == "Autoencoder":
        return Autoencoder(
             input_dim=input_dim,
             hidden_dim=config["autoencoder"]["hidden_dim"],
        )
    if name == "RNN":
        return RNN(
             input_dim=input_dim,
             hidden_dim=config["rnn"]["hidden_dim"],
             output_dim=num_classes,
             num_layers=config["rnn"]["num_layers"],
        )
    if name == "BiLSTM":
        return BiLSTM(
             input_dim=input_dim,
             hidden_dim=config["bilstm"]["hidden_dim"],
             output_dim=num_classes,
             num_layers=config["bilstm"]["num_layers"],
             readout=readout,
        )
    raise ValueError(f"Unknown model: {name}")


def _model_input_dim(name, feat_dim, seq_len):
     # Input feature size each model type expects.
    if name == "Autoencoder":
        return seq_len * feat_dim   # flattened window (matches input_dim=100)
    return feat_dim   # per-step channel width for RNN/BiLSTM


def _class_weights(labels):
     # Return inverse-frequency weights for the two classes (0, 1).
    labels = np.asarray(labels).ravel()
    counts = np.bincount(labels.astype(int), minlength=2).astype(np.float64)
    if counts.sum() == 0:
        return None
    weights = counts.sum() / (len(counts) * counts)
    weights = weights / weights.mean()   # normalize so the average weight is 1
    return torch.as_tensor(weights, dtype=torch.float32)


def _make_criterion(name, train_labels=None):
     # Loss matching the model type. RNN/BiLSTM are binary fault classifiers; when
     # ``train_labels`` are provided the loss is class-weighted (inverse class
     # frequency) to counter the fault/normal imbalance. The autoencoder uses MSE.
    if name == "Autoencoder":
        return nn.MSELoss()
    weights = None
    if train_labels is not None:
        weights = _class_weights(train_labels)
        if weights is not None:
            print(
                 "   Class weights (inverse frequency): "
                 f"normal={weights[0].item():.4f}, fault={weights[1].item():.4f}"
            )
    return nn.CrossEntropyLoss(weight=weights)


# --- Evaluation --------------------------------------------------------------
def _predict(name, model, windows, batch_size=1024):
     # Run a model over windows; return predictions / scores as a numpy array.
     # Classifiers yield a 0/1 fault prediction; the autoencoder yields the
     # per-window reconstruction error (anomaly score).
    flatten = name == "Autoencoder"
    model.eval()
    out = []
    with torch.no_grad():
        for start in range(0, len(windows), batch_size):
            x = torch.as_tensor(
                 np.ascontiguousarray(windows[start:start + batch_size]),
                 dtype=torch.float32,
            )
            if flatten:
                xf = x.flatten(start_dim=1)
                out.append(((model(xf) - xf) ** 2).mean(dim=1).numpy())
            else:
                out.append(torch.argmax(model(x), dim=1).numpy())
    return np.concatenate(out).astype(np.float64)


def _anomaly_labels(scores, labels, threshold=None):
     # Turn autoencoder error scores into binary labels via a threshold. When no
     # threshold is given it is the max mean-error of the *normal* windows,
     # flagging anything that reconstructs worse than the worst normal case.
    if threshold is not None:   # scalar or one value per window
        return (scores > threshold).astype(int), threshold
    if not np.any(labels == 0):
        threshold = float(np.median(scores))
    else:
        threshold = float(scores[labels == 0].max())
    return (scores > threshold).astype(int), threshold


def _channel_keys(info):
    return (info["sensor"] + "/" + info["column"]).to_numpy()


def _clean_error_thresholds(model, clean_windows, clean_info, percentile, weight_dir):
     # Per-channel fault thresholds for the autoencoder: the given percentile of
     # the reconstruction error over clean *training* windows of each channel
     # (noisy IMU axes and smooth wheel speeds have very different error floors).
     # Saved beside the weights so ``--mode run`` reuses them.
    errors = _predict("Autoencoder", model, clean_windows)
    keys = _channel_keys(clean_info)
    thresholds = {k: float(np.percentile(errors[keys == k], percentile))
                  for k in sorted(set(keys))}
    with open(os.path.join(weight_dir, "threshold.json"), "w", encoding="utf-8") as f:
        json.dump({"percentile": percentile, "per_channel": thresholds,
                   "clean_train_windows": len(errors)}, f, indent=2)
    print(f"   Clean-train reconstruction error p{percentile:g} per channel "
          f"({len(errors)} windows):")
    for key, value in thresholds.items():
        print(f"      {key:22s} {value:.4e}")
    return thresholds


def _window_thresholds(thresholds, info):
     # Map per-channel thresholds onto each window of ``info``.
    return np.array([thresholds[k] for k in _channel_keys(info)])


def print_summary(name, model, windows, labels, positive="fault", info=None,
                  threshold=None):
     # Evaluate ``model`` on ``windows`` and print a P/R/F1 summary. Classifiers
     # are scored on their fault predictions; the autoencoder is scored by
     # thresholding its reconstruction error (``threshold`` when given).
    scores = _predict(name, model, windows)
    if name == "Autoencoder":
        preds, threshold = _anomaly_labels(scores, labels, threshold)
        if np.ndim(threshold) == 0:
            print(f"   Autoencoder anomaly threshold = {threshold:.4e}")
        positive = "anomaly"
    else:
        preds = scores.astype(int)
    text, result = metrics.summary(labels, preds, label=positive)
    print(f"   {name} {positive} metrics on {len(labels)} windows:\n{text}")
    if info is not None:
        # Recall per injected fault type, and false-positive rate on clean windows.
        for kind in sorted(set(info["fault_type"]) - {"none"}):
            rows = (info["fault_type"] == kind).to_numpy()
            print(f"   {kind:>8} recall = {preds[rows].mean():.3f} ({rows.sum()} windows)")
        clean = (info["fault_type"] == "none").to_numpy()
        print(f"   clean false-positive rate = {preds[clean].mean():.4f}")
    return result


# --- Data loading ------------------------------------------------------------
def build_train_val(args):
     # Load the single sensor channel and return the train/val data as
     # ``((train_windows, train_labels), (val_windows, val_labels))`` NumPy arrays
     # (validation may be ``(None, None)``).
    if args.dataset == "ugv":
        return build_ugv_train_val(args)
    print(
         f'Loading channel "{args.column}" from topic "{args.topic}" '
         f"(domain={args.domain}, features_per_step={args.features}, "
         f"seq_len={args.seq_len}) ..."
    )
    windows, labels, _ = D.build_dataset(
         DATA_DIR,
         topic=args.topic,
         column=args.column,
         seq_len=args.seq_len,
         features_per_step=args.features,
         domain=args.domain,
    )
    fault = int((labels == 1).sum())
    normal = int((labels == 0).sum())
    print(f"    {len(labels)} scenarios -> {fault} faulty, {normal} normal")

    return D.make_train_val(
         windows, labels, val_fraction=args.val_fraction, seed=args.seed
    )


def build_ugv_train_val(args):
     # Load every faultable channel of the simulated UGV variants (ugv_dataset.py),
     # split train / val / test by source run. All splits and their per-window
     # info stay on ``args`` (``args.splits``) for thresholds and test summaries.
    if getattr(args, "splits", None) is None:
        print(f'Loading UGV fault variants from "{args.ugv_dir}" (domain={args.domain}, '
              f"seq_len={args.seq_len}, paired={args.paired}) ...")
        args.splits = U.build_ugv_splits(
             args.ugv_dir, seq_len=args.seq_len, val_runs=args.val_runs,
             test_runs=args.test_runs, seed=args.seed, domain=args.domain,
             paired=args.paired, extra_train_dirs=args.extra_train_dir,
        )
        for split, (windows, labels, _) in args.splits.items():
            print(f"    {split}: {len(labels)} windows -> {int(labels.sum())} faulty")
    args.train_info = args.splits["train"][2]
    args.val_info = args.splits["val"][2]
    (tw, ty, _), (vw, vy, _) = args.splits["train"], args.splits["val"]
    return (tw, ty), (vw, vy)


def ugv_summaries(name, model, args, thresholds=None):
     # Print validation and held-out test summaries; return the test result.
    for split in ("val", "test"):
        windows, labels, info = args.splits[split]
        threshold = None if thresholds is None else _window_thresholds(thresholds, info)
        print(f"--- {split} summary ({len(labels)} windows) ---")
        result = print_summary(name, model, windows, labels, info=info, threshold=threshold)
    return result


# --- Training ----------------------------------------------------------------
def train_model(name, train_windows, train_labels, val_windows, val_labels,
                args, feat_dim):
     # Train one model type and checkpoint its weights to its subfolder.
    print(f"\n=== Training {name} ===")
    input_dim = _model_input_dim(name, feat_dim, args.window_len)
    flatten = name == "Autoencoder"

    if val_windows is None:
        val_windows = val_labels = None
    if flatten and args.dataset == "ugv":
        # Reconstruction models learn normal behaviour only.
        clean_info = args.train_info[train_labels == 0].reset_index(drop=True)
        train_windows = train_windows[train_labels == 0]
        train_labels = train_labels[train_labels == 0]
        if val_windows is not None:
            val_windows, val_labels = val_windows[val_labels == 0], val_labels[val_labels == 0]

     # The autoencoder ignores labels (it reconstructs its own input); the
     # RNN/BiLSTM classify fault vs normal.
    train_ds = WindowDataset(
         train_windows,
         labels=None if flatten else train_labels,
         flatten=flatten,
    )
    val_ds = (
        WindowDataset(val_windows, labels=None if flatten else val_labels,
                      flatten=flatten)
        if val_windows is not None
        else None
    )

    model = build_model(name, input_dim)
    criterion = _make_criterion(name, train_labels if not flatten else None)
    optimizer = OptimizerClass(
         model.parameters(), lr=config["Trainer"]["learning_rate"]
    )

    weight_dir = weight_folder(name, args.domain, args.dataset, args.paired, args.tag)
    os.makedirs(weight_dir, exist_ok=True)
    # Record the readout so --mode run rebuilds the same network.
    with open(os.path.join(weight_dir, "model_config.json"), "w", encoding="utf-8") as f:
        json.dump({"readout": "full", "window_len": args.window_len}, f, indent=2)

    epochs = (
        args.epochs if args.epochs is not None
        else config["Trainer"]["num_epochs"]
    )

    # Live curves: tensorboard --logdir runs  (one subfolder per training run).
    writer = None
    if not args.no_tensorboard:
        run_name = (f"{args.dataset}{'_paired' if args.paired else ''}"
                    f"{'_' + args.tag if args.tag else ''}_"
                    f"{name.lower()}_{args.domain}_"
                    f"{time.strftime('%Y%m%d-%H%M%S')}")
        writer = SummaryWriter(os.path.join(args.tensorboard_dir, run_name))
        writer.add_text("args", json.dumps(
             {k: v for k, v in vars(args).items()
              if not k.endswith("_info") and k != "splits"},
             default=str, indent=2))
        print(f"TensorBoard run: {os.path.join(args.tensorboard_dir, run_name)}")

    trainer = Trainer(
        model=model,
        train_data=train_ds,
        val_data=val_ds,
        criterion=criterion,
        optimizer=optimizer,
        batch_size=args.batch_size or config["Trainer"]["batch_size"],
        num_epochs=epochs,
        save_dir=weight_dir,
        save_every=args.save_every,
        device=args.device,
        writer=writer,
        # UGV classifiers keep the best validation F1; faults are ~1% of windows.
        select_by="f1" if args.dataset == "ugv" and not flatten else "loss",
    )
    trainer.train()
    model.to("cpu")   # evaluation below runs on the CPU
    print(f"{name} weights saved to {weight_dir}")

     # Report precision / recall / F1 on the held-out validation set when
     # available, otherwise on the training windows.
    if args.dataset == "ugv":
        # Score best_model.pt on every validation and held-out test window.
        model.load_state_dict(torch.load(os.path.join(weight_dir, "best_model.pt"),
                                         map_location="cpu"))
        thresholds = (_clean_error_thresholds(   # train_windows are clean-only here
             model, train_windows, clean_info, args.ae_percentile, weight_dir)
             if flatten else None)
        print("=== best_model.pt ===")
        result = ugv_summaries(name, model, args, thresholds)
    elif val_windows is not None:
        print("--- Validation summary ---")
        result = print_summary(name, model, val_windows, val_labels)
    else:
        print("--- Training-data summary ---")
        result = print_summary(name, model, train_windows, train_labels)
    if writer is not None:
        for key in ("precision", "recall", "f1", "accuracy"):
            writer.add_scalar(f"final/{key}", result[key], epochs)
        writer.close()
    return model


# --- Inference / running a trained model -------------------------------------
def _saved_readout(weight_dir):
     # BiLSTM readout used when ``weight_dir`` was trained; folders without
     # model_config.json predate the 'full' readout and use 'last'.
    path = os.path.join(weight_dir, "model_config.json")
    if not os.path.exists(path):
        return "last"
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f).get("readout", "last")


def _ugv_ae_thresholds(model, args, weight_dir):
     # Recompute the autoencoder's per-channel thresholds from clean train windows.
    train_windows, train_labels, train_info = args.splits["train"]
    clean = train_labels == 0
    return _clean_error_thresholds(model, train_windows[clean],
                                   train_info[clean].reset_index(drop=True),
                                   args.ae_percentile, weight_dir)


def _load_trained(name, args, feat_dim):
    path = os.path.join(weight_folder(name, args.domain, args.dataset, args.paired, args.tag),
                        "best_model.pt")
    model = build_model(name, _model_input_dim(name, feat_dim, args.window_len),
                        readout=_saved_readout(os.path.dirname(path)))
    model.load_state_dict(torch.load(path, map_location="cpu"))
    model.eval()
    return model, os.path.dirname(path)


def run_ensemble(args, feat_dim):
     # UGV ensemble: a window is faulty when the BiLSTM classifier OR the
     # autoencoder (per-channel reconstruction threshold) flags it. The two are
     # complementary: the classifier finds offsets, the autoencoder finds spikes
     # and noise bursts.
    bilstm, _ = _load_trained("BiLSTM", args, feat_dim)
    autoencoder, ae_dir = _load_trained("Autoencoder", args, feat_dim)
    thresholds = _ugv_ae_thresholds(autoencoder, args, ae_dir)
    for split in ("val", "test"):
        windows, labels, info = args.splits[split]
        classifier = _predict("BiLSTM", bilstm, windows).astype(int)
        errors = _predict("Autoencoder", autoencoder, windows)
        anomaly = (errors > _window_thresholds(thresholds, info)).astype(int)
        print(f"--- Ensemble (BiLSTM OR Autoencoder) {split} summary ({len(labels)} windows) ---")
        for label, preds in (("BiLSTM", classifier), ("Autoencoder", anomaly),
                             ("Ensemble", classifier | anomaly)):
            text, _ = metrics.summary(labels, preds)
            per_type = ", ".join(
                 f"{kind} {preds[(info['fault_type'] == kind).to_numpy()].mean():.2f}"
                 for kind in ("short", "noise", "constant"))
            print(f"   {label}: {text.splitlines()[1]}\n      recall {per_type}; "
                  f"clean FPR {preds[(info['fault_type'] == 'none').to_numpy()].mean():.4f}")



def run_model(name, args, feat_dim):
     # Load saved weights and run a trained model once over the data.
    weight_path = os.path.join(
         weight_folder(name, args.domain, args.dataset, args.paired, args.tag), "best_model.pt")
    if not os.path.exists(weight_path):
        print(
             f"No trained weights at {weight_path}. Train first: "
             f"./bin/python main.py --model {name} --domain {args.domain}"
        )
        return None

    print(f"\n=== Running {name} (loading {weight_path}) ===")
    input_dim = _model_input_dim(name, feat_dim, args.window_len)
    model = build_model(name, input_dim,
                        readout=_saved_readout(os.path.dirname(weight_path)))
    model.load_state_dict(torch.load(weight_path, map_location="cpu"))
    model.eval()

    if args.dataset == "ugv":
        thresholds = None
        if name == "Autoencoder":
            thresholds = _ugv_ae_thresholds(model, args, os.path.dirname(weight_path))
        ugv_summaries(name, model, args, thresholds)
        return model
    windows, labels, _ = D.build_dataset(
         DATA_DIR,
         topic=args.topic,
         column=args.column,
         seq_len=args.seq_len,
         features_per_step=args.features,
         domain=args.domain,
    )

    print(f"--- Inference on {len(labels)} windows (domain={args.domain}) ---")
    print_summary(name, model, windows, labels)
    return model


# --- Entry point -------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Train / run single-sensor-channel fault-detection models."
    )
    parser.add_argument("--mode", choices=["train", "run"], default="train")
    parser.add_argument(
         "--model",
        choices=["all", "autoencoder", "rnn", "bilstm", "ensemble"],
        default="all",
    )
    parser.add_argument("--dataset", choices=["alfa", "ugv"], default="alfa",
                        help="ALFA flights (datasets/processed) or simulated UGV "
                             "fault variants (fault_injection.py output).")
    parser.add_argument("--ugv-dir", default=UGV_DIR)
    parser.add_argument("--tag", default="",
                        help="Suffix for weight folders and TensorBoard runs, e.g. long.")
    parser.add_argument("--extra-train-dir", action="append", default=[],
                        help="UGV: additional fault_injection.py output used for training "
                             "only (repeatable); validation and test stay in --ugv-dir.")
    parser.add_argument("--val-runs", type=int, default=5,
                        help="UGV: source runs held out for validation.")
    parser.add_argument("--test-runs", type=int, default=5,
                        help="UGV: source runs held out for the final test.")
    parser.add_argument("--single-channel", dest="paired", action="store_false",
                        help="UGV: omit the cross-sensor reference feature.")
    parser.add_argument("--tensorboard-dir", default=RUNS_DIR,
                        help="TensorBoard log root; view with tensorboard --logdir runs.")
    parser.add_argument("--no-tensorboard", action="store_true")
    parser.add_argument("--device", default="cpu",
                        help="Training device: cpu, mps (Apple GPU), or cuda.")
    parser.add_argument("--ae-percentile", type=float, default=99.0,
                        help="UGV autoencoder: flag windows whose reconstruction error exceeds "
                             "this percentile of their channel's clean training errors.")
    parser.add_argument("--batch-size", type=int, default=None,
                        help="Override batch_size from the config.")
    parser.add_argument("--topic", default=D.DEFAULT_TOPIC)
    parser.add_argument("--column", default=D.DEFAULT_COLUMN)
    parser.add_argument(
         "--domain", choices=["time", "frequency"], default="time",
        help="Analysis domain: raw time-series samples (default) "
               "or FFT frequency spectrum.",
    )
    parser.add_argument("--features", type=int, default=1,
                        help="Feature vector width per timestep (1 = raw channel).")
    parser.add_argument("--seq-len", type=int, default=100)
    parser.add_argument("--val-fraction", type=float, default=0.2)
    parser.add_argument("--epochs", type=int, default=None,
                        help="Override num_epochs from the config.")
    parser.add_argument("--save-every", type=int, default=10,
                        help="Checkpoint every N epochs to the model folder.")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    (train_windows, train_labels), (val_windows, val_labels) = build_train_val(
         args
    )
    if train_windows is None:
        print("No training data could be built; aborting.")
        return

    feat_dim = train_windows.shape[2]
    args.window_len = train_windows.shape[1]   # time steps (or FFT bins) per window
    print(
         f"Train: {len(train_windows)} windows, "
         f"Val: {0 if val_windows is None else len(val_windows)} windows, "
         f"feat_dim={feat_dim}"
    )

    if args.mode == "run":
        if args.model == "ensemble":
            if args.dataset != "ugv":
                raise SystemExit("--model ensemble is only defined for --dataset ugv")
            run_ensemble(args, feat_dim)
        else:
            run_model(_normalize_model(args.model), args, feat_dim)
        return
    if args.model == "ensemble":
        raise SystemExit("--model ensemble combines trained models; use --mode run")

    models = (
        ["Autoencoder", "RNN", "BiLSTM"]
        if args.model == "all"
        else [_normalize_model(args.model)]
    )
    for name in models:
        train_model(
             name, train_windows, train_labels,
             val_windows, val_labels, args, feat_dim,
        )

    print("\nDone. Run inference with, e.g.:")
    print("    ./bin/python main.py --mode run --model autoencoder")


def _normalize_model(model_arg):
    return {"autoencoder": "Autoencoder", "rnn": "RNN", "bilstm": "BiLSTM"}[
        model_arg
    ]


if __name__ == "__main__":
    main()
