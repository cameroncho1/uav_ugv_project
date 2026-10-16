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
import argparse

import numpy as np
import torch
from torch import nn
from torch import optim
from torch.utils.data import Dataset

import dataset as D
import metrics
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
MODELS_DIR = os.path.join(ROOT, "models")

# Weight subfolder for each model type (created if missing).
WEIGHT_DIRS = {
     "Autoencoder": os.path.join(MODELS_DIR, "autoencoderWeights"),
     "RNN": os.path.join(MODELS_DIR, "rnnWeights"),
     "BiLSTM": os.path.join(MODELS_DIR, "bilstmWeights"),
}


def weight_folder(model_name, domain):
     # Return the checkpoint folder for a model and analysis domain. The time
     # domain keeps its historical subfolder; the frequency domain uses a
     # sibling ``<name>Weights_freq`` folder so the two never overwrite each other.
    base = WEIGHT_DIRS[model_name]
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
def build_model(name, input_dim, num_classes=2):
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
def _predict(name, model, windows):
     # Run a model over windows; return predictions / scores as a numpy array.
     # Classifiers yield a 0/1 fault prediction; the autoencoder yields the
     # per-window reconstruction error (anomaly score).
    flatten = name == "Autoencoder"
    model.eval()
    out = []
    with torch.no_grad():
        for i in range(len(windows)):
            x = torch.as_tensor(
                 np.ascontiguousarray(windows[i]), dtype=torch.float32
            )
            if flatten:
                xf = x.flatten().unsqueeze(0)
                pred = model(xf)
                err = float(((pred - xf) ** 2).mean().item())
                out.append(err)
            else:
                prob = torch.softmax(model(x.unsqueeze(0)), dim=1)
                out.append(int(torch.argmax(prob, dim=1).item()))
    return np.asarray(out, dtype=np.float64)


def _anomaly_labels(scores, labels):
     # Turn autoencoder error scores into binary labels via a threshold. The
     # threshold is the max mean-error of the *normal* windows, flagging anything
     # that reconstructs worse than the worst normal case as faulty.
    if not np.any(labels == 0):
        threshold = float(np.median(scores))
    else:
        threshold = float(scores[labels == 0].max())
    return (scores > threshold).astype(int), threshold


def print_summary(name, model, windows, labels, positive="fault"):
     # Evaluate ``model`` on ``windows`` and print a P/R/F1 summary. Classifiers
     # are scored on their fault predictions; the autoencoder is scored by
     # thresholding its reconstruction error against the normal data.
    scores = _predict(name, model, windows)
    if name == "Autoencoder":
        preds, threshold = _anomaly_labels(scores, labels)
        print(f"   Autoencoder anomaly threshold = {threshold:.4e}")
        positive = "anomaly"
    else:
        preds = scores.astype(int)
    text, result = metrics.summary(labels, preds, label=positive)
    print(f"   {name} {positive} metrics on {len(labels)} windows:\n{text}")
    return result


# --- Data loading ------------------------------------------------------------
def build_train_val(args):
     # Load the single sensor channel and return the train/val data as
     # ``((train_windows, train_labels), (val_windows, val_labels))`` NumPy arrays
     # (validation may be ``(None, None)``).
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


# --- Training ----------------------------------------------------------------
def train_model(name, train_windows, train_labels, val_windows, val_labels,
                args, feat_dim):
     # Train one model type and checkpoint its weights to its subfolder.
    print(f"\n=== Training {name} ===")
    input_dim = _model_input_dim(name, feat_dim, args.seq_len)
    flatten = name == "Autoencoder"

    if val_windows is None:
        val_windows = val_labels = None

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

    weight_dir = weight_folder(name, args.domain)
    os.makedirs(weight_dir, exist_ok=True)

    epochs = (
        args.epochs if args.epochs is not None
        else config["Trainer"]["num_epochs"]
    )

    trainer = Trainer(
        model=model,
        train_data=train_ds,
        val_data=val_ds,
        criterion=criterion,
        optimizer=optimizer,
        batch_size=config["Trainer"]["batch_size"],
        num_epochs=epochs,
        save_dir=weight_dir,
        save_every=args.save_every,
    )
    trainer.train()
    print(f"{name} weights saved to {weight_dir}")

     # Report precision / recall / F1 on the held-out validation set when
     # available, otherwise on the training windows.
    if val_windows is not None:
        print("--- Validation summary ---")
        print_summary(name, model, val_windows, val_labels)
    else:
        print("--- Training-data summary ---")
        print_summary(name, model, train_windows, train_labels)
    return model


# --- Inference / running a trained model -------------------------------------
def run_model(name, args, feat_dim):
     # Load saved weights and run a trained model once over the data.
    weight_path = os.path.join(weight_folder(name, args.domain), "best_model.pt")
    if not os.path.exists(weight_path):
        print(
             f"No trained weights at {weight_path}. Train first: "
             f"./bin/python main.py --model {name} --domain {args.domain}"
        )
        return None

    print(f"\n=== Running {name} (loading {weight_path}) ===")
    input_dim = _model_input_dim(name, feat_dim, args.seq_len)
    model = build_model(name, input_dim)
    model.load_state_dict(torch.load(weight_path, map_location="cpu"))
    model.eval()

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
        choices=["all", "autoencoder", "rnn", "bilstm"],
        default="all",
    )
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
    print(
         f"Train: {len(train_windows)} windows, "
         f"Val: {0 if val_windows is None else len(val_windows)} windows, "
         f"feat_dim={feat_dim}"
    )

    if args.mode == "run":
        run_model(_normalize_model(args.model), args, feat_dim)
        return

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
