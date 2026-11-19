# main.py -- training + inference entry point for single-sensor-channel
# fault detection on the ALFA UAV dataset.
#
# Usage (run from this project folder with the project venv):
#     ./bin/python main.py                                # train ALL models
#     ./bin/python main.py --model gru                    # train a single model
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
#     - GRU / BiLSTM ingest a (seq_len, 1) window of the single channel and
#       output a fault probability (binary classification). The plain RNN
#       (--model rnn) remains for the weights trained before the GRU replaced it.
#     - Autoencoder ingests the flattened (seq_len,) window -- 100 features, which
#       matches input_dim in the config -- and reconstructs it; the reconstruction
#       error is the anomaly score.
#
# Simulated UGV data (--dataset ugv, see MODELS.md):
#     --layout channel       one window per sensor channel (ugv_dataset.py); the
#                            classifiers output fault / clean for that channel.
#     --layout multivariate  every time step carries all sensors (ugv_multivariate.py);
#                            the classifiers output one fault logit per channel
#                            (sigmoid, multi-label) and the autoencoder scores each
#                            channel by its feature's reconstruction error.
#     Scores for both layouts go to models/results/<weight folder>.json (ugv_eval.py).

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
import ugv_eval as E
import ugv_multivariate as M
from models.Trainer import Trainer
from models.Autoencoder import LEGACY as LEGACY_AE, Autoencoder
from models.BiLstm import BiLSTM
from models.Gru import GRU
from models.Rnn import RNN
from models.Specialists import BlockBiLSTM, BlockGRU, GroupedAutoencoder, effective_parameters


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
RESULTS_DIR = os.path.join(MODELS_DIR, "results")   # UGV scores per weight folder

# Weight subfolder for each model type (created if missing).
WEIGHT_DIRS = {
     "Autoencoder": os.path.join(MODELS_DIR, "autoencoderWeights"),
     "RNN": os.path.join(MODELS_DIR, "rnnWeights"),
     "GRU": os.path.join(MODELS_DIR, "gruWeights"),
     "BiLSTM": os.path.join(MODELS_DIR, "bilstmWeights"),
}


def weight_folder(model_name, domain, dataset="alfa", paired=False, tag="", layout="channel"):
     # Return the checkpoint folder for a model and analysis domain. The time
     # domain keeps its historical subfolder; the frequency domain uses a
     # sibling ``<name>Weights_freq`` folder so the two never overwrite each other.
     # UGV runs add ``_ugv`` (``_paired`` for reference features, ``_mv`` for the
     # multivariate layout) so they never overwrite the ALFA or other UGV weights.
    base = WEIGHT_DIRS[model_name]
    if dataset == "ugv" and layout == "multivariate":
        base += "_ugv_mv"
    elif dataset == "ugv":
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
     # - multi-label models (2-D labels): target is a float 0/1 vector per window.
     # When ``flatten`` is True the window is flattened to a 1-D vector so the
     # autoencoder can treat the whole window as its input feature vector.
     # ``idx`` may be a list of indices: the Trainer then fetches whole batches.

    batch_indexable = True

    def __init__(self, windows, labels=None, flatten=False):
        self.windows = torch.as_tensor(
             np.ascontiguousarray(windows), dtype=torch.float32
        )
        self.labels = (
             torch.as_tensor(labels, dtype=torch.float32 if np.ndim(labels) == 2
                             else torch.long)
            if labels is not None
            else None
        )
        self.flatten = flatten

    def __len__(self):
        return self.windows.shape[0]

    def __getitem__(self, idx):
        x = self.windows[idx]
        if self.flatten:
            x = x.flatten(start_dim=-2)   # (L, F) -> (L*F,), also for a batch
        target = x if self.labels is None else self.labels[idx]
        return x, target


# --- Model factory -----------------------------------------------------------
def build_model(name, input_dim, num_classes=2, readout="full", ae_layout=None):
     # Instantiate a model by name for the given per-step input dimension.
     # The RNN/BiLSTM are binary fault classifiers and therefore always use
     # ``num_classes`` output units (fault vs normal), independent of the
     # config's ``shared.output_dim``. ``ae_layout`` overrides the autoencoder's
     # hidden layers and activation from the config (see _ae_layout).
    if name == "Autoencoder":
        return Autoencoder(input_dim=input_dim, **(ae_layout or _ae_layout()))
    if name == "RNN":
        return RNN(
             input_dim=input_dim,
             hidden_dim=config["rnn"]["hidden_dim"],
             output_dim=num_classes,
             num_layers=config["rnn"]["num_layers"],
        )
    if name == "GRU":
        return GRU(
             input_dim=input_dim,
             hidden_dim=config["gru"]["hidden_dim"],
             output_dim=num_classes,
             num_layers=config["gru"]["num_layers"],
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
    if train_labels is not None and np.ndim(train_labels) == 2:
        # Multi-label: one sigmoid per channel, positives weighted by the
        # channel's clean / faulty window ratio (the same balance as above).
        positives = train_labels.sum(axis=0)
        pos_weight = np.where(positives > 0,
                              (len(train_labels) - positives) / np.maximum(positives, 1), 1.0)
        print("   Positive weights (clean / faulty windows per channel): "
              + ", ".join(f"{t}={w:.0f}" for t, w in zip(M.TARGETS, pos_weight)))
        return nn.BCEWithLogitsLoss(pos_weight=torch.as_tensor(pos_weight, dtype=torch.float32))
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
def _predict(name, model, windows, target_features=None, batch_size=1024):
     # Run a model over windows; return predictions / scores as a numpy array.
     # Classifiers yield a 0/1 fault prediction; the autoencoder yields the
     # per-window reconstruction error (anomaly score). With ``target_features``
     # (multivariate layout) both are ``(N, channels)``: one sigmoid decision per
     # channel, or the reconstruction error of each channel's feature column.
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
                error = ((model(xf) - xf) ** 2).reshape(x.shape)
                out.append(error.mean(dim=(1, 2)).numpy() if target_features is None
                           else error[:, :, target_features].mean(dim=1).numpy())
            elif target_features is None:
                out.append(torch.argmax(model(x), dim=1).numpy())
            else:
                out.append((model(x) > 0).numpy())   # logit > 0 is probability > 0.5
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


def _multivariate(args):
    return args.dataset == "ugv" and getattr(args, "layout", "channel") == "multivariate"


def _ugv_scores(name, model, windows, args):
     # Classifier decisions or autoencoder errors: (N,) in the channel layout,
     # (N, 11) in the multivariate layout.
    return _predict(name, model, windows,
                    M.TARGET_FEATURES if _multivariate(args) else None)


def _clean_mask(labels):
     # Fault-free windows: no channel faulty (multi-label) or label 0.
    labels = np.asarray(labels)
    return ~labels.any(axis=1) if labels.ndim == 2 else labels == 0


def _clean_error_thresholds(model, clean_windows, clean_info, percentile, weight_dir, args):
     # Per-channel fault thresholds for the autoencoder: the given percentile of
     # the reconstruction error over clean *training* windows of each channel
     # (noisy IMU axes and smooth wheel speeds have very different error floors).
     # Saved beside the weights so ``--mode run`` reuses them.
    errors = _ugv_scores("Autoencoder", model, clean_windows, args)
    if errors.ndim == 2:   # multivariate: one error column per channel
        thresholds = {t: float(np.percentile(errors[:, k], percentile))
                      for k, t in enumerate(M.TARGETS)}
    else:
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


def _window_thresholds(thresholds, info, args):
     # Map per-channel thresholds onto each window of ``info`` (channel layout)
     # or onto the channel axis of (N, 11) errors (multivariate layout).
    if _multivariate(args):
        return np.array([thresholds[t] for t in M.TARGETS])
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
        print(f'Loading UGV fault variants from "{args.ugv_dir}" (layout={args.layout}, '
              f"domain={args.domain}, seq_len={args.seq_len}, paired={args.paired}) ...")
        if _multivariate(args):
            args.splits = M.build_mv_splits(
                 args.ugv_dir, seq_len=args.seq_len, train_stride=args.train_stride,
                 val_runs=args.val_runs, test_runs=args.test_runs, seed=args.seed,
                 domain=args.domain, extra_train_dirs=args.extra_train_dir,
                 extra_val_dirs=args.extra_val_dir, extra_test_dirs=args.extra_test_dir)
        else:
            args.splits = U.build_ugv_splits(
                 args.ugv_dir, seq_len=args.seq_len, val_runs=args.val_runs,
                 test_runs=args.test_runs, seed=args.seed, domain=args.domain,
                 paired=args.paired, extra_train_dirs=args.extra_train_dir,
                 extra_val_dirs=args.extra_val_dir, extra_test_dirs=args.extra_test_dir,
            )
        for split, (windows, labels, _) in args.splits.items():
            print(f"    {split}: {len(labels)} windows -> {int(labels.sum())} faulty"
                  + (" (window, channel) pairs" if labels.ndim == 2 else ""))
    args.train_info = args.splits["train"][2]
    args.val_info = args.splits["val"][2]
    (tw, ty, _), (vw, vy, _) = args.splits["train"], args.splits["val"]
    return (tw, ty), (vw, vy)


def _ugv_preds(name, model, windows, info, args, thresholds=None):
     # 0/1 fault decisions per window (channel layout) or per (window, channel).
    scores = _ugv_scores(name, model, windows, args)
    if name == "Autoencoder":
        return (scores > _window_thresholds(thresholds, info, args)).astype(int)
    return scores.astype(int)


def _ugv_pairs(preds, labels, info, args):
    if _multivariate(args):
        return E.multivariate_pairs(info, labels, preds, M.TARGETS, args.seq_len)
    return E.channel_pairs(info, labels, preds, args.seq_len)


def _eval_splits(args):
    return [s for s in ("val", "test", "test_extra") if s in args.splits]


def _save_results(results, filename):
    os.makedirs(RESULTS_DIR, exist_ok=True)
    path = os.path.join(RESULTS_DIR, filename)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)
    print(f"Scores saved to {path}")


def ugv_summaries(name, model, args, thresholds=None, weight_dir=None):
     # Print validation and held-out test summaries (window-level, event-level,
     # and per channel; see ugv_eval.py); save them to models/results/; return
     # the window-level test result.
    results = {}
    for split in _eval_splits(args):
        windows, labels, info = args.splits[split]
        preds = _ugv_preds(name, model, windows, info, args, thresholds)
        results[split] = E.evaluate(_ugv_pairs(preds, labels, info, args))
        print(f"--- {name} {split} summary ({len(labels)} windows) ---\n"
              f"{E.format_result(results[split])}")
    if weight_dir is not None:
        _save_results(results, os.path.basename(weight_dir) + ".json")
    return results["test"]["windows"]


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
        clean = _clean_mask(train_labels)
        clean_info = args.train_info[clean].reset_index(drop=True)
        train_windows, train_labels = train_windows[clean], train_labels[clean]
        if val_windows is not None:
            clean = _clean_mask(val_labels)
            val_windows, val_labels = val_windows[clean], val_labels[clean]

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

    if args.specialists:
        if not _multivariate(args):
            raise SystemExit("--specialists needs --layout multivariate")
        model = build_specialists(name, args.window_len, feat_dim, config["specialists"])
        print(f"   {name} specialists: {len(M.TARGETS)} per-channel networks, "
              f"{effective_parameters(model):,} parameters")
    else:
        model = build_model(name, input_dim, num_classes=_num_classes(args),
                            ae_layout=_ae_layout(args) if flatten else None)
    criterion = _make_criterion(name, train_labels if not flatten else None)
    optimizer = OptimizerClass(
         model.parameters(), lr=config["Trainer"]["learning_rate"]
    )

    weight_dir = _weight_dir(name, args)
    os.makedirs(weight_dir, exist_ok=True)
    # Record the readout so --mode run rebuilds the same network.
    model_config = {"readout": "full", "window_len": args.window_len}
    if args.specialists:
        model_config["specialists"] = config["specialists"]
    elif flatten:
        model_config["autoencoder"] = _ae_layout(args)
    if _multivariate(args):
        model_config.update(layout="multivariate", seq_len=args.seq_len,
                            features=M.FEATURES, targets=M.TARGETS)
    with open(os.path.join(weight_dir, "model_config.json"), "w", encoding="utf-8") as f:
        json.dump(model_config, f, indent=2)

    epochs = (
        args.epochs if args.epochs is not None
        else config["Trainer"]["num_epochs"]
    )

    # Live curves: tensorboard --logdir runs  (one subfolder per training run).
    writer = None
    if not args.no_tensorboard:
        layout = "_mv" if _multivariate(args) else "_paired" if args.paired else ""
        run_name = (f"{args.dataset}{layout}"
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
        grad_clip=args.grad_clip,
        patience=args.patience,
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
             model, train_windows, clean_info, args.ae_percentile, weight_dir, args)
             if flatten else None)
        print("=== best_model.pt ===")
        result = ugv_summaries(name, model, args, thresholds, weight_dir)
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


def build_specialists(name, window_len, feat_dim, spec):
     # Multivariate layout only: one small network per faultable channel, each seeing
     # only that channel's feature column (models/Specialists.py). ``spec`` holds the
     # per-channel widths (config "specialists"); total parameters match the single
     # networks within about 10%.
    columns = M.TARGET_FEATURES
    if name == "BiLSTM":
        return BlockBiLSTM(columns, hidden=spec["bilstm_hidden"])
    if name == "GRU":
        return BlockGRU(columns, hidden=spec["gru_hidden"])
    if name == "Autoencoder":
        return GroupedAutoencoder(columns, window_len, feat_dim, hidden=spec["ae_hidden"],
                                  bottleneck=spec["ae_bottleneck"])
    raise ValueError(f"No per-channel specialists for {name}")


def _ae_layout(args=None):
     # Autoencoder layout from the config: bottleneck width, hidden layer widths,
     # and activation; --ae-encoder-dims / --ae-decoder-dims override the widths.
     # Training records it in model_config.json.
    ae = config["autoencoder"]
    layout = {"hidden_dim": ae["hidden_dim"], "encoder_dims": ae.get("encoder_dims", []),
              "decoder_dims": ae.get("decoder_dims", []),
              "activation": ae.get("activation", "relu")}
    for key in ("encoder_dims", "decoder_dims"):
        override = getattr(args, "ae_" + key, None)
        if override is not None:
            layout[key] = list(override)
    return layout


# --- Inference / running a trained model -------------------------------------
def _saved_ae_layout(weight_dir):
     # Autoencoder layout used when ``weight_dir`` was trained; folders without an
     # "autoencoder" entry predate the hidden encoder layer (one ReLU layer).
    path = os.path.join(weight_dir, "model_config.json")
    saved = {}
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            saved = json.load(f).get("autoencoder", {})
    return saved or {"hidden_dim": config["autoencoder"]["hidden_dim"], **LEGACY_AE}


def _saved_readout(weight_dir):
     # BiLSTM readout used when ``weight_dir`` was trained; folders without
     # model_config.json predate the 'full' readout and use 'last'.
    path = os.path.join(weight_dir, "model_config.json")
    if not os.path.exists(path):
        return "last"
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f).get("readout", "last")


def _weight_dir(name, args):
    return weight_folder(name, args.domain, args.dataset, args.paired, args.tag,
                         getattr(args, "layout", "channel"))


def _num_classes(args):
     # Binary fault / clean logits, or one logit per channel (multivariate).
    return len(M.TARGETS) if _multivariate(args) else 2


def _ugv_ae_thresholds(model, args, weight_dir):
     # Recompute the autoencoder's per-channel thresholds from clean train windows.
    train_windows, train_labels, train_info = args.splits["train"]
    clean = _clean_mask(train_labels)
    return _clean_error_thresholds(model, train_windows[clean],
                                   train_info[clean].reset_index(drop=True),
                                   args.ae_percentile, weight_dir, args)


def _load_trained(name, args, feat_dim):
    path = os.path.join(_weight_dir(name, args), "best_model.pt")
    weight_dir = os.path.dirname(path)
    saved = {}
    if os.path.exists(os.path.join(weight_dir, "model_config.json")):
        with open(os.path.join(weight_dir, "model_config.json"), "r", encoding="utf-8") as f:
            saved = json.load(f)
    if "specialists" in saved:
        model = build_specialists(name, args.window_len, feat_dim, saved["specialists"])
        model.load_state_dict(torch.load(path, map_location="cpu"))
        model.eval()
        return model, weight_dir
    model = build_model(name, _model_input_dim(name, feat_dim, args.window_len),
                        num_classes=_num_classes(args), readout=_saved_readout(weight_dir),
                        ae_layout=_saved_ae_layout(weight_dir) if name == "Autoencoder" else None)
    model.load_state_dict(torch.load(path, map_location="cpu"))
    model.eval()
    return model, os.path.dirname(path)


def run_ensemble(args, feat_dim):
     # UGV ensemble: a window (channel layout) or a (window, channel) pair
     # (multivariate) is faulty when the BiLSTM classifier OR the autoencoder
     # (per-channel reconstruction threshold) flags it. The two are
     # complementary: the classifier finds offsets, the autoencoder finds spikes
     # and noise bursts.
    bilstm, bilstm_dir = _load_trained("BiLSTM", args, feat_dim)
    autoencoder, ae_dir = _load_trained("Autoencoder", args, feat_dim)
    thresholds = _ugv_ae_thresholds(autoencoder, args, ae_dir)
    results = {}
    for split in _eval_splits(args):
        windows, labels, info = args.splits[split]
        classifier = _ugv_preds("BiLSTM", bilstm, windows, info, args)
        anomaly = _ugv_preds("Autoencoder", autoencoder, windows, info, args, thresholds)
        results[split] = E.evaluate(_ugv_pairs(classifier | anomaly, labels, info, args))
        print(f"--- Ensemble (BiLSTM OR Autoencoder) {split} summary "
              f"({len(labels)} windows) ---\n{E.format_result(results[split])}")
    _save_results(results, os.path.basename(bilstm_dir).replace("bilstm", "ensemble") + ".json")


VOTERS = ("BiLSTM", "GRU", "Autoencoder")


def run_vote(args, feat_dim):
     # Multivariate layout: a (window, channel) pair is faulty when at least two of
     # the BiLSTM, GRU, and autoencoder flag it. Members load from --tag, or from
     # --vote-tag MODEL=TAG per member; all must share the training directories.
    if not _multivariate(args):
        raise SystemExit("--model vote needs --layout multivariate")
    tags = dict(item.split("=", 1) for item in args.vote_tag)
    base_tag, members = args.tag, {}
    for name in VOTERS:
        args.tag = tags.get(name.lower(), base_tag)
        model, weight_dir = _load_trained(name, args, feat_dim)
        thresholds = (_ugv_ae_thresholds(model, args, weight_dir)
                      if name == "Autoencoder" else None)
        members[name] = (model, thresholds)
        print(f"   vote member {name}: {weight_dir}")
    args.tag = base_tag
    results = {}
    for split in _eval_splits(args):
        windows, labels, info = args.splits[split]
        votes = sum(_ugv_preds(name, model, windows, info, args, thresholds)
                    for name, (model, thresholds) in members.items())
        results[split] = E.evaluate(_ugv_pairs((votes >= 2).astype(int), labels, info, args))
        print(f"--- Majority vote (2 of {len(members)}) {split} summary "
              f"({len(labels)} windows) ---\n{E.format_result(results[split])}")
    folder = os.path.basename(_weight_dir("BiLSTM", args)).replace("bilstm", "vote")
    _save_results(results, folder + ".json")


def run_model(name, args, feat_dim):
     # Load saved weights and run a trained model once over the data.
    weight_path = os.path.join(_weight_dir(name, args), "best_model.pt")
    if not os.path.exists(weight_path):
        print(
             f"No trained weights at {weight_path}. Train first: "
             f"./bin/python main.py --model {name} --domain {args.domain}"
        )
        return None

    print(f"\n=== Running {name} (loading {weight_path}) ===")
    model, weight_dir = _load_trained(name, args, feat_dim)

    if args.dataset == "ugv":
        thresholds = None
        if name == "Autoencoder":
            thresholds = _ugv_ae_thresholds(model, args, weight_dir)
        ugv_summaries(name, model, args, thresholds, weight_dir)
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
        choices=["all", "autoencoder", "gru", "rnn", "bilstm", "ensemble", "vote"],
        default="all",
    )
    parser.add_argument("--dataset", choices=["alfa", "ugv"], default="ugv",
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
    parser.add_argument("--layout", choices=["channel", "multivariate"], default="channel",
                        help="UGV: 'channel' windows each sensor channel separately "
                             "(ugv_dataset.py); 'multivariate' gives every time step all "
                             "sensors and predicts which channels are faulty "
                             "(ugv_multivariate.py).")
    parser.add_argument("--train-stride", type=int, default=None,
                        help="UGV multivariate: step between training windows "
                             "(default seq_len // 4); validation and test do not overlap.")
    parser.add_argument("--extra-val-dir", action="append", default=[],
                        help="UGV: further fault_injection.py variants of the validation "
                             "drives, added to the validation split (repeatable).")
    parser.add_argument("--extra-test-dir", action="append", default=[],
                        help="UGV: further fault_injection.py variants of the test drives, "
                             "scored as a separate test_extra split (repeatable).")
    parser.add_argument("--tensorboard-dir", default=RUNS_DIR,
                        help="TensorBoard log root; view with tensorboard --logdir runs.")
    parser.add_argument("--no-tensorboard", action="store_true")
    parser.add_argument("--device", default="cpu",
                        help="Training device: cpu, mps (Apple GPU), or cuda.")
    parser.add_argument("--ae-percentile", type=float, default=99.0,
                        help="UGV autoencoder: flag windows whose reconstruction error exceeds "
                             "this percentile of their channel's clean training errors.")
    parser.add_argument("--ae-encoder-dims", type=int, nargs="*", default=None, metavar="N",
                        help="Autoencoder hidden layer widths between the input and the "
                             "bottleneck (default: config encoder_dims; none for no layer).")
    parser.add_argument("--ae-decoder-dims", type=int, nargs="*", default=None, metavar="N",
                        help="Autoencoder hidden layer widths between the bottleneck and the "
                             "output (default: config decoder_dims).")
    parser.add_argument("--specialists", action="store_true",
                        help="UGV multivariate: train one small network per faultable channel "
                             "(config \"specialists\") instead of one network for all channels.")
    parser.add_argument("--vote-tag", action="append", default=[], metavar="MODEL=TAG",
                        help="--model vote: weight-folder tag for one member, e.g. "
                             "autoencoder=w100 (default: --tag).")
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
    parser.add_argument("--patience", type=int, default=None,
                        help="Stop after N epochs without a better validation score "
                             "(default: run every epoch).")
    parser.add_argument("--grad-clip", type=float, default=None,
                        help="Clip each step's gradient to this L2 norm (default: off).")
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
        elif args.model == "vote":
            run_vote(args, feat_dim)
        else:
            run_model(_normalize_model(args.model), args, feat_dim)
        return
    if args.model in ("ensemble", "vote"):
        raise SystemExit(f"--model {args.model} combines trained models; use --mode run")

    models = (
        ["Autoencoder", "GRU", "BiLSTM"]
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
    return {"autoencoder": "Autoencoder", "gru": "GRU", "rnn": "RNN", "bilstm": "BiLSTM"}[
        model_arg
    ]


if __name__ == "__main__":
    main()
