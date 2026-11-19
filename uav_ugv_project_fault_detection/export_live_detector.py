# Export a trained UGV BiLSTM + autoencoder for the live ROS detector.
#
# Writes one .npz with the model weights, the per-channel standardization
# statistics of the clean training windows (recomputed exactly as
# ugv_dataset.build_ugv_splits does), and the autoencoder thresholds. The ROS
# node ``fault_management/scripts/sensor_fault_detector.py`` runs it with numpy
# only. ``--verify`` checks the numpy features and forward passes against
# ugv_dataset and torch on the validation and test drives.
#
#   python3 export_live_detector.py --verify

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import torch

import ugv_dataset as U

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "ros2_ws/src/fault_management/scripts"))
import sensor_fault_model as M  # noqa: E402

DEFAULT_OUTPUT = ROOT / "ros2_ws/src/fault_management/models/ugv_sensor_fault_detector.npz"


def channel_statistics(splits, seq_len, extra_train_dirs):
    train = list(splits["train"])
    for folder in extra_train_dirs:
        train += U.find_variants(folder)
    windows, labels, info = U.channel_windows(train, seq_len, seq_len, paired=True)
    return U.channel_stats(windows, labels, info)


def live_streams(variant):
    """CSV columns arranged as the live node buffers them (map-frame odometry)."""
    frames = {name: pd.read_csv(variant / f"{name}.csv")
              for name in ("imu", "wheel_odom", "gps", "cmd")}
    streams = {name: {c: frame[c].to_numpy(np.float64) for c in frame.columns
                      if frame[c].dtype.kind in "fi"} for name, frame in frames.items()}
    streams["wheel_odom"]["x"] = streams["wheel_odom"]["x"] + U.SPAWN_EAST
    return frames, streams


def verify(model, splits, bilstm, autoencoder, seq_len):
    origin = (U.ORIGIN_LATITUDE, U.ORIGIN_LONGITUDE)
    worst_feature = 0.0
    torch_flags, numpy_flags, torch_errors, numpy_errors = [], [], [], []
    for variant in splits["val"] + splits["test"]:
        frames, streams = live_streams(variant)
        expected = U.references({k: frames[k] for k in ("imu", "wheel_odom", "gps")},
                                frames["cmd"])
        windows = []
        for channel in M.CHANNELS:
            stamps = streams[channel[0]]["stamp"]
            live = M.reference(channel, stamps, streams, origin)
            if channel in expected:
                scale = model.std[model.index[channel]]
                worst_feature = max(worst_feature,
                                    float(np.max(np.abs(live - expected[channel])) / scale))
            else:
                assert live is None, channel
            values = np.stack([streams[channel[0]][channel[1]],
                               np.full(len(stamps), np.nan) if live is None else live], axis=1)
            for start in range(0, len(stamps) - seq_len + 1, seq_len):
                windows.append(model.standardize(channel, values[start:start + seq_len]))
        x = np.stack(windows)
        with torch.no_grad():
            t = torch.as_tensor(x, dtype=torch.float32)
            torch_flags.append(torch.argmax(bilstm(t), dim=1).numpy())
            flat = t.flatten(start_dim=1)
            torch_errors.append(((autoencoder(flat) - flat) ** 2).mean(dim=1).numpy())
        numpy_flags.append(model.classify(x) > 0.5)
        numpy_errors.append(model.reconstruction_error(x))
    torch_flags, numpy_flags = np.concatenate(torch_flags), np.concatenate(numpy_flags)
    torch_errors, numpy_errors = np.concatenate(torch_errors), np.concatenate(numpy_errors)
    agreement = float((torch_flags == numpy_flags).mean())
    relative = float(np.max(np.abs(torch_errors - numpy_errors) / np.maximum(torch_errors, 1e-9)))
    print(f"Verified {len(torch_flags)} val+test windows: reference max |diff| "
          f"{worst_feature:.2e} std, BiLSTM flag agreement {agreement:.5f} "
          f"({int((torch_flags != numpy_flags).sum())} differ), "
          f"autoencoder max relative error {relative:.2e}")
    if worst_feature > 1e-6 or agreement < 0.9999 or relative > 1e-4:
        raise SystemExit("numpy export does not match the training pipeline")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ugv-dir", default=HERE / "datasets/ugv_faulty", type=Path)
    parser.add_argument("--extra-train-dir", action="append", default=[], type=Path)
    parser.add_argument("--bilstm-dir", default=HERE / "models/bilstmWeights_ugv_paired", type=Path)
    parser.add_argument("--autoencoder-dir", default=HERE / "models/autoencoderWeights_ugv_paired",
                        type=Path)
    parser.add_argument("--val-runs", type=int, default=5)
    parser.add_argument("--test-runs", type=int, default=5)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", default=DEFAULT_OUTPUT, type=Path)
    parser.add_argument("--verify", action="store_true")
    args = parser.parse_args()

    from models.Autoencoder import LEGACY, Autoencoder
    from models.BiLstm import BiLSTM

    config = json.loads((args.bilstm_dir / "model_config.json").read_text())
    if config.get("readout") != "full":
        raise SystemExit("the live detector implements the 'full' BiLSTM readout only")
    seq_len = int(config.get("window_len", 100))
    bilstm_state = torch.load(args.bilstm_dir / "best_model.pt", map_location="cpu")
    ae_state = torch.load(args.autoencoder_dir / "best_model.pt", map_location="cpu")
    hidden = bilstm_state["lstm.weight_hh_l0"].shape[1]
    layers = sum(1 for k in bilstm_state if k.startswith("lstm.weight_ih_l") and "reverse" not in k)
    if bilstm_state["lstm.weight_ih_l0"].shape[1] != 2:
        raise SystemExit("the live detector needs paired (2-feature) time-domain weights")
    bilstm = BiLSTM(input_dim=2, hidden_dim=hidden, output_dim=2, num_layers=layers, readout="full")
    bilstm.load_state_dict(bilstm_state)
    # sensor_fault_model.py implements the single-layer ReLU autoencoder; build that
    # layout explicitly, since the config default now adds a hidden encoder layer.
    if "encoder.weight" not in ae_state:
        raise SystemExit("the live detector implements the single-layer ReLU autoencoder only")
    autoencoder = Autoencoder(input_dim=ae_state["encoder.weight"].shape[1],
                              hidden_dim=ae_state["encoder.weight"].shape[0], **LEGACY)
    autoencoder.load_state_dict(ae_state)
    bilstm.eval()
    autoencoder.eval()

    splits = U.split_runs(U.find_variants(args.ugv_dir), args.val_runs, args.test_runs, args.seed)
    stats = channel_statistics(splits, seq_len, args.extra_train_dir)
    thresholds = json.loads((args.autoencoder_dir / "threshold.json").read_text())
    names = [M.channel_name(c) for c in M.CHANNELS]
    commit = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"],
                            capture_output=True, text=True).stdout.strip()
    metadata = {
        "bilstm": str(args.bilstm_dir.relative_to(HERE)),
        "autoencoder": str(args.autoencoder_dir.relative_to(HERE)),
        "ae_percentile": thresholds["percentile"],
        "stats_from": [str(Path(args.ugv_dir).name)] + [Path(d).name for d in args.extra_train_dir],
        "split": {"val_runs": args.val_runs, "test_runs": args.test_runs, "seed": args.seed},
        "origin": [U.ORIGIN_LATITUDE, U.ORIGIN_LONGITUDE],
        "git_commit": commit,
    }
    arrays = {
        "channels": np.array(names),
        "mean": np.array([stats[c][0] for c in M.CHANNELS]),
        "std": np.array([stats[c][1] for c in M.CHANNELS]),
        "ae_threshold": np.array([thresholds["per_channel"][n] for n in names]),
        "window_len": np.array(seq_len),
        "num_layers": np.array(layers),
        "metadata": np.array(json.dumps(metadata)),
    }
    arrays.update({f"bilstm.{k}": v.numpy() for k, v in bilstm_state.items()})
    arrays.update({f"ae.{k}": v.numpy() for k, v in ae_state.items()})
    # Torch outputs on a few fixed standardized windows, so ROS tests can check
    # the numpy forward passes without torch.
    check = np.random.default_rng(0).normal(size=(4, seq_len, 2))
    check[1, :, 0] += 3.0   # an offset between channel and reference
    with torch.no_grad():
        t = torch.as_tensor(check, dtype=torch.float32)
        arrays["check.windows"] = check.astype(np.float32)
        arrays["check.probability"] = torch.softmax(bilstm(t), dim=1)[:, 1].numpy()
        flat = t.flatten(start_dim=1)
        arrays["check.ae_error"] = ((autoencoder(flat) - flat) ** 2).mean(dim=1).numpy()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output, **arrays)
    print(f"Wrote {args.output} ({args.output.stat().st_size / 1024:.0f} KiB)")
    for name, mean, std in zip(names, arrays["mean"], arrays["std"]):
        print(f"   {name:16s} mean {mean: .9g}  std {std:.6g}")
    if args.verify:
        verify(M.SensorFaultModel.load(args.output), splits, bilstm, autoencoder, seq_len)


if __name__ == "__main__":
    main()
