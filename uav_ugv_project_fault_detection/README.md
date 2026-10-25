# Training Datasets
https://kilthub.cmu.edu/articles/dataset/ALFA_A_Dataset_for_UAV_Fault_and_Anomaly_Detection/12707963

# General Approach

Each folder under `datasets/processed/` is one flight scenario from the ALFA
dataset, exported as many per-topic CSVs (IMU, navigation, control outputs,
...). This pipeline isolates a **single sensor channel** — by default measured
yaw (`mavros-nav_info-yaw` → `field.measured`, present in every scenario) — and
turns it into a fixed-length window that the RNN / BiLSTM / Autoencoder can
consume. A scenario is labelled *faulty* (1) when its `*-failure_status-*.csv`
is present and non-zero, and *normal* (0) otherwise; the one
`*no_ground_truth*` flight is skipped. 46 scenarios are usable: **36 faulty,
10 normal**.

Every analysis runs on a **train / validation split** (20% held out, fixed
seed) and checkpoints its weights to the model's folder periodically (and keeps
the best + final model).

## Frequency Domain Analysis

For the frequency domain, each channel window is transformed with an FFT
(`models/utils/FrequencyDomain.py::time_to_frequency_domain`); the magnitude of
the spectrum (padded/truncated to the window length) becomes the feature vector.
Faults such as engine or control-surface failures show up as changes in the
spectral content, so the models learn to flag anomalous spectra.

## Time Series Analysis

For the time domain, the channel is resampled to a fixed number of samples
(per-window z-scored for scale invariance) and fed to the models as the raw
sequence. The RNN / BiLSTM read the `(seq_len, 1)` sequence; the Autoencoder
reconstructs the flattened `(seq_len,)` window and the reconstruction error is
the anomaly score.

The domain is chosen with `--domain {time,frequency}`; the two analyses save to
separate weight folders (`<name>Weights` vs `<name>Weights_freq`) so they never
overwrite each other.

# Models

## Autoencoder
Reconstruction model (MSE loss). Trained to reconstruct normal windows; a high
reconstruction error on a new window is the fault/anomaly signal.

## RNN
Recurrent classifier over the `(seq_len, 1)` window (binary fault/normal,
cross-entropy).

## BiLSTM
Bidirectional LSTM classifier over the same window (binary fault/normal,
cross-entropy).

# Usage

Run from this folder with the project venv (`.bin/python`):

```
# Train every model, time domain (default)
./bin/python main.py

# Train every model, frequency domain
./bin/python main.py --domain frequency

# Train a single model, a chosen channel, custom epochs / checkpoint interval
./bin/python main.py --model rnn --domain frequency --topic mavros-nav_info-roll --column field.measured --epochs 50 --save-every 25

# Load the saved best model and run inference
./bin/python main.py --mode run --model autoencoder --domain frequency

# Plot the training loss curves and save PNGs
./bin/python plot_losses.py --all-domains
```

## Class weighting

The fault/normal classes are heavily imbalanced (36 vs 10), so the RNN/BiLSTM
cross-entropy loss is **class-weighted** by inverse class frequency
(`main.py::_class_weights`). The autoencoder uses plain MSE. The applied weights
are printed at the start of each classifier training run.

## Evaluation metrics

After training, each model prints a **precision / recall / F1 / accuracy** summary
(`metrics.py`, the positive class being *fault* for the classifiers and *anomaly*
for the autoencoder, which is scored by thresholding its reconstruction error
against the normal windows). `--mode run` reports the same summary on the full
dataset using the saved `best_model.pt`.

## Loss plots

The Trainer writes a per-epoch `loss_history.json` (train + validation) into each
weight folder. `plot_losses.py` reads those histories and saves PNGs to
`models/loss_plots/<domain>/`: one per model plus a combined three-model
comparison (training and validation).

```
./bin/python plot_losses.py                     # plot whichever histories exist
./bin/python plot_losses.py --domain frequency  # one domain
./bin/python plot_losses.py --all-domains
```

Weights are written under `models/<name>Weights/` (and
`models/<name>Weights_freq/` for the frequency domain), each containing
`best_model.pt`, `final_model.pt`, `model_epoch_NNNN.pt` checkpoints, and a
`loss_history.json`.

# Simulated UGV sensor faults

See [MODELS.md](MODELS.md) for each UGV model's inputs, outputs,
architecture, training data, evaluation metrics, and results.

`datasets/ugv_clean/` holds clean simulated UGV runs from
[`scripts/collect_ugv_dataset.sh`](../scripts/collect_ugv_dataset.sh) (see
[sensor dataset collection](../docs/sensor_dataset.md#batch-clean-ugv-runs)).
`fault_injection.py` turns each clean run into labelled variants:

```
./bin/python fault_injection.py --input datasets/ugv_clean --output datasets/ugv_faulty
./bin/python test_fault_injection.py
```

Every variant first receives nominal Gaussian noise (the
`sim_ugv_sensors.py` defaults: IMU 0.01 rad/s and 0.1 m/s², wheel speed
0.05 m/s and 0.02 rad/s, GPS 1.5 m horizontal and 3 m vertical). One fault is
then injected into one randomly chosen channel (IMU `wx..az`, wheel `vx`/`wz`,
GPS latitude/longitude/altitude). Magnitudes are multiples of that channel's
nominal sigma:

| Fault | Effect | Default parameters |
| --- | --- | --- |
| `short` | 3–8 spikes of 1–3 samples inside a 5–20 s window; only spike samples are labelled | 6–15 sigma, random sign |
| `noise` | Extra Gaussian noise over a 5–30 s window | 3–8 sigma std |
| `constant` | Fixed offset from onset to the end of the run | 3–10 sigma, random sign |

Defaults write one fault-free (`normal`) and one variant per fault type for
each run (`run_<seed>_<type>_<index>/`); change counts with `--per-type`,
`--normal-variants`, and `--types`. Each sensor CSV gains `fault` (0/1),
`fault_type`, and `fault_column` per sample; `faults.json` stores the exact
injection, its seed, and `source_run`. Split train/validation/test by
`source_run` so variants of one drive never appear in two splits. Wheel
odometry pose columns are left as recorded and drift with simulated wheel
slip; faults are injected into the per-sample channels only.

## Training on the UGV variants

```
./bin/python main.py --dataset ugv --model bilstm --epochs 2500 --batch-size 256 --device mps --save-every 250
./bin/python main.py --dataset ugv --mode run --model bilstm     # val + test summaries
./bin/python main.py --dataset ugv --mode run --model ensemble   # BiLSTM OR autoencoder
```

`ugv_dataset.py` cuts every faultable channel into 100-sample windows (2 s IMU,
5 s wheel, 20 s GPS); a window is faulty when any sample in it carries that
channel's fault label. Source runs are split 20 train / 5 validation / 5 test
(`--val-runs`, `--test-runs`); checkpoints and thresholds use train and
validation only, and summaries report the held-out test runs.

Each window has two features: the channel and a reference for the same
quantity from another sensor or the drive command (IMU yaw rate vs wheel yaw
rate, wheel speed vs commanded speed, IMU ax/ay vs commanded acceleration,
GPS vs wheel-odometry position; zeros for wx, wy, az, and altitude). On clean
training windows the yaw-rate, speed, and GPS references correlate 0.89–0.99
with their channels. `--single-channel` drops the reference. Windows are
standardized with per-channel statistics from clean training windows, because
a per-window z-score would erase constant offsets; the frequency domain uses
orthonormal FFT magnitudes.

- RNN / BiLSTM save `best_model.pt` at the highest validation F1 (faults are
  about 1% of windows, so validation loss tracks confidence, not detection).
- The autoencoder trains on fault-free windows and flags a window when its
  reconstruction error exceeds the `--ae-percentile` (default 99) of clean
  training errors for that channel; thresholds go to `threshold.json`.
- `--model ensemble` (with `--mode run`) flags a window when the BiLSTM or the
  autoencoder flags it.

Weights go to `models/<name>Weights_ugv_paired[_freq]/`
(`<name>Weights_ugv` holds the earlier single-channel run on a 24/6 split).
`--device mps` uses the Apple GPU for the BiLSTM and autoencoder; `nn.RNN`
trains faster on `--device cpu`.

## Live training curves (TensorBoard)

Every training run writes TensorBoard logs to `runs/<dataset>_<model>_<domain>_<time>/`:
per-epoch `loss/train`, `loss/val`, `f1/val` (classifiers), `learning_rate`, `epoch_seconds`, the run
arguments, and the final validation precision / recall / F1 / accuracy.

```
./bin/tensorboard --logdir runs     # then open http://localhost:6006
```

Use `--tensorboard-dir` to log elsewhere or `--no-tensorboard` to skip it.
`runs/` is gitignored; per-epoch histories are also saved as
`loss_history.json` in each weight folder.
