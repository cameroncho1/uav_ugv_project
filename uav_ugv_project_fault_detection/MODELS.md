# UGV Sensor Fault Detection Models

This document describes the three fault-detection models trained on simulated
UGV sensor data: what data they see, how windows are built, what each network
computes, how it is trained, and how it is scored. It covers the `--dataset
ugv` path of `main.py`. The ALFA flight-data path is described in
[README.md](README.md).

All numbers come from the committed runs in `models/*Weights_ugv_paired*/` and
`models/train_logs/ugv_paired_*.log` (commit `0f81b62`, October 25, 2026). They
describe one simulated dataset, one seeded split, and the injected fault
ranges below; they are not measurements of real sensors.

## Contents

1. [Pipeline overview](#1-pipeline-overview)
2. [Training data](#2-training-data)
3. [Model inputs](#3-model-inputs)
4. [Models](#4-models)
5. [Training](#5-training)
6. [Evaluation metrics](#6-evaluation-metrics)
7. [Results](#7-results)
8. [Reproducing](#8-reproducing)
9. [Limitations and known issues](#9-limitations-and-known-issues)

## 1. Pipeline overview

```text
Gazebo ugv_dataset world ──► scripts/collect_ugv_dataset.sh ──► datasets/ugv_clean/run_<seed>/
   (raw IMU, wheel odometry, GPS, truth, commands; no added noise)
                                   │
                                   ▼
                    fault_injection.py ──► datasets/ugv_faulty/run_<seed>_<type>_<k>/
   (nominal Gaussian noise + one labelled fault; per-sample labels, faults.json)
                                   │
                                   ▼
                    ugv_dataset.py ──► windows (N, 100, F), labels (N,), info
   (per channel, 100-sample windows, cross-sensor reference, per-channel scaling,
    20 / 5 / 5 split by source drive)
                                   │
                                   ▼
                    main.py --dataset ugv ──► Autoencoder | RNN | BiLSTM | ensemble
```

Each **window** is 100 consecutive samples of one sensor channel. Every model
produces one decision per window: *faulty* or *clean*.

## 2. Training data

### 2.1 Clean simulated drives

| Item | Value |
| --- | --- |
| Simulator | Gazebo Harmonic 8.15, ROS 2 Humble, UTM Ubuntu 22.04 ARM64 VM |
| World | [`simulation/worlds/ugv_dataset.sdf`](../simulation/worlds/ugv_dataset.sdf): `simple_goal_demo` UGV with two frictionless casters (keeps the two-wheel chassis level) |
| Recorder | [`ugv_dataset_recorder.py`](../ros2_ws/src/fault_management/scripts/ugv_dataset_recorder.py) via [`collect_ugv_dataset.sh`](../scripts/collect_ugv_dataset.sh) |
| Drives | 30 (seeds 1000–1029), 120 s each, 60 min total |
| Driving | Seeded open-loop segments: 3 s stationary start, then stop, straight, arc, spin, weave, and speed-ramp segments (657 segments in total). Limits 0.8 m/s and 1.2 rad/s; acceleration limits 0.8 m/s² and 2 rad/s². No truth is used to steer. |
| Output | `datasets/ugv_clean/` (gitignored; regenerate with the same seeds) |

Raw sensor streams per drive:

| File | Rate | Channels used by the models | Frame / units |
| --- | ---: | --- | --- |
| `imu.csv` | 50 Hz | `wx, wy, wz` (rad/s), `ax, ay, az` (m/s², includes gravity) | FLU chassis |
| `wheel_odom.csv` | ~19 Hz | `vx` (m/s), `wz` (rad/s) | FLU chassis twist |
| `gps.csv` | 5 Hz | `latitude, longitude` (deg), `altitude` (m) | WGS84 |
| `cmd.csv` | 20 Hz | `linear`, `angular` (reference features only) | commanded twist |
| `truth.csv` | 50 Hz | not used by the models | ENU `map` |

Wheel-odometry pose (`x, y, q*`) and IMU orientation are not fault targets.
Wheel-odometry pose is used only to build the GPS reference (Section 3.2).

The raw readings carry no added noise, but they are not ideal: wheel odometry
drifts with simulated wheel slip (mean end-of-drive position error 3.8 m,
maximum 17.5 m), and IMU `az` shows about 0.23 m/s² of wheel-contact vibration
while driving.

### 2.2 Fault injection

[`fault_injection.py`](fault_injection.py) writes four variants per clean drive
(120 variants total): one fault-free `normal` variant and one variant per fault
type. Each variant:

1. **Adds nominal Gaussian noise** to every faultable channel. The sigmas are
   the defaults of `sim_ugv_sensors.py`:

   | Channel | Nominal sigma |
   | --- | --- |
   | IMU `wx, wy, wz` | 0.01 rad/s |
   | IMU `ax, ay, az` | 0.1 m/s² |
   | Wheel `vx` | 0.05 m/s |
   | Wheel `wz` | 0.02 rad/s |
   | GPS latitude / longitude | 1.5 m (converted to degrees) |
   | GPS altitude | 3.0 m |

2. **Injects one fault** into one randomly chosen channel (sensor chosen
   uniformly, then a channel within it). Magnitudes are multiples of that
   channel's nominal sigma:

   | Fault | Effect | Parameters | Labelled samples |
   | --- | --- | --- | --- |
   | `short` | 3–8 spikes, each 1–3 samples, inside a 5–20 s window | 6–15 sigma, random sign per spike | spike samples only (0.9% of that sensor's samples on average) |
   | `noise` | Extra zero-mean Gaussian noise over a 5–30 s window | std 3–8 sigma | the whole window (14% on average) |
   | `constant` | Fixed offset from a random onset to the end of the drive | 3–10 sigma, random sign | onset to end (48% on average) |

3. **Labels every sample**: each sensor CSV gains `fault` (0/1), `fault_type`
   (`none`, `short`, `noise`, `constant`), and `fault_column`. `faults.json`
   records the exact fault, its seed, and the `source_run`.

## 3. Model inputs

[`ugv_dataset.py`](ugv_dataset.py) turns variants into model-ready windows.

### 3.1 Windowing and labels

- **Per channel:** each of the 11 faultable channels (6 IMU, 2 wheel, 3 GPS) is
  windowed separately. One model handles all channels; it is not told which
  channel a window comes from.
- **Length and stride:** 100 samples, non-overlapping. That is 2 s of IMU,
  ~5.2 s of wheel odometry, or 20 s of GPS.
- **Label:** 1 when any sample in the window carries a fault label for *that*
  channel, otherwise 0. Windows from a faulty variant's other channels, and
  from before a fault starts, are labelled clean.
- **Per-window info** (not a model input): variant, sensor, channel, and fault
  type, used for per-type recall and for per-channel autoencoder thresholds.

### 3.2 Features per time step

With the default paired features (`F = 2`), every time step has:

| Feature | Content |
| --- | --- |
| 0 | The channel's measured value |
| 1 | A reference for the same quantity from another sensor or the drive command |

| Channel | Reference (feature 1) | Correlation with channel on clean training windows |
| --- | --- | ---: |
| IMU `wz` | wheel-odometry `wz` | 0.994 |
| Wheel `wz` | IMU `wz` | 0.994 |
| Wheel `vx` | commanded linear speed | 0.989 |
| GPS longitude | wheel-odometry position → longitude | 0.952 |
| GPS latitude | wheel-odometry position → latitude | 0.891 |
| IMU `ay` | commanded speed × commanded yaw rate (centripetal) | 0.647 |
| IMU `ax` | time derivative of commanded speed | 0.301 |
| IMU `wx`, `wy`, `az`, GPS altitude | none: zeros | — |

References are interpolated onto the channel's sample stamps. They never use
simulator truth. A reference sensor can itself be faulty in a variant (only the
target channel's label counts), so the model sees both features rather than
only their difference.

`--single-channel` gives `F = 1` (feature 0 only). The older committed weights
in `models/<name>Weights_ugv/` use that layout and an earlier 24 / 6 split.

### 3.3 Scaling

Both features are standardized with the **channel's** mean and standard
deviation, computed over clean training windows only:
`(value − mean_channel) / std_channel`. Missing references become 0.

Per-window z-scoring (as in the ALFA path) is deliberately avoided: it would
remove a constant offset from every window after onset.

### 3.4 Domains

| Domain | Feature per time step | Notes |
| --- | --- | --- |
| `time` (default) | standardized samples, shape `(100, F)` | |
| `frequency` | FFT magnitude of each standardized feature over the window, divided by √100, shape `(100, F)` | Bin 0 is the window mean × 10, so offsets appear there. Bins 51–99 mirror bins 1–49 for real input. |

### 3.5 Splits

Splits are by source drive, so the four variants of one drive stay together
(`--seed 42`):

| Split | Drives | Windows | Faulty windows (constant / noise / short) |
| --- | --- | ---: | --- |
| Train | 1000–1004, 1006, 1008, 1009, 1011–1015, 1017–1020, 1023, 1027, 1028 (20) | 33,920 | 373 (219 / 97 / 57) |
| Validation | 1007, 1016, 1024, 1025, 1029 (5) | 8,480 | 176 (143 / 23 / 10) |
| Test | 1005, 1010, 1021, 1022, 1026 (5) | 8,480 | 91 (52 / 29 / 10) |

Windows per split come from IMU (85%), wheel odometry (11%), and GPS (4%),
because IMU has six channels at the highest rate. About 1.1–2.1% of windows are
faulty.

## 4. Models

All three models are defined in [`models/`](models) and use the
hyperparameters in [`models/utils/ModelParameters.json`](models/utils/ModelParameters.json)
(hidden size 50; 2 recurrent layers).

### 4.1 Autoencoder ([`models/Autoencoder.py`](models/Autoencoder.py))

| | |
| --- | --- |
| Input | Flattened window, `100 × F` values (200 for paired, 100 for single-channel) |
| Architecture | `Linear(100F → 50)` → ReLU → `Linear(50 → 100F)`; linear output |
| Output | Reconstruction of the input window |
| Parameters | 20,250 (paired), 10,150 (single-channel) |
| Fault score | Mean squared reconstruction error of the window |
| Decision | Faulty when the error exceeds the channel's threshold (Section 6.2) |

The autoencoder trains on clean windows only, so it learns normal behaviour; a
window it cannot reconstruct is anomalous. It treats the window as an unordered
vector of 100F numbers and does not model time order. The decoder output is
linear because standardized inputs are negative about half the time; an earlier
sigmoid output could not reconstruct them.

### 4.2 RNN ([`models/Rnn.py`](models/Rnn.py))

| | |
| --- | --- |
| Input | Window sequence, shape `(batch, 100, F)` |
| Architecture | `nn.RNN(F, 50, num_layers=2, tanh)`, zero initial state → hidden state at the last time step → `Linear(50 → 2)` |
| Output | Two logits (clean, faulty); prediction = argmax |
| Parameters | 7,902 (paired), 7,852 (single-channel) |

### 4.3 BiLSTM ([`models/BiLstm.py`](models/BiLstm.py))

| | |
| --- | --- |
| Input | Window sequence, shape `(batch, 100, F)` |
| Architecture | `nn.LSTM(F, 50, num_layers=2, bidirectional=True)`, zero initial states → concatenated forward/backward outputs at the last time step (100 values) → `Linear(100 → 2)` |
| Output | Two logits (clean, faulty); prediction = argmax |
| Parameters | 82,602 (paired), 82,202 (single-channel) |

The classifier reads `out[:, -1, :]`. At the last time step the forward half
has read the whole window, but the backward half has read only that final
sample (see Section 9).

### 4.4 Ensemble (`--mode run --model ensemble`)

The ensemble loads the trained BiLSTM and autoencoder for the same domain and
feature layout. It flags a window as faulty when **either** model flags it. It
has no trainable parameters.

## 5. Training

| Setting | Autoencoder | RNN | BiLSTM |
| --- | --- | --- | --- |
| Training windows | clean windows only (33,547) | all training windows (33,920) | all training windows (33,920) |
| Loss | MSE (reconstruction) | class-weighted cross-entropy | class-weighted cross-entropy |
| Class weights | — | clean 0.022, faulty 1.978 | clean 0.022, faulty 1.978 |
| Optimizer | Adam, lr 0.001 | Adam, lr 0.001 | Adam, lr 0.001 |
| Batch size | 256 | 256 | 256 |
| Epochs | 2500 | 2500 | 2500 |
| Device used | Apple GPU (`mps`) | CPU | Apple GPU (`mps`) |
| `best_model.pt` chosen by | lowest validation MSE on clean validation windows | highest validation F1 | highest validation F1 |
| Time per epoch (this Mac) | ~0.5 s | ~4 s | ~3 s |

Class weights are inverse class frequency, normalized to a mean of 1
(`main.py::_class_weights`).

Selecting classifiers by validation F1 rather than validation loss matters
here: in the earlier single-channel run, the lowest-loss BiLSTM checkpoint
(epoch 9) scored F1 0.08, while the last checkpoint scored 0.40. With about 1%
faulty windows, the validation loss mostly tracks confidence on misclassified
windows.

Every run saves `best_model.pt`, `final_model.pt`, `model_epoch_NNNN.pt` every
`--save-every` epochs, and `loss_history.json`. The autoencoder also saves
`threshold.json`. TensorBoard logs go to `runs/` (`./bin/tensorboard --logdir runs`).

## 6. Evaluation metrics

### 6.1 Window-level metrics

All metrics are per window, with *faulty* as the positive class
([`metrics.py`](metrics.py)):

| Metric | Definition |
| --- | --- |
| Precision | TP / (TP + FP): fraction of flagged windows that are faulty |
| Recall | TP / (TP + FN): fraction of faulty windows that are flagged |
| F1 | 2 · precision · recall / (precision + recall): the main selection and comparison metric |
| Accuracy | (TP + TN) / all; reported but uninformative at ~1% faults (flagging nothing scores ~98.9%) |
| Recall per fault type | recall over windows of one fault type (`short`, `noise`, `constant`) |
| Clean false-positive rate | fraction of clean windows that are flagged |

`--mode train` and `--mode run` print these for the validation and test splits.
Test drives are used only for these summaries, never for checkpoint or
threshold selection.

### 6.2 Autoencoder thresholds

For each of the 11 channels, the threshold is the `--ae-percentile` (default
99th) percentile of reconstruction error over that channel's **clean training**
windows. By construction, about 1% of clean windows per channel are flagged on
the training drives. One shared threshold performs poorly because error floors
differ widely between channels. The paired time-domain thresholds range from
0.031 (wheel `vx`) to 1.42 (IMU `az`). Thresholds are stored in
`threshold.json` and recomputed by `--mode run`.

## 7. Results

Held-out test drives (5 drives, 8,480 windows, 91 faulty), paired features,
`best_model.pt`:

| Model | Domain | Best epoch (val F1) | Test F1 | Precision | Recall | Clean FPR | Short | Noise | Constant |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| BiLSTM | time | 1964 (0.752) | **0.714** | 0.779 | 0.659 | 0.20% | 0.20 | 0.28 | 0.96 |
| BiLSTM | frequency | 1264 (0.718) | 0.558 | 0.535 | 0.582 | 0.55% | 0.30 | 0.24 | 0.83 |
| RNN | frequency | 79 (0.552) | 0.373 | 0.275 | 0.582 | 1.67% | 0.40 | 0.24 | 0.81 |
| RNN | time | 630 (0.534) | 0.356 | 0.263 | 0.549 | 1.67% | 0.00 | 0.17 | 0.87 |
| Autoencoder | time | 2088 (loss) | 0.345 | 0.237 | 0.637 | 2.23% | 0.20 | 0.31 | 0.90 |
| Autoencoder | frequency | 1638 (loss) | 0.292 | 0.222 | 0.429 | 1.63% | 0.50 | 0.31 | 0.48 |
| Ensemble | time | — | 0.352 | 0.236 | 0.692 | 2.43% | 0.30 | 0.31 | 0.98 |
| Ensemble | frequency | — | 0.355 | 0.246 | 0.637 | 2.12% | 0.50 | 0.31 | 0.85 |

Observations on these runs:

- The time-domain BiLSTM gives the best F1 and the lowest false-positive rate.
- Constant offsets are the most detectable fault type once the reference
  feature is present. Without it, the earlier single-channel autoencoder
  recalled 6% of constant-fault windows on its validation split; the paired one
  recalls 90% on test.
- Short spikes and noise bursts remain hard (0–50% recall across models). The test split
  contains only 10 short-fault and 29 noise-fault windows, so these recalls
  change by 0.03–0.10 per window.
- The OR ensemble adds recall but inherits the autoencoder's ~2% false
  positives, which lowers precision and F1 below the BiLSTM alone at the
  default 99th-percentile threshold.

Logs: `models/train_logs/ugv_paired_<domain>_<model>_2500.log` and
`ugv_paired_<domain>_ensemble.log`.

## 8. Reproducing

From this folder, with the project venv:

```bash
# 1. Clean drives (Ubuntu VM; see ../docs/sensor_dataset.md)
SEEDS="$(seq 1000 1029)" DURATION=120 ../scripts/collect_ugv_dataset.sh datasets/ugv_clean

# 2. Faulty variants and their tests
./bin/python fault_injection.py --input datasets/ugv_clean --output datasets/ugv_faulty
./bin/python test_fault_injection.py

# 3. Train (add --domain frequency for the frequency domain; RNN on --device cpu)
./bin/python main.py --dataset ugv --model bilstm --epochs 2500 --batch-size 256 \
  --device mps --save-every 250

# 4. Evaluate saved models on validation and test
./bin/python main.py --dataset ugv --mode run --model bilstm
./bin/python main.py --dataset ugv --mode run --model ensemble

# 5. Live curves
./bin/tensorboard --logdir runs
```

Useful options: `--single-channel`, `--val-runs`, `--test-runs`,
`--ae-percentile`, `--seq-len`, `--seed`, `--no-tensorboard`.

## 9. Limitations and known issues

- **Simulation only.** Noise and faults are injected assumptions with the
  ranges in Section 2.2. Results describe these conditions, not real sensors.
- **Small evaluation sets.** 5 test drives give 91 faulty windows; per-type
  recalls for `short` and `noise` rest on 10 and 29 windows.
- **One fault per variant.** Simultaneous faults are not represented.
- **Window-level decisions.** A window with one spike sample counts as faulty;
  models do not locate the faulty samples within a window.
- **No channel identity.** One model serves all 11 channels without being told
  which channel a window belongs to.
- **BiLSTM readout.** The classifier uses the last time step only, where the
  backward LSTM has seen one sample. Concatenating the forward state at the
  last step with the backward state at the first step would use both full
  passes.
- **Redundant frequency bins.** The magnitude spectrum of a real window is
  symmetric, so half of the frequency bins repeat the other half.
- **Overfitting.** Training loss falls throughout, while validation loss rises
  after early epochs for the BiLSTM. F1-based checkpointing keeps the best
  validation epoch; dropout or early stopping would shorten training.
- **RNN instability.** RNN validation F1 varies widely between epochs (for
  example, the frequency RNN's best epoch is 79 of 2500).
- **Reference quality.** The IMU `ax` reference correlates only 0.30 with
  measured `ax`, because commanded acceleration leads the vehicle's response.
