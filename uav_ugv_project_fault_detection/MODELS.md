# UGV Sensor Fault Detection Models

This document describes the fault-detection models trained on simulated
UGV sensor data: what data they see, how windows are built, what each network
computes, how it is trained, and how it is scored. It covers the `--dataset
ugv` path of `main.py`. The ALFA flight-data path is described in
[README.md](README.md).

Sections 3–7 describe the channel layout. All numbers in them come from the
committed runs in `models/*Weights_ugv_paired*/` and
`models/train_logs/ugv_paired_*.log` (October 25, 2026; the BiLSTMs and the
frequency-domain models were retrained after commit `177bae8`). Section 10
describes the multivariate layout and its November 18, 2026 runs. They
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
   - [7.1 Long, noise-heavy training data](#71-long-noise-heavy-training-data)
8. [Reproducing](#8-reproducing)
9. [Limitations and known issues](#9-limitations-and-known-issues)
10. [Multivariate layout](#10-multivariate-layout)

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
| `frequency` | One-sided FFT magnitude of each standardized feature over the window (bins 0–50), divided by √100, shape `(51, F)` | Bin 0 is the window mean × 10, so offsets appear there. The other 49 bins of a 100-point FFT mirror bins 1–49 for real input and are dropped. |

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
| Input | Flattened window, `L × F` values, where L = 100 time steps or 51 frequency bins (paired: 200 time, 102 frequency) |
| Architecture | `Linear(100F → 50)` → ReLU → `Linear(50 → 100F)`; linear output |
| Output | Reconstruction of the input window |
| Parameters | 20,250 (paired time), 10,352 (paired frequency), 10,150 (single-channel time) |
| Fault score | Mean squared reconstruction error of the window |
| Decision | Faulty when the error exceeds the channel's threshold (Section 6.2) |

The autoencoder trains on clean windows only, so it learns normal behaviour; a
window it cannot reconstruct is anomalous. It treats the window as an unordered
vector of L·F numbers and does not model time order. The decoder output is
linear because standardized inputs are negative about half the time; an earlier
sigmoid output could not reconstruct them.

### 4.2 RNN ([`models/Rnn.py`](models/Rnn.py))

| | |
| --- | --- |
| Input | Window sequence, shape `(batch, L, F)` (L = 100 time steps or 51 frequency bins) |
| Architecture | `nn.RNN(F, 50, num_layers=2, tanh)`, zero initial state → hidden state at the last time step → `Linear(50 → 2)` |
| Output | Two logits (clean, faulty); prediction = argmax |
| Parameters | 7,902 (paired), 7,852 (single-channel) |

### 4.3 BiLSTM ([`models/BiLstm.py`](models/BiLstm.py))

| | |
| --- | --- |
| Input | Window sequence, shape `(batch, L, F)` (L = 100 time steps or 51 frequency bins) |
| Architecture | `nn.LSTM(F, 50, num_layers=2, bidirectional=True)`, zero initial states → top-layer forward output at the last step concatenated with the backward output at the first step (100 values) → `Linear(100 → 2)` |
| Output | Two logits (clean, faulty); prediction = argmax |
| Parameters | 82,602 (paired), 82,202 (single-channel) |

Both halves of the readout have read the whole window: the forward LSTM ends
at the last step and the backward LSTM ends at the first. Training writes
`model_config.json` (`"readout": "full"`) beside the weights. Folders without
it, such as `models/<name>Weights_ugv/`, were trained with the earlier `last`
readout (both halves at the last step, where the backward half has read one
sample), and `main.py` loads them with that readout.

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
| BiLSTM | time | 2210 (0.799) | **0.716** | 0.817 | 0.637 | 0.15% | 0.10 | 0.24 | 0.96 |
| BiLSTM | frequency | 2165 (0.713) | 0.592 | 0.602 | 0.582 | 0.42% | 0.30 | 0.17 | 0.87 |
| RNN | frequency | 53 (0.636) | 0.423 | 0.812 | 0.286 | 0.07% | 0.10 | 0.10 | 0.42 |
| RNN | time | 630 (0.534) | 0.356 | 0.263 | 0.549 | 1.67% | 0.00 | 0.17 | 0.87 |
| Autoencoder | time | 2088 (loss) | 0.345 | 0.237 | 0.637 | 2.23% | 0.20 | 0.31 | 0.90 |
| Autoencoder | frequency | 1719 (loss) | 0.270 | 0.202 | 0.407 | 1.74% | 0.40 | 0.31 | 0.46 |
| Ensemble | time | — | 0.356 | 0.240 | 0.692 | 2.38% | 0.30 | 0.31 | 0.98 |
| Ensemble | frequency | — | 0.362 | 0.251 | 0.648 | 2.10% | 0.50 | 0.31 | 0.87 |

The BiLSTMs use the full readout. The frequency-domain models use the 51
one-sided bins. The time-domain RNN and autoencoder are unchanged by those
fixes and were not retrained.

Change from the previous version (last-step BiLSTM readout, 100 mirrored
frequency bins; commit `0f81b62`), test F1:

| Model | Previous | Current |
| --- | ---: | ---: |
| BiLSTM, time | 0.714 | 0.716 |
| BiLSTM, frequency | 0.558 | 0.592 |
| RNN, frequency | 0.373 | 0.423 |
| Autoencoder, frequency | 0.292 | 0.270 |

The time-domain BiLSTM's validation F1 rose from 0.752 to 0.799, but its test
F1 is unchanged within one or two windows. On this test split, 0.01 of recall
is one window.

Observations on these runs:

- The time-domain BiLSTM gives the best F1 (0.716) with a 0.15% clean
  false-positive rate. The frequency RNN flags fewer clean windows (0.07%) but
  recalls only 29% of faulty ones.
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

### 7.1 Long, noise-heavy training data

This experiment adds 30 training drives without changing validation or test.

| Item | Value |
| --- | --- |
| Drives | seeds 1030–1059, 300 s each (150 min), `datasets/ugv_clean_long/`; recorded at `UPDATE_RATE=1000` (about 3× real time). Sensor and command rates match the original drives; truth is about 15 Hz. |
| Variants per drive | 1 normal, 1 `short`, 1 `constant`, and 3 `noise` faults lasting 30–120 s (mean 74 s): `fault_injection.py --count noise=3 --noise-window 30 120 --seed 1` → `datasets/ugv_faulty_long/` |
| Training split | 224,540 windows, 3,793 faulty (1.69%): 2,095 noise, 1,565 constant, 133 short. The original training split has 97 noise-fault windows. |
| Training | `--extra-train-dir datasets/ugv_faulty_long --tag long`, 400 epochs (about the same number of optimizer steps as 2500 epochs on the original split), other settings as Section 5 |
| Weights / logs | `models/<name>Weights_ugv_paired_long[_freq]/`, `models/train_logs/ugv_paired_long_<domain>_<model>_400.log` |

Held-out test results (same 5 drives as Section 7):

| Model | Domain | Best epoch (val F1) | Test F1 | Precision | Recall | Clean FPR | Short | Noise | Constant | Test F1 in Section 7 |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| BiLSTM | frequency | 325 (0.766) | **0.621** | 0.651 | 0.593 | 0.35% | 0.40 | 0.28 | 0.81 | 0.592 |
| BiLSTM | time | 182 (0.814) | 0.593 | 0.525 | 0.681 | 0.67% | 0.60 | 0.38 | 0.87 | 0.716 |
| RNN | frequency | 55 (0.629) | 0.430 | 0.374 | 0.505 | 0.92% | 0.10 | 0.24 | 0.73 | 0.423 |
| Autoencoder | time | 355 (loss) | 0.318 | 0.400 | 0.264 | 0.43% | 0.10 | 0.28 | 0.29 | 0.345 |
| RNN | time | 8 (0.327) | 0.250 | 0.154 | 0.659 | 3.92% | 0.20 | 0.28 | 0.96 | 0.356 |
| Autoencoder | frequency | 29 (loss) | 0.132 | 0.132 | 0.132 | 0.94% | 0.30 | 0.31 | 0.00 | 0.270 |
| Ensemble | time | — | 0.508 | 0.405 | 0.681 | 1.08% | 0.60 | 0.38 | 0.87 | 0.356 |
| Ensemble | frequency | — | 0.445 | 0.345 | 0.626 | 1.29% | 0.50 | 0.34 | 0.81 | 0.362 |

Observations:

- For the time-domain BiLSTM, the noise-heavy data raised recall for short
  faults (0.10 → 0.60) and noise faults (0.24 → 0.38). It also raised clean
  false positives (0.15% → 0.67%), so test F1 fell from 0.716 to 0.593 while
  validation F1 rose from 0.799 to 0.814. Most new false positives are GPS
  windows: 2.9% of clean GPS test windows, compared with 0.3% before.
- The frequency BiLSTM improved slightly (0.592 → 0.621) and is the best model
  of this experiment.
- The time-domain autoencoder flags fewer clean windows (2.2% → 0.4%) but
  recalls far fewer constant faults (0.90 → 0.29). The ensemble therefore
  improves (0.356 → 0.508) because its false positives fall.
- The time RNN's best validation epoch is 8, so its result reflects an
  unstable run rather than the data change.
- **The test noise recall rests on about four fault events.** 19 of the 29
  noise-fault test windows come from one fault on IMU `wz`, where nominal noise
  (0.01 rad/s) is 2% of the channel's spread while driving. Both BiLSTMs miss
  most of it (0/19 and 2/19). The other noise windows (IMU `wy`, GPS altitude,
  GPS longitude) are mostly detected.
- The training split is now 55% noise faults, while validation and test have
  13% and 32%. That shift may contribute to the higher false-positive rate.

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

Useful options: `--extra-train-dir`, `--tag`, `--single-channel`, `--val-runs`, `--test-runs`,
`--ae-percentile`, `--seq-len`, `--seed`, `--no-tensorboard`.

## 9. Limitations and known issues

- **Simulation only.** Noise and faults are injected assumptions with the
  ranges in Section 2.2. Results describe these conditions, not real sensors.
- **Small evaluation sets.** 5 test drives give 91 faulty windows; per-type
  recalls for `short` and `noise` rest on 10 and 29 windows, and the noise
  windows come from about four fault events.
- **One fault per variant.** Simultaneous faults are not represented.
- **Window-level decisions.** A window with one spike sample counts as faulty;
  models do not locate the faulty samples within a window.
- **No channel identity.** One model serves all 11 channels without being told
  which channel a window belongs to. The multivariate layout (Section 10)
  removes this limitation.
- **Overfitting.** Training loss falls throughout, while validation loss rises
  after early epochs for the BiLSTM. F1-based checkpointing keeps the best
  validation epoch; dropout or early stopping would shorten training.
- **RNN instability.** RNN validation F1 varies widely between epochs (for
  example, the frequency RNN's best epoch is 53 of 2500). In the multivariate
  layout the RNN does not fit its training data, and a GRU replaces it
  (Section 10.7).
- **Reference quality.** The IMU `ax` reference correlates only 0.30 with
  measured `ax`, because commanded acceleration leads the vehicle's response.

## 10. Multivariate layout

`--layout multivariate` ([`ugv_multivariate.py`](ugv_multivariate.py)) gives
every time step all sensor channels at once. The classifiers output one fault
decision per faultable channel, so a model can check a channel against the
other sensors in the same window. The channel layout of Sections 3–7 scores
each channel window separately, without channel identity or the other sensors.

### 10.1 Inputs and labels

| Item | Value |
| --- | --- |
| Time base | IMU stamps (50 Hz). Wheel odometry (~19 Hz), GPS (5 Hz), and the drive command (20 Hz) are sample-and-hold: each step carries the latest sample at or before it. |
| Window | 100 steps (2 s) for every channel |
| Stride | 25 steps for training windows (75% overlap); validation and test windows do not overlap |
| Features per step (`F = 15`) | IMU `wx, wy, wz, ax, ay, az`; wheel `vx, wz`; GPS east, north, up (m); wheel-odometry east, north (m); commanded linear and angular speed |
| Positions | GPS east / north and odometry east / north are relative to the odometry position at the window's first step; GPS altitude stays absolute |
| Scaling | Per-feature mean and standard deviation over fault-free training windows |
| Frequency domain | One-sided orthonormal FFT magnitude of each standardized feature, shape `(51, 15)` |
| Labels | `(N, 11)` 0/1, one column per faultable channel. A (window, channel) pair is faulty when any held sample of that channel in the window carries its fault label. |

Simulator truth is not an input. Each window holds 11 (window, channel) pairs.

### 10.2 Models

| Model | Architecture | Output and decision | Parameters |
| --- | --- | --- | ---: |
| BiLSTM | Section 4.3 with `F = 15` and `Linear(100 → 11)` | 11 logits; a channel is faulty when its logit > 0 | 88,711 |
| RNN | Section 4.2 with `F = 15` and `Linear(50 → 11)` | as BiLSTM | 9,011 |
| GRU ([`models/Gru.py`](models/Gru.py)) | `nn.GRU(15, 50, num_layers=2)` → hidden state at the last step → `Linear(50 → 11)` | as BiLSTM | 25,911 |
| Autoencoder | `Linear(L·15 → 50)` → ReLU → `Linear(50 → L·15)` | Channel score: mean squared reconstruction error of that channel's feature column; faulty above the channel's 99th-percentile clean-training error | 151,550 (time), 77,315 (frequency) |
| Ensemble | BiLSTM OR autoencoder per (window, channel) | — | — |

The autoencoder row describes the weights in Section 10.6. The config now
adds a 60-unit hidden encoder layer and LeakyReLU activations
(`encoder_dims`, `activation` in `ModelParameters.json`; Section 10.8);
training records the layout in `model_config.json`, and folders without that
entry load as the single-layer ReLU network. Section 10.9 describes per-channel
specialist versions of each model and a 2-of-3 majority vote across model types.

### 10.3 Training

| Setting | First round (Section 10.6) | Second round (Section 10.7) |
| --- | --- | --- |
| Training variants | `datasets/ugv_faulty` training drives + `datasets/ugv_faulty_long` | first round + `datasets/ugv_faulty_train_balanced` |
| Validation variants | 5 validation drives, 15 fault events | + `datasets/ugv_faulty_val_balanced`, 180 fault events |
| Training windows | 126,160 (27,573 faulty pairs) | 281,920 (66,302 faulty pairs) |
| Classifier loss | `BCEWithLogitsLoss`, positive weight per channel = clean / faulty training windows of that channel (19 for wheel `vx` to 161 for IMU `wy`) | same (28–74) |
| Optimizer, batch | Adam lr 0.001, batch 256 | same, gradient L2 norm clipped to 1.0 (`--grad-clip 1.0`) |
| Epochs | 700 (about the optimizer steps of Section 7.1) | at most 200; stop after 30 epochs without a better validation score (`--patience 30`) |
| `best_model.pt` | Classifiers: highest micro-averaged F1 over validation (window, channel) pairs. Autoencoder: lowest validation MSE on clean windows. | same |
| Models trained | Autoencoder, RNN, BiLSTM | GRU, BiLSTM |

### 10.4 Layout-independent metrics

Window F1 depends on window length: a GPS fault spans one 20 s window in the
channel layout and ten 2 s windows here. [`ugv_eval.py`](ugv_eval.py) reduces
both layouts to (window, channel) pairs and adds two measures that compare them:

| Metric | Definition |
| --- | --- |
| Event isolation recall | Fraction of injected faults (one fault on one channel of one variant) for which at least one window containing faulty samples is flagged on **that** channel |
| Clean time flagged | Duration of falsely flagged clean pairs over the duration of all clean pairs, per channel, averaged over the 11 channels |

Both layouts save these scores, with per-type and per-channel breakdowns, to
`models/results/<weight folder>.json`; [`summarize_results.py`](summarize_results.py)
prints the comparison tables.

### 10.5 Balanced test set

The test split of Section 3.5 holds 15 fault events, and three channels (IMU
`wx`, `ax`, `az`) have none. `datasets/ugv_faulty_test_balanced/` holds 165 variants of the five test
drives: one per fault type and faultable channel, so each channel and type is
scored 15 times. It is scored as the separate `test_extra` split and is not
used for training, checkpoints, or thresholds:

```bash
./bin/python fault_injection.py --input datasets/ugv_clean --output datasets/ugv_faulty_test_balanced \
  --runs run_1005 run_1010 run_1021 run_1022 run_1026 --balanced --normal-variants 0 --seed 2
```

Fault magnitudes, durations, and onsets are drawn from the ranges in
Section 2.2.

### 10.6 Results: first round

`test_extra` (165 fault events: 55 per type), `best_model.pt`. Channel-layout
rows are the Section 7 and 7.1 weights, rescored with the same metrics.

| Model | Layout | Domain | Event recall | Short | Noise | Constant | Clean time flagged | Window precision |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| BiLSTM | channel + long | time | 0.770 | 0.47 | 0.95 | 0.89 | 0.99% | 0.760 |
| BiLSTM | channel + long | frequency | 0.776 | 0.51 | 0.93 | 0.89 | 0.39% | 0.797 |
| Ensemble | channel + long | frequency | 0.806 | 0.56 | 0.96 | 0.89 | 1.28% | 0.551 |
| BiLSTM | multivariate | time | 0.812 | 0.65 | 0.96 | 0.82 | 0.30% | 0.842 |
| BiLSTM | multivariate | frequency | **0.885** | 0.71 | 0.98 | 0.96 | **0.18%** | **0.905** |
| RNN | multivariate | time | 0.812 | 0.49 | 1.00 | 0.95 | 12.13% | 0.108 |
| RNN | multivariate | frequency | 0.782 | 0.69 | 1.00 | 0.65 | 5.80% | 0.102 |
| Autoencoder | multivariate | time | 0.618 | 0.49 | 0.73 | 0.64 | 1.21% | 0.506 |
| Autoencoder | multivariate | frequency | 0.673 | 0.47 | 0.73 | 0.82 | 2.58% | 0.337 |
| Ensemble | multivariate | time | 0.848 | 0.67 | 0.98 | 0.89 | 1.49% | 0.537 |
| Ensemble | multivariate | frequency | 0.903 | 0.71 | 1.00 | 1.00 | 2.74% | 0.394 |

Per-channel event recall of the two frequency BiLSTMs (15 events per channel):

| Channel | Channel layout + long | Multivariate |
| --- | ---: | ---: |
| IMU `wx` | 0.93 | 1.00 |
| IMU `wy` | 0.67 | 0.60 |
| IMU `wz` | 0.67 | 0.80 |
| IMU `ax` | 0.67 | 0.80 |
| IMU `ay` | 0.80 | 0.67 |
| IMU `az` | 0.73 | 1.00 |
| Wheel `vx` | 0.87 | 1.00 |
| Wheel `wz` | 0.87 | 1.00 |
| GPS latitude | 0.67 | 0.87 |
| GPS longitude | 0.67 | 1.00 |
| GPS altitude | 1.00 | 1.00 |

Observations:

- The multivariate frequency BiLSTM finds 146 of 165 faults (0.885), compared
  with 128 (0.776) for the best channel-layout model, and flags 0.18% of clean
  time instead of 0.39%. The gain is largest for short spikes (0.51 → 0.71)
  and on the GPS horizontal, IMU `az`, and wheel channels; IMU `wy` and `ay`
  are slightly lower.
- The multivariate autoencoders find more faults than the channel-layout
  autoencoders trained on the same drives (0.618 and 0.673, compared with
  0.412 and 0.448), mostly constant offsets, and flag more clean time.
- The RNN does not fit the training data. Its training loss stays near 0.75
  (BiLSTM: below 0.01), and the frequency RNN's training loss rises from 0.75
  to 1.16 after epoch 300. Its event recall comes from over-flagging: the
  time and frequency RNNs flag 12.1% and 5.8% of clean time at window precision
  0.11 and 0.10. The frequency RNN's best validation epoch is 16, and its final
  epoch flags 27% of clean time. Raising the logit threshold up to log(positive
  weight) gives at most window F1 0.29 on `test_extra`.
- The ensembles add up to 3.6 points of event recall over the BiLSTM and flag
  5–15 times as much clean time.
- Longer training does not raise held-out recall. The frequency BiLSTM's
  saved checkpoints reach event recall 0.879 at epoch 100 and 0.861 at epoch
  700, while its validation loss rises from 0.87 to 8.6 and its training loss
  falls below 0.01.
- Constant offsets are found as events (0.96) but only 71% of their faulty
  windows are flagged, so a detection is not sustained for the rest of the
  drive.
- **The first-round validation split is small.** It holds 15 fault events, with
  none on IMU `wz`, IMU `az`, or GPS latitude. The frequency BiLSTM's best
  validation F1 is 0.654 while its `test_extra` window F1 is 0.804, and one
  missed event moves validation F1 by several points.

Weights, logs, and scores: `models/<name>Weights_ugv_mv[_freq]/`,
`models/train_logs/ugv_mv_<domain>_<model>_700.log`,
`models/results/<name>Weights_ugv_mv[_freq].json`.

### 10.7 Second round: balanced faults, GRU, early stopping

Training faults per channel in the first round (20 original and 30 long
training drives; 210 events):

| Channel | Short | Noise | Constant |
| --- | ---: | ---: | ---: |
| IMU `wx` | 1 | 7 | 1 |
| IMU `wy` | 5 | 3 | 2 |
| IMU `wz` | 5 | 6 | 3 |
| IMU `ax` | 2 | 11 | 3 |
| IMU `ay` | 0 | 7 | 4 |
| IMU `az` | 3 | 8 | 1 |
| Wheel `vx` | 9 | 24 | 11 |
| Wheel `wz` | 8 | 12 | 6 |
| GPS latitude | 6 | 13 | 9 |
| GPS longitude | 7 | 12 | 7 |
| GPS altitude | 4 | 7 | 3 |

Faults are assigned to a random channel, so IMU channels, which have the most
windows, have the fewest training faults; IMU `ay` has no short-spike example.
The second round changes four things:

1. **Balanced training faults:** `datasets/ugv_faulty_train_balanced/` adds 660
   variants of the 20 training drives (one per fault type and channel; seed 4),
   giving every channel and type 20 more training events.
2. **Balanced validation faults:** `datasets/ugv_faulty_val_balanced/` adds 165
   variants of the five validation drives (seed 3) to the validation split
   (`--extra-val-dir`), so checkpoint selection and early stopping use 180
   events with every channel and type represented.
3. **GRU instead of RNN:** gated recurrence, same hidden size and depth.
4. **Gradient clipping and early stopping** (Section 10.3).

```bash
./bin/python fault_injection.py --input datasets/ugv_clean --output datasets/ugv_faulty_train_balanced \
  --runs run_1000 run_1001 run_1002 run_1003 run_1004 run_1006 run_1008 run_1009 run_1011 run_1012 \
         run_1013 run_1014 run_1015 run_1017 run_1018 run_1019 run_1020 run_1023 run_1027 run_1028 \
  --balanced --normal-variants 0 --seed 4
./bin/python fault_injection.py --input datasets/ugv_clean --output datasets/ugv_faulty_val_balanced \
  --runs run_1007 run_1016 run_1024 run_1025 run_1029 --balanced --normal-variants 0 --seed 3
./bin/python main.py --dataset ugv --layout multivariate --model bilstm --device mps --domain frequency \
  --epochs 200 --patience 30 --grad-clip 1.0 --batch-size 256 --save-every 25 --tag balanced \
  --extra-train-dir datasets/ugv_faulty_long --extra-train-dir datasets/ugv_faulty_train_balanced \
  --extra-val-dir datasets/ugv_faulty_val_balanced --extra-test-dir datasets/ugv_faulty_test_balanced
```

The GRU trains with `--model gru --device cpu` and `OMP_NUM_THREADS=1`: its
small layers run at 36 ms per batch on one thread and 70 ms on eight.

Results on `test_extra` (165 fault events), `best_model.pt`:

| Model | Domain | Best epoch / last epoch | Validation F1 | Event recall | Short | Noise | Constant | Clean time flagged | Window precision |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| BiLSTM | time | 107 / 137 | 0.924 | 0.964 | 0.93 | 1.00 | 0.96 | **0.13%** | **0.936** |
| GRU | time | 122 / 152 | 0.904 | **0.970** | 0.93 | 1.00 | 0.98 | 0.16% | 0.925 |
| BiLSTM | frequency | 106 / 136 | 0.853 | 0.945 | 0.84 | 1.00 | 1.00 | 0.49% | 0.801 |
| GRU | frequency | 63 / 93 | 0.741 | 0.964 | 0.89 | 1.00 | 1.00 | 1.02% | 0.664 |
| *First round:* BiLSTM | frequency | 571 / 700 | 0.654 | 0.885 | 0.71 | 0.98 | 0.96 | 0.18% | 0.905 |
| *Channel layout + long:* BiLSTM | frequency | 325 / 400 | 0.766 | 0.776 | 0.51 | 0.93 | 0.89 | 0.39% | 0.797 |

Per-channel event recall of the time-domain models (15 events per channel):

| Channel | BiLSTM | GRU |
| --- | ---: | ---: |
| IMU `wx`, `az`, `ay`; wheel `vx`; GPS longitude, altitude | 1.00 | 1.00 |
| IMU `ax` | 1.00 | 0.93 |
| IMU `wy` | 0.80 | 0.87 |
| IMU `wz` | 1.00 | 0.93 |
| Wheel `wz` | 0.93 | 1.00 |
| GPS latitude | 0.87 | 0.93 |

Observations:

- The time-domain BiLSTM and GRU find 159 and 160 of 165 faults and flag
  0.13% and 0.16% of clean time. Both detect 93% of short-spike faults,
  compared with 71% for the best first-round model.
- IMU `ay`, which has no short-spike training fault in the first round, rises
  from 0.67 to 1.00 (frequency BiLSTM to time BiLSTM). IMU `wy` rises from 0.60
  to 0.80–0.87 and holds 2–3 of the 5–6 remaining misses.
- The GRU (25,911 parameters) matches the BiLSTM (88,711) in the time domain.
  Its training loss falls to about 0.015, where the first-round RNN stays near 0.75.
- Both frequency-domain models flag more clean time than in the first round,
  mostly on IMU `ax` and `wz` (1.5% each for the BiLSTM; 2.6% and 3.9% for the
  GRU).
- Validation now agrees with `test_extra`: the time BiLSTM's validation F1 is
  0.924 and its `test_extra` window F1 is 0.886. In the first round the
  frequency BiLSTM had 0.654 and 0.804.
- Early stopping ends the runs after 93–152 epochs; the BiLSTMs take about 50
  minutes each with four runs sharing this Mac.
- **The balanced training and test faults share one generator.** Both draw
  from the Section 2.2 types and ranges, on different drives. These results
  measure coverage of the simulated fault types, not detection of fault types
  absent from training.

Weights, logs, and scores: `models/<name>Weights_ugv_mv_balanced[_freq]/`,
`models/train_logs/ugv_mv_balanced_<domain>_<model>.log`,
`models/results/<name>Weights_ugv_mv_balanced[_freq].json`.

### 10.8 Autoencoder redesign

Changes to the autoencoder of Section 10.2:

- a 60-unit hidden layer between the input and the 50-unit bottleneck
  (15 sensors × 4 states: three fault types and normal);
- LeakyReLU (slope 0.01) instead of ReLU. The first-round frequency
  autoencoder has 33 of its 50 bottleneck units inactive on every clean
  validation window, and in the time autoencoder at least half the units are
  active on every window, so it is nearly linear;
- optionally, a mirrored 60-unit decoder layer (`--ae-decoder-dims 60`);
- windows of 100 or 50 steps (`--seq-len`), both with training windows every 25
  steps (`--train-stride 25`).

Each variant trains on the second-round data of Section 10.7 (clean windows
only), at most 300 epochs with `--patience 30`. Thresholds are a per-channel
percentile of clean training error. To compare variants at an equal false-alarm
rate, the percentile is the one whose clean time flagged on **validation** is
closest to 1%; `test_extra` is scored at that percentile:

| Domain | Autoencoder | Percentile | Validation event recall | `test_extra` event recall | Short | Noise | Constant | `test_extra` clean time flagged |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| time | first round, single layer, 100 steps | 99.5 | 0.572 | 0.594 | 0.47 | 0.69 | 0.62 | 0.82% |
| time | hidden layer, 100 steps | 99.3 | 0.600 | 0.636 | 0.47 | 0.76 | 0.67 | 0.97% |
| time | hidden layer + mirrored decoder, 100 steps | 99.3 | 0.594 | **0.648** | 0.49 | 0.76 | 0.69 | 1.07% |
| time | hidden layer, 50 steps | 99.3 | 0.589 | 0.582 | 0.49 | 0.75 | 0.51 | 1.08% |
| time | hidden layer + mirrored decoder, 50 steps | 99.3 | 0.589 | 0.576 | 0.49 | 0.73 | 0.51 | 1.10% |
| frequency | first round, single layer, 100 steps | 99.9 | 0.600 | 0.545 | 0.44 | 0.56 | 0.64 | 1.08% |
| frequency | hidden layer, 100 steps | 99.95 | 0.589 | 0.570 | 0.42 | 0.65 | 0.64 | 4.57% |
| frequency | hidden layer + mirrored decoder, 100 steps | 99.95 | 0.572 | 0.570 | 0.44 | 0.64 | 0.64 | 5.33% |
| frequency | hidden layer, 50 steps | 99.95 | 0.517 | 0.521 | 0.40 | 0.58 | 0.58 | 2.10% |
| frequency | hidden layer + mirrored decoder, 50 steps | 99.95 | 0.533 | 0.521 | 0.38 | 0.60 | 0.58 | 2.70% |

Observations:

- In the time domain, the hidden layer and LeakyReLU raise `test_extra` event
  recall from 0.594 to 0.636–0.648 at about 1% clean time flagged. The mirrored
  decoder changes it by 0.012, two events.
- 50-step windows score lower than 100-step windows in both domains, mostly on
  constant offsets (time: 0.67 → 0.51).
- The redesigned frequency autoencoders do not reach 1% clean time flagged on
  validation at any tested percentile up to 99.95; on `test_extra` they flag
  2.1–5.3%.
- All autoencoders remain far below the time-domain classifiers of Section 10.7
  (0.964–0.970 event recall at 0.13–0.16% clean time flagged).
- **Clean reconstruction error is set by sensor noise on several channels.**
  Mean clean validation error of the mirrored 100-step time autoencoder, in
  standardized units: IMU `wx` 1.01, `az` 0.88, `wy` 0.59, `ay` 0.53, `ax`
  0.51; IMU `wz`, wheel, and GPS channels 0.01–0.17. An error near 1 means the
  reconstruction explains almost none of that channel's variance. On flat
  ground, IMU roll rate is mostly the injected white noise and `az` carries
  wheel-contact vibration, which no autoencoder can predict from the rest of
  the window.

At the default 99th percentile, the `test_extra` scores are in
`models/results/autoencoderWeights_ugv_mv_<tag>[_freq].json` (`<tag>`: `w100`,
`w50`, `w100_mirror`, `w50_mirror`); logs are
`models/train_logs/ugv_mv_<tag>_<domain>_autoencoder.log`.

```bash
./bin/python main.py --dataset ugv --layout multivariate --model autoencoder --device mps \
  --domain time --seq-len 100 --train-stride 25 --ae-decoder-dims 60 --tag w100_mirror \
  --epochs 300 --patience 30 --grad-clip 1.0 --batch-size 256 --save-every 50 \
  --extra-train-dir datasets/ugv_faulty_long --extra-train-dir datasets/ugv_faulty_train_balanced \
  --extra-val-dir datasets/ugv_faulty_val_balanced --extra-test-dir datasets/ugv_faulty_test_balanced
```

### 10.9 Per-channel specialists and majority vote

This experiment compares one network over all sensors with ensembles of small
networks. For each model type, `--specialists` trains 11 specialists. Specialist
*k* sees only the feature column of channel *k* (no other sensors, no commands)
and outputs only channel *k*'s decision. `--model vote` then combines model
types: a (window, channel) pair is faulty when at least two of the BiLSTM, GRU,
and autoencoder flag it. The vote runs once over the specialists and once over
the single networks.

| Model | One specialist | 11 specialists, trainable parameters | Single network |
| --- | --- | ---: | ---: |
| BiLSTM | 2-layer BiLSTM, hidden 15, input 1 → `Linear(30 → 1)` | 86,141 | 88,711 |
| GRU | 2-layer GRU, hidden 15, input 1 → `Linear(15 → 1)` | 24,926 | 25,911 |
| Autoencoder | `Linear(L → 128)` → LeakyReLU → `Linear(128 → 4)` → LeakyReLU → `Linear(4 → L)` | 153,384 (time), 81,697 (frequency) | 169,610 / 88,025 (Section 10.8, 100 steps) |

[`models/Specialists.py`](models/Specialists.py) packs the 11 recurrent
networks into one `nn.LSTM` or `nn.GRU`, with weights masked to a
block-diagonal pattern. Weights between channels start at zero and receive zero
gradient, so they stay zero. [`test_training.py`](test_training.py) checks two
things: the packed model's outputs equal those of 11 separately built networks,
and masked weights stay zero after Adam steps. Packing runs 3–4 times faster
than 11 separate networks on the Apple GPU.

Packed specialists share some training choices:

- batches;
- the stopping epoch and checkpoint (validation micro-F1 over all channels);
- gradient-norm clipping, computed over all 11 networks together.

Their weights do not interact.

The specialists use the second-round data and settings of Section 10.7: at most
200 epochs for the classifiers and 300 for the autoencoder, all with
`--patience 30`. The single-network vote uses the BiLSTM and GRU of Section 10.7
and the 100-step autoencoder of Section 10.8. Autoencoder members use the 99th
percentile of clean training error.

```bash
DATA="--extra-train-dir datasets/ugv_faulty_long --extra-train-dir datasets/ugv_faulty_train_balanced \
  --extra-val-dir datasets/ugv_faulty_val_balanced --extra-test-dir datasets/ugv_faulty_test_balanced"
./bin/python main.py --dataset ugv --layout multivariate --specialists --model gru --device mps \
  --domain time --tag specialists --epochs 200 --patience 30 --grad-clip 1.0 --batch-size 256 \
  --save-every 25 --train-stride 25 $DATA
./bin/python main.py --dataset ugv --layout multivariate --mode run --model vote --domain time \
  --tag specialists $DATA
./bin/python main.py --dataset ugv --layout multivariate --mode run --model vote --domain time \
  --tag balanced --vote-tag autoencoder=w100 $DATA
```

Results on `test_extra` (165 fault events), `best_model.pt`, default decision
rules (logit > 0; autoencoder 99th percentile):

| Model | Domain | Best epoch / last epoch | Event recall | Short | Noise | Constant | Clean time flagged | Window precision |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| BiLSTM, single | time | 107 / 137 | 0.964 | 0.93 | 1.00 | 0.96 | 0.13% | 0.936 |
| BiLSTM, specialists | time | 44 / 74 | 0.964 | 0.89 | 1.00 | 1.00 | 2.76% | 0.411 |
| GRU, single | time | 122 / 152 | **0.970** | 0.93 | 1.00 | 0.98 | 0.16% | 0.925 |
| GRU, specialists | time | 17 / 47 | 0.927 | 0.87 | 1.00 | 0.91 | 2.10% | 0.475 |
| Autoencoder, single | time | 98 / 128 | 0.642 | 0.47 | 0.78 | 0.67 | 1.16% | 0.524 |
| Autoencoder, specialists | time | 63 / 93 | 0.636 | 0.51 | 0.80 | 0.60 | 0.77% | 0.574 |
| Vote, single networks | time | — | 0.939 | 0.85 | 1.00 | 0.96 | **0.03%** | **0.985** |
| Vote, specialists | time | — | 0.909 | 0.82 | 1.00 | 0.91 | 1.32% | 0.583 |
| BiLSTM, single | frequency | 106 / 136 | 0.945 | 0.84 | 1.00 | 1.00 | 0.49% | 0.801 |
| BiLSTM, specialists | frequency | 40 / 70 | 0.982 | 0.96 | 1.00 | 0.98 | 2.90% | 0.396 |
| GRU, single | frequency | 63 / 93 | 0.964 | 0.89 | 1.00 | 1.00 | 1.02% | 0.664 |
| GRU, specialists | frequency | 39 / 69 | 0.982 | 0.96 | 1.00 | 0.98 | 2.94% | 0.390 |
| Autoencoder, single | frequency | 163 / 193 | 0.642 | 0.44 | 0.76 | 0.73 | 6.22% | 0.179 |
| Autoencoder, specialists | frequency | 48 / 78 | 0.612 | 0.47 | 0.82 | 0.55 | 0.73% | 0.594 |
| Vote, single networks | frequency | — | 0.939 | 0.82 | 1.00 | 1.00 | 0.21% | 0.900 |
| Vote, specialists | frequency | — | 0.970 | 0.93 | 1.00 | 0.98 | 1.89% | 0.496 |

The specialist classifiers flag 13–21 times as much clean time as the single
time-domain networks, so their event recall is not comparable at logit > 0.
To compare at an equal false-alarm rate, each specialist channel's logit
threshold is the **validation** quantile that flags the same fraction of clean
(window, channel) pairs as the single network does at logit > 0.
`test_extra` is scored once at those thresholds:

| Model | Domain | Single network: event recall / clean time flagged | Specialists, matched: event recall / clean time flagged | Constant offsets, single → specialists |
| --- | --- | ---: | ---: | ---: |
| BiLSTM | time | 0.964 / 0.13% | 0.885 / 0.25% | 0.96 → 0.85 |
| GRU | time | 0.970 / 0.16% | 0.848 / 0.17% | 0.98 → 0.73 |
| BiLSTM | frequency | 0.945 / 0.49% | 0.909 / 0.69% | 1.00 → 0.93 |
| GRU | frequency | 0.964 / 1.02% | 0.921 / 1.37% | 1.00 → 0.89 |

The autoencoders are compared as in Section 10.8, at the clean-training
percentile whose validation clean time flagged is closest to 1%:

| Domain | Autoencoder | Percentile | `test_extra` event recall | `test_extra` clean time flagged |
| --- | --- | ---: | ---: | ---: |
| time | single, 100 steps | 99.3 | 0.636 | 0.97% |
| time | specialists | 98.5 | 0.642 | 1.20% |
| frequency | single, 100 steps | 99.95 | 0.570 | 4.57% |
| frequency | specialists | 98.5 | 0.636 | 1.03% |

Per-channel clean time flagged on `test_extra` at logit > 0, time domain:

| Channel | BiLSTM, single | BiLSTM, specialists | GRU, single | GRU, specialists |
| --- | ---: | ---: | ---: | ---: |
| GPS latitude, longitude, altitude | 0.06–0.22% | 0.06–0.17% | 0.03–0.23% | 0.10–0.40% |
| IMU `wx`, `az`, `ax` | 0.01–0.10% | 0.38–0.79% | 0.00–0.15% | 0.15–0.48% |
| IMU `wy` | 0.09% | 0.79% | 0.18% | 4.39% |
| IMU `ay` | 0.05% | 4.05% | 0.03% | 3.53% |
| Wheel `vx` | 0.09% | 3.91% | 0.06% | 2.74% |
| IMU `wz` | 0.51% | 7.08% | 0.62% | 1.13% |
| Wheel `wz` | 0.09% | 12.36% | 0.23% | 9.53% |

At matched thresholds, the time GRU specialists find 6 of 15 IMU `wz` faults
and 11 of 15 wheel `wz` faults; the single GRU finds 14 and 15.

Observations:

- **The single classifiers beat the specialist ensembles at equal false-alarm
  rates.** At thresholds matched to the single networks' validation
  false-alarm rate, the specialists find 0.848–0.921 of `test_extra` faults.
  The single networks find 0.945–0.970. On `test_extra` the specialists still
  flag slightly more clean time (0.17–1.37% against 0.13–1.02%).
- **The gap is on channels that follow the vehicle's motion.** Specialist
  false alarms concentrate on several channels:
  - wheel `wz`;
  - IMU `ay` (lateral acceleration, the product of speed and yaw rate);
  - wheel `vx`;
  - IMU `wz` (BiLSTM) or `wy` (GRU).

  A turn or
  speed change looks like a fault on one of these channels unless it is checked
  against the drive command or the redundant sensor (wheel yaw rate against IMU
  yaw rate). The single network sees both. On GPS channels the specialists
  match the single networks.
- **Constant offsets lose the most** (0.96–1.00 → 0.73–0.93 at matched
  thresholds). An offset on a motion channel resembles a steady turn or a speed
  change. Noise bursts stay at 1.00 detection with either design, since they
  show in one channel alone.
- **The specialist autoencoders equal or beat the single autoencoder.** In the
  time domain they score within one event at about 1% clean time flagged. The
  single autoencoder already scores each channel by its own reconstruction
  error, and that error is bounded by sensor noise (Section 10.8). In the
  frequency domain the specialists reach 1% clean time flagged at the 98.5th
  percentile, which the single frequency autoencoder does not reach at any tested
  percentile (0.636 at 1.03% against 0.570 at 4.57%). The cause of the single
  frequency autoencoder's false alarms is not isolated.
- **The single-network vote trades recall for precision.** It flags 0.03% of
  clean time at window precision 0.985 (time), against 0.13% for the best
  member, and finds 155 of 165 faults rather than 159–160. The autoencoder
  finds only 0.64 of faults, so a detection usually needs both classifiers;
  the vote acts close to "BiLSTM and GRU".
- **The specialist vote lowers the specialists' false alarms** from 2.1–2.9% to
  1.3–1.9%. That is still 9 times (frequency) to 44 times (time) the
  single-network vote.
- Validation micro-F1 of the specialist classifiers is 0.52–0.57, against
  0.74–0.92 for the single networks. Validation stops them after 47–74 epochs.
- Training time, four runs sharing this Mac: specialist BiLSTMs 81–114
  minutes, specialist GRUs 124–132 minutes (single BiLSTMs about 50 minutes),
  specialist autoencoders 7–9 minutes.
- **Caveats.** The GPS east and north columns are relative to the odometry
  position at the window's first step (Section 10.1), so GPS specialists see one
  odometry sample indirectly. Packed specialists share a stopping epoch and
  gradient clipping, unlike separately tuned networks, and their per-channel
  thresholds were not tuned beyond the matched comparison above. The balanced
  training and test faults share one generator (Section 10.7).

Weights, logs, and scores:

- `models/<name>Weights_ugv_mv_specialists[_freq]/`
- `models/train_logs/ugv_mv_specialists_<domain>_<model>.log`
- `models/train_logs/ugv_mv_{specialists,balanced}_<domain>_vote.log`
- `models/results/voteWeights_ugv_mv_{specialists,balanced}[_freq].json`

### 10.10 Limitations of the multivariate layout

- **Positive weights move the decision threshold.** With positive weight `w`,
  logit > 0 corresponds to an unweighted fault probability above 1 / (1 + w),
  that is 0.6–5% in the first round.
- **Normalization statistics are not saved with the weights.** `--mode run`
  must pass the same `--extra-train-dir` options as training.
- **One fault per variant.** Simultaneous faults on several channels, where
  cross-sensor checks matter most, are not represented in training or test.
- **Balanced sets share drives.** The 165 `test_extra` variants come from five
  drives, so faults on the same drive share the vehicle motion.
