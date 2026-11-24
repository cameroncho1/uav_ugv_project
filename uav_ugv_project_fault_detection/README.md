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
`--normal-variants`, and `--types`. `--count TYPE=N` overrides the count for one
type, and `--noise-window MIN MAX` sets the noise-fault length in seconds.
`--runs RUN ...` restricts injection to the named clean runs, and `--balanced`
writes one variant per fault type and faultable channel
(`run_<seed>_<type>_<sensor>-<column>/`, 33 per run) instead of random channels.

`datasets/ugv_faulty_test_balanced/` holds 165 such variants of the five test
drives, so every channel and fault type is scored 15 times; it is used only as
the `test_extra` split (`--extra-test-dir`):

```
./bin/python fault_injection.py --input datasets/ugv_clean --output datasets/ugv_faulty_test_balanced \
  --runs run_1005 run_1010 run_1021 run_1022 run_1026 --balanced --normal-variants 0 --seed 2
```

`datasets/ugv_clean_long/` holds 30 further 300 s drives (seeds 1030–1059,
recorded at `UPDATE_RATE=1000`, so truth is about 15 Hz; sensor rates are
unchanged). Their training variants add three longer noise faults per drive:

```
./bin/python fault_injection.py --input datasets/ugv_clean_long --output datasets/ugv_faulty_long \
  --count noise=3 --noise-window 30 120 --seed 1
./bin/python main.py --dataset ugv --extra-train-dir datasets/ugv_faulty_long ...
```

`--extra-train-dir` adds variants to the training split only; validation and
test remain the five held-out drives each from `datasets/ugv_faulty`. With the
long drives the training split grows from 33,920 to 224,540 windows and its
noise-fault windows from 97 to 2,095. Each sensor CSV gains `fault` (0/1),
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

Normalization statistics come from the training split and are not stored with
the weights, so `--mode run` must use the training directories of the run
(for `*_long` weights: `--tag long --extra-train-dir datasets/ugv_faulty_long`).

### Multivariate layout

`--layout multivariate` ([`ugv_multivariate.py`](ugv_multivariate.py)) gives
every 50 Hz time step all sensor channels at once (15 features: IMU, wheel
speeds, GPS and wheel-odometry position, drive command). The RNN and BiLSTM
output one sigmoid per faultable channel (11), so a window can be clean or
faulty on any subset of channels; the autoencoder flags a channel when its
feature's reconstruction error exceeds that channel's threshold. See
[MODELS.md, Section 10](MODELS.md#10-multivariate-layout).

```
./bin/python main.py --dataset ugv --layout multivariate --model bilstm --device mps \
  --epochs 700 --batch-size 256 --save-every 100 \
  --extra-train-dir datasets/ugv_faulty_long --extra-test-dir datasets/ugv_faulty_test_balanced
./bin/python main.py --dataset ugv --layout multivariate --mode run --model ensemble \
  --extra-train-dir datasets/ugv_faulty_long --extra-test-dir datasets/ugv_faulty_test_balanced
./bin/python summarize_results.py              # layout comparison on test_extra
./bin/python test_training.py                  # Trainer options, GRU, specialists, split rules
```

`datasets/ugv_faulty_train_balanced/` and `datasets/ugv_faulty_val_balanced/`
hold balanced variants of the training and validation drives
(`fault_injection.py --balanced`). `--extra-val-dir` adds variants of the
validation drives to the validation split. `--model gru` trains the gated
recurrent classifier that replaces the plain RNN; train it on `--device cpu`
with `OMP_NUM_THREADS=1`. `--grad-clip` limits each step's gradient norm, and
`--patience` stops training after that many epochs without a better
validation score ([MODELS.md, Section 10.7](MODELS.md#107-second-round-balanced-faults-gru-early-stopping)).

`--specialists` trains 11 small networks for the chosen model, one per
faultable channel. Each sees only its own channel's feature column. With
`--mode run`, `--model vote` flags a (window, channel) pair when at least two of
the BiLSTM, GRU, and autoencoder flag it. Members load from `--tag`, or per
model from `--vote-tag MODEL=TAG` (for example `--vote-tag autoencoder=w100`).
See [MODELS.md, Section 10.9](MODELS.md#109-per-channel-specialists-and-majority-vote).

Weights go to `models/<name>Weights_ugv_mv[_freq]/`. Both layouts save their
validation and test scores to `models/results/<weight folder>.json`, including
event isolation recall and clean time flagged, which compare layouts with
different window lengths ([`ugv_eval.py`](ugv_eval.py)).

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

## Live detection in the simulation

[`sensor_fault_detector.py`](../ros2_ws/src/fault_management/scripts/sensor_fault_detector.py)
runs the paired time-domain BiLSTM (`bilstmWeights_ugv_paired`) on the prepared
UGV measurements (`/ugv/sensors/{imu,wheel_odom,gps}`) and the drive command.
It rebuilds each channel's last 100 samples with the training reference
([`sensor_fault_model.py`](../ros2_ws/src/fault_management/scripts/sensor_fault_model.py),
numpy only), scores every channel each 0.5 s of sim time (`evaluation_period`), and publishes one
`diagnostic_msgs/DiagnosticStatus` per channel (`ERROR` = fault) on
`/ugv/sensor_faults`. The goal demo starts it with `fault_detection:=true` (default
`false`); `use_autoencoder:=true` also flags windows above the autoencoder threshold.

Each evaluation also publishes one `uav_ugv_msgs/FaultStatus` class decision on
`/fault_detection/status` for the navigation supervisor
([`fault_decision`](../ros2_ws/src/fault_management/scripts/sensor_fault_model.py)):

| Flag | Set by | Supervisor response |
| --- | --- | --- |
| `odometry_scale` | BiLSTM flag on `wheel_odom/vx` | hold |
| `imu_drift` | BiLSTM flag on `imu/wz` | hold |
| `gps_outage` | no GPS fix for `gps_timeout` (1 s), or a flag on `gps/latitude` or `gps/longitude` | hold only if the pose guard fails |
| `uav_loss` | no UAV relative measurement for `uav_timeout` (2 s); off when `uav_assistance:=false` | hold only if the pose guard fails |

With `uav_dropout_control:=true` (default `false`) the goal demo's camera
producer publishes on `/ugv/sensors/uav_relative_pose_raw`, and
[`uav_measurement_gate`](../ros2_ws/src/fault_management/scripts/uav_measurement_gate.py)
forwards it to `/ugv/sensors/uav_relative_pose`. Without it the camera publishes
there directly and no gate runs. To inject F4, add `initial_uav_enabled:=false` or call
`ros2 service call /evaluation/set_uav_enabled std_srvs/srv/SetBool "{data: false}"`.
With `evaluation_trace:=true` each change is a `fault_event` on `/evaluation/faults`.
`uav_loss` also rises whenever the camera loses the marker for 2 s, for
example before the UAV is airborne.

`imu/{wx,wy,ax,ay,az}`, `wheel_odom/wz`, and `gps/altitude` set no flag; the
simulated wheel-speed scale fault changes only the forward speed, so
`wheel_odom/wz` cannot show it. A decision is valid when `imu/wz` and
`wheel_odom/vx` each have a current evaluation, and is stamped with the older
of their window ends. Before that it
is published with `window_valid: false`. If no newer window exists, nothing is
published, and the supervisor's 1 s decision-age limit holds. A clock reset
starts a new `source_id` and sequence. `fault_hold:=true` sets the goal
controller's `detector_enabled`, so the supervisor holds on wheel-scale or
yaw-drift flags and on missing or invalid decisions. It defaults to `false`
and needs `fault_detection:=true`; without decisions the supervisor holds.

A model-derived class flag (`odometry_scale`, `imu_drift`, and the model part
of `gps_outage`) is published only after its channel is flagged in
`flag_persistence` consecutive decision windows (default 3, 1.5 s). An invalid
decision resets the count. Isolated single-window flags therefore never reach
the supervisor, and a persistent fault is reported about 1 s later than its
first flagged window. Missing GPS fixes and missing UAV measurements set their
flags without this delay. The node limits numpy to one BLAS thread: with
multithreaded OpenBLAS on a CPU shared with Gazebo and PX4, one evaluation took
about 1 s instead of 0.1 s, decisions exceeded the supervisor's 1 s age limit,
and the UGV held on missing decisions.

Clean goal-demo trials with `fault_detection:=true fault_hold:=true
evaluation_trace:=true` after both changes (`flag_persistence` 3, single-thread
BLAS, `wheel_odom/wz` still mapped to `odometry_scale`; seed 513, goal (4, 0),
RViz on, one simulation at a time, 2026-11-24):

| Trial | Sim time recorded | Fault holds after driving began | Hold-channel flags → published class flags | Goal |
| --- | ---: | ---: | --- | --- |
| 1 | 64 s | 0 | 2 → 0 | reached |
| 2 | 143 s | 0 | 1 → 0 | reached |
| 3 | 146 s | 0 | 1 → 0 | reached |

No decision gap exceeded 0.9 s. Each run held only at startup, before driving,
until ten clear decisions spanned 5 s. In trial 1 the Gazebo and RViz windows
were closed at about 71 s sim time, after the goal; the simulation stopped and
the supervisor held on the stale pose. Three runs bound no false-alarm rate.
No faulted trial has been run with `flag_persistence` 3.

Earlier closed-loop goal-demo trials with `fault_detection:=true fault_hold:=true`
(seed 513, goal (4, 0), 150 s recorded, one simulation at a time, 2026-11-18),
before flag persistence, single-thread BLAS, and the `wheel_odom/wz` mapping
change. Counts are decisions with the flag set out of all decisions:

| Trial | `odometry_scale` | `imu_drift` | `gps_outage` | Fault holds while driving | Goal |
| --- | ---: | ---: | ---: | --- | --- |
| Nominal | 2 / 288 | 1 / 288 | 0 / 288 | one, `wheel_odom/wz` at 23.5 s, released after 5.7 s | reached |
| `measurement_profile:=drift` (wheel scale 1.15, yaw bias 0.002 rad/s, from start) | 4 / 282 | 2 / 282 | 10 / 282 | two, `wheel_odom/wz` at 25 s and `wheel_odom/vx` at 31 s, each released after 5–9 s | reached |

Both runs start in a fault hold until ten clear decisions span 5 s. Each flag
holds the UGV and recovery releases it, as the supervisor specifies. At the
envelope fault sizes the BiLSTM flags only isolated windows, and the nominal
run shows the same pattern, so these trials show that the hold path works.
They do not show detection of the drift faults. The drift run's `gps_outage`
flags and 1–3 s decision gaps near 101–112 s sim time come from sensor windows
that stopped advancing; the supervisor held on the missing decisions.

The node loads `fault_management/models/ugv_sensor_fault_detector.npz`: the
weights, the per-channel training statistics, and the autoencoder thresholds.
Re-export it after retraining (needs torch):

```
python3 export_live_detector.py --verify
```

`--verify` rebuilds the validation and test windows through the live feature
code and compares them with `ugv_dataset.py` and torch; on the current export
all 16,960 BiLSTM decisions match.

Live 120 s trials on the `ugv_dataset` world (seeded recorder maneuvers, unseen
seeds, nominal `measurement_model.yaml` noise, one simulation at a time). Values
are the fraction of 0.5 s evaluations the BiLSTM flags:

| Trial | Seed | Odometry drift | Faulted channel | Other channels |
| --- | --- | ---: | --- | --- |
| Nominal | 2001 | 3.0 m | — | flag onsets only (fractions not recorded): `gps/latitude` 6, `imu/ay` 2, `imu/wz`, `wheel_odom/wz`, `gps/longitude` 1 each |
| Nominal | 2002 | 9.9 m | — | `gps/longitude` 46%, `gps/latitude` 33%, `imu/ay` 1.7%, rest ≤ 0.4% |
| `imu_yaw_bias_rate:=0.1` (10σ `wz` offset) | 2003 | 2.9 m | `imu/wz` 66% | run in parallel with two other simulations |
| `wheel_speed_scale:=1.6` | 2004 | 3.2 m | `wheel_odom/vx` 68% | run in parallel with two other simulations |

The GPS reference is wheel-odometry position. When simulated wheel slip moves
odometry several meters from truth (seed 2002), GPS and its reference disagree
and the BiLSTM flags GPS; the offline pipeline flags the same drive at the same
rate. Three simulations in parallel on 5 cores also raised clean-channel flags
in the nominal run, so the two fault rows bound detection, not false alarms.
In the goal demo the command comes from the 10 Hz, unramped `/ugv/cmd_vel`
rather than the training recorder; the trials above do not cover it.
