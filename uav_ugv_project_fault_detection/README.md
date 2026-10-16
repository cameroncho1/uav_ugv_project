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
