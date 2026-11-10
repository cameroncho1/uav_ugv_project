# Offline sensor-fault injection for clean simulated UGV runs.
#
# Each input folder is one clean run written by
# ``scripts/collect_ugv_dataset.sh`` (imu.csv, wheel_odom.csv, gps.csv, ...).
# This module first adds nominal Gaussian sensor noise (the defaults of
# ``fault_management/scripts/sim_ugv_sensors.py``) and then injects one of three
# fault types into a single sensor channel:
#
#   - short:    isolated 1-3 sample spikes inside a burst window
#   - noise:    extra Gaussian noise over a time window
#   - constant: a fixed offset from onset to the end of the run
#
# Fault magnitudes are multiples of the channel's nominal noise standard
# deviation (sigma), so one configuration applies to every sensor. Every output
# CSV gains per-sample ``fault`` / ``fault_type`` / ``fault_column`` labels, and
# ``faults.json`` records the exact injection and the source run.

import argparse
import json
import math
import shutil
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

FAULT_TYPES = ("short", "noise", "constant")

# Nominal noise sigma per faultable channel. GPS values are meters and are
# converted to degrees for latitude / longitude.
NOMINAL_SIGMA = {
    "imu": {"wx": 0.01, "wy": 0.01, "wz": 0.01, "ax": 0.1, "ay": 0.1, "az": 0.1},
    "wheel_odom": {"vx": 0.05, "wz": 0.02},
    "gps": {"latitude": 1.5, "longitude": 1.5, "altitude": 3.0},
}

# Magnitude ranges in sigma units and window lengths in seconds.
SPIKE_SIGMA = (6.0, 15.0)
SPIKES_PER_WINDOW = (3, 8)
SPIKE_WIDTH = (1, 3)
NOISE_SIGMA = (3.0, 8.0)
OFFSET_SIGMA = (3.0, 10.0)
SHORT_WINDOW = (5.0, 20.0)
NOISE_WINDOW = (5.0, 30.0)
MIN_CONSTANT_LENGTH = 10.0
EARTH_RADIUS = 6378137.0


@dataclass
class FaultSpec:
    """One injected fault on one channel of one sensor."""

    sensor: str
    column: str
    kind: str
    start: float  # seconds from the first sample of the run
    end: float
    magnitude: float  # sigma units (spike amplitude, noise std, or offset)
    sign: int = 1
    spikes: list = field(default_factory=list)  # [(time, width, signed sigma)]


def channel_scale(sensor, column, latitude_deg):
    """Return the factor converting one sigma of ``column`` into its CSV units."""
    sigma = NOMINAL_SIGMA[sensor][column]
    if column == "latitude":
        return math.degrees(sigma / EARTH_RADIUS)
    if column == "longitude":
        return math.degrees(sigma / (EARTH_RADIUS * math.cos(math.radians(latitude_deg))))
    return sigma


def load_run(folder):
    folder = Path(folder)
    return {name: pd.read_csv(folder / f"{name}.csv") for name in NOMINAL_SIGMA}


def add_nominal_noise(frames, rng):
    """Add nominal Gaussian noise to every faultable channel (in place)."""
    latitude = frames["gps"]["latitude"].mean()
    for sensor, columns in NOMINAL_SIGMA.items():
        frame = frames[sensor]
        for column in columns:
            frame[column] = frame[column] + rng.normal(
                0.0, channel_scale(sensor, column, latitude), len(frame))


def random_fault(kind, duration, rng, settle=3.0, noise_window=NOISE_WINDOW):
    """Draw a random FaultSpec of ``kind`` for a run lasting ``duration`` seconds.

    ``noise_window`` is the (min, max) length in seconds of a ``noise`` fault.
    """
    sensor = str(rng.choice(list(NOMINAL_SIGMA)))
    column = str(rng.choice(list(NOMINAL_SIGMA[sensor])))
    if kind == "constant":
        start = rng.uniform(settle, duration - MIN_CONSTANT_LENGTH)
        return FaultSpec(sensor, column, kind, start, duration,
                         rng.uniform(*OFFSET_SIGMA), int(rng.choice([-1, 1])))
    length = min(rng.uniform(*(SHORT_WINDOW if kind == "short" else noise_window)),
                 duration - settle - 1.0)
    start = rng.uniform(settle, duration - length)
    if kind == "noise":
        return FaultSpec(sensor, column, kind, start, start + length,
                         rng.uniform(*NOISE_SIGMA))
    count = rng.integers(SPIKES_PER_WINDOW[0], SPIKES_PER_WINDOW[1] + 1)
    spikes = sorted(
        (float(rng.uniform(start, start + length)),
         int(rng.integers(SPIKE_WIDTH[0], SPIKE_WIDTH[1] + 1)),
         float(rng.choice([-1, 1]) * rng.uniform(*SPIKE_SIGMA)))
        for _ in range(count))
    # Each spike draws its own amplitude; record the largest.
    return FaultSpec(sensor, column, kind, start, start + length,
                     max(abs(spike[2]) for spike in spikes), spikes=spikes)


def apply_fault(frames, spec, rng):
    """Inject ``spec`` into ``frames`` (in place) and return the faulted row mask."""
    frame = frames[spec.sensor]
    scale = channel_scale(spec.sensor, spec.column, frames["gps"]["latitude"].mean())
    t = frame["stamp"].to_numpy() - first_stamp(frames)
    mask = np.zeros(len(frame), dtype=bool)
    values = frame[spec.column].to_numpy(dtype=np.float64).copy()
    if spec.kind == "short":
        for time, width, signed_sigma in spec.spikes:
            index = int(np.searchsorted(t, time))
            rows = slice(index, min(index + width, len(frame)))
            values[rows] += signed_sigma * scale
            mask[rows] = True
    else:
        mask = (t >= spec.start) & (t < spec.end)
        if spec.kind == "noise":
            values[mask] += rng.normal(0.0, spec.magnitude * scale, mask.sum())
        elif spec.kind == "constant":
            values[mask] += spec.sign * spec.magnitude * scale
        else:
            raise ValueError(f"Unknown fault kind '{spec.kind}'")
    frame[spec.column] = values
    return mask


def first_stamp(frames):
    return min(frame["stamp"].iloc[0] for frame in frames.values())


def label_frames(frames, faults):
    """Add per-sample ``fault``, ``fault_type`` and ``fault_column`` labels."""
    for frame in frames.values():
        frame["fault"] = 0
        frame["fault_type"] = "none"
        frame["fault_column"] = ""
    for spec, mask in faults:
        frame = frames[spec.sensor]
        frame.loc[mask, "fault"] = 1
        frame.loc[mask, "fault_type"] = spec.kind
        frame.loc[mask, "fault_column"] = spec.column


def make_variant(run_folder, output_folder, kind, seed, nominal_noise=True,
                 noise_window=NOISE_WINDOW):
    """Write one variant of a clean run with fault ``kind`` (or ``None``)."""
    run_folder, output_folder = Path(run_folder), Path(output_folder)
    rng = np.random.default_rng(seed)
    frames = load_run(run_folder)
    if nominal_noise:
        add_nominal_noise(frames, rng)
    duration = max(f["stamp"].iloc[-1] for f in frames.values()) - first_stamp(frames)
    faults = []
    if kind is not None:
        spec = random_fault(kind, duration, rng, noise_window=noise_window)
        faults.append((spec, apply_fault(frames, spec, rng)))
    label_frames(frames, faults)
    output_folder.mkdir(parents=True, exist_ok=True)
    for name, frame in frames.items():
        frame.to_csv(output_folder / f"{name}.csv", index=False, float_format="%.10g")
    for name in ("truth.csv", "cmd.csv", "metadata.json"):
        shutil.copy(run_folder / name, output_folder / name)
    (output_folder / "faults.json").write_text(json.dumps({
        "source_run": run_folder.name,
        "seed": seed,
        "nominal_noise": nominal_noise,
        "noise_window_s": list(noise_window),
        "nominal_sigma": NOMINAL_SIGMA,
        "faults": [{**asdict(spec), "labelled_samples": int(mask.sum())}
                   for spec, mask in faults],
    }, indent=2))


def main():
    parser = argparse.ArgumentParser(
        description="Inject labelled sensor faults into clean UGV runs.")
    parser.add_argument("--input", default="datasets/ugv_clean")
    parser.add_argument("--output", default="datasets/ugv_faulty")
    parser.add_argument("--types", nargs="+", default=list(FAULT_TYPES), choices=FAULT_TYPES)
    parser.add_argument("--per-type", type=int, default=1,
                        help="Faulty variants per fault type per clean run.")
    parser.add_argument("--normal-variants", type=int, default=1,
                        help="Fault-free variants (nominal noise only) per clean run.")
    parser.add_argument("--count", action="append", default=[], metavar="TYPE=N",
                        help="Variants of one fault type per clean run, overriding "
                             "--per-type (repeatable), e.g. --count noise=3.")
    parser.add_argument("--noise-window", type=float, nargs=2, default=list(NOISE_WINDOW),
                        metavar=("MIN", "MAX"), help="Noise-fault length range in seconds.")
    parser.add_argument("--no-nominal-noise", action="store_true")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    runs = sorted(p for p in Path(args.input).glob("run_*") if (p / "metadata.json").exists())
    if not runs:
        raise SystemExit(f"No completed runs under {args.input}")
    plan = [("normal", None, i) for i in range(args.normal_variants)]
    counts = dict.fromkeys(args.types, args.per_type)
    for item in args.count:
        kind, _, number = item.partition("=")
        if kind not in FAULT_TYPES or not number.isdigit():
            raise SystemExit(f"--count expects TYPE=N with TYPE in {FAULT_TYPES}, got {item!r}")
        counts[kind] = int(number)
    plan += [(kind, kind, i) for kind, count in counts.items() for i in range(count)]
    for run_index, run in enumerate(runs):
        for slot, (name, kind, index) in enumerate(plan):
            seed = args.seed * 1_000_003 + run_index * 1009 + slot
            make_variant(run, Path(args.output) / f"{run.name}_{name}_{index}", kind, seed,
                         nominal_noise=not args.no_nominal_noise,
                         noise_window=tuple(args.noise_window))
    print(f"Wrote {len(runs) * len(plan)} variants from {len(runs)} runs to {args.output}")


if __name__ == "__main__":
    main()
