# Plot training loss curves for the fault-detection runs and save them as PNGs.
#
# The Trainer writes a per-epoch loss history to
#      models/<name>Weights/loss_history.json
# and
#      models/<name>Weights_freq/loss_history.json
# for each model. This script reads those histories and renders:
#      - one loss curve (train vs validation) per model/domain, and
#      - a combined comparison of the three models per domain.
#
# Usage (run from this folder with the project venv):
#      ./bin/python plot_losses.py
#      ./bin/python plot_losses.py --domain frequency
#      ./bin/python plot_losses.py --all-domains --out models/loss_plots

import os
import json
import argparse

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

matplotlib.use("Agg")    # headless: save PNGs without a display

ROOT = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR = os.path.join(ROOT, "models")

# (display name, time-domain weight folder, frequency-domain weight folder)
MODELS = [
      ("Autoencoder", "autoencoderWeights", "autoencoderWeights_freq"),
      ("RNN", "rnnWeights", "rnnWeights_freq"),
      ("BiLSTM", "bilstmWeights", "bilstmWeights_freq"),
]


def weight_folder(base_name, domain):
    return base_name + ("_freq" if domain == "frequency" else "")


def load_history(base_name, domain):
      # Return the loss history dict for a model/domain, or None if missing.
    path = os.path.join(
         MODELS_DIR, weight_folder(base_name, domain), "loss_history.json"
     )
    if not os.path.exists(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def plot_one(path, history, title):
      # Plot a single train/val loss curve to ``path`` (log-scaled).
    train = np.asarray(history["train"], dtype=float)
    val = np.asarray(history.get("val", []), dtype=float)
    epochs = np.arange(1, len(train) + 1)
    val_epochs = np.arange(1, len(val) + 1)

    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(epochs, train, label="train", color="tab:blue")
    if len(val):
        ax.plot(val_epochs, val, label="validation", color="tab:orange")
    ax.set_yscale("log")
    ax.set_ylabel("loss (log scale)")
    ax.set_xlabel("epoch")
    ax.set_title(title)
    ax.grid(True, which="both", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"   wrote {path}")


def plot_domain(domain, out_dir):
      # Render per-model and combined loss curves for a domain.
    os.makedirs(out_dir, exist_ok=True)
    print(f"Domain: {domain}")
    written = 0
    combined_train, combined_val = {}, {}

    for disp, time_folder, freq_folder in MODELS:
        hist = load_history(time_folder, domain)
        if hist is None:
            print(f"      (no loss_history.json for {disp}/{domain}; skipping)")
            continue
        tag = disp.lower()
        per_model = os.path.join(out_dir, f"loss_{tag}_{domain}.png")
        plot_one(per_model, hist, f"{disp} loss ({domain} domain)")
        written += 1
        combined_train[disp] = np.asarray(hist["train"], dtype=float)
        combined_val[disp] = np.asarray(hist.get("val", []), dtype=float)

      # Combined comparison of the three models' training / validation loss.
    if combined_train:
        fig, axes = plt.subplots(1, 2, figsize=(13, 4.5))
        for disp, losses in combined_train.items():
            axes[0].plot(
                  np.arange(1, len(losses) + 1),
                  losses,
                 label=disp,
            )

        axes[0].set_yscale("log")
        axes[0].set_title(f"Training loss ({domain})")
        axes[0].set_xlabel("epoch")
        axes[0].set_ylabel("loss (log scale)")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()

        for disp, losses in combined_val.items():
            axes[1].plot(
                  np.arange(1, len(losses) + 1),
                  losses,
                 label=disp,
            )
            
        axes[1].set_yscale("log")
        axes[1].set_title(f"Validation loss ({domain})")
        axes[1].set_xlabel("epoch")
        axes[1].set_ylabel("loss (log scale)")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        fig.suptitle(f"Loss comparison -- {domain} domain")
        fig.tight_layout()
        combined_path = os.path.join(out_dir, f"loss_comparison_{domain}.png")
        fig.savefig(combined_path, dpi=150)
        plt.close(fig)
        print(f"   wrote {combined_path}")
        written += 1

    if written == 0:
        print(f'   No histories found for domain "{domain}". Train first, e.g.:')
        print(f"        ./bin/python main.py --domain {domain}")
    return written


def main():
    parser = argparse.ArgumentParser(
        description="Plot training loss curves and save them as PNGs."
     )
    parser.add_argument("--domain", choices=["time", "frequency"], default=None,
                        help="Domain to plot. Default: plot whichever "
                                "histories exist.")
    parser.add_argument("--all-domains", action="store_true",
                        help="Plot both time and frequency domains.")
    parser.add_argument("--out", default=None,
                        help="Output directory for PNGs. "
                                "Default: models/loss_plots/<domain>.")
    args = parser.parse_args()

    if args.all_domains:
        domains = ["time", "frequency"]
    elif args.domain is not None:
        domains = [args.domain]
    else:
        domains = ["time", "frequency"]

    any_written = 0
    for domain in domains:
        out_dir = (
            args.out
            if args.out
            else os.path.join(MODELS_DIR, "loss_plots", domain)
        )
        any_written += plot_domain(domain, out_dir)

    if any_written == 0:
        print("No loss plots were written. Run training first, e.g.:")
        print("     ./bin/python main.py --domain time")
    else:
        print(f"\nDone. {any_written} loss plot(s) written.")


if __name__ == "__main__":
    main()
