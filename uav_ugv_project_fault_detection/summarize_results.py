# Print markdown tables of the UGV scores that main.py saves in models/results/.
#
#     ./bin/python summarize_results.py                      # balanced test split
#     ./bin/python summarize_results.py --split test         # original test split
#     ./bin/python summarize_results.py --per-channel bilstmWeights_ugv_mv bilstmWeights_ugv_paired_long
#
# Window F1 is computed on each layout's own windows, so it is comparable within a
# layout only; event isolation recall and clean time flagged compare layouts
# (see ugv_eval.py).

import argparse
import json
import os

RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models", "results")
CONFIGS = [("ugv_paired", "channel"), ("ugv_paired_long", "channel + long"),
           ("ugv_mv", "multivariate + long"),
           ("ugv_mv_balanced", "multivariate + long + balanced"),
           ("ugv_mv_w100", "multivariate + long + balanced, AE 100 steps"),
           ("ugv_mv_w50", "multivariate + long + balanced, AE 50 steps"),
           ("ugv_mv_w100_mirror", "multivariate + long + balanced, AE 100 steps, mirrored"),
           ("ugv_mv_w50_mirror", "multivariate + long + balanced, AE 50 steps, mirrored"),
           ("ugv_mv_specialists", "multivariate + long + balanced, per-channel specialists")]
MODELS = [("bilstm", "BiLSTM"), ("gru", "GRU"), ("rnn", "RNN"), ("autoencoder", "Autoencoder"),
          ("ensemble", "Ensemble"), ("vote", "Majority vote")]
KINDS = ("short", "noise", "constant")


def load(name):
    path = os.path.join(RESULTS_DIR, name + ".json")
    if not os.path.exists(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def overview(split):
    rows = ["| Model | Layout | Domain | Window F1 | P | R | Event recall | Short | Noise | "
            "Constant | Clean time flagged |",
            "| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    counted = None
    for model, label in MODELS:
        for config, layout in CONFIGS:
            for domain, suffix in (("time", ""), ("frequency", "_freq")):
                result = load(f"{model}Weights_{config}{suffix}")
                if result is None or split not in result:
                    continue
                r = counted = result[split]
                w = r["windows"]
                by_type = " | ".join(f"{r['event_recall_by_type'][k]:.2f}" for k in KINDS)
                rows.append(f"| {label} | {layout} | {domain} | {w['f1']:.3f} | "
                            f"{w['precision']:.3f} | {w['recall']:.3f} | "
                            f"{r['event_recall']:.3f} | {by_type} | "
                            f"{100 * r['clean_time_flagged']:.2f}% |")
    if counted is not None:
        rows.append(f"\n{split}: {counted['events']} fault events ("
                    + ", ".join(f"{k} {counted['events_by_type'][k]}" for k in KINDS) + ")")
    return "\n".join(rows)


def per_channel(names, split):
    results = {name: load(name)[split] for name in names}
    header = "| Channel | " + " | ".join(f"{n} event recall | {n} clean time flagged"
                                         for n in names) + " |"
    rows = [header, "| --- |" + " ---: | ---: |" * len(names)]
    channels = next(iter(results.values()))["per_channel"]
    for channel in channels:
        cells = []
        for name in names:
            c = results[name]["per_channel"][channel]
            cells.append(f"{c['event_recall']:.2f} ({c['events']}) | "
                         f"{100 * c['clean_time_flagged']:.2f}%")
        rows.append(f"| {channel} | " + " | ".join(cells) + " |")
    return "\n".join(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--split", default="test_extra", choices=["val", "test", "test_extra"])
    parser.add_argument("--per-channel", nargs="+", metavar="RESULT",
                        help="Result names (models/results/<name>.json) to compare per channel.")
    args = parser.parse_args()
    print(per_channel(args.per_channel, args.split) if args.per_channel
          else overview(args.split))


if __name__ == "__main__":
    main()
