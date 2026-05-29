from __future__ import annotations

import argparse
from pathlib import Path

from .io import load_log
from .metrics import EvalConfig, compute_metrics, save_metrics
from .plots import make_plots


def parse_args():
    p = argparse.ArgumentParser(description="Evaluate PX4 attitude-control logs and output assessment figures.")
    p.add_argument("--log", required=True, help="Input .ulg, merged .csv, or a folder of ulog2csv CSV files.")
    p.add_argument("--out", required=True, help="Output directory.")
    p.add_argument("--start", type=float, default=None, help="Start time in seconds relative to log start.")
    p.add_argument("--end", type=float, default=None, help="End time in seconds relative to log start.")
    p.add_argument("--step-threshold-deg", type=float, default=5.0, help="Step detection threshold in deg.")
    p.add_argument("--settle-band-deg", type=float, default=2.0, help="Settling band in deg.")
    p.add_argument("--settle-hold-s", type=float, default=0.5, help="Required time inside settling band.")
    p.add_argument("--no-plots", action="store_true", help="Only save metrics; do not generate PNG figures.")
    return p.parse_args()


def main():
    args = parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    cfg = EvalConfig(
        start=args.start,
        end=args.end,
        step_threshold_deg=args.step_threshold_deg,
        settle_band_deg=args.settle_band_deg,
        settle_hold_s=args.settle_hold_s,
    )

    topics = load_log(args.log)
    metrics = compute_metrics(topics, cfg)
    save_metrics(metrics, out)

    if not args.no_plots:
        make_plots(topics, metrics, cfg, out)

    print(f"Evaluation finished. Output directory: {out}")
    print(f"Overall score: {metrics.get('scores', {}).get('overall', float('nan')):.2f}")


if __name__ == "__main__":
    main()
