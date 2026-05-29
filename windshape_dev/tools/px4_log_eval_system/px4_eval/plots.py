from __future__ import annotations

from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from .metrics import extract_attitude_euler, extract_rate, extract_control, extract_eso, EvalConfig


def _save(fig, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def _plot_series(t, series: dict[str, np.ndarray], title: str, ylabel: str, path: Path):
    fig, ax = plt.subplots(figsize=(9, 4.8))
    for label, y in series.items():
        ax.plot(t, y, label=label, linewidth=1.2)
    ax.set_title(title)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.35)
    ax.legend(loc="best")
    _save(fig, path)


def plot_attitude(attitude: dict[str, pd.DataFrame], out_dir: Path):
    if "aligned" not in attitude:
        return

    df = attitude["aligned"]

    for i, axis in enumerate(["roll", "pitch", "yaw"], start=1):
        _plot_series(
            df["t"],
            {
                "actual": df[f"{axis}_actual"],
                "setpoint": df[f"{axis}_setpoint"],
            },
            f"{axis.upper()} attitude tracking",
            "Angle [deg]",
            out_dir / f"{i:02d}_{axis}_tracking.png",
        )

    for i, axis in enumerate(["roll", "pitch", "yaw"], start=4):
        _plot_series(
            df["t"],
            {f"{axis} error": df[f"{axis}_error"]},
            f"{axis.upper()} tracking error",
            "Error [deg]",
            out_dir / f"{i:02d}_{axis}_error.png",
        )


def plot_rates(rates: pd.DataFrame | None, out_dir: Path):
    if rates is None:
        return

    for idx, axis in enumerate(["p_deg_s", "q_deg_s", "r_deg_s"], start=7):
        name = axis.split("_")[0]
        _plot_series(
            rates["t"],
            {name: rates[axis]},
            f"Body rate {name}",
            "Angular velocity [deg/s]",
            out_dir / f"{idx:02d}_rate_{name}.png",
        )


def plot_control(control: dict[str, pd.DataFrame], out_dir: Path):
    tq = control.get("torque")
    if tq is not None:
        for idx, axis in enumerate(["roll", "pitch", "yaw"], start=10):
            _plot_series(
                tq["t"],
                {f"torque {axis}": tq[axis]},
                f"Normalized torque setpoint - {axis}",
                "Normalized torque [-]",
                out_dir / f"{idx:02d}_torque_{axis}.png",
            )

    th = control.get("thrust")
    if th is not None:
        _plot_series(
            th["t"],
            {"thrust": th["thrust"]},
            "Normalized thrust setpoint",
            "Normalized thrust [-]",
            out_dir / "13_thrust.png",
        )


def plot_eso(eso: pd.DataFrame | None, out_dir: Path):
    if eso is None:
        return

    for idx, axis in enumerate(["roll", "pitch", "yaw"], start=14):
        _plot_series(
            eso["t"],
            {f"ESO {axis}": eso[axis]},
            f"ESO disturbance estimation - {axis}",
            "Estimated disturbance",
            out_dir / f"{idx:02d}_eso_{axis}.png",
        )


def plot_scores(metrics: dict, out_dir: Path):
    scores = metrics.get("scores", {})
    if not scores:
        return

    keys = ["tracking", "stability", "control_margin", "smoothness", "eso_sanity", "overall"]
    values = [scores.get(k, np.nan) for k in keys]

    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.bar(keys, values)
    ax.set_ylim(0, 100)
    ax.set_title("Controller evaluation score")
    ax.set_ylabel("Score [0-100]")
    ax.grid(True, axis="y", alpha=0.35)
    for idx, v in enumerate(values):
        if np.isfinite(v):
            ax.text(idx, v + 1.5, f"{v:.1f}", ha="center", va="bottom", fontsize=9)
    _save(fig, out_dir / "17_score_summary.png")

    att = metrics.get("attitude", {})
    labels = ["roll", "pitch", "yaw"]
    rmse = [att.get(axis, {}).get("rmse_deg", np.nan) for axis in labels]

    if any(np.isfinite(rmse)):
        fig, ax = plt.subplots(figsize=(7, 4.8))
        ax.bar(labels, rmse)
        ax.set_title("Attitude RMSE by axis")
        ax.set_ylabel("RMSE [deg]")
        ax.grid(True, axis="y", alpha=0.35)
        for idx, v in enumerate(rmse):
            if np.isfinite(v):
                ax.text(idx, v + 0.05, f"{v:.2f}", ha="center", va="bottom", fontsize=9)
        _save(fig, out_dir / "18_attitude_rmse.png")


def make_plots(topics: dict[str, pd.DataFrame], metrics: dict, cfg: EvalConfig, out_dir: str | Path):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    attitude = extract_attitude_euler(topics, cfg)
    rates = extract_rate(topics, cfg)
    control = extract_control(topics, cfg)
    eso = extract_eso(topics, cfg)

    plot_attitude(attitude, out)
    plot_rates(rates, out)
    plot_control(control, out)
    plot_eso(eso, out)
    plot_scores(metrics, out)
