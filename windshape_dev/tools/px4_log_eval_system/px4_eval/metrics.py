from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np
import pandas as pd

from .io import get_series
from .math_utils import (
    quat_to_euler_deg,
    interp_to,
    unwrap_deg,
    angle_diff_deg,
    robust_rms,
    robust_mae,
    robust_max_abs,
    robust_percentile_abs,
    derivative,
)


@dataclass
class EvalConfig:
    start: float | None = None
    end: float | None = None
    step_threshold_deg: float = 5.0
    settle_band_deg: float = 2.0
    settle_hold_s: float = 0.5


def _slice_df(df: pd.DataFrame, cfg: EvalConfig) -> pd.DataFrame:
    out = df.copy()
    if cfg.start is not None:
        out = out[out["t"] >= cfg.start]
    if cfg.end is not None:
        out = out[out["t"] <= cfg.end]
    if not out.empty:
        out = out.copy()
        out["t"] = out["t"] - out["t"].iloc[0]
    return out.reset_index(drop=True)


def extract_attitude_euler(topics: dict[str, pd.DataFrame], cfg: EvalConfig) -> dict[str, pd.DataFrame]:
    result: dict[str, pd.DataFrame] = {}

    att = topics.get("vehicle_attitude")
    if att is not None:
        att = _slice_df(att, cfg)
        q_cols = [
            get_series(att, ["q[0]", "q_0", "q0"]),
            get_series(att, ["q[1]", "q_1", "q1"]),
            get_series(att, ["q[2]", "q_2", "q2"]),
            get_series(att, ["q[3]", "q_3", "q3"]),
        ]
        if all(s is not None for s in q_cols):
            roll, pitch, yaw = quat_to_euler_deg(*q_cols)
            result["actual"] = pd.DataFrame({
                "t": att["t"].to_numpy(),
                "roll": roll,
                "pitch": pitch,
                "yaw": unwrap_deg(yaw),
            })

    sp = topics.get("vehicle_attitude_setpoint")
    if sp is not None:
        sp = _slice_df(sp, cfg)
        q_cols = [
            get_series(sp, ["q_d[0]", "qd[0]", "q_d_0", "q_d0", "q_d.0"]),
            get_series(sp, ["q_d[1]", "qd[1]", "q_d_1", "q_d1", "q_d.1"]),
            get_series(sp, ["q_d[2]", "qd[2]", "q_d_2", "q_d2", "q_d.2"]),
            get_series(sp, ["q_d[3]", "qd[3]", "q_d_3", "q_d3", "q_d.3"]),
        ]

        if all(s is not None for s in q_cols):
            roll, pitch, yaw = quat_to_euler_deg(*q_cols)
            result["setpoint"] = pd.DataFrame({
                "t": sp["t"].to_numpy(),
                "roll": roll,
                "pitch": pitch,
                "yaw": unwrap_deg(yaw),
            })

    if "actual" in result and "setpoint" in result:
        t = result["actual"]["t"].to_numpy()
        aligned = {"t": t}

        for axis in ["roll", "pitch", "yaw"]:
            aligned[f"{axis}_actual"] = result["actual"][axis].to_numpy()
            aligned[f"{axis}_setpoint"] = interp_to(
                result["setpoint"]["t"].to_numpy(),
                result["setpoint"][axis].to_numpy(),
                t,
            )
            if axis == "yaw":
                aligned[f"{axis}_error"] = angle_diff_deg(
                    aligned[f"{axis}_actual"],
                    aligned[f"{axis}_setpoint"],
                )
            else:
                aligned[f"{axis}_error"] = aligned[f"{axis}_actual"] - aligned[f"{axis}_setpoint"]

        result["aligned"] = pd.DataFrame(aligned).dropna().reset_index(drop=True)

    return result


def extract_rate(topics: dict[str, pd.DataFrame], cfg: EvalConfig) -> pd.DataFrame | None:
    df = topics.get("vehicle_angular_velocity")
    if df is None:
        return None

    df = _slice_df(df, cfg)

    cols = [
        get_series(df, ["xyz[0]", "xyz_0", "x", "rollspeed", "p"]),
        get_series(df, ["xyz[1]", "xyz_1", "y", "pitchspeed", "q"]),
        get_series(df, ["xyz[2]", "xyz_2", "z", "yawspeed", "r"]),
    ]

    if not all(s is not None for s in cols):
        return None

    return pd.DataFrame({
        "t": df["t"].to_numpy(),
        "p_deg_s": np.rad2deg(cols[0].to_numpy(dtype=float)),
        "q_deg_s": np.rad2deg(cols[1].to_numpy(dtype=float)),
        "r_deg_s": np.rad2deg(cols[2].to_numpy(dtype=float)),
    })


def extract_control(topics: dict[str, pd.DataFrame], cfg: EvalConfig) -> dict[str, pd.DataFrame]:
    result: dict[str, pd.DataFrame] = {}

    torque = topics.get("vehicle_torque_setpoint")
    if torque is not None:
        torque = _slice_df(torque, cfg)
        cols = [
            get_series(torque, ["xyz[0]", "xyz_0", "x", "roll"]),
            get_series(torque, ["xyz[1]", "xyz_1", "y", "pitch"]),
            get_series(torque, ["xyz[2]", "xyz_2", "z", "yaw"]),
        ]
        if all(s is not None for s in cols):
            result["torque"] = pd.DataFrame({
                "t": torque["t"].to_numpy(),
                "roll": cols[0].to_numpy(dtype=float),
                "pitch": cols[1].to_numpy(dtype=float),
                "yaw": cols[2].to_numpy(dtype=float),
            })

    thrust = topics.get("vehicle_thrust_setpoint")
    if thrust is not None:
        thrust = _slice_df(thrust, cfg)
        z = get_series(thrust, ["xyz[2]", "xyz_2", "z"])
        if z is not None:
            result["thrust"] = pd.DataFrame({
                "t": thrust["t"].to_numpy(),
                "thrust": -z.to_numpy(dtype=float),
            })

    return result


def extract_eso(topics: dict[str, pd.DataFrame], cfg: EvalConfig) -> pd.DataFrame | None:
    df = topics.get("rate_ctrl_status")
    if df is None:
        return None

    df = _slice_df(df, cfg)
    cols = [
        get_series(df, ["rollspeed_integ", "roll_rate_integ"]),
        get_series(df, ["pitchspeed_integ", "pitch_rate_integ"]),
        get_series(df, ["yawspeed_integ", "yaw_rate_integ"]),
    ]

    if not all(s is not None for s in cols):
        return None

    return pd.DataFrame({
        "t": df["t"].to_numpy(),
        "roll": cols[0].to_numpy(dtype=float),
        "pitch": cols[1].to_numpy(dtype=float),
        "yaw": cols[2].to_numpy(dtype=float),
    })


def detect_steps_and_settling(aligned: pd.DataFrame, axis: str, threshold_deg: float, band_deg: float, hold_s: float):
    t = aligned["t"].to_numpy()
    sp = aligned[f"{axis}_setpoint"].to_numpy()
    err = np.abs(aligned[f"{axis}_error"].to_numpy())

    if len(t) < 3:
        return []

    sp_u = unwrap_deg(sp)
    dsp = np.diff(sp_u, prepend=sp_u[0])
    idxs = np.where(np.abs(dsp) >= threshold_deg)[0]
    events = []

    min_gap = max(1, int(0.5 / np.nanmedian(np.diff(t)))) if len(t) > 2 else 1
    selected = []
    for idx in idxs:
        if not selected or idx - selected[-1] > min_gap:
            selected.append(idx)

    for idx in selected:
        t0 = t[idx]
        final_sp = sp_u[min(idx + min_gap, len(sp_u)-1)]
        step_size = float(abs(sp_u[idx] - sp_u[max(idx-1, 0)]))
        window = np.where((t >= t0) & (t <= t0 + 10.0))[0]
        settling = np.nan

        if window.size:
            dt = np.nanmedian(np.diff(t)) if len(t) > 2 else 0.02
            hold_n = max(1, int(hold_s / max(dt, 1e-3)))
            inside = err[window] <= band_deg

            for j in range(0, len(window) - hold_n):
                if inside[j:j + hold_n].all():
                    settling = float(t[window[j]] - t0)
                    break

        events.append({
            "axis": axis,
            "time_s": float(t0),
            "step_deg": step_size,
            "settling_time_s": settling,
            "final_setpoint_deg": float(final_sp),
        })

    return events


def compute_metrics(topics: dict[str, pd.DataFrame], cfg: EvalConfig) -> dict:
    attitude = extract_attitude_euler(topics, cfg)
    rates = extract_rate(topics, cfg)
    control = extract_control(topics, cfg)
    eso = extract_eso(topics, cfg)

    metrics: dict = {
        "available_topics": sorted(list(topics.keys())),
        "attitude": {},
        "rates": {},
        "control": {},
        "eso": {},
        "settling_events": [],
        "scores": {},
    }

    if "aligned" in attitude:
        aligned = attitude["aligned"]
        for axis in ["roll", "pitch", "yaw"]:
            err = aligned[f"{axis}_error"].to_numpy()
            metrics["attitude"][axis] = {
                "rmse_deg": robust_rms(err),
                "mae_deg": robust_mae(err),
                "max_abs_deg": robust_max_abs(err),
                "p95_abs_deg": robust_percentile_abs(err, 95),
                "bias_deg": float(np.nanmean(err)) if np.isfinite(err).any() else float("nan"),
            }
            metrics["settling_events"].extend(
                detect_steps_and_settling(
                    aligned,
                    axis,
                    cfg.step_threshold_deg,
                    cfg.settle_band_deg,
                    cfg.settle_hold_s,
                )
            )

        if len(aligned) > 2:
            dt = np.diff(aligned["t"].to_numpy())
            metrics["time"] = {
                "duration_s": float(aligned["t"].iloc[-1] - aligned["t"].iloc[0]),
                "median_dt_s": float(np.nanmedian(dt)),
                "sample_rate_hz": float(1.0 / np.nanmedian(dt)) if np.nanmedian(dt) > 0 else float("nan"),
            }

    if rates is not None:
        for axis in ["p_deg_s", "q_deg_s", "r_deg_s"]:
            x = rates[axis].to_numpy()
            metrics["rates"][axis] = {
                "rms_deg_s": robust_rms(x),
                "max_abs_deg_s": robust_max_abs(x),
                "p95_abs_deg_s": robust_percentile_abs(x, 95),
            }

    if "torque" in control:
        tq = control["torque"]
        sat_masks = []
        for axis in ["roll", "pitch", "yaw"]:
            x = tq[axis].to_numpy()
            dx = derivative(tq["t"].to_numpy(), x)
            sat = np.abs(x) >= 0.98
            sat_masks.append(sat)
            metrics["control"][f"torque_{axis}"] = {
                "rms": robust_rms(x),
                "max_abs": robust_max_abs(x),
                "sat_ratio": float(np.nanmean(sat)) if len(sat) else float("nan"),
                "derivative_rms": robust_rms(dx),
            }

        if sat_masks:
            metrics["control"]["torque_any_sat_ratio"] = float(np.nanmean(np.logical_or.reduce(sat_masks)))

    if "thrust" in control:
        th = control["thrust"]
        x = th["thrust"].to_numpy()
        sat = (x <= 0.02) | (x >= 0.98)
        dx = derivative(th["t"].to_numpy(), x)
        metrics["control"]["thrust"] = {
            "mean": float(np.nanmean(x)) if np.isfinite(x).any() else float("nan"),
            "rms": robust_rms(x),
            "min": float(np.nanmin(x)) if np.isfinite(x).any() else float("nan"),
            "max": float(np.nanmax(x)) if np.isfinite(x).any() else float("nan"),
            "sat_ratio": float(np.nanmean(sat)) if len(sat) else float("nan"),
            "derivative_rms": robust_rms(dx),
        }

    if eso is not None:
        for axis in ["roll", "pitch", "yaw"]:
            x = eso[axis].to_numpy()
            metrics["eso"][axis] = {
                "rms": robust_rms(x),
                "max_abs": robust_max_abs(x),
                "p95_abs": robust_percentile_abs(x, 95),
            }

    metrics["scores"] = compute_scores(metrics)
    return metrics


def _finite_or(default: float, *values) -> float:
    vals = [float(v) for v in values if v is not None and np.isfinite(float(v))]
    if not vals:
        return default
    return float(np.mean(vals))


def compute_scores(metrics: dict) -> dict:
    att = metrics.get("attitude", {})
    mean_rmse = _finite_or(
        20.0,
        *(att.get(axis, {}).get("rmse_deg", np.nan) for axis in ["roll", "pitch", "yaw"]),
    )
    tracking = float(np.clip(100.0 * (1.0 - mean_rmse / 20.0), 0.0, 100.0))

    rates = metrics.get("rates", {})
    mean_rate_rms = _finite_or(
        200.0,
        *(rates.get(axis, {}).get("rms_deg_s", np.nan) for axis in ["p_deg_s", "q_deg_s", "r_deg_s"]),
    )
    stability = float(np.clip(100.0 * (1.0 - mean_rate_rms / 250.0), 0.0, 100.0))

    control = metrics.get("control", {})
    sat_ratio = _finite_or(
        1.0,
        control.get("torque_any_sat_ratio", np.nan),
        control.get("thrust", {}).get("sat_ratio", np.nan) if isinstance(control.get("thrust"), dict) else np.nan,
    )
    control_margin = float(np.clip(100.0 * (1.0 - 2.0 * sat_ratio), 0.0, 100.0))

    derivs = []
    for key, val in control.items():
        if isinstance(val, dict) and "derivative_rms" in val:
            derivs.append(val["derivative_rms"])
    mean_deriv = _finite_or(20.0, *derivs)
    smoothness = float(np.clip(100.0 * (1.0 - mean_deriv / 20.0), 0.0, 100.0))

    eso = metrics.get("eso", {})
    mean_eso_rms = _finite_or(
        20.0,
        *(eso.get(axis, {}).get("rms", np.nan) for axis in ["roll", "pitch", "yaw"]),
    )
    eso_sanity = float(np.clip(100.0 * (1.0 - mean_eso_rms / 20.0), 0.0, 100.0))

    overall = (
        0.35 * tracking
        + 0.20 * stability
        + 0.20 * control_margin
        + 0.15 * smoothness
        + 0.10 * eso_sanity
    )

    return {
        "tracking": tracking,
        "stability": stability,
        "control_margin": control_margin,
        "smoothness": smoothness,
        "eso_sanity": eso_sanity,
        "overall": float(overall),
        "notes": "Scores are heuristic and intended for controller comparison, not certification.",
    }


def metrics_to_rows(metrics: dict) -> list[dict]:
    rows = []

    def walk(prefix, obj):
        if isinstance(obj, dict):
            for k, v in obj.items():
                walk(prefix + [str(k)], v)
        elif isinstance(obj, list):
            rows.append({"metric": ".".join(prefix), "value": json.dumps(obj, ensure_ascii=False)})
        else:
            rows.append({"metric": ".".join(prefix), "value": obj})

    walk([], metrics)
    return rows


def save_metrics(metrics: dict, out_dir: str | Path):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    with open(out / "metrics_summary.json", "w", encoding="utf-8") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)

    pd.DataFrame(metrics_to_rows(metrics)).to_csv(out / "metrics_summary.csv", index=False)
