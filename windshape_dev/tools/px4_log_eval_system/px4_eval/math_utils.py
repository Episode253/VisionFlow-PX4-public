from __future__ import annotations

import numpy as np
import pandas as pd


def to_seconds(ts: pd.Series | np.ndarray) -> np.ndarray:
    """Convert PX4 timestamp-like values to seconds."""
    arr = np.asarray(ts, dtype=float)
    arr = arr - np.nanmin(arr)

    # PX4 ULog timestamps are usually in microseconds.
    max_val = np.nanmax(arr) if arr.size else 0.0

    if max_val > 1e12:      # nanoseconds
        return arr / 1e9
    if max_val > 1e5:       # microseconds
        return arr / 1e6
    return arr              # already seconds


def unwrap_deg(x: np.ndarray) -> np.ndarray:
    return np.rad2deg(np.unwrap(np.deg2rad(np.asarray(x, dtype=float))))


def angle_diff_deg(actual: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Return wrapped angle difference actual-target in [-180, 180)."""
    return (np.asarray(actual) - np.asarray(target) + 180.0) % 360.0 - 180.0


def quat_to_euler_deg(q0, q1, q2, q3):
    """PX4 quaternion order is [w, x, y, z]. Returns roll, pitch, yaw in degrees."""
    w = np.asarray(q0, dtype=float)
    x = np.asarray(q1, dtype=float)
    y = np.asarray(q2, dtype=float)
    z = np.asarray(q3, dtype=float)

    norm = np.sqrt(w*w + x*x + y*y + z*z)
    norm = np.where(norm > 0, norm, np.nan)

    w, x, y, z = w/norm, x/norm, y/norm, z/norm

    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w*y - z*x)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))

    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)


def interp_to(t_src: np.ndarray, y_src: np.ndarray, t_dst: np.ndarray) -> np.ndarray:
    """1-D interpolation with NaN handling."""
    t_src = np.asarray(t_src, dtype=float)
    y_src = np.asarray(y_src, dtype=float)
    t_dst = np.asarray(t_dst, dtype=float)

    mask = np.isfinite(t_src) & np.isfinite(y_src)
    if mask.sum() < 2:
        return np.full_like(t_dst, np.nan, dtype=float)

    t = t_src[mask]
    y = y_src[mask]
    order = np.argsort(t)
    t = t[order]
    y = y[order]

    _, unique_idx = np.unique(t, return_index=True)
    t = t[unique_idx]
    y = y[unique_idx]

    if len(t) < 2:
        return np.full_like(t_dst, np.nan, dtype=float)

    return np.interp(t_dst, t, y, left=np.nan, right=np.nan)


def derivative(t: np.ndarray, y: np.ndarray) -> np.ndarray:
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    if len(t) < 3:
        return np.full_like(y, np.nan, dtype=float)

    dt = np.gradient(t)
    dy = np.gradient(y)
    out = np.divide(dy, dt, out=np.full_like(dy, np.nan, dtype=float), where=np.abs(dt) > 1e-9)
    return out


def robust_rms(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(x*x)))


def robust_mae(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return float("nan")
    return float(np.mean(np.abs(x)))


def robust_percentile_abs(x: np.ndarray, p: float) -> float:
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return float("nan")
    return float(np.percentile(np.abs(x), p))


def robust_max_abs(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return float("nan")
    return float(np.max(np.abs(x)))
