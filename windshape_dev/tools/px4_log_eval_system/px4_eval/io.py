from __future__ import annotations

from collections import OrderedDict
from pathlib import Path
import re
import pandas as pd

from .math_utils import to_seconds


TOPIC_ALIASES = OrderedDict([
    ("vehicle_attitude_setpoint", ["vehicle_attitude_setpoint", "attitude_setpoint"]),
    ("vehicle_attitude", ["vehicle_attitude"]),
    ("vehicle_angular_velocity", ["vehicle_angular_velocity", "angular_velocity"]),
    ("vehicle_torque_setpoint", ["vehicle_torque_setpoint", "torque_setpoint"]),
    ("vehicle_thrust_setpoint", ["vehicle_thrust_setpoint", "thrust_setpoint"]),
    ("rate_ctrl_status", ["rate_ctrl_status"]),
    ("vehicle_local_position", ["vehicle_local_position", "local_position"]),
    ("manual_control_setpoint", ["manual_control_setpoint"]),
    ("vehicle_control_mode", ["vehicle_control_mode"]),
    ("vehicle_status", ["vehicle_status"]),
    ("vehicle_land_detected", ["vehicle_land_detected", "land_detected"]),
    ("battery_status", ["battery_status"]),
])


def canonical_topic_name(name: str) -> str:
    lower = name.lower()
    lower = re.sub(r"\.csv$", "", lower)

    for topic, aliases in TOPIC_ALIASES.items():
        if any(alias in lower for alias in aliases):
            return topic

    return lower


def standardize_time(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()

    timestamp_candidates = [
        "timestamp_sample",
        "timestamp",
        "time_us",
        "time_usec",
        "time",
        "t",
        "TimeUS",
    ]

    selected = None
    for c in timestamp_candidates:
        if c in df.columns:
            selected = c
            break

    if selected is None:
        # Some CSV exporters create the timestamp as the first unnamed column.
        if df.columns.size and str(df.columns[0]).lower().startswith("unnamed"):
            selected = df.columns[0]

    if selected is None:
        raise ValueError("Cannot find a timestamp column. Expected timestamp/timestamp_sample/time/t.")

    df["t"] = to_seconds(df[selected])
    return df.sort_values("t").reset_index(drop=True)


def read_csv_file(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    return standardize_time(df)


def load_ulg(path: Path) -> dict[str, pd.DataFrame]:
    try:
        from pyulog import ULog
    except Exception as exc:
        raise RuntimeError(
            "Reading .ulg requires pyulog. Install it with: pip install pyulog"
        ) from exc

    ulog = ULog(str(path))
    topics: dict[str, pd.DataFrame] = {}

    for data in ulog.data_list:
        raw_name = data.name if data.multi_id == 0 else f"{data.name}_{data.multi_id}"
        name = canonical_topic_name(raw_name)
        df = pd.DataFrame(data.data)

        if df.empty:
            continue

        try:
            df = standardize_time(df)
        except ValueError:
            continue

        # Keep the first matching instance by default.
        topics.setdefault(name, df)

    return topics


def load_csv_folder(path: Path) -> dict[str, pd.DataFrame]:
    topics: dict[str, pd.DataFrame] = {}

    for csv_path in sorted(path.glob("*.csv")):
        try:
            df = read_csv_file(csv_path)
        except Exception:
            continue

        topic = canonical_topic_name(csv_path.stem)
        topics.setdefault(topic, df)

    return topics


def split_merged_csv(path: Path) -> dict[str, pd.DataFrame]:
    """Try to parse one merged CSV whose columns are prefixed by topic names."""
    df = pd.read_csv(path)
    try:
        df = standardize_time(df)
    except Exception:
        pass

    topics: dict[str, pd.DataFrame] = {}

    for topic, aliases in TOPIC_ALIASES.items():
        cols = ["t"] if "t" in df.columns else []
        for col in df.columns:
            low = col.lower()
            if any(alias in low for alias in aliases):
                cols.append(col)

        if len(cols) > 1:
            sub = df[cols].copy()
            rename = {}
            for col in sub.columns:
                new = col
                for alias in aliases:
                    new = re.sub(alias + r"[._/:-]*", "", new, flags=re.IGNORECASE)
                new = new.strip("._/:-")
                rename[col] = new or col
            sub = sub.rename(columns=rename)
            topics[topic] = standardize_time(sub) if "t" not in sub.columns else sub

    if not topics:
        topic = canonical_topic_name(path.stem)
        topics[topic] = read_csv_file(path)

    return topics


def load_log(path_like: str | Path) -> dict[str, pd.DataFrame]:
    path = Path(path_like)

    if not path.exists():
        raise FileNotFoundError(f"Log path does not exist: {path}")

    if path.is_dir():
        topics = load_csv_folder(path)
    elif path.suffix.lower() == ".ulg":
        topics = load_ulg(path)
    elif path.suffix.lower() == ".csv":
        topics = split_merged_csv(path)
    else:
        raise ValueError("Unsupported input. Use .ulg, .csv, or a folder of CSV files.")

    if not topics:
        raise RuntimeError("No usable topics found in the log.")

    return topics


def require_topic(topics: dict[str, pd.DataFrame], topic: str) -> pd.DataFrame | None:
    return topics.get(topic)


def find_col(df: pd.DataFrame, candidates: list[str]) -> str | None:
    lower_map = {str(c).lower(): c for c in df.columns}

    for cand in candidates:
        if cand.lower() in lower_map:
            return lower_map[cand.lower()]

    # Normalize q[0], q_0, q.0, q/0 style.
    def norm(s: str) -> str:
        return re.sub(r"[^a-z0-9]+", "", s.lower())

    norm_map = {norm(str(c)): c for c in df.columns}
    for cand in candidates:
        key = norm(cand)
        if key in norm_map:
            return norm_map[key]

    return None


def get_series(df: pd.DataFrame, candidates: list[str]):
    c = find_col(df, candidates)
    if c is None:
        return None
    return df[c]
