"""turns recorded ros2 bags into the normalized csv and scalers that bc and sac train on."""
from __future__ import annotations

import os
import sys
import subprocess
import tempfile
import pandas as pd
import numpy as np
import joblib
from sklearn.preprocessing import MinMaxScaler

# same module the deploy nodes import, so the lidar cap cannot drift between train and deploy
try:
    from learned_control.preprocessing.scan import MAX_RANGE_M
except ImportError:
    from scan import MAX_RANGE_M

BAG_PATHS = [
    ("data/gap_following_data", 24),
    ("data/gap_following_data_v2", 24),
    ("data/gap_following_data_v3", 148),
]
OUTPUT_DIR = "processed"
# column slicing matches the linspace downsample the nodes use, since 1080/180 is exactly 6
LIDAR_STEP = 6
MIN_SPEED_MPS = 0.05
MAX_STEER_RAD = 0.5


def bag_to_df(bag_path) -> pd.DataFrame:
    """run extract_dataset.py on a bag and load the csv it writes."""
    with tempfile.NamedTemporaryFile(suffix=".csv", delete=False) as f:
        tmp_path = f.name
    try:
        subprocess.run(
            [sys.executable, "preprocessing/extract_dataset.py", "--bag", bag_path, "--output", tmp_path],
            check=True
        )
        df = pd.read_csv(tmp_path)
    finally:
        os.remove(tmp_path)
    return df


def label_laps(df, session_id, lap_duration_sec) -> pd.DataFrame:
    """tag each row with a lap id from elapsed time and drop the trailing partial lap."""
    df = df.sort_values("timestamp").reset_index(drop=True)
    lap_duration_ns = lap_duration_sec * 1_000_000_000
    start_ts = df["timestamp"].iloc[0]
    df = df.copy()
    df["lap_id"] = ((df["timestamp"] - start_ts) // lap_duration_ns).astype(int)
    df["session"] = session_id
    if df["lap_id"].max() > 0:
        df = df[df["lap_id"] < df["lap_id"].max()].reset_index(drop=True)
    return df


def clean(df) -> pd.DataFrame:
    """cap lidar, fill non-finite rays, clip steering, and drop stationary rows."""
    lidar_cols = [c for c in df.columns if c.startswith("lidar_")]
    df = df.copy()
    df[lidar_cols] = (
        df[lidar_cols]
        .replace([np.inf, -np.inf], np.nan)
        .clip(0, MAX_RANGE_M)
        .fillna(MAX_RANGE_M)
    )
    # raw bags carry the occasional +/- pi steering value from a bad reading
    df["steering_angle"] = df["steering_angle"].clip(-MAX_STEER_RAD, MAX_STEER_RAD)
    df = df[df["odom_vx"].abs() > MIN_SPEED_MPS].reset_index(drop=True)
    return df


def downsample_lidar(df) -> pd.DataFrame:
    """keep every LIDAR_STEP-th lidar column plus the metadata columns."""
    lidar_cols = [c for c in df.columns if c.startswith("lidar_")]
    keep = lidar_cols[::LIDAR_STEP]
    meta = ["timestamp", "steering_angle", "speed", "odom_vx", "lap_id", "session"]
    return df[meta + keep]


def augment(df) -> pd.DataFrame:
    """double the rows by mirroring each scan and negating its steering."""
    lidar_cols = [c for c in df.columns if c.startswith("lidar_")]
    mirrored = df.copy()
    mirrored[lidar_cols] = df[lidar_cols].values[:, ::-1]
    mirrored["steering_angle"] = -df["steering_angle"]
    return pd.concat([df, mirrored], ignore_index=True)


def normalize(df) -> tuple[pd.DataFrame, MinMaxScaler, MinMaxScaler]:
    """fit min-max scalers and map lidar and action columns into [0, 1]."""
    lidar_cols = [c for c in df.columns if c.startswith("lidar_")]
    scaler_lidar = MinMaxScaler()
    scaler_action = MinMaxScaler()
    df = df.copy()
    df[lidar_cols] = scaler_lidar.fit_transform(df[lidar_cols])
    df[["steering_angle", "speed"]] = scaler_action.fit_transform(df[["steering_angle", "speed"]])
    return df, scaler_lidar, scaler_action


def main() -> None:
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    frames = []
    for i, (bag_path, lap_sec) in enumerate(BAG_PATHS):
        if not os.path.exists(bag_path):
            continue
        df = bag_to_df(bag_path)
        df = label_laps(df, session_id=i, lap_duration_sec=lap_sec)
        df = clean(df)
        df = downsample_lidar(df)
        frames.append(df)

    if not frames:
        return
    combined = pd.concat(frames, ignore_index=True)
    # bags with different ray counts leave gaps in the union of lidar columns
    lidar_cols = [c for c in combined.columns if c.startswith("lidar_")]
    combined[lidar_cols] = combined[lidar_cols].fillna(MAX_RANGE_M)
    combined = augment(combined)
    combined, scaler_lidar, scaler_action = normalize(combined)
    combined.to_csv(f"{OUTPUT_DIR}/data.csv", index=False)
    joblib.dump(scaler_lidar, f"{OUTPUT_DIR}/scaler_lidar.pkl")
    joblib.dump(scaler_action, f"{OUTPUT_DIR}/scaler_action.pkl")

    # the npz is what the ros2 inference nodes read; the pkls are for offline work
    np.savez(
        f"{OUTPUT_DIR}/scalers.npz",
        lidar_scale=scaler_lidar.scale_.astype(np.float32),
        lidar_min=scaler_lidar.min_.astype(np.float32),
        action_scale=scaler_action.scale_.astype(np.float32),
        action_min=scaler_action.min_.astype(np.float32),
    )
    print(f"scalers saved to {OUTPUT_DIR}/scalers.npz")
    print(f"  steering range: [{-MAX_STEER_RAD}, {MAX_STEER_RAD}] rad")
    print(f"  action_scale: {scaler_action.scale_}")
    print(f"  action_min:   {scaler_action.min_}")


if __name__ == "__main__":
    main()
