"""
Preprocessing pipeline for BC/SAC training data.

Converts ROS2 bag files to a normalized CSV dataset ready for training.
Steps: bag → CSV, label laps, clean, downsample LiDAR, augment, normalize.

Usage:
    pip install pandas scikit-learn joblib
    python preprocessing/preprocess.py
"""
from __future__ import annotations

import os
import sys
import subprocess
import tempfile
import pandas as pd
import numpy as np
import joblib
from sklearn.preprocessing import MinMaxScaler

# Config
BAG_PATHS = [
    ("data/gap_following_data", 24),
    ("data/gap_following_data_v2", 24),
    ("data/gap_following_data_v3", 148),
]
# Output directory
OUTPUT_DIR = "processed"
# Lap duration in seconds
LAP_DURATION_SEC = 148  # approximate lap time in seconds
# Lidar step
LIDAR_STEP = 6          # keep every nth ray (1081 → 181 rays)
# Maximum range
MAX_RANGE = 10.0        # cap lidar readings at this distance in meters
MIN_SPEED = 0.05        # drop rows where car is stationary
MAX_STEER = 0.5         # clip steering to ±0.5 rad (car max is ~0.42)

def bag_to_df(bag_path) -> pd.DataFrame:
    """
    Convert a ROS2 bag to a pandas DataFrame via a temporary CSV.

    Args:
        bag_path: Path to the ROS2 bag directory.

    Returns:
        A DataFrame with columns for timestamp, LiDAR rays, steering, speed, and odom_vx.
    """
    # Convert the bag to a temporary CSV then load it into a DataFrame
    with tempfile.NamedTemporaryFile(suffix=".csv", delete=False) as f:
        tmp_path = f.name
    try:
        # Run the extract_dataset.py script
        subprocess.run(
            [sys.executable, "preprocessing/extract_dataset.py", "--bag", bag_path, "--output", tmp_path],
            check=True
        )
        df = pd.read_csv(tmp_path)
    finally:
        os.remove(tmp_path)
    return df

def label_laps(df, session_id, lap_duration_sec) -> pd.DataFrame:
    """
    Assign a lap ID to each row based on elapsed time, and drop the last partial lap.

    Args:
        df: The raw DataFrame with a timestamp column in nanoseconds.
        session_id: An integer identifier for this recording session.
        lap_duration_sec: Approximate lap duration in seconds.

    Returns:
        The DataFrame with added lap_id and session columns.
    """
    # Sort the DataFrame by timestamp
    df = df.sort_values("timestamp").reset_index(drop=True)
    # Calculate the lap duration in nanoseconds
    lap_duration_ns = lap_duration_sec * 1_000_000_000
    # Get the start timestamp
    start_ts = df["timestamp"].iloc[0]
    # Copy the DataFrame
    df = df.copy()
    # Assign the lap ID
    df["lap_id"] = ((df["timestamp"] - start_ts) // lap_duration_ns).astype(int)
    # Assign the session ID
    df["session"] = session_id
    # drop last partial lap (only if there are multiple laps)
    if df["lap_id"].max() > 0:
        df = df[df["lap_id"] < df["lap_id"].max()].reset_index(drop=True)
    return df

def clean(df) -> pd.DataFrame:
    """
    Remove bad rows and clip outliers from the DataFrame.

    Caps LiDAR at MAX_RANGE, fills NaN/inf, clips steering to ±MAX_STEER,
    and drops rows where the car is stationary (|odom_vx| < MIN_SPEED).

    Args:
        df: The raw DataFrame.

    Returns:
        The cleaned DataFrame.
    """
    # Get the lidar columns
    lidar_cols = [c for c in df.columns if c.startswith("lidar_")]
    # Copy the DataFrame
    df = df.copy()
    # Cap the lidar at the maximum range and fill missing values
    # cap lidar at max range and fill missing values
    df[lidar_cols] = (
        df[lidar_cols]
        .replace([np.inf, -np.inf], np.nan)
        .clip(0, MAX_RANGE)
        .fillna(MAX_RANGE)
    )
    # Clip steering outliers (raw data can have ±π from bad readings)
    df["steering_angle"] = df["steering_angle"].clip(-MAX_STEER, MAX_STEER)
    # Drop rows where the car is stationary
    df = df[df["odom_vx"].abs() > MIN_SPEED].reset_index(drop=True)
    return df

def downsample_lidar(df) -> pd.DataFrame:
    """
    Keep every LIDAR_STEP-th LiDAR column to reduce input size.

    Args:
        df: The full DataFrame with all raw LiDAR columns.

    Returns:
        A DataFrame with only the downsampled LiDAR columns and metadata columns.
    """
    # Get the lidar columns
    lidar_cols = [c for c in df.columns if c.startswith("lidar_")]
    # Get the columns to keep
    keep = lidar_cols[::LIDAR_STEP]
    # Get the metadata columns
    meta = ["timestamp", "steering_angle", "speed", "odom_vx", "lap_id", "session"]
    return df[meta + keep]

def augment(df) -> pd.DataFrame:
    """
    Double the dataset by mirroring LiDAR scans and flipping steering.

    Creates a copy of each row with the LiDAR scan reversed left-to-right
    and the steering angle negated, simulating the opposite driving direction.

    Args:
        df: The cleaned and downsampled DataFrame.

    Returns:
        A DataFrame containing the original and mirrored rows.
    """
    # Mirror lidar scans and flip steering to simulate driving the other direction
    lidar_cols = [c for c in df.columns if c.startswith("lidar_")]
    mirrored = df.copy()
    # Mirror the lidar scans
    mirrored[lidar_cols] = df[lidar_cols].values[:, ::-1]
    mirrored["steering_angle"] = -df["steering_angle"]
    return pd.concat([df, mirrored], ignore_index=True)

def normalize(df) -> tuple[pd.DataFrame, MinMaxScaler, MinMaxScaler]:
    """
    Fit MinMaxScalers and normalize LiDAR and action columns to [0, 1].

    Args:
        df: The augmented DataFrame.

    Returns:
        A tuple of (normalized_df, scaler_lidar, scaler_action).
    """
    # Get the lidar columns
    lidar_cols = [c for c in df.columns if c.startswith("lidar_")]
    # Initialize the lidar scaler
    scaler_lidar = MinMaxScaler()
    # Initialize the action scaler
    scaler_action = MinMaxScaler()
    # Copy the DataFrame
    df = df.copy()
    # Fit the lidar scaler
    df[lidar_cols] = scaler_lidar.fit_transform(df[lidar_cols])
    # Fit the action scaler
    df[["steering_angle", "speed"]] = scaler_action.fit_transform(df[["steering_angle", "speed"]])
    return df, scaler_lidar, scaler_action

def main() -> None:
    """
    Run the full preprocessing pipeline and save data and scalers.

    Args:
        None

    Returns:
        None
    """
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
    # Concatenate the frames
    combined = pd.concat(frames, ignore_index=True)
    # Fill NaN from bags with different ray counts
    lidar_cols = [c for c in combined.columns if c.startswith("lidar_")]
    combined[lidar_cols] = combined[lidar_cols].fillna(MAX_RANGE)
    combined = augment(combined)
    combined, scaler_lidar, scaler_action = normalize(combined)
    # Save the combined DataFrame to a CSV file
    combined.to_csv(f"{OUTPUT_DIR}/data.csv", index=False)
    # Save the lidar scaler to a pkl file
    joblib.dump(scaler_lidar, f"{OUTPUT_DIR}/scaler_lidar.pkl")
    # Save the action scaler to a pkl file
    joblib.dump(scaler_action, f"{OUTPUT_DIR}/scaler_action.pkl")

    # Save scalers as .npz for the ROS2 inference nodes
    np.savez(
        f"{OUTPUT_DIR}/scalers.npz",
        lidar_scale=scaler_lidar.scale_.astype(np.float32),
        lidar_min=scaler_lidar.min_.astype(np.float32),
        action_scale=scaler_action.scale_.astype(np.float32),
        action_min=scaler_action.min_.astype(np.float32),
    )
    print(f"Scalers saved to {OUTPUT_DIR}/scalers.npz")
    print(f"  Steering range: [{-MAX_STEER}, {MAX_STEER}] rad")
    print(f"  action_scale: {scaler_action.scale_}")
    print(f"  action_min:   {scaler_action.min_}")

if __name__ == "__main__":
    main()
