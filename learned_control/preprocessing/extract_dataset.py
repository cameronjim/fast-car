"""
Convert a ROS2 bag file to a CSV of synchronized scan/drive/odom data.

Reads /scan (LaserScan), /drive (AckermannDriveStamped), and /odom or
/ego_racecar/odom (Odometry) topics from a ROS2 bag, time-syncs them,
and writes one row per scan to a CSV file.

Usage:
    python preprocessing/extract_dataset.py --bag data/my_bag --output training_data.csv --max-time-diff 50
"""
from __future__ import annotations

import argparse
import csv
import bisect
import os
from typing import Any
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

# Message types
ACKERMANN_DRIVE = """\
float32 steering_angle
float32 steering_angle_velocity
float32 speed
float32 acceleration
float32 jerk
"""

# Message types
ACKERMANN_DRIVE_STAMPED = """\
std_msgs/Header header
ackermann_msgs/AckermannDrive drive
"""

def make_typestore() -> Any:
    """
    Build a ROS2 Humble typestore with AckermannDrive message types registered.

    Args:
        None

    Returns:
        The configured typestore.
    """
    # Build the typestore
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    # Register the message types
    extra = {}
    extra.update(get_types_from_msg(ACKERMANN_DRIVE, 'ackermann_msgs/msg/AckermannDrive'))
    extra.update(get_types_from_msg(ACKERMANN_DRIVE_STAMPED, 'ackermann_msgs/msg/AckermannDriveStamped'))
    typestore.register(extra)
    return typestore

def extract_messages(bag_path) -> tuple[list, list, list]:
    """
    Read all scan, drive, and odom messages from a ROS2 bag file.

    Args:
        bag_path: Path to the ROS2 bag directory.

    Returns:
        A tuple of (scans, drives, odoms), each a list of (timestamp, msg) pairs.
    """
    # Initialize the lists
    scans, drives, odoms = [], [], []
    # Initialize the topic counts
    topic_counts = {}
    # Make the typestore
    typestore = make_typestore()
    # Print the bag path
    print(f"opening bag: {bag_path}")
    # Read the bag file
    with Reader(bag_path) as reader:
        print(f"topics in bag: {[(c.topic, c.msgtype) for c in reader.connections]}")
        for connection, timestamp, rawdata in reader.messages():
            # Get the topic and increment the topic count
            topic = connection.topic
            topic_counts[topic] = topic_counts.get(topic, 0) + 1
            try:
                # Deserialize the message
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            except Exception as e:
                if topic in ('/odom', '/ego_racecar/odom'):
                    print(f"odom deserialize error: {e}")
                continue
            # Append the message to the appropriate list
            if topic == '/scan':
                scans.append((timestamp, msg))
            elif topic == '/drive':
                drives.append((timestamp, msg))
            elif topic in ('/odom', '/ego_racecar/odom'):
                odoms.append((timestamp, msg))
    # Print the topic counts
    print(f"all topic counts: {topic_counts}")
    print(f"extracted: {len(scans)} scans, {len(drives)} drives, {len(odoms)} odoms")
    return scans, drives, odoms

def find_closest(target_ts, timestamps, messages) -> tuple[Any, float]:
    """
    Find the message with the closest timestamp to target_ts.

    Args:
        target_ts: The target timestamp in nanoseconds.
        timestamps: Sorted list of timestamps in nanoseconds.
        messages: List of messages corresponding to each timestamp.

    Returns:
        A tuple of (message, time_diff_ns) for the closest match,
        or (None, inf) if timestamps is empty.
    """
    # Binary search for the nearest timestamp
    if not timestamps:
        return None, float('inf')
    idx = bisect.bisect_left(timestamps, target_ts)
    best_idx, best_diff = None, float('inf')
    for i in [idx - 1, idx]:
        if 0 <= i < len(timestamps):
            diff = abs(timestamps[i] - target_ts)
            if diff < best_diff:
                best_diff = diff
                best_idx = i
    # If the best index is None, return None and infinity
    if best_idx is None:
        return None, float('inf')
    return messages[best_idx], best_diff

def sync_messages(scans, drives, odoms, max_time_diff_ms=50) -> list:
    """
    Time-synchronize scan, drive, and odom messages.

    For each scan, find the nearest drive and odom messages within
    max_time_diff_ms milliseconds. Drops rows where no close match exists.

    Args:
        scans: List of (timestamp, LaserScan) pairs.
        drives: List of (timestamp, AckermannDriveStamped) pairs.
        odoms: List of (timestamp, Odometry) pairs.
        max_time_diff_ms: Maximum allowed time difference in milliseconds.

    Returns:
        A list of rows, each a tuple of (timestamp, lidar_ranges, steering, speed, vx).
    """
    # Calculate the maximum time difference in nanoseconds
    max_diff_ns = max_time_diff_ms * 1_000_000
    # Initialize the rows
    rows = []
    # Initialize the dropped count
    dropped = 0

    # If there are no scans, print a message and return the rows
    if not scans:
        print("no scans found — nothing to sync")
        return rows
    if not drives:
        print("no drive messages found — nothing to sync")
        return rows
    if not odoms:
        print("no odom messages found — nothing to sync")
        return rows

    # Pre-extract timestamps for binary search
    drive_ts = [ts for ts, _ in drives]
    drive_msgs = [msg for _, msg in drives]
    odom_ts = [ts for ts, _ in odoms]
    odom_msgs = [msg for _, msg in odoms]

    # For each scan, find the nearest drive and odom messages within the maximum time difference
    for ts, scan_msg in scans:
        drive_msg, drive_diff = find_closest(ts, drive_ts, drive_msgs)
        odom_msg, odom_diff = find_closest(ts, odom_ts, odom_msgs)

        if drive_diff > max_diff_ns or odom_diff > max_diff_ns:
            dropped += 1
            continue

        # Append the row to the rows list
        rows.append((
            ts,
            list(scan_msg.ranges),
            drive_msg.drive.steering_angle,
            drive_msg.drive.speed,
            odom_msg.twist.twist.linear.x
        ))

    # Print the synced rows
    print(f"synced: {len(rows)} rows kept, {dropped} dropped (max_time_diff={max_time_diff_ms}ms)")
    if dropped > 0 and len(rows) == 0:
        print("all rows dropped — try increasing --max-time-diff")
    return rows

def save_csv(rows, output_path) -> None:
    """
    Write synchronized rows to a CSV file.

    Columns: timestamp, lidar_0 ... lidar_N, steering_angle, speed, odom_vx.

    Args:
        rows: List of (timestamp, lidar_ranges, steering, speed, vx) tuples.
        output_path: Path to the output CSV file.

    Returns:
        None
    """
    # If there are no rows, print a message and return
    if not rows:
        print("no rows to save — output file not created")
        return
    # Make the output directory if it doesn't exist
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    # Get the number of lidar rays
    num_lidar_rays = len(rows[0][1])
    headers = ["timestamp"] + [f"lidar_{i}" for i in range(num_lidar_rays)] + ["steering_angle", "speed", "odom_vx"]
    # Write the headers to the CSV file
    with open(output_path, 'w', newline='') as f:
        # Create the writer
        writer = csv.writer(f)
        writer.writerow(headers)
        # Write the rows to the CSV file
        for ts, lidar, steering, speed, vx in rows:
            writer.writerow([ts] + lidar + [steering, speed, vx])
    # Print the number of rows saved
    print(f"saved {len(rows)} rows to {output_path}")

def main() -> None:
    """
    Main function to extract and save bag data to CSV.

    Args:
        None

    Returns:
        None
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--output', default='training_data.csv')
    parser.add_argument('--max-time-diff', type=int, default=50)
    args = parser.parse_args()

    print(f"bag path exists: {os.path.exists(args.bag)}")
    scans, drives, odoms = extract_messages(args.bag)
    rows = sync_messages(scans, drives, odoms, max_time_diff_ms=args.max_time_diff)
    save_csv(rows, args.output)

if __name__ == '__main__':
    main()
