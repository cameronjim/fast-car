"""converts a ros2 bag into one csv row per scan, time-synced with drive and odom."""
from __future__ import annotations

import argparse
import csv
import bisect
import os
from typing import Any
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

# ackermann_msgs is not in the rosbags typestore, so its definitions are registered by hand
ACKERMANN_DRIVE = """\
float32 steering_angle
float32 steering_angle_velocity
float32 speed
float32 acceleration
float32 jerk
"""

ACKERMANN_DRIVE_STAMPED = """\
std_msgs/Header header
ackermann_msgs/AckermannDrive drive
"""


def make_typestore() -> Any:
    """ros2 humble typestore with the ackermann message types registered."""
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    extra = {}
    extra.update(get_types_from_msg(ACKERMANN_DRIVE, 'ackermann_msgs/msg/AckermannDrive'))
    extra.update(get_types_from_msg(ACKERMANN_DRIVE_STAMPED, 'ackermann_msgs/msg/AckermannDriveStamped'))
    typestore.register(extra)
    return typestore


def extract_messages(bag_path) -> tuple[list, list, list]:
    """read every scan, drive, and odom message out of a bag as (timestamp, msg) pairs."""
    scans, drives, odoms = [], [], []
    topic_counts = {}
    typestore = make_typestore()
    print(f"opening bag: {bag_path}")
    with Reader(bag_path) as reader:
        print(f"topics in bag: {[(c.topic, c.msgtype) for c in reader.connections]}")
        for connection, timestamp, rawdata in reader.messages():
            topic = connection.topic
            topic_counts[topic] = topic_counts.get(topic, 0) + 1
            try:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            except Exception as e:
                if topic in ('/odom', '/ego_racecar/odom'):
                    print(f"odom deserialize error: {e}")
                continue
            if topic == '/scan':
                scans.append((timestamp, msg))
            elif topic == '/drive':
                drives.append((timestamp, msg))
            elif topic in ('/odom', '/ego_racecar/odom'):
                odoms.append((timestamp, msg))
    print(f"all topic counts: {topic_counts}")
    print(f"extracted: {len(scans)} scans, {len(drives)} drives, {len(odoms)} odoms")
    return scans, drives, odoms


def find_closest(target_ts, timestamps, messages) -> tuple[Any, float]:
    """nearest message by timestamp, with its gap in nanoseconds."""
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
    if best_idx is None:
        return None, float('inf')
    return messages[best_idx], best_diff


def sync_messages(scans, drives, odoms, max_time_diff_ms=50) -> list:
    """one row per scan, matched to the nearest drive and odom inside max_time_diff_ms."""
    max_diff_ns = max_time_diff_ms * 1_000_000
    rows = []
    dropped = 0

    if not scans:
        print("no scans found, nothing to sync")
        return rows
    if not drives:
        print("no drive messages found, nothing to sync")
        return rows
    if not odoms:
        print("no odom messages found, nothing to sync")
        return rows

    # timestamps pulled out once so find_closest can binary search them
    drive_ts = [ts for ts, _ in drives]
    drive_msgs = [msg for _, msg in drives]
    odom_ts = [ts for ts, _ in odoms]
    odom_msgs = [msg for _, msg in odoms]

    for ts, scan_msg in scans:
        drive_msg, drive_diff = find_closest(ts, drive_ts, drive_msgs)
        odom_msg, odom_diff = find_closest(ts, odom_ts, odom_msgs)

        if drive_diff > max_diff_ns or odom_diff > max_diff_ns:
            dropped += 1
            continue

        rows.append((
            ts,
            list(scan_msg.ranges),
            drive_msg.drive.steering_angle,
            drive_msg.drive.speed,
            odom_msg.twist.twist.linear.x
        ))

    print(f"synced: {len(rows)} rows kept, {dropped} dropped (max_time_diff={max_time_diff_ms}ms)")
    if dropped > 0 and len(rows) == 0:
        print("all rows dropped, try increasing --max-time-diff")
    return rows


def save_csv(rows, output_path) -> None:
    """write the synced rows as timestamp, lidar_0..lidar_n, steering_angle, speed, odom_vx."""
    if not rows:
        print("no rows to save, output file not created")
        return
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    num_lidar_rays = len(rows[0][1])
    headers = ["timestamp"] + [f"lidar_{i}" for i in range(num_lidar_rays)] + ["steering_angle", "speed", "odom_vx"]
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        for ts, lidar, steering, speed, vx in rows:
            writer.writerow([ts] + lidar + [steering, speed, vx])
    print(f"saved {len(rows)} rows to {output_path}")


def main() -> None:
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
