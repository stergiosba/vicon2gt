#!/usr/bin/env python3
"""
Extract PoseStamped topic from ROS2 bag and write to CSV.
"""

import argparse
import csv
import os
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
import numpy as np


def extract_pose_from_bag(bag_path, topic_name, output_csv=None):
    """
    Extract PoseStamped messages from a ROS2 bag and save to CSV.

    Args:
        bag_path: Path to the ROS2 bag directory
        topic_name: Topic name to extract (must be PoseStamped)
        output_csv: Output CSV file path (default: same name as topic)
    """
    if not os.path.exists(bag_path):
        print(f"Error: Bag path not found: {bag_path}")
        return None

    if output_csv is None:
        safe_topic = topic_name.replace("/", "_").strip("_")
        output_csv = f"{safe_topic}_poses.csv"

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    topic_info = reader.get_all_topics_and_types()
    available_topics = [t.name for t in topic_info]

    if topic_name not in available_topics:
        print(f"Error: Topic '{topic_name}' not found in bag")
        print(f"Available topics: {available_topics}")
        return None

    topic_type = None
    for t in topic_info:
        if t.name == topic_name:
            topic_type = t.type
            break

    if topic_type != "geometry_msgs/msg/PoseStamped":
        print(
            f"Warning: Topic type is '{topic_type}', expected 'geometry_msgs/msg/PoseStamped'"
        )

    poses = []
    timestamps = []

    print(f"Reading from bag: {bag_path}")
    print(f"Topic: {topic_name} ({topic_type})")

    while reader.has_next():
        topic, data, t = reader.read_next()

        if topic == topic_name:
            msg = deserialize_message(data, PoseStamped)
            timestamp_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec

            poses.append(
                [
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                    msg.pose.orientation.w,
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                ]
            )
            timestamps.append(timestamp_ns)

    if not poses:
        print(f"No messages found on topic '{topic_name}'")
        return None

    with open(output_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp_ns", "x", "y", "z", "qw", "qx", "qy", "qz"])
        for i, (pose, ts) in enumerate(zip(poses, timestamps)):
            writer.writerow([ts] + pose)

    print(f"Saved {len(poses)} poses to {output_csv}")
    return output_csv


def main():
    parser = argparse.ArgumentParser(
        description="Extract PoseStamped topic from ROS2 bag to CSV"
    )
    parser.add_argument("--bag_path", type=str, help="Path to the ROS2 bag directory")
    parser.add_argument("--topic", type=str, help="PoseStamped topic name to extract")
    parser.add_argument("--output", type=str, default=None, help="Output CSV file path")

    args = parser.parse_args()

    extract_pose_from_bag(args.bag_path, args.topic, args.output)


if __name__ == "__main__":
    main()
