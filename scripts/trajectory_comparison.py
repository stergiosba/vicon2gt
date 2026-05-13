#!/usr/bin/env python3
"""
Trajectory visualizer for vicon2gt output.
Reads CSV file and plots the full trajectory using matplotlib.
Can compare with another trajectory from CSV or ROS2 bag.
"""

import csv
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def load_trajectory_from_csv(csv_path):
    """Load trajectory from CSV file."""
    positions = []
    orientations = []
    timestamps = []

    with open(csv_path, "r") as f:
        reader = csv.reader(f)
        header = next(reader)

        for row in reader:
            if not row or row[0].startswith("#"):
                continue

            try:
                timestamp_ns = float(row[0])
                px, py, pz = float(row[1]), float(row[2]), float(row[3])
                qw, qx, qy, qz = (
                    float(row[4]),
                    float(row[5]),
                    float(row[6]),
                    float(row[7]),
                )

                positions.append([px, py, pz])
                orientations.append([qw, qx, qy, qz])
                timestamps.append(timestamp_ns)
            except (ValueError, IndexError) as e:
                print(f"Skipping row: {row} - {e}")
                continue

    return np.array(positions), np.array(orientations), np.array(timestamps)


def load_trajectory_from_bag(bag_path, topic_name):
    """Load trajectory from ROS2 bag PoseStamped topic."""
    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        from rclpy.serialization import deserialize_message
        from geometry_msgs.msg import PoseStamped
    except ImportError:
        print("Error: rosbag2_py or rclpy not installed. Cannot read ROS2 bags.")
        return None, None, None

    if not os.path.exists(bag_path):
        print(f"Error: Bag path not found: {bag_path}")
        return None, None, None

    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    topic_info = reader.get_all_topics_and_types()
    available_topics = [t.name for t in topic_info]

    if topic_name not in available_topics:
        print(f"Error: Topic '{topic_name}' not found in bag")
        print(f"Available topics: {available_topics}")
        return None, None, None

    positions = []
    orientations = []
    timestamps = []

    print(f"Reading from bag: {bag_path}")
    print(f"Topic: {topic_name}")

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(data, PoseStamped)
            timestamp_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
            positions.append(
                [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            )
            orientations.append(
                [
                    msg.pose.orientation.w,
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                ]
            )
            timestamps.append(timestamp_ns)

    if not positions:
        print(f"No messages found on topic '{topic_name}'")
        return None, None, None

    print(f"Loaded {len(positions)} poses from bag")
    return np.array(positions), np.array(orientations), np.array(timestamps)


def align_trajectories(traj1, traj2):
    """Align trajectories by finding best scale, rotation, and translation (Umeyama)."""
    if len(traj1) == 0 or len(traj2) == 0:
        return traj2

    # Subsample to same length
    min_len = min(len(traj1), len(traj2))
    t1 = traj1[:min_len]
    t2 = traj2[:min_len]

    # Compute means
    mu1 = np.mean(t1, axis=0)
    mu2 = np.mean(t2, axis=0)

    # Center the trajectories
    t1_centered = t1 - mu1
    t2_centered = t2 - mu2

    # Compute covariance matrix
    C = t1_centered.T @ t2_centered / min_len

    # SVD
    U, S, Vt = np.linalg.svd(C)

    # Compute rotation
    R = Vt.T @ U.T

    # Handle reflection case
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # Compute scale
    var1 = np.var(t1_centered, axis=0).sum()
    scale = np.sum(S) / var1 if var1 > 0 else 1.0

    # Transform traj2
    t2_aligned = scale * (t2 @ R.T) + mu1

    return t2_aligned


def compute_ATE(traj1, traj2):
    """Compute Absolute Trajectory Error."""
    min_len = min(len(traj1), len(traj2))
    t1 = traj1[:min_len]
    t2 = traj2[:min_len]
    errors = np.linalg.norm(t1 - t2, axis=1)
    return np.mean(errors), np.std(errors), np.max(errors)


def plot_trajectories(
    positions1,
    timestamps1,
    positions2=None,
    timestamps2=None,
    label1="Vicon2GT",
    label2="ROS2 Bag",
    csv_path="",
):
    """Plot and compare trajectories."""
    if len(positions1) == 0:
        print("No trajectory data to plot")
        return

    has_comparison = positions2 is not None and len(positions2) > 0

    if has_comparison:
        fig = plt.figure(figsize=(18, 5))
    else:
        fig = plt.figure(figsize=(15, 5))

    # 3D Trajectory Plot
    ax1 = fig.add_subplot(131 if has_comparison else 131, projection="3d")
    ax1.plot(
        positions1[:, 0],
        positions1[:, 1],
        positions1[:, 2],
        "b-",
        linewidth=2,
        label=label1,
    )
    ax1.scatter(
        positions1[0, 0],
        positions1[0, 1],
        positions1[0, 2],
        c="b",
        s=100,
        marker="^",
        label=f"{label1} Start",
    )
    ax1.scatter(
        positions1[-1, 0],
        positions1[-1, 1],
        positions1[-1, 2],
        c="b",
        s=100,
        marker="v",
        label=f"{label1} End",
    )

    if has_comparison:
        ax1.plot(
            positions2[:, 0],
            positions2[:, 1],
            positions2[:, 2],
            "r-",
            linewidth=2,
            label=label2,
        )
        ax1.scatter(
            positions2[0, 0],
            positions2[0, 1],
            positions2[0, 2],
            c="r",
            s=100,
            marker="^",
            label=f"{label2} Start",
        )
        ax1.scatter(
            positions2[-1, 0],
            positions2[-1, 1],
            positions2[-1, 2],
            c="r",
            s=100,
            marker="v",
            label=f"{label2} End",
        )

    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_zlabel("Z (m)")
    ax1.set_title("3D Trajectory")
    ax1.legend()
    ax1.grid(True)

    # XY Projection
    ax2 = fig.add_subplot(132)
    ax2.plot(positions1[:, 0], positions1[:, 1], "b-", linewidth=2, label=label1)
    ax2.scatter(positions1[0, 0], positions1[0, 1], c="b", s=100, marker="^")
    ax2.scatter(positions1[-1, 0], positions1[-1, 1], c="b", s=100, marker="v")

    if has_comparison:
        ax2.plot(positions2[:, 0], positions2[:, 1], "r-", linewidth=2, label=label2)
        ax2.scatter(positions2[0, 0], positions2[0, 1], c="r", s=100, marker="^")
        ax2.scatter(positions2[-1, 0], positions2[-1, 1], c="r", s=100, marker="v")

    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Y (m)")
    ax2.set_title("XY Projection (Top View)")
    ax2.legend()
    ax2.grid(True)
    ax2.axis("equal")

    # XZ Projection
    ax3 = fig.add_subplot(133)
    ax3.plot(positions1[:, 0], positions1[:, 2], "b-", linewidth=2, label=label1)
    ax3.scatter(positions1[0, 0], positions1[0, 2], c="b", s=100, marker="^")
    ax3.scatter(positions1[-1, 0], positions1[-1, 2], c="b", s=100, marker="v")

    if has_comparison:
        ax3.plot(positions2[:, 0], positions2[:, 2], "r-", linewidth=2, label=label2)
        ax3.scatter(positions2[0, 0], positions2[0, 2], c="r", s=100, marker="^")
        ax3.scatter(positions2[-1, 0], positions2[-1, 2], c="r", s=100, marker="v")
        ax3.set_title("XZ Projection (Side View) - Comparison")
    else:
        ax3.set_title("XZ Projection (Side View)")

    ax3.set_xlabel("X (m)")
    ax3.set_ylabel("Z (m)")
    ax3.legend()
    ax3.grid(True)

    plt.suptitle(f"Trajectory Visualization\n{os.path.basename(csv_path)}", fontsize=12)
    plt.tight_layout()

    # Print statistics
    print(f"\nTrajectory Statistics ({label1}):")
    print(f"  Total points: {len(positions1)}")
    if len(timestamps1) > 0:
        print(f"  Duration: {(timestamps1[-1] - timestamps1[0]) / 1e9:.2f} seconds")
    print(f"  X range: [{positions1[:, 0].min():.3f}, {positions1[:, 0].max():.3f}] m")
    print(f"  Y range: [{positions1[:, 1].min():.3f}, {positions1[:, 1].max():.3f}] m")
    print(f"  Z range: [{positions1[:, 2].min():.3f}, {positions1[:, 2].max():.3f}] m")
    print(
        f"  Total distance: {np.sum(np.linalg.norm(np.diff(positions1, axis=0), axis=1)):.3f} m"
    )

    if has_comparison:
        print(f"\nTrajectory Statistics ({label2}):")
        print(f"  Total points: {len(positions2)}")
        if len(timestamps2) > 0:
            print(f"  Duration: {(timestamps2[-1] - timestamps2[0]) / 1e9:.2f} seconds")
        print(
            f"  X range: [{positions2[:, 0].min():.3f}, {positions2[:, 0].max():.3f}] m"
        )
        print(
            f"  Y range: [{positions2[:, 1].min():.3f}, {positions2[:, 1].max():.3f}] m"
        )
        print(
            f"  Z range: [{positions2[:, 2].min():.3f}, {positions2[:, 2].max():.3f}] m"
        )
        print(
            f"  Total distance: {np.sum(np.linalg.norm(np.diff(positions2, axis=0), axis=1)):.3f} m"
        )

        # Compute ATE
        aligned_traj2 = align_trajectories(positions1, positions2)
        ate_mean, ate_std, ate_max = compute_ATE(positions1, aligned_traj2)
        print(f"\nAbsolute Trajectory Error (ATE) after alignment:")
        print(f"  Mean: {ate_mean:.4f} m")
        print(f"  Std:  {ate_std:.4f} m")
        print(f"  Max:  {ate_max:.4f} m")

    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Visualize vicon2gt trajectory from CSV"
    )
    parser.add_argument(
        "--csv_path",
        type=str,
        default=None,
        help="Path to the CSV file containing trajectory data",
    )

    # Comparison options
    parser.add_argument(
        "--compare_csv", type=str, default=None, help="Path to CSV file for comparison"
    )
    parser.add_argument(
        "--compare_bag", type=str, default=None, help="Path to ROS2 bag for comparison"
    )
    parser.add_argument(
        "--bag_topic", type=str, default=None, help="PoseStamped topic name in ROS2 bag"
    

    args = parser.parse_args()

    if not os.path.exists(args.csv_path):
        print(f"Error: CSV file not found: {args.csv_path}")
        return

    print(f"Loading primary trajectory from: {args.csv_path}")
    positions1, orientations1, timestamps1 = load_trajectory_from_csv(args.csv_path)
    print(f"Loaded {len(positions1)} poses")

    positions2 = None
    timestamps2 = None

    if args.compare_csv:
        if not os.path.exists(args.compare_csv):
            print(f"Error: Compare CSV not found: {args.compare_csv}")
        else:
            print(f"Loading comparison trajectory from: {args.compare_csv}")
            positions2, orientations2, timestamps2 = load_trajectory_from_csv(
                args.compare_csv
            )
            print(f"Loaded {len(positions2)} poses for comparison")

    elif args.compare_bag:
        if not args.bag_topic:
            print("Error: --bag_topic required when using --compare_bag")
        else:
            positions2, orientations2, timestamps2 = load_trajectory_from_bag(
                args.compare_bag, args.bag_topic
            )

    if len(positions1) > 0:
        plot_trajectories(
            positions1,
            timestamps1,
            positions2,
            timestamps2,
            label1="Inertial",
            label2="Optitrack",
            csv_path=args.csv_path,
        )


if __name__ == "__main__":
    main()
