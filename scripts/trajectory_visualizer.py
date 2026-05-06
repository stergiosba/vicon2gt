#!/usr/bin/env python3
"""
Trajectory visualizer for vicon2gt output.
Reads CSV file and replays trajectory incrementally for RVIZ visualization.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
import csv
import os


class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__("trajectory_visualizer")

        self.declare_parameter("csv_path", "/tmp/eth_vicon2gt_states.csv")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("replay_speed", 1.0)

        self.csv_path = self.get_parameter("csv_path").value
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.replay_speed = self.get_parameter("replay_speed").value

        self.path_pub = self.create_publisher(Path, "trajectory", 1)
        self.current_pose_pub = self.create_publisher(PoseStamped, "current_pose", 1)

        self.get_logger().info(f"Loading trajectory from: {self.csv_path}")

        if not os.path.exists(self.csv_path):
            self.get_logger().error(f"CSV file not found: {self.csv_path}")
            return

        self.poses, self.timestamps = self.load_trajectory()
        self.get_logger().info(f"Loaded {len(self.poses)} poses")

        if self.poses:
            self.current_idx = 0
            self.timer = self.create_timer(
                1.0 / self.publish_rate, self.publish_next_pose
            )
        else:
            self.get_logger().error("No poses loaded")

    def load_trajectory(self):
        poses = []
        timestamps = []
        with open(self.csv_path, "r") as f:
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

                    pose = PoseStamped()
                    pose.header.frame_id = self.frame_id
                    pose.pose.position.x = px
                    pose.pose.position.y = py
                    pose.pose.position.z = pz
                    pose.pose.orientation.x = qx
                    pose.pose.orientation.y = qy
                    pose.pose.orientation.z = qz
                    pose.pose.orientation.w = qw

                    poses.append(pose)
                    timestamps.append(timestamp_ns)
                except (ValueError, IndexError) as e:
                    self.get_logger().warn(f"Skipping row: {row} - {e}")
                    continue

        return poses, timestamps

    def publish_next_pose(self):
        if not self.poses or self.current_idx >= len(self.poses):
            self.current_idx = 0
            self.get_logger().info("Replay loop - restarting")

        step = max(1, int(self.replay_speed))
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        start_idx = max(0, self.current_idx - 100)
        msg.poses = self.poses[start_idx : self.current_idx + 1]

        self.path_pub.publish(msg)

        current_pose = self.poses[self.current_idx]
        current_pose.header.stamp = self.get_clock().now().to_msg()
        self.current_pose_pub.publish(current_pose)

        self.get_logger().debug(
            f"Publishing pose {self.current_idx + 1}/{len(self.poses)}"
        )

        self.current_idx = (self.current_idx + step) % len(self.poses)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        pass
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()
