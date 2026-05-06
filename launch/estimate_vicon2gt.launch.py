#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("vicon2gt")
    config_file = os.path.join(pkg_share, "config", "params.yaml")

    config_arg = DeclareLaunchArgument(
        "config_file", default_value=config_file, description="Path to YAML config file"
    )

    bag_path_arg = DeclareLaunchArgument(
        "bag_path", default_value="", description="Path to rosbag2 folder"
    )

    vicon_topic_arg = DeclareLaunchArgument(
        "vicon_topic",
        default_value="/vicon/publisher_topic",
        description="Vicon pose topic name",
    )

    imu_topic_arg = DeclareLaunchArgument(
        "imu_topic", default_value="/imu topic name", description="IMU topic name"
    )

    cam_topic_arg = DeclareLaunchArgument(
        "cam_topic",
        default_value="/camera topic name",
        description="Camera timestamp topic name",
    )

    output_csv_arg = DeclareLaunchArgument(
        "output_csv",
        default_value="/tmp/vicon2gt_output.csv",
        description="Output CSV file path",
    )

    estimate_node = Node(
        package="vicon2gt",
        executable="estimate_vicon2gt",
        name="estimate_vicon2gt",
        output="screen",
        parameters=[
            {
                "config_file": LaunchConfiguration("config_file"),
                "bag_path": LaunchConfiguration("bag_path"),
                "topic_vicon": LaunchConfiguration("vicon_topic"),
                "topic_imu": LaunchConfiguration("imu_topic"),
                "topic_cam": LaunchConfiguration("cam_topic"),
                "path_states": LaunchConfiguration("output_csv"),
            }
        ],
    )

    return LaunchDescription(
        [
            config_arg,
            bag_path_arg,
            vicon_topic_arg,
            imu_topic_arg,
            cam_topic_arg,
            output_csv_arg,
            estimate_node,
        ]
    )
