#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("vicon2gt")
    config_file = os.path.join(pkg_share, "config", "exp_eth.yaml")

    dataset_arg = DeclareLaunchArgument(
        "dataset",
        default_value="rosbag2_2026_05_12-17_52_37_0",
        description="Dataset name: V1_01_easy, V1_02_medium, V1_03_difficult, V2_01_easy, V2_02_medium, V2_03_difficult",
    )

    # Bag folder path. By default the ROS2 bag is expected under:
    # ~/Downloads/vicon_room1/<dataset>_ros2
    folder_arg = DeclareLaunchArgument(
        "folder",
        default_value="/home/horclab/optitrack_client/go1_imu_optitrack_bag1",
        description="Path to dataset folder containing rosbag2 directory",
    )

    vicon_topic_arg = DeclareLaunchArgument(
        "topic_vicon",
        default_value="/go1/pose",
        description="Vicon topic name",
    )

    imu_topic_arg = DeclareLaunchArgument(
        "topic_imu", default_value="/imu", description="IMU topic name"
    )

    # The config file path is passed as the first argument.
    # The folder is passed as the second argument (optional bag override).
    estimate_node = Node(
        package="vicon2gt",
        executable="estimate_vicon2gt",
        name="estimate_vicon2gt",
        output="screen",
        arguments=[
            config_file,
            [
                LaunchConfiguration("folder"),
                "/",
                LaunchConfiguration("dataset"),
            ],
        ],
        parameters=[
            {
                "topic_vicon": LaunchConfiguration("topic_vicon"),
                "topic_imu": LaunchConfiguration("topic_imu"),
            }
        ],
    )

    return LaunchDescription(
        [dataset_arg, folder_arg, vicon_topic_arg, imu_topic_arg, estimate_node]
    )
