#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("rgbd_fullcalib"))
        / "config"
        / "mid360s_imu_calibration.yaml"
    )
    config = LaunchConfiguration("config_file")
    project_root = LaunchConfiguration("project_root")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="YAML parameters for calibrated IMU runtime",
        ),
        DeclareLaunchArgument(
            "project_root",
            description="Explicit calibration artifact root; independent of launch cwd",
        ),
        Node(
            package="rgbd_fullcalib",
            executable="mid360s_imu_runtime.py",
            name="mid360s_imu_runtime",
            output="screen",
            parameters=[config, {"project_root": project_root}],
        ),
    ])
