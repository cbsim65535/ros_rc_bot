#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="basecam_node",
                executable="basecam_node",
                output="screen",
            ),
            Node(
                package="odom_node",
                executable="odom_node",
                output="screen",
            ),
            Node(
                package="bno055_node",
                executable="bno055_node",
                output="screen",
            ),
            Node(
                package="pca9685_node",
                executable="pca9685_node",
                output="screen",
            ),
            Node(
                package="driving_node",
                executable="driving_node",
                output="screen",
            ),
            Node(
                package="driving_node",
                executable="manual_node",
                output="screen",
            ),
            Node(
                package="joystick_ros2",
                executable="joystick_ros2",
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("rc_bringup"),
                        "tf.launch.py",
                    )
                )
            ),
        ]
    )
