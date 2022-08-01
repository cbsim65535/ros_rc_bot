#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="basecam_node",
                executable="basecam_node",
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
                package="joystick_ros2",
                executable="joystick_ros2",
                output="screen",
            ),
        ]
    )
