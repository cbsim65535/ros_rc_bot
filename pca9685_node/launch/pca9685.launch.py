#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pca9685_node",
                executable="pca9685_node",
                output="screen",
            ),
        ]
    )
