#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pca9685_node",
                node_namespace="pca9685_node",
                node_executable="pca9685_node",
                node_name="pca9685_node",
            ),
        ]
    )
