#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="basecam",
                node_namespace="basecam",
                node_executable="basecam",
                node_name="basecam",
            )
        ]
    )
