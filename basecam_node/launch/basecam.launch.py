#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="basecam_node",
                node_namespace="basecam_node",
                node_executable="basecam_node",
                node_name="basecam_node",
            )
        ]
    )
