#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="odom_node",
                executable="odom_node",
                output="screen",
            ),
        ]
    )
