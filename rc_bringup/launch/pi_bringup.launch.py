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
            ),
            Node(
                package="bno055_node",
                node_namespace="bno055_node",
                node_executable="bno055_node",
                node_name="bno055_node",
            ),
            Node(
                package="pca9685_node",
                node_namespace="pca9685_node",
                node_executable="pca9685_node",
                node_name="pca9685_node",
            ),
            Node(
                package="driving_node",
                node_namespace="driving_node",
                node_executable="driving_node",
                node_name="driving_node",
            ),
        ]
    )
