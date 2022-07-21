#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pca9685",
                node_namespace="pca9685",
                node_executable="pca9685_node",
                node_name="pca9685",
            ),
            Node(
                package="turtlesim",
                node_namespace="turtlesim2",
                node_executable="turtlesim_node",
                node_name="sim",
            ),
            Node(
                package="turtlesim",
                node_executable="mimic",
                node_name="mimic",
                remappings=[
                    ("/input/pose", "/turtlesim1/turtle1/pose"),
                    ("/output/cmd_vel", "/turtlesim2/turtle1/cmd_vel"),
                ],
            ),
        ]
    )
