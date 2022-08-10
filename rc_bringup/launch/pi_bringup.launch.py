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
                package="driving_node",
                executable="manual_node",
                output="screen",
            ),
            Node(
                package="joystick_ros2",
                executable="joystick_ros2",
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_joint",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "1",
                    "base_link",
                    "map",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_pub_laser0",
                arguments=[
                    "0.16",
                    "0.08",
                    "0.35",
                    "0",
                    "0",
                    "0.9681",
                    "0.2503",
                    "base_link",
                    "laser_frame0",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_pub_laser1",
                arguments=[
                    "-0.165",
                    "-0.08",
                    "0.35",
                    "0",
                    "0",
                    "0",
                    "1",
                    "base_link",
                    "laser_frame1",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="gimbal_joint",
                arguments=[
                    "0.08",
                    "0.00",
                    "0.54",
                    "0",
                    "0",
                    "0",
                    "1",
                    "base_link",
                    "gimbal_camera_mount",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_joint",
                arguments=[
                    "0.04",
                    "-0.013",
                    "0.0",
                    "0",
                    "0",
                    "0",
                    "1",
                    "gimbal_camera_mount",
                    "camera_base",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_joint",
                arguments=[
                    "0.0",
                    "0.0",
                    "0.0",
                    "0",
                    "0",
                    "0",
                    "1",
                    "camera_base",
                    "zed_base_link",
                ],
            ),
        ]
    )
