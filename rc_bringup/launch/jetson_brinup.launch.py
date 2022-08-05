#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import os


def generate_launch_description():
    share_dir = get_package_share_directory("rc_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(share_dir, "config", "ydlidar1.yaml"),
            ),
            # DeclareLaunchArgument(
            #     "ydlidar_params_file1",
            #     default_value=os.path.join(share_dir, "config", "ydlidar1.yaml"),
            # ),
            # Node(
            #     package="ydlidar",
            #     executable="ydlidar_node",
            #     name="ydlidar_node0",
            #     output="screen",
            #     emulate_tty=True,
            #     parameters=[LaunchConfiguration("ydlidar_params_file0")],
            # ),
            Node(
                package="ydlidar",
                executable="ydlidar_node",
                name="ydlidar_node1",
                output="screen",
                emulate_tty=True,
                parameters=[LaunchConfiguration("params_file")],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_pub_laser0",
                arguments=[
                    "0",
                    "0",
                    "0.02",
                    "0",
                    "0",
                    "0",
                    "1",
                    "base_link",
                    "laser_frame0",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_pub_laser1",
                arguments=[
                    "0",
                    "0",
                    "0.02",
                    "0",
                    "0",
                    "0",
                    "1",
                    "base_link",
                    "laser_frame1",
                ],
            ),
        ]
    )
