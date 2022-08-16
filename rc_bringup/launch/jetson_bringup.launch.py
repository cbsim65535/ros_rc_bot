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

import lifecycle_msgs.msg
import os


def generate_launch_description():
    share_dir = get_package_share_directory("rc_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file0",
                default_value=os.path.join(
                    share_dir, "config", "ydlidar0.yaml"),
                description="FPath to the ROS2 parameters file to use.",
            ),
            LifecycleNode(
                package="ydlidar_ros2_driver",
                executable="ydlidar_ros2_driver_node",
                name="ydlidar_ros2_driver_node0",
                output="screen",
                emulate_tty=True,
                parameters=[LaunchConfiguration("params_file0")],
                remappings=[
                    ("/start_scan", "/ydlidar_ros2_driver_node0/start_scan"),
                    ("/stop_scan", "/ydlidar_ros2_driver_node0/stop_scan"),
                    ("/scan", "/ydlidar_ros2_driver_node0/scan"),
                ],
            ),
            DeclareLaunchArgument(
                "params_file1",
                default_value=os.path.join(
                    share_dir, "config", "ydlidar1.yaml"),
                description="FPath to the ROS2 parameters file to use.",
            ),
            LifecycleNode(
                package="ydlidar_ros2_driver",
                executable="ydlidar_ros2_driver_node",
                name="ydlidar_ros2_driver_node1",
                output="screen",
                emulate_tty=True,
                parameters=[LaunchConfiguration("params_file1")],
                remappings=[
                    ("/start_scan", "/ydlidar_ros2_driver_node1/start_scan"),
                    ("/stop_scan", "/ydlidar_ros2_driver_node1/stop_scan"),
                    ("/scan", "/ydlidar_ros2_driver_node1/scan"),
                ],
            ),
        ]
    )
