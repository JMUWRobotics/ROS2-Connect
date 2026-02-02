# Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare a launch argument for the parameter file
    param_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([FindPackageShare('connect'),'launch','server.yaml'])
    )

    # Node definition, loading parameters from the specified file
    server = Node(
        package='connect',
        executable='server',
        name='server',
        parameters=[LaunchConfiguration('params_file')]
    )

    return LaunchDescription([
        param_file_arg,
        server
    ])
