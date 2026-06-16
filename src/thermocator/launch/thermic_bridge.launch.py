#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    bridge_config = os.path.join(
        get_package_share_directory('thermocator'),
        'config',
        'domain_bridge.yaml',
    )

    bridge = ExecuteProcess(
        cmd=['ros2', 'run', 'domain_bridge', 'domain_bridge', bridge_config],
        output='screen',
    )

    return LaunchDescription([bridge])
