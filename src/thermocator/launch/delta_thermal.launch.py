#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("thermocator")
    bridge_config = os.path.join(pkg, "config", "domain_bridge.yaml")

    world_name_arg = DeclareLaunchArgument("world_name", default_value="default")
    robot_entity_arg = DeclareLaunchArgument(
        "robot_entity_name", default_value="turtlebot3_burger"
    )

    world_name = LaunchConfiguration("world_name")
    robot_entity_name = LaunchConfiguration("robot_entity_name")

    bridge = Node(
        package="domain_bridge",
        executable="domain_bridge",
        name="dt_domain_bridge",
        output="screen",
        arguments=[bridge_config],
    )

    dt_nodes = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="thermocator",
                executable="advisory_node",
                name="advisory_node",
                output="screen",
            ),
            Node(
                package="thermocator",
                executable="pose_sync_node",
                name="pose_sync_node",
                output="screen",
                parameters=[
                    {
                        "world_name": world_name,
                        "robot_entity_name": robot_entity_name,
                    }
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            world_name_arg,
            robot_entity_arg,
            bridge,  # t=0s
            dt_nodes,  # t=3s
        ]
    )
