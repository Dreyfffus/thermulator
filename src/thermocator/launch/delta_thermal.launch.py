#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="thermaria",
        description="Gazebo world name (must match <world name=...> in .world file)",
    )
    robot_entity_arg = DeclareLaunchArgument(
        "robot_entity_name",
        default_value="turtlebot3_burger",
        description="Gazebo model name of the sim robot",
    )

    world_name = LaunchConfiguration("world_name")
    robot_entity_name = LaunchConfiguration("robot_entity_name")

    # All nodes in this file run on Domain 1
    set_domain = SetEnvironmentVariable("ROS_DOMAIN_ID", "1")

    advisory = Node(
        package="thermocator",
        executable="advisory_node",
        name="advisory_node",
        output="screen",
    )

    pose_sync = Node(
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
    )

    return LaunchDescription(
        [
            world_name_arg,
            robot_entity_arg,
            set_domain,
            advisory,
            pose_sync,
        ]
    )
