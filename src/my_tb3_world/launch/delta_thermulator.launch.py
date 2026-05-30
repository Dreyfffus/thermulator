#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("thermocator")
    my_world_pkg = get_package_share_directory("my_tb3_world")

    bridge_config = os.path.join(pkg, "config", "domain_bridge_config.yaml")

    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="my_world",
        description="Gazebo world name (must match the .world file)",
    )
    robot_entity_arg = DeclareLaunchArgument(
        "robot_entity_name",
        default_value="turtlebot3_burger",
        description="Gazebo model name of the sim robot",
    )

    domain_bridge_node = Node(
        package="domain_bridge",
        executable="domain_bridge",
        name="dt_domain_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
    )

    sim_group = GroupAction(
        [
            SetEnvironmentVariable("ROS_DOMAIN_ID", "1"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(my_world_pkg, "launch", "bringup.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "true",
                }.items(),
            ),
            Node(
                package="thermocator",
                executable="pose_sync_node",
                name="pose_sync_node",
                output="screen",
                parameters=[
                    {
                        "world_name": LaunchConfiguration("world_name"),
                        "robot_entity_name": LaunchConfiguration("robot_entity_name"),
                        "map_frame": "map",
                        "robot_frame": "base_footprint",
                        "sync_rate_hz": 1.0,
                        "translation_deadband": 0.05,
                        "rotation_deadband": 0.05,
                        "robot_spawn_z": 0.01,
                    }
                ],
            ),
            Node(
                package="thermocator",
                executable="advisory_node",
                name="advisory_node",
                output="screen",
                parameters=[
                    {
                        "map_frame": "map",
                        "robot_frame": "base_footprint",
                        "sensor_radius": 0.3,
                        "goal_min_distance": 0.5,
                        "coverage_threshold": 0.95,
                        "radius_initial": 1.5,
                        "radius_step": 0.5,
                        "radius_max": 8.0,
                        "samples_per_cycle": 40,
                        "corridor_bonus": 0.3,
                        "advisory_stale_secs": 2.0,
                    }
                ],
            ),
        ]
    )

    return LaunchDescription(
        [
            world_name_arg,
            robot_entity_arg,
            domain_bridge_node,  # outside the group — no domain ID env
            sim_group,
        ]
    )
