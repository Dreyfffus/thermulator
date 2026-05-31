#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_thermocator = get_package_share_directory("thermocator")
    pkg_cartographer = get_package_share_directory("turtlebot3_cartographer")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    nav2_params_file = os.path.join(pkg_thermocator, "config", "nav2_slam_params.yaml")
    thermocator_launch_file = os.path.join(
        pkg_thermocator, "launch", "thermocator.launch.py"
    )

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    use_sim_time = LaunchConfiguration("use_sim_time")

    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_cartographer, "launch", "cartographer.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    nav2 = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "params_file": nav2_params_file,
                    "use_sim_time": use_sim_time,
                    "autostart": "false",
                }.items(),
            )
        ],
    )

    lifecycle_manager = TimerAction(
        period=18.0,
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": [
                            "controller_server",
                            "smoother_server",
                            "planner_server",
                            "behavior_server",
                            "velocity_smoother",
                            "collision_monitor",
                            "bt_navigator",
                            "waypoint_follower",
                        ],
                    }
                ],
            )
        ],
    )

    thermocator = TimerAction(
        period=25.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(thermocator_launch_file),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            cartographer,  # t=0s
            nav2,  # t=10s
            lifecycle_manager,  # t=18s
            thermocator,  # t=25s
        ]
    )
