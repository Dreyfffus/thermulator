#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
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
    rviz_config_file = os.path.join(pkg_thermocator, "config", "thermocator.rviz")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 with thermocator config",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    # -------------------------------------------------------------------------
    # Cartographer -- launched first, owns /map and map->odom TF
    # -------------------------------------------------------------------------
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_cartographer, "launch", "cartographer.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # -------------------------------------------------------------------------
    # Nav2 -- delayed 5s, autostart disabled so our lifecycle manager controls it
    # -------------------------------------------------------------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "params_file": nav2_params_file,
            "use_sim_time": use_sim_time,
            "autostart": "false",
        }.items(),
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
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

    nav2_delayed = TimerAction(period=5.0, actions=[nav2, lifecycle_manager])

    # -------------------------------------------------------------------------
    # Thermocator stack -- delayed 10s
    #   0s  -> cartographer
    #   5s  -> nav2 + lifecycle manager
    #  10s  -> thermal_broadcaster        (thermocator.launch.py t=0)
    #  13s  -> thermal_map_builder        (thermocator.launch.py t=3)
    #  18s  -> decision_node              (thermocator.launch.py t=8)
    # -------------------------------------------------------------------------
    thermocator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(thermocator_launch_file),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    thermocator_delayed = TimerAction(period=10.0, actions=[thermocator])

    # -------------------------------------------------------------------------
    # RViz2 -- pre-loaded with thermocator.rviz if it exists
    # Falls back to blank RViz2 if the config has not been saved yet
    # Disable with use_rviz:=false
    # -------------------------------------------------------------------------
    rviz_args = ["-d", rviz_config_file] if os.path.isfile(rviz_config_file) else []

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=rviz_args,
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            use_rviz_arg,
            cartographer,
            nav2_delayed,
            thermocator_delayed,
            rviz,
        ]
    )
