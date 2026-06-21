import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_thermocator = get_package_share_directory("thermocator")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    nav2_params_file = os.path.join(
        pkg_thermocator, "config", "nav2_thermal_params.yaml"
    )

    map_file = os.path.join(pkg_thermocator, "worlds", "my_map.yaml")

    thermocator_launch_file = os.path.join(
        pkg_thermocator, "launch", "thermocator.launch.py"
    )
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "localization_launch.py")
        ),
        launch_arguments={
            "params_file": nav2_params_file,
            "use_sim_time": "true",
            "map": map_file,
            "autostart": "true",
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "params_file": nav2_params_file,
            "use_sim_time": "true",
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
    thermocator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(thermocator_launch_file),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    thermocator_delayed = TimerAction(period=10.0, actions=[thermocator_launch])

    return LaunchDescription(
        [
            localization_launch,
            navigation_launch,
            lifecycle_manager,
            thermocator_delayed,
        ]
    )
