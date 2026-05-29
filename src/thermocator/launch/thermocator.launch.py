#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory("thermocator")
    default_params_file = os.path.join(pkg, "config", "thermocator_params.yaml")

    # -------------------------------------------------------------------------
    # Arguments
    # If params_file is provided and exists on disk, it overrides everything.
    # If not found, individual arguments and their defaults are used instead.
    # -------------------------------------------------------------------------

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to thermocator_params.yaml. "
        "If the file exists it overrides all individual arguments.",
    )

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    map_frame_arg = DeclareLaunchArgument("map_frame", default_value="map")
    robot_frame_arg = DeclareLaunchArgument(
        "robot_frame", default_value="base_footprint"
    )
    zone_centers_x_arg = DeclareLaunchArgument(
        "zone_centers_x", default_value="[0.0, 1.5]"
    )
    zone_centers_y_arg = DeclareLaunchArgument(
        "zone_centers_y", default_value="[0.0, -1.0]"
    )
    zone_peak_temps_arg = DeclareLaunchArgument(
        "zone_peak_temps", default_value="[80.0, 60.0]"
    )
    zone_sigmas_arg = DeclareLaunchArgument("zone_sigmas", default_value="[1.2, 1.2]")
    cold_threshold_arg = DeclareLaunchArgument("cold_threshold", default_value="0.0")
    hot_threshold_arg = DeclareLaunchArgument("hot_threshold", default_value="80.0")
    min_confidence_arg = DeclareLaunchArgument("min_confidence", default_value="0.5")
    publish_rate_arg = DeclareLaunchArgument("publish_rate", default_value="1.0")
    heat_detection_threshold_arg = DeclareLaunchArgument(
        "heat_detection_threshold", default_value="20.0"
    )
    scoring_radius_arg = DeclareLaunchArgument("scoring_radius", default_value="1.5")
    w_unknown_hot_arg = DeclareLaunchArgument("w_unknown_hot", default_value="3.0")
    w_dist_hottest_arg = DeclareLaunchArgument("w_dist_hottest", default_value="2.0")
    w_cold_penalty_arg = DeclareLaunchArgument("w_cold_penalty", default_value="0.2")
    revisit_penalty_radius_arg = DeclareLaunchArgument(
        "revisit_penalty_radius", default_value="0.8"
    )
    max_visited_goals_arg = DeclareLaunchArgument(
        "max_visited_goals", default_value="10"
    )
    frontier_min_distance_arg = DeclareLaunchArgument(
        "frontier_min_distance", default_value="0.8"
    )
    investigation_duration_arg = DeclareLaunchArgument(
        "investigation_duration", default_value="5.0"
    )
    control_rate_arg = DeclareLaunchArgument("control_rate", default_value="1.0")

    # -------------------------------------------------------------------------
    # OpaqueFunction resolves at launch time whether to use yaml or arguments
    # -------------------------------------------------------------------------
    def launch_nodes(context):
        params_file = LaunchConfiguration("params_file").perform(context)
        use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"
        use_yaml = os.path.isfile(params_file)

        if use_yaml:
            print(f"[thermocator.launch] Loading parameters from: {params_file}")
            broadcaster_params = [params_file, {"use_sim_time": use_sim_time}]
            map_builder_params = [params_file, {"use_sim_time": use_sim_time}]
            decision_params = [params_file, {"use_sim_time": use_sim_time}]

        else:
            print(
                f"[thermocator.launch] No yaml at {params_file} -- using launch arguments"
            )

            def floats(key):
                return [
                    float(v)
                    for v in LaunchConfiguration(key)
                    .perform(context)
                    .strip("[]")
                    .split(",")
                ]

            def f(key):
                return float(LaunchConfiguration(key).perform(context))

            def i(key):
                return int(LaunchConfiguration(key).perform(context))

            def s(key):
                return LaunchConfiguration(key).perform(context)

            broadcaster_params = [
                {
                    "use_sim_time": use_sim_time,
                    "map_frame": s("map_frame"),
                    "robot_frame": s("robot_frame"),
                    "publish_rate": 2.5,
                    "noise_stdev": 0.5,
                    "zone_centers_x": floats("zone_centers_x"),
                    "zone_centers_y": floats("zone_centers_y"),
                    "zone_peak_temps": floats("zone_peak_temps"),
                    "zone_sigmas": floats("zone_sigmas"),
                }
            ]

            map_builder_params = [
                {
                    "use_sim_time": use_sim_time,
                    "map_frame": s("map_frame"),
                    "robot_frame": s("robot_frame"),
                    "cold_threshold": f("cold_threshold"),
                    "hot_threshold": f("hot_threshold"),
                    "min_confidence": f("min_confidence"),
                    "publish_rate": f("publish_rate"),
                    "tf_timeout": 0.1,
                }
            ]

            decision_params = [
                {
                    "use_sim_time": use_sim_time,
                    "map_frame": s("map_frame"),
                    "robot_frame": s("robot_frame"),
                    "heat_detection_threshold": f("heat_detection_threshold"),
                    "scoring_radius": f("scoring_radius"),
                    "w_unknown_hot": f("w_unknown_hot"),
                    "w_dist_hottest": f("w_dist_hottest"),
                    "w_cold_penalty": f("w_cold_penalty"),
                    "revisit_penalty_radius": f("revisit_penalty_radius"),
                    "max_visited_goals": i("max_visited_goals"),
                    "frontier_min_distance": f("frontier_min_distance"),
                    "investigation_duration": f("investigation_duration"),
                    "control_rate": f("control_rate"),
                }
            ]

        thermal_broadcaster = Node(
            package="thermocator",
            executable="thermal_broadcaster",
            name="thermal_broadcaster",
            output="screen",
            parameters=broadcaster_params,
        )

        thermal_map_builder = TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="thermocator",
                    executable="thermocator",
                    name="thermal_map_builder",
                    output="screen",
                    parameters=map_builder_params,
                )
            ],
        )

        decision_node = TimerAction(
            period=8.0,
            actions=[
                Node(
                    package="thermocator",
                    executable="decision_node",
                    name="decision_node",
                    output="screen",
                    parameters=decision_params,
                )
            ],
        )

        status_monitor = Node(
            package="thermocator",
            executable="status_monitor",
            name="status_monitor",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        )

        return [thermal_broadcaster, thermal_map_builder, decision_node, status_monitor]

    return LaunchDescription(
        [
            # Arguments
            params_file_arg,
            use_sim_time_arg,
            map_frame_arg,
            robot_frame_arg,
            zone_centers_x_arg,
            zone_centers_y_arg,
            zone_peak_temps_arg,
            zone_sigmas_arg,
            cold_threshold_arg,
            hot_threshold_arg,
            min_confidence_arg,
            publish_rate_arg,
            heat_detection_threshold_arg,
            scoring_radius_arg,
            w_unknown_hot_arg,
            w_dist_hottest_arg,
            w_cold_penalty_arg,
            revisit_penalty_radius_arg,
            max_visited_goals_arg,
            frontier_min_distance_arg,
            investigation_duration_arg,
            control_rate_arg,
            # Node launcher
            OpaqueFunction(function=launch_nodes),
        ]
    )
