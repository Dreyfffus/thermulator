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

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to thermocator_params.yaml. "
        "If found, overrides all individual arguments.",
    )

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    map_frame_arg = DeclareLaunchArgument("map_frame", default_value="map")
    robot_frame_arg = DeclareLaunchArgument(
        "robot_frame", default_value="base_footprint"
    )
    # "LOCAL" on Domain 38, "TWINNED" on Domain 1. The decision node tags every
    # goal candidate it publishes to /thermocator/goals with this source so the
    # arbiter can tell where the goal came from.
    goal_source_arg = DeclareLaunchArgument("goal_source", default_value="LOCAL")
    arrival_radius_arg = DeclareLaunchArgument("arrival_radius", default_value="0.4")

    # Broadcaster
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

    # Thermal map builder
    cold_threshold_arg = DeclareLaunchArgument("cold_threshold", default_value="0.0")
    hot_threshold_arg = DeclareLaunchArgument("hot_threshold", default_value="80.0")
    min_confidence_arg = DeclareLaunchArgument("min_confidence", default_value="0.5")
    publish_rate_arg = DeclareLaunchArgument("publish_rate", default_value="1.0")
    prefill_map_arg = DeclareLaunchArgument("prefill_map", default_value="false")
    prefill_temperature_arg = DeclareLaunchArgument(
        "prefill_temperature", default_value="20.0"
    )

    # Decision node -- Phase 1
    coverage_threshold_arg = DeclareLaunchArgument(
        "coverage_threshold", default_value="0.95"
    )
    sensor_coverage_radius_arg = DeclareLaunchArgument(
        "sensor_coverage_radius", default_value="0.3"
    )
    goal_min_distance_arg = DeclareLaunchArgument(
        "goal_min_distance", default_value="0.5"
    )
    goal_timeout_seconds_arg = DeclareLaunchArgument(
        "goal_timeout_seconds", default_value="30.0"
    )
    rescan_interval_seconds_arg = DeclareLaunchArgument(
        "rescan_interval_seconds", default_value="10.0"
    )
    radius_initial_arg = DeclareLaunchArgument("radius_initial", default_value="1.5")
    radius_step_arg = DeclareLaunchArgument("radius_step", default_value="0.5")
    radius_max_arg = DeclareLaunchArgument("radius_max", default_value="8.0")
    samples_per_cycle_arg = DeclareLaunchArgument(
        "samples_per_cycle", default_value="40"
    )
    corridor_bonus_arg = DeclareLaunchArgument("corridor_bonus", default_value="0.3")

    # Decision node -- Phase 2
    action_zone_heat_threshold_arg = DeclareLaunchArgument(
        "action_zone_heat_threshold", default_value="60.0"
    )
    action_zone_cluster_radius_arg = DeclareLaunchArgument(
        "action_zone_cluster_radius", default_value="1.5"
    )
    action_zone_base_sigma_arg = DeclareLaunchArgument(
        "action_zone_base_sigma", default_value="0.4"
    )
    action_delay_seconds_arg = DeclareLaunchArgument(
        "action_delay_seconds", default_value="1.0"
    )
    max_search_cells_arg = DeclareLaunchArgument(
        "max_search_cells", default_value="200"
    )
    control_rate_arg = DeclareLaunchArgument("control_rate", default_value="1.0")

    def launch_nodes(context):
        params_file = LaunchConfiguration("params_file").perform(context)
        use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"
        goal_source = LaunchConfiguration("goal_source").perform(context)
        arrival_radius = float(LaunchConfiguration("arrival_radius").perform(context))
        use_yaml = os.path.isfile(params_file)

        # goal_source / arrival_radius always come from the launch args so the
        # same params file can be shared by the LOCAL and TWINNED stacks.
        decision_overrides = {
            "use_sim_time": use_sim_time,
            "goal_source": goal_source,
            "arrival_radius": arrival_radius,
        }

        if use_yaml:
            print(f"[thermocator.launch] Loading parameters from: {params_file}")
            broadcaster_params = [params_file, {"use_sim_time": use_sim_time}]
            map_builder_params = [params_file, {"use_sim_time": use_sim_time}]
            decision_params = [params_file, decision_overrides]
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

            def b(key):
                return LaunchConfiguration(key).perform(context).lower() == "true"

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
                    "prefill_map": b("prefill_map"),
                    "prefill_temperature": f("prefill_temperature"),
                }
            ]

            decision_params = [
                {
                    "use_sim_time": use_sim_time,
                    "goal_source": goal_source,
                    "arrival_radius": arrival_radius,
                    "map_frame": s("map_frame"),
                    "robot_frame": s("robot_frame"),
                    "coverage_threshold": f("coverage_threshold"),
                    "sensor_coverage_radius": f("sensor_coverage_radius"),
                    "goal_min_distance": f("goal_min_distance"),
                    "goal_timeout_seconds": f("goal_timeout_seconds"),
                    "rescan_interval_seconds": f("rescan_interval_seconds"),
                    "radius_initial": f("radius_initial"),
                    "radius_step": f("radius_step"),
                    "radius_max": f("radius_max"),
                    "samples_per_cycle": i("samples_per_cycle"),
                    "corridor_bonus": f("corridor_bonus"),
                    "action_zone_heat_threshold": f("action_zone_heat_threshold"),
                    "action_zone_cluster_radius": f("action_zone_cluster_radius"),
                    "action_zone_base_sigma": f("action_zone_base_sigma"),
                    "action_delay_seconds": f("action_delay_seconds"),
                    "max_search_cells": i("max_search_cells"),
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

        return [thermal_broadcaster, thermal_map_builder, decision_node]

    return LaunchDescription(
        [
            params_file_arg,
            use_sim_time_arg,
            map_frame_arg,
            robot_frame_arg,
            goal_source_arg,
            arrival_radius_arg,
            zone_centers_x_arg,
            zone_centers_y_arg,
            zone_peak_temps_arg,
            zone_sigmas_arg,
            cold_threshold_arg,
            hot_threshold_arg,
            min_confidence_arg,
            publish_rate_arg,
            prefill_map_arg,
            prefill_temperature_arg,
            coverage_threshold_arg,
            sensor_coverage_radius_arg,
            goal_min_distance_arg,
            goal_timeout_seconds_arg,
            rescan_interval_seconds_arg,
            radius_initial_arg,
            radius_step_arg,
            radius_max_arg,
            samples_per_cycle_arg,
            corridor_bonus_arg,
            action_zone_heat_threshold_arg,
            action_zone_cluster_radius_arg,
            action_zone_base_sigma_arg,
            action_delay_seconds_arg,
            max_search_cells_arg,
            control_rate_arg,
            OpaqueFunction(function=launch_nodes),
        ]
    )
