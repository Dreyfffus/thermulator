# Technical Status

## Existing project components

- `thermocator` provides the thermal map builder node. It waits for `/map`, subscribes to `/thermal_reading`, and publishes `/thermal_map`.
- `thermal_broadcaster` provides a simulated thermal sensor stream on `/thermal_reading`.
- `decision_node` proposes thermal-coverage goals based on frontier/zone logic. It now publishes scored `GoalCandidate`s to `/thermocator/goals` instead of driving Nav2 directly.
- `goal_arbiter` selects the best `LOCAL`/`TWINNED` candidate by score, drives Nav2, and marks the active goal source in RViz.
- `battery_monitor` logs the bridged battery state on the twin every 10 s.
- `thermocator_display` is an RViz display plugin intended to overlay the thermal map.
- `my_tb3_world` provides a custom Gazebo Sim world and launch files for the TurtleBot3 simulation.

## Digital twin design (current)

- Both domains run the same full stack (SLAM + Nav2 + thermocator). The twin is no longer an advisory-only node set.
- `src/thermocator_msgs` defines `GoalCandidate { geometry_msgs/PoseStamped pose, string source, float64 score }`, the message on `/thermocator/goals`. `domain_bridge` can forward it because the type is installed in both domains.
- `src/thermocator/config/domain_bridge.yaml` now bridges only `/cmd_vel` (38ΓåÆ1), `/battery_state` (38ΓåÆ1) and `/thermocator/goals` (1ΓåÆ38). Maps/costmap/odom are produced locally per domain and are no longer bridged.
- `src/thermocator/launch/thermulator.launch.py` takes `goal_source`, `run_arbiter` and `run_battery_monitor` args so the same file launches the LOCAL (D38) and TWINNED (D1) stacks.
- `src/thermocator/launch/thermic_bridge.launch.py` starts the `domain_bridge` process.

## Removed components

- `advisory_node` and `pose_sync_node` were deleted; the twin now runs the real stack and moves via bridged `/cmd_vel` rather than `set_pose` teleporting.
- `delta_thermal.launch.py` was removed (the twin uses `thermulator.launch.py`).
- The `BUILD_DT` CMake option and the `ros_gz_interfaces` dependency were removed.

## Home Docker (two-sim) fix

- gz-transport is not isolated by `ROS_DOMAIN_ID`. The robot sim (`new_world.launch.py`) and twin sim (`delta_thermulator.launch.py`) now each set a unique `GZ_PARTITION` (`thermulator_robot` / `thermulator_twin`), so the two Gazebo servers and their `ros_gz_bridge` instances no longer share gz topics and no longer move both robots in unison. The twin moves only from the `/cmd_vel` explicitly bridged in by `domain_bridge`.

## Remaining untested items

- Score scales differ between Explorer (corridor gain) and Actor (zone strength); cross-source comparison during phase transitions may need tuning.
- The twin runs Nav2 only to provide its costmap; verify its cmd_vel chain stays idle and never fights the bridged `/cmd_vel`.
- End-to-end timing tolerance must be measured during a live demo run.

## Known risks intentionally not fixed here

- The custom RViz thermal display plugin is known to crash visualization. This pass does not modify or fix it.
- Thermal grid math/indexing issues were previously identified. This pass does not modify or fix them.

