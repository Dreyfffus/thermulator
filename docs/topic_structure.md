# Topic Structure

## Original ROS 2 topics

- `/cmd_vel`: velocity command consumed by the robot or simulation.
- `/odom`: robot odometry.
- `/scan`: laser scan for obstacle detection.
- `/tf`: dynamic transforms.
- `/tf_static`: static transforms.
- `/map`: occupancy grid from SLAM or map server.

## Thermal mapping topics

- `/thermal_reading`: simulated thermal sensor reading from `thermal_broadcaster`.
- `/thermal_map`: occupancy-grid style thermal map from `thermocator`.

## Digital twin bridge topics

- Domain 38 `/map` -> Domain 1 `/map`
- Domain 38 `/thermal_map` -> Domain 1 `/thermal_map`
- Domain 38 `/global_costmap/costmap` -> Domain 1 `/global_costmap/costmap`
- Domain 38 `/odom` -> Domain 1 `/odom`
- Domain 1 `/advisory/goal` -> Domain 38 `/advisory/goal`

## Advisory flow

1. The physical robot stack publishes maps, costmap, thermal map, and odometry on Domain 38.
2. `domain_bridge` republishes those selected topics into Domain 1.
3. `advisory_node` consumes the bridged state and publishes `/advisory/goal` on Domain 1.
4. `domain_bridge` republishes `/advisory/goal` back into Domain 38 for `decision_node`.

## Gazebo DT flow

1. `delta_thermulator.launch.py` starts Gazebo on Domain 1.
2. `ros_gz_bridge` bridges Gazebo `/clock`, `/scan`, `/odom`, `/tf`, and `/tf_static` into ROS on Domain 1.
3. `pose_sync_node` reads bridged `/odom` from Domain 38 and calls `/world/thermaria/set_pose` to keep the Gazebo model aligned.

## Map and thermal flow

1. SLAM or map server publishes `/map`.
2. `thermocator` initializes from `/map`.
3. `thermal_broadcaster` publishes `/thermal_reading`.
4. `thermocator` publishes `/thermal_map`.
5. `domain_bridge` makes `/map` and `/thermal_map` available to Domain 1.
