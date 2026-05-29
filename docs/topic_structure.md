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

## Digital twin topics

- `/dt/odom`: mirror of `/odom`.
- `/dt/scan`: mirror of `/scan`.
- `/dt/map`: mirror of `/map`.
- `/dt/thermal_map`: mirror of `/thermal_map`.
- `/dt/thermal_reading`: mirror of `/thermal_reading`.
- `/dt/robot_status`: mirror of `/robot/status`, including operating mode, sensor health, speed, thermal sensor health, and obstacle state.
- `/dt/environment_event`: mirror of `/robot/environment_event`, including whether the robot currently sees a nearby obstacle.
- `/dt/cmd_vel`: digital-twin command input forwarded to `/cmd_vel`.

## Command flow

1. A test operator, demo script, or future DT controller publishes `geometry_msgs/msg/Twist` to `/dt/cmd_vel`.
2. `dt_mediator` wraps the command in `geometry_msgs/msg/TwistStamped` and republishes it to `/cmd_vel`.
3. The robot or Gazebo simulation consumes `/cmd_vel`.

## Sensor flow

1. Gazebo or the robot publishes sensor/pose topics such as `/scan` and `/odom`.
2. `dt_mediator` mirrors those topics to `/dt/scan` and `/dt/odom`.
3. `sync_monitor` compares original and mirrored streams.

## State and environment flow

1. `status_monitor` subscribes to `/scan`, `/odom`, and `/thermal_reading`.
2. It publishes `/robot/status` with internal status such as mode, sensor health, movement state, and obstacle detection state.
3. It publishes `/robot/environment_event` when the scan indicates a nearby obstacle or a clear path.
4. `dt_mediator` mirrors these streams to `/dt/robot_status` and `/dt/environment_event`.
5. `sync_monitor` compares the original and mirrored status streams using arrival-time delay.

## Map and thermal flow

1. SLAM or map server publishes `/map`.
2. `thermocator` initializes from `/map`.
3. `thermal_broadcaster` publishes `/thermal_reading`.
4. `thermocator` publishes `/thermal_map`.
5. `dt_mediator` mirrors `/map`, `/thermal_reading`, and `/thermal_map` into `/dt/*` topics.
