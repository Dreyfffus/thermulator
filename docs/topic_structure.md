# Topic Structure

## Original ROS 2 topics

- `/cmd_vel`: velocity command consumed by the robot or simulation.
- `/odom`: robot odometry.
- `/scan`: laser scan for obstacle detection.
- `/tf`: dynamic transforms.
- `/tf_static`: static transforms.
- `/map`: occupancy grid from SLAM or map server.
- `/battery_state`: robot battery level (`sensor_msgs/msg/BatteryState`).

## Thermal mapping topics

- `/thermal_reading`: simulated thermal sensor reading from `thermal_broadcaster`.
- `/thermal_map`: occupancy-grid style thermal map from `thermocator`.

## Goal arbitration topics

- `/thermocator/goals` (`thermocator_msgs/msg/GoalCandidate`): scored goal
  candidates published by every decision node. Each carries the goal pose, a
  `source` tag (`LOCAL` / `TWINNED`) and a `score`.
- `/thermocator/goal_markers`: arbiter marker showing the active goal + source.
- `/thermocator/action_zones`, `/action_map`: decision-node action-phase output.

## Phase-sync topics

The robot and twin move through Explore -> Act -> Done together. Each decision
node publishes its `MissionState` on its own source topic and listens to the
peer's (paired one-way so the bridge can't loop):

- `/thermocator/state/local`  (`thermocator_msgs/msg/MissionState`, 38 -> 1)
- `/thermocator/state/twinned` (`thermocator_msgs/msg/MissionState`, 1 -> 38)

`MissionState` carries the phase, the side's currently-detected zones (while
exploring), and the agreed merged zone plan (`plan=true`) once Act begins.

## Digital twin bridge topics

Both domains run a full, independent stack, so `/map`, `/thermal_map` and
`/global_costmap` are produced locally and are **not** bridged. Only:

- Domain 38 `/cmd_vel` -> Domain 1 `/cmd_vel`  (drives the twin sim)
- Domain 38 `/battery_state` -> Domain 1 `/battery_state`  (state synced, logged)
- Domain 1 `/thermocator/goals` -> Domain 38 `/thermocator/goals`  (TWINNED candidates)
- Domain 38 `/thermocator/state/local` -> Domain 1  (phase sync)
- Domain 1 `/thermocator/state/twinned` -> Domain 38  (phase sync)

## Goal flow

1. The Domain 38 decision node publishes `LOCAL` candidates to `/thermocator/goals`.
2. The Domain 1 decision node publishes `TWINNED` candidates; `domain_bridge`
   forwards them 1 -> 38 onto the same topic.
3. `goal_arbiter` (Domain 38) keeps the latest fresh candidate per source, picks
   the highest score, sends it to Nav2, and marks it `LOCAL` / `TWINNED` in RViz.

## Phase-sync flow

1. Both sides explore and continuously advertise their detected zones in
   `MissionState`.
2. The first side to finish exploring becomes the Act coordinator: it merges its
   own zones with the peer's last-reported zones and publishes the agreed plan
   (`plan=true`). On a simultaneous trigger, `LOCAL`'s plan wins.
3. Both sides switch to Act together and action the same agreed set independently
   (each nudging zones to its own navigable cells).
4. The first side to finish acting publishes `PHASE_DONE`; both sides stop.

Because both sides are always in the same phase, the arbiter only ever compares
like-for-like candidate scores.

## Twin motion flow

1. The arbiter's chosen goal drives Domain 38 Nav2, which publishes `/cmd_vel`.
2. `domain_bridge` forwards `/cmd_vel` 38 -> 1.
3. `ros_gz_bridge` (Domain 1, in its own `GZ_PARTITION`) replays it into the twin
   Gazebo, so the twin moves along with the real robot instead of being teleported.

## Map and thermal flow (per domain, independent)

1. SLAM (Cartographer) publishes `/map`.
2. `thermocator` initializes from `/map`.
3. `thermal_broadcaster` publishes `/thermal_reading`.
4. `thermocator` publishes `/thermal_map`.
