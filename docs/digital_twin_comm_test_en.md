# Digital Twin Communication Test Guide

This guide tests only the communication path between the robot-side stack and
the digital twin.

## Goal

Verify that the following communication chain works:

```text
Domain 38: /map, /thermal_map, /global_costmap/costmap, /odom
        -> domain_bridge
Domain 1: advisory_node
        -> /advisory/goal
        -> domain_bridge
Domain 38: decision_node
```

Domain meanings:

| Domain | Meaning |
|---|---|
| `38` | Physical robot stack, or the robot-side simulation in the home setup |
| `1` | Gazebo digital twin, advisory node, and pose sync node |

## 0. Source The Environment In Every Terminal

Inside Docker:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source /ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

On the lab laptop without Docker:

```bash
source /opt/ros/jazzy/setup.bash
source ~/cbl-digital-twins/jazzy_tb3_packages/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

Build once before testing:

```bash
colcon build --packages-select my_tb3_world thermocator --symlink-install
source install/setup.bash
```

Pass condition:

- The build finishes without errors.

If the build fails:

- Make sure ROS Jazzy has been sourced.
- Make sure `ros-jazzy-domain-bridge` is installed.
- If digital twin nodes are missing, rebuild with DT support:

```bash
colcon build --packages-select my_tb3_world thermocator --symlink-install --cmake-args -DBUILD_DT=ON
```

## 1. Start The Domain 38 Robot-Side Stack

Home/Docker robot-side simulation:

```bash
scripts/bin/dock remote sim
```

Open another terminal:

```bash
scripts/bin/dock remote thermulator
```

Lab physical TurtleBot3:

```bash
robot bringup <robot_ip>
```

Open another terminal:

```bash
robot thermulator
```

Check Domain 38 topics:

```bash
ROS_DOMAIN_ID=38 ros2 topic list | grep -E '^/(scan|odom|map|thermal_map|global_costmap/costmap)$'
```

Pass condition:

- `/scan`, `/odom`, `/map`, and `/thermal_map` are visible.
- `/global_costmap/costmap` appears after Nav2 is running.

Check that data is actually published:

```bash
ROS_DOMAIN_ID=38 ros2 topic echo /scan --once
ROS_DOMAIN_ID=38 ros2 topic echo /odom --once
ROS_DOMAIN_ID=38 ros2 topic echo /map --once --qos-durability transient_local
ROS_DOMAIN_ID=38 ros2 topic echo /thermal_map --once --qos-durability transient_local
```

If `/scan` or `/odom` is missing:

- The robot bringup or robot-side simulation is not running correctly.
- On the real robot, check Wi-Fi, robot bringup, and `ROS_DOMAIN_ID=38`.
- In Docker, make sure the `scripts/bin/dock remote sim` terminal is still running.

If `/map` is missing:

- Cartographer or the map server is not running.
- Check:

```bash
ROS_DOMAIN_ID=38 ros2 node list | grep -E 'cartographer|map'
ROS_DOMAIN_ID=38 ros2 topic echo /submap_list --once
```

If `/thermal_map` is missing:

- The thermal pipeline is not running, or it has not received `/map` yet.
- Check:

```bash
ROS_DOMAIN_ID=38 ros2 node list | grep -E 'thermal|thermocator'
ROS_DOMAIN_ID=38 ros2 topic echo /thermal_reading --once
ROS_DOMAIN_ID=38 ros2 topic info /thermal_map -v
```

## 2. Start The Domain 1 Digital Twin Gazebo

Home/Docker:

```bash
scripts/bin/dock remote dt
```

Lab laptop:

```bash
robot sim
```

Check Domain 1 Gazebo topics:

```bash
ROS_DOMAIN_ID=1 ros2 topic list | grep -E '^/(clock|scan|odom|tf|tf_static)$'
ROS_DOMAIN_ID=1 ros2 topic echo /clock --once
ROS_DOMAIN_ID=1 ros2 topic echo /scan --once
```

Pass condition:

- `/clock` publishes data.
- `/scan` publishes data.

If `/clock` has two publishers:

```bash
ROS_DOMAIN_ID=1 ros2 topic info /clock -v
```

Likely cause:

- A duplicate ROS-Gazebo bridge is running.

Fix:

- Stop extra Gazebo or bridge terminals.
- Keep only one digital twin Gazebo launch running.

If `/clock` or `/scan` is missing:

- Gazebo digital twin is not running, or it launched on the wrong domain.
- Check:

```bash
ROS_DOMAIN_ID=1 ros2 node list
ROS_DOMAIN_ID=1 ros2 service list | grep set_pose
```

The pose sync service should usually be:

```text
/world/thermaria/set_pose
```

## 3. Start The Domain Bridge

Open another terminal.

Home/Docker:

```bash
scripts/bin/dock remote bridge
```

Lab laptop:

```bash
robot bridge
```

Important:

- Do not manually set `ROS_DOMAIN_ID` in the bridge terminal.
- `domain_bridge` manages Domain 38 and Domain 1 internally.

Check the bridge process:

```bash
pgrep -af domain_bridge
```

Pass condition:

- One active `domain_bridge` process is running.
- A stale `<defunct>` process can be ignored if there is also a live process.

If the bridge exits immediately:

- Check the config file:

```bash
sed -n '1,160p' src/thermocator/config/domain_bridge.yaml
```

The correct format should look like this:

```yaml
name: thermocator_domain_bridge
topics:
  map:
    type: nav_msgs/msg/OccupancyGrid
    from_domain: 38
    to_domain: 1
```

If the bridge is running but Domain 1 does not show bridged topics:

- Confirm that the original Domain 38 topics exist.
- Stop the old bridge and restart it.
- Check QoS on Domain 38:

```bash
ROS_DOMAIN_ID=38 ros2 topic info /map -v
ROS_DOMAIN_ID=38 ros2 topic info /thermal_map -v
```

## 4. Test Domain 38 To Domain 1

Check whether robot-side topics are available in the digital twin domain:

```bash
ROS_DOMAIN_ID=1 ros2 topic list | grep -E '^/(map|thermal_map|global_costmap/costmap|odom)$'
```

Pass condition:

```text
/global_costmap/costmap
/map
/odom
/thermal_map
```

Test `/map`:

```bash
ROS_DOMAIN_ID=1 ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once --qos-reliability reliable --qos-durability transient_local --qos-history keep_last --qos-depth 1
```

Test `/thermal_map`:

```bash
ROS_DOMAIN_ID=1 ros2 topic echo /thermal_map nav_msgs/msg/OccupancyGrid --once --qos-reliability reliable --qos-durability transient_local --qos-history keep_last --qos-depth 1
```

Pass condition:

- Both commands print `header`, `info`, `width`, `height`, and `data`.
- `frame_id` should normally be `map`.

If `/map` appears in `topic list` but `echo` blocks:

- Use the full command above with explicit message type and QoS.
- `/map` and `/thermal_map` use transient local durability, so a default echo can wait indefinitely.

If Domain 38 has `/map` but Domain 1 does not:

- The bridge may still be using an old config.
- Rebuild and restart the bridge:

```bash
colcon build --packages-select thermocator --symlink-install
source install/setup.bash
```

Then press `Ctrl+C` in the bridge terminal and run:

```bash
robot bridge
```

Or, in Docker:

```bash
scripts/bin/dock remote bridge
```

## 5. Start Advisory And Pose Sync

Home/Docker:

```bash
scripts/bin/dock remote delta_thermal
```

Lab laptop:

```bash
robot dt
```

Check Domain 1 advisory topics:

```bash
ROS_DOMAIN_ID=1 ros2 topic list | grep advisory
```

Pass condition:

```text
/advisory/candidates
/advisory/goal
```

Check whether advisory subscribes to the bridged `/map`:

```bash
ROS_DOMAIN_ID=1 ros2 topic info /map -v
```

Pass condition:

- Publisher includes `thermocator_domain_bridge_1`.
- Subscriber includes `advisory_node`.

If `/advisory/goal` is missing on Domain 1:

- `advisory_node` is not running.
- Check:

```bash
ROS_DOMAIN_ID=1 ros2 node list | grep advisory
```

If `advisory_node` is running but no goal is published:

- This is not necessarily a communication failure. There may be no valid candidate yet.
- Check candidates first:

```bash
ROS_DOMAIN_ID=1 ros2 topic echo /advisory/candidates --once
```

## 6. Test Domain 1 Back To Domain 38

Check whether advisory goals return to the robot side:

```bash
ROS_DOMAIN_ID=38 ros2 topic list | grep advisory
ROS_DOMAIN_ID=38 ros2 topic info /advisory/goal -v
```

Pass condition:

- Publisher count is `1`.
- Publisher node is `thermocator_domain_bridge_38`.
- Subscriber includes `decision_node`.

Expected output pattern:

```text
Publisher count: 1
Node name: thermocator_domain_bridge_38

Subscription count: 1
Node name: decision_node
```

If `/advisory/goal` exists but `echo` blocks:

```bash
ROS_DOMAIN_ID=38 ros2 topic echo /advisory/goal --once
```

This can be normal. `/advisory/goal` only prints when a new advisory goal is published.

For communication testing, prefer:

```bash
ROS_DOMAIN_ID=38 ros2 topic info /advisory/goal -v
```

If Domain 38 has no publisher:

- The bridge is not running, or Domain 1 has no `/advisory/goal` publisher.
- Check both sides:

```bash
ROS_DOMAIN_ID=1 ros2 topic info /advisory/goal -v
ROS_DOMAIN_ID=38 ros2 topic info /advisory/goal -v
```

## 7. Test Pose Sync

Check the Gazebo set-pose service:

```bash
ROS_DOMAIN_ID=1 ros2 service list | grep set_pose
```

Pass condition:

```text
/world/thermaria/set_pose
```

Check `pose_sync_node`:

```bash
ROS_DOMAIN_ID=1 ros2 node list | grep pose_sync
```

If pose sync waits for the service forever:

- Check the actual world name:

```bash
ROS_DOMAIN_ID=1 ros2 service list | grep /world
```

- If the world is not `thermaria`, launch with the actual world name:

```bash
ros2 launch thermocator delta_thermal.launch.py world_name:=<world_name>
```

If the service exists but the Gazebo robot does not follow:

- Check the Gazebo entity name.
- The default is:

```text
turtlebot3_burger
```

- If the actual name is different, launch with:

```bash
ros2 launch thermocator delta_thermal.launch.py robot_entity_name:=<entity_name>
```

## 8. Final Pass Criteria

The digital twin communication test passes when all of these are true:

- Domain 38 can echo `/map`.
- Domain 38 can echo `/thermal_map`.
- Domain 1 can echo `/map`.
- Domain 1 can echo `/thermal_map`.
- Domain 1 has `/advisory/goal`.
- Domain 38 sees `/advisory/goal` from `thermocator_domain_bridge_38`.
- Domain 38 `decision_node` subscribes to `/advisory/goal`.
- Domain 1 has `/world/thermaria/set_pose`, or `pose_sync_node` is launched with the correct `world_name`.

## 9. Quick Health Check

Use these commands when you only need a fast communication check:

```bash
ROS_DOMAIN_ID=38 ros2 topic echo /map --once --qos-durability transient_local
ROS_DOMAIN_ID=1 ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once --qos-reliability reliable --qos-durability transient_local --qos-history keep_last --qos-depth 1
ROS_DOMAIN_ID=1 ros2 topic echo /thermal_map nav_msgs/msg/OccupancyGrid --once --qos-reliability reliable --qos-durability transient_local --qos-history keep_last --qos-depth 1
ROS_DOMAIN_ID=38 ros2 topic info /advisory/goal -v
ROS_DOMAIN_ID=1 ros2 service list | grep set_pose
```

