# Thermocator Digital Twin Test Report

Date: 2026-06-16

## Purpose

This report summarizes the local build, automated test, and runtime communication
checks performed for the Thermocator ROS 2 digital twin workspace. The goal was
to verify that the project builds successfully, that its automated test suite
passes, and that the digital twin running on `ROS_DOMAIN_ID=1` can send decision
outputs into the robot-side stack on `ROS_DOMAIN_ID=38`.

## Environment

- Workspace: `/home/robocup/cbl-digital-twins/jazzy_tb3_packages`
- Validation environment: Docker
- Docker image: `turtlebot3_ws:latest`
- ROS domains:
- `Domain 38`: robot-side stack / local simulation
- `Domain 1`: digital twin Gazebo + independent Thermocator stack

The host machine did not have a native ROS 2 Jazzy installation at
`/opt/ros/jazzy/setup.bash`, so validation was performed in the provided Docker
environment.

## Build and Automated Test Results

The workspace was built inside Docker with ROS 2 Jazzy and the TurtleBot3
workspace sourced.

`colcon build` result:

```text
Summary: 3 packages finished

Packages:
  thermocator_msgs
  my_tb3_world
  thermocator
```

The full automated test run passed successfully.

`colcon test` result:

```text
100% tests passed, 0 tests failed out of 6

Tests passed:
  cppcheck
  flake8
  lint_cmake
  pep257
  uncrustify
  xmllint
```

## Headless Digital Twin Smoke Test

A headless launch of the digital twin Gazebo world was performed in Docker.
Gazebo, `ros_gz_bridge`, entity creation, and `robot_state_publisher` started
successfully.

Observed Domain 1 topics during the smoke test:

```text
/clock
/cmd_vel
/joint_states
/odom
/parameter_events
/robot_description
/rosout
/scan
/tf
/tf_static
```

Observed nodes:

```text
/robot_state_publisher
/ros_gz_bridge
```

This confirms that the digital twin simulation can start and publish the basic
Gazebo bridge topics required by the rest of the stack.

## Runtime System Checks

The user then started the full runtime setup through `scripts/bin/dock` and
inspected topics from an attached Docker shell.

### Domain 38 Robot-Side Topics

Command:

```bash
ROS_DOMAIN_ID=38 ros2 topic list | grep -E '^/(scan|odom|cmd_vel|thermocator/goals|thermocator/state/twinned)$'
```

Observed:

```text
/cmd_vel
/odom
/scan
/thermocator/goals
/thermocator/state/twinned
```

This shows that the robot-side simulation/stack is active and that the bridged
twin mission state and goal topic are present on Domain 38.

### Domain 1 Digital Twin Topics

Command:

```bash
ROS_DOMAIN_ID=1 ros2 topic list | grep -E '^/(clock|scan|odom|cmd_vel|thermocator/goals|thermocator/state/local)$'
```

Observed:

```text
/clock
/cmd_vel
/odom
/scan
/thermocator/state/local
```

The initial filtered check showed the core digital twin simulation topics and
the local mission state bridged from Domain 38 into Domain 1.

### Domain 1 Digital Twin Stack Nodes

Command:

```bash
ROS_DOMAIN_ID=1 ros2 node list | grep -E 'thermocator|decision|thermal|cartographer|planner|controller'
```

Observed:

```text
/cartographer_node
/cartographer_occupancy_grid_node
/controller_server
/decision_node
/planner_server
/thermal_broadcaster
/thermal_map_builder
/thermocator_domain_bridge_1
```

This confirms that the digital twin is not only a Gazebo model. It is running
its own SLAM, Nav2, thermal map, decision, and bridge-related components.

### Domain 1 Mapping and Decision Topics

Observed relevant Domain 1 topics:

```text
/action_map
/global_costmap/costmap
/goal_pose
/local_costmap/costmap
/map
/submap_list
/thermal_map
/thermal_reading
/thermocator/goals
```

This confirms that the digital twin stack produces map, thermal map, costmap,
and goal candidate topics.

## Goal Bridge Verification

The most important digital twin communication check was whether goal candidates
generated on Domain 1 appear on Domain 38.

### Domain 1 Twin Goal Candidate

Command:

```bash
ROS_DOMAIN_ID=1 ros2 topic echo /thermocator/goals --once
```

Observed sample:

```text
pose.header.frame_id: map
pose.pose.position.x: 1.3339702577560866
pose.pose.position.y: -0.2727715481081707
source: TWINNED
score: 93.53957250144425
```

This proves that the digital twin `decision_node` publishes `TWINNED` goal
candidates on Domain 1.

### Domain 38 Bridged Goal Candidates

Command:

```bash
ROS_DOMAIN_ID=38 ros2 topic echo /thermocator/goals
```

Observed source sequence:

```text
source: TWINNED
source: LOCAL
source: TWINNED
source: LOCAL
source: TWINNED
source: LOCAL
source: TWINNED
source: LOCAL
```

This is the key evidence that the digital twin goal stream is bridged into the
robot-side domain. Domain 38 receives both local goal candidates and twin
generated goal candidates on the same `/thermocator/goals` channel.

## Interpretation

The observed alternating `LOCAL` and `TWINNED` messages on Domain 38 mean:

- The Domain 38 `decision_node` publishes `LOCAL` goal candidates.
- The Domain 1 digital twin `decision_node` publishes `TWINNED` goal candidates.
- `domain_bridge` successfully forwards `TWINNED` goal candidates from Domain 1
  to Domain 38.
- The robot-side `goal_arbiter` can compare local and twin-generated candidates.

## Conclusion

The project implements a functional ROS 2 / Gazebo digital twin prototype. The
digital twin runs an independent stack on Domain 1 and contributes decision
outputs to the robot-side Domain 38 stack through `domain_bridge`.

Verified status:

```text
Build:                  PASS
Automated tests/lint:   PASS
Digital twin launch:    PASS
Domain 1 stack active:  PASS
Goal bridge 1 -> 38:    PASS
```

Recommended report wording:

```text
The system implements a ROS 2 based digital twin architecture in which a
Gazebo-based twin runs an independent Thermocator stack and publishes TWINNED
goal candidates. These candidates are bridged into the robot-side domain,
where they appear alongside LOCAL candidates and can be evaluated by the
goal arbiter.
```
