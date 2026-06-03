# Test Plan

## 1. Setup and familiarization

- Objective: verify the workspace builds and ROS environment is sourced.
- Command: `colcon build --packages-select my_tb3_world thermocator --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`
- Expected topics/nodes: build artifacts for both packages.
- Pass condition: build succeeds and `source install/setup.bash` exposes the packages.
- Current status: partially implemented.

## 2. Gazebo digital twin simulation

- Objective: launch TurtleBot3 in the custom Gazebo world and optionally inspect in RViz.
- Command: `robot sim` or `ros2 launch my_tb3_world delta_thermulator.launch.py use_sim_time:=true`
- Expected topics/nodes: Gazebo process, `ros_gz_parameter_bridge`, `/clock`.
- Pass condition: Gazebo opens, robot spawns, `/clock` is visible in ROS.
- Current status: implemented; ROS-Gazebo bridge is now launched by `delta_thermulator.launch.py`.

## 3. Simulated movement and sensors

- Objective: verify simulated motion and sensor output.
- Command: `ros2 topic echo /scan --once` and `ros2 topic echo /odom --once`
- Expected topics/nodes: `/scan`, `/odom`, `/tf`.
- Pass condition: scan and odometry messages are received while Gazebo is running.
- Current status: partially implemented, untested.

## 4. Domain bridge

- Objective: verify physical-robot streams on Domain 38 are available to digital-twin nodes on Domain 1, and advisory goals return to Domain 38.
- Command: `robot bridge` or `ros2 launch thermocator thermic_bridge.launch.py`
- Expected topics/nodes: `domain_bridge`, `/map`, `/thermal_map`, `/global_costmap/costmap`, `/odom`, `/advisory/goal`.
- Pass condition: with the bridge running, Domain 1 receives the robot map/state topics and Domain 38 receives `/advisory/goal` when the advisory node publishes.
- Current status: implemented; physical-robot-side tests reportedly pass, DT communication needs runtime verification.

## 5. Bridged Gazebo stream

- Objective: verify at least one ROS/Gazebo bridged stream.
- Command: `ros2 topic echo /clock --once`
- Expected topics/nodes: `/clock` from Gazebo through `ros_gz_bridge`.
- Pass condition: `/clock` publishes while Gazebo is running.
- Current status: implemented, untested.

## 6. Advisory communication

- Objective: verify the DT advisory path from Domain 1 back to the physical robot stack on Domain 38.
- Command: with `robot bridge` running, publish or wait for `/advisory/goal` on Domain 1 and echo it on Domain 38.
- Expected topics/nodes: `advisory_node`, `domain_bridge`, `/advisory/goal`.
- Pass condition: `/advisory/goal` appears in Domain 38 and `decision_node` can consume it.
- Current status: implemented, untested.

## 7. Digital twin pose synchronization

- Objective: verify the Gazebo DT robot follows the physical robot pose.
- Command: `robot dt` or `ros2 launch thermocator delta_thermal.launch.py`
- Expected topics/nodes: `pose_sync_node`, `/odom`, `/world/thermaria/set_pose`.
- Pass condition: `pose_sync_node` receives `/odom` on Domain 1 and successfully calls Gazebo's set-pose service.
- Current status: implemented, likely blocked if `/odom` is not bridged or Gazebo set-pose service name differs.

## 8. Obstacle detection and avoidance

- Objective: verify Nav2 uses `/scan` and avoids Gazebo obstacles.
- Command: `ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true params_file:=/ws/src/thermocator/config/nav2_thermal_params.yaml`
- Expected topics/nodes: Nav2 lifecycle nodes, costmaps, `/scan`.
- Pass condition: robot plans around obstacles in the custom world.
- Current status: partially implemented, untested.

## 9. Object interaction / simplified equivalent

- Objective: satisfy the interaction requirement with a project-specific simplification.
- Command: run Gazebo, Nav2, and DT integration, then send or select a thermal target goal.
- Expected topics/nodes: `/scan`, Nav2 costmaps, `/thermal_map`, `decision_node`.
- Pass condition: TurtleBot3 detects boxes/obstacles through `/scan`, avoids them with Nav2, and treats thermal hotspot/environment state as the object/state of interest.
- Current status: limitation, documented simplified equivalent.

## 10. Scenario testing

- Objective: run a repeatable scenario with Gazebo, ROS-Gazebo bridge, domain bridge, Nav2, thermal mapping, advisory, and pose sync.
- Command: follow `docs/demo_checklist.md`.
- Expected topics/nodes: Gazebo, ROS-Gazebo bridge, `domain_bridge`, Nav2, thermal nodes, advisory node, pose sync node.
- Pass condition: all required topics are active, `/advisory/goal` bridges back to Domain 38, thermal map publishes, and the Gazebo robot pose follows `/odom`.
- Current status: implemented as checklist, untested.

## 11. Final demo preparation

- Objective: provide a clear final demonstration sequence.
- Command: follow `docs/demo_checklist.md`.
- Expected topics/nodes: all demo nodes, bridge topics, `/advisory/goal`, and Gazebo set-pose service.
- Pass condition: demonstration can be repeated from clean terminals.
- Current status: implemented as documentation, untested.
