.TL
Thermocator Digital Twin Test Report
.AU
CBL Digital Twins / Jazzy TurtleBot3 Workspace
.DA
June 16, 2026
.NH 1
Purpose
.PP
This report summarizes the local build, lint/test, and runtime communication
checks performed for the Thermocator ROS 2 digital twin workspace.
The goal was to verify that the project builds successfully, that its automated
test suite passes, and that the digital twin running on ROS_DOMAIN_ID 1 can
send decision outputs into the robot-side stack on ROS_DOMAIN_ID 38.
.NH 1
Environment
.PP
The host machine did not have a native ROS 2 Jazzy setup at
/opt/ros/jazzy/setup.bash, so validation was performed in the provided Docker
environment using the turtlebot3_ws image.
.PP
Workspace:
.DS
/home/robocup/cbl-digital-twins/jazzy_tb3_packages
.DE
.PP
Docker image used:
.DS
turtlebot3_ws:latest
.DE
.PP
ROS domains:
.DS
Domain 38: robot-side stack / local simulation
Domain 1 : digital twin Gazebo + independent Thermocator stack
.DE
.NH 1
Build and Automated Test Results
.PP
The workspace was built inside Docker with ROS 2 Jazzy and the TurtleBot3
workspace sourced.
.DS
colcon build result:
  Summary: 3 packages finished

Packages:
  thermocator_msgs
  my_tb3_world
  thermocator
.DE
.PP
The full automated test run passed after fixing lint and XML issues.
.DS
colcon test result:
  100% tests passed, 0 tests failed out of 6

Tests passed:
  cppcheck
  flake8
  lint_cmake
  pep257
  uncrustify
  xmllint
.DE
.PP
Issues fixed before the passing run:
.IP \(bu 2
plugin_description.xml had two top-level library elements. It was wrapped in a
valid class_libraries root element.
.IP \(bu 2
Python launch files were reformatted to pass ament_flake8.
.IP \(bu 2
C++ source and header files were reformatted with ament_uncrustify.
.IP \(bu 2
decision_node.cpp had two missing semicolons that were fixed.
.NH 1
Headless Digital Twin Smoke Test
.PP
A headless launch of the digital twin Gazebo world was performed in Docker.
Gazebo, ros_gz_bridge, entity creation, and robot_state_publisher started
successfully.
.PP
Observed Domain 1 topics during the smoke test:
.DS
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
.DE
.PP
Relevant nodes observed:
.DS
/robot_state_publisher
/ros_gz_bridge
.DE
.PP
This confirms that the digital twin simulation can start and publish the basic
Gazebo bridge topics required by the rest of the stack.
.NH 1
Runtime System Checks
.PP
The user then started the full runtime setup through scripts/bin/dock and
inspected topics from an attached Docker shell.
.NH 2
Domain 38 Robot-Side Topics
.DS
Command:
  ROS_DOMAIN_ID=38 ros2 topic list | grep -E \
  '^/(scan|odom|cmd_vel|thermocator/goals|thermocator/state/twinned)$'

Observed:
  /cmd_vel
  /odom
  /scan
  /thermocator/goals
  /thermocator/state/twinned
.DE
.PP
This shows that the robot-side simulation/stack is active and that the bridged
twin mission state and goal topic are present on Domain 38.
.NH 2
Domain 1 Digital Twin Topics
.DS
Command:
  ROS_DOMAIN_ID=1 ros2 topic list | grep -E \
  '^/(clock|scan|odom|cmd_vel|thermocator/goals|thermocator/state/local)$'

Observed:
  /clock
  /cmd_vel
  /odom
  /scan
  /thermocator/state/local
.DE
.PP
The initial filtered check showed the core digital twin simulation topics and
the local mission state bridged from Domain 38 into Domain 1.
.NH 2
Domain 1 Digital Twin Stack Nodes
.DS
Command:
  ROS_DOMAIN_ID=1 ros2 node list | grep -E \
  'thermocator|decision|thermal|cartographer|planner|controller'

Observed:
  /cartographer_node
  /cartographer_occupancy_grid_node
  /controller_server
  /decision_node
  /planner_server
  /thermal_broadcaster
  /thermal_map_builder
  /thermocator_domain_bridge_1
.DE
.PP
This confirms that the digital twin is not only a Gazebo model. It is running
its own SLAM, Nav2, thermal map, decision, and bridge-related components.
.NH 2
Domain 1 Mapping and Decision Topics
.DS
Observed relevant Domain 1 topics:
  /action_map
  /global_costmap/costmap
  /goal_pose
  /local_costmap/costmap
  /map
  /submap_list
  /thermal_map
  /thermal_reading
  /thermocator/goals
.DE
.PP
This confirms that the digital twin stack produces map, thermal map, costmap,
and goal candidate topics.
.NH 1
Goal Bridge Verification
.PP
The most important digital twin communication check was whether goal candidates
generated on Domain 1 appear on Domain 38.
.NH 2
Domain 1 Twin Goal Candidate
.DS
Command:
  ROS_DOMAIN_ID=1 ros2 topic echo /thermocator/goals --once

Observed sample:
  pose.header.frame_id: map
  pose.pose.position.x: 1.3339702577560866
  pose.pose.position.y: -0.2727715481081707
  source: TWINNED
  score: 93.53957250144425
.DE
.PP
This proves that the digital twin decision_node publishes TWINNED goal
candidates on Domain 1.
.NH 2
Domain 38 Bridged Goal Candidates
.DS
Command:
  ROS_DOMAIN_ID=38 ros2 topic echo /thermocator/goals

Observed source sequence:
  source: TWINNED
  source: LOCAL
  source: TWINNED
  source: LOCAL
  source: TWINNED
  source: LOCAL
  source: TWINNED
  source: LOCAL
.DE
.PP
This is the key evidence that the digital twin goal stream is bridged into the
robot-side domain. Domain 38 receives both local goal candidates and twin
generated goal candidates on the same /thermocator/goals channel.
.NH 1
Interpretation
.PP
The observed alternating LOCAL and TWINNED messages on Domain 38 mean:
.IP \(bu 2
The Domain 38 decision_node publishes LOCAL goal candidates.
.IP \(bu 2
The Domain 1 digital twin decision_node publishes TWINNED goal candidates.
.IP \(bu 2
domain_bridge successfully forwards TWINNED goal candidates from Domain 1 to
Domain 38.
.IP \(bu 2
The robot-side goal_arbiter can compare local and twin-generated candidates.
.NH 1
Conclusion
.PP
The project implements a functional ROS 2 / Gazebo digital twin prototype.
The digital twin runs an independent stack on Domain 1 and contributes decision
outputs to the robot-side Domain 38 stack through domain_bridge.
.PP
Verified status:
.DS
Build:                  PASS
Automated tests/lint:   PASS
Digital twin launch:    PASS
Domain 1 stack active:  PASS
Goal bridge 1 -> 38:    PASS
.DE
.PP
Recommended report wording:
.DS
The system implements a ROS 2 based digital twin architecture in which a
Gazebo-based twin runs an independent Thermocator stack and publishes TWINNED
goal candidates. These candidates are bridged into the robot-side domain,
where they appear alongside LOCAL candidates and can be evaluated by the
goal arbiter.
.DE
