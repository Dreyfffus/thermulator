#!/bin/bash

echo "Starting Simulation ..."

source /opt/ros/jazzy/setup.bash
source /opt/turlebot3_ws/install/setup.bash
colcon build
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch my_tb3_world new_world.launch.py
