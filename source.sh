#!/bin/bash
echo "Sourcing..."
source /opt/ros/jazzy/setup.bash
source /opt/turlebot3_ws/install/setup.bash
colcon build
source install/setup.bash
export TURTLEBOT3_MODEL=burger
