#!/bin/bash

#Docker Image WRKDIR = /ws, no need to cd there.

source install/setup.bash

export TURTLEBOT3_MODEL=burger

ros2 run turtlebot3_teleop teleop_keyboard
