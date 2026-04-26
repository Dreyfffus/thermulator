#!/bin/bash

source install/setup.bash

export TURTLEBOT_MODEL=burger

ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
