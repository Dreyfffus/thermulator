#!/bin/bash
PACKAGE=$1

#Use this like ./build.sh <Package Name>

#This needs to be run only inside the docker container. WSL does not have the proper setup to compile with image compatibility.

source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash

#Now we build only the package we need

echo "Building package $PACKAGE ..."

colcon build --packages-select $PACKAGE --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

