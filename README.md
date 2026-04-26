# Assignment Package (ROS2 Jazzy)

> Project for Autonomous System Twinning CBL :

This project is edited on WSL Ubuntu-24.04 LST, and compiled with colcon on Docker image of osrf/ros:jazzy-desktop-full.

## Prerequisite

This package needs the full ros2-jazzy installation setup provided with WSL and Docker.
To edit the package you need to also have the minimal installation of ros2-jazzy-base which
provides the headers necessary to develop the application within WSL.

> This can be found on the Module section of Canvas under "Setting up your workspace in WSL.pdf" 
or you can source the included pdf file SETUP.pdf

## Package build instructions

To build the package use included scripts [not yet initialized inside the repository] or do the following:

> Do this to start the docker containing the full jazzy installation:

```bash
> docker run --rm -it --name turtlebot3_container --net=host -e DISPLAY=$DISPLAY -v \
    /tmp/.X11-unix:/tmp/.X11-unix -v /home/c2irr10/turtlebot3_ws:/ws turtlebot3_ws bash
```

> After this step, you can use the scripts linked through the docker working directory or run :

```bash
# Provides the environment for running ros2 commands
> source /opt/ros/jazzy/setup.bash
> source /opt/turtlebot3_ws/install/setup.bash

# This builds the source files of the package
> cd <package name> && colcon build #or 
> colcon build --packages-select <package name>

# This Step is for launching the application.
> source install/setup.bash
> export TURTLEBOT3_MODEL=burger
> ros2 launch <package name> <launch file>.launch.py
```
