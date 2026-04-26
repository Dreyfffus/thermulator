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

To build the packages do the following:

> Do this to start the docker containing the full jazzy installation:

```bash
> docker run --rm -it --name turtlebot3_container --net=host -e DISPLAY=$DISPLAY -v \
    /tmp/.X11-unix:/tmp/.X11-unix -v /home/c2irr10/turtlebot3_ws:/ws turtlebot3_ws bash
```

> After this step, you can use the scripts linked through the docker working directory or run :

```bash
# Installs the specified dependencies of packages
> rosedep install -i --from-path src --rosdistro jazzy -y #you have to be in the package directory 

# Provides the environment for running ros2 commands
> source /opt/ros/jazzy/setup.bash
> source /opt/turtlebot3_ws/install/setup.bash

# This builds the source files of the package
> cd <package name> && colcon build #or 
> colcon build --packages-select <package name>
```
## Running any package

To run any package you source a `*.launch.py` and run the following :

> Specify the package name in ***package name*** and ***launch file***

```bash
> source install/setup.bash
> export TURTLEBOT3_MODEL=burger
> ros2 launch <package name> <launch file>.launch.py
```

Or alternatively you can run any node with :

> Specify both ***package name*** and ***node name***. You can find their alias and source file in CMakeLists.txt

```bash
> source install/setup.bash
> export TURTLEBOT3_MODEL=burger
> ros2 run <package name> <node name>
```
