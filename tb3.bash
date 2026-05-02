#!/bin/bash

attach="$1"

service="${2:-}"

if  [ -z "$attach" ]; then 
    echo "Usage: $0 <start | attach> [service]"
    exit 1
fi

case "$attach" in 
    start)
        echo "Starting session ..."
        docker run --rm -it --name turtlebot3_container --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/c2irr10/turtlebot3_ws:/ws turtlebot3_ws bash
        ;;
    attach)
        echo "Attaching to session ..."
        docker exec -it turtlebot3_container bash
        ;;
    *)
        echo "Error: first argument must be attach | start"
        echo "Usage: $0 <start | attach> [service]"
        exit 1 
        ;;
esac

if [ -n "$service" ]; then
    exit 1 
fi

source install/setup.bash

export TURTLEBOT3_MODEL=burger

if [ "$service" == "teleop" ]; then
    ros2 run turtlebot3_teleop teleop_keyboard
fi 

if [ "$service" == "rviz" ]; then
    ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
fi

if [ "$service" == "sim" ]; then
    ros2 launch my_tb3_world new_world.launch.py
fi
