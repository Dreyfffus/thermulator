#!/bin/bash

echo "Starting docker image ..."

#Starts the docker image  for the turtlebot3_ws environment. Deletes state on exit with --rm flag. Starts as user c2irr10 (ID:1001)

docker run --rm -it --name turtlebot3_container --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/c2irr10/turtlebot3_ws:/ws turtlebot3_ws bash
