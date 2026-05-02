#!/bin/bash

echo "Attaching to current session ..."

# Attaching this shell to the current running turtlebot3 container

docker exec -it turtlebot3_container bash

