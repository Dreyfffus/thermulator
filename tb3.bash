#!/bin/bash
attach="$1"
service="${2:-}"

if [ -z "$attach" ]; then
    echo "Usage: $0 <start | attach | remote> [service]"
    exit 1
fi

case "$attach" in
    start)
        echo "Starting session ..."
        docker run -it --name turtlebot3_container --net=host \
            -e DISPLAY=$DISPLAY \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v /home/c2irr10/turtlebot3_ws:/ws \
            turtlebot3_ws bash -c \
            "source /ws/install/setup.bash && \
             export TURTLEBOT3_MODEL=burger && \
             exec bash"
        ;;
    attach)
        echo "Attaching to session ..."
        docker exec -it turtlebot3_container bash -c \
            "source /ws/install/setup.bash && \
             export TURTLEBOT3_MODEL=burger && \
             exec bash"
        ;;
    remote)
        echo "Executing $service ..."
        # All in ONE exec call — separate execs don't share environment
        if [ "$service" == "teleop" ]; then
            docker exec -it turtlebot3_container bash -c \
                "source /ws/install/setup.bash && \
                 export TURTLEBOT3_MODEL=burger && \
                 ros2 run turtlebot3_teleop teleop_keyboard"

        elif [ "$service" == "rviz" ]; then
            docker exec -it turtlebot3_container bash -c \
                "source /ws/install/setup.bash && \
                 export TURTLEBOT3_MODEL=burger && \
                 ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True"

        elif [ "$service" == "sim" ]; then
            docker exec -it turtlebot3_container bash -c \
                "source /ws/install/setup.bash && \
                 export TURTLEBOT3_MODEL=burger && \
                 ros2 launch my_tb3_world new_world.launch.py"

        elif [ "$service" == "nav" ]; then
            docker exec -it turtlebot3_container bash -c \
                "source /ws/install/setup.bash && \
                 export TURTLEBOT3_MODEL=burger && \
                 ros2 launch nav2_bringup navigation2.launch.py use_sim_time:=True"

        elif [ "$service" == "slam" ]; then
        docker exec -it turtlebot3_container bash -c \
                "source /ws/install/setup.bash && \
                 export TURTLEBOT3_MODEL=burger && \
                 ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True"
        else
            echo "Error: unknown service '$service'"
            echo "Usage: $0 remote <teleop | rviz | sim | nav | slam>"
            exit 1
        fi
        ;;
    *)
        echo "Error: first argument must be start | attach | remote"
        echo "Usage: $0 <start | attach | remote> [service]"
        exit 1
        ;;
esac
