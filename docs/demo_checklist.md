# Demo Checklist

## Terminal 1: source environment

```bash
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source /ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

## Terminal 2: launch Gazebo with bridge

```bash
robot sim
```

## Terminal 3: launch physical robot stack

```bash
robot thermulator
```

## Terminal 4: launch domain bridge

```bash
robot bridge
```

## Terminal 5: launch DT advisory and pose sync

```bash
robot dt
```

## Terminal 6: inspect bridge topics

```bash
ROS_DOMAIN_ID=1 ros2 topic echo /odom --once
ROS_DOMAIN_ID=1 ros2 topic echo /map --once
ROS_DOMAIN_ID=1 ros2 topic echo /thermal_map --once
ROS_DOMAIN_ID=38 ros2 topic echo /advisory/goal --once
```

## Check Gazebo bridge topics

```bash
ROS_DOMAIN_ID=1 ros2 topic echo /clock --once
ROS_DOMAIN_ID=1 ros2 topic echo /scan --once
```

Expected result: `/clock`, `/scan`, and `/odom` are visible on Domain 1 while Gazebo, `ros_gz_bridge`, and `domain_bridge` are running.

## Check thermal mapping topics

```bash
ros2 topic echo /thermal_reading --once
ros2 topic echo /thermal_map --once
ros2 topic hz /thermal_reading
ros2 topic hz /thermal_map
```

## Optional RViz safe launch

Use standard TurtleBot3/Nav2/Cartographer RViz displays for map, TF, robot model, laser scan, and goals. Do not add the custom `thermocator/ThermalDisplay` plugin during the safe demo path, because it is a known crash risk.

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
```
