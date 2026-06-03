# Digital Twin 通信测试文档

这份文档只测试 **physical robot / robot-side stack 和 digital twin 之间的通信**。

## 目标

确认下面这条链路是通的：

```text
Domain 38: /map, /thermal_map, /global_costmap/costmap, /odom
        -> domain_bridge
Domain 1: advisory_node
        -> /advisory/goal
        -> domain_bridge
Domain 38: decision_node
```

Domain 含义：

| Domain | 含义 |
|---|---|
| `38` | physical robot stack，或者 home setup 里的 robot-side simulation |
| `1` | Gazebo digital twin 和 advisory/pose_sync 节点 |

## 0. 每个终端先 source 环境

Docker 容器里：

```bash
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source /ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

实验室电脑上，如果不用 Docker：

```bash
source /opt/ros/jazzy/setup.bash
source ~/cbl-digital-twins/jazzy_tb3_packages/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

测试前先 build 一次：

```bash
colcon build --packages-select my_tb3_world thermocator --symlink-install
source install/setup.bash
```

通过标准：

- build 没有报错。

如果 build 失败：

- 确认已经 source ROS Jazzy。
- 确认安装了 `ros-jazzy-domain-bridge`。
- 如果提示找不到 digital twin 相关节点，重新用 DT 选项 build：

```bash
colcon build --packages-select my_tb3_world thermocator --symlink-install --cmake-args -DBUILD_DT=ON
```

## 1. 启动 Domain 38 机器人侧

Home/Docker 模拟机器人侧：

```bash
scripts/bin/dock remote sim
```

另开一个终端：

```bash
scripts/bin/dock remote thermulator
```

实验室真实 TurtleBot3：

```bash
robot bringup <robot_ip>
```

另开一个终端：

```bash
robot thermulator
```

检查 Domain 38 上的 topic：

```bash
ROS_DOMAIN_ID=38 ros2 topic list | grep -E '^/(scan|odom|map|thermal_map|global_costmap/costmap)$'
```

通过标准：

- 能看到 `/scan`、`/odom`、`/map`、`/thermal_map`。
- `/global_costmap/costmap` 会在 Nav2 跑起来之后出现。

继续检查数据是否真的能收到：

```bash
ROS_DOMAIN_ID=38 ros2 topic echo /scan --once
ROS_DOMAIN_ID=38 ros2 topic echo /odom --once
ROS_DOMAIN_ID=38 ros2 topic echo /map --once --qos-durability transient_local
ROS_DOMAIN_ID=38 ros2 topic echo /thermal_map --once --qos-durability transient_local
```

如果 `/scan` 或 `/odom` 没有：

- 机器人 bringup 或 robot-side simulation 没有正常运行。
- 真机上检查 Wi-Fi、robot bringup、`ROS_DOMAIN_ID=38`。
- Docker 里检查 `scripts/bin/dock remote sim` 那个终端是不是还在运行。

如果 `/map` 没有：

- Cartographer 或 map server 没跑起来。
- 检查：

```bash
ROS_DOMAIN_ID=38 ros2 node list | grep -E 'cartographer|map'
ROS_DOMAIN_ID=38 ros2 topic echo /submap_list --once
```

如果 `/thermal_map` 没有：

- thermal pipeline 没跑起来，或者还没拿到 `/map`。
- 检查：

```bash
ROS_DOMAIN_ID=38 ros2 node list | grep -E 'thermal|thermocator'
ROS_DOMAIN_ID=38 ros2 topic echo /thermal_reading --once
ROS_DOMAIN_ID=38 ros2 topic info /thermal_map -v
```

## 2. 启动 Domain 1 Digital Twin Gazebo

Home/Docker：

```bash
scripts/bin/dock remote dt
```

实验室电脑：

```bash
robot sim
```

检查 Domain 1 Gazebo topic：

```bash
ROS_DOMAIN_ID=1 ros2 topic list | grep -E '^/(clock|scan|odom|tf|tf_static)$'
ROS_DOMAIN_ID=1 ros2 topic echo /clock --once
ROS_DOMAIN_ID=1 ros2 topic echo /scan --once
```

通过标准：

- `/clock` 有数据。
- `/scan` 有数据。

如果 `/clock` 有两个 publisher：

```bash
ROS_DOMAIN_ID=1 ros2 topic info /clock -v
```

可能原因：

- 启动了重复的 ROS-Gazebo bridge。

解决方法：

- 停掉多余的 Gazebo/bridge 终端。
- 只保留一个 digital twin Gazebo launch。

如果 `/clock` 或 `/scan` 没有：

- Gazebo digital twin 没跑起来，或者跑到了错误的 domain。
- 检查：

```bash
ROS_DOMAIN_ID=1 ros2 node list
ROS_DOMAIN_ID=1 ros2 service list | grep set_pose
```

pose sync 需要的服务一般应该是：

```text
/world/thermaria/set_pose
```

## 3. 启动 Domain Bridge

另开一个终端。

Home/Docker：

```bash
scripts/bin/dock remote bridge
```

实验室电脑：

```bash
robot bridge
```

注意：

- 不要手动给 bridge 终端设置 `ROS_DOMAIN_ID`。
- `domain_bridge` 自己会同时管理 Domain 38 和 Domain 1。

检查 bridge 进程：

```bash
pgrep -af domain_bridge
```

通过标准：

- 有一个正在运行的 `domain_bridge` 进程。
- 如果看到一个 `<defunct>`，但同时还有一个 live process，可以先忽略。

如果 bridge 一启动就退出：

- 检查配置文件：

```bash
sed -n '1,160p' src/thermocator/config/domain_bridge.yaml
```

正确格式应该类似：

```yaml
name: thermocator_domain_bridge
topics:
  map:
    type: nav_msgs/msg/OccupancyGrid
    from_domain: 38
    to_domain: 1
```

如果 bridge 在跑，但 Domain 1 看不到 topic：

- 先确认 Domain 38 上原始 topic 存在。
- 停掉旧 bridge，重新启动 bridge。
- 检查 Domain 38 上的 QoS：

```bash
ROS_DOMAIN_ID=38 ros2 topic info /map -v
ROS_DOMAIN_ID=38 ros2 topic info /thermal_map -v
```

## 4. 测试 Domain 38 到 Domain 1

检查 robot-side topic 有没有被桥到 digital twin domain：

```bash
ROS_DOMAIN_ID=1 ros2 topic list | grep -E '^/(map|thermal_map|global_costmap/costmap|odom)$'
```

通过标准：

```text
/global_costmap/costmap
/map
/odom
/thermal_map
```

测试 `/map`：

```bash
ROS_DOMAIN_ID=1 ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once --qos-reliability reliable --qos-durability transient_local --qos-history keep_last --qos-depth 1
```

测试 `/thermal_map`：

```bash
ROS_DOMAIN_ID=1 ros2 topic echo /thermal_map nav_msgs/msg/OccupancyGrid --once --qos-reliability reliable --qos-durability transient_local --qos-history keep_last --qos-depth 1
```

通过标准：

- 两个命令都能打印 `header`、`info`、`width`、`height`、`data`。
- `frame_id` 正常应该是 `map`。

如果 `topic list` 里有 `/map`，但 `echo` 一直卡住：

- 用上面带 message type 和 QoS 的完整命令。
- `/map` 和 `/thermal_map` 是 `transient_local` 类型，默认 echo 有时会等不到数据。

如果 Domain 38 有 `/map`，Domain 1 没有 `/map`：

- bridge 可能没重启，仍然用旧配置。
- 重新 build 并重启 bridge：

```bash
colcon build --packages-select thermocator --symlink-install
source install/setup.bash
```

然后在 bridge 终端按 `Ctrl+C`，再重新跑：

```bash
robot bridge
```

或者 Docker 里：

```bash
scripts/bin/dock remote bridge
```

## 5. 启动 Advisory 和 Pose Sync

Home/Docker：

```bash
scripts/bin/dock remote delta_thermal
```

实验室电脑：

```bash
robot dt
```

检查 Domain 1 advisory topic：

```bash
ROS_DOMAIN_ID=1 ros2 topic list | grep advisory
```

通过标准：

```text
/advisory/candidates
/advisory/goal
```

检查 advisory 是否订阅到了 bridge 过来的 `/map`：

```bash
ROS_DOMAIN_ID=1 ros2 topic info /map -v
```

通过标准：

- Publisher 里有 `thermocator_domain_bridge_1`。
- Subscriber 里有 `advisory_node`。

如果 Domain 1 上没有 `/advisory/goal`：

- `advisory_node` 没跑。
- 检查：

```bash
ROS_DOMAIN_ID=1 ros2 node list | grep advisory
```

如果 `advisory_node` 在跑，但一直没有 goal：

- 这不一定是通信问题，可能是当前没有合适的 candidate。
- 先看 candidates：

```bash
ROS_DOMAIN_ID=1 ros2 topic echo /advisory/candidates --once
```

## 6. 测试 Domain 1 回到 Domain 38

检查 advisory goal 是否回到机器人侧：

```bash
ROS_DOMAIN_ID=38 ros2 topic list | grep advisory
ROS_DOMAIN_ID=38 ros2 topic info /advisory/goal -v
```

通过标准：

- Publisher count 是 `1`。
- Publisher node 是 `thermocator_domain_bridge_38`。
- Subscriber 里有 `decision_node`。

正确结果大概长这样：

```text
Publisher count: 1
Node name: thermocator_domain_bridge_38

Subscription count: 1
Node name: decision_node
```

如果 `/advisory/goal` 存在，但 echo 一直卡住：

```bash
ROS_DOMAIN_ID=38 ros2 topic echo /advisory/goal --once
```

这可能是正常的。`/advisory/goal` 只有在 advisory node 发布新 goal 时才会打印。

通信测试优先看：

```bash
ROS_DOMAIN_ID=38 ros2 topic info /advisory/goal -v
```

如果 Domain 38 没有 publisher：

- bridge 没跑，或者 Domain 1 上没有 `/advisory/goal` 发布者。
- 两边都查：

```bash
ROS_DOMAIN_ID=1 ros2 topic info /advisory/goal -v
ROS_DOMAIN_ID=38 ros2 topic info /advisory/goal -v
```

## 7. 测试 Pose Sync

检查 Gazebo set-pose 服务：

```bash
ROS_DOMAIN_ID=1 ros2 service list | grep set_pose
```

通过标准：

```text
/world/thermaria/set_pose
```

检查 `pose_sync_node`：

```bash
ROS_DOMAIN_ID=1 ros2 node list | grep pose_sync
```

如果 pose sync 一直等待服务：

- 检查实际 world name：

```bash
ROS_DOMAIN_ID=1 ros2 service list | grep /world
```

- 如果不是 `thermaria`，用实际 world name 启动：

```bash
ros2 launch thermocator delta_thermal.launch.py world_name:=<world_name>
```

如果服务存在，但 Gazebo 里的机器人不跟随：

- 检查 Gazebo entity name。
- 默认是：

```text
turtlebot3_burger
```

- 如果不是这个名字，用实际 entity name 启动：

```bash
ros2 launch thermocator delta_thermal.launch.py robot_entity_name:=<entity_name>
```

## 8. 最终通过标准

Digital twin 通信测试通过，需要满足：

- Domain 38 能 echo `/map`。
- Domain 38 能 echo `/thermal_map`。
- Domain 1 能 echo `/map`。
- Domain 1 能 echo `/thermal_map`。
- Domain 1 有 `/advisory/goal`。
- Domain 38 的 `/advisory/goal` publisher 是 `thermocator_domain_bridge_38`。
- Domain 38 的 `decision_node` 订阅 `/advisory/goal`。
- Domain 1 有 `/world/thermaria/set_pose`，或者 `pose_sync_node` 使用了正确的 `world_name`。

## 9. 快速健康检查命令

如果只想快速确认通信是否还活着，跑这些：

```bash
ROS_DOMAIN_ID=38 ros2 topic echo /map --once --qos-durability transient_local
ROS_DOMAIN_ID=1 ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once --qos-reliability reliable --qos-durability transient_local --qos-history keep_last --qos-depth 1
ROS_DOMAIN_ID=1 ros2 topic echo /thermal_map nav_msgs/msg/OccupancyGrid --once --qos-reliability reliable --qos-durability transient_local --qos-history keep_last --qos-depth 1
ROS_DOMAIN_ID=38 ros2 topic info /advisory/goal -v
ROS_DOMAIN_ID=1 ros2 service list | grep set_pose
```
