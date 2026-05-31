# Thermocator

A ROS2 Jazzy package for thermal temperature mapping and autonomous exploration, designed for the TurtleBot3 Burger robot. It includes nodes for thermal map building, coverage-based navigation, an advisory digital twin, and an RViz2 visualization plugin.

## Prerequisites

Two supported setups:

**Lab laptop (native Ubuntu 24.04)**
- ROS2 Jazzy (full desktop)
- Nav2, Cartographer ROS
- Gazebo with ROS bridge (for digital twin)
- TurtleBot3 packages

**Home (WSL Ubuntu 24.04 + Docker)**
- Docker with the provided `turtlebot3_ws` image
- WSLg or X11 forwarding for display

> [!NOTE]
> Setup instructions are available in `SETUP.pdf` included in this repository, or on Canvas under **"Setting up your workspace in WSL.pdf"**.

---

## Domain Architecture

Two ROS2 domain IDs are in use:

| Domain | Purpose |
|---|---|
| `38` | Physical robot stack (or robot sim in home setup) |
| `1` | Digital twin Gazebo sim + advisory nodes |

The `domain_bridge` node connects the two domains. It must be launched without `ROS_DOMAIN_ID` set — it manages both domains internally.

---

## Build Instructions

### Lab laptop

Source the environment in every terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=38
```

Build:

```bash
# Standard build
colcon build --packages-select thermocator --symlink-install

# With digital twin nodes (requires ros_gz_interfaces)
colcon build --packages-select thermocator --symlink-install \
  --cmake-args -DBUILD_DT=ON
```

> [!TIP]
> `--symlink-install` means changes to yaml and config files take effect immediately without rebuilding.

### Docker (home/WSL)

Start a container:

```bash
dock start
```

Inside the container:

```bash
cd /ws
colcon build --packages-select thermocator my_tb3_world --symlink-install
source install/setup.bash
```

With digital twin nodes:

```bash
colcon build --packages-select thermocator my_tb3_world --symlink-install \
  --cmake-args -DBUILD_DT=ON
```

Or use the script from the host:

```bash
dock remote build               # all packages
dock remote build thermocator   # selective
dock remote build_dt            # with DT nodes
```

---

## Scripts

All scripts are in `scripts/bin/`. Source `scripts/install/setup.bash` to add them to your PATH.

### `robot` — lab laptop

Runs everything locally. `bringup` is the only service that SSH's into the robot.

```bash
robot <service> [robot_ip] [domain_id] [package]
```

`domain_id` defaults to `38`. `robot_ip` is only required for `bringup`.

#### Robot services (SSH)

| Service | What it does |
|---|---|
| `bringup <robot_ip>` | SSH into robot and start TurtleBot3 bringup |

#### Local services — robot stack (Domain 38)

| Service | What it runs |
|---|---|
| `teleop` | Keyboard teleoperation |
| `rviz` | Cartographer SLAM + RViz2 |
| `nav` | Nav2 with thermocator params |
| `lifecycle` | Nav2 lifecycle manager |
| `thermal` | Thermal map builder |
| `broadcaster` | Thermal sensor publisher |
| `decision` | Decision node |
| `thermocator` | Full thermocator pipeline (staggered) |
| `thermulator` | Entire stack: SLAM + Nav2 + thermocator (staggered) |
| `map_save` | Save current map to config folder |

#### Local services — digital twin (Domain 1)

| Service | What it runs |
|-----------------|-------------|
| `sim`           | Gazebo sim on Domain 1 (`delta_thermulator.launch.py`) |
| `advisory`      | Advisory node only |
| `pose_sync`     | Pose sync node only |
| `delta_thermal` | Full DT stack: advisory + pose_sync (`delta_thermal_launch.py`) |

#### Local services — bridge

| Service | What it runs |
|---|---|
| `bridge` | `domain_bridge` connecting Domain 38 ↔ Domain 1 |

#### Build

| Service | What it runs |
|---|---|
| `build [package]` | Standard build |
| `build_dt [package]` | Build with `BUILD_DT=ON` |

---

### `dock` — Docker (home/WSL)

Manages a persistent container (`turtlebot3_container`). Starts it automatically if not running.

```bash
dock <start | attach | remote> [service] [package]
```

| Command | What it does |
|---|---|
| `start` | Start a new interactive container session |
| `attach` | Open a shell into the running container |
| `remote <service>` | Run a service inside the container |

#### Remote services

| Service | What it runs |
|---|---|
| `teleop` | Keyboard teleoperation |
| `sim` | Gazebo simulation (`new_world.launch.py`) |
| `dt` | Gazebo simulation (`delta_thermal.launch.py`) |
| `rviz` | Cartographer SLAM + RViz2 |
| `nav` | Nav2 |
| `lifecycle` | Nav2 lifecycle manager |
| `thermal` | Thermal map builder |
| `broadcaster` | Thermal sensor publisher |
| `decision` | Decision node |
| `launch` | Full thermocator pipeline |
| `stack` | Entire stack: SLAM + Nav2 + thermocator |
| `build [package]` | Standard build |
| `build_dt [package]` | Build with `BUILD_DT=ON` |
| `bridge` | `domain_bridge` (no domain ID — manages both internally) |
| `advisory` | Advisory node only |
| `pose_sync` | Pose sync node only |
| `delta_thermal` | Full DT stack: bridge + advisory + pose_sync |

---

## Launch Files

| File | Package | Purpose |
|---|---|---|
| `thermocator.launch.py` | `thermocator` | Broadcaster + thermal map builder + decision node |
| `thermulator.launch.py` | `thermocator` | Full stack: Cartographer + Nav2 + lifecycle + thermocator (staggered) |
| `delta_thermal.launch.py` | `thermocator` | Digital twin: domain_bridge + advisory + pose_sync |
| `new_world.launch.py` | `my_tb3_world` | Gazebo sim for home/Docker setup |
| `delta_thermulator.launch.py` | `my_tb3_world` | Gazebo sim for lab digital twin (sets Domain 1 internally) |

### thermulator.launch.py

Staggered startup order:

| Time | What starts |
|---|---|
| `0s` | Cartographer SLAM + RViz2 |
| `10s` | Nav2 (`autostart:=false`) |
| `18s` | Nav2 lifecycle manager |
| `25s` | Thermocator pipeline |

```bash
# Simulation
ros2 launch thermocator thermulator.launch.py use_sim_time:=true

# Real robot
ros2 launch thermocator thermulator.launch.py use_sim_time:=false
```

> [!IMPORTANT]
> `use_sim_time:=false` for the real robot — there is no simulation clock on hardware.

### dt_launch.py

```bash
ros2 launch thermocator dt_launch.py
ros2 launch thermocator dt_launch.py world_name:=my_world robot_entity_name:=turtlebot3_burger
```

> [!NOTE]
> To find your world name: `ros2 service list | grep set_pose` — the name appears between `/world/` and `/set_pose`. It can also be found in the `<world name="...">` tag of your `.world` file.

---

## Recommended Launch Order

### Lab — robot only

```
Terminal 1: robot bringup <robot_ip>    ← wait for sensors confirmed
Terminal 2: robot thermulator
```

### Lab — robot + digital twin

```
Terminal 1: robot bringup <robot_ip>    ← wait for sensors confirmed
Terminal 2: robot thermulator
Terminal 3: robot sim                   ← Gazebo on Domain 1
Terminal 4: robot bridge                ← connects Domain 38 ↔ Domain 1
Terminal 5: robot dt                    ← advisory + pose_sync on Domain 1
```

### Home — simulation only

```
Terminal 1: dock remote sim             ← wait for Gazebo up
Terminal 2: dock remote stack
```

### Home — simulating lab setup (two sims)

Run the "robot" sim on Domain 38 and the digital twin on Domain 1:

```
Terminal 1: ROS_DOMAIN_ID=38 ros2 launch my_tb3_world sim_home.launch.py
Terminal 2: ROS_DOMAIN_ID=38 ros2 launch thermocator thermulator.launch.py use_sim_time:=true
Terminal 3: dock remote sim             ← Domain 1
Terminal 4: dock remote bridge          ← no domain ID
Terminal 5: dock remote dt              ← Domain 1
```

---

## Nodes

### `thermal_broadcaster`

Simulates a thermal sensor by publishing temperature readings based on configurable Gaussian heat zones with configurable noise. On real hardware this node is replaced by an actual sensor driver publishing to the same `/thermal_reading` topic.

**Publishes:** `/thermal_reading` (`sensor_msgs/msg/Temperature`)

---

### `thermocator` (thermal map builder)

Receives thermal readings and the occupancy map from Cartographer. For each reading it looks up the robot's position via TF2 and projects the temperature onto the corresponding grid cell. Builds a thermal occupancy map scored 0–100 relative to `cold_threshold` and `hot_threshold`. Unobserved cells are published as `-1`.

**Subscribes:** `/thermal_reading`, `/map`
**Publishes:** `/thermal_map` (`nav_msgs/msg/OccupancyGrid`, transient local)

---

### `decision_node`

Autonomous thermal coverage brain. Operates in two phases:

**Phase 1 — Explorer:** Drives thermal coverage using random candidate sampling within an expanding radius. At each cycle, candidate poses are sampled uniformly within the current radius, filtered for navigability (costmap) and thermal coverage state. The best candidate is chosen by corridor gain — an estimate of how many thermally unsampled cells fall within the sensor's proximity radius along the path to that candidate. If no candidates are found the radius expands until it reaches `radius_max`, at which point Phase 1 completes.

**Phase 2 — Actor:** Clusters thermally hot cells into action zones and visits each in nearest-neighbour order. Zone centroids are nudged to the nearest navigable costmap cell before being sent as Nav2 goals.

The decision node optionally consumes `/advisory/goal` from the digital twin. Advisory goals are used if they arrive within a configurable staleness window; otherwise the node falls back to its own detection.

**Subscribes:** `/map`, `/thermal_map`, `/global_costmap/costmap`, `/advisory/goal`
**Publishes:** `/thermocator/goal_markers`, `/thermocator/action_zones`, `/action_map`
**Action client:** `navigate_to_pose` (Nav2)

---

### `advisory_node` *(digital twin, BUILD_DT=ON)*

Runs on Domain 1. Receives the real robot's maps and TF via `domain_bridge` and runs the same candidate detection logic as the Explorer. Publishes the best candidate goal as an advisory `PoseStamped` on `/advisory/goal` which is bridged back to Domain 38 for the decision node to consume.

Has no Nav2 client and sends no commands to any robot.

**Subscribes:** `/map`, `/thermal_map`, `/global_costmap/costmap`
**Publishes:** `/advisory/goal` (`geometry_msgs/msg/PoseStamped`), `/advisory/candidates` (markers)

---

### `pose_sync_node` *(digital twin, BUILD_DT=ON)*

Runs on Domain 1. Reads the real robot's `map → base_footprint` TF (bridged from Domain 38) and teleports the Gazebo sim robot to match using the `/world/<name>/set_pose` service. Rate-limited to 1 Hz with a deadband filter to avoid contact solver instability.

**Requires:** `ros_gz_interfaces`

---

## Parameter File

All node parameters are in `config/thermocator_params.yaml`. Edit without recompiling when using `--symlink-install`.

```yaml
thermal_broadcaster:
  ros__parameters:
    map_frame: "map"
    robot_frame: "base_footprint"
    publish_rate: 2.5
    noise_stdev: 0.5
    zone_centers_x:  [0.0, 1.5]
    zone_centers_y:  [0.0, -1.0]
    zone_peak_temps: [80.0, 60.0]
    zone_sigmas:     [1.2, 1.2]

thermal_map_builder:
  ros__parameters:
    map_frame: "map"
    robot_frame: "base_footprint"
    cold_threshold: 0.0
    hot_threshold:  80.0
    min_confidence: 0.5
    publish_rate:   1.0
    tf_timeout:     0.1

decision_node:
  ros__parameters:
    map_frame:                "map"
    robot_frame:              "base_footprint"
    coverage_threshold:       0.95
    sensor_coverage_radius:   0.3
    goal_min_distance:        0.5
    goal_timeout_seconds:     30.0
    rescan_interval_seconds:  8.0
    radius_initial:           1.5
    radius_step:              0.5
    radius_max:               8.0
    samples_per_cycle:        40
    corridor_bonus:           0.3
    action_zone_heat_threshold:  60.0
    action_zone_cluster_radius:  1.5
    action_zone_base_sigma:      0.4
    action_delay_seconds:        1.0
    control_rate:                1.0
    advisory_stale_secs:         2.0
```

---

## RViz2 Thermal Display Plugin

The package includes a custom RViz2 plugin that overlays the thermal map on top of the Cartographer map as a color-coded heat visualization.

### Setup

In RViz2:

1. Set **Fixed Frame** to `map`
2. **Add** → `Map` → topic `/map`
3. **File** → **Open Config** → Select `/ws/src/thermocator/config/thermisual.rviz` → If prompted **Discard**

The plugin supports configurable cold and hot colors and transparency from the Displays panel.

---

## Digital Twin Architecture

The digital twin couples a Gazebo simulation to the real robot using domain isolation rather than topic remapping. The two stacks run on separate ROS2 domain IDs and are connected by `domain_bridge`.

```
Domain 38 (robot/robot sim)          Domain 1 (Gazebo DT)
─────────────────────────            ─────────────────────
/map              ──────────────────▶ /map
/thermal_map      ──────────────────▶ /thermal_map
/global_costmap   ──────────────────▶ /global_costmap
/tf, /tf_static   ──────────────────▶ /tf, /tf_static
                  ◀────────────────── /advisory/goal
```

The `advisory_node` on Domain 1 consumes the bridged maps, runs candidate detection, and publishes goal suggestions back to Domain 38. The `pose_sync_node` keeps the Gazebo robot co-located with the real robot for visualization. Neither node sends velocity commands to anything.

> [!IMPORTANT]
> `domain_bridge` must be launched without `ROS_DOMAIN_ID` set in the environment. Setting it will break one side of the bridge. The `robot bridge` and `dock remote bridge` services handle this correctly.
