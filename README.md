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
| `38` | Physical robot stack (or robot sim in home setup) + **goal arbiter** |
| `1` | Digital twin: a **full, independent thermocator stack** + Gazebo sim |

Both domains run the *same* thermocator stack (SLAM, Nav2, thermal map builder, decision node). Each decision node publishes scored goal candidates to `/thermocator/goals`; the **goal arbiter** (Domain 38) selects the best one, sends it to Nav2, and labels it `LOCAL` or `TWINNED` in RViz. The twin robot is no longer teleported — it moves along with the real robot's navigation commands via bridged `/cmd_vel`.

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
# Build the message package first (or just build everything), then thermocator
colcon build --packages-select thermocator_msgs thermocator --symlink-install
```

> [!NOTE]
> `thermocator` depends on the `thermocator_msgs` interface package (it defines
> `GoalCandidate`, the message on `/thermocator/goals`). Build `thermocator_msgs`
> first, or build the whole workspace so colcon orders it for you. There is no
> longer a `BUILD_DT` flag — the digital-twin nodes were removed and the twin now
> runs the standard stack.

> [!TIP]
> `--symlink-install` means changes to yaml and config files take effect immediately without rebuilding.

### Docker (home/WSL)

To build the container:

```bash
# ../thermulator
docker build -t turtlebot3_ws .
```

Start a container:

```bash
dock start
```

Inside the container:

```bash
cd /ws
colcon build --packages-select thermocator_msgs thermocator my_tb3_world --symlink-install
source install/setup.bash
```

Or use the script from the host:

```bash
dock remote build               # all packages (recommended — orders deps for you)
dock remote build thermocator   # selective (build thermocator_msgs first)
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
| `thermulator` | LOCAL stack: SLAM + Nav2 + thermocator + arbiter (staggered) |
| `arbiter` | Goal arbiter only |
| `map_save` | Save current map to config folder |

#### Local services — digital twin (Domain 1)

| Service | What it runs |
|-----------------|-------------|
| `sim`           | Twin Gazebo sim on Domain 1 (`delta_thermulator.launch.py`) |
| `twin`          | TWINNED stack: full thermocator (publishes goals) + battery monitor |
| `battery`       | Battery monitor only (logs bridged battery every 10 s) |

#### Local services — bridge

| Service | What it runs |
|---|---|
| `bridge` | `domain_bridge` connecting Domain 38 ↔ Domain 1 |

#### Build

| Service | What it runs |
|---|---|
| `build [package]` | Standard build |

---

### `dock` — Docker (home/WSL)

Manages a persistent container (`turtlebot3_container`). Starts it automatically if not running.

```bash
dock <start | attach | remote> [service] [package]
```

| Command | What it does |
|---|---|
| `setup` | First build session |
| `start` | Start a new interactive container session |
| `attach` | Open a shell into the running container |
| `remote <service>` | Run a service inside the container |

>[!IMPORTANT] 
> The `setup` command should be ran before anything else. It builds the sourced environment
> necessary for all other `dock` commands.

#### Remote services

| Service | What it runs |
|---|---|
| `teleop` | Keyboard teleoperation |
| `sim` | Robot-side Gazebo simulation on Domain 38 (`new_world.launch.py`) |
| `dt` | Digital twin Gazebo simulation on Domain 1 (`delta_thermulator.launch.py`) |
| `rviz` | Cartographer SLAM + RViz2 |
| `nav` | Nav2 |
| `lifecycle` | Nav2 lifecycle manager |
| `thermal` | Thermal map builder |
| `broadcaster` | Thermal sensor publisher |
| `decision` | Decision node |
| `thermocator` | Full thermocator pipeline |
| `thermulator` | LOCAL stack: SLAM + Nav2 + thermocator + arbiter |
| `arbiter` | Goal arbiter only |
| `build [package]` | Standard build |
| `bridge` | `domain_bridge` (no domain ID — manages both internally) |
| `dt` | Twin Gazebo sim (Domain 1, `delta_thermulator.launch.py`) |
| `twin` | TWINNED stack: full thermocator + battery monitor (Domain 1) |
| `battery` | Battery monitor only (Domain 1) |

---

## Launch Files

| File | Package | Purpose |
|---|---|---|
| `thermocator.launch.py` | `thermocator` | Broadcaster + thermal map builder + decision node (`goal_source` arg) |
| `thermulator.launch.py` | `thermocator` | Full stack: Cartographer + Nav2 + lifecycle + thermocator, with optional `goal_arbiter` / `battery_monitor` (staggered) |
| `thermic_bridge.launch.py` | `thermocator` | `domain_bridge` process |
| `new_world.launch.py` | `my_tb3_world` | Robot-side Gazebo sim (home/Docker), isolated gz partition |
| `delta_thermulator.launch.py` | `my_tb3_world` | Twin Gazebo sim (Domain 1, isolated gz partition) |

`thermulator.launch.py` arguments:

| Arg | Default | Meaning |
|---|---|---|
| `goal_source` | `LOCAL` | Tag for this stack's goal candidates (`LOCAL` on D38, `TWINNED` on D1) |
| `run_arbiter` | `true` | Run the `goal_arbiter` (Domain 38 only) |
| `run_battery_monitor` | `false` | Run the `battery_monitor` (twin / Domain 1 only) |

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

### Twin stack (Domain 1)

The twin runs the same `thermulator.launch.py`, tagged `TWINNED`, with the arbiter
off and the battery monitor on:

```bash
ROS_DOMAIN_ID=1 ros2 launch thermocator thermulator.launch.py \
  use_sim_time:=true goal_source:=TWINNED run_arbiter:=false run_battery_monitor:=true
```

---

## Test Documents

| Document | Purpose |
|---|---|
| `docs/digital_twin_comm_test.md` | Chinese step-by-step digital twin communication test and troubleshooting guide |
| `docs/digital_twin_comm_test_en.md` | English step-by-step digital twin communication test and troubleshooting guide |
| `docs/demo_checklist.md` | Full demo launch checklist |
| `docs/test_plan.md` | Broader project test plan |

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
Terminal 2: robot thermulator           ← LOCAL stack + arbiter (Domain 38)
Terminal 3: robot sim                   ← twin Gazebo on Domain 1
Terminal 4: robot bridge                ← connects Domain 38 ↔ Domain 1
Terminal 5: robot twin                  ← TWINNED stack + battery (Domain 1)
```

### Home — simulation only

```
Terminal 1: dock remote sim             ← wait for Gazebo up
Terminal 2: dock remote thermulator
```

### Home — simulating lab setup (two sims)

Run the "robot" sim on Domain 38 and the full digital twin on Domain 1:

```
Terminal 1: dock remote sim             ← robot sim (Domain 38, gz partition thermulator_robot)
Terminal 2: dock remote thermulator     ← LOCAL stack + arbiter (Domain 38)
Terminal 3: dock remote dt              ← twin sim (Domain 1, gz partition thermulator_twin)
Terminal 4: dock remote bridge          ← no domain ID
Terminal 5: dock remote twin            ← TWINNED stack + battery (Domain 1)
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

The decision node no longer drives Nav2 directly. It publishes scored goal
candidates to `/thermocator/goals` (tagged `LOCAL` on Domain 38, `TWINNED` on
Domain 1) and tracks goal completion locally via robot-pose proximity + timeout
(no Nav2 action feedback). The `goal_arbiter` is what actually talks to Nav2.

**Subscribes:** `/map`, `/thermal_map`, `/global_costmap/costmap`
**Publishes:** `/thermocator/goals` (`thermocator_msgs/msg/GoalCandidate`), `/thermocator/action_zones`, `/action_map`

---

### `goal_arbiter` *(Domain 38)*

Collects goal candidates from every decision node on `/thermocator/goals` — `LOCAL`
ones published directly on Domain 38 and `TWINNED` ones bridged from Domain 1.
Each cycle it discards stale candidates, picks the highest-scoring fresh one
(score = corridor gain / zone strength), sends it to Nav2, and draws a labelled
marker (`LOCAL` / `TWINNED`) showing which part of the system produced the active
goal.

**Subscribes:** `/thermocator/goals` (`thermocator_msgs/msg/GoalCandidate`)
**Publishes:** `/thermocator/goal_markers` (markers)
**Action client:** `navigate_to_pose` (Nav2)

---

### `battery_monitor` *(digital twin, Domain 1)*

Part of the "state synced" features. The robot's `/battery_state` is bridged from
Domain 38 to Domain 1 by `domain_bridge`; this node subscribes and logs the latest
level to the terminal every 10 seconds.

**Subscribes:** `/battery_state` (`sensor_msgs/msg/BatteryState`)

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

The digital twin runs a **full, independent thermocator stack** on Domain 1 (its
own SLAM, Nav2, thermal map builder and decision node), mirroring Domain 38. The
two stacks are connected by `domain_bridge`. Because each side produces its own
`/map`, `/thermal_map` and `/global_costmap`, those are **not** bridged (that would
create duplicate publishers). Only three things cross domains:

```
Domain 38 (robot / robot sim + arbiter)     Domain 1 (full twin stack + Gazebo)
──────────────────────────────────────     ──────────────────────────────────
/cmd_vel            ──────────────────────▶ /cmd_vel      (drives the twin sim)
/battery_state      ──────────────────────▶ /battery_state(logged every 10 s)
/thermocator/goals  ◀────────────────────── /thermocator/goals (TWINNED candidates)
```

- Both decision nodes publish scored `GoalCandidate`s to `/thermocator/goals`. The
  twin's are bridged 1 → 38 into the `goal_arbiter`, which selects the best of
  `LOCAL` vs `TWINNED` by score and sends it to Nav2.
- The twin robot is **no longer teleported**. It moves along with the real robot's
  navigation commands via bridged `/cmd_vel`, replayed into Gazebo by `ros_gz_bridge`.
- Battery level is bridged to the twin and logged there every 10 seconds.

> [!IMPORTANT]
> `domain_bridge` must be launched without `ROS_DOMAIN_ID` set in the environment. Setting it will break one side of the bridge. The `robot bridge` and `dock remote bridge` services handle this correctly.

> [!NOTE]
> gz-transport is not isolated by `ROS_DOMAIN_ID`. On the home Docker setup the
> robot sim and twin sim share a host, so each is launched in its own
> `GZ_PARTITION` (`thermulator_robot` / `thermulator_twin`). Without this the two
> Gazebo servers cross-talk and both robots move in unison.
