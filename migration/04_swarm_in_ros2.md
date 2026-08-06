# 04 — Porting the Swarm Architecture to ROS2

## The core problem: 3 drones, 1 ROS2 domain

In ROS2, all nodes on the same `ROS_DOMAIN_ID` can see each other's topics. For a 3-drone
swarm on the same WiFi network, this means drone1 can accidentally receive drone2's thermal
frames if the topic names clash.

The solution is **namespacing**: prefix every topic with the drone's ID.

```
/drone1/thermal/frame    ← only drone1 publishes here
/drone2/thermal/frame    ← only drone2 publishes here
/drone3/thermal/frame    ← only drone3 publishes here
/master/mine_database    ← only master publishes here
```

**Reference:** https://medium.com/@ultroninverse/multi-robot-coordination-in-ros-2-from-namespace-isolation-to-fleet-management-967737ef282e

---

## Node layout — what runs where

### On each slave drone (drone1 / drone2 / drone3):

```
/droneN/
├── thermal_camera_node   (reads MLX90640, publishes /droneN/thermal/frame)
├── vision_filter_node    (subscribes frame, publishes /droneN/detection/candidate)
├── persistence_node      (subscribes candidate, re-hovers, publishes /droneN/detection/confirmed)
├── coordinate_math_node  (subscribes confirmed, publishes /droneN/detection/mine_gps)
├── flight_controller_node (subscribes to /droneN/cmd_waypoint, controls PX4)
└── telemetry_node        (publishes /droneN/gps and /droneN/grid_snapshot)
```

### On the master drone:

```
/master/
├── mission_planner_node  (A* path planning, publishes waypoints to all slaves)
├── grid_server_node      (merges grid snapshots from all slaves)
├── mine_db_node          (deduplication, stores confirmed mines)
├── voice_interface_node  (Vosk, Flask, translates voice → service calls)
└── swarm_commander_node  (dispatches GOTO/LAND/SIDE_MOVE to slaves)
```

---

## Workspace structure

```
ros2_ws/
└── src/
    ├── mine_interfaces/          ← Custom message and service definitions
    │   ├── msg/
    │   │   ├── MineDetection.msg
    │   │   ├── ThermalFrame.msg
    │   │   └── GridSnapshot.msg
    │   ├── srv/
    │   │   ├── GotoWaypoint.srv
    │   │   └── SideMove.srv
    │   └── action/
    │       └── ScanLane.action
    │
    ├── drone_thermal/            ← MLX90640 camera node
    ├── drone_detection/          ← vision_filter + persistence + coordinate_math nodes
    ├── drone_flight/             ← flight_controller node (PX4 via px4_msgs)
    ├── drone_telemetry/          ← telemetry publisher node
    └── drone_master/             ← all master nodes
```

---

## Custom message definitions

### `mine_interfaces/msg/ThermalFrame.msg`
```
# One 32x24 thermal frame from the MLX90640
std_msgs/Header header
float32[768] data          # row-major, degrees C
float32[768] fpn_corrected # after FPN subtraction
```

### `mine_interfaces/msg/MineDetection.msg`
```
# A confirmed mine report from slave → master
std_msgs/Header header
float64 latitude
float64 longitude
float32 local_x            # metres from origin, East
float32 local_y            # metres from origin, North
float32 confidence         # 0.0–1.0
string mine_type           # "buried" or "surface"
uint8[] thermal_image_jpeg # JPEG of the thermal frame at detection time
string drone_id            # "drone1", "drone2", "drone3"
```

### `mine_interfaces/msg/GridSnapshot.msg`
```
# Coverage grid from one slave → master for merging
std_msgs/Header header
string drone_id
int32[] cell_ci            # cell column indices
int32[] cell_cj            # cell row indices
int32[] cell_flags         # SCANNED=1, DETECTION=2, HAZARD=4, FORBIDDEN=8
```

### `mine_interfaces/srv/GotoWaypoint.srv`
```
# Master → Slave: fly to this position
float64 latitude
float64 longitude
float32 altitude_m
---
bool success
string message
```

### `mine_interfaces/action/ScanLane.action`
```
# Master → Slave: scan a full lane
float64[] waypoint_lats
float64[] waypoint_lons
float32 altitude_m
---
bool success
int32 mines_found
---
float32 progress_pct       # feedback: 0.0–100.0
float64 current_lat
float64 current_lon
```

---

## Launch file: one slave drone

```python
# drone_thermal/launch/slave.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument(
        'drone_id', default_value='drone1',
        description='Unique drone identifier')

    drone_id = LaunchConfiguration('drone_id')

    return LaunchDescription([
        drone_id_arg,

        Node(
            package='drone_thermal',
            executable='thermal_camera_node',
            namespace=drone_id,
            name='thermal_camera',
            parameters=[{'fpn_path': 'config/fpn_pattern.npy'}],
        ),
        Node(
            package='drone_detection',
            executable='vision_filter_node',
            namespace=drone_id,
            name='vision_filter',
            parameters=[{
                'delta_t_low': 0.15,
                'delta_t_high': 1.25,
            }],
        ),
        Node(
            package='drone_detection',
            executable='persistence_node',
            namespace=drone_id,
            name='persistence',
            parameters=[{'max_drift_m': 1.5, 'frames': 12}],
        ),
        Node(
            package='drone_detection',
            executable='coordinate_math_node',
            namespace=drone_id,
            name='coordinate_math',
        ),
        Node(
            package='drone_flight',
            executable='flight_controller_node',
            namespace=drone_id,
            name='flight_controller',
        ),
        Node(
            package='drone_telemetry',
            executable='telemetry_node',
            namespace=drone_id,
            name='telemetry',
        ),
    ])
```

Run drone1:
```bash
ros2 launch drone_thermal slave.launch.py drone_id:=drone1
```

Run drone2 (on a different Pi):
```bash
ros2 launch drone_thermal slave.launch.py drone_id:=drone2
```

All topics are automatically namespaced: `/drone1/thermal/frame`, `/drone2/thermal/frame`, etc.

---

## Launch file: master drone

```python
# drone_master/launch/master.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_master',
            executable='mission_planner_node',
            name='mission_planner',
            parameters=[{
                'drone_ids': ['drone1', 'drone2', 'drone3'],
                'field_x_min': -2.0, 'field_x_max': 32.0,
                'field_y_min': -60.0, 'field_y_max': 2.0,
                'lane_step_m': 1.4,
                'scan_step_m': 0.5,
            }],
        ),
        Node(
            package='drone_master',
            executable='grid_server_node',
            name='grid_server',
        ),
        Node(
            package='drone_master',
            executable='mine_db_node',
            name='mine_db',
            parameters=[{'dedup_radius_m': 1.5}],
        ),
        Node(
            package='drone_master',
            executable='voice_interface_node',
            name='voice_interface',
            parameters=[{
                'vosk_model_path': 'vosk_model/vosk-model-small-en-us-0.15',
                'flask_port': 443,
            }],
        ),
    ])
```

---

## How swarm communication works in ROS2

### Current system (manual sockets):
```
slave → UDP packet → master (manual parsing)
master → TCP packet → slave (manual dispatch)
```

### ROS2 system (DDS):
```
slave publishes /drone1/detection/mine_gps
master subscribes /drone1/detection/mine_gps (auto-received via DDS)

master calls /drone1/goto_waypoint service
slave handles service, flies, responds "success"
```

DDS handles the WiFi transport automatically. Both Pi boards need to be on the same WiFi
and have the same `ROS_DOMAIN_ID`:

```bash
# Set on ALL drones and master (in ~/.bashrc):
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0   # allow cross-machine communication
```

---

## swarm_ros_bridge (optional — for isolated domains)

If you want each drone on a separate `ROS_DOMAIN_ID` (for cleaner isolation), use
`swarm_ros_bridge` to selectively forward topics between domains:

https://github.com/carryowen/swarm_ros_bridge

```bash
cd ~/ros2_ws/src
git clone https://github.com/carryowen/swarm_ros_bridge.git
cd ~/ros2_ws && colcon build --packages-select swarm_ros_bridge
```

Configure which topics to forward in `swarm_ros_bridge/config/bridge.yaml`:
```yaml
# Forward mine detections from domain 1 (drone1) to domain 0 (master)
bridge_topics:
  - topic: /drone1/detection/mine_gps
    msg_type: mine_interfaces/msg/MineDetection
    direction: send   # drone domain → master domain
  - topic: /drone1/cmd_waypoint
    msg_type: mine_interfaces/srv/GotoWaypoint
    direction: recv   # master domain → drone domain
```

---

## Using RViz2 for visualisation

RViz2 replaces our custom `tools/pc_visualizer.py`. It can display:
- Drone positions as TF frames or markers
- Thermal images on `/droneN/thermal/frame`
- The coverage grid as a nav2 occupancy grid
- Mine positions as sphere markers

```bash
rviz2
```

Add displays:
- Add → By Topic → `/drone1/thermal/frame` (Image display)
- Add → Marker → `/master/mine_markers`
- Add → Map → `/master/coverage_grid`

This is far more powerful than our custom visualiser and requires zero extra code.

---

## Multi-machine setup checklist

Before running the full swarm on 4 separate Raspberry Pis:

- [ ] All Pis connected to same WiFi network (master Pi's hotspot)
- [ ] `export ROS_DOMAIN_ID=42` in `~/.bashrc` on all machines
- [ ] `export ROS_LOCALHOST_ONLY=0` on all machines
- [ ] `source /opt/ros/humble/setup.bash` in `~/.bashrc` on all machines
- [ ] `source ~/ros2_ws/install/setup.bash` in `~/.bashrc` on all machines
- [ ] uXRCE-DDS Agent running on each slave Pi before starting nodes
- [ ] `px4_msgs` branch matches PX4 firmware version on all machines
- [ ] Firewall rules allow DDS UDP ports (7400–7500 range by default)

Test discovery:
```bash
# On master Pi:
ros2 topic list   # should show topics from all drones

# Monitor a specific drone's GPS:
ros2 topic echo /drone1/fmu/out/sensor_gps
```
