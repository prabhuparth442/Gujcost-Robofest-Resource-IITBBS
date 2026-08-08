# 05 — Step-by-Step Migration Plan

## Overview

Migrating from MAVSDK scripts to ROS2 is a significant change. This plan breaks it into
phases so the system stays flyable at every step — you never end up with a half-migrated
codebase that doesn't work.

**Total estimated effort:** 6–8 weeks for a 3–4 person team with some ROS2 experience.

---

## Phase 0: Learning (1–2 weeks, no code changes)

### Goal: Everyone on the team understands ROS2 basics before touching the codebase.

**Tasks:**
1. Install ROS2 Humble on each team member's laptop:
   https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

2. Complete the official beginner tutorials (takes ~4 hours):
   https://docs.ros.org/en/humble/Tutorials.html
   - "Configuring your ROS 2 environment"
   - "Using turtlesim, ros2, and rqt"
   - "Understanding nodes"
   - "Understanding topics"
   - "Writing a simple publisher and subscriber (Python)"
   - "Writing a simple service and client (Python)"

3. Run the ArduPilot ROS2 example against SITL:
   https://ardupilot.org/dev/docs/ros2-ap_dds.html

4. Read `migration/01_ros2_concepts.md` and `migration/02_mavsdk_to_ros2.md`

**Done when:** Every team member can write a publisher/subscriber pair from memory.

---

## Phase 1: Infrastructure Setup (1 week)

### Goal: ROS2 workspace exists and builds; ArduPilot AP_DDS bridge works in SITL.

**Tasks:**

**1.1 — Create the ROS2 workspace:**
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone ArduPilot message definitions
git clone https://github.com/ArduPilot/ardupilot_msgs.git
git checkout release/1.14   # match your firmware

# Clone the ArduPilot ROS2 package (reference code)
git clone https://github.com/ArduPilot/ardupilot_ros.git

cd ~/ros2_ws
colcon build
source install/setup.bash
```

**1.2 — Create `mine_interfaces` package (custom messages):**
```bash
cd ~/ros2_ws/src
ros2 pkg create mine_interfaces --build-type ament_cmake
# Then create the .msg and .srv files from migration/04_swarm_in_ros2.md
```

**1.3 — Verify AP_DDS (ArduPilot DDS bridge) Agent with SITL:**
```bash
# Terminal 1: ArduCopter SITL
cd ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map

# Terminal 2: Agent (SITL uses UDP)
MicroXRCEAgent udp4 --port 8888

# Terminal 3: Verify topics
source ~/ros2_ws/install/setup.bash
ros2 topic list   # should show /fmu/out/... topics
ros2 topic echo /fmu/out/vehicle_local_position
```

**Done when:** `ros2 topic echo /fmu/out/sensor_gps` shows GPS data from SITL.

---

## Phase 2: Migrate Flight Controller (1 week)

### Goal: Replace MAVSDK flight commands with ardupilot_msgs. Keep detection pipeline unchanged.

**Why this first?** Flight is the most safety-critical part. Get it right before touching detection.

**Tasks:**

**2.1 — Create `drone_flight` package:**
```bash
cd ~/ros2_ws/src
ros2 pkg create drone_flight --build-type ament_python --dependencies rclpy ardupilot_msgs
```

**2.2 — Write `flight_controller_node.py`:**

Copy the `minimal_offboard.py` from `migration/03_ardupilot_ros2_bridge.md` as a starting point.
Extend it to handle our full waypoint sequence:

```python
# Key additions over the minimal example:
# - Subscribe to /droneN/cmd_waypoint (GotoWaypoint) service
# - Publish position feedback to /droneN/local_position
# - Implement wait_for_arrival() by checking distance
# - Forward TF-Luna failsafe (still runs as asyncio or a separate node)
```

**2.3 — Test in SITL:**
```bash
ros2 launch drone_flight test_waypoints.launch.py
```

Write a test launch file that sends the drone to 5 waypoints in SITL and verifies arrival.

**2.4 — Test on real drone (tethered, indoors):**

Strap the Pi to a bench with SpeedyBee F405 attached. Test arm/takeoff/land only.
Do not fly freely until Phase 3 is complete.

**Done when:** `ros2 service call /drone1/goto_waypoint ...` makes the SITL drone reach the target.

---

## Phase 3: Migrate Telemetry (3 days)

### Goal: Replace UDP telemetry (udp_channel.py) with ROS2 topics.

**Tasks:**

**3.1 — Create `drone_telemetry` package:**
```bash
ros2 pkg create drone_telemetry --build-type ament_python \
    --dependencies rclpy ardupilot_msgs mine_interfaces
```

**3.2 — Write `telemetry_node.py`:**

```python
# Subscribes to /fmu/out/sensor_gps and /fmu/out/vehicle_local_position
# Republishes as our namespace: /droneN/gps
# Also publishes /droneN/grid_snapshot (GridSnapshot msg) every 5 ticks
```

**3.3 — Write `grid_server_node.py` (in master package):**

```python
# Subscribes to /drone1/grid_snapshot, /drone2/grid_snapshot, /drone3/grid_snapshot
# Merges them using the same logic as grid_map.merge_from()
# Publishes /master/coverage_grid as nav_msgs/OccupancyGrid (for RViz2)
```

**Done when:** RViz2 shows the merged coverage grid updating in real time during SITL.

---

## Phase 4: Migrate Detection Pipeline (1–2 weeks)

### Goal: Thermal detection pipeline becomes ROS2 nodes.

This is the most complex phase because the pipeline has 5 stages.

**Recommended order:**
1. `thermal_camera_node` — wraps the C++ subprocess, publishes `ThermalFrame`
2. `vision_filter_node` — subscribes `ThermalFrame`, publishes `DetectionCandidate`
3. `surface_filter_node` — parallel to vision_filter, different thresholds
4. `persistence_node` — subscribes candidates, re-hovers via flight_controller service
5. `coordinate_math_node` — converts pixel offset to GPS, publishes `MineDetection`

**4.1 — thermal_camera_node.py:**
```python
class ThermalCameraNode(Node):
    def __init__(self):
        super().__init__('thermal_camera')
        self.pub = self.create_publisher(
            ThermalFrame, 'thermal/frame', 10)

        # Start the C++ subprocess (same as current code)
        self.proc = subprocess.Popen(
            ['sudo', './bin/mlx_stdout'],
            stdout=subprocess.PIPE)
        fpn = np.load(self.get_parameter('fpn_path').value)
        self.fpn = fpn

        # Read at 2 Hz
        self.create_timer(0.5, self.read_and_publish)

    def read_and_publish(self):
        raw = self.proc.stdout.read(768 * 4)
        frame = np.frombuffer(raw, dtype=np.float32)
        corrected = frame - self.fpn.flatten()

        msg = ThermalFrame()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = frame.tolist()
        msg.fpn_corrected = corrected.tolist()
        self.pub.publish(msg)
```

**4.2 — vision_filter_node.py:**
```python
class VisionFilterNode(Node):
    def __init__(self):
        super().__init__('vision_filter')
        self.sub = self.create_subscription(
            ThermalFrame, 'thermal/frame', self.on_frame, 10)
        self.pub = self.create_publisher(
            DetectionCandidate, 'detection/candidate', 10)
        # Parameters (replaces hardcoded constants in 02_vision_filter.py)
        self.declare_parameter('delta_t_low', 0.15)
        self.declare_parameter('delta_t_high', 1.25)

    def on_frame(self, msg):
        frame = np.array(msg.fpn_corrected).reshape(24, 32)
        # Your existing vision filter logic here
        result = self.your_existing_filter(frame)
        if result.detected:
            candidate = DetectionCandidate()
            candidate.pixel_dx = result.dx
            candidate.pixel_dy = result.dy
            candidate.confidence = result.confidence
            self.pub.publish(candidate)
```

**Done when:** `ros2 topic echo /drone1/detection/mine_gps` shows GPS coordinates when
the drone flies over a thermal anomaly in SITL (using pre-recorded test data injected
into the `ThermalFrame` topic).

---

## Phase 5: Migrate Master Coordinator (1 week)

### Goal: Replace Flask + manual dispatch with ROS2 service calls.

**Tasks:**

**5.1 — `mission_planner_node.py`:**
Replace the A* code in `master/app.py` with a proper ROS2 node. The A* algorithm itself
doesn't change — just the interface around it.

**5.2 — `mine_db_node.py`:**
Subscribes to `/droneN/detection/mine_gps` from all 3 drones, runs deduplication,
publishes confirmed mines to `/master/confirmed_mines`.

**5.3 — `voice_interface_node.py`:**
Keep the Flask + Vosk code, but instead of directly calling `tcp_commander.py`, call
ROS2 services:

```python
# When voice command "start mission" detected:
client = self.create_client(Trigger, '/master/start_mission')
client.call_async(Trigger.Request())
```

**Done when:** Speaking "start mission" into the phone browser causes all 3 SITL drones
to begin their scan pattern.

---

## Phase 6: Integration Test (1 week)

### Goal: Full system test — all nodes, all drones, SITL.

**6.1 — Multi-drone SITL:**

ArduCopter supports spawning multiple SITL instances on different UDP ports:
```bash
# Terminal 1: Drone 1 on port 14540
sim_vehicle.py -v ArduCopter -f gazebo-iris -I 1 --sysid 1 --out=udp:127.0.0.1:14551

# Terminal 2: Drone 2 on port 14541
sim_vehicle.py -v ArduCopter -f gazebo-iris -I 2 --sysid 2 --out=udp:127.0.0.1:14552

# Terminal 3: Drone 3 on port 14542
sim_vehicle.py -v ArduCopter -f gazebo-iris -I 3 --sysid 3 --out=udp:127.0.0.1:14553
```

**6.2 — Full launch:**
```bash
# Master:
ros2 launch drone_master master.launch.py

# Slave 1:
ros2 launch drone_thermal slave.launch.py drone_id:=drone1

# Slave 2:
ros2 launch drone_thermal slave.launch.py drone_id:=drone2

# Slave 3:
ros2 launch drone_thermal slave.launch.py drone_id:=drone3
```

**6.3 — Test checklist:**
- [ ] All 3 drones arm and take off on "start mission" voice command
- [ ] Coverage grid updates in RViz2 during flight
- [ ] Injected thermal anomaly triggers mine report on master
- [ ] Deduplication correctly merges reports from 2 drones on same mine
- [ ] "land all" voice command lands all 3 drones
- [ ] SIDE_MOVE between passes works correctly
- [ ] System handles one drone disconnecting gracefully

---

## Phase 7: Hardware Testing (ongoing)

Only after Phase 6 passes completely: test on real Raspberry Pis with real Pixhawks.
Start with one drone, then two, then all three.

---

## Effort summary

| Phase | Effort | Milestone |
|-------|--------|-----------|
| 0 — Learning | 1–2 weeks | Team knows ROS2 basics |
| 1 — Infrastructure | 1 week | Workspace builds, SITL bridge works |
| 2 — Flight controller | 1 week | Drone flies waypoints via ROS2 |
| 3 — Telemetry | 3 days | Grid updates in RViz2 |
| 4 — Detection pipeline | 1–2 weeks | Mines detected via ROS2 topics |
| 5 — Master coordinator | 1 week | Voice commands dispatch via ROS2 |
| 6 — Integration test | 1 week | Full SITL mission passes |
| 7 — Hardware test | ongoing | Real drones fly full mission |

---

## What NOT to migrate (keep as-is)

| Component | Reason to keep |
|-----------|---------------|
| `mlx_stdout.cpp` C++ binary | C++ subprocess pattern still works perfectly in ROS2 |
| `fpn_pattern.npy` calibration | File format unchanged; just pass path as ROS2 parameter |
| A* algorithm logic | The algorithm doesn't change, only its ROS2 node wrapper |
| Vosk speech recognition | Keep in voice_interface_node; only the Flask dispatch changes |
| `fieldmap.py` field geometry | Import directly into ROS2 nodes as a Python module |

---

## Key references for migration

| Topic | URL |
|-------|-----|
| ROS2 Humble installation | https://docs.ros.org/en/humble/Installation.html |
| ROS2 beginner tutorials | https://docs.ros.org/en/humble/Tutorials.html |
| ArduPilot ROS2 AP_DDS example | https://ardupilot.org/dev/docs/ros2-ap_dds.html |
| ArduPilot ROS2 User Guide | https://ardupilot.org/dev/docs/ros2.html |
| ArduPilot GUIDED mode docs | https://ardupilot.org/copter/docs/ac2_guidedmode.html |
| AP_DDS bridge docs | https://ardupilot.org/dev/docs/ros2-ap_dds.html |
| ardupilot_ros package | https://github.com/ArduPilot/ardupilot_ros |
| swarm_ros_bridge | https://github.com/carryowen/swarm_ros_bridge |
| Multi-robot namespaces | https://turtlebot.github.io/turtlebot4-user-manual/tutorials/multiple_robots.html |
| Nav2 for path planning | https://nav2.org/ |
