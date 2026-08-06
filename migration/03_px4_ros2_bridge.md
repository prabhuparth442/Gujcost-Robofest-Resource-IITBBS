# 03 — PX4 ↔ ROS2 Bridge Setup (uXRCE-DDS)

## What is uXRCE-DDS?

uXRCE-DDS (micro eXtremely Resource Constrained Environments DDS) is the official bridge
between PX4 (running on the Pixhawk) and ROS2 (running on the Raspberry Pi).

**In our current system:**
```
Raspberry Pi  ←serial→  MAVProxy  ←MAVLink→  Pixhawk (PX4)
```

**With uXRCE-DDS:**
```
Raspberry Pi (uXRCE-DDS Agent)  ←serial/UDP→  Pixhawk (uXRCE-DDS Client built into PX4)
```

The Agent runs on the Pi. The Client is already compiled into PX4 firmware (v1.14+).
Topics appear in ROS2 as if they were published by regular ROS2 nodes.

**Official docs:**
- uXRCE-DDS bridge: https://docs.px4.io/main/en/middleware/uxrce_dds
- PX4 ROS2 User Guide: https://docs.px4.io/main/en/ros2/user_guide
- ROS2 User Guide (gitbook): https://px4.gitbook.io/px4-user-guide/robotics/ros/ros2/ros2_comm

---

## Step 1: Update PX4 firmware

uXRCE-DDS client is built into PX4 v1.14 and later. Check your version in QGroundControl:
Vehicle Setup → Summary → Firmware Version.

If below v1.14, update via QGroundControl:
1. Connect Pixhawk via USB
2. QGroundControl → Vehicle Setup → Firmware
3. Select "PX4 Pro" and "Advanced" → specify v1.14 or stable

---

## Step 2: Enable uXRCE-DDS client in PX4

The client needs to be enabled via PX4 parameters.

In QGroundControl → Vehicle Setup → Parameters, search for:

| Parameter | Set to | Meaning |
|-----------|--------|---------|
| `UXRCE_DDS_CFG` | 102 (TELEM2) or 0 (disabled) | Which serial port to use |
| `SER_TEL2_BAUD` | 921600 | Serial baud rate |
| `UXRCE_DDS_DOM_ID` | 0 | ROS_DOMAIN_ID (match your ROS2 env var) |
| `UXRCE_DDS_KEY` | 1 | Session key (leave as 1) |

If you prefer UDP (for SITL or WiFi testing), set:
| `UXRCE_DDS_CFG` | 1000 (ethernet/UDP) |

Restart the Pixhawk after changing parameters.

---

## Step 3: Install Micro XRCE-DDS Agent on Raspberry Pi

The Agent bridges PX4's serial/UDP stream to ROS2 DDS topics.

### Option A: Install from pip (easiest)

```bash
pip3 install micro-xrce-dds-agent --break-system-packages
```

### Option B: Build from source (more control)

```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake -DUXRCE_BUILD_PROFILE=Release ..
make -j4
sudo make install
```

### Option C: Inside a ROS2 workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd ~/ros2_ws
colcon build --packages-select micro_ros_agent
source install/setup.bash
```

---

## Step 4: Start the Agent

### Serial connection (physical drone):

The Pixhawk's TELEM2 port → USB-to-serial adapter (or directly to Pi GPIO UART):

```bash
# If using /dev/ttyAMA0 (Pi GPIO UART, same as our MAVProxy setup):
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600

# If using USB-to-serial adapter:
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

### UDP connection (SITL testing):

```bash
# PX4 SITL sends uXRCE-DDS data to UDP port 8888 by default
MicroXRCEAgent udp4 --port 8888
```

You should see output like:
```
[1706789012.123456] info     | UDPv4AgentLinux.cpp | init | running in port 8888
[1706789015.456789] info     | Root.cpp | set_verbose_level | Session established
```

If the session is established, PX4 topics are now live in ROS2.

---

## Step 5: Verify topics are visible

```bash
# Source ROS2 and your workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# List all PX4 topics:
ros2 topic list

# You should see topics like:
# /fmu/in/vehicle_command
# /fmu/in/offboard_control_mode
# /fmu/in/trajectory_setpoint
# /fmu/out/vehicle_local_position
# /fmu/out/vehicle_gps_position
# /fmu/out/sensor_combined
# /fmu/out/battery_status
# ... (many more)

# Monitor GPS in real time:
ros2 topic echo /fmu/out/sensor_gps
```

---

## Step 6: Clone px4_msgs into your workspace

Your ROS2 nodes need the `px4_msgs` package to understand PX4 message types.
**The version of `px4_msgs` must match your PX4 firmware version.**

```bash
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd px4_msgs

# Checkout the branch matching your PX4 firmware version:
git checkout release/1.14   # for PX4 v1.14
# git checkout main          # for latest development PX4

cd ~/ros2_ws
colcon build --packages-select px4_msgs
source install/setup.bash
```

---

## Key px4_msgs topics reference

### Topics PX4 publishes (we subscribe to these):

| ROS2 topic | px4_msgs type | What it contains |
|------------|--------------|-----------------|
| `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` | x, y, z in metres from origin (NED) |
| `/fmu/out/sensor_gps` | `SensorGps` | latitude, longitude, altitude |
| `/fmu/out/vehicle_attitude` | `VehicleAttitude` | quaternion orientation |
| `/fmu/out/battery_status` | `BatteryStatus` | voltage, current, remaining % |
| `/fmu/out/vehicle_status` | `VehicleStatus` | armed status, nav state |
| `/fmu/out/sensor_combined` | `SensorCombined` | accelerometer + gyro (raw IMU) |

### Topics we publish to (PX4 subscribes to these):

| ROS2 topic | px4_msgs type | What it does |
|------------|--------------|-------------|
| `/fmu/in/vehicle_command` | `VehicleCommand` | Arm, disarm, takeoff, land, set mode |
| `/fmu/in/offboard_control_mode` | `OffboardControlMode` | Keep-alive for offboard mode (>2 Hz!) |
| `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` | Go to position / velocity setpoint |

**Critical:** PX4 will exit offboard mode if `OffboardControlMode` messages stop arriving
for >500 ms. Always publish this at 10+ Hz in a background timer, not just when moving.

---

## Complete minimal offboard example

This is a complete node that: connects, arms, takes off, flies to a local position, lands.
Adapted from: https://github.com/Jaeyoung-Lim/px4-offboard

```python
#!/usr/bin/env python3
"""
minimal_offboard.py — Minimal PX4 offboard position control via ROS2
Replaces our main_orchestrator_competition.py flight section.

Run: ros2 run drone_flight minimal_offboard
Requires: px4_msgs installed, MicroXRCEAgent running
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint,
    VehicleCommand, VehicleLocalPosition, VehicleStatus
)

class MinimalOffboard(Node):
    def __init__(self):
        super().__init__('minimal_offboard')

        # QoS profile that matches PX4's publisher settings
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # Subscribers
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.on_position, qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.on_status, qos)

        self.pos = None
        self.armed = False
        self.offboard_counter = 0
        self.target = [0.0, 0.0, -1.5]   # NED: x=North, y=East, z=Up(negative)

        # 10 Hz control loop — must be faster than 2 Hz!
        self.timer = self.create_timer(0.1, self.control_loop)

    def on_position(self, msg):
        self.pos = (msg.x, msg.y, msg.z)

    def on_status(self, msg):
        self.armed = (msg.arming_state == 2)   # 2 = ARMED

    def control_loop(self):
        # Always send offboard keepalive first
        self.publish_offboard_mode()

        self.offboard_counter += 1

        if self.offboard_counter == 10:
            # After 1 second of setpoints, enable offboard mode
            self.set_offboard_mode()

        if self.offboard_counter == 20:
            # After 2 seconds, arm
            self.arm()

        # Always publish setpoint
        self.publish_setpoint(*self.target)

        # Check arrival
        if self.pos and self.armed:
            dx = self.pos[0] - self.target[0]
            dy = self.pos[1] - self.target[1]
            dist = (dx**2 + dy**2)**0.5
            if dist < 0.3:
                self.get_logger().info(f'Reached target! dist={dist:.2f}m')

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.setpoint_pub.publish(msg)

    def arm(self):
        self._vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                              param1=1.0)
        self.get_logger().info('ARM command sent')

    def land(self):
        self._vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def set_offboard_mode(self):
        self._vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,   # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            param2=6.0)   # PX4_CUSTOM_MAIN_MODE_OFFBOARD

    def _vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = MinimalOffboard()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Run it:
```bash
# Terminal 1: Start uXRCE-DDS agent
MicroXRCEAgent udp4 --port 8888   # for SITL

# Terminal 2: Start PX4 SITL
cd PX4-Autopilot && make px4_sitl gazebo-classic_iris

# Terminal 3: Run the node
source ~/ros2_ws/install/setup.bash
ros2 run drone_flight minimal_offboard
```

---

## Comparison: MAVProxy vs uXRCE-DDS Agent

| Feature | MAVProxy | uXRCE-DDS Agent |
|---------|---------|-----------------|
| Protocol | MAVLink | DDS |
| Interface | Python/C API (MAVSDK) | ROS2 topics |
| CPU usage | Low | Medium |
| Latency | ~10 ms | ~5 ms |
| PX4 version | Any | v1.14+ |
| Dependency | `pip install MAVProxy` | Build from source or pip |
| Compatibility | ArduPilot too | PX4 only |
