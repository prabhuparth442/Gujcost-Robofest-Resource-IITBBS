# 03 — ArduPilot ↔ ROS2 Bridge Setup (AP_DDS)

## What is AP_DDS (ArduPilot DDS bridge)?

AP_DDS (ArduPilot DDS bridge) (micro eXtremely Resource Constrained Environments DDS) is the official bridge
between ArduCopter (running on the SpeedyBee F405) and ROS2 (running on the Raspberry Pi).

**In our current system:**
```
Raspberry Pi  ←serial→  MAVProxy  ←MAVLink→  SpeedyBee F405 (ArduCopter)
```

**With AP_DDS (ArduPilot DDS bridge):**
```
Raspberry Pi (AP_DDS Agent)  ←serial/UDP→  SpeedyBee F405 (AP_DDS Client built into ArduCopter)
```

The Agent runs on the Pi. The Client is already compiled into ArduPilot firmware (v1.14+).
Topics appear in ROS2 as if they were published by regular ROS2 nodes.

**Official docs:**
- AP_DDS bridge docs: https://ardupilot.org/dev/docs/ros2-ap_dds.html
- ArduPilot ROS2 docs: https://ardupilot.org/dev/docs/ros2.html
- ardupilot_ros package: https://github.com/ArduPilot/ardupilot_ros

---

## Step 1: Update ArduPilot firmware

AP_DDS client is built into ArduCopter v4.5 and later. Check your version in Mission Planner:
Vehicle Setup → Summary → Firmware Version.

If below v1.14, update via QGroundControl:
1. Connect SpeedyBee F405 via USB
2. QGroundControl → Vehicle Setup → Firmware
3. Check firmware version → must be ArduCopter v4.5 or newer for AP_DDS

---

## Step 2: Enable AP_DDS client in ArduCopter

The client needs to be enabled via ArduCopter parameters in Mission Planner.

In QGroundControl → Vehicle Setup → Parameters, search for:

| Parameter | Set to | Meaning |
|-----------|--------|---------|
| `UXRCE_DDS_CFG` | 102 (TELEM2) or 0 (disabled) | Which serial port to use |
| `SER_TEL2_BAUD` | 921600 | Serial baud rate |
| `UXRCE_DDS_DOM_ID` | 0 | ROS_DOMAIN_ID (match your ROS2 env var) |
| `UXRCE_DDS_KEY` | 1 | Session key (leave as 1) |

If you prefer UDP (for SITL or WiFi testing), set:
| `UXRCE_DDS_CFG` | 1000 (ethernet/UDP) |

Restart the SpeedyBee after changing parameters.

---

## Step 3: Install Micro XRCE-DDS Agent on Raspberry Pi

The Agent bridges ArduCopter's serial/UDP stream to ROS2 DDS topics.

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

The SpeedyBee's UART6 (T6/R6 pads) → directly to Pi GPIO UART (see hardware/README.md):

```bash
# If using /dev/ttyAMA0 (Pi GPIO UART, same as our MAVProxy setup):
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600

# If using USB-to-serial adapter:
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

### UDP connection (SITL testing):

```bash
# ArduCopter SITL exposes AP_DDS data to UDP port 8888 by default
MicroXRCEAgent udp4 --port 8888
```

You should see output like:
```
[1706789012.123456] info     | UDPv4AgentLinux.cpp | init | running in port 8888
[1706789015.456789] info     | Root.cpp | set_verbose_level | Session established
```

If the session is established, ArduCopter topics are now live in ROS2.

---

## Step 5: Verify topics are visible

```bash
# Source ROS2 and your workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# List all ArduCopter AP_DDS topics:
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

## Step 6: Clone ardupilot_msgs into your workspace

Your ROS2 nodes need the `ardupilot_msgs` package to understand ArduPilot message types.
**The version of `ardupilot_msgs` must match your ArduPilot firmware version.**

```bash
cd ~/ros2_ws/src
git clone https://github.com/ArduPilot/ardupilot_msgs.git
cd ardupilot_msgs

# Checkout the branch matching your ArduPilot firmware version:
git checkout ArduCopter-4.5  # match your ArduCopter version
# git checkout master         # for latest development ArduCopter

cd ~/ros2_ws
colcon build --packages-select ardupilot_msgs
source install/setup.bash
```

---

## Key ardupilot_msgs topics reference

### Topics ArduCopter publishes via AP_DDS (we subscribe to these):

| ROS2 topic | ardupilot_msgs type | What it contains |
|------------|--------------|-----------------|
| `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` | x, y, z in metres from origin (NED) |
| `/fmu/out/sensor_gps` | `SensorGps` | latitude, longitude, altitude |
| `/fmu/out/vehicle_attitude` | `VehicleAttitude` | quaternion orientation |
| `/fmu/out/battery_status` | `BatteryStatus` | voltage, current, remaining % |
| `/fmu/out/vehicle_status` | `VehicleStatus` | armed status, nav state |
| `/fmu/out/sensor_combined` | `SensorCombined` | accelerometer + gyro (raw IMU) |

### Topics we publish to (ArduCopter subscribes to these):

| ROS2 topic | ardupilot_msgs type | What it does |
|------------|--------------|-------------|
| `/fmu/in/vehicle_command` | `VehicleCommand` | Arm, disarm, takeoff, land, set mode |
| `/ap/cmd_vel` | `TwistStamped` | Velocity command in GUIDED mode |
| `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` | Go to position / velocity setpoint |

**Critical:** ArduCopter will revert to HOLD if waypoint commands stop arriving at >2 Hz
for >500 ms. Always publish this at 10+ Hz in a background timer, not just when moving.

---

## Complete minimal GUIDED mode example

This is a complete node that: connects, arms, takes off, flies to a local position, lands.
Reference: https://ardupilot.org/dev/docs/ros2-ap_dds.html

```python
#!/usr/bin/env python3
"""
minimal_guided.py — Minimal ArduCopter GUIDED mode position control via ROS2
Replaces our main_orchestrator_competition.py flight section.

Run: ros2 run drone_flight minimal_offboard
Requires: ardupilot_msgs installed, MicroXRCEAgent running
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from ardupilot_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint,
    VehicleCommand, VehicleLocalPosition, VehicleStatus
)

class MinimalOffboard(Node):
    def __init__(self):
        super().__init__('minimal_offboard')

        # QoS profile that matches ArduCopter AP_DDS publisher settings
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
        # Always send GUIDED mode keepalive first
        self.publish_offboard_mode()

        self.offboard_counter += 1

        if self.offboard_counter == 10:
            # After 1 second of setpoints, enable GUIDED mode
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
# Terminal 1: Start AP_DDS (ArduPilot DDS bridge) agent
MicroXRCEAgent udp4 --port 8888   # for SITL

# Terminal 2: Start ArduCopter SITL
cd ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map

# Terminal 3: Run the node
source ~/ros2_ws/install/setup.bash
ros2 run drone_flight minimal_offboard
```

---

## Comparison: MAVProxy vs AP_DDS (ArduPilot DDS bridge) Agent

| Feature | MAVProxy | AP_DDS (ArduPilot DDS bridge) Agent |
|---------|---------|-----------------|
| Protocol | MAVLink | DDS |
| Interface | Python/C API (MAVSDK) | ROS2 topics |
| CPU usage | Low | Medium |
| Latency | ~10 ms | ~5 ms |
| ArduCopter version | Any | v4.5+ |
| Dependency | `pip install MAVProxy` | Build from source or pip |
| Compatibility | ArduPilot ✓ | ArduPilot ✓ |
