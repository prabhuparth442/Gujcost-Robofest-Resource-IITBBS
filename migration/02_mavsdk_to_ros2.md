# 02 — MAVSDK Code → ROS2 Equivalent Code

This file shows our exact MAVSDK code alongside the ROS2 equivalent, section by section.
You don't need to delete MAVSDK — you can wrap it inside a ROS2 node as a transition step.

**Reference implementations:**
- ArduPilot ROS2 offboard example: https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_DDS
- MAVROS2 (alternative): https://github.com/mavlink/mavros/blob/ros2/mavros/README.md
- ArduPilot ROS2 docs: https://ardupilot.org/dev/docs/ros2.html

---

## Approach A: Wrap MAVSDK inside a ROS2 node (easiest migration)

This is the lowest-effort transition. You keep all the MAVSDK flight code unchanged,
but expose it to the rest of the system through ROS2 topics and services.

The `drone_mavsdk` package on GitHub does exactly this:
https://github.com/slaghuis/drone_mavsdk

```python
# drone_flight/drone_flight/mavsdk_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import asyncio
from mavsdk import System

class MavsdkNode(Node):
    """
    Wraps MAVSDK inside a ROS2 node.
    Subscribes to /droneX/cmd_pose for goto commands.
    Provides /droneX/arm and /droneX/land services.
    """
    def __init__(self):
        super().__init__('mavsdk_flight')
        self.drone = System()
        self.loop = asyncio.get_event_loop()

        # Subscribe to waypoint commands
        self.create_subscription(
            PoseStamped, 'cmd_pose', self.on_cmd_pose, 10)

        # Expose arm/land as services
        self.create_service(Trigger, 'arm',  self.handle_arm)
        self.create_service(Trigger, 'land', self.handle_land)

        # Connect to ArduCopter (SpeedyBee F405) on startup
        self.loop.run_until_complete(self.connect())

    async def connect(self):
        await self.drone.connect(system_address="udp://:14540")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("ArduCopter connected")
                break

    def on_cmd_pose(self, msg):
        lat = msg.pose.position.x   # repurposed fields for GPS
        lon = msg.pose.position.y
        alt = msg.pose.position.z
        self.loop.run_until_complete(
            self.drone.action.goto_location(lat, lon, alt, 0))

    def handle_arm(self, request, response):
        self.loop.run_until_complete(self.drone.action.arm())
        response.success = True
        return response

    def handle_land(self, request, response):
        self.loop.run_until_complete(self.drone.action.land())
        response.success = True
        return response
```

---

## Approach B: Native ROS2 with ardupilot_msgs (cleanest, recommended long-term)

Instead of using MAVSDK, talk directly to ArduCopter over AP_DDS. ArduCopter publishes and
subscribes to `ardupilot_msgs` topics directly — no MAVProxy needed.

This is the approach described in:
https://ardupilot.org/dev/docs/ros2-ap_dds.html

### Installation

```bash
# 1. Install ROS2 Humble
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

# 2. Install Micro XRCE-DDS Agent (runs on RPi, bridges ArduCopter to ROS2)
pip3 install micro-xrce-dds-agent
# OR build from source:
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make && sudo make install

# 3. Create workspace and clone ardupilot_msgs
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/ArduPilot/ardupilot_msgs.git
# (px4_ros_com is PX4-only; for ArduPilot use ardupilot_ros instead)
git clone https://github.com/ArduPilot/ardupilot_ros.git

# 4. Build
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Start the bridge (replaces MAVProxy)

```bash
# On the Raspberry Pi — connect via serial to SpeedyBee F405 (ArduCopter):
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600

# OR over UDP (for SITL testing):
MicroXRCEAgent udp4 --port 8888
```

---

## Code comparison: arm and takeoff

### Current MAVSDK code (`main_orchestrator_competition.py`):

```python
from mavsdk import System

async def arm_and_takeoff(drone, altitude=1.5):
    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(5)   # wait to reach altitude
```

### ROS2 equivalent with ardupilot_msgs:

```python
import rclpy
from rclpy.node import Node
from ardupilot_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint

class FlightNode(Node):
    def __init__(self):
        super().__init__('flight_node')

        # Publishers — send commands to ArduCopter via AP_DDS
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        # IMPORTANT: Must send setpoints before enabling GUIDED mode
        # ArduCopter requires >2 Hz stream or it exits GUIDED
        self.timer = self.create_timer(0.1, self.send_keepalive)

    def send_keepalive(self):
        """Send control mode at 10 Hz to keep ArduCopter in GUIDED mode."""
        msg = OffboardControlMode()
        msg.position = True    # we're doing position control
        msg.velocity = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_pub.publish(msg)

    def arm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0   # 1.0 = arm
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.cmd_pub.publish(msg)

    def takeoff(self, altitude=1.5):
        """Go to altitude at current XY position."""
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -altitude]  # NED: negative Z = up!
        msg.yaw = 0.0
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.setpoint_pub.publish(msg)
```

**Key difference:** In ArduPilot's NED frame, **negative Z is UP**. Our current MAVSDK code
uses positive altitude (GPS altitude above sea level). Be careful with this sign convention.

---

## Code comparison: goto a waypoint

### Current MAVSDK (`main_orchestrator_competition.py`):

```python
async def fly_to(drone, lat, lon, alt=1.5):
    await drone.action.goto_location(lat, lon, alt + origin_alt, 0)
    # Wait until drone is close enough
    async for pos in drone.telemetry.position():
        dist = haversine(pos.latitude_deg, pos.longitude_deg, lat, lon)
        if dist < 0.5:
            break
```

### ROS2 equivalent with ardupilot_msgs:

```python
def goto_local(self, x, y, z=-1.5):
    """
    Go to local NED position.
    x = North metres from origin
    y = East metres from origin
    z = negative altitude (NED convention)
    """
    msg = TrajectorySetpoint()
    msg.position = [x, y, z]
    msg.yaw = float('nan')    # nan = keep current heading
    msg.timestamp = self.get_clock().now().nanoseconds // 1000
    self.setpoint_pub.publish(msg)
    # Note: this is fire-and-forget. Subscribe to /fmu/out/vehicle_local_position
    # to check when you've arrived (like our GPS position subscription)
```

**Subscribe to position feedback:**

```python
from ardupilot_msgs.msg import VehicleLocalPosition

class FlightNode(Node):
    def __init__(self):
        # ... publishers above ...
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.on_position,
            10)
        self.current_pos = None

    def on_position(self, msg):
        self.current_pos = (msg.x, msg.y, msg.z)   # NED metres from origin

    def is_at_target(self, target_x, target_y, tolerance=0.5):
        if self.current_pos is None:
            return False
        dx = self.current_pos[0] - target_x
        dy = self.current_pos[1] - target_y
        return (dx**2 + dy**2)**0.5 < tolerance
```

---

## Code comparison: telemetry (position streaming)

### Current code (`udp_channel.py`):

```python
# Slave manually subscribes to MAVSDK telemetry and packs into UDP
async def stream_telemetry(drone, master_ip, port=14550):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    async for pos in drone.telemetry.position():
        packet = {"lat": pos.latitude_deg, "lon": pos.longitude_deg}
        sock.sendto(json.dumps(packet).encode(), (master_ip, port))
```

### ROS2 equivalent:

```python
# No manual UDP code needed!
# ardupilot_msgs already publishes to /fmu/out/vehicle_gps_position
# The master simply subscribes:

from ardupilot_msgs.msg import SensorGps

class MasterNode(Node):
    def __init__(self):
        super().__init__('master')
        # Listen to all 3 drones' GPS
        for drone_id in ['drone1', 'drone2', 'drone3']:
            self.create_subscription(
                SensorGps,
                f'/{drone_id}/fmu/out/sensor_gps',
                lambda msg, d=drone_id: self.on_gps(msg, d),
                10)

    def on_gps(self, msg, drone_id):
        lat = msg.latitude_deg
        lon = msg.longitude_deg
        self.update_grid(drone_id, lat, lon)
```

---

## Code comparison: mine report (slave → master)

### Current code (`08_comms_link.py`):

```python
# Manual TCP socket with 4-byte length prefix
def send_anomaly_data(self, gps_lat, gps_lon, raw_frame_stack):
    packet = {"type": "anomaly_report", "latitude": gps_lat, ...}
    raw = json.dumps(packet).encode()
    sock.sendall(len(raw).to_bytes(4, 'big'))
    sock.sendall(raw)
```

### ROS2 equivalent:

First, define a custom message `mine_interfaces/msg/MineDetection.msg`:
```
float64 latitude
float64 longitude
float32 confidence
string mine_type
uint8[] thermal_image_jpeg
```

Then publish from slave:
```python
from mine_interfaces.msg import MineDetection

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection')
        self.pub = self.create_publisher(
            MineDetection, 'mine_detection', 10)
        # Note: no IP address needed — DDS handles routing

    def report_mine(self, lat, lon, conf, jpeg_bytes):
        msg = MineDetection()
        msg.latitude = lat
        msg.longitude = lon
        msg.confidence = conf
        msg.thermal_image_jpeg = list(jpeg_bytes)
        self.pub.publish(msg)
```

Master subscribes:
```python
class MasterNode(Node):
    def __init__(self):
        for drone_id in ['drone1', 'drone2', 'drone3']:
            self.create_subscription(
                MineDetection,
                f'/{drone_id}/mine_detection',
                self.on_mine_detected,
                10)

    def on_mine_detected(self, msg):
        self.add_to_mine_db(msg.latitude, msg.longitude, msg.confidence)
```

No TCP sockets. No length prefixing. ROS2 handles it all.

---

## Summary: what disappears with ROS2

| Current file | Replaced by |
|-------------|------------|
| `udp_channel.py` | DDS auto-discovery + topic subscriptions |
| `tcp_channel.py` | ROS2 services + actions |
| `08_comms_link.py` | Custom `MineDetection` topic publisher |
| MAVProxy serial bridge | `MicroXRCEAgent serial` command |
| Manual asyncio task management | `rclpy.spin()` + ROS2 executors |
| `launch.py` (root) | ROS2 launch files |
