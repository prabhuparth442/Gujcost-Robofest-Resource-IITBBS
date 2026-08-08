# 01 — ROS2 Concepts for Beginners

If you've never used ROS2 before, this is the right place to start.
We'll explain every concept by comparing it to something in our current codebase.

**Official beginner tutorials:** https://docs.ros.org/en/humble/Tutorials.html

---

## What is ROS2?

ROS stands for **Robot Operating System**. Despite the name, it's not an operating system
like Linux or Windows. It's a **framework** — a collection of tools, libraries, and
conventions for building robot software.

Think of it like this: instead of writing raw Python scripts that manually manage sockets,
threads, and process communication, ROS2 gives you a standard way to connect software
components together.

**Without ROS2 (our current system):**
```
main_orchestrator.py
  ├─ manually opens subprocess for mlx_stdout
  ├─ manually creates asyncio tasks for vision filter
  ├─ manually opens UDP socket to master
  └─ manually opens TCP socket for mine reports
```

**With ROS2:**
```
Each of those is a "node". Nodes communicate via "topics".
The framework handles all the socket management, threading, and discovery.
```

ROS2 is the successor to ROS1 (released 2017). We use ROS2 Humble (LTS, released 2022).

---

## Core Concept 1: Nodes

A **node** is just a Python (or C++) program that does one specific thing.

In our current system, `main_orchestrator_competition.py` does everything: flies the drone,
reads frames, runs detection, communicates with master. In ROS2, you'd split this into
separate nodes:

| Our current code | ROS2 node equivalent |
|-----------------|----------------------|
| mlx_stdout C++ subprocess | `thermal_camera_node` |
| Vision filter section | `vision_filter_node` |
| Persistence gate section | `persistence_node` |
| MAVSDK flight section | `flight_controller_node` |
| UDP telemetry section | `telemetry_node` |

Each node runs independently and communicates via topics.

**Starting a node:**
```python
import rclpy
from rclpy.node import Node

class ThermalCameraNode(Node):
    def __init__(self):
        super().__init__('thermal_camera')   # node name
        # ... your code here

def main():
    rclpy.init()
    node = ThermalCameraNode()
    rclpy.spin(node)   # keeps node running until Ctrl+C
    rclpy.shutdown()
```

---

## Core Concept 2: Topics

A **topic** is a named channel for streaming data between nodes.

One node **publishes** data to a topic. Other nodes **subscribe** to receive it.

This is exactly like our UDP telemetry, but:
- No manual socket code
- Topics are named and typed (you can't accidentally send the wrong data)
- Discovery is automatic (nodes find each other without knowing IP addresses)

**Publisher (like our `udp_channel.py`):**
```python
from sensor_msgs.msg import Image
from rclpy.node import Node

class ThermalCameraNode(Node):
    def __init__(self):
        super().__init__('thermal_camera')
        # Create a publisher on topic '/drone1/thermal/frame'
        self.pub = self.create_publisher(Image, '/drone1/thermal/frame', 10)
        # Publish at 2 Hz
        self.timer = self.create_timer(0.5, self.publish_frame)

    def publish_frame(self):
        msg = Image()
        msg.data = self.get_thermal_frame()   # your frame reading code
        self.pub.publish(msg)
```

**Subscriber (like our `02_vision_filter.py` reading frames):**
```python
class VisionFilterNode(Node):
    def __init__(self):
        super().__init__('vision_filter')
        # Subscribe to the same topic
        self.sub = self.create_subscription(
            Image, '/drone1/thermal/frame', self.on_frame, 10)

    def on_frame(self, msg):
        frame = np.frombuffer(msg.data, dtype=np.float32).reshape(24, 32)
        # run your detection here
```

The `10` is the **queue size** — how many messages to buffer if the subscriber is slow.

**Official tutorial:** https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

---

## Core Concept 3: Services

A **service** is for request-response communication (not streaming).

Compare:
- Topic = like sending WhatsApp messages (fire and forget, no reply expected)
- Service = like making a phone call (you wait for the other side to respond)

In our current code, the TCP command channel (`tcp_channel.py`) is similar to a service:
master sends `GOTO` command, slave receives it and acts on it. But our TCP channel has
no built-in way to reply with "done" or "error". A ROS2 service handles this:

```python
# Service definition (like a function signature):
# std_srvs/srv/Trigger.srv
# Request: (empty)
# Response: bool success, string message

# Server (slave drone):
from std_srvs.srv import Trigger

class FlightNode(Node):
    def __init__(self):
        super().__init__('flight_controller')
        self.srv = self.create_service(
            Trigger, '/drone1/arm', self.handle_arm)

    def handle_arm(self, request, response):
        # Arm the drone (using MAVSDK or ardupilot_msgs)
        response.success = True
        response.message = "Armed"
        return response

# Client (master drone):
class MasterNode(Node):
    def arm_drone(self, drone_id):
        client = self.create_client(Trigger, f'/{drone_id}/arm')
        client.wait_for_service()
        future = client.call_async(Trigger.Request())
        # future.result() gives the response
```

**Official tutorial:** https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html

---

## Core Concept 4: Actions

An **action** is for long-running tasks where you want progress updates.

Compare:
- Service = "turn on the light" (instant)
- Action = "fly to coordinate (takes 30 seconds, tell me progress)"

Our current `goto_location()` MAVSDK call blocks the asyncio task until the drone arrives.
An action is non-blocking and sends feedback:

```python
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class MasterNode(Node):
    def send_drone_to_waypoint(self, x, y, z):
        client = ActionClient(self, NavigateToPose, '/drone1/navigate_to_pose')
        goal = NavigateToPose.Goal()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z

        future = client.send_goal_async(goal, feedback_callback=self.on_feedback)

    def on_feedback(self, feedback):
        # called periodically with distance remaining
        dist = feedback.feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {dist:.2f}m')
```

---

## Core Concept 5: Messages (msg types)

ROS2 has standard message types for common data. You can also define custom ones.

| Our data | ROS2 message type |
|----------|------------------|
| Thermal frame (32×24 float array) | `sensor_msgs/Image` |
| GPS position (lat, lon, alt) | `sensor_msgs/NavSatFix` |
| Drone position in local frame | `geometry_msgs/PoseStamped` |
| String command (GOTO, PAUSE…) | `std_msgs/String` |
| Custom mine detection report | Define your own `.msg` file |

Custom message example — we would define `mine_interfaces/msg/MineDetection.msg`:
```
# MineDetection.msg
float64 latitude
float64 longitude
float32 confidence
string mine_type        # "buried" or "surface"
sensor_msgs/Image thermal_image
```

---

## Core Concept 6: Packages and Workspaces

All ROS2 code lives in **packages**. A package is a directory with:
- `package.xml` — metadata (name, version, dependencies)
- `setup.py` — Python package setup
- Your node Python files

Multiple packages live in a **workspace**:
```
ros2_ws/
├── src/
│   ├── drone_thermal/       ← package for thermal camera node
│   ├── drone_detection/     ← package for vision pipeline
│   ├── drone_flight/        ← package for ArduPilot/pymavlink control
│   ├── drone_master/        ← package for master coordinator
│   └── mine_interfaces/     ← package for custom message types
├── build/                   ← compiled output (auto-generated)
├── install/                 ← installed packages (auto-generated)
└── log/                     ← build logs (auto-generated)
```

Build everything with:
```bash
cd ros2_ws
colcon build
source install/setup.bash   # make ROS2 find your packages
```

---

## Core Concept 7: Launch files

Instead of starting 6 terminal windows and running one node in each, a **launch file**
starts everything at once.

```python
# launch/slave_drone.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_thermal',
            executable='thermal_camera_node',
            namespace='drone1',
            name='thermal_camera',
        ),
        Node(
            package='drone_detection',
            executable='vision_filter_node',
            namespace='drone1',
            name='vision_filter',
        ),
        Node(
            package='drone_flight',
            executable='flight_controller_node',
            namespace='drone1',
            name='flight_controller',
        ),
    ])
```

Run with: `ros2 launch drone_thermal slave_drone.launch.py`

This is equivalent to our `launch.py` multi-threaded launcher at the repo root.

---

## Core Concept 8: DDS and discovery

ROS2 uses **DDS (Data Distribution Service)** under the hood. DDS handles:
- Finding nodes on the network automatically (no IP addresses needed!)
- Matching publishers to subscribers
- Handling message serialisation

Two drones on the same WiFi network with the same `ROS_DOMAIN_ID` will automatically
discover each other and communicate. No manual socket setup required.

```bash
# Set on all machines that should communicate:
export ROS_DOMAIN_ID=42

# If you want to isolate drones (separate domains):
# Drone 1: export ROS_DOMAIN_ID=1
# Drone 2: export ROS_DOMAIN_ID=2
# (they won't see each other's topics)
```

For multiple drones on the same domain, use **namespaces** to separate their topics:
- `/drone1/thermal/frame` — drone 1's thermal frames
- `/drone2/thermal/frame` — drone 2's thermal frames

---

## Comparison table: our system vs ROS2

| Feature | Our current system | ROS2 equivalent |
|---------|-------------------|-----------------|
| Start all processes | `launch.py` (Python threads) | `ros2 launch ...` |
| Inter-drone data | Manual UDP sockets | ROS2 topics over DDS |
| Commands (master→slave) | Manual TCP sockets | ROS2 services / actions |
| Mine reports | TCP JSON + length prefix | ROS2 topic (`MineDetection` msg) |
| Path planning | Custom A* in `master/app.py` | Nav2 stack or custom node |
| Coverage grid | `grid_map.py` (custom dict) | `nav2_map_server` or custom node |
| Debugging | `print()` statements | `ros2 topic echo /drone1/status` |
| Visualisation | `tools/pc_visualizer.py` | RViz2 (built-in ROS2 tool) |
| Logging | Files / print | `ros2 bag record` (records everything) |
