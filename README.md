# Gujcost Robofest 6.0 — Aerial Robotics: Minefield Navigation

**Team IIT BBS** · Autonomous 3-drone swarm for thermal mine detection

[![GitHub Pages](https://img.shields.io/badge/GitHub%20Pages-live-green)](https://prabhuparth442.github.io/Gujcost-Robofest-Resource-IITBBS/)

---

## What this is

A working multi-drone system that autonomously scans a 15 m × 60 m field, detects buried and surface anti-personnel mines using thermal IR, and reports GPS coordinates back to a master controller — all without GPS on the drones.

**Competition constraints:** 3 drones max, each ≤ 750 g, 10-minute mission window, no GPS inside field, no human intervention after start.

---

## System overview

```
Master Pi (ground)          Slave Drone × 3 (in air)
─────────────────           ──────────────────────────
Flask HTTPS server          main_orchestrator.py
Vosk speech recognition     MLX90640 thermal camera (C++ pipe)
A* path planner             TF-Luna LIDAR failsafe
Grid map aggregator         ArduCopter / pymavlink (GUIDED mode)
TCP commander   ←──TCP──→   tcp_channel.py  (port 14560)
                ←──UDP──→   udp_channel.py  (port 14550, 5 Hz)
                ←──TCP──→   mine reports    (port 5000)
```

Each slave flies a 4-pass lane pattern (`PASS_LANES` dict), scans at 2 m altitude, and pipes thermal frames from a C++ MLX90640 driver into Python for detection. Mines are reported as local coordinates (origin = start GPS fix).

---

## Repository structure

```
.
├── slave/                  ← Flight code for each scanning drone (Raspberry Pi)
│   ├── main_orchestrator.py         Entry point — runs the full scan mission
│   ├── main_orchestrator_competition.py  Competition-locked version
│   ├── 02_vision_filter.py          Buried mine detector (ΔT ±0.15–1.25 °C)
│   ├── 06_surface_filter.py         Surface disc detector (ΔT +3–40 °C)
│   ├── 03_persistence.py            12-frame re-hover verification
│   ├── 04_coordinate_math.py        GPS-local coordinate conversion
│   ├── 05_map_verifier.py           Grid deduplication + hazard bitmask
│   ├── fieldmap.py                  A* path planner (8-dir, 0.5 m grid)
│   ├── grid_map.py                  Sparse occupancy grid
│   ├── tf_luna_failsafe.py          Asyncio obstacle avoidance
│   ├── tcp_channel.py               TCP comms to master
│   ├── udp_channel.py               UDP telemetry broadcast
│   ├── swarm_state.py               Shared inter-pass state
│   ├── src/                         C++ MLX90640 driver + BCM2835
│   ├── config/                      Per-drone YAML configs
│   └── README.md
│
├── master/                 ← Ground controller (Raspberry Pi or laptop)
│   ├── app.py              Flask HTTPS + Vosk speech recognition
│   ├── grid_map.py         Global grid aggregator
│   ├── fieldmap.py         Field coordinate system
│   ├── swarm_state.py      Drone state tracker
│   ├── tcp_channel.py / tcp_commander.py / udp_channel.py
│   ├── config/             Swarm config YAML
│   └── README.md
│
├── docs/                   ← Documentation
│   ├── getting_started.md       Zero-to-flying setup guide (start here)
│   ├── architecture.md          Full system architecture
│   ├── competition_brief.md     PS summary and constraints
│   ├── field_layout.md          15×60 m field coordinate system
│   ├── concepts.md              Deep-dives: thermal physics, A*, grid bitmask…
│   └── reports/                 Historical SOPs and technical notes
│
├── research/               ← Research background
│   ├── 01_thermal_mine_detection.md
│   ├── 02_gps_denied_navigation.md
│   ├── 03_swarm_coordination.md
│   ├── 04_path_planning.md
│   ├── 05_sensor_hardware.md
│   ├── 06_implementation_references.md
│   └── papers/             12 reference PDFs (D* Lite, probabilistic robotics…)
│
├── migration/              ← MAVSDK → ROS2 migration guide
│   ├── 01_ros2_concepts.md
│   ├── 02_mavsdk_to_ros2.md
│   ├── 03_ardupilot_ros2_bridge.md  (AP_DDS setup)
│   ├── 04_swarm_in_ros2.md
│   └── 05_migration_plan.md   (7-phase, 6–8 week roadmap)
│
├── tools/                  ← PC-side debug and visualisation tools
│   ├── pc_thermal_viewer.py    Live MLX90640 stream viewer
│   ├── pc_binary_viewer.py     Grid map renderer
│   ├── pc_viewer.py            General telemetry viewer
│   ├── mlx_stdout.cpp          C++ thermal driver (pipe mode)
│   └── README.md
│
├── hardware/               ← Hardware setup guide
│   └── README.md               Wiring, BCM2835, TF-Luna, MLX90640
│
├── simulation/             ← Earlier simulation iterations
│   ├── Iteration_1/        Gazebo single-drone sim
│   ├── Iteration_2/        Enhanced thermal field sim
│   └── Virtual_Camera_Feed/  Virtual MLX sensor
│
├── scripts/                ← Git workflow scripts
│   ├── 00_git_init.sh          One-time repo connect (already done)
│   ├── daily_push.py           Gradual daily commit pusher
│   └── push_schedule.json      25-day push schedule
│
├── archive/                ← Old code and development experiments
├── launch.py               ← Quick-start launcher (slave orchestrator)
├── Drone_Robofest_PS.pdf   ← Official problem statement
└── Planned_Roadmap.png     ← Project roadmap image
```

---

## Quickstart

> Full setup walkthrough: [`docs/getting_started.md`](docs/getting_started.md)

### 1. Hardware needed per drone

| Part | Spec |
|------|------|
| Frame | <750 g total AUW |
| FC | SpeedyBee F405 running ArduCopter |
| Companion | Raspberry Pi 4 (4 GB) |
| Thermal | MLX90640 32×24 IR (I2C, 8 Hz) |
| LIDAR | TF-Luna (UART, 100 Hz) |

### 2. Slave drone setup

```bash
# On each slave Pi:
cd slave/
pip install -r requirements.txt

# Build C++ thermal driver
cd src/
make

# Configure this drone
nano config/drone_config.yaml   # set DRONE_ID, MASTER_IP, LANE assignment

# Calibrate thermal sensor (30-frame FPN baseline)
python3 00_preflight_calib.py

# Run mission
python3 launch.py
```

### 3. Master setup

```bash
cd master/
pip install -r requirements.txt

# Generate TLS cert (needed for browser microphone access)
openssl req -x509 -newkey rsa:4096 -keyout key.pem -out cert.pem -days 365 -nodes

# Start master controller
python3 app.py
# Opens HTTPS dashboard at https://<master-ip>:5000
```

### 4. SITL testing (no hardware)

See [`docs/getting_started.md`](docs/getting_started.md) for full ArduCopter SITL + MAVProxy setup.

---

## Key design decisions

**Why thermal IR for buried mines?**
Burial disturbs soil's thermal inertia. During morning warm-up or evening cool-down a buried void shows ±0.15–1.25 °C contrast against surrounding soil. Two separate filters handle buried mines vs surface discs. See [`docs/concepts.md`](docs/concepts.md) and [`research/01_thermal_mine_detection.md`](research/01_thermal_mine_detection.md).

**Why no GPS on drones?**
Competition rules prohibit GPS inside the field. Drones use a local coordinate frame: origin locked at the start GPS fix (averaged compass + 5-sample GPS mean), then dead-reckoned via ArduPilot EKF3 velocity integration.

**Why 4-pass lane strategy?**
A single 15×60 m pass at 55° FOV thermal width leaves gaps. Four offset passes at 1.4 m lane step give ~2× overlap for confident detection. The `PASS_LANES` dict and `SIDE_MOVE` TCP protocol coordinate all three slaves sequentially with 8 s stagger.

**Persistence verification**
Any thermal hit triggers a re-hover: 12 additional frames, pixel drift check < 1.5 m physical. Eliminates atmospheric shimmer false positives.

---

## GitHub Pages (live dashboards)

| Dashboard | Link |
|-----------|------|
| Project overview | [index.html](https://prabhuparth442.github.io/Gujcost-Robofest-Resource-IITBBS/index.html) |
| Information pipeline | [Information_pipeline.html](https://prabhuparth442.github.io/Gujcost-Robofest-Resource-IITBBS/Information_pipeline.html) |
| Planned roadmap | [plan.html](https://prabhuparth442.github.io/Gujcost-Robofest-Resource-IITBBS/plan.html) |

---

## For newcomers

Start with **[`docs/getting_started.md`](docs/getting_started.md)** — it explains what every piece of hardware does and walks you from zero to a working SITL run.

Then read **[`docs/architecture.md`](docs/architecture.md)** for the full system design, and **[`docs/concepts.md`](docs/concepts.md)** for deep-dives into specific algorithms.

If you're planning to migrate to ROS2, the entire **[`migration/`](migration/)** folder is a self-contained guide with code examples.

---

## License

Educational and research use. Reference PDFs in `research/papers/` remain property of their respective authors.
