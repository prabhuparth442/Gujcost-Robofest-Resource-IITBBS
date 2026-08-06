# Slave Drone — S.A.F.E. Competition Software

Each of the **3 scanning drones** runs this code. The slave drone:
1. Takes off autonomously and hovers at 1.5 m
2. Sweeps assigned scan lanes cell by cell (0.5 m steps)
3. Captures thermal frames at each position
4. Detects mines via two independent filters
5. Broadcasts telemetry to the master drone
6. Receives commands (PAUSE, SIDE_MOVE, LAND) from master via TCP

---

## File map (in pipeline order)

```
slave/
│
├── 00_preflight_calib.py       ← RUN FIRST (before every flight)
│                                  Captures FPN noise pattern, locks GPS+heading
│
├── main_orchestrator_competition.py   ← MAIN ENTRY POINT
│                                         Controls flight loop + detection pipeline
│
│   ── Detection pipeline (called by orchestrator) ──────────────────────────
│
├── 02_vision_filter.py         ← Buried mine detector
│                                  Bidirectional thermal filter ΔT ±0.15–1.25°C
│                                  Returns (dx, dy, conf) in 640×480 pixel space
│
├── 06_surface_filter.py        ← Surface disc detector
│                                  High-signal thermal filter ΔT +3–40°C
│                                  Returns (dx, dy, conf)
│
├── 03_persistence.py           ← Confirmation gate
│                                  Re-hovers, re-captures; rejects GPS-drift ghosts
│                                  Returns bool (confirmed / ghost)
│
├── 04_coordinate_math.py       ← GPS localisation
│                                  Pixel offset → body-frame metres → GPS (lat, lon)
│
├── 05_map_verifier.py          ← Cross-reference with known mines
│                                  Checks confirmed detection against A4_map.json
│
├── 08_comms_link.py            ← Sends mine reports to master
│                                  TCP packet with thermal image + GPS
│
│   ── Field geometry + grid ─────────────────────────────────────────────────
│
├── fieldmap.py                 ← Single source of truth for all field geometry
│                                  BURIED_MINES, FORBIDDEN_ZONES, PASS_LANES
│                                  gps_to_local(), local_to_gps()
│
├── grid_map.py                 ← Live coverage map
│                                  Marks scanned/detected cells (0.5 m resolution)
│                                  Sends grid snapshots to master via UDP
│
│   ── Communications ────────────────────────────────────────────────────────
│
├── tcp_channel.py              ← TCP server on :14560
│                                  Receives GOTO / PAUSE / RESUME / LAND / SIDE_MOVE
│
├── udp_channel.py              ← UDP sender to master:14550
│                                  Broadcasts {lat, lng, alt, heading, grid...} at 5 Hz
│
├── swarm_state.py              ← Shared swarm state data structure
│
│   ── Safety ─────────────────────────────────────────────────────────────────
│
├── tf_luna_failsafe.py         ← Background LIDAR monitor
│                                  If obstacle < 1.0 m for 3 consecutive frames
│                                  → emergency sidestep (West or East)
│
│   ── Testing ────────────────────────────────────────────────────────────────
│
├── test_mission.py             ← Integration test: full simulated mission
├── test_suite.py               ← Unit tests for individual modules
├── main_orchestrator.py        ← Generic (non-competition) orchestrator
│
│   ── Config + binaries ──────────────────────────────────────────────────────
│
├── SETUP.md                    ← Installation steps
├── requirements.txt
├── config/
│   ├── fpn_pattern.npy         ← Created by 00_preflight_calib.py
│   ├── origin_state.json       ← Created by 00_preflight_calib.py
│   └── A4_map.json             ← Created at startup by fieldmap.generate_a4_map_json()
└── bin/
    ├── mlx_stdout              ← C++ binary: streams raw MLX90640 frames to stdout
    └── camera_bridge           ← C++ binary: FPN calibration frame capture
```

---

## Quick start

### 1. Install dependencies

```bash
sudo apt-get install -y python3-pip libopencv-dev
pip3 install -r requirements.txt
```

### 2. Set identity (different for each drone)

```bash
export DRONE_ID=slave_1        # slave_1, slave_2, or slave_3
export MASTER_IP=10.42.0.1    # master drone's IP
export MAVSDK_ADDR=udp://:14540
export LUNA_PORT=/dev/serial0  # TF-Luna port (optional)
```

### 3. Preflight calibration (MANDATORY before every flight)

```bash
# Point the thermal sensor at a uniform surface (lens cap or flat sky)
# Drone must be at the field start position, motors off, facing scan direction
python3 00_preflight_calib.py
```

This creates two files:
- `config/fpn_pattern.npy` — the thermal sensor's fixed-pattern noise (dead pixels, response non-uniformity). Without this, the vision filters see noise as detections.
- `config/origin_state.json` — the GPS position and compass heading at start. Without this, `04_coordinate_math.py` can't convert pixel positions to GPS coordinates.

### 4. Start MAVProxy (in a separate terminal)

```bash
# If using USB connection to flight controller:
mavproxy.py --master=/dev/ttyACM0 --out=udp:127.0.0.1:14540
# If using UART (RPi GPIO pins):
mavproxy.py --master=/dev/serial0 --out=udp:127.0.0.1:14540
```

### 5. Run the competition mission

```bash
python3 main_orchestrator_competition.py
```

---

## Environment variables reference

| Variable | Default | Set to |
|----------|---------|--------|
| `DRONE_ID` | `slave_1` | `slave_1`, `slave_2`, or `slave_3` |
| `MASTER_IP` | `10.42.0.1` | Master drone's IP on swarm network |
| `MAVSDK_ADDR` | `udp://:14540` | MAVSDK connection string |
| `LUNA_PORT` | `/dev/serial0` | TF-Luna serial port |

---

## How the flight loop works

```
Startup:
  1. Import all pipeline modules (02, 03, 04, 05, 06, 08)
  2. Load FPN pattern from config/fpn_pattern.npy
  3. Pre-plan ALL 4 passes using A* before takeoff
  4. Connect to flight controller (MAVSDK)
  5. Start UDP telemetry sender
  6. Start TCP command server (listens for master commands)
  7. Optionally: start TF-Luna lidar failsafe task

Flight loop (per pass):
  For each pass (1–4):
    Wait for SIDE_MOVE command from master (pass > 1)
    Fly to start of pass lane
    For each 0.5m step cell:
      1. Fly to cell GPS coordinate
      2. Hover 0.6s (sensor settle)
      3. Capture 10 thermal frames from C++ pipe
      4. Run 06_surface_filter on frames
      5. If surface disc detected (conf ≥ 0.45):
           Queue DiscCandidate for persistence check
      6. If candidate in queue:
           Force hover 1.0s
           Re-capture 12 frames
           Run 03_persistence to confirm not GPS drift
           If confirmed:
             Run 04_coordinate_math → GPS coordinates
             Run 05_map_verifier → cross-check
             Run 08_comms_link → send to master
             Mark detection in grid_map

  After all passes: LAND
```

---

## Ports used

| Port | Protocol | Direction | Purpose |
|------|----------|-----------|---------|
| 14550 | UDP | → master | Telemetry (position, heading, grid) at 5 Hz |
| 14560 | TCP | ← master | Commands (GOTO, PAUSE, LAND, SIDE_MOVE) |
| 5000 | TCP | → master | Confirmed mine reports with thermal image |

---

## Troubleshooting

**"FPN missing — run 00_preflight_calib.py"**  
Run `00_preflight_calib.py` with the thermal sensor pointing at a uniform surface. Without FPN correction, every scan will produce many false positives.

**"Module import failed: 06_surface_filter"**  
The numbered modules (02_, 03_, etc.) are imported by name using `importlib`. Ensure all files are in the same directory as the orchestrator.

**"A* failed: lane x=..."**  
The A* planner couldn't find a path in that lane. Usually means the forbidden zone radii in `fieldmap.py` are too large. Check `POLE_FORBIDDEN_RADIUS_M` and `STATUE_FORBIDDEN_RADIUS_M`.

**"SIDE_MOVE: Awaiting master…" (stuck)**  
Master hasn't sent the SIDE_MOVE command yet. Check master is running and connected to the same network. The slave will wait indefinitely.

**High false-positive rate**  
Lower `CONFIDENCE_THRESHOLD` in `main_orchestrator_competition.py` (currently 0.45). Or re-run `00_preflight_calib.py` in better lighting/temperature conditions.
