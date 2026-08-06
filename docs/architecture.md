# System Architecture — S.A.F.E. Drone Swarm

## Overview

S.A.F.E. (Swarm Autonomous Field Explorer) is a **master–slave drone swarm** where:

- **3 slave drones** fly autonomously over the minefield, scan with thermal sensors, detect mines, and broadcast telemetry
- **1 master drone** (same hardware, different software role) aggregates data from all slaves, runs a web server + A* path planner, and accepts voice/gesture commands from the operator's phone

All computation is **onboard**. No GPS for navigation. No external servers. No off-board control.

---

## High-level data flow

```
 ┌─────────────────────────────────────────────────────────────────┐
 │                         SLAVE DRONE (×3)                        │
 │                                                                  │
 │  MLX90640 Thermal Sensor                                         │
 │       │ (raw 24×32 float32 frames via C++ pipe)                  │
 │       ▼                                                          │
 │  bin/mlx_stdout ──────────────────────────────────────────┐      │
 │                                                           │      │
 │  00_preflight_calib.py (run once before each mission)     │      │
 │    → config/fpn_pattern.npy  (thermal noise correction)   │      │
 │    → config/origin_state.json  (GPS origin + heading)     │      │
 │                                                           │      │
 │  main_orchestrator_competition.py  (flight loop)          │      │
 │    │                                                      │      │
 │    ├─ PipeCamera.read_frame()  ◄──────────────────────────┘      │
 │    │    reads raw frames from mlx_stdout C++ pipe                │
 │    │                                                              │
 │    ├─ 06_surface_filter.process_surface_stack()                  │
 │    │    detects on-ground discs (ΔT +3 to +40°C)                 │
 │    │    → (dx, dy, conf) in 640×480 pixel space                  │
 │    │                                                              │
 │    ├─ 02_vision_filter.process_memory_stack()                    │
 │    │    detects buried mines (ΔT +0.15 to +1.25°C)               │
 │    │    → (dx, dy, conf)                                          │
 │    │                                                              │
 │    ├─ 03_persistence.PersistenceFilter.verify()                  │
 │    │    confirms detection isn't GPS drift or a rock              │
 │    │    → bool (confirmed / ghost)                                │
 │    │                                                              │
 │    ├─ 04_coordinate_math.get_pixels_to_meters()                  │
 │    │    pixel offset → body-frame metres using FOV geometry       │
 │    │                                                              │
 │    ├─ 04_coordinate_math.compute_global_gps()                    │
 │    │    body metres + heading → GPS (lat, lon)                    │
 │    │                                                              │
 │    ├─ 05_map_verifier.verify_and_log()                           │
 │    │    cross-check against pre-known buried mine positions       │
 │    │                                                              │
 │    ├─ 08_comms_link.DroneTunnel.send_anomaly_data()              │
 │    │    TCP → master port 5000                                    │
 │    │                                                              │
 │    ├─ grid_map.GRID.mark_position()  (passive, every tick)       │
 │    │    records which cells the sensor footprint covered          │
 │    │                                                              │
 │    ├─ udp_channel.UDPSender  →  master:14550  (5 Hz)             │
 │    │    {lat, lng, altitude, heading, armed, airborne, grid…}     │
 │    │                                                              │
 │    └─ tcp_channel.TCPCommandServer  ←  master:14560              │
 │         GOTO / PAUSE / RESUME / LAND / SIDE_MOVE / ARM_TAKEOFF   │
 │                                                                   │
 │  tf_luna_failsafe.py  (background task)                           │
 │    TF-Luna LIDAR: if obstacle < 1.0 m → emergency sidestep        │
 └─────────────────────────────────────────────────────────────────┘

                         ↕ WiFi (no internet, no GPS)

 ┌─────────────────────────────────────────────────────────────────┐
 │                         MASTER DRONE                            │
 │                                                                  │
 │  app.py (Flask + threads)                                        │
 │    │                                                              │
 │    ├─ Thread 1: Flask web server (HTTPS :443)                    │
 │    │    /              → serves index.html (map UI)              │
 │    │    /api/drone_update  ← slave mine reports (TCP :5000)      │
 │    │    /api/status       → drone positions, mines, coverage     │
 │    │    /api/audio_chunk  ← phone browser streams mic audio      │
 │    │    /api/get_my_location → A* safe path for person-at-risk   │
 │    │    /api/set_origin   → calibrate coordinate system          │
 │    │                                                              │
 │    ├─ Thread 2: Vosk offline speech recognizer                   │
 │    │    reads from audio_queue (filled by /api/audio_chunk)      │
 │    │    maps speech → intent → pending_commands                  │
 │    │                                                              │
 │    ├─ A* path planner (calculate_path())                         │
 │    │    field-aware: avoids buried mines + forbidden zones        │
 │    │    0.5 m grid, 8-directional moves                           │
 │    │                                                              │
 │    ├─ fieldmap.py  (shared with slaves)                          │
 │    │    BURIED_MINES, FORBIDDEN_ZONES, lane geometry             │
 │    │                                                              │
 │    └─ grid_map.GridMap  (master merges grids from all 3 slaves)  │
 │         coverage_pct() → % of field scanned                      │
 │                                                                   │
 │  index.html (phone browser UI)                                    │
 │    ├─ Live map: drone positions, mine markers, coverage cells     │
 │    ├─ getUserMedia() → streams phone mic → /api/audio_chunk      │
 │    └─ Shows Vosk transcript so operator can confirm commands      │
 └─────────────────────────────────────────────────────────────────┘

                    ↕ Phone browser (HTTPS, same WiFi)

 ┌──────────────────────────────┐
 │   Operator's Phone           │
 │   Browser: https://<master>/ │
 │   Voice commands → Vosk →   │
 │   start / pause / land / …  │
 └──────────────────────────────┘
```

---

## Slave pipeline module map

| File | Pipeline step | What it does |
|------|--------------|--------------|
| `00_preflight_calib.py` | Pre-mission | Captures FPN noise map; locks GPS origin + heading |
| `bin/mlx_stdout` | Sensor I/O | C++ binary streams raw 24×32 float32 thermal frames |
| `main_orchestrator_competition.py` | Coordinator | Flight loop, step-by-step scan, queues detections |
| `02_vision_filter.py` | Detect buried | ΔT +0.15–+1.25°C bidirectional filter, blob detect |
| `06_surface_filter.py` | Detect surface | ΔT +3–+40°C, catches surface discs (MDF, HDPE) |
| `03_persistence.py` | Confirm | Re-hovers, re-captures; ensures detection isn't noise |
| `04_coordinate_math.py` | Localise | Pixel offset → body metres → GPS (lat/lon) |
| `05_map_verifier.py` | Cross-check | Compare against pre-known buried mine positions |
| `08_comms_link.py` | Report | TCP packet with thermal image + GPS → master |
| `fieldmap.py` | Geometry | All field boundaries, mine positions, scan lanes |
| `grid_map.py` | Coverage | 0.5 m cell dict; marks scanned + detection cells |
| `tf_luna_failsafe.py` | Safety | Background LIDAR monitor; sidestepped on obstacle |
| `tcp_channel.py` | Comms in | Receives GOTO/PAUSE/LAND/SIDE_MOVE from master |
| `udp_channel.py` | Comms out | Broadcasts position + grid snapshot to master |

---

## Multi-pass scan strategy

The 15 m × 60 m field is divided into **4 passes** (columns), each drone flying a lane:

```
West ←──────────────────────────────────────────────────────────────→ East

Pass 1:   slave_1@5.0m  slave_2@6.4m  slave_3@7.8m
Pass 2:   slave_1@9.2m  slave_2@10.6m slave_3@11.2m  (terminates West of Pole)
Pass 3:   slave_1@15.2m slave_2@16.6m slave_3@18.0m  (resumes East of Pole)
Pass 4:   slave_1@19.4m slave_2@20.8m slave_3@22.0m
```

- **Lane step**: 1.4 m (≈ sensor footprint at 1.5 m alt with 55° FOV)
- **Cell step**: 0.5 m forward per hover-and-scan
- **SIDE_MOVE protocol**: after each pass, master commands each slave (lead → mid → tail) to shift to the next lane

The A* planner on each slave handles the forbidden Pole and Statue zones automatically.

---

## Key design decisions

### No GPS for navigation
Competition rules ban GNSS. The drone uses MAVSDK's `goto_location()` with relative waypoints derived from the local coordinate system (origin = start position, +X = East, +Y = North). The FPN-corrected thermal camera is the only sensor for mine detection.

### Two-stage detection
Surface discs (+3–+40°C) and buried mines (+0.15–+1.25°C) have completely different thermal signatures. Two separate filters run on the same frame stack. If either fires, the drone hovers and re-captures for persistence verification.

### Passive grid marking
`grid_map.py` is called every UDP telemetry tick (5 Hz) using the *real GPS position* (not the planned waypoint). This means GPS drift naturally fills in slightly overlapping cells — which is correct because the sensor footprint covered that area anyway.

### Vosk offline voice recognition
No internet dependency. The Vosk model runs entirely on the master Pi. The phone browser streams raw 16-bit PCM via `/api/audio_chunk` POST; Vosk decodes it and maps to intents (start / pause / land / …).

---

## Adding a new slave drone

1. Copy the `slave/` directory to the new drone's Pi
2. Set `DRONE_ID=slave_4` and `MASTER_IP=...`
3. Add `slave_4` lane entries in `fieldmap.py` → `PASS_LANES`
4. Run `00_preflight_calib.py` on the field
5. Start `main_orchestrator_competition.py`

The master automatically merges any drone's telemetry and grid data as long as it uses UDP port 14550 and the correct packet format.
