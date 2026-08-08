# 03 — Swarm Coordination

## Our architecture in brief

Master–slave, centralised coordination:
- 1 master drone (Flask server, A* planner, mine database)
- 3 slave drones (scan, detect, report)
- All slaves send telemetry to master (UDP port 14550, 5 Hz)
- Master sends flight commands to slaves (TCP port 14560)
- Mine reports go slave → master (TCP port 5000, length-prefixed JSON)

This is a **fully centralised** swarm: the master is a single point of failure but also
the authoritative truth for field state. The trade-off was intentional for the competition's
small scale (3 drones, 15×60 m field, 10 minutes).

---

## Key Paper: Swarm-LIO2 — Decentralised LiDAR-Inertial Odometry for UAV Swarms

**Reference:** arXiv:2409.17798 — Zhu et al. (2024)  
**URL:** https://arxiv.org/abs/2409.17798  
**PDF:** https://arxiv.org/pdf/2409.17798

### What it says

- Proposes a **fully decentralised** swarm where every drone runs its own state estimator
- Communication: only low-bandwidth, low-dimensional packets exchanged — identity, ego-state,
  mutual observations, global extrinsic transformations
- Key components:
  - Reflectivity-based UAV detection (drones detect each other via LiDAR)
  - Trajectory matching for temporal offset initialisation
  - ESIKF (Error-State Iterated Kalman Filter) fusing LiDAR + IMU + mutual observations
- Plug-and-play: new drones can join the swarm automatically

### Contrast with our approach

| Aspect | Our system | Swarm-LIO2 |
|--------|-----------|-----------|
| Coordination | Centralised (master) | Decentralised (peer-to-peer) |
| Localisation | ArduPilot EKF3 + GPS origin | LiDAR-Inertial Odometry |
| Failure mode | Master crash = mission stop | Graceful degradation |
| Bandwidth | ~5 Hz UDP telemetry | Low-dim peer packets |
| Complexity | Low (easy to debug) | High (needs LiDAR on each drone) |

### For Robofest 6.0

Swarm-LIO2 is overkill for 3 drones in a 15×60 m field. But the bandwidth efficiency
principle is useful: our current UDP telemetry sends a full grid snapshot every 5th packet
(~1 Hz). For larger swarms (4+ drones), a diff-only grid update would save bandwidth.

The **plug-and-play** concept is directly applicable: currently adding a 4th slave drone
requires editing `fieldmap.py` PASS_LANES dict and `swarm_state.py`. A proper registration
protocol would let a drone announce itself to the master and get assigned scan lanes
automatically.

---

## Key Paper: Advancement Challenges in UAV Swarm Formation Control

**Reference:** MDPI Drones 8(7):320 (2024)  
**URL:** https://www.mdpi.com/2504-446X/8/7/320

### What it says

Comprehensive review covering:
- **Leader-follower formation**: one drone sets the path, others maintain relative offsets
- **Virtual structure**: all drones track points in a virtual rigid body
- **Behaviour-based**: each drone runs local rules (flocking/separation/cohesion)
- **Consensus-based**: distributed agreement algorithms

### Our formation model

We use a **fixed-lane formation** — not technically leader-follower but conceptually similar:
- Each drone is assigned a fixed X column (lane) in PASS_LANES
- Drones fly their lane independently; formation is implicit through lane assignment
- The SIDE_MOVE command shifts all drones to the next set of lanes between passes

This is the simplest possible formation control. It works because the field is a rectangle
and the task is a systematic sweep, not dynamic pursuit.

**For Robofest 6.0:** If drones need to regroup or swap lanes (e.g., a drone battery dies
mid-mission), there is no re-assignment logic. The surviving drones continue their own lanes
but don't cover the dead drone's lane. Implement a "drone dropout" handler in `master/app.py`:
when a slave stops sending telemetry for >15 s, redistribute its remaining scan waypoints
across the surviving drones.

---

## SIDE_MOVE Protocol — how inter-pass transitions work

Between each of the 4 scan passes, the master sends SIDE_MOVE to each slave sequentially:

```
master/app.py → execute_side_move()
    → TCP to drone_lead:  SIDE_MOVE {"dx": 1.4, "dy": 0}
    → wait 8 s
    → TCP to drone_mid:   SIDE_MOVE {"dx": 1.4, "dy": 0}
    → wait 8 s
    → TCP to drone_tail:  SIDE_MOVE {"dx": 1.4, "dy": 0}
```

The 8-second stagger prevents mid-air collisions when drones shift laterally.
`LANE_STEP_M = 1.4 m` — each pass shifts the formation 1.4 m East.

**Known weakness:** If the lead drone's SIDE_MOVE times out (drone busy), the master
currently retries once and then continues. If the drone actually moved only halfway,
subsequent pass lanes are misaligned. A position confirmation handshake after each
SIDE_MOVE would improve reliability.

---

## Grid Map Merging

Each slave maintains its own `GridMap` instance. The master merges all slaves' grids
every time it receives a telemetry packet that includes a grid snapshot:

```python
# master/app.py  (simplified)
def handle_telemetry(packet):
    if "grid" in packet:
        GLOBAL_GRID.merge_from(packet["grid"])
```

Merge semantics (from `grid_map.py`):
- DETECTION flag always wins over SCANNED
- Cells OR'd together (flags are additive bitmasks)
- Thread-safe via `threading.Lock()`

This means the master's global grid is a **union** of all coverage — no cell can go
from detected back to unvisited.

---

## Communication Architecture Diagram

```
                     ┌─────────────────┐
                     │  Master Drone   │
                     │  Flask :443     │
                     │  TCP cmd :14560 │
                     │  UDP tel :14550 │
                     │  Mine   :5000   │
                     └────────┬────────┘
                              │ WiFi AP (10.42.0.x)
              ┌───────────────┼───────────────┐
              ▼               ▼               ▼
       ┌──────────┐    ┌──────────┐    ┌──────────┐
       │  Slave 1 │    │  Slave 2 │    │  Slave 3 │
       │ (lead)   │    │  (mid)   │    │  (tail)  │
       └──────────┘    └──────────┘    └──────────┘

UDP telemetry: slave → master  (5 Hz)
TCP commands:  master → slave  (on demand)
Mine reports:  slave → master  (on detection, TCP port 5000)
Phone browser: phone → master  (HTTPS :443 + /api/audio_chunk)
```

---

## For Robofest 6.0 — Recommended Improvements

1. **Drone dropout handler** — if a slave stops sending telemetry >15 s, redistribute its lane
2. **SIDE_MOVE confirmation** — slave ACKs after reaching new position
3. **Dynamic lane assignment** — master assigns lanes at runtime instead of hardcoded PASS_LANES
4. **Bandwidth optimisation** — send grid diffs not full snapshots (saves ~60% UDP payload)
5. **Multi-master fallback** — designate slave_1 as backup master if primary crashes
