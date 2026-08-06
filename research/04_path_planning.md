# 04 — Path Planning

## What we use and why

**Coverage:** Fixed-lane boustrophedon (lawnmower) pattern defined by PASS_LANES in `fieldmap.py`.
Not computed dynamically — the lanes are pre-calculated offline for the known field dimensions.

**Obstacle avoidance:** A* planner in `master/app.py` computes per-pass, per-slave paths.
Hazard circles (buried mines) and forbidden zones (Pole, Statue) inflate obstacles on the grid.

---

## Our A* Planner — Implementation Details

Located in `master/app.py`. Key parameters:

```python
GRID_CELL_M = 0.5          # 0.5 m grid resolution
ITER_LIMIT  = 200_000      # max A* iterations before giving up
DIRECTIONS  = 8            # 8-directional movement
HAZARD_R    = MINE_AVOIDANCE_RADIUS_M   # 0.75 m per buried mine
FORBIDDEN_R_POLE   = 1.5 m
FORBIDDEN_R_STATUE = 2.5 m
```

The heuristic is standard Euclidean distance (not Manhattan — 8-directional movement means
diagonal shortcuts are valid, so Manhattan overestimates).

**Obstacle inflation:** Hazard circles are expanded by one cell (0.5 m) to create a safety
buffer. A* will never plan a path through a cell flagged HAZARD or FORBIDDEN.

---

## Key Paper: Comprehensive Review of Path Planning Techniques for UAVs

**Reference:** ACM doi/10.1145/3737280  
**URL:** https://doi.org/10.1145/3737280

### What it says

Surveys the main families of UAV path planning algorithms:

| Family | Examples | Best for |
|--------|---------|---------|
| Graph-based | A*, Dijkstra, D* | Known static environments |
| Sampling-based | RRT, RRT* | High-dimensional, unknown environments |
| Potential field | APF | Reactive obstacle avoidance |
| Bio-inspired | GA, PSO, ACO | Multi-objective optimisation |
| Learning-based | DRL | Dynamic, unpredictable environments |

### Why A* is right for us

Our environment is:
- Known field geometry (fixed 15×60 m)
- Static obstacles (known mine positions, Pole, Statue)
- Repeated execution (same field every run)
- Low-power hardware (Raspberry Pi — cannot run RRT* or DRL)

A* on a 0.5 m grid (60×120 = 7200 cells) with 200K iteration limit is extremely fast on Pi.
The 200K limit is hit only when the planner is asked to path through a fully blocked corridor.

---

## Key Paper: A Review of UAV Path-Planning Algorithms and Obstacle Avoidance Methods

**Reference:** MDPI Remote Sensing 16(21):4019 (2024)  
**URL:** https://www.mdpi.com/2072-4292/16/21/4019

### Obstacle avoidance methods compared

| Method | Pros | Cons |
|--------|------|------|
| A* (ours) | Optimal, complete | Replan cost if obstacles change |
| D* Lite | Efficient replanning | More complex |
| Potential field | Real-time reactive | Local minima |
| RRT | Handles high-dim | Not optimal |

### For our TF-Luna failsafe

The TF-Luna LIDAR runs a **reactive** avoidance: if obstacle < 1.0 m for 3 frames,
emergency sidestep regardless of A* path. This is essentially a potential field approach
for real-time close-range obstacles — complementing our global A* planner.

This two-layer approach (global A* + local reactive) is the standard industrial pattern
and is validated by this review.

---

## Key Paper: Land-Coverage Aware Path-Planning for Multi-UAV Swarms in Search and Rescue

**Reference:** arXiv:2505.08060 (2025)  
**URL:** https://arxiv.org/abs/2505.08060

### What it says

- Problem: coverage path planning for **irregular polygon** regions with internal holes
- Current CPP pipelines overfragment complex polygons → too many sub-regions → excess travel
- Solution: recursive dual-axis monotonicity criterion + cumulative gap severity metric
- Result: **lowest mean overhead in path length and completion time** across 13 CPP pipelines tested

### Relevance to our field

Our field is a simple rectangle — we don't have the irregular-polygon problem this paper solves.
But if Robofest 6.0 changes the field shape (non-rectangular boundary, exclusion zones with
irregular shapes), this algorithm would be directly applicable.

**For Robofest 6.0:** Read the field rules carefully. If the field has non-rectangular zones,
implement a proper cellular decomposition CPP instead of hardcoded PASS_LANES.

---

## Key Paper: Enhanced Multi-UAV Path Planning with Voronoi-Based Obstacle Modelling

**Reference:** Wiley doi 10.1155/2024/5114696 (2024)  
**URL:** https://doi.org/10.1155/2024/5114696

### What it says

- Uses Voronoi diagrams to model obstacle proximity (roads through the Voronoi graph naturally
  maximise clearance from all obstacles)
- Combined with Q-learning (reinforcement learning) for path optimisation
- Particularly effective for multi-UAV coordination where collision avoidance between drones
  is also required

### Connection to our code

We don't use Voronoi — our inter-drone collision avoidance is implicit (fixed lane spacing 1.4 m,
sequential SIDE_MOVE transitions). But the Voronoi approach would help if drones need to share
lanes or dynamically re-route.

---

## Key Paper: Incremental Coverage Path Planning for UAV Ground Mapping

**Reference:** Int. Journal of Micro Air Vehicles (SAGE, 2024)  
**URL:** https://journals.sagepub.com/doi/10.1177/17568293241262323

### What it says

- Combines boustrophedon motion with D* algorithm for unknown environments
- Drone starts with no map; D* handles re-routing when new obstacles discovered mid-flight

### For Robofest 6.0

We currently have a **known obstacle map** (fieldmap.py has all mine positions pre-surveyed).
If the field has unmapped obstacles (e.g., random surface clutter), switching from our static
A* to D* Lite for dynamic replanning would be worthwhile.

---

## Our 4-Pass Scan Strategy — Explained

```
PASS_LANES = {
    1: {drone_lead: x=2.0, drone_mid: x=3.4, drone_tail: x=4.8},
    2: {drone_lead: x=6.2, drone_mid: x=7.6, drone_tail: x=9.0},
    3: {drone_lead: x=10.4, drone_mid: x=11.8, drone_tail: x=13.2},
    4: {drone_lead: x=14.6, drone_mid: x=16.0, drone_tail: x=17.4},
}
LANE_STEP_M = 1.4 m       # horizontal spacing between lanes
SCAN_STEP_M = 0.5 m       # vertical step between waypoints
```

Field is 15 m wide (X: 0–15 m). 3 drones × 4 passes = 12 lanes × 1.4 m ≈ 16.8 m — slightly
overlapping at edges to avoid gaps.

Each pass, each drone sweeps its X lane from Y=0 to Y=-60 in 0.5 m steps. That's 120
waypoints per pass per drone, 1440 total waypoints for the full mission.

At nominal speed (0.3 m/s with sensor dwell at each step), one pass takes ~200 s. Four passes
= ~800 s = ~13 min. **This exceeds the 10-minute window.**

**The scan is currently too slow for the competition time limit.** See recommended improvements.

---

## For Robofest 6.0 — Recommended Improvements

1. **Speed up SCAN_STEP_M** — increase from 0.5 m to 0.8–1.0 m. Sensor footprint at 1.5 m
   altitude is ~1.56 m wide (FOOTPRINT_RADIUS_M=0.78 m → diameter ~1.56 m). 0.8 m steps
   still provide overlapping coverage.

2. **Reduce passes** — 2 or 3 passes instead of 4 if the full field is reachable. Evaluate
   whether 100% coverage is required by scoring rules or if partial (e.g., 80%) coverage is
   acceptable given mine detection accuracy.

3. **Adaptive scan** — skip waypoints where grid already shows SCANNED. After pass 1, passes
   2–4 could skip lanes already covered.

4. **D* Lite for dynamic replanning** — if a drone detects an unmapped obstacle, replan on
   the fly instead of freezing.

5. **Parallel mine reporting** — currently mine reporting (TCP + re-hover confirmation) pauses
   scanning for ~30 s. Implement async reporting so the drone continues scanning while the
   master processes the report.
