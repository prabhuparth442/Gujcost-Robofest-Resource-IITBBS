# Competition Brief — Robofest Gujarat 6.0
## Aerial Robotics: Minefield Navigation Challenge

> Source: `Drone_Robofest_PS.pdf`  
> Organiser: GUJCOST, Dept. of Science & Technology, Govt. of Gujarat

---

## The challenge in one sentence

Deploy a swarm of autonomous miniature drones to sweep a minefield, detect mines, create a 1-metre-clearance safe path, and guide a person-at-risk from start to exit — entirely onboard, no GPS, no external control.

---

## Two stages

### Stage 1 — Proof of Concept (PoC)

**Field:** 10 m × 40 m | ~20 simulated mines

Demonstrate a **scaled-down version** with:

1. Autonomous takeoff and stable hover
2. Navigate through 3–5 static obstacles (poles/trees)
3. Identify and mark ≥5 "simulated mines" (visible markers)
4. Generate a simple safe path using LEDs or visual indicators
5. Execute ≥1 gesture/voice/button command

**Constraints:** Max weight 500 g (excl. compute). Minimum 1 drone (swarm optional at PoC).

**Scoring (PoC):**

| Criteria | Marks |
|----------|-------|
| Stability & Control | 15 |
| Obstacle Avoidance | 20 |
| Detection Accuracy | 20 |
| Basic AI/Logic | 15 |
| Path Visualization | 10 |
| Innovation | 10 |
| Presentation | 10 |
| **Total** | **100** |

---

### Stage 2 — Grand Finale

**Field:** 15 m × 60 m | ~40 randomly placed mines

Deploy a **minimum 3-drone swarm** to:

1. **Swarm Deployment** — coordinated multi-drone operation at ≥20 ft, formation for ≥3 min
2. **Autonomous Mapping** — generate real-time minefield map
3. **Dynamic Path Planning** — ensure 1-metre safe clearance, re-route on deviation
4. **Human Tracking & Guidance** — guide person using visual cues (LEDs, lights)
5. **Mission Completion** — person crosses safely within 10 min, drones land autonomously

**Scoring (Grand Finale):**

| Task | Points |
|------|--------|
| Safe takeoff + swarm activation | 10 |
| Gesture/voice command recognition | 10 |
| Swarm formation in sky | 10 |
| Mine detection & correct mapping | **25** |
| Safe path creation & marking | 15 |
| Person crosses safely | **20** |
| Time bonus | 10 |
| Collision or unsafe proximity | **−10** |
| Drone crash or surface contact | **−20** |
| **TOTAL** | **100** |

---

## Critical rules

### No external navigation (most important)
> GNSS/GPS or any external positioning is **strictly prohibited**. All localisation, navigation, mapping, and decision-making must use onboard sensors and onboard computation only.

This is why we use the MLX90640 thermal camera + TF-Luna LIDAR for positioning, and MAVSDK relative waypoints derived from a local coordinate system.

### No external data links
> No communication with external computers, cloud servers, base stations, or remote operators during the competition.

Drones may only communicate with **each other** (inter-drone swarm coordination) using license-exempt wireless bands.

### Emergency kill switch
> An independent emergency kill switch is **mandatory** for all teams. The kill-switch link shall be used exclusively for flight termination or motor disarming and shall **not** carry any navigation, control, mission, or payload-related commands.

### Weight limit
> All-Up Weight (AUW) ≤ 750 g per drone (±10 g tolerance), including airframe, battery, propellers, motors, ESCs, flight controller, compute board, sensors, camera(s), comms modules, wiring, landing gear, and payload mechanism.

### Technical scrutiny
> The Technical Advisory Committee (TAC) may inspect the communication architecture and software. Their decision is final.

---

## Mine specifications

| Property | Detail |
|----------|--------|
| Field size | 15 m wide × 60 m long (Stage 2) |
| Total mines | ~40 |
| Surface mines | ~75% — flat discs on ground, opaque, different colours |
| Buried mines | ~25% — discs buried 30–50 mm deep in sand/mud/soil |
| Buried markers | Coloured peg, painted ring, or disturbed soil above each buried mine |
| Disc sizes | 20–30 cm diameter (circular or elliptical) |
| Exclusion zone | 1.0 m horizontal radius from each mine centre |
| Obstacles | Poles (≤5 m height), trees — exact locations undisclosed until competition |

**Detection note:** Buried mines are indicated by a **surface marker** (peg/ring/disturbed soil). Use onboard computer vision to detect these markers — do not rely on thermal alone for buried mines.

---

## Field zones

```
0 m ────────────────── 15 m wide ─────────────────── 15 m
│  Start Zone (1m×15m)                               │
│  Drone deployment + human starting area            │
│─────────────────────────────────────────────────  │
│                                                    │
│           Minefield Zone (58m×15m)                 │
│           ~40 mines, poles, trees                  │
│                                                    │
│─────────────────────────────────────────────────  │
│  Exit Zone (1m×15m)                               │
│  Finish line for person-at-risk                   │
0 m ───────────────────────────────────────────── 60 m
```

---

## Tie-breaker (in order)

1. Maximum challenges completed
2. Minimum penalty points
3. Fastest completion time
4. TAC decision is final

Only **1 re-run** permitted (with score penalty).

---

## What the judges look for

From the PS:
- **Autonomous navigation** — no hand-holding; the drone must find its own way
- **Sensor integration** — thermal, optical, or IR used effectively
- **Basic AI/decision-making** — path planner, mine detection logic
- **Drone stability and control** — no crashes, clean hover
- **Innovation** — novel approaches to detection or coordination are rewarded

Our implementation addresses all five areas:
- Navigation: local coordinate system + A* path planner, no GPS
- Sensors: MLX90640 thermal (two-band), TF-Luna LIDAR
- AI: two-stage detection pipeline, persistence verification, A* path planning
- Stability: MAVSDK PX4 with automatic failsafe sidestep
- Innovation: Vosk offline voice control, passive coverage grid, dual thermal bands
