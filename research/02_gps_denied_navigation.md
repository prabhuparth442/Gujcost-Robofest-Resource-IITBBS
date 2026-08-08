# 02 — GPS-Denied Navigation

## Why this matters for us

The Robofest PS explicitly bans GPS during the 10-minute mission window. Our approach:
origin-lock at the start position (compass + GPS fix → `origin_state.json`), then use
pymavlink `SET_POSITION_TARGET_GLOBAL_INT` which internally uses ArduPilot EKF3 fusing barometer + optical flow
+ compass. This works, but drift accumulates. Understanding the alternatives helps us
harden the system and reduce positional errors over a 60 m field.

---

## Our Current Approach

**Origin-lock + dead reckoning via ArduPilot EKF3:**

1. Pre-flight: GPS fix + compass heading averaged over 5 samples → `origin_state.json`
2. In-flight: convert all waypoints to GPS delta from origin using `04_coordinate_math.py`
3. ArduCopter takes GPS target coordinates, fuses barometer + accelerometer + compass internally
4. `grid_map.py` paints coverage using *real GPS feedback* (not planned waypoints) — this
   means GPS drift gets absorbed into the coverage map passively

**Weakness:** Over 60 m of field, ArduPilot EKF3 drift at ~0.3–0.5 m/min means positional error
at far end of the field can reach 1–2 m after 10 minutes.

---

## Key Paper: Relative Positioning for Aerial Robot Path Planning in GPS-Denied Environment

**Reference:** arXiv:2409.10193 (2024)  
**URL:** https://arxiv.org/abs/2409.10193

### What it says

- Target application: bushfire monitoring swarms where GPS is unreliable
- Solution: **relative positioning** — each drone establishes its position relative to a
  known base of operations (not absolute world coordinates)
- Core idea: use structured landmarks visible to the drone cameras as anchors
- Achieves localisation good enough for search-and-reconnaissance without GPS

### Connection to our approach

Our "origin-lock" is conceptually similar — we establish a fixed reference at the start
point and everything is relative to it. The paper validates that this approach is viable.

**For Robofest 6.0:** The competition field has two fixed visual landmarks: the Pole and
the Statue. Their positions are pre-surveyed in `fieldmap.py`. A computer-vision step that
*detects and re-localises against these landmarks* mid-flight could correct accumulated drift.
This would be a significant improvement worth implementing.

---

## Key Paper: Autonomous Navigation for Drone Swarms in GPS-Denied Environments

**Reference:** PMC7256583  
**URL:** https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7256583/

### What it says

- Uses **structured learning** (a supervised ML approach) to train navigation policies in GPS-denied environments
- Key insight: when GPS is absent, the swarm needs a shared reference frame — typically
  established by the lead drone which has the most reliable localisation
- Demonstrates that master–slave architectures (our exact topology) handle GPS denial
  well because only the master needs reliable position; slaves can follow relative paths

### Connection to our architecture

We already use this pattern: the master drone is the reference (it holds `origin_state.json`
and the authoritative grid). Slaves operate relative to master-issued waypoints.

**For Robofest 6.0:** When a slave loses its ArduPilot EKF3 position estimate (EKF diverges), the
master can re-issue its last-known GPS position as a re-anchor command. Currently no such
re-anchor command exists in `tcp_channel.py`. Worth adding as a fallback.

---

## Key Paper: UAV Autonomous Navigation System Based on Air-Ground Collaboration

**Reference:** MDPI Drones 9(6):442 (2025)  
**URL:** https://www.mdpi.com/2504-446X/9/6/442

### What it says

- Combines ground-based reference station with aerial drones for GPS-denied navigation
- The ground station provides a stable coordinate anchor; drones triangulate relative to it
- Achieves 0.1–0.2 m positional accuracy without GPS

### Relevance to our setup

Our "master drone" acts as the ground reference — it has the fixed `origin_state.json`
and broadcasts the authoritative field coordinate system via UDP telemetry.

**For Robofest 6.0:** Consider whether the master drone should hover stationary over the
origin point for the entire mission (acting purely as coordinator / reference station)
rather than also scanning. This would improve overall accuracy at the cost of one scanner.

---

## Coordinate System — Implementation Details

Our coordinate system is defined in `04_coordinate_math.py` and `fieldmap.py`:

```
Origin:   Start GPS fix (lat0, lon0, alt0) saved in origin_state.json
+X axis:  East  (compass 90°)
+Y axis:  North (compass 0°)
+Z axis:  Up

All mine positions have NEGATIVE Y (they are South of the start line).
Field runs: X = [-2, 32] m,  Y = [-60, 2] m
```

The conversion math (GPS → local metres):
```python
# From 04_coordinate_math.py
METERS_PER_DEG_LAT = 111_320.0
METERS_PER_DEG_LON = 111_320.0 * cos(lat0_rad)

dx = (lon - lon0) * METERS_PER_DEG_LON   # East
dy = (lat - lat0) * METERS_PER_DEG_LAT   # North
```

Camera intrinsics used for pixel → physical offset:
```
fx = 614.5  (55° horizontal FOV at 640 px wide)
fy = 761.2  (35° vertical FOV at 480 px tall)
```

These intrinsics match the 55°×35° FOV MLX90640 lens. If you change lens, recalculate:
```
fx = (width_px / 2) / tan(hfov_rad / 2)
```

---

## Drift Mitigation Techniques Used

| Technique | Where in code | Benefit |
|-----------|--------------|---------|
| Compass averaging (5 samples, circular mean) | `00_preflight_calib.py` | Accurate initial heading |
| Passive grid from real GPS (not planned path) | `grid_map.mark_position()` | Drift-tolerant coverage map |
| Fuzzy footprint disc (r=0.78m) | `grid_map.py` | GPS noise fills coverage gaps |
| Persistence gate (re-hover + 12 frames) | `03_persistence.py` | Rejects false positives from drift |
| Mine deduplication radius 1.5 m | `05_map_verifier.py` | Fuses near-duplicate reports |

---

## For Robofest 6.0 — Recommended Improvements

1. **Optical flow sensor** (e.g. ArduPilot-compatible optical flow sensor (PX4Flow or Pi cam + Lucas-Kanade)) — can cut EKF drift
   from ~0.3 m/min to ~0.05 m/min over flat ground
2. **UWB anchor beacons** at field corners — provide cm-level indoor positioning with no GPS;
   check if PS rules permit placing anchors at field boundary
3. **Landmark detection** — detect Pole/Statue via camera and correct drone position mid-flight
4. **EKF re-anchor TCP command** — master can send corrected position to drifting slave
