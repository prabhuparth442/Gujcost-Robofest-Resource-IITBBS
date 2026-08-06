# Field Layout — Robofest Gujarat 6.0

## Coordinate system

All code uses a **local Cartesian system** anchored at the drone start position:

```
Origin (0, 0) = Start GPS = 23.0779530°N, 72.4953475°E

+X = East   (metres)
−X = West
+Y = North  (metres)
−Y = South  ← this is the scan direction (drones fly SOUTH into the field)
```

> **Why −Y for the field?** The minefield is South of the start line, so all mine positions have negative Y values. The drones fly from (x, 0) toward (x, −22) during a scan pass.

---

## Field bounds (in local metres)

| Boundary | Value | Notes |
|----------|-------|-------|
| X min | −2.0 m | 2 m West buffer |
| X max | +32.0 m | field extends to ~28.4 m East |
| Y min | −24.0 m | 2 m past furthest mine (−19.23 m) |
| Y max | +2.0 m | 2 m North of origin |
| Grass West limit | −1.0 m | no drone centre-line goes West of this |

---

## Known mine positions (pre-surveyed)

These are **pre-known buried mines** loaded at startup. The path planner routes around them. The thermal pipeline is NOT run for these (their positions are already known and fed into the A* planner as avoidance circles).

| Mine | GPS (lat, lon) | Local (x, y) | Avoidance radius |
|------|---------------|-------------|----------------|
| Mine_1 | 23.0779203, 72.4954246 | +7.9 m, −3.6 m | 0.75 m |
| Mine_2 | 23.0778746, 72.4954139 | +6.8 m, −8.6 m | 0.75 m |
| Mine_3 | 23.0779049, 72.4954863 | +14.2 m, −5.3 m | 0.75 m |
| Mine_4 | 23.0778450, 72.4955319 | +18.9 m, −15.5 m | 0.75 m |
| Mine_5 | 23.0777803, 72.4955299 | +18.7 m, −19.2 m | 0.75 m |

> Mine cluster spans approximately **x = +6.8 to +18.9 m** East, **y = −3.6 to −19.2 m** South.

---

## Forbidden zones (hard no-fly obstacles)

These are tall physical obstacles that drones must **never overfly**. Any A* waypoint inside a forbidden zone is rejected outright.

| Object | GPS | Local (x, y) | Forbidden radius |
|--------|-----|-------------|----------------|
| Pole | 23.0778851, 72.4954763 | +13.2 m, −7.0 m | 1.5 m |
| Statue | 23.0778228, 72.4956064 | +26.5 m, −16.4 m | 2.5 m |

The Pole at x ≈ 13.2 m **splits the field into West and East corridors**:
- Pass 2 terminates at x ≈ 11.2 m (West of Pole)
- Pass 3 begins at x ≈ 15.2 m (East of Pole)

---

## Scan lane layout (4 passes, 3 drones)

Each pass, all 3 slaves fly their assigned X lane simultaneously, scanning South:

```
West ←──────────────────────────────────────────────────────→ East
        Pass 1      Pass 2      [POLE]    Pass 3      Pass 4
       ─────────  ──────────  ─── × ───  ──────────  ──────────
s1 →    5.0 m      9.2 m               15.2 m      19.4 m
s2 →    6.4 m     10.6 m               16.6 m      20.8 m
s3 →    7.8 m     11.2 m               18.0 m      22.0 m
       ─────────  ──────────           ──────────  ──────────
         Covers:  Covers:               Covers:     Covers:
       Mine1,2   open ground           Mine3        Mine4,5
                  approach Pole        E of Pole    E flank
```

**Lane step:** 1.4 m (inter-drone spacing). Chosen to match the MLX90640 ground footprint at 1.5 m altitude with a 55° lens (footprint radius ≈ 0.78 m → 1.56 m diameter → 0.16 m overlap between adjacent lanes).

**Scan step:** 0.5 m forward per hover cell. Drone pauses 0.6 s at each cell to let the sensor integrate before capturing 10 frames.

---

## Scan parameters

| Parameter | Value | Reason |
|-----------|-------|--------|
| Cruise altitude | 1.5 m | Optimal for MLX90640 55° FOV and mine size |
| Step size | 0.5 m | Half-cell = guaranteed overlap |
| Step hover | 0.6 s | Sensor settle time |
| Rolling window | 10 frames | Averaged stack for detection |
| Persistence window | 12 frames | Re-capture for confirmation |
| Confidence threshold | 0.45 | Empirically tuned; below this = ghost |
| Lidar obstacle threshold | 1.0 m | TF-Luna: sidestep if obstacle closer |

---

## Thermal detection bands

Two filters run on every captured frame stack:

| Filter | File | Target | ΔT band | Why different? |
|--------|------|--------|---------|---------------|
| Buried mine | `02_vision_filter.py` | Shallow mines (2–12 cm depth) | +0.15 to +1.25°C (hot) and −0.50 to −0.08°C (cold) | Thermal wave attenuated by soil; very small signature |
| Surface disc | `06_surface_filter.py` | On-ground plastic/MDF discs | +3 to +40°C | Direct solar heating; strong signal |

The **two-band buried mine filter** handles time-of-day variation:
- Morning (10:00–11:00 IST): deeper mines are thermally COLD relative to surface
- Afternoon (12:00–14:00 IST): shallow mines are thermally HOT

---

## GPS ↔ local coordinate conversions

Used in `fieldmap.py` and `04_coordinate_math.py`:

```python
# GPS → local (x=East metres, y=North metres from origin)
def gps_to_local(lat, lon):
    d_lat = radians(lat - ORIGIN_LAT)
    d_lon = radians(lon - ORIGIN_LON)
    y = d_lat * EARTH_R                              # North
    x = d_lon * EARTH_R * cos(radians(ORIGIN_LAT))  # East
    return x, y

# Local (x, y) → GPS
def local_to_gps(x, y):
    lat = ORIGIN_LAT + degrees(y / EARTH_R)
    lon = ORIGIN_LON + degrees(x / (EARTH_R * cos(radians(ORIGIN_LAT))))
    return lat, lon
```

> **Remember:** +Y = North (toward origin), −Y = South (into the field). Scan direction is −Y.

---

## Updating mine positions for a new venue

Edit `fieldmap.py`:

1. Update `ORIGIN_LAT` / `ORIGIN_LON` to the new start GPS fix
2. Update `_BURIED_MINES_GPS` with pre-surveyed mine GPS coordinates
3. Update `POLE_GPS` / `STATUE_GPS` with actual obstacle positions
4. Verify `PASS_LANES` — adjust if the mine cluster is at different X ranges
5. Run `python3 fieldmap.py` to print the field summary and verify safety checks

The A* planner in both `main_orchestrator_competition.py` and `app.py` will automatically pick up the new geometry.
