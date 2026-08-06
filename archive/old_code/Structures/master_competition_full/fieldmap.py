#!/usr/bin/env python3
"""
fieldmap.py  —  S.A.F.E. Competition Field Map
===============================================
Single source of truth for ALL known geometry in this year's minefield.

Three layers:
  1. BURIED_MINES   — confirmed underground positions from pre-survey GPS.
                      These are loaded into the path planner as 0.75m hazard
                      discs.  The thermal vision pipeline is NOT used for these —
                      their locations are already known.

  2. FORBIDDEN_ZONES — pole and statue.  Both are tall physical obstacles the
                        drone must never overfly.  Hard exclusion radius — any
                        waypoint inside is rejected outright, not just avoided.

  3. GRASS_PATCH     — soft exclusion band along the left edge of the field.
                        Drone stays East of GRASS_EAST_LIMIT_M at all times.

  4. FIELD_BOUNDS    — the full scannable rectangle in local (x, y) metres.
                        Origin (0, 0) = Start GPS.  +X = East, +Y = North.
                        NOTE: competition field is oriented so North is the
                        forward direction.  +Y increases as the drone advances.
                        The Y axis is *negative* for all mine positions because
                        they are South of the origin start point.

Coordinate system
-----------------
  Origin  : 23.0779530, 72.4953475   → (0.00 m,    0.00 m)
  All local coords below are in metres:
    +X = East,  −X = West
    +Y = North, −Y = South

Usage
-----
    from fieldmap import FIELD, is_waypoint_safe, clamp_to_field

    # Check before sending a GOTO:
    if not is_waypoint_safe(x, y):
        ...

    # Get all hazard circles for path planner:
    hazards = FIELD.all_hazards()
"""

import math
from dataclasses import dataclass, field
from typing import NamedTuple


# ══════════════════════════════════════════════════════════════════════════════
#  COORDINATE REFERENCE
# ══════════════════════════════════════════════════════════════════════════════

ORIGIN_LAT = 23.0779530
ORIGIN_LON = 72.4953475
EARTH_R    = 6_378_137.0


def gps_to_local(lat: float, lon: float) -> tuple[float, float]:
    """Convert GPS → local (x=East, y=North) metres from origin."""
    d_lat = math.radians(lat - ORIGIN_LAT)
    d_lon = math.radians(lon - ORIGIN_LON)
    y = d_lat * EARTH_R
    x = d_lon * EARTH_R * math.cos(math.radians(ORIGIN_LAT))
    return round(x, 3), round(y, 3)


def local_to_gps(x: float, y: float) -> tuple[float, float]:
    """Convert local (x=East, y=North) metres → GPS."""
    lat = ORIGIN_LAT + math.degrees(y / EARTH_R)
    lon = ORIGIN_LON + math.degrees(
        x / (EARTH_R * math.cos(math.radians(ORIGIN_LAT)))
    )
    return round(lat, 7), round(lon, 7)


# ══════════════════════════════════════════════════════════════════════════════
#  DATA TYPES
# ══════════════════════════════════════════════════════════════════════════════

class HazardCircle(NamedTuple):
    name:        str
    x:           float   # local metres
    y:           float
    radius_m:    float
    forbidden:   bool    # True = hard exclusion; False = path-planner avoidance


# ══════════════════════════════════════════════════════════════════════════════
#  FIELD GEOMETRY
# ══════════════════════════════════════════════════════════════════════════════

# Scannable field bounds (local metres).
# Y is negative because all objects are South of the start line.
FIELD_X_MIN = -2.0     # metres West of origin  (leave 2 m margin)
FIELD_X_MAX = 32.0     # metres East  (End Point is at +28.43 m; add margin)
FIELD_Y_MIN = -24.0    # metres South (furthest object at −19.23 m; add margin)
FIELD_Y_MAX =  2.0     # metres North of origin

# Grass patch: a rough/soft-ground area on the western edge.
# Drone swath must stay East of this boundary at all times.
# Adjust this value if the exact grass boundary is measured differently on the day.
GRASS_EAST_LIMIT_M = -1.0   # no drone centre-line goes West of x = −1.0 m


# ══════════════════════════════════════════════════════════════════════════════
#  KNOWN BURIED MINES  (pre-surveyed GPS → local coords)
# ══════════════════════════════════════════════════════════════════════════════
#  Hazard radius  = 0.75 m  (competition scoring disc + GPS error margin)
#  These are AVOIDANCE circles only — vision detection is NOT run over them.

MINE_AVOIDANCE_RADIUS_M = 0.75

_BURIED_MINES_GPS = [
    ("Mine_1", 23.0779203, 72.4954246),
    ("Mine_2", 23.0778746, 72.4954139),
    ("Mine_3", 23.0779049, 72.4954863),
    ("Mine_4", 23.0778450, 72.4955319),
    ("Mine_5", 23.0777803, 72.4955299),
]

BURIED_MINES: list[HazardCircle] = [
    HazardCircle(
        name=name,
        x=gps_to_local(lat, lon)[0],
        y=gps_to_local(lat, lon)[1],
        radius_m=MINE_AVOIDANCE_RADIUS_M,
        forbidden=False,   # path planner routes around, doesn't reject
    )
    for name, lat, lon in _BURIED_MINES_GPS
]


# ══════════════════════════════════════════════════════════════════════════════
#  FORBIDDEN ZONES  (tall physical obstacles — hard NO-FLY)
# ══════════════════════════════════════════════════════════════════════════════

POLE_GPS   = (23.0778851, 72.4954763)
STATUE_GPS = (23.0778228, 72.4956064)

# Forbidden buffer:
#   Pole   — thin vertical obstacle; 1.5 m keeps rotors well clear
#   Statue — wider base; 2.5 m keeps drone clear of the structure
POLE_FORBIDDEN_RADIUS_M   = 1.5
STATUE_FORBIDDEN_RADIUS_M = 2.5

FORBIDDEN_ZONES: list[HazardCircle] = [
    HazardCircle(
        name="Pole",
        x=gps_to_local(*POLE_GPS)[0],
        y=gps_to_local(*POLE_GPS)[1],
        radius_m=POLE_FORBIDDEN_RADIUS_M,
        forbidden=True,
    ),
    HazardCircle(
        name="Statue",
        x=gps_to_local(*STATUE_GPS)[0],
        y=gps_to_local(*STATUE_GPS)[1],
        radius_m=STATUE_FORBIDDEN_RADIUS_M,
        forbidden=True,
    ),
]


# ══════════════════════════════════════════════════════════════════════════════
#  SURFACE DISC TARGETS  (the composite discs placed above ground)
# ══════════════════════════════════════════════════════════════════════════════
#  These are what the thermal vision pipeline IS trying to find.
#  We don't know their exact positions in advance — the drone discovers them.
#  This section just records the expected thermal signature params for the
#  surface-disc vision filter (06_surface_filter.py).

SURFACE_DISC_HOT_MIN  =  3.0   # °C above local background — post-rain wet soil
SURFACE_DISC_HOT_MAX  = 40.0   # °C  (MDF can reach ~25+°C; HDPE ~9°C)
SURFACE_DISC_MIN_CONF =  0.45  # minimum blob confidence to report


# ══════════════════════════════════════════════════════════════════════════════
#  SWARM SCAN LANES  — multi-pass schedule
# ══════════════════════════════════════════════════════════════════════════════
#  3 slaves, 1.4 m inter-drone gap, scanning South from the start line.
#  Each slave is assigned a fixed X-offset within each pass; the pass number
#  shifts the whole formation East by 3×1.4 = 4.2 m per pass.
#
#  Swath per drone at 1.5 m altitude, 55° FOV ≈ 1.56 m.
#  With 1.4 m lane step there is ~0.16 m overlap between adjacent strips.
#
#  Field mine cluster spans x ≈ 6.8 → 18.9 m East.
#  5 passes × 4.2 m = 21 m → covers the full mine cluster plus margins.
#  Passes are ordered from West to East so the mine cluster is hit in passes 1–4.
#
#  Pass X ranges (approximate swath centres):
#    Pass 1:  5.0 / 6.4 / 7.8   — Mine_1(7.9), Mine_2(6.8)  ← left cluster
#    Pass 2:  9.2 / 10.6 / 12.0 — open ground, approach Pole(13.2)
#    Pass 3: 13.4 / 14.8 / 16.2 — Mine_3(14.2), Pole avoided
#    Pass 4: 17.6 / 19.0 / 20.4 — Mine_4(18.9), Mine_5(18.7)
#    Pass 5: 21.8 / 23.2 / 24.6 — far East, approach Statue(26.5)
#
#  Note: The A* planner handles all obstacle avoidance per-pass, so
#  passes 3–5 that approach the pole and statue are automatically routed around.

LANE_STEP_M   = 1.4    # inter-drone spacing (matches physical rig)

# ── Validated 4-pass lane plan ───────────────────────────────────────────────
# Mine cluster spans x = +6.8 to +18.9 m East (12.1 m total width).
# Pole at x=+13.19 m creates a forbidden band x ∈ [11.4, 15.0] m.
# This splits field into West corridor and East corridor.
# Pass 2 terminates West of pole; Pass 3 starts East of it.
#
# Swath per drone at 1.5 m alt, 55° FOV = ±0.78 m → 1.56 m coverage.
# 3 drones × 1.4 m step = 4.2 m combined per pass.
#
# All clearances verified (min = 0.19 m, Pass 2 slave_3 vs pole edge):
PASS_LANES: dict[int, dict[str, float]] = {
    1: {"slave_1":  5.0, "slave_2":  6.4, "slave_3":  7.8},  # covers x=[4.2, 8.6]
    2: {"slave_1":  9.2, "slave_2": 10.6, "slave_3": 11.2},  # covers x=[8.4,12.0], West of pole
    3: {"slave_1": 15.2, "slave_2": 16.6, "slave_3": 18.0},  # covers x=[14.4,18.8], East of pole
    4: {"slave_1": 19.4, "slave_2": 20.8, "slave_3": 22.0},  # covers x=[18.6,22.8], East flank
}
NUM_PASSES = len(PASS_LANES)

# Legacy helpers (backward compat)
PASS1_BASE_X = 5.0
_DRONE_OFFSETS: dict[str, float] = {
    "slave_1": 0.0,
    "slave_2": LANE_STEP_M,
    "slave_3": 2 * LANE_STEP_M,
}

# Default lane set (Pass 1) — orchestrator overrides via PASS_NUMBER env var
LANE_STARTS: dict[str, float] = PASS_LANES[1].copy()

SCAN_Y_START = -1.0    # begin 1 m South of origin (skip take-off wash)
SCAN_Y_END   = -22.0   # stop 2 m past the furthest object (Mine_5 at −19.23 m)
SCAN_STEP_M  =  0.5    # forward step per grid cell (matches orchestrator)


def lane_x_for_pass(drone_id: str, pass_number: int) -> float:
    """
    Return the X lane centre for a given drone and pass (1-indexed).

    Example:
        lane_x_for_pass("slave_2", 1)  →  6.4
        lane_x_for_pass("slave_2", 3)  →  16.6
    """
    if pass_number not in PASS_LANES:
        raise ValueError(f"pass_number must be 1–{NUM_PASSES}, got {pass_number}")
    lanes = PASS_LANES[pass_number]
    if drone_id not in lanes:
        raise ValueError(f"Unknown drone_id '{drone_id}'")
    return lanes[drone_id]


# ══════════════════════════════════════════════════════════════════════════════
#  COMPOSITE FIELD CLASS
# ══════════════════════════════════════════════════════════════════════════════

class FieldMap:
    """Convenience wrapper — exposes all hazard geometry in one object."""

    def __init__(self):
        self.buried_mines    = BURIED_MINES
        self.forbidden_zones = FORBIDDEN_ZONES

    def all_hazards(self) -> list[HazardCircle]:
        """Return every hazard circle: buried mines + forbidden zones."""
        return list(self.buried_mines) + list(self.forbidden_zones)

    def is_forbidden(self, x: float, y: float) -> bool:
        """True if (x, y) is inside any HARD forbidden zone (pole / statue)."""
        for fz in self.forbidden_zones:
            if math.hypot(x - fz.x, y - fz.y) < fz.radius_m:
                return True
        return False

    def is_mine_hazard(self, x: float, y: float) -> bool:
        """True if (x, y) is inside any buried mine avoidance disc."""
        for m in self.buried_mines:
            if math.hypot(x - m.x, y - m.y) < m.radius_m:
                return True
        return False

    def is_safe(self, x: float, y: float, drone_margin: float = 0.3) -> bool:
        """
        True if (x, y) is a safe waypoint:
          • outside all forbidden zones (pole, statue)
          • outside all mine hazard discs
          • East of the grass patch limit
          • inside field bounds
        drone_margin: extra buffer added to all radii for the drone body.
        """
        if x < GRASS_EAST_LIMIT_M:
            return False
        if not (FIELD_X_MIN <= x <= FIELD_X_MAX and
                FIELD_Y_MIN <= y <= FIELD_Y_MAX):
            return False
        for hz in self.all_hazards():
            if math.hypot(x - hz.x, y - hz.y) < hz.radius_m + drone_margin:
                return False
        return True

    def nearest_hazard_dist(self, x: float, y: float) -> float:
        """Minimum distance to any hazard edge (negative if inside a hazard)."""
        min_dist = float("inf")
        for hz in self.all_hazards():
            d = math.hypot(x - hz.x, y - hz.y) - hz.radius_m
            min_dist = min(min_dist, d)
        return min_dist

    def buried_mine_gps_list(self) -> list[tuple[float, float]]:
        """Return list of (lat, lon) for all buried mines — for A4_map.json."""
        return [local_to_gps(m.x, m.y) for m in self.buried_mines]

    def print_summary(self):
        print("=" * 56)
        print("  S.A.F.E. FIELD MAP  —  Competition layout")
        print("=" * 56)
        print(f"  Origin : {ORIGIN_LAT}, {ORIGIN_LON}")
        print(f"  Bounds : X [{FIELD_X_MIN}, {FIELD_X_MAX}] m  "
              f"Y [{FIELD_Y_MIN}, {FIELD_Y_MAX}] m")
        print(f"  Grass  : no drone West of x = {GRASS_EAST_LIMIT_M} m")
        print()
        print("  Buried mines (pre-known, avoidance only):")
        for m in self.buried_mines:
            lat, lon = local_to_gps(m.x, m.y)
            print(f"    {m.name:8s}  x={m.x:+7.2f}  y={m.y:+7.2f}  "
                  f"r={m.radius_m:.2f}m  GPS=({lat},{lon})")
        print()
        print("  Forbidden zones (hard no-fly):")
        for fz in self.forbidden_zones:
            lat, lon = local_to_gps(fz.x, fz.y)
            print(f"    {fz.name:8s}  x={fz.x:+7.2f}  y={fz.y:+7.2f}  "
                  f"r={fz.radius_m:.2f}m  GPS=({lat},{lon})")
        print("=" * 56)


# ══════════════════════════════════════════════════════════════════════════════
#  MODULE-LEVEL SINGLETON
# ══════════════════════════════════════════════════════════════════════════════

FIELD = FieldMap()


# ══════════════════════════════════════════════════════════════════════════════
#  CONVENIENCE FUNCTIONS  (for import)
# ══════════════════════════════════════════════════════════════════════════════

def is_waypoint_safe(x: float, y: float, drone_margin: float = 0.3) -> bool:
    return FIELD.is_safe(x, y, drone_margin)


def clamp_to_field(x: float, y: float) -> tuple[float, float]:
    """Hard-clamp a position to field bounds (does not avoid hazards)."""
    x = max(FIELD_X_MIN, min(FIELD_X_MAX, x))
    y = max(FIELD_Y_MIN, min(FIELD_Y_MAX, y))
    return x, y


def generate_a4_map_json() -> dict:
    """
    Generate the A4_map.json content that 05_map_verifier.py uses.
    Call this once at startup to pre-populate the known-mines config file.
    """
    mines = []
    for i, m in enumerate(BURIED_MINES):
        lat, lon = local_to_gps(m.x, m.y)
        mines.append({
            "id":     m.name,
            "lat":    lat,
            "lon":    lon,
            "x_m":   m.x,
            "y_m":   m.y,
            "source": "pre_survey_gps",
        })
    return {"mines": mines, "origin": {"lat": ORIGIN_LAT, "lon": ORIGIN_LON}}


# ══════════════════════════════════════════════════════════════════════════════
#  SELF-TEST
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    FIELD.print_summary()

    print("\nSafety checks:")
    test_pts = [
        ("Origin",           0.0,   0.0),
        ("Mine_1 centre",    7.9,  -3.64),
        ("Pole centre",     FORBIDDEN_ZONES[0].x, FORBIDDEN_ZONES[0].y),
        ("Statue centre",   FORBIDDEN_ZONES[1].x, FORBIDDEN_ZONES[1].y),
        ("Grass edge",      -1.5, -10.0),
        ("Clear area",      10.0, -15.0),
    ]
    for label, x, y in test_pts:
        safe = is_waypoint_safe(x, y)
        print(f"  {label:20s}  ({x:+6.2f}, {y:+6.2f})  →  "
              f"{'SAFE    ' if safe else 'BLOCKED '}")

    import json
    print("\nA4 map JSON:")
    print(json.dumps(generate_a4_map_json(), indent=2))
