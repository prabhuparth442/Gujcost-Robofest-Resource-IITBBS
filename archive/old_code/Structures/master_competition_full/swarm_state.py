"""
swarm_state.py  —  Single Source of Truth for the S.A.F.E. Swarm
=================================================================
Both udp_telemetry.py and tcp_commander.py import from this module.
Nothing else should hold its own copy of drone positions or mine data.

Thread / async safety
---------------------
All public helpers are either:
  (a) pure reads of immutable snapshots  — safe without locking, or
  (b) wrapped in  async with STATE.lock  — safe for concurrent coroutines.

Never await while holding the lock.  Grab the lock, mutate, release,
then do any slow work (network I/O, path-planning) outside it.

Collision avoidance
-------------------
Two drones are "too close" when their horizontal separation < SAFE_SEP_M.
_find_safe_waypoint() nudges a proposed waypoint until it is clear of
every other drone's current position AND every known mine.
The TCP commander calls this before sending any GOTO command.
"""

import asyncio
import math
import time
from dataclasses import dataclass, field
from typing import Optional


# ════════════════════════════════════════════════════════════════════════════
#  CONSTANTS
# ════════════════════════════════════════════════════════════════════════════
SAFE_SEP_M   = 3.0      # minimum inter-drone horizontal separation (metres)
MINE_CLEAR_M = 1.5      # drone must stay this far from any mine centre
EARTH_R      = 6_378_137.0

# Drone IDs recognised by the system
DRONE_IDS = ("master", "slave_1", "slave_2", "slave_3")


# ════════════════════════════════════════════════════════════════════════════
#  DATA CLASSES
# ════════════════════════════════════════════════════════════════════════════
@dataclass
class DroneState:
    drone_id:   str
    lat:        float = 0.0
    lng:        float = 0.0
    alt:        float = 0.0          # metres AGL
    heading:    float = 0.0          # degrees 0-360
    speed:      float = 0.0          # m/s horizontal ground speed
    armed:      bool  = False
    airborne:   bool  = False
    last_seen:  float = field(default_factory=time.time)  # unix timestamp

    # Local X/Y relative to mission origin (set after origin is fixed)
    x: float = 0.0
    y: float = 0.0

    def age(self) -> float:
        """Seconds since last telemetry packet."""
        return time.time() - self.last_seen

    def is_stale(self, timeout: float = 2.0) -> bool:
        return self.age() > timeout


@dataclass
class MineRecord:
    mine_id:     int
    lat:         float
    lng:         float
    x:           float   # local metres
    y:           float
    detected_by: str
    confirmed_at: float = field(default_factory=time.time)


@dataclass
class MissionConfig:
    origin_lat:     Optional[float] = None
    origin_lng:     Optional[float] = None
    origin_heading: float           = 0.0
    field_width_m:  float           = 20.0
    field_length_m: float           = 100.0
    cruise_alt_m:   float           = 3.0
    started:        bool            = False
    ended:          bool            = False
    start_time:     Optional[float] = None

    def elapsed(self) -> Optional[float]:
        if self.start_time is None:
            return None
        return round(time.time() - self.start_time, 1)


# ════════════════════════════════════════════════════════════════════════════
#  SWARM STATE  — the one global object both modules share
# ════════════════════════════════════════════════════════════════════════════
class SwarmState:
    """
    Central, lock-protected state store.

    Usage pattern (always inside an async context):

        async with STATE.lock:
            STATE.drones["slave_1"].lat = new_lat
            snapshot = STATE.snapshot()   # take a cheap copy before releasing

        # do slow work with snapshot outside the lock
        path = plan_path(snapshot)
    """

    def __init__(self):
        self.lock    = asyncio.Lock()
        self.mission = MissionConfig()
        self.drones: dict[str, DroneState] = {
            did: DroneState(drone_id=did) for did in DRONE_IDS
        }
        self.mines: list[MineRecord] = []

        # Latest voice command queued by the ground server / phone
        self.pending_voice_cmd: Optional[str] = None

        # Per-slave command queues  { "slave_1": asyncio.Queue, … }
        # TCP commander drains these; telemetry receiver may also push
        self.cmd_queues: dict[str, asyncio.Queue] = {
            did: asyncio.Queue() for did in DRONE_IDS if did != "master"
        }

    # ── coordinate helpers ───────────────────────────────────────────────

    def gps_to_local(self, lat: float, lng: float) -> tuple[float, float]:
        """Convert absolute GPS → local (x, y) metres aligned to heading."""
        cfg = self.mission
        if cfg.origin_lat is None:
            return 0.0, 0.0
        d_lat  = math.radians(lat - cfg.origin_lat)
        d_lng  = math.radians(lng - cfg.origin_lng)
        y_raw  =  d_lat * EARTH_R
        x_raw  =  d_lng * EARTH_R * math.cos(math.radians(cfg.origin_lat))
        h      = math.radians(cfg.origin_heading)
        x_rot  =  x_raw * math.cos(h) - y_raw * math.sin(h)
        y_rot  =  x_raw * math.sin(h) + y_raw * math.cos(h)
        return round(x_rot, 3), round(y_rot, 3)

    def local_to_gps(self, x: float, y: float) -> tuple[float, float]:
        """Convert local (x, y) metres back to GPS lat/lng."""
        cfg = self.mission
        if cfg.origin_lat is None:
            return cfg.origin_lat or 0.0, cfg.origin_lng or 0.0
        h     = math.radians(cfg.origin_heading)
        # Inverse rotation
        x_raw =  x * math.cos(h) + y * math.sin(h)
        y_raw = -x * math.sin(h) + y * math.cos(h)
        lat   = cfg.origin_lat + math.degrees(y_raw / EARTH_R)
        lng   = cfg.origin_lng + math.degrees(
                    x_raw / (EARTH_R * math.cos(math.radians(cfg.origin_lat)))
                )
        return round(lat, 7), round(lng, 7)

    # ── collision / safety geometry ──────────────────────────────────────

    def _drone_positions_except(self, exclude_id: str) -> list[tuple[float, float]]:
        """Return (x, y) of all drones except the one being planned for."""
        return [
            (d.x, d.y)
            for did, d in self.drones.items()
            if did != exclude_id and d.airborne and not d.is_stale()
        ]

    def _mine_positions(self) -> list[tuple[float, float]]:
        return [(m.x, m.y) for m in self.mines]

    def is_position_safe(
        self,
        x: float, y: float,
        exclude_drone: str,
        drone_sep: float = SAFE_SEP_M,
        mine_clear: float = MINE_CLEAR_M,
    ) -> bool:
        """
        Return True if (x, y) is:
          • at least drone_sep metres from every other active drone, AND
          • at least mine_clear metres from every confirmed mine.
        Call this BEFORE sending any GOTO waypoint.
        """
        for dx, dy in self._drone_positions_except(exclude_drone):
            if math.hypot(x - dx, y - dy) < drone_sep:
                return False
        for mx, my in self._mine_positions():
            if math.hypot(x - mx, y - my) < mine_clear:
                return False
        return True

    def find_safe_waypoint(
        self,
        desired_x: float, desired_y: float,
        drone_id: str,
        search_radius: float = 6.0,
        step: float = 0.5,
    ) -> tuple[float, float] | None:
        """
        Try to find the closest safe position to (desired_x, desired_y)
        by spiralling outward in a grid search up to search_radius metres.
        Returns (x, y) or None if no safe point found in range.
        """
        # Check desired point first
        if self.is_position_safe(desired_x, desired_y, drone_id):
            return desired_x, desired_y

        # Spiral grid search
        r = step
        while r <= search_radius:
            candidates = []
            s = r
            while s >= -r:
                for t in [r, -r]:
                    candidates.append((desired_x + t, desired_y + s))
                    candidates.append((desired_x + s, desired_y + t))
                s -= step

            # Sort by distance to desired point
            candidates.sort(key=lambda p: math.hypot(p[0]-desired_x, p[1]-desired_y))
            for cx, cy in candidates:
                if self.is_position_safe(cx, cy, drone_id):
                    return cx, cy
            r += step

        return None   # no safe point in range — caller must decide

    # ── snapshot for UI / path planning (no lock needed — caller holds it) ─

    def snapshot(self) -> dict:
        """
        Return a plain-dict snapshot of current state suitable for JSON
        serialisation and passing to path-planners outside the lock.
        """
        return {
            "mission": {
                "started":  self.mission.started,
                "ended":    self.mission.ended,
                "elapsed":  self.mission.elapsed(),
                "origin_set": self.mission.origin_lat is not None,
            },
            "drones": {
                did: {
                    "lat":      d.lat,  "lng":    d.lng,
                    "x":        d.x,    "y":      d.y,
                    "alt":      d.alt,  "heading": d.heading,
                    "speed":    d.speed,
                    "armed":    d.armed, "airborne": d.airborne,
                    "stale":    d.is_stale(),
                    "age_ms":   round(d.age() * 1000),
                }
                for did, d in self.drones.items()
            },
            "mines": [
                {"id": m.mine_id, "lat": m.lat, "lng": m.lng,
                 "x": m.x, "y": m.y, "detected_by": m.detected_by}
                for m in self.mines
            ],
            "mine_count": len(self.mines),
        }

    # ── helpers called by UDP receiver ───────────────────────────────────

    async def update_drone(self, pkt: dict) -> None:
        """
        Ingest a telemetry packet from a slave (or master forwarding its own pos).
        Called by the UDP receiver coroutine.
        Lock is acquired here.
        """
        did = pkt.get("drone_id")
        if did not in self.drones:
            return

        async with self.lock:
            d          = self.drones[did]
            d.lat      = float(pkt.get("lat",      d.lat))
            d.lng      = float(pkt.get("lng",      d.lng))
            d.alt      = float(pkt.get("altitude", d.alt))
            d.heading  = float(pkt.get("heading",  d.heading))
            d.speed    = float(pkt.get("speed",    d.speed))
            d.armed    = bool(pkt.get("armed",     d.armed))
            d.airborne = bool(pkt.get("airborne",  d.airborne))
            d.last_seen = time.time()

            # Recompute local XY
            d.x, d.y = self.gps_to_local(d.lat, d.lng)

    async def add_mine(self, lat: float, lng: float, detected_by: str) -> MineRecord:
        """
        Register a newly confirmed mine. Returns the MineRecord.
        Called by the UDP receiver after persistence check.
        Lock is acquired here.
        """
        async with self.lock:
            x, y = self.gps_to_local(lat, lng)
            rec  = MineRecord(
                mine_id=len(self.mines),
                lat=lat, lng=lng, x=x, y=y,
                detected_by=detected_by,
            )
            self.mines.append(rec)
        return rec


# ════════════════════════════════════════════════════════════════════════════
#  MODULE-LEVEL SINGLETON  — import this everywhere
# ════════════════════════════════════════════════════════════════════════════
STATE = SwarmState()
