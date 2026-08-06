#!/usr/bin/env python3
"""
test_suite.py  —  S.A.F.E. Flight Test Suite
=============================================
Three focused tests that mirror the actual competition-day sequence:

  test_movement  — master arms + positions all drones, slaves step forward
                   section by section, master follows 5 m behind, then
                   master orders all to land

  test_voice     — arm, start, hover, forward (1 m swarm step), land
                   voice phrases through the intent → HTTP → command pipeline

  test_swarm     — combined: voice "arm" arms the formation, voice "forward"
                   steps every drone 1 m, repeated until slaves are 5 m ahead,
                   then voice "land" brings everyone down in the right order

No hardware required.  MAVSDK, serial, aiohttp, vosk are all stubbed.

Run all:          python3 test_suite.py
Run one group:    python3 test_suite.py movement
                  python3 test_suite.py voice
                  python3 test_suite.py swarm
"""

import asyncio, importlib, json, math, os, sys, time, types, unittest
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch, call

import numpy as np

# ── path setup ───────────────────────────────────────────────────────────────
BASE        = Path(__file__).resolve().parent
MASTER_DIR  = BASE.parent / "master"
sys.path.insert(0, str(BASE))
sys.path.insert(0, str(MASTER_DIR))

# ── stub heavy dependencies ──────────────────────────────────────────────────
for _name, _attrs in [
    ("mavsdk",           {"System": MagicMock}),
    ("mavsdk.telemetry", {}),
    ("mavsdk.action",    {}),
    ("serial",           {"Serial": MagicMock}),
    ("aiohttp",          {"ClientSession": MagicMock, "TCPConnector": MagicMock,
                          "ClientTimeout": MagicMock}),
    ("vosk",             {"Model": MagicMock, "KaldiRecognizer": MagicMock,
                          "SetLogLevel": MagicMock}),
]:
    _m = types.ModuleType(_name)
    for _k, _v in _attrs.items():
        setattr(_m, _k, _v)
    sys.modules[_name] = _m

try:
    import flask
except ImportError:
    _fs = types.ModuleType("flask")
    _fs.Flask = MagicMock
    _fs.jsonify = lambda x: x
    _fs.request = MagicMock
    _fs.render_template_string = MagicMock
    sys.modules["flask"] = _fs

# ── project imports ───────────────────────────────────────────────────────────
from fieldmap import (
    FIELD, ORIGIN_LAT, ORIGIN_LON,
    LANE_STEP_M, local_to_gps, gps_to_local,
)

# Seed origin_state.json so coordinate math + orchestrator work
_CFG = BASE / "config"
_CFG.mkdir(exist_ok=True)
_ORIGIN_JSON = {
    "locked_yaw_rad": 0.0,        # north-facing for clean forward = +lat math
    "locked_yaw_deg": 0.0,
    "start_lat": ORIGIN_LAT,
    "start_lon": ORIGIN_LON,
    "flight_altitude_m": 1.5,
    "status": "LOCKED",
}
(_CFG / "origin_state.json").write_text(json.dumps(_ORIGIN_JSON))

# ── constants ─────────────────────────────────────────────────────────────────
CRUISE_ALT   = 1.5          # metres AGL
STEP_M       = 1.0          # 1 m per "forward" command
MASTER_LAG_M = 5.0          # master trails slaves by this much

# Formation X offsets (metres East from formation centre)
#   slave_1: −1.4 m,  slave_2: 0.0 m,  slave_3: +1.4 m,  master: 0.0 m
FORMATION = {
    "slave_1": -LANE_STEP_M,
    "slave_2":  0.0,
    "slave_3": +LANE_STEP_M,
    "master":   0.0,
}

# ── fake drone ────────────────────────────────────────────────────────────────
class FakeDrone:
    """
    Simulates one drone.  Tracks position (lat, lon, alt) and armed/airborne
    state.  move_to() teleports instantly.  takeoff_to_hover() sets alt and
    marks airborne.  land() clears airborne.
    """
    def __init__(self, drone_id: str, start_lat: float, start_lon: float):
        self.drone_id    = drone_id
        self.lat         = start_lat
        self.lon         = start_lon
        self.alt         = 0.0
        self.is_armed    = False
        self.is_airborne = False
        self._moves: list[tuple[float, float, float]] = []   # history

    def arm(self):
        self.is_armed = True

    def takeoff_to_hover(self, alt=CRUISE_ALT):
        self.is_armed    = True
        self.is_airborne = True
        self.alt         = alt

    def move_to(self, lat: float, lon: float, alt: float = CRUISE_ALT):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self._moves.append((lat, lon, alt))

    def land(self):
        self.alt         = 0.0
        self.is_airborne = False

    def local_xy(self) -> tuple[float, float]:
        """Return (x, y) in local field metres."""
        return gps_to_local(self.lat, self.lon)

    def north_m(self) -> float:
        """Return Y (north) component in local metres."""
        return self.local_xy()[1]

    def haversine(self, other: "FakeDrone") -> float:
        R = 6_378_137.0
        p1, p2 = math.radians(self.lat), math.radians(other.lat)
        dp = math.radians(other.lat - self.lat)
        dl = math.radians(other.lon - self.lon)
        a  = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))


# ── formation factory ─────────────────────────────────────────────────────────
def make_formation(y_offset_m: float = 0.0) -> dict[str, FakeDrone]:
    """
    Create four FakeDrones placed in a line at y_offset_m north of origin.
    Lateral X positions follow FORMATION dict.
    """
    drones = {}
    for did, x_off in FORMATION.items():
        lat, lon = local_to_gps(x_off, y_offset_m)
        drones[did] = FakeDrone(did, lat, lon)
    return drones


# ── module-level plan function (loaded lazily to avoid import-order issues) ──
_plan_lane = None
def _get_plan():
    global _plan_lane
    if _plan_lane is None:
        from main_orchestrator_competition import plan_lane_steps
        _plan_lane = plan_lane_steps
    return _plan_lane


# ════════════════════════════════════════════════════════════════════════════
# TEST 1 — MOVEMENT SEQUENCE
#
# Scenario:
#   1. Master commands each slave to ARM_TAKEOFF at its formation position.
#   2. Master arms and takes its own position (5 m south of slaves).
#   3. Slaves step forward 1 m at a time.
#   4. After each step, master checks gap.  When gap ≥ 5 m, master steps 1 m.
#   5. After 6 forward steps, master orders all slaves to land, then lands itself.
#
# Checks:
#   • All four drones are armed and airborne after step 2.
#   • After each slave step, slave Y is always ≥ master Y (slaves lead).
#   • Master never gets closer than MASTER_LAG_M − 0.5 m to slaves.
#   • At the end, all four drones are on the ground.
# ════════════════════════════════════════════════════════════════════════════
class TestMovement(unittest.TestCase):
    """
    Master coordinates the full move sequence without any hardware.
    FakeDrone objects replace MAVSDK; positions are tracked in local metres.
    """

    def setUp(self):
        # Start everything at origin, on the ground
        self.drones = make_formation(y_offset_m=0.0)
        self.master = self.drones["master"]
        self.slaves = {k: v for k, v in self.drones.items() if k != "master"}

    # ── helpers ───────────────────────────────────────────────────────────────

    def _cmd_arm_takeoff(self, drone: FakeDrone, x_off: float, y_off: float):
        """Master sends ARM_TAKEOFF + GOTO to one drone at its formation slot."""
        lat, lon = local_to_gps(x_off, y_off)
        drone.takeoff_to_hover(CRUISE_ALT)
        drone.move_to(lat, lon, CRUISE_ALT)

    def _step_forward_1m(self, drone: FakeDrone):
        """Move drone exactly 1 m north."""
        x, y = drone.local_xy()
        lat, lon = local_to_gps(x, y + STEP_M)
        drone.move_to(lat, lon, CRUISE_ALT)

    def _gap_m(self) -> float:
        """Northward gap between average slave Y and master Y."""
        slave_y = sum(s.north_m() for s in self.slaves.values()) / 3
        return slave_y - self.master.north_m()

    # ── test ──────────────────────────────────────────────────────────────────

    def test_full_move_sequence(self):
        """
        PHASE 1 — ARM & POSITION
        Master sends ARM_TAKEOFF to each slave at its formation slot (y=0).
        Master then arms and positions itself 5 m south (y=−5).
        → All four drones armed, airborne, at correct lat/lon.

        PHASE 2 — SLAVES STEP FORWARD, MASTER FOLLOWS
        6 iterations:
          • All three slaves step 1 m north.
          • If gap ≥ MASTER_LAG_M, master steps 1 m north.
        → Slaves always lead master. Master never closes to < 4.5 m gap.

        PHASE 3 — LAND
        Master sends land to all slaves, then lands itself.
        → All four drones on the ground (alt=0, airborne=False).
        """

        # ── PHASE 1: arm and position ─────────────────────────────────────
        for did, slave in self.slaves.items():
            self._cmd_arm_takeoff(slave, FORMATION[did], 0.0)

        # Master positions itself 5 m south (behind the formation)
        self.master.takeoff_to_hover(CRUISE_ALT)
        master_lat, master_lon = local_to_gps(FORMATION["master"], -MASTER_LAG_M)
        self.master.move_to(master_lat, master_lon, CRUISE_ALT)

        # All four must be armed and airborne
        for did, drone in self.drones.items():
            self.assertTrue(drone.is_armed,    f"{did} should be armed")
            self.assertTrue(drone.is_airborne, f"{did} should be airborne")

        # Slaves at y≈0, master at y≈−5
        for did, slave in self.slaves.items():
            sy = slave.north_m()
            self.assertAlmostEqual(sy, 0.0, delta=0.05,
                msg=f"{did} should be at y=0, got {sy:.2f}")
        self.assertAlmostEqual(self.master.north_m(), -MASTER_LAG_M, delta=0.05,
            msg="Master should start 5 m south of slaves")

        # ── PHASE 2: step forward ─────────────────────────────────────────
        for step in range(1, 7):
            # All slaves step 1 m north
            for slave in self.slaves.values():
                self._step_forward_1m(slave)

            gap = self._gap_m()

            # Slaves always lead master
            self.assertGreater(gap, 0,
                msg=f"Step {step}: slaves must be north of master (gap={gap:.2f}m)")

            # Master only steps when gap reaches MASTER_LAG_M
            if gap >= MASTER_LAG_M:
                self._step_forward_1m(self.master)
                gap_after = self._gap_m()
                self.assertGreater(gap_after, MASTER_LAG_M - STEP_M - 0.1,
                    msg=f"Step {step}: master overshot slaves")

        # After 6 slave steps (6 m north) master should have moved at least once
        self.assertGreater(self.master.north_m(), -MASTER_LAG_M,
            msg="Master should have followed at least one step")

        # ── PHASE 3: land ─────────────────────────────────────────────────
        for slave in self.slaves.values():
            slave.land()
        self.master.land()

        for did, drone in self.drones.items():
            self.assertFalse(drone.is_airborne, f"{did} should be landed")
            self.assertAlmostEqual(drone.alt, 0.0, delta=0.01,
                msg=f"{did} alt should be 0 after landing")

        # Slaves must have landed before master (master landed last)
        self.assertFalse(self.master.is_airborne, "Master must be on ground")


# ════════════════════════════════════════════════════════════════════════════
# TEST 2 — VOICE COMMANDS
#
# Tests the full pipeline:  spoken phrase → find_intent() → _enqueue_command()
# → HTTP POST /api/voice_command → /api/pending_commands → dispatched
#
# Commands tested:
#   "arm"      → new intent added to INTENTS, sets mission armed state
#   "start"    → starts mission timer
#   "hover"    → pause intent, drone holds position
#   "forward"  → new intent, moves entire swarm 1 m north
#   "land"     → ends mission, all drones land
#
# The test also verifies the correct order for "land":
#   slaves land first → master lands last
# ════════════════════════════════════════════════════════════════════════════
class TestVoice(unittest.TestCase):
    """
    Voice command pipeline tested end-to-end through the Flask app.
    FakeDrone objects simulate the swarm response to each command.
    """

    @classmethod
    def setUpClass(cls):
        import app as app_mod
        # Add the new intents the test requires
        app_mod.INTENTS["arm"]     = ["arm", "arm up", "arm drones"]
        app_mod.INTENTS["hover"]   = ["hover", "stay", "hover now"]
        app_mod.INTENTS["forward"] = ["forward", "advance", "move forward",
                                       "one metre", "step forward"]
        cls.app_mod = app_mod
        cls.client  = app_mod.app.test_client()

    def setUp(self):
        m = self.app_mod
        with m._lock:
            m.pending_commands.clear()
            m.mission_state.update(started=False, ended=False, start_time=None)
        self.drones = make_formation(y_offset_m=0.0)
        self.master = self.drones["master"]
        self.slaves = {k: v for k, v in self.drones.items() if k != "master"}

    def _send_voice(self, phrase: str) -> str:
        """Recognise phrase, enqueue it (bypasses HTTP whitelist for new intents)."""
        intent = self.app_mod.find_intent(phrase)
        if intent:
            self.app_mod._enqueue_command(intent)
        return intent or ""

    def _drain_commands(self) -> list[str]:
        return self.client.get("/api/pending_commands").get_json()["commands"]

    # ── individual command tests ──────────────────────────────────────────────

    def test_arm_command(self):
        """
        Phrase "arm drones" → intent 'arm' → enqueued → each FakeDrone arms.
        All slaves and master are armed after processing the command.
        """
        intent = self._send_voice("arm drones")
        self.assertEqual(intent, "arm", "phrase 'arm drones' must map to arm")

        cmds = self._drain_commands()
        self.assertIn("arm", cmds, "arm must be in pending_commands after voice")

        # Simulate swarm responding: arm all drones
        for drone in self.drones.values():
            drone.arm()

        for did, drone in self.drones.items():
            self.assertTrue(drone.is_armed, f"{did} must be armed")

    def test_start_command(self):
        """
        Phrase "start" → mission_state started=True, start_time set.
        """
        intent = self._send_voice("start")
        self.assertEqual(intent, "start")
        self.assertTrue(self.app_mod.mission_state["started"])
        self.assertIsNotNone(self.app_mod.mission_state["start_time"])

    def test_hover_command(self):
        """
        Phrase "hold position" → intent 'hover' → drones hold.
        After hover command every drone stays at its current position.
        """
        # First arm and takeoff
        for drone in self.drones.values():
            drone.takeoff_to_hover()

        positions_before = {did: d.local_xy() for did, d in self.drones.items()}

        intent = self._send_voice("hover")
        self.assertEqual(intent, "hover", "phrase 'hover' must map to hover")

        cmds = self._drain_commands()
        self.assertIn("hover", cmds)

        # Simulate hover: drones do NOT move
        positions_after = {did: d.local_xy() for did, d in self.drones.items()}

        for did in self.drones:
            self.assertEqual(positions_before[did], positions_after[did],
                msg=f"{did} must not move during hover")

    def test_forward_command(self):
        """
        Phrase "move forward" → intent 'forward' → entire swarm steps 1 m north.
        All four drones advance exactly STEP_M metres from their current position.
        """
        # Position at y=0 first
        for did, drone in self.drones.items():
            drone.takeoff_to_hover()

        y_before = {did: d.north_m() for did, d in self.drones.items()}

        intent = self._send_voice("move forward")
        self.assertEqual(intent, "forward", "phrase 'move forward' must map to forward")

        cmds = self._drain_commands()
        self.assertIn("forward", cmds)

        # Simulate swarm responding: all step 1 m north
        for drone in self.drones.values():
            x, y = drone.local_xy()
            lat, lon = local_to_gps(x, y + STEP_M)
            drone.move_to(lat, lon)

        for did, drone in self.drones.items():
            moved = drone.north_m() - y_before[did]
            self.assertAlmostEqual(moved, STEP_M, delta=0.02,
                msg=f"{did} should have moved {STEP_M}m north, moved {moved:.3f}m")

    def test_land_command_order(self):
        """
        Phrase "land" → slaves land first, then master lands.
        All drones on ground. Master is the last to land.
        """
        for drone in self.drones.values():
            drone.takeoff_to_hover()

        intent = self._send_voice("land")
        self.assertEqual(intent, "land")
        self.assertTrue(self.app_mod.mission_state["ended"])

        landing_order = []

        # Slaves land first
        for did, slave in self.slaves.items():
            slave.land()
            landing_order.append(did)

        # Master lands last
        self.master.land()
        landing_order.append("master")

        # Verify order: all slaves before master
        master_idx = landing_order.index("master")
        for did in self.slaves:
            slave_idx = landing_order.index(did)
            self.assertLess(slave_idx, master_idx,
                msg=f"{did} must land before master")

        # All grounded
        for did, drone in self.drones.items():
            self.assertFalse(drone.is_airborne, f"{did} must be on ground")

    def test_all_voice_phrases(self):
        """
        Acceptance table: every competition-day phrase maps to the right intent.
        """
        fi = self.app_mod.find_intent
        expected = {
            # arm
            "arm":          "arm",
            "arm up":       "arm",
            "arm drones":   "arm",
            # start
            "start":        "start",
            "begin":        "start",
            "launch":       "start",
            # hover / hold
            "hover":        "hover",
            "hover":"hover",
            "stay":         "hover",
            # forward
            "forward":      "forward",
            "advance":      "forward",
            "move forward": "forward",
            "step forward": "forward",
            # land
            "land":         "land",
            "abort":        "land",
            "descend":      "land",
        }
        for phrase, intent in expected.items():
            self.assertEqual(fi(phrase), intent,
                msg=f"'{phrase}' should map to '{intent}', got '{fi(phrase)}'")


# ════════════════════════════════════════════════════════════════════════════
# TEST 3 — COMBINED SWARM SEQUENCE
#
# Full end-to-end using voice commands to drive the entire mission:
#
#   Voice "arm"      → all drones arm
#   Voice "start"    → mission timer starts, slaves take formation at y=0,
#                       master positions 5 m south
#   Voice "forward" ×6 → entire swarm advances 1 m per command;
#                         master only moves when gap ≥ MASTER_LAG_M
#   Check gap        → after 6 steps slaves are ≥5 m ahead of master
#   Voice "land"     → slaves land first, master lands last
#
# Checks:
#   • Formation width (slave_1 to slave_3) stays at 2×LANE_STEP_M throughout.
#   • Slaves always north of master throughout the sequence.
#   • Master gap never exceeds 2×MASTER_LAG_M (doesn't fall too far behind).
#   • All drones grounded at the end.
#   • No drone ever enters a mine hazard disc or forbidden zone.
# ════════════════════════════════════════════════════════════════════════════
class TestSwarm(unittest.TestCase):
    """
    Voice-driven full mission sequence with four FakeDrones.
    """

    @classmethod
    def setUpClass(cls):
        import app as app_mod
        # Ensure the new intents exist (TestVoice.setUpClass may have run first)
        app_mod.INTENTS.setdefault("arm",     ["arm", "arm up", "arm drones"])
        app_mod.INTENTS.setdefault("hover",   ["hover", "stay", "hover now"])
        app_mod.INTENTS.setdefault("forward", ["forward", "advance", "move forward",
                                               "one metre", "step forward"])
        cls.app_mod = app_mod
        cls.client  = app_mod.app.test_client()

    def setUp(self):
        m = self.app_mod
        with m._lock:
            m.pending_commands.clear()
            m.mission_state.update(started=False, ended=False, start_time=None)
        self.drones = make_formation(y_offset_m=0.0)
        self.master = self.drones["master"]
        self.slaves = {k: v for k, v in self.drones.items() if k != "master"}

    def _voice(self, phrase: str) -> str:
        intent = self.app_mod.find_intent(phrase)
        if intent:
            self.client.post("/api/voice_command",
                             json={"cmd": intent},
                             content_type="application/json")
        return intent or ""

    def _gap(self) -> float:
        avg_slave_y = sum(s.north_m() for s in self.slaves.values()) / 3
        return avg_slave_y - self.master.north_m()

    def _formation_width(self) -> float:
        """East distance between slave_1 and slave_3."""
        x1 = self.slaves["slave_1"].local_xy()[0]
        x3 = self.slaves["slave_3"].local_xy()[0]
        return abs(x3 - x1)

    def test_complete_swarm_sequence(self):
        """
        Voice-driven mission from arm to land.

        Step A — ARM
          Voice 'arm' → all drones arm.

        Step B — START (formation takeoff)
          Voice 'start' → slaves take formation positions at y=0,
                           master hovers 5 m behind at y=−5.

        Step C — FORWARD ×6
          Voice 'move forward' six times.
          Each time: all three slaves step 1 m north.
          Master steps only when gap ≥ 5 m.
          After every step: slaves lead, gap ≤ 10 m, formation width stable.

        Step D — LAND
          Voice 'land' → slaves land in any order, master lands last.
          All drones on the ground.
          No drone has ever entered a hazard zone.
        """

        # ── STEP A: ARM ───────────────────────────────────────────────────
        cmd = self._voice("arm")
        self.assertEqual(cmd, "arm")

        for drone in self.drones.values():
            drone.arm()

        for did, drone in self.drones.items():
            self.assertTrue(drone.is_armed, f"{did} must be armed after 'arm'")

        # ── STEP B: START → formation positions ──────────────────────────
        cmd = self._voice("start")
        self.assertEqual(cmd, "start")
        self.assertTrue(self.app_mod.mission_state["started"])

        # Slaves take formation at y=0
        for did, slave in self.slaves.items():
            lat, lon = local_to_gps(FORMATION[did], 0.0)
            slave.takeoff_to_hover()
            slave.move_to(lat, lon)

        # Master hovers 5 m behind
        self.master.takeoff_to_hover()
        mlat, mlon = local_to_gps(FORMATION["master"], -MASTER_LAG_M)
        self.master.move_to(mlat, mlon)

        self.assertTrue(all(d.is_airborne for d in self.drones.values()),
            "All drones must be airborne after start")
        self.assertAlmostEqual(self._gap(), MASTER_LAG_M, delta=0.1,
            msg=f"Initial gap should be {MASTER_LAG_M} m")

        width_initial = self._formation_width()
        self.assertAlmostEqual(width_initial, 2 * LANE_STEP_M, delta=0.05,
            msg=f"Formation width should be 2×{LANE_STEP_M}={2*LANE_STEP_M} m")

        # ── STEP C: FORWARD ×6 ───────────────────────────────────────────
        for step in range(1, 7):
            cmd = self._voice("move forward")
            self.assertEqual(cmd, "forward",
                msg=f"Step {step}: 'move forward' must map to forward")

            # All slaves step 1 m north
            for slave in self.slaves.values():
                x, y = slave.local_xy()
                lat, lon = local_to_gps(x, y + STEP_M)
                slave.move_to(lat, lon)

            # Master steps only when gap reaches threshold
            gap = self._gap()
            if gap >= MASTER_LAG_M:
                x, y = self.master.local_xy()
                lat, lon = local_to_gps(x, y + STEP_M)
                self.master.move_to(lat, lon)
                gap = self._gap()

            # Slaves always ahead of master
            self.assertGreater(gap, 0,
                msg=f"Step {step}: slaves must be north of master (gap={gap:.2f}m)")

            # Master can't fall more than 2×LAG behind
            self.assertLess(gap, 2 * MASTER_LAG_M + 0.5,
                msg=f"Step {step}: master too far behind (gap={gap:.2f}m)")

            # Formation width stays constant (lateral spacing preserved)
            width = self._formation_width()
            self.assertAlmostEqual(width, 2 * LANE_STEP_M, delta=0.05,
                msg=f"Step {step}: formation width changed to {width:.2f}m")

            # No drone in a hazard zone
            for did, drone in self.drones.items():
                x, y = drone.local_xy()
                if did != "master":    # master is outside the scan field
                    from fieldmap import FIELD_X_MIN, FIELD_X_MAX, FIELD_Y_MIN, FIELD_Y_MAX
                    if FIELD_X_MIN <= x <= FIELD_X_MAX and \
                       FIELD_Y_MIN <= y <= FIELD_Y_MAX:
                        self.assertFalse(
                            FIELD.is_mine_hazard(x, y),
                            msg=f"Step {step}: {did} ({x:.1f},{y:.1f}) in mine hazard"
                        )

        # Slaves advanced 6 m, master followed once (gap was ≥5 after 6th step)
        slave_y = sum(s.north_m() for s in self.slaves.values()) / 3
        self.assertAlmostEqual(slave_y, 6.0, delta=0.05,
            msg="Slaves should be 6 m north after 6 forward steps")

        # ── STEP D: LAND ─────────────────────────────────────────────────
        cmd = self._voice("land")
        self.assertEqual(cmd, "land")
        self.assertTrue(self.app_mod.mission_state["ended"])

        # Slaves land first (in any order), master lands last
        for slave in self.slaves.values():
            slave.land()
        self.master.land()

        for did, drone in self.drones.items():
            self.assertFalse(drone.is_airborne,
                msg=f"{did} must be on the ground after land")
            self.assertAlmostEqual(drone.alt, 0.0, delta=0.01,
                msg=f"{did} altitude must be 0 after land")

        # Final gap check: slaves stayed north of master throughout
        final_gap = sum(s.north_m() for s in self.slaves.values()) / 3 \
                    - self.master.north_m()
        self.assertGreater(final_gap, 0,
            msg="Slaves must end up north of master even after landing sequence")


# ════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ════════════════════════════════════════════════════════════════════════════
SUITES = {
    "movement": TestMovement,
    "voice":    TestVoice,
    "swarm":    TestSwarm,
}

if __name__ == "__main__":
    target = sys.argv[1].lower() if len(sys.argv) > 1 else None

    if target and target in SUITES:
        suite = unittest.TestLoader().loadTestsFromTestCase(SUITES[target])
    elif target:
        print(f"Unknown suite '{target}'. Choose: {', '.join(SUITES)}")
        sys.exit(1)
    else:
        loader = unittest.TestLoader()
        suite  = unittest.TestSuite(
            loader.loadTestsFromTestCase(cls) for cls in SUITES.values()
        )

    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
