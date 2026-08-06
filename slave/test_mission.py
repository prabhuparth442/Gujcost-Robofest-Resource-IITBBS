#!/usr/bin/env python3
"""
test_mission.py  —  S.A.F.E. Integrated Mission Test
=====================================================
Simulates a real 4-drone mission entirely in-process:

  Step 1  FORMATION    Master tells all 3 slaves: arm and take position
                       (slave_1 at x=-1.4m, slave_2 at x=0m, slave_3 at x=+1.4m)
                       Then master arms and takes its own position (1m behind slaves)

  Step 2  VOICE: arm   Master receives voice "arm" command →
                       broadcasts ARM_TAKEOFF to all slaves simultaneously

  Step 3  VOICE: start Mission starts. All drones confirm airborne.

  Step 4  VOICE: hover Swarm holds position.

  Step 5  ADVANCE      Slaves step forward 0.5m at a time.
                       Once slave lead is 5m ahead of master, master begins following.

  Step 6  VOICE: forward   Entire swarm moves 1m forward together.

  Step 7  VOICE: land  Master tells slaves to land first (in order),
                       then master lands itself.

No real hardware. Drones are in-process state machines.
TCP sockets are real loopback connections (127.0.0.1).
Run:  python3 test_mission.py
"""

import asyncio
import json
import math
import socket
import sys
import time
import types
from unittest.mock import MagicMock

# ── stub heavy deps before any project import ────────────────────────────────
for mod, attrs in [
    ("mavsdk",           {"System": MagicMock}),
    ("mavsdk.telemetry", {}),
    ("mavsdk.action",    {}),
    ("serial",           {"Serial": MagicMock}),
    ("aiohttp",          {"ClientSession": MagicMock,
                          "TCPConnector": MagicMock,
                          "ClientTimeout": MagicMock}),
    ("vosk",             {"Model": MagicMock,
                          "KaldiRecognizer": MagicMock,
                          "SetLogLevel": MagicMock}),
]:
    m = types.ModuleType(mod)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[mod] = m

import sys, json, os
from pathlib import Path
BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE))
sys.path.insert(0, str(BASE.parent.parent / "rebuild" / "master"))

from fieldmap import ORIGIN_LAT, ORIGIN_LON, local_to_gps

# ── helpers ──────────────────────────────────────────────────────────────────
EARTH_R = 6_378_137.0

def metres_north(lat, lon, dist_m):
    """Return GPS position dist_m north of (lat, lon)."""
    return lat + math.degrees(dist_m / EARTH_R), lon

def metres_east(lat, lon, dist_m):
    """Return GPS position dist_m east of (lat, lon)."""
    return lat, lon + math.degrees(dist_m / (EARTH_R * math.cos(math.radians(lat))))

def haversine(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = math.sin(math.radians(lat2-lat1)/2)**2 + \
        math.cos(p1)*math.cos(p2)*math.sin(math.radians(lon2-lon1)/2)**2
    return EARTH_R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

# ── per-drone state ───────────────────────────────────────────────────────────
class DroneState:
    """In-memory state of one drone — master or slave."""
    def __init__(self, name, start_lat, start_lon):
        self.name       = name
        self.lat        = start_lat
        self.lon        = start_lon
        self.alt        = 0.0
        self.armed      = False
        self.airborne   = False
        self.target_lat = start_lat
        self.target_lon = start_lon
        self.target_alt = 0.0

    def arm_and_takeoff(self, alt=1.5):
        self.armed    = True
        self.airborne = True
        self.alt      = alt
        self.target_alt = alt
        return f"{self.name} armed & airborne at {alt}m"

    def goto(self, lat, lon, alt=1.5):
        old = (self.lat, self.lon)
        self.lat, self.lon, self.alt = lat, lon, alt
        self.target_lat, self.target_lon = lat, lon
        dist = haversine(*old, lat, lon)
        return f"{self.name} moved {dist:.2f}m → ({lat:.6f},{lon:.6f})"

    def forward(self, dist_m):
        new_lat, new_lon = metres_north(self.lat, self.lon, dist_m)
        return self.goto(new_lat, new_lon, self.alt)

    def land(self):
        self.alt      = 0.0
        self.airborne = False
        self.armed    = False
        return f"{self.name} landed"

    def dist_to(self, other):
        return haversine(self.lat, self.lon, other.lat, other.lon)

    def __repr__(self):
        return (f"{self.name}  lat={self.lat:.6f}  lon={self.lon:.6f}  "
                f"alt={self.alt:.1f}m  armed={self.armed}  air={self.airborne}")

# ── fake slave TCP server ─────────────────────────────────────────────────────
class FakeSlaveServer:
    """
    Listens on a loopback port. Accepts commands from the master's SlaveConnection.
    Executes them against a DroneState and ACKs back.
    """
    def __init__(self, drone: DroneState, port: int):
        self.drone  = drone
        self.port   = port
        self._server = None
        self._log   = []

    async def start(self):
        self._server = await asyncio.start_server(
            self._handle, "127.0.0.1", self.port)
        return self

    async def _handle(self, reader, writer):
        async for raw in reader:
            try:
                pkt = json.loads(raw.decode().strip())
            except Exception:
                continue

            cmd = pkt.get("cmd", "").upper()
            seq = pkt.get("seq", 0)
            msg = self._execute(cmd, pkt)
            self._log.append(f"[{self.drone.name}] ← {cmd}  →  {msg}")

            ack = json.dumps({"ack": cmd, "seq": seq, "status": "ok"}) + "\n"
            writer.write(ack.encode())
            await writer.drain()

    def _execute(self, cmd, pkt):
        d = self.drone
        if cmd == "ARM_TAKEOFF":
            return d.arm_and_takeoff(float(pkt.get("alt", 1.5)))
        if cmd == "ARM_ONLY":
            d.armed = True
            return f"{d.name} armed (no takeoff)"
        if cmd == "GOTO":
            return d.goto(float(pkt["lat"]), float(pkt["lng"]),
                          float(pkt.get("alt", 1.5)))
        if cmd == "FORWARD":
            return d.forward(float(pkt.get("dist_m", 1.0)))
        if cmd == "HOVER":
            return f"{d.name} holding position"
        if cmd == "LAND":
            return d.land()
        if cmd in ("PAUSE", "RESUME", "START", "SIDE_MOVE", "DISARM"):
            return f"{d.name} ACK {cmd}"
        return f"{d.name} unknown cmd {cmd}"

    async def stop(self):
        if self._server:
            self._server.close()
            await self._server.wait_closed()

# ── master controller ─────────────────────────────────────────────────────────
class MasterController:
    """
    Uses the real SlaveConnection class from tcp_commander.py.
    Sends commands and awaits ACKs over real loopback TCP.
    """
    def __init__(self, slave_ports: dict[str, int]):
        from tcp_commander import SlaveConnection
        self.conns: dict[str, SlaveConnection] = {
            did: SlaveConnection(did, "127.0.0.1", port)
            for did, port in slave_ports.items()
        }
        self._tasks = []

    async def connect_all(self):
        for did, conn in self.conns.items():
            t = asyncio.create_task(conn.connect_loop(), name=f"conn_{did}")
            self._tasks.append(t)
        await asyncio.sleep(0.15)   # let all connections establish

    async def send(self, did: str, cmd: dict) -> dict:
        return await self.conns[did].send_command(cmd)

    async def broadcast(self, cmd: dict) -> dict[str, dict]:
        """Send the same command to all slaves simultaneously."""
        results = await asyncio.gather(
            *[self.conns[did].send_command(dict(cmd)) for did in self.conns]
        )
        return dict(zip(self.conns.keys(), results))

    async def stop(self):
        for t in self._tasks:
            t.cancel()

# ── voice command simulation ──────────────────────────────────────────────────
def simulate_voice(phrase: str) -> str:
    """
    Runs the same find_intent() logic as app.py.
    Returns the matched intent or raises if unrecognised.
    Adds arm, hover, forward to the intent table for this test.
    """
    INTENTS = {
        "arm":     ["arm", "arm up"],
        "start":   ["start", "begin", "launch"],
        "hover":   ["hover", "hold", "freeze"],
        "forward": ["forward", "advance", "move up"],
        "land":    ["land", "descend", "abort"],
    }
    lower = phrase.lower()
    for intent, triggers in INTENTS.items():
        for trigger in triggers:
            if trigger in lower:
                return intent
    raise ValueError(f"No intent matched: '{phrase}'")

# ── assertions ────────────────────────────────────────────────────────────────
_passed = 0
_failed = 0

def check(label: str, condition: bool, detail: str = ""):
    global _passed, _failed
    if condition:
        _passed += 1
        print(f"  ✓  {label}")
    else:
        _failed += 1
        print(f"  ✗  FAIL: {label}" + (f"  ({detail})" if detail else ""))

# ── main mission test ─────────────────────────────────────────────────────────
async def run_mission():
    print("=" * 60)
    print("  S.A.F.E. Mission Test")
    print("=" * 60)

    # ── spawn in-memory drones ──────────────────────────────────────────────
    # Formation: slaves side-by-side, 1.4m apart in X (east)
    # Master starts 2m south of the slave formation
    FORM_LAT, FORM_LON = ORIGIN_LAT, ORIGIN_LON

    slave_starts = {
        "slave_1": metres_east(FORM_LAT, FORM_LON, -1.4),
        "slave_2": (FORM_LAT, FORM_LON),
        "slave_3": metres_east(FORM_LAT, FORM_LON, +1.4),
    }
    master_start = metres_north(FORM_LAT, FORM_LON, -2.0)  # 2m behind

    drones = {
        "slave_1": DroneState("slave_1", *slave_starts["slave_1"]),
        "slave_2": DroneState("slave_2", *slave_starts["slave_2"]),
        "slave_3": DroneState("slave_3", *slave_starts["slave_3"]),
    }
    master = DroneState("master", *master_start)

    # ── find free ports ─────────────────────────────────────────────────────
    def free_port():
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            return s.getsockname()[1]

    ports = {did: free_port() for did in drones}

    # ── start fake slave TCP servers ────────────────────────────────────────
    servers = {did: FakeSlaveServer(drones[did], ports[did]) for did in drones}
    for srv in servers.values():
        await srv.start()

    # ── connect master TCP clients ──────────────────────────────────────────
    ctrl = MasterController(ports)
    await ctrl.connect_all()

    # ════════════════════════════════════════════════════════════════════════
    # STEP 1 — FORMATION: slaves take positions, then master takes position
    # ════════════════════════════════════════════════════════════════════════
    print("\n── Step 1: Formation ──────────────────────────────────────────")

    # Arm and position all three slaves simultaneously
    results = await ctrl.broadcast({"cmd": "ARM_TAKEOFF", "alt": 1.5})
    for did in drones:
        check(f"{did} armed & airborne", results[did].get("status") == "ok")
        check(f"{did} physically airborne", drones[did].airborne)

    # Send each slave to its formation GPS position
    for did, (slat, slon) in slave_starts.items():
        r = await ctrl.send(did, {"cmd": "GOTO",
                                  "lat": slat, "lng": slon, "alt": 1.5})
        check(f"{did} reached formation position",
              r.get("status") == "ok",
              f"lat={drones[did].lat:.6f}")

    # Inter-drone spacing check: each slave should be ~1.4m from its neighbour
    sep_12 = drones["slave_1"].dist_to(drones["slave_2"])
    sep_23 = drones["slave_2"].dist_to(drones["slave_3"])
    check("slave_1 ↔ slave_2 spacing ≈ 1.4m",
          abs(sep_12 - 1.4) < 0.3, f"{sep_12:.2f}m")
    check("slave_2 ↔ slave_3 spacing ≈ 1.4m",
          abs(sep_23 - 1.4) < 0.3, f"{sep_23:.2f}m")

    # Master arms and takes its own position (2m behind slave_2)
    master.arm_and_takeoff(1.5)
    master.goto(*master_start, 1.5)
    check("master airborne", master.airborne)

    master_to_s2 = master.dist_to(drones["slave_2"])
    check("master is behind slaves",
          master_to_s2 > 1.0, f"{master_to_s2:.2f}m behind slave_2")

    # ════════════════════════════════════════════════════════════════════════
    # STEP 2 — VOICE: arm  (already armed above; test the intent mapping)
    # ════════════════════════════════════════════════════════════════════════
    print("\n── Step 2: Voice 'arm' ────────────────────────────────────────")

    intent = simulate_voice("arm up please")
    check("voice 'arm up' → intent arm", intent == "arm")
    # Broadcast ARM_TAKEOFF (idempotent if already airborne)
    results = await ctrl.broadcast({"cmd": "ARM_TAKEOFF", "alt": 1.5})
    check("all slaves ACK second arm", all(
        r.get("status") == "ok" for r in results.values()))

    # ════════════════════════════════════════════════════════════════════════
    # STEP 3 — VOICE: start
    # ════════════════════════════════════════════════════════════════════════
    print("\n── Step 3: Voice 'start' ──────────────────────────────────────")

    intent = simulate_voice("begin mission")
    check("voice 'begin mission' → intent start", intent == "start")
    # On START all drones confirm they are airborne
    all_air = all(d.airborne for d in drones.values()) and master.airborne
    check("all drones airborne after start", all_air)

    # ════════════════════════════════════════════════════════════════════════
    # STEP 4 — VOICE: hover
    # ════════════════════════════════════════════════════════════════════════
    print("\n── Step 4: Voice 'hover' ──────────────────────────────────────")

    intent = simulate_voice("hover")
    check("voice 'hover' → intent hover", intent == "hover")
    results = await ctrl.broadcast({"cmd": "HOVER"})
    check("all slaves ACK hover", all(
        r.get("status") == "ok" for r in results.values()))

    # ════════════════════════════════════════════════════════════════════════
    # STEP 5 — ADVANCE: slaves step 0.5m at a time; master follows at 5m gap
    # ════════════════════════════════════════════════════════════════════════
    print("\n── Step 5: Advance — slaves lead, master follows at 5m gap ───")

    LEAD_GAP_M   = 5.0   # slaves must be this far ahead before master moves
    STEP_M       = 0.5   # one step size
    TOTAL_STEPS  = 14    # 7m total advance

    for step in range(TOTAL_STEPS):
        # All slaves step forward simultaneously
        results = await ctrl.broadcast({"cmd": "FORWARD", "dist_m": STEP_M})
        check(f"step {step+1:02d}: all slaves ACK FORWARD", all(
            r.get("status") == "ok" for r in results.values()))

        # Check how far the slave centroid is ahead of master
        avg_slave_lat = sum(d.lat for d in drones.values()) / 3
        avg_slave_lon = sum(d.lon for d in drones.values()) / 3
        gap = haversine(master.lat, master.lon, avg_slave_lat, avg_slave_lon)

        # Master follows once gap ≥ LEAD_GAP_M
        if gap >= LEAD_GAP_M:
            follow_lat, follow_lon = metres_north(
                avg_slave_lat, avg_slave_lon, -LEAD_GAP_M)
            master.goto(follow_lat, follow_lon, master.alt)

    # Final gap check
    avg_lat = sum(d.lat for d in drones.values()) / 3
    avg_lon = sum(d.lon for d in drones.values()) / 3
    final_gap = haversine(master.lat, master.lon, avg_lat, avg_lon)
    check("master is behind slaves at end of advance",
          master.lat < avg_lat,
          f"gap={final_gap:.2f}m")

    total_advance = haversine(FORM_LAT, FORM_LON,
                              drones["slave_2"].lat, drones["slave_2"].lon)
    check(f"slaves advanced ≈ {TOTAL_STEPS * STEP_M:.0f}m",
          abs(total_advance - TOTAL_STEPS * STEP_M) < 0.5,
          f"actual={total_advance:.2f}m")

    # ════════════════════════════════════════════════════════════════════════
    # STEP 6 — VOICE: forward (1m — entire swarm together)
    # ════════════════════════════════════════════════════════════════════════
    print("\n── Step 6: Voice 'forward' — swarm moves 1m ──────────────────")

    intent = simulate_voice("forward one metre")
    check("voice 'forward one metre' → intent forward", intent == "forward")

    # Record positions before
    s2_lat_before = drones["slave_2"].lat
    m_lat_before  = master.lat

    # Move all slaves 1m forward
    results = await ctrl.broadcast({"cmd": "FORWARD", "dist_m": 1.0})
    check("all slaves ACK 1m forward", all(
        r.get("status") == "ok" for r in results.values()))

    # Move master 1m forward too
    master.forward(1.0)

    # Verify everyone moved ~1m north
    s2_moved = haversine(s2_lat_before, drones["slave_2"].lon,
                         drones["slave_2"].lat, drones["slave_2"].lon)
    m_moved  = haversine(m_lat_before,  master.lon,
                         master.lat,    master.lon)
    check("slave_2 moved ≈ 1m forward", abs(s2_moved - 1.0) < 0.1, f"{s2_moved:.2f}m")
    check("master   moved ≈ 1m forward", abs(m_moved  - 1.0) < 0.1, f"{m_moved:.2f}m")

    # Formation spacing still correct after collective move
    sep = drones["slave_1"].dist_to(drones["slave_3"])
    check("slave_1 ↔ slave_3 still ≈ 2.8m after swarm forward",
          abs(sep - 2.8) < 0.4, f"{sep:.2f}m")

    # ════════════════════════════════════════════════════════════════════════
    # STEP 7 — VOICE: land  (slaves first in order, then master)
    # ════════════════════════════════════════════════════════════════════════
    print("\n── Step 7: Voice 'land' ───────────────────────────────────────")

    intent = simulate_voice("land now")
    check("voice 'land now' → intent land", intent == "land")

    # Land slaves in order: slave_1 → slave_2 → slave_3
    for did in ("slave_1", "slave_2", "slave_3"):
        r = await ctrl.send(did, {"cmd": "LAND"})
        check(f"{did} ACK land", r.get("status") == "ok")
        check(f"{did} physically landed", not drones[did].airborne)

    # Master lands itself
    master.land()
    check("master landed", not master.airborne)

    # All armed flags cleared
    none_armed = not any(d.armed for d in drones.values()) and not master.armed
    check("all drones disarmed after landing", none_armed)

    # ════════════════════════════════════════════════════════════════════════
    # TEARDOWN
    # ════════════════════════════════════════════════════════════════════════
    await ctrl.stop()
    for srv in servers.values():
        await srv.stop()

    # ── results ─────────────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    total = _passed + _failed
    print(f"  Results: {_passed}/{total} passed", end="")
    if _failed:
        print(f"  ({_failed} FAILED)")
    else:
        print("  — all OK ✓")
    print("=" * 60)

    # Print each drone's command log
    print("\nCommand log per slave:")
    for did, srv in servers.items():
        for entry in srv._log:
            print(f"  {entry}")

    return _failed == 0


if __name__ == "__main__":
    ok = asyncio.run(run_mission())
    sys.exit(0 if ok else 1)
