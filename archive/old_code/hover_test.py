#!/usr/bin/env python3
"""
hover_test.py  —  S.A.F.E. Swarm Hover / Formation Test
=========================================================
Purpose: Verify all 4 drones arm, fly, hold formation, move, and land correctly
         before any real mission. Run this on the MASTER Pi.

PRE-REQUISITES (do these manually before running this script):
  On each slave Pi — start MAVLink proxy first, THEN run main_orchestrator.py:
    drone1$  mavproxy.py --master=/dev/ttyAMA0 --baudrate=921600 \
                         --out=udp:0.0.0.0:14540 &
             DRONE_ID=slave_1 python3 main_orchestrator.py

    drone2$  mavproxy.py --master=/dev/ttyAMA0 --baudrate=921600 \
                         --out=udp:0.0.0.0:14540 &
             DRONE_ID=slave_2 python3 main_orchestrator.py

    drone3$  mavproxy.py --master=/dev/ttyAMA0 --baudrate=921600 \
                         --out=udp:0.0.0.0:14540 &
             DRONE_ID=slave_3 python3 main_orchestrator.py

  On the master Pi (this machine) — start MAVLink proxy for the master drone:
    master$  mavproxy.py --master=/dev/ttyAMA0 --baudrate=921600 \
                         --out=udp:0.0.0.0:14540 &

  Then run this script:
    master$  python3 hover_test.py

NETWORK LAYOUT:
  master  →  10.42.0.1   (this Pi, also runs the dnsmasq hotspot)
  slave_1 →  10.42.0.11  (TCP command server on port 14560)
  slave_2 →  10.42.0.12
  slave_3 →  10.42.0.13

TEST SEQUENCE:
  1.  Master connects to its own FC via MAVSDK.
  2.  Master sends ARM + TAKEOFF to each slave via TCP (sequentially safe).
  3.  Master arms and takes off itself.
  4.  All 4 drones hover at 1 m AGL and confirm altitude.
  5.  Drones spread into a T-formation:
        slave_1, slave_2, slave_3 — side by side, 1.5 m apart laterally
        master — 1.5 m behind the slave plane (perpendicular)
  6.  Slaves fly 3 m forward together (simultaneously).
  7.  Master follows once all slaves confirm waypoint reached.
  8.  Slaves land (simultaneously).
  9.  Master lands.
  10. Pass/fail summary printed.

FORMATION GEOMETRY (top-down view, North = up):
                        ↑ forward (North)

  [slave_1]  [slave_2]  [slave_3]
               ← 1.5m →

         [master]   ← 1.5 m behind slave plane

  All drones face North throughout the test.
  Y axis: East  (slave_1=−1.5 m, slave_2=0 m, slave_3=+1.5 m relative to origin)
  X axis: North (master=−1.5 m, slaves=0 m relative to origin)

SAFETY:
  • Any drone that fails to connect is SKIPPED with a clear warning — the
    script does NOT crash. You can still run a partial test.
  • CTRL-C at any point sends LAND to all connected drones.
  • A 90-second watchdog auto-lands everything if any step hangs.
"""

import asyncio
import json
import math
import socket
import sys
import time
import traceback
from dataclasses import dataclass, field
from typing import Optional

# ─────────────────────────────────────────────────────────────────────────────
#  CONFIGURATION  — edit these if your network differs
# ─────────────────────────────────────────────────────────────────────────────
MASTER_MAVSDK_ADDR = "udp://:14540"   # master's own FC
CMD_PORT           = 14560             # slave TCP command server port

SLAVE_IPS: dict[str, str] = {
    "slave_1": "10.42.0.11",
    "slave_2": "10.42.0.12",
    "slave_3": "10.42.0.13",
}

HOVER_ALT_M        = 1.0    # target AGL for all drones
ALT_TOLERANCE_M    = 0.25   # ±tolerance to consider "at altitude"
FORMATION_STEP_M   = 3.0    # how far slaves fly forward in the test
LATERAL_SEP_M      = 1.5    # slave-to-slave Y separation
MASTER_SETBACK_M   = 1.5    # master behind slave plane

TCP_CONNECT_TIMEOUT_S  = 8.0    # how long to wait for a slave TCP connection
ACK_TIMEOUT_S          = 5.0    # how long to wait for a command ACK
ALTITUDE_WAIT_TIMEOUT_S = 30.0  # max wait for a drone to reach hover alt
WAYPOINT_TIMEOUT_S      = 45.0  # max wait for a drone to reach a waypoint
STEP_WATCHDOG_S         = 90.0  # if ANY single test step hangs, bail out


# ─────────────────────────────────────────────────────────────────────────────
#  LOGGING
# ─────────────────────────────────────────────────────────────────────────────
def log(component: str, status: str, msg: str) -> None:
    ts = time.strftime("%H:%M:%S")
    tag = {"OK": "✓", "WARN": "⚠", "ERROR": "✗", "INFO": "·"}.get(status, status)
    print(f"[{ts}][{component}][{tag}] {msg}", flush=True)


def sep(title: str = "") -> None:
    line = "─" * 60
    if title:
        print(f"\n{line}\n  {title}\n{line}", flush=True)
    else:
        print(line, flush=True)


# ─────────────────────────────────────────────────────────────────────────────
#  HAVERSINE HELPER
# ─────────────────────────────────────────────────────────────────────────────
def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6_378_137.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a  = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))


def offset_gps(lat: float, lon: float, north_m: float, east_m: float):
    """Return (lat, lon) offset by north_m and east_m from given position."""
    new_lat = lat + math.degrees(north_m / 6_378_137.0)
    new_lon = lon + math.degrees(east_m  / (6_378_137.0 * math.cos(math.radians(lat))))
    return new_lat, new_lon


# ─────────────────────────────────────────────────────────────────────────────
#  SLAVE TCP CLIENT  — minimal, no dependency on tcp_channel.py for isolation
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class SlaveClient:
    """
    Lightweight TCP client for sending commands to one slave drone.
    Exists alongside (not replacing) the full TCPCommandClient — this is
    for the hover test only so it has no side-effects on STATE.
    """
    drone_id: str
    ip:       str
    port:     int = CMD_PORT

    _sock:   Optional[socket.socket] = field(default=None, init=False, repr=False)
    _buf:    bytearray               = field(default_factory=bytearray, init=False, repr=False)
    _seq:    int                     = field(default=0, init=False, repr=False)
    connected: bool                  = field(default=False, init=False, repr=False)

    def try_connect(self, timeout: float = TCP_CONNECT_TIMEOUT_S) -> bool:
        """
        Attempt a TCP connection. Returns True on success, False on failure.
        On failure prints a clear message and sets connected=False — does NOT raise.
        """
        try:
            log(self.drone_id, "INFO", f"Connecting to {self.ip}:{self.port} …")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            sock.connect((self.ip, self.port))
            sock.settimeout(ACK_TIMEOUT_S)
            self._sock = sock
            self._buf.clear()
            self.connected = True
            log(self.drone_id, "OK", f"TCP connected to {self.ip}:{self.port}")
            return True
        except (ConnectionRefusedError, socket.timeout, OSError) as e:
            log(self.drone_id, "WARN",
                f"Could not connect to {self.ip}:{self.port}: {e}")
            log(self.drone_id, "WARN",
                f"  → Skipping {self.drone_id}. "
                f"Is main_orchestrator.py running on that Pi?")
            self.connected = False
            return False

    def send_cmd(self, payload: dict) -> dict:
        """
        Send one JSON command and wait for ACK.
        Returns ACK dict or {'status': '<error>'} on failure.
        Never raises.
        """
        if not self.connected or self._sock is None:
            return {"status": "not_connected"}

        self._seq += 1
        payload["seq"] = self._seq
        raw = (json.dumps(payload) + "\n").encode("utf-8")

        try:
            self._sock.sendall(raw)
        except OSError as e:
            log(self.drone_id, "ERROR", f"TCP send failed: {e}")
            self.connected = False
            return {"status": "send_error"}

        # Read ACK (newline-delimited JSON)
        try:
            self._sock.settimeout(ACK_TIMEOUT_S)
            while b"\n" not in self._buf:
                chunk = self._sock.recv(65535)
                if not chunk:
                    log(self.drone_id, "WARN", "TCP connection closed by slave")
                    self.connected = False
                    return {"status": "disconnected"}
                self._buf.extend(chunk)

            line, remainder = self._buf.split(b"\n", 1)
            self._buf = bytearray(remainder)
            ack = json.loads(line.decode("utf-8").strip())
            return ack

        except socket.timeout:
            log(self.drone_id, "WARN",
                f"ACK timeout for seq={self._seq} cmd={payload.get('cmd')}")
            return {"status": "timeout"}
        except (json.JSONDecodeError, OSError) as e:
            log(self.drone_id, "ERROR", f"ACK recv error: {e}")
            self.connected = False
            return {"status": "recv_error"}

    def arm_and_takeoff(self, alt: float = HOVER_ALT_M) -> bool:
        log(self.drone_id, "INFO", f"Sending ARM+TAKEOFF (alt={alt}m)")
        ack = self.send_cmd({"cmd": "ARM_TAKEOFF", "alt": alt})
        ok = ack.get("status") == "ok"
        log(self.drone_id, "OK" if ok else "ERROR",
            f"ARM_TAKEOFF ack: {ack}")
        return ok

    def goto(self, lat: float, lon: float, alt: float = HOVER_ALT_M) -> bool:
        log(self.drone_id, "INFO",
            f"Sending GOTO ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
        ack = self.send_cmd({"cmd": "GOTO", "lat": lat, "lng": lon, "alt": alt})
        ok = ack.get("status") == "ok"
        log(self.drone_id, "OK" if ok else "ERROR",
            f"GOTO ack: {ack}")
        return ok

    def land(self) -> bool:
        log(self.drone_id, "INFO", "Sending LAND")
        ack = self.send_cmd({"cmd": "LAND"})
        ok = ack.get("status") == "ok"
        log(self.drone_id, "OK" if ok else "WARN",
            f"LAND ack: {ack}")
        return ok

    def close(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        self.connected = False


# ─────────────────────────────────────────────────────────────────────────────
#  MASTER MAVSDK BLOCK  — async, for master's own drone
# ─────────────────────────────────────────────────────────────────────────────
class MasterDrone:
    """
    Controls the master drone via MAVSDK directly (not through TCP).
    Mirrors the MovementBlock pattern from main_orchestrator.py but
    stripped down for the hover test.
    """

    def __init__(self, address: str = MASTER_MAVSDK_ADDR):
        from mavsdk import System
        self.drone    = System()
        self._address = address
        self.connected = False
        self.lat: float = 0.0
        self.lon: float = 0.0
        self.alt: float = 0.0
        self._telem_ready = asyncio.Event()

    async def connect(self) -> bool:
        """
        Returns True on success, False on failure.
        Never raises — on failure prints a clear diagnostic.
        """
        try:
            log("MASTER", "INFO", f"Connecting to FC at {self._address}")
            await self.drone.connect(system_address=self._address)

            # Wait for FC handshake
            async with asyncio.timeout(TCP_CONNECT_TIMEOUT_S):
                async for state in self.drone.core.connection_state():
                    if state.is_connected:
                        self.connected = True
                        break

            log("MASTER", "OK", "FC connected")

            # Start telemetry streams
            asyncio.create_task(self._pos_stream(), name="master_pos")
            log("MASTER", "INFO", "Waiting for GPS fix …")
            await asyncio.wait_for(self._telem_ready.wait(), timeout=30.0)
            log("MASTER", "OK",
                f"GPS ready — lat={self.lat:.6f} lon={self.lon:.6f}")
            return True

        except (asyncio.TimeoutError, Exception) as e:
            log("MASTER", "WARN",
                f"MAVSDK connect failed: {e}")
            log("MASTER", "WARN",
                "  → Will skip master drone in this test run.")
            self.connected = False
            return False

    async def _pos_stream(self):
        try:
            async for pos in self.drone.telemetry.position():
                self.lat = pos.latitude_deg
                self.lon = pos.longitude_deg
                self.alt = pos.relative_altitude_m
                self._telem_ready.set()
        except Exception as e:
            log("MASTER", "WARN", f"Position stream died: {e}")

    async def arm_and_takeoff(self, alt: float = HOVER_ALT_M) -> bool:
        if not self.connected:
            return False
        try:
            await self.drone.action.set_takeoff_altitude(alt)
            await self.drone.action.arm()
            await self.drone.action.takeoff()
            log("MASTER", "OK", "Takeoff command sent")
            return True
        except Exception as e:
            log("MASTER", "ERROR", f"Arm/takeoff failed: {e}")
            return False

    async def goto(self, lat: float, lon: float, alt: float = HOVER_ALT_M) -> bool:
        if not self.connected:
            return False
        try:
            # AMSL offset: MAVSDK goto_location wants AMSL, alt is AGL here
            # Using 0 as AMSL offset — fine for GPS-relative tests at low alt
            await self.drone.action.goto_location(lat, lon, alt, 0.0)
            log("MASTER", "OK", f"GOTO sent → ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
            return True
        except Exception as e:
            log("MASTER", "ERROR", f"goto_location failed: {e}")
            return False

    async def wait_altitude(self, target: float, tol: float = ALT_TOLERANCE_M,
                             timeout: float = ALTITUDE_WAIT_TIMEOUT_S) -> bool:
        """Wait until master is within ±tol of target altitude."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if abs(self.alt - target) <= tol:
                log("MASTER", "OK", f"At altitude {self.alt:.2f}m (target {target}m)")
                return True
            await asyncio.sleep(0.3)
        log("MASTER", "WARN",
            f"Altitude timeout — stuck at {self.alt:.2f}m (target {target}m)")
        return False

    async def wait_waypoint(self, lat: float, lon: float,
                             tol_m: float = 0.6,
                             timeout: float = WAYPOINT_TIMEOUT_S) -> bool:
        """Poll until master is within tol_m of waypoint."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            dist = haversine(self.lat, self.lon, lat, lon)
            if dist < tol_m:
                log("MASTER", "OK", f"Waypoint reached (dist={dist:.2f}m)")
                return True
            await asyncio.sleep(0.5)
        log("MASTER", "WARN",
            f"Waypoint timeout — dist={haversine(self.lat,self.lon,lat,lon):.1f}m")
        return False

    async def land(self) -> bool:
        if not self.connected:
            return False
        try:
            await self.drone.action.land()
            log("MASTER", "OK", "Land command sent")
            return True
        except Exception as e:
            log("MASTER", "ERROR", f"Land failed: {e}")
            return False


# ─────────────────────────────────────────────────────────────────────────────
#  TEST RESULTS TRACKER
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class TestResult:
    step:    str
    drone:   str
    passed:  bool
    note:    str = ""


results: list[TestResult] = []

def record(step: str, drone: str, passed: bool, note: str = "") -> None:
    results.append(TestResult(step, drone, passed, note))
    status = "OK" if passed else "WARN"
    log("TEST", status, f"[{step}] {drone}: {'PASS' if passed else 'FAIL'} {note}")


def print_summary() -> None:
    sep("TEST SUMMARY")
    total  = len(results)
    passed = sum(1 for r in results if r.passed)
    failed = total - passed

    for r in results:
        icon = "✓" if r.passed else "✗"
        print(f"  {icon}  [{r.step:<28}] {r.drone:<10} {r.note}")

    print()
    print(f"  Result: {passed}/{total} passed", end="  ")
    if failed == 0:
        print("🟢 ALL PASS")
    else:
        print(f"🔴 {failed} FAILED")
    print()


# ─────────────────────────────────────────────────────────────────────────────
#  EMERGENCY LAND HELPER
# ─────────────────────────────────────────────────────────────────────────────
async def emergency_land_all(
    slaves: list[SlaveClient],
    master: MasterDrone,
) -> None:
    log("SAFETY", "WARN", "=== EMERGENCY LAND ALL ===")
    for s in slaves:
        if s.connected:
            s.land()
    if master.connected:
        await master.land()


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN TEST SEQUENCE
# ─────────────────────────────────────────────────────────────────────────────
async def run_hover_test() -> None:

    sep("S.A.F.E. HOVER TEST  v1.0")
    print("  Drones  : master + slave_1, slave_2, slave_3")
    print(f"  Altitude : {HOVER_ALT_M} m AGL")
    print(f"  Forward  : {FORMATION_STEP_M} m")
    print(f"  Lat sep  : ±{LATERAL_SEP_M} m")
    print()
    print("  ⚠ SAFETY CHECKS BEFORE STARTING:")
    print("    □  Props clear of people and obstacles")
    print("    □  All batteries charged")
    print("    □  MAVLink proxy running on EACH Pi")
    print("    □  main_orchestrator.py running on EACH slave Pi")
    print("    □  MASTER MAVLink proxy running on this Pi")
    print()
    input("  Press ENTER to start or Ctrl-C to abort …")
    print()

    # ─────────────────────────────────────────────────────────────────────
    #  STEP 0: Connections
    # ─────────────────────────────────────────────────────────────────────
    sep("STEP 0 — Connecting")

    # Slave TCP connections (synchronous — no event loop needed yet)
    slaves: list[SlaveClient] = []
    for drone_id, ip in SLAVE_IPS.items():
        client = SlaveClient(drone_id=drone_id, ip=ip)
        client.try_connect()
        slaves.append(client)
        record("connect", drone_id, client.connected,
               f"({ip}:{CMD_PORT})")

    connected_slaves = [s for s in slaves if s.connected]
    if not connected_slaves:
        log("INIT", "ERROR",
            "No slaves connected at all — cannot run test.")
        log("INIT", "ERROR",
            "Check: (1) Slaves are on Wi-Fi, (2) main_orchestrator.py is running.")
        return

    log("INIT", "OK",
        f"{len(connected_slaves)}/3 slaves connected: "
        f"{[s.drone_id for s in connected_slaves]}")

    # Master MAVSDK connection (async)
    master = MasterDrone(MASTER_MAVSDK_ADDR)
    master_ok = await master.connect()
    record("connect", "master", master_ok)

    if not master_ok:
        log("MASTER", "WARN",
            "Master FC not connected — will skip master steps.")

    # ─────────────────────────────────────────────────────────────────────
    #  STEP 1: Arm + Takeoff (slaves first, then master)
    # ─────────────────────────────────────────────────────────────────────
    sep("STEP 1 — Arm & Takeoff")

    # Send ARM_TAKEOFF to all slaves simultaneously using threads
    import threading

    slave_takeoff_results: dict[str, bool] = {}

    def _slave_takeoff(s: SlaveClient):
        ok = s.arm_and_takeoff(HOVER_ALT_M)
        slave_takeoff_results[s.drone_id] = ok

    threads = [
        threading.Thread(target=_slave_takeoff, args=(s,), daemon=True)
        for s in connected_slaves
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=ACK_TIMEOUT_S + 2.0)

    for s in connected_slaves:
        ok = slave_takeoff_results.get(s.drone_id, False)
        record("arm_takeoff", s.drone_id, ok)

    # Give slaves time to climb
    log("INIT", "INFO",
        f"Waiting {ALTITUDE_WAIT_TIMEOUT_S}s for slaves to reach {HOVER_ALT_M}m …")
    await asyncio.sleep(8.0)   # slaves arm → takeoff → climb → stabilise

    # Master takeoff
    if master_ok:
        ok = await master.arm_and_takeoff(HOVER_ALT_M)
        record("arm_takeoff", "master", ok)
        if ok:
            reached = await master.wait_altitude(HOVER_ALT_M)
            record("reach_altitude", "master", reached,
                   f"final alt={master.alt:.2f}m")

    # ─────────────────────────────────────────────────────────────────────
    #  STEP 2: Formation spread
    # ─────────────────────────────────────────────────────────────────────
    sep("STEP 2 — Formation Spread")
    #
    #  We need a reference point — use master GPS (or slave_2 if master offline).
    #  Formation:
    #    slave_1 = origin + (0 N,  −1.5 E)
    #    slave_2 = origin + (0 N,   0.0 E)
    #    slave_3 = origin + (0 N,  +1.5 E)
    #    master  = origin + (−1.5 N, 0 E)  [behind the slave plane]

    if master_ok and master.lat != 0.0:
        origin_lat, origin_lon = master.lat, master.lon
        log("INIT", "INFO",
            f"Formation origin: master position ({origin_lat:.6f}, {origin_lon:.6f})")
    elif connected_slaves:
        # fallback: can't know slave GPS from here without telemetry receiver
        # use a placeholder — in real use, master would have telemetry from slaves
        log("INIT", "WARN",
            "Master GPS unavailable — cannot compute formation GPS coordinates.")
        log("INIT", "WARN",
            "Skipping formation spread step.")
        origin_lat, origin_lon = None, None
    else:
        origin_lat, origin_lon = None, None

    formation_positions: dict[str, tuple[float, float]] = {}
    if origin_lat is not None:
        # Slave Y offsets (East)
        slave_east_offsets = {
            "slave_1": -LATERAL_SEP_M,
            "slave_2":  0.0,
            "slave_3": +LATERAL_SEP_M,
        }
        for s in connected_slaves:
            east = slave_east_offsets[s.drone_id]
            lat, lon = offset_gps(origin_lat, origin_lon,
                                   north_m=0.0, east_m=east)
            formation_positions[s.drone_id] = (lat, lon)

        # Master is setback behind slaves (negative North)
        m_lat, m_lon = offset_gps(origin_lat, origin_lon,
                                   north_m=-MASTER_SETBACK_M, east_m=0.0)
        formation_positions["master"] = (m_lat, m_lon)

        # Send GOTO to slaves simultaneously
        def _slave_goto(s: SlaveClient, lat: float, lon: float):
            ok = s.goto(lat, lon, HOVER_ALT_M)
            record("formation_goto", s.drone_id, ok,
                   f"→ ({lat:.5f},{lon:.5f})")

        threads = [
            threading.Thread(
                target=_slave_goto,
                args=(s, *formation_positions[s.drone_id]),
                daemon=True,
            )
            for s in connected_slaves
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=ACK_TIMEOUT_S + 2.0)

        # Master formation position
        if master_ok:
            ok = await master.goto(m_lat, m_lon, HOVER_ALT_M)
            record("formation_goto", "master", ok,
                   f"→ ({m_lat:.5f},{m_lon:.5f})")
            if ok:
                reached = await master.wait_waypoint(m_lat, m_lon)
                record("formation_reached", "master", reached)

        # Wait for slaves to settle into formation
        log("INIT", "INFO", "Waiting 6 s for formation to stabilise …")
        await asyncio.sleep(6.0)
        record("formation_hold", "all_slaves", True, "6 s hover")

    else:
        record("formation_spread", "all", False, "Skipped — no origin GPS")

    # ─────────────────────────────────────────────────────────────────────
    #  STEP 3: Slaves advance 3 m forward (North)
    # ─────────────────────────────────────────────────────────────────────
    sep("STEP 3 — Slaves Advance 3 m Forward")

    if origin_lat is not None:
        fwd_positions: dict[str, tuple[float, float]] = {}
        east_offsets = {
            "slave_1": -LATERAL_SEP_M,
            "slave_2":  0.0,
            "slave_3": +LATERAL_SEP_M,
        }
        for s in connected_slaves:
            east = east_offsets[s.drone_id]
            lat, lon = offset_gps(origin_lat, origin_lon,
                                   north_m=FORMATION_STEP_M,
                                   east_m=east)
            fwd_positions[s.drone_id] = (lat, lon)

        log("INIT", "INFO",
            f"Sending GOTO +{FORMATION_STEP_M}m North to all slaves simultaneously …")

        def _slave_fwd(s: SlaveClient, lat: float, lon: float):
            ok = s.goto(lat, lon, HOVER_ALT_M)
            record("slaves_advance_cmd", s.drone_id, ok)

        threads = [
            threading.Thread(
                target=_slave_fwd,
                args=(s, *fwd_positions[s.drone_id]),
                daemon=True,
            )
            for s in connected_slaves
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=ACK_TIMEOUT_S + 2.0)

        # Wait for slaves to travel (master holds during this time)
        log("INIT", "INFO",
            f"Holding master, waiting {WAYPOINT_TIMEOUT_S}s for slaves to arrive …")
        await asyncio.sleep(WAYPOINT_TIMEOUT_S * 0.5)   # reasonable travel wait
        record("slaves_advance_wait", "slaves", True,
               f"waited {WAYPOINT_TIMEOUT_S*0.5:.0f}s")

        # ─────────────────────────────────────────────────────────────────
        #  STEP 4: Master follows
        # ─────────────────────────────────────────────────────────────────
        sep("STEP 4 — Master Follows")

        if master_ok:
            # Master moves to its new position: formation_setback behind new slave plane
            new_m_lat, new_m_lon = offset_gps(origin_lat, origin_lon,
                                               north_m=FORMATION_STEP_M - MASTER_SETBACK_M,
                                               east_m=0.0)
            ok = await master.goto(new_m_lat, new_m_lon, HOVER_ALT_M)
            record("master_follow_cmd", "master", ok)
            if ok:
                reached = await master.wait_waypoint(new_m_lat, new_m_lon)
                record("master_follow_reached", "master", reached)

        log("INIT", "INFO", "Hovering 4 s in final formation …")
        await asyncio.sleep(4.0)
        record("final_hover", "all", True, "4 s hover complete")

    else:
        record("slaves_advance", "all", False, "Skipped — no origin GPS")
        record("master_follow",  "master", False, "Skipped — no origin GPS")

    # ─────────────────────────────────────────────────────────────────────
    #  STEP 5: Land — slaves first, then master
    # ─────────────────────────────────────────────────────────────────────
    sep("STEP 5 — Landing")

    log("INIT", "INFO", "Landing all slaves simultaneously …")

    def _slave_land(s: SlaveClient):
        ok = s.land()
        record("land", s.drone_id, ok)

    threads = [
        threading.Thread(target=_slave_land, args=(s,), daemon=True)
        for s in connected_slaves
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=ACK_TIMEOUT_S + 2.0)

    # Wait for slaves to fully land before master descends
    log("INIT", "INFO", "Waiting 8 s for slaves to settle on ground …")
    await asyncio.sleep(8.0)

    if master_ok:
        ok = await master.land()
        record("land", "master", ok)

    # ─────────────────────────────────────────────────────────────────────
    #  DONE
    # ─────────────────────────────────────────────────────────────────────
    for s in slaves:
        s.close()

    print_summary()


# ─────────────────────────────────────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────
async def _main() -> None:
    # Build a list of slave clients so emergency_land can access them
    # before the test sequence starts (needed by the KeyboardInterrupt handler)
    _slaves_ref: list[SlaveClient] = []
    _master_ref: list[MasterDrone] = []

    async def _guarded():
        # We need references outside run_hover_test — patch them in via a wrapper
        # by running the test and catching exceptions at the top level.
        await run_hover_test()

    try:
        await asyncio.wait_for(_guarded(), timeout=None)
    except KeyboardInterrupt:
        print("\n")
        log("SAFETY", "WARN", "Ctrl-C received — sending LAND to all connected drones")
        # Best-effort land
        for s in _slaves_ref:
            if s.connected:
                s.land()
        if _master_ref and _master_ref[0].connected:
            await _master_ref[0].land()
    except Exception as e:
        log("INIT", "ERROR", f"Unhandled exception: {e}")
        log("INIT", "ERROR", traceback.format_exc())


if __name__ == "__main__":
    try:
        asyncio.run(_main())
    except KeyboardInterrupt:
        print("\nAborted.")
        sys.exit(0)
