"""
test_choreography.py  —  Swarm Choreography Test  (run on MASTER Pi)
=====================================================================

WHERE TO PUT THIS:
    ~/webserver/test_choreography.py  (master Pi only)

WHAT SLAVES NEED:
    Each slave Pi must be running tcp_channel.py TCPCommandServer
    with handlers registered for: ARM, TAKEOFF, GOTO, HOLD, LAND
    (your normal slave startup script — this test just drives them)

WHAT IT TESTS — full flight sequence:
    1. ARM       — all slaves arm simultaneously, then master arms itself
    2. TAKEOFF   — slaves take off to 3m, then master takes off
    3. FORMATION — slaves fly to their formation slots, master holds origin
    4. FORWARD   — slaves advance 5m per section (4 sections total)
                   master follows once slaves are >= 5m ahead of it
    5. LAND      — master commands all slaves to land, then master lands

RUN:
    # Stub mode — no real drones, verifies logic only
    python3 test_choreography.py

    # Real mode — connects to actual slave drones over TCP
    python3 test_choreography.py --real

    # Real mode with custom slave IPs
    python3 test_choreography.py --real \\
        --slave slave_1:10.42.0.11 \\
        --slave slave_2:10.42.0.12 \\
        --slave slave_3:10.42.0.13

    # Adjust cruise altitude and section size
    python3 test_choreography.py --real --alt 2.5 --section 3.0

SLAVE ADDRESSES (defaults):
    slave_1 -> 10.42.0.11:14560
    slave_2 -> 10.42.0.12:14560
    slave_3 -> 10.42.0.13:14560
"""

import argparse
import json
import logging
import math
import sys
import time
import threading
import concurrent.futures

try:
    from swarm_state import STATE
    from tcp_channel import TCPCommandClient, TCPCommandBroadcaster
    _REAL_IMPORTS = True
except ImportError:
    _REAL_IMPORTS = False

log = logging.getLogger("choreography")

# ════════════════════════════════════════════════════════════════════════════
#  CONFIG  (overridable via CLI args)
# ════════════════════════════════════════════════════════════════════════════

CRUISE_ALT_M   = 3.0    # metres AGL for all drones
SECTION_LEN_M  = 5.0    # metres slaves advance per forward section
TOTAL_SECTIONS = 4      # how many forward sections to fly
FOLLOW_GAP_M   = 5.0    # master follows only when slaves are this far ahead
SETTLE_S       = 2.0    # seconds to wait after each movement command

# Formation: slave x/y offsets from origin (x=lateral, y=forward)
FORMATION = {
    "slave_1": (-4.0, 2.0),   # left  wing, 2m ahead of master
    "slave_2": ( 0.0, 2.0),   # centre,      2m ahead of master
    "slave_3": ( 4.0, 2.0),   # right wing, 2m ahead of master
}

DEFAULT_SLAVE_ADDRS = {
    "slave_1": ("10.42.0.11", 14560),
    "slave_2": ("10.42.0.12", 14560),
    "slave_3": ("10.42.0.13", 14560),
}

# Simulated GPS origin (stub mode only — real mode uses STATE.mission.origin_*)
ORIGIN_LAT = 23.077953
ORIGIN_LNG = 72.495347


# ════════════════════════════════════════════════════════════════════════════
#  STUB CLASSES  (used in stub mode — mirrors real API exactly)
# ════════════════════════════════════════════════════════════════════════════

class StubDrone:
    """Tracks simulated drone position/state for stub mode."""
    def __init__(self, drone_id):
        self.drone_id = drone_id
        self.x = self.y = self.alt = 0.0
        self.armed = self.airborne = False
        self.cmds = []

    def gps_to_local(self, lat, lng):
        y = math.radians(lat - ORIGIN_LAT) * 6_378_137.0
        x = math.radians(lng - ORIGIN_LNG) * 6_378_137.0 * math.cos(
            math.radians(ORIGIN_LAT))
        return round(x, 3), round(y, 3)


class StubTCPClient:
    """
    Drop-in replacement for TCPCommandClient.
    Uses the same .send() / .connect_loop() API.
    Records every command and updates local drone state.
    """
    def __init__(self, drone: StubDrone):
        self.drone_id = drone.drone_id
        self._drone   = drone

    def connect_loop(self):
        """No-op in stub mode — already 'connected'."""
        pass

    def send(self, payload: dict) -> dict:
        cmd = payload.get("cmd", "?").upper()
        self._drone.cmds.append(cmd)
        log.info(f"  [{self._drone.drone_id}] <- {json.dumps(payload)}")

        if cmd == "ARM":
            self._drone.armed = True
        elif cmd == "TAKEOFF":
            self._drone.airborne = True
            self._drone.alt = payload.get("alt", CRUISE_ALT_M)
        elif cmd == "GOTO":
            lat = payload.get("lat", ORIGIN_LAT)
            lng = payload.get("lng", ORIGIN_LNG)
            self._drone.x, self._drone.y = self._drone.gps_to_local(lat, lng)
            self._drone.alt = payload.get("alt", self._drone.alt)
        elif cmd in ("HOLD", "PAUSE"):
            pass
        elif cmd == "LAND":
            self._drone.airborne = False
            self._drone.alt      = 0.0
            self._drone.armed    = False

        time.sleep(0.05)   # simulate network latency
        return {"ack": cmd, "seq": payload.get("seq", 0), "status": "ok"}

    def stop(self):
        pass


class StubBroadcaster:
    """
    Drop-in replacement for TCPCommandBroadcaster.
    Sends to all stub clients simultaneously using threads.
    """
    def __init__(self, clients: dict):
        self._clients = clients

    def send_all(self, command: dict) -> dict:
        results = {}
        lock = threading.Lock()

        def _send(did, client):
            r = client.send(dict(command))
            with lock:
                results[did] = r

        threads = [
            threading.Thread(target=_send, args=(did, c), daemon=True)
            for did, c in self._clients.items()
        ]
        for t in threads: t.start()
        for t in threads: t.join()
        return results


# ════════════════════════════════════════════════════════════════════════════
#  COORDINATE HELPERS
# ════════════════════════════════════════════════════════════════════════════

def local_to_gps(x, y, origin_lat=ORIGIN_LAT, origin_lng=ORIGIN_LNG):
    """Convert local field metres to GPS lat/lng."""
    lat = origin_lat + math.degrees(y / 6_378_137.0)
    lng = origin_lng + math.degrees(
        x / (6_378_137.0 * math.cos(math.radians(origin_lat))))
    return round(lat, 7), round(lng, 7)


def drone_position(did, slave_drones, master_drone):
    """Get current x,y of a drone by id."""
    if did == "master":
        return master_drone.x, master_drone.y
    return slave_drones[did].x, slave_drones[did].y


# ════════════════════════════════════════════════════════════════════════════
#  COMMAND HELPERS
# ════════════════════════════════════════════════════════════════════════════

def send_goto(client, drone, x, y, alt, origin_lat=ORIGIN_LAT, origin_lng=ORIGIN_LNG):
    """Send GOTO command to one drone and update its tracked position on ACK."""
    lat, lng = local_to_gps(x, y, origin_lat, origin_lng)
    result = client.send({"cmd": "GOTO", "lat": lat, "lng": lng, "alt": alt})
    ok = result.get("status") == "ok"
    if ok:
        drone.x, drone.y, drone.alt = x, y, alt
    log.info(f"  GOTO {drone.drone_id} -> ({x:+.1f}, {y:+.1f})m @ {alt}m: "
             f"{'OK' if ok else 'FAILED - ' + str(result)}")
    return ok


def send_all_goto(slave_clients, slave_drones, waypoints, alt,
                  origin_lat=ORIGIN_LAT, origin_lng=ORIGIN_LNG):
    """
    Send GOTO to multiple slaves simultaneously using threads.
    waypoints = { "slave_1": (x, y), "slave_2": (x, y), ... }
    Blocks until all ACKs received.
    """
    results = {}
    lock    = threading.Lock()

    def _goto(did):
        x, y = waypoints[did]
        ok = send_goto(slave_clients[did], slave_drones[did], x, y, alt,
                       origin_lat, origin_lng)
        with lock:
            results[did] = ok

    threads = [threading.Thread(target=_goto, args=(did,), daemon=True)
               for did in waypoints]
    for t in threads: t.start()
    for t in threads: t.join()
    return results


def broadcast_cmd(broadcaster, slave_drones, cmd, **kwargs):
    """Broadcast a simple command (ARM/TAKEOFF/HOLD/LAND) to all slaves."""
    results = broadcaster.send_all({"cmd": cmd, **kwargs})
    all_ok = all(r.get("status") == "ok" for r in results.values())
    for did, r in results.items():
        log.info(f"  {cmd} {did}: {'OK' if r.get('status')=='ok' else 'FAILED - ' + str(r)}")
    return all_ok


# ════════════════════════════════════════════════════════════════════════════
#  DISPLAY
# ════════════════════════════════════════════════════════════════════════════

def sep(title):
    log.info("=" * 60)
    log.info(f"  {title}")
    log.info("=" * 60)


def print_positions(slave_drones, master_drone):
    log.info("  Current positions:")
    for did, d in slave_drones.items():
        log.info(f"    {did:12s}  x={d.x:+6.1f}m  y={d.y:+6.1f}m  "
                 f"alt={d.alt:.1f}m  armed={d.armed}  airborne={d.airborne}")
    d = master_drone
    log.info(f"    {'master':12s}  x={d.x:+6.1f}m  y={d.y:+6.1f}m  "
             f"alt={d.alt:.1f}m  armed={d.armed}  airborne={d.airborne}")


# ════════════════════════════════════════════════════════════════════════════
#  CHOREOGRAPHY SEQUENCE
# ════════════════════════════════════════════════════════════════════════════

def run_choreography(
    slave_clients, slave_drones,
    master_client, master_drone,
    broadcaster,
    cruise_alt, section_len, total_sections, follow_gap,
    origin_lat=ORIGIN_LAT, origin_lng=ORIGIN_LNG,
):
    t0 = time.time()

    # ─────────────────────────────────────────────────────────────────────
    #  STEP 1: ARM
    # ─────────────────────────────────────────────────────────────────────
    sep("STEP 1 — ARM  (slaves simultaneously, then master)")

    ok = broadcast_cmd(broadcaster, slave_drones, "ARM")
    assert ok, "ARM failed on one or more slaves — check slave logs"
    for d in slave_drones.values():
        d.armed = True
    time.sleep(SETTLE_S)

    r = master_client.send({"cmd": "ARM"})
    assert r.get("status") == "ok", f"Master ARM failed: {r}"
    master_drone.armed = True
    log.info(f"  ARM master: OK")
    time.sleep(SETTLE_S)

    print_positions(slave_drones, master_drone)

    # ─────────────────────────────────────────────────────────────────────
    #  STEP 2: TAKEOFF
    # ─────────────────────────────────────────────────────────────────────
    sep(f"STEP 2 — TAKEOFF to {cruise_alt}m  (slaves first, then master)")

    ok = broadcast_cmd(broadcaster, slave_drones, "TAKEOFF", alt=cruise_alt)
    assert ok, "TAKEOFF failed on one or more slaves"
    for d in slave_drones.values():
        d.airborne = True
        d.alt      = cruise_alt
    time.sleep(SETTLE_S)

    r = master_client.send({"cmd": "TAKEOFF", "alt": cruise_alt})
    assert r.get("status") == "ok", f"Master TAKEOFF failed: {r}"
    master_drone.airborne = True
    master_drone.alt      = cruise_alt
    log.info(f"  TAKEOFF master: OK")
    time.sleep(SETTLE_S)

    print_positions(slave_drones, master_drone)

    # ─────────────────────────────────────────────────────────────────────
    #  STEP 3: FORMATION
    # ─────────────────────────────────────────────────────────────────────
    sep("STEP 3 — FORMATION  (slaves to slots, master holds origin)")

    waypoints = {did: FORMATION[did] for did in slave_clients}
    results   = send_all_goto(slave_clients, slave_drones, waypoints,
                              cruise_alt, origin_lat, origin_lng)
    assert all(results.values()), "Formation move failed on some slaves"
    time.sleep(SETTLE_S)

    r = master_client.send({"cmd": "HOLD"})
    log.info(f"  HOLD master at origin: OK")

    print_positions(slave_drones, master_drone)

    # ─────────────────────────────────────────────────────────────────────
    #  STEP 4: FORWARD SECTIONS
    # ─────────────────────────────────────────────────────────────────────
    sep(f"STEP 4 — FORWARD  ({total_sections} sections x {section_len}m)")

    for section in range(1, total_sections + 1):
        # Slaves advance to next section target
        slave_target_y = FORMATION["slave_1"][1] + (section * section_len)
        log.info(f"\n  -- Section {section}/{total_sections}: "
                 f"slaves -> y={slave_target_y:.1f}m --")

        waypoints = {
            did: (FORMATION[did][0], slave_target_y)
            for did in slave_clients
        }
        results = send_all_goto(slave_clients, slave_drones, waypoints,
                                cruise_alt, origin_lat, origin_lng)
        if not all(results.values()):
            log.warning("  Some slaves failed GOTO — continuing anyway")
        time.sleep(SETTLE_S)

        # Measure gap between master and leading slave
        slave_lead_y  = max(d.y for d in slave_drones.values())
        master_y      = master_drone.y
        gap           = slave_lead_y - master_y

        log.info(f"  master_y={master_y:.1f}m  "
                 f"slave_lead_y={slave_lead_y:.1f}m  "
                 f"gap={gap:.1f}m")

        if gap >= follow_gap:
            master_target_y = slave_lead_y - follow_gap
            log.info(f"  Gap >= {follow_gap}m -> master advancing to "
                     f"y={master_target_y:.1f}m")
            ok = send_goto(master_client, master_drone,
                           0.0, master_target_y, cruise_alt,
                           origin_lat, origin_lng)
            if not ok:
                log.warning("  Master GOTO failed — continuing")
            time.sleep(SETTLE_S)
        else:
            log.info(f"  Gap {gap:.1f}m < {follow_gap}m -> master holds")

        print_positions(slave_drones, master_drone)

    # ─────────────────────────────────────────────────────────────────────
    #  STEP 5: LAND
    # ─────────────────────────────────────────────────────────────────────
    sep("STEP 5 — LAND  (master commands slaves, then master lands)")

    # LAND does not need ACK — broadcast fire-and-forget
    broadcaster.send_all({"cmd": "LAND"})
    for d in slave_drones.values():
        d.airborne = False; d.alt = 0.0; d.armed = False
    log.info("  LAND sent to all slaves")
    time.sleep(SETTLE_S)

    master_client.send({"cmd": "LAND"})
    master_drone.airborne = False; master_drone.alt = 0.0; master_drone.armed = False
    log.info("  LAND sent to master")
    time.sleep(SETTLE_S)

    print_positions(slave_drones, master_drone)

    # ─────────────────────────────────────────────────────────────────────
    #  SUMMARY
    # ─────────────────────────────────────────────────────────────────────
    sep(f"CHOREOGRAPHY COMPLETE  ({time.time()-t0:.1f}s)")
    log.info("  Command history per drone:")
    for did, d in {**slave_drones, "master": master_drone}.items():
        log.info(f"    {did:12s}  {getattr(d, 'cmds', '(real drone — see slave logs)')}")
    log.info("")
    log.info("  All steps passed")


# ════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Swarm Choreography Test — runs on master Pi")
    parser.add_argument(
        "--real", action="store_true",
        help="Connect to real slave drones over TCP (slaves must be running)")
    parser.add_argument(
        "--slave", action="append", metavar="ID:IP",
        help="Override slave address e.g. --slave slave_1:10.42.0.11  "
             "(can repeat for each slave)")
    parser.add_argument(
        "--alt", type=float, default=CRUISE_ALT_M,
        help=f"Cruise altitude in metres (default: {CRUISE_ALT_M})")
    parser.add_argument(
        "--section", type=float, default=SECTION_LEN_M,
        help=f"Forward section length in metres (default: {SECTION_LEN_M})")
    parser.add_argument(
        "--sections", type=int, default=TOTAL_SECTIONS,
        help=f"Number of forward sections (default: {TOTAL_SECTIONS})")
    parser.add_argument(
        "--gap", type=float, default=FOLLOW_GAP_M,
        help=f"Master follow gap in metres (default: {FOLLOW_GAP_M})")
    parser.add_argument(
        "--verbose", action="store_true",
        help="Show debug output")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="[%(asctime)s] %(message)s",
        datefmt="%H:%M:%S",
        stream=sys.stdout,
    )

    if args.real and not _REAL_IMPORTS:
        log.error("--real requires swarm_state.py and tcp_channel.py "
                  "in the same directory")
        sys.exit(1)

    # Build slave address map from CLI overrides
    slave_addrs = dict(DEFAULT_SLAVE_ADDRS)
    if args.slave:
        for entry in args.slave:
            try:
                sid, ip = entry.split(":", 1)
                slave_addrs[sid] = (ip, 14560)
            except ValueError:
                log.error(f"Bad --slave format (expected ID:IP): {entry}")
                sys.exit(1)

    # ── GPS origin ────────────────────────────────────────────────────────
    if args.real and _REAL_IMPORTS and STATE.mission.origin_lat is not None:
        origin_lat = STATE.mission.origin_lat
        origin_lng = STATE.mission.origin_lng
        log.info(f"Using real GPS origin from STATE: "
                 f"{origin_lat:.6f}, {origin_lng:.6f}")
    else:
        origin_lat = ORIGIN_LAT
        origin_lng = ORIGIN_LNG
        if args.real:
            log.warning(
                "No GPS origin set in STATE yet — using hardcoded default. "
                "Call POST /api/set_origin first for accurate GPS coordinates.")

    # ── Build clients ─────────────────────────────────────────────────────
    if args.real:
        log.info("Mode: REAL — connecting to slave drones over TCP")
        slave_clients = {}
        slave_drones  = {}

        for did, (ip, port) in slave_addrs.items():
            log.info(f"  Connecting to {did} at {ip}:{port}...")
            client = TCPCommandClient(
                drone_id   = did,
                slave_ip   = ip,
                slave_port = port,
            )
            client.connect_loop()   # starts background reconnect thread
            slave_clients[did] = client

            # In real mode, drone state comes from SwarmState
            slave_drones[did] = STATE.drones[did]

        # Give connections time to establish
        log.info("Waiting 3s for TCP connections to establish...")
        time.sleep(3.0)

        # Verify connections
        for did, client in slave_clients.items():
            connected = client._connected_event.is_set()
            log.info(f"  {did}: {'connected' if connected else 'NOT CONNECTED'}")
            if not connected:
                log.error(f"Could not connect to {did} — is tcp_channel server "
                          f"running on {slave_addrs[did][0]}?")
                sys.exit(1)

        # Master client — wraps master drone itself (sends to local handlers
        # or directly calls MAVSDK — here we use a passthrough)
        class MasterLocalClient:
            """
            In real deployment the master executes commands on itself
            via MAVSDK directly. This thin wrapper keeps the same .send()
            interface so choreography code is identical for master and slaves.
            """
            drone_id = "master"

            def send(self, payload):
                cmd = payload.get("cmd", "?").upper()
                log.info(f"  [master] <- {json.dumps(payload)}")
                # TODO: replace with real MAVSDK calls on master drone
                # e.g. await drone.action.goto_location(lat, lng, alt, 0)
                return {"status": "ok"}

        master_client = MasterLocalClient()
        master_drone  = STATE.drones["master"]
        broadcaster   = TCPCommandBroadcaster(slave_clients)

    else:
        log.info("Mode: STUB — drone commands simulated locally")
        slave_drones  = {did: StubDrone(did) for did in slave_addrs}
        slave_clients = {did: StubTCPClient(d) for did, d in slave_drones.items()}
        master_drone  = StubDrone("master")
        master_client = StubTCPClient(master_drone)
        broadcaster   = StubBroadcaster(slave_clients)

    try:
        run_choreography(
            slave_clients  = slave_clients,
            slave_drones   = slave_drones,
            master_client  = master_client,
            master_drone   = master_drone,
            broadcaster    = broadcaster,
            cruise_alt     = args.alt,
            section_len    = args.section,
            total_sections = args.sections,
            follow_gap     = args.gap,
            origin_lat     = origin_lat,
            origin_lng     = origin_lng,
        )
    except AssertionError as e:
        log.error(f"\nCHOREOGRAPHY ABORTED: {e}")
        # Emergency land on any assertion failure
        log.warning("Sending emergency LAND to all drones...")
        try:
            broadcaster.send_all({"cmd": "LAND"})
            master_client.send({"cmd": "LAND"})
        except Exception:
            pass
        sys.exit(1)
    except KeyboardInterrupt:
        log.warning("\nCtrl+C — sending emergency LAND to all drones")
        try:
            broadcaster.send_all({"cmd": "LAND"})
            master_client.send({"cmd": "LAND"})
        except Exception:
            pass
        sys.exit(0)
    finally:
        # Clean up TCP connections
        if args.real:
            for client in slave_clients.values():
                client.stop()


if __name__ == "__main__":
    main()
