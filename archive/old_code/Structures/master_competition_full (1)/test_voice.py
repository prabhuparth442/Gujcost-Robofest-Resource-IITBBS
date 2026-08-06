"""
test_voice.py  —  Voice Command Test  (run on MASTER Pi)
=========================================================

WHERE TO PUT THIS:
    Master Pi only — ~/webserver/test_voice.py
    Slaves must be running tcp_channel.py (their normal server).
    app.py must be running — this script polls it for commands.

HOW IT WORKS:
    - Does NOT simulate or type commands itself
    - Polls app.py's /api/pending_commands every 200ms (same as tcp_commander)
    - You speak into the phone browser (or click buttons in app.py UI)
    - When a command arrives it is executed on the real/stub drones
    - If NO command arrives for 10 seconds -> auto-land all drones and exit

COMMANDS HANDLED (same as app.py intents):
    arm      ->  ARM all drones
    start    ->  TAKEOFF slaves then master
    pause    ->  HOLD entire swarm
    resume   ->  resume movement
    forward  ->  move entire swarm 1m forward
    land     ->  LAND slaves then master  (also triggered by 10s timeout)

RUN:
    # Stub mode — drone commands are simulated, but app.py must be running
    python3 test_voice.py

    # Real mode — app.py running + slave drones online
    python3 test_voice.py --real

    # Point at a different app.py host
    python3 test_voice.py --app-url https://192.168.1.10

REQUIRES:
    pip install requests
"""

import argparse
import json
import logging
import math
import sys
import time
import concurrent.futures

try:
    import requests
    _REQUESTS_OK = True
except ImportError:
    _REQUESTS_OK = False

try:
    from swarm_state import STATE
    from tcp_channel import TCPCommandClient
    _REAL_IMPORTS = True
except ImportError:
    _REAL_IMPORTS = False

log = logging.getLogger("voice_test")

# ════════════════════════════════════════════════════════════════════════════
#  CONFIG
# ════════════════════════════════════════════════════════════════════════════

CRUISE_ALT_M    = 3.0
POLL_INTERVAL_S = 0.2    # how often to ask app.py for new commands
NO_CMD_TIMEOUT  = 10.0   # seconds of silence before auto-land

SLAVE_ADDRS = {
    "slave_1": ("10.42.0.11", 14560),
    "slave_2": ("10.42.0.12", 14560),
    "slave_3": ("10.42.0.13", 14560),
}

DEFAULT_APP_URL = "https://10.42.0.1"

ORIGIN_LAT = 23.077953
ORIGIN_LNG = 72.495347


# ════════════════════════════════════════════════════════════════════════════
#  STUB CLASSES
# ════════════════════════════════════════════════════════════════════════════

class StubDrone:
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


class StubClient:
    def __init__(self, drone):
        self.drone = drone

    def send(self, payload):
        cmd = payload.get("cmd", "?").upper()
        self.drone.cmds.append(cmd)
        log.info(f"    [{self.drone.drone_id}] <- {json.dumps(payload)}")
        if cmd == "ARM":
            self.drone.armed = True
        elif cmd == "TAKEOFF":
            self.drone.airborne = True
            self.drone.alt = payload.get("alt", CRUISE_ALT_M)
        elif cmd == "GOTO":
            lat = payload.get("lat", ORIGIN_LAT)
            lng = payload.get("lng", ORIGIN_LNG)
            self.drone.x, self.drone.y = self.drone.gps_to_local(lat, lng)
            self.drone.alt = payload.get("alt", self.drone.alt)
        elif cmd in ("HOLD", "PAUSE"):
            pass
        elif cmd == "LAND":
            self.drone.airborne = False
            self.drone.alt = 0.0
            self.drone.armed = False
        return {"status": "ok"}


# ════════════════════════════════════════════════════════════════════════════
#  HELPERS
# ════════════════════════════════════════════════════════════════════════════

def local_to_gps(x, y, origin_lat=None, origin_lng=None):
    """Convert local field metres to GPS. Uses STATE origin in real mode if set."""
    if origin_lat is None:
        # Try real STATE origin first, fall back to hardcoded constant
        if _REAL_IMPORTS and STATE.mission.origin_lat is not None:
            origin_lat = STATE.mission.origin_lat
            origin_lng = STATE.mission.origin_lng
        else:
            origin_lat = ORIGIN_LAT
            origin_lng = ORIGIN_LNG
    lat = origin_lat + math.degrees(y / 6_378_137.0)
    lng = origin_lng + math.degrees(
        x / (6_378_137.0 * math.cos(math.radians(origin_lat))))
    return round(lat, 7), round(lng, 7)


def sep(title):
    log.info("-" * 55)
    log.info(f"  {title}")
    log.info("-" * 55)


def print_positions(drones):
    for did, d in drones.items():
        log.info(f"    {did:12s}  x={d.x:+5.1f}m  y={d.y:+5.1f}m  "
                 f"alt={d.alt:.1f}m  armed={d.armed}  air={d.airborne}")


def broadcast(clients, cmd, **kwargs):
    with concurrent.futures.ThreadPoolExecutor() as ex:
        futures = {
            did: ex.submit(client.send, {"cmd": cmd, **kwargs})
            for did, client in clients.items()
        }
        return {did: fut.result() for did, fut in futures.items()}


# ════════════════════════════════════════════════════════════════════════════
#  POLL app.py FOR COMMANDS
# ════════════════════════════════════════════════════════════════════════════

def fetch_pending_commands(app_url):
    """
    GET /api/pending_commands from app.py.
    app.py clears the list on each GET so commands are consumed once.
    Returns list of strings like ["start", "forward"] or [] if none/unreachable.
    """
    try:
        resp = requests.get(
            f"{app_url}/api/pending_commands",
            timeout=0.4,
            verify=False,   # self-signed cert on Pi
        )
        if resp.status_code == 200:
            return resp.json().get("commands", [])
    except Exception as e:
        log.debug(f"  [poll] app.py unreachable: {e}")
    return []


# ════════════════════════════════════════════════════════════════════════════
#  EXECUTE COMMAND
# ════════════════════════════════════════════════════════════════════════════

def execute_command(cmd, slave_clients, slave_drones, master_client, master_drone):
    """
    Execute a command string that came from app.py.
    This mirrors what tcp_commander._voice_bridge() does in production.
    """
    cmd = cmd.lower().strip()
    all_clients = {**slave_clients, "master": master_client}
    all_drones  = {**slave_drones,  "master": master_drone}

    if cmd == "arm":
        results = broadcast(all_clients, "ARM")
        for d in all_drones.values():
            d.armed = True
        ok = all(r.get("status") == "ok" for r in results.values())
        log.info(f"  -> ARM all: {'OK' if ok else 'FAILED'}")

    elif cmd == "start":
        results = broadcast(slave_clients, "TAKEOFF", alt=CRUISE_ALT_M)
        for d in slave_drones.values():
            d.airborne = True; d.alt = CRUISE_ALT_M
        ok = all(r.get("status") == "ok" for r in results.values())
        log.info(f"  -> TAKEOFF slaves: {'OK' if ok else 'FAILED'}")
        time.sleep(0.3)
        r = master_client.send({"cmd": "TAKEOFF", "alt": CRUISE_ALT_M})
        master_drone.airborne = True; master_drone.alt = CRUISE_ALT_M
        log.info(f"  -> TAKEOFF master: {'OK' if r.get('status')=='ok' else 'FAILED'}")

    elif cmd == "pause":
        results = broadcast(all_clients, "HOLD")
        ok = all(r.get("status") == "ok" for r in results.values())
        log.info(f"  -> HOLD all: {'OK' if ok else 'FAILED'}")

    elif cmd == "resume":
        log.info(f"  -> RESUME acknowledged (swarm continues)")

    elif cmd == "scan":
        log.info(f"  -> SCAN (master-only, not forwarded to slaves)")

    elif cmd == "forward":
        with concurrent.futures.ThreadPoolExecutor() as ex:
            futures = {}
            for did, client in all_clients.items():
                d = all_drones[did]
                lat, lng = local_to_gps(d.x, d.y + 1.0)
                futures[did] = ex.submit(
                    client.send,
                    {"cmd": "GOTO", "lat": lat, "lng": lng,
                     "alt": d.alt or CRUISE_ALT_M}
                )
            results = {did: fut.result() for did, fut in futures.items()}
        for d in all_drones.values():
            d.y += 1.0
        ok = all(r.get("status") == "ok" for r in results.values())
        log.info(f"  -> FORWARD 1m all: {'OK' if ok else 'FAILED'}")

    elif cmd in ("land", "abort"):
        do_land(slave_clients, slave_drones, master_client, master_drone)

    else:
        log.warning(f"  -> Unhandled command from app.py: '{cmd}'")


def do_land(slave_clients, slave_drones, master_client, master_drone):
    broadcast(slave_clients, "LAND")
    for d in slave_drones.values():
        d.airborne = False; d.alt = 0.0; d.armed = False
    log.info("  -> LAND slaves: sent")
    time.sleep(0.3)
    master_client.send({"cmd": "LAND"})
    master_drone.airborne = False; master_drone.alt = 0.0; master_drone.armed = False
    log.info("  -> LAND master: sent")


# ════════════════════════════════════════════════════════════════════════════
#  MAIN LOOP
# ════════════════════════════════════════════════════════════════════════════

def run_voice_test(app_url, slave_clients, slave_drones, master_client, master_drone):
    all_drones = {**slave_drones, "master": master_drone}

    sep("VOICE TEST RUNNING — waiting for commands from app.py")
    log.info(f"  Polling: {app_url}/api/pending_commands")
    log.info(f"  Poll interval: {POLL_INTERVAL_S}s")
    log.info(f"  Auto-land if silent for: {NO_CMD_TIMEOUT}s")
    log.info(f"")
    log.info(f"  -> Speak into the phone browser or use app.py command buttons")
    log.info(f"  -> Commands: arm / start / pause / resume / forward / land")
    log.info(f"")

    last_cmd_time = time.time()
    total_commands = 0
    landed = False
    last_countdown_print = -1

    try:
        while True:
            time.sleep(POLL_INTERVAL_S)

            silence = time.time() - last_cmd_time
            remaining = int(NO_CMD_TIMEOUT - silence)

            # Auto-land on timeout
            if silence >= NO_CMD_TIMEOUT:
                log.warning(
                    f"\n  TIMEOUT — no command for {NO_CMD_TIMEOUT:.0f}s "
                    f"-> AUTO-LAND"
                )
                do_land(slave_clients, slave_drones, master_client, master_drone)
                print_positions(all_drones)
                landed = True
                break

            # Print countdown once per second (only when waiting)
            if remaining != last_countdown_print and total_commands == 0:
                log.info(f"  Waiting for first command... "
                         f"({remaining}s until auto-land)")
                last_countdown_print = remaining
            elif remaining != last_countdown_print and total_commands > 0:
                log.info(f"  Ready for next command "
                         f"({remaining}s silence so far)")
                last_countdown_print = remaining

            # Poll app.py
            commands = fetch_pending_commands(app_url)
            if not commands:
                continue

            # Got at least one command — reset the timeout
            last_cmd_time = time.time()
            last_countdown_print = -1

            for cmd in commands:
                total_commands += 1
                log.info(f"\n  [CMD #{total_commands}] '{cmd}' received from app.py")

                execute_command(
                    cmd,
                    slave_clients, slave_drones,
                    master_client, master_drone,
                )
                print_positions(all_drones)

                if cmd.lower() in ("land", "abort"):
                    landed = True
                    break

            if landed:
                break

    except KeyboardInterrupt:
        log.warning("\n  Ctrl+C pressed -> emergency land")
        do_land(slave_clients, slave_drones, master_client, master_drone)
        landed = True

    sep(f"VOICE TEST ENDED — {total_commands} command(s) processed")
    if landed:
        log.info("  Drones landed safely")
    print_positions(all_drones)


# ════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="Voice Command Test — driven by app.py")
    parser.add_argument("--real",    action="store_true",
                        help="Connect to real slave drones over TCP")
    parser.add_argument("--verbose", action="store_true",
                        help="Show debug output")
    parser.add_argument("--app-url", type=str, default=DEFAULT_APP_URL,
                        help=f"app.py base URL (default: {DEFAULT_APP_URL})")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="[%(asctime)s] %(message)s",
        datefmt="%H:%M:%S",
        stream=sys.stdout,
    )

    if not _REQUESTS_OK:
        log.error("Missing: pip install requests")
        sys.exit(1)

    if args.real and not _REAL_IMPORTS:
        log.error("--real requires swarm_state.py and tcp_channel.py alongside this file")
        sys.exit(1)

    try:
        import urllib3
        urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)
    except Exception:
        pass

    if args.real:
        log.info("Mode: REAL — connecting to slave drones over TCP")
        slave_clients = {}
        slave_drones  = {}
        for did, (host, port) in SLAVE_ADDRS.items():
            client = TCPCommandClient(did, host, port)
            client.connect_loop()
            slave_clients[did] = client
            slave_drones[did]  = STATE.drones[did]

        class LocalMasterClient:
            def send(self, payload):
                log.info(f"    [master] <- {json.dumps(payload)}")
                return {"status": "ok"}

        master_client = LocalMasterClient()
        master_drone  = STATE.drones["master"]
        log.info("TCP connections ready")
        log.info("Waiting 3s for connections to establish...")
        time.sleep(3.0)
        for did, client in slave_clients.items():
            connected = client._connected_event.is_set()
            log.info(f"  {did}: {'connected ✓' if connected else 'NOT CONNECTED ✗'}")
            if not connected:
                log.error(f"Could not connect to {did} — is slave.py running on that Pi?")
                sys.exit(1)

    else:
        log.info("Mode: STUB — drone commands simulated, app.py commands are real")
        slave_drones  = {did: StubDrone(did) for did in SLAVE_ADDRS}
        slave_clients = {did: StubClient(d) for did, d in slave_drones.items()}
        master_drone  = StubDrone("master")
        master_client = StubClient(master_drone)

    run_voice_test(
        app_url       = args.app_url,
        slave_clients = slave_clients,
        slave_drones  = slave_drones,
        master_client = master_client,
        master_drone  = master_drone,
    )


if __name__ == "__main__":
    main()
