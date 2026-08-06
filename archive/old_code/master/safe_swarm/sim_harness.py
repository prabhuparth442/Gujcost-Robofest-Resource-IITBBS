#!/usr/bin/env python3
"""
sim_harness.py  —  S.A.F.E. Ground-Level Integration Test
==========================================================
Tests every communication path without touching motors, MAVSDK, or
the thermal sensor. Run this entirely on your laptop or the master Pi.

What it simulates
-----------------
  SlaveSimulator × 3   — pretends to be drone1/2/3
    • Sends real JSON UDP telemetry to master port 14550
    • Opens TCP command server on port 14560 (receives GOTO/PAUSE/LAND)
    • Sends a fake mine report via DroneTunnel TCP to master port 5000
    • Logs every command it receives from master

  MasterSimulator      — pretends to be the master swarm backend
    • Starts udp_telemetry.start_udp_server() (real code, not a mock)
    • Starts tcp_commander.start_tcp_commander() (real code)
    • Verifies STATE gets populated with drone positions
    • Verifies mine reaches STATE and app.py

  AppSimulator         — pretends to be app.py
    • Minimal Flask server that accepts /api/drone_update and /api/pending_commands
    • Logs every mine_detected and drone_position it receives

How to run
----------
  # On master Pi (or your laptop with everything in same directory):
  pip install flask aiohttp
  python3 sim_harness.py

  # Expected output — every line should say PASS:
  [SIM] UDP telemetry path       ... PASS
  [SIM] TCP command path         ... PASS
  [SIM] Mine TCP path            ... PASS
  [SIM] app.py mine forwarding   ... PASS
  [SIM] Voice bridge             ... PASS
  [SIM] STATE population         ... PASS

  Total: 6/6 PASS

Press Ctrl+C to stop early.
"""

import asyncio
import json
import socket
import struct
import sys
import threading
import time
import traceback
from pathlib import Path
from typing import Optional

# ─────────────────────────────────────────────────────────────────────────────
#  PORTS  — must match your real config
# ─────────────────────────────────────────────────────────────────────────────
MASTER_IP        = "127.0.0.1"   # run everything locally
UDP_TELEM_PORT   = 14550
TCP_CMD_PORT     = 14560
MINE_TCP_PORT    = 5000
APP_HTTP_PORT    = 8080           # use 8080 to avoid needing sudo for 443

# Fake GPS origin — Bhubaneswar competition field area
ORIGIN_LAT = 20.296000
ORIGIN_LON = 85.824000

# ─────────────────────────────────────────────────────────────────────────────
#  SIMPLE LOGGER
# ─────────────────────────────────────────────────────────────────────────────
def log(component, status, msg):
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}][{component}][{status}] {msg}", flush=True)


# ─────────────────────────────────────────────────────────────────────────────
#  SHARED RESULTS  — each test writes True/False here
# ─────────────────────────────────────────────────────────────────────────────
results = {
    "UDP telemetry path":    None,
    "TCP command path":      None,
    "Mine TCP path":         None,
    "app.py mine forward":   None,
    "Voice bridge":          None,
    "STATE population":      None,
}

# Shared state for cross-thread tracking
received_udp_from    = set()   # set of drone_ids that sent UDP
received_tcp_cmds    = {}      # drone_id → list of commands received
mine_in_state        = []      # mines added to simulated STATE
mine_in_app          = []      # mines received by fake app.py
voice_cmds_received  = []      # commands app.py served to voice bridge


# ─────────────────────────────────────────────────────────────────────────────
#  FAKE app.py  (Flask replacement — minimal HTTP server)
# ─────────────────────────────────────────────────────────────────────────────
def run_fake_app():
    """
    Minimal HTTP server on APP_HTTP_PORT that mimics the two app.py endpoints
    used by the swarm:
      POST /api/drone_update  — receives telemetry + mine events
      GET  /api/pending_commands — returns queued voice commands
    """
    from http.server import BaseHTTPRequestHandler, HTTPServer

    pending_voice = ["start"]   # pre-load a START command so voice bridge test works

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass   # silence default access log

        def do_POST(self):
            length = int(self.headers.get("Content-Length", 0))
            body   = json.loads(self.rfile.read(length))
            t      = body.get("type")

            if t == "mine_detected":
                mine_in_app.append(body)
                log("APP", "OK",
                    f"mine_detected received: ({body.get('lat'):.6f},{body.get('lng'):.6f}) "
                    f"from {body.get('drone_id')}")

            elif t == "drone_position":
                did = body.get("drone_id", "?")
                log("APP", "INFO",
                    f"drone_position from {did}: "
                    f"({body.get('lat',0):.5f},{body.get('lng',0):.5f})")

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"status":"ok"}')

        def do_GET(self):
            if self.path == "/api/pending_commands":
                cmds = pending_voice.copy()
                pending_voice.clear()
                voice_cmds_received.extend(cmds)
                payload = json.dumps({"commands": cmds}).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
                if cmds:
                    log("APP", "OK", f"Served voice commands: {cmds}")
            else:
                self.send_response(404)
                self.end_headers()

    server = HTTPServer(("0.0.0.0", APP_HTTP_PORT), Handler)
    log("APP", "OK", f"Fake app.py running on port {APP_HTTP_PORT}")
    server.serve_forever()


# ─────────────────────────────────────────────────────────────────────────────
#  FAKE SWARM STATE  (replaces swarm_state.STATE for simulation)
# ─────────────────────────────────────────────────────────────────────────────
class FakeState:
    """Minimal stand-in for swarm_state.SwarmState — no MAVSDK needed."""

    def __init__(self):
        self.lock    = asyncio.Lock()
        self.drones  = {}
        self.mines   = []
        self._mine_id = 0

    async def update_drone(self, pkt):
        did = pkt.get("drone_id", "unknown")
        async with self.lock:
            self.drones[did] = {
                "lat":      pkt.get("lat", 0),
                "lng":      pkt.get("lng", 0),
                "altitude": pkt.get("altitude", 0),
                "heading":  pkt.get("heading", 0),
                "armed":    pkt.get("armed", False),
                "airborne": pkt.get("airborne", False),
            }
        received_udp_from.add(did)
        log("STATE", "INFO",
            f"update_drone: {did} at ({pkt.get('lat',0):.5f},{pkt.get('lng',0):.5f})")

    async def add_mine(self, lat, lng, detected_by="?"):
        async with self.lock:
            record = {"mine_id": self._mine_id, "lat": lat, "lng": lng,
                      "detected_by": detected_by}
            self.mines.append(record)
            mine_in_state.append(record)
            self._mine_id += 1
        log("STATE", "OK",
            f"add_mine #{record['mine_id']}: ({lat:.6f},{lng:.6f}) by {detected_by}")
        return type("MineRecord", (), record)()   # duck-type with mine_id attr

    def snapshot(self):
        return {"drones": self.drones, "mines": self.mines}


# ─────────────────────────────────────────────────────────────────────────────
#  SLAVE SIMULATOR
# ─────────────────────────────────────────────────────────────────────────────
class SlaveSimulator:
    """
    Simulates one slave drone — sends UDP telemetry and listens for TCP commands.
    Does NOT start MAVSDK or touch any hardware.
    """

    def __init__(self, drone_id: str, strip_y: float):
        self.drone_id = drone_id
        self.strip_y  = strip_y
        self.lat      = ORIGIN_LAT + (strip_y / 111_320)
        self.lon      = ORIGIN_LON
        self._seq     = 0
        self._commands_received = []
        received_tcp_cmds[drone_id] = self._commands_received

    # ── UDP telemetry sender ─────────────────────────────────────────────────
    def _send_udp_packet(self, sensor=0.0):
        pkt = json.dumps({
            "drone_id": self.drone_id,
            "seq":      self._seq,
            "lat":      round(self.lat, 7),
            "lng":      round(self.lon, 7),
            "altitude": 1.5,
            "heading":  0.0,
            "speed":    0.5,
            "armed":    True,
            "airborne": True,
            "bat_pct":  85,
            "sensor":   sensor,
        }).encode()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(pkt, (MASTER_IP, UDP_TELEM_PORT))
        sock.close()
        self._seq += 1

    def send_telemetry(self, count=3, interval=0.25):
        """Send `count` telemetry packets to master."""
        log("SLAVE", "INFO",
            f"{self.drone_id}: sending {count} UDP packets → {MASTER_IP}:{UDP_TELEM_PORT}")
        for i in range(count):
            self._send_udp_packet(sensor=0.0)
            time.sleep(interval)
        log("SLAVE", "OK", f"{self.drone_id}: UDP telemetry sent")

    def send_mine_spike(self, count=6):
        """Send sensor=0.9 for `count` packets to trigger mine detection via UDP."""
        log("SLAVE", "INFO",
            f"{self.drone_id}: sending mine sensor spike ({count} packets)")
        for _ in range(count):
            self._send_udp_packet(sensor=0.9)
            time.sleep(0.1)
        log("SLAVE", "OK", f"{self.drone_id}: mine spike sent")

    # ── DroneTunnel mine report (TCP, 4-byte prefix) ─────────────────────────
    def send_drone_tunnel_mine(self, lat, lon):
        """
        Send an anomaly_report exactly as 08_comms_link.DroneTunnel does —
        4-byte big-endian length prefix + JSON body.
        """
        pkt = json.dumps({
            "type":      "anomaly_report",
            "drone_id":  self.drone_id,
            "latitude":  lat,
            "longitude": lon,
            "payload":   "simulated_base64_image_data",
        }).encode("utf-8")

        log("SLAVE", "INFO",
            f"{self.drone_id}: sending DroneTunnel mine → {MASTER_IP}:{MINE_TCP_PORT} "
            f"({lat:.6f},{lon:.6f})")
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5.0)
            s.connect((MASTER_IP, MINE_TCP_PORT))
            s.sendall(len(pkt).to_bytes(4, "big"))
            s.sendall(pkt)
            s.close()
            log("SLAVE", "OK", f"{self.drone_id}: DroneTunnel mine sent")
        except Exception as e:
            log("SLAVE", "ERROR", f"{self.drone_id}: DroneTunnel send failed: {e}")

    # ── TCP command server ───────────────────────────────────────────────────
    def run_tcp_server(self):
        """
        Listen on TCP_CMD_PORT for commands from master.
        Logs every command received and sends a simple ACK.
        """
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Each slave needs a distinct port — offset by slave number
        offset = {"slave_1": 0, "slave_2": 1, "slave_3": 2}.get(self.drone_id, 0)
        port   = TCP_CMD_PORT + offset
        srv.bind(("0.0.0.0", port))
        srv.listen(1)
        srv.settimeout(30.0)
        log("SLAVE", "OK", f"{self.drone_id}: TCP command server on port {port}")

        try:
            conn, addr = srv.accept()
            log("SLAVE", "INFO", f"{self.drone_id}: master connected from {addr}")
            buf = bytearray()
            conn.settimeout(10.0)
            while True:
                try:
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    buf.extend(chunk)
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        try:
                            cmd = json.loads(line.decode())
                            cmd_name = cmd.get("cmd", "?")
                            seq      = cmd.get("seq", "?")
                            self._commands_received.append(cmd_name)
                            log("SLAVE", "OK",
                                f"{self.drone_id}: received cmd={cmd_name} seq={seq}")
                            ack = json.dumps({
                                "ack":    cmd_name,
                                "seq":    seq,
                                "status": "ok",
                            }) + "\n"
                            conn.sendall(ack.encode())
                        except json.JSONDecodeError:
                            pass
                except socket.timeout:
                    break
            conn.close()
        except socket.timeout:
            log("SLAVE", "WARN", f"{self.drone_id}: no master connection within 30s")
        finally:
            srv.close()


# ─────────────────────────────────────────────────────────────────────────────
#  TEST RUNNER
# ─────────────────────────────────────────────────────────────────────────────
async def run_tests(fake_state: FakeState):
    """
    Runs all integration tests sequentially.
    Each test waits for an observable side effect then records PASS/FAIL.
    """
    log("TEST", "INFO", "=" * 56)
    log("TEST", "INFO", "Starting integration tests")
    log("TEST", "INFO", "=" * 56)

    # ── Fake the GROUND_SERVER URL that udp_telemetry._post_to_ground uses ───
    import udp_telemetry as udt
    udt.GROUND_SERVER = f"http://127.0.0.1:{APP_HTTP_PORT}"
    udt.STATE = fake_state   # type: ignore
    udt.UDP_PORT = UDP_TELEM_PORT

    # ── Test 1: UDP telemetry ─────────────────────────────────────────────────
    log("TEST", "INFO", "[1/6] UDP telemetry path")
    log("TEST", "INFO", "  Waiting for slave simulators to send packets...")
    deadline = time.time() + 8.0
    while time.time() < deadline:
        if len(received_udp_from) >= 3:
            break
        await asyncio.sleep(0.2)

    if len(received_udp_from) >= 3:
        results["UDP telemetry path"] = True
        log("TEST", "OK",
            f"  PASS — master received UDP from: {sorted(received_udp_from)}")
    else:
        results["UDP telemetry path"] = False
        log("TEST", "ERROR",
            f"  FAIL — only received from: {sorted(received_udp_from)} (expected 3)")

    # ── Test 2: STATE population ──────────────────────────────────────────────
    log("TEST", "INFO", "[2/6] STATE population")
    snap = fake_state.snapshot()
    if len(snap["drones"]) >= 3:
        results["STATE population"] = True
        log("TEST", "OK",
            f"  PASS — {len(snap['drones'])} drones in STATE: {list(snap['drones'].keys())}")
    else:
        results["STATE population"] = False
        log("TEST", "ERROR",
            f"  FAIL — only {len(snap['drones'])} drones in STATE: {list(snap['drones'].keys())}")

    # ── Test 3: Mine TCP path (DroneTunnel anomaly_report) ────────────────────
    log("TEST", "INFO", "[3/6] Mine TCP path (DroneTunnel)")
    log("TEST", "INFO", "  Slave 1 sending fake mine via DroneTunnel...")
    mine_lat = ORIGIN_LAT + 0.00010
    mine_lon = ORIGIN_LON + 0.00010

    def _send_mine():
        time.sleep(0.5)
        SlaveSimulator("slave_1", -4.45).send_drone_tunnel_mine(mine_lat, mine_lon)

    threading.Thread(target=_send_mine, daemon=True).start()

    deadline = time.time() + 8.0
    while time.time() < deadline:
        if any(abs(m["lat"] - mine_lat) < 0.0001 for m in mine_in_state):
            break
        await asyncio.sleep(0.2)

    if any(abs(m["lat"] - mine_lat) < 0.0001 for m in mine_in_state):
        results["Mine TCP path"] = True
        log("TEST", "OK", f"  PASS — mine reached STATE: {mine_in_state[-1]}")
    else:
        results["Mine TCP path"] = False
        log("TEST", "ERROR",
            f"  FAIL — mine NOT in STATE after 8s. "
            f"STATE.mines={mine_in_state}")

    # ── Test 4: app.py mine forwarding ────────────────────────────────────────
    log("TEST", "INFO", "[4/6] app.py mine forwarding")
    deadline = time.time() + 5.0
    while time.time() < deadline:
        if any(abs(m.get("lat", 0) - mine_lat) < 0.0001 for m in mine_in_app):
            break
        await asyncio.sleep(0.2)

    if any(abs(m.get("lat", 0) - mine_lat) < 0.0001 for m in mine_in_app):
        results["app.py mine forward"] = True
        log("TEST", "OK",
            f"  PASS — mine forwarded to app.py: {mine_in_app[-1]}")
    else:
        results["app.py mine forward"] = False
        log("TEST", "ERROR",
            f"  FAIL — mine NOT in app.py after 5s. "
            f"Check GROUND_SERVER URL and that fake app is running.")

    # ── Test 5: TCP command path ──────────────────────────────────────────────
    log("TEST", "INFO", "[5/6] TCP command path (master → slave)")
    log("TEST", "INFO", "  Injecting PAUSE command into STATE.cmd_queues...")

    import tcp_commander as tcpc
    tcpc.STATE = fake_state  # type: ignore

    # Push a PAUSE command into slave_1's queue
    try:
        await fake_state_cmd_queues["slave_1"].put({"cmd": "PAUSE"})
        deadline = time.time() + 6.0
        while time.time() < deadline:
            if "PAUSE" in received_tcp_cmds.get("slave_1", []):
                break
            await asyncio.sleep(0.2)

        if "PAUSE" in received_tcp_cmds.get("slave_1", []):
            results["TCP command path"] = True
            log("TEST", "OK",
                f"  PASS — slave_1 received commands: {received_tcp_cmds['slave_1']}")
        else:
            results["TCP command path"] = False
            log("TEST", "ERROR",
                f"  FAIL — slave_1 did not receive PAUSE. "
                f"Received: {received_tcp_cmds.get('slave_1', [])}")
    except Exception as e:
        results["TCP command path"] = False
        log("TEST", "ERROR", f"  FAIL — exception: {e}")

    # ── Test 6: Voice bridge ──────────────────────────────────────────────────
    log("TEST", "INFO", "[6/6] Voice bridge (app.py → master)")
    log("TEST", "INFO", "  Waiting for voice bridge to poll and receive 'start' cmd...")
    deadline = time.time() + 6.0
    while time.time() < deadline:
        if voice_cmds_received:
            break
        await asyncio.sleep(0.2)

    if voice_cmds_received:
        results["Voice bridge"] = True
        log("TEST", "OK",
            f"  PASS — voice bridge consumed commands: {voice_cmds_received}")
    else:
        results["Voice bridge"] = False
        log("TEST", "ERROR",
            f"  FAIL — voice bridge never polled app.py or got no commands. "
            f"Check tcp_commander._voice_bridge is running.")

    # ── Print summary ─────────────────────────────────────────────────────────
    await asyncio.sleep(0.5)
    print("\n" + "=" * 56, flush=True)
    print("  SIMULATION RESULTS", flush=True)
    print("=" * 56, flush=True)
    passed = 0
    for name, ok in results.items():
        if ok is None:
            ok = False
        status = "PASS" if ok else "FAIL"
        passed += int(ok)
        print(f"  [{status}]  {name}", flush=True)
    print("=" * 56, flush=True)
    print(f"  Total: {passed}/{len(results)} PASS", flush=True)
    print("=" * 56, flush=True)

    if passed < len(results):
        print("\nDiagnosis tips:", flush=True)
        if not results.get("UDP telemetry path"):
            print("  • UDP FAIL: Check nothing is blocking port 14550. "
                  "Run: lsof -i :14550", flush=True)
        if not results.get("Mine TCP path"):
            print("  • Mine TCP FAIL: Check port 5000 is free. "
                  "Run: lsof -i :5000", flush=True)
        if not results.get("app.py mine forward"):
            print("  • app.py FAIL: Check fake app is on port 8080 and "
                  "GROUND_SERVER was patched correctly.", flush=True)
        if not results.get("TCP command path"):
            print("  • TCP cmd FAIL: Check slave simulator TCP server started "
                  "and port 14560 is free.", flush=True)
        if not results.get("Voice bridge"):
            print("  • Voice FAIL: Check tcp_commander._voice_bridge coroutine "
                  "is running and APP_HTTP_PORT matches.", flush=True)


# ─────────────────────────────────────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────
fake_state_cmd_queues = {}

async def main():
    print("=" * 56, flush=True)
    print("  S.A.F.E. INTEGRATION SIMULATION", flush=True)
    print(f"  Master IP  : {MASTER_IP}", flush=True)
    print(f"  UDP port   : {UDP_TELEM_PORT}", flush=True)
    print(f"  TCP cmd    : {TCP_CMD_PORT}", flush=True)
    print(f"  Mine TCP   : {MINE_TCP_PORT}", flush=True)
    print(f"  App HTTP   : {APP_HTTP_PORT}", flush=True)
    print("=" * 56, flush=True)

    # Check required ports are free before starting
    for port, name in [
        (UDP_TELEM_PORT, "UDP telem"),
        (MINE_TCP_PORT,  "Mine TCP"),
        (APP_HTTP_PORT,  "Fake app"),
        (TCP_CMD_PORT,   "TCP cmd slave_1"),
    ]:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("127.0.0.1", port))
            s.close()
        except OSError:
            log("INIT", "WARN",
                f"Port {port} ({name}) appears in use — test may fail. "
                f"Run: lsof -i :{port}")

    # Start fake app.py in background thread
    threading.Thread(target=run_fake_app, daemon=True).start()
    await asyncio.sleep(0.3)   # give Flask time to start

    # Build fake STATE with cmd queues
    fake_state = FakeState()
    for did in ["slave_1", "slave_2", "slave_3"]:
        q = asyncio.Queue()
        fake_state_cmd_queues[did] = q

    # Monkey-patch swarm modules to use fake STATE and local ports
    try:
        import udp_telemetry as udt
        import tcp_commander as tcpc
        udt.STATE         = fake_state
        udt.GROUND_SERVER = f"http://127.0.0.1:{APP_HTTP_PORT}"
        udt.UDP_PORT      = UDP_TELEM_PORT
        # Patch tcp_commander voice bridge URL to hit fake app
        import tcp_commander
        # Replace APP_URL inside the voice bridge by patching the module-level constant
        tcp_commander_src_path = Path(tcp_commander.__file__)
        log("INIT", "OK", f"tcp_commander loaded from: {tcp_commander_src_path}")

        # Patch slave addresses to use local ports
        tcpc.SLAVE_ADDRS = {
            "slave_1": (MASTER_IP, TCP_CMD_PORT + 0),
            "slave_2": (MASTER_IP, TCP_CMD_PORT + 1),
            "slave_3": (MASTER_IP, TCP_CMD_PORT + 2),
        }
        # Give tcp_commander the fake state cmd_queues
        import swarm_state
        swarm_state.STATE = fake_state
        swarm_state.STATE.cmd_queues = {k: v for k, v in fake_state_cmd_queues.items()}
        log("INIT", "OK", "Swarm modules patched with fake STATE")
    except ImportError as e:
        log("INIT", "WARN",
            f"Could not import swarm module: {e}. "
            "Run from the directory containing udp_telemetry.py and tcp_commander.py")

    # Start slave TCP servers in threads (one per slave)
    slaves = [
        SlaveSimulator("slave_1", -4.45),
        SlaveSimulator("slave_2",  0.00),
        SlaveSimulator("slave_3", +4.45),
    ]
    for slave in slaves:
        threading.Thread(
            target=slave.run_tcp_server, daemon=True,
            name=f"tcp_srv_{slave.drone_id}"
        ).start()

    await asyncio.sleep(0.5)   # give TCP servers time to bind

    # Start real udp_telemetry server
    try:
        import udp_telemetry as udt
        asyncio.create_task(udt.start_udp_server(), name="udp_server")
        log("INIT", "OK", "udp_telemetry.start_udp_server() started")
    except Exception as e:
        log("INIT", "ERROR", f"Could not start udp_telemetry: {e}")

    # Start real tcp_commander
    try:
        import tcp_commander as tcpc
        asyncio.create_task(
            tcpc.start_tcp_commander(tcpc.SLAVE_ADDRS),
            name="tcp_commander"
        )
        log("INIT", "OK", "tcp_commander.start_tcp_commander() started")
    except Exception as e:
        log("INIT", "ERROR", f"Could not start tcp_commander: {e}")

    await asyncio.sleep(1.0)   # let servers fully bind

    # Start slave UDP telemetry in threads
    for slave in slaves:
        threading.Thread(
            target=slave.send_telemetry,
            kwargs={"count": 10, "interval": 0.2},
            daemon=True,
            name=f"udp_{slave.drone_id}"
        ).start()

    # Run the test suite
    await run_tests(fake_state)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[SIM] Stopped by operator.", flush=True)
