#!/usr/bin/env python3
"""
launch.py  —  S.A.F.E. Swarm System Launcher
==============================================

Pre-flight checklist → spin up 3 threads:

  Thread 1 — FLIGHT CONTROL
      run_master()  — MAVSDK offboard arming, takeoff, waypoint following.
      Also runs receive_telemetry_loop() in the same asyncio event loop
      (it feeds the shared `slaves` dict that Thread 2 reads).

  Thread 2 — SWARM INTELLIGENCE
      Tight 50 ms loop calling:
        • evaluate_swarm_state()  — collision avoidance, comms-tether,
                                    anomaly detection → UDP CMD to slaves
        • plan_path_chunk()       — incremental A* path re-planning
        • master_ui_loop() render — OpenCV map
      All three run in their own asyncio event loop on a dedicated OS thread.

  Thread 3 — WEB / VOICE SERVER
      Two sub-threads inside this thread:
        3a.  app.py  Flask HTTPS ground server  (port 443)
             Serves the map UI, receives phone voice commands, mine/position
             updates from master, safe-path queries from ground operators.
        3b.  _run_voice_server()  lightweight HTTP server  (port 9000)
             Receives voice intents from the phone and sets mission_flags.

Pre-flight checks:
  1. Python >= 3.10
  2. Required packages  (flask, mavsdk, cv2, numpy)
  3. Optional packages  (vosk, sounddevice)  — Pi-mic; warns only
  4. SSL certs          (cert.pem / key.pem) — needed by Flask HTTPS
  5. Vosk model         — warns only
  6. index.html         — served by Flask
  7. UDP ports free     (8000 master, 8001-8003 slaves)
  8. TCP ports free     (443 Flask, 9000 voice-HTTP)
  9. MAVSDK gRPC port   (50050) — warns only, master.py retries

Usage:
  python launch.py                # full pre-flight + 3 threads
  python launch.py --skip-checks  # dev / SITL mode — skip checks
"""

import argparse
import asyncio
import os
import signal
import socket
import sys
import threading
import time
from pathlib import Path

# ══════════════════════════════════════════════════════════════════════════════
#  ANSI colours
# ══════════════════════════════════════════════════════════════════════════════
GREEN  = "\033[92m"
YELLOW = "\033[93m"
RED    = "\033[91m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

def ok(msg):     print(f"  {GREEN}OK{RESET}  {msg}")
def warn(msg):   print(f"  {YELLOW}WARN{RESET}  {msg}")
def fail(msg):   print(f"  {RED}FAIL{RESET}  {msg}")
def info(msg):   print(f"  {CYAN}-->{RESET}  {msg}")
def header(msg): print(f"\n{BOLD}{CYAN}{msg}{RESET}")
def banner(msg): print(f"\n{BOLD}{msg}{RESET}")

# ══════════════════════════════════════════════════════════════════════════════
#  CONFIG  (mirrors master.py constants)
# ══════════════════════════════════════════════════════════════════════════════
REQUIRED_PYTHON   = (3, 10)
REQUIRED_PACKAGES = [
    ("flask",  "flask"),
    ("mavsdk", "mavsdk"),
    ("cv2",    "opencv-python"),
    ("numpy",  "numpy"),
]
OPTIONAL_PACKAGES = [
    ("vosk",        "vosk"),
    ("sounddevice", "sounddevice"),
]

SSL_CERT    = Path("cert.pem")
SSL_KEY     = Path("key.pem")
VOSK_DIR    = Path(os.environ.get("VOSK_MODEL", "vosk-model-small-en-us-0.15"))
INDEX_HTML  = Path("index.html")

UDP_PORTS      = [8000, 8001, 8002, 8003]
TCP_PORTS      = [443, 9000]
MAVSDK_HOST    = "127.0.0.1"
MAVSDK_GRPC    = 50050
MAVSDK_TIMEOUT = 2

# ══════════════════════════════════════════════════════════════════════════════
#  PRE-FLIGHT CHECKS
# ══════════════════════════════════════════════════════════════════════════════

def check_python() -> bool:
    v = sys.version_info
    label = f"Python {v.major}.{v.minor}.{v.micro}"
    if (v.major, v.minor) >= REQUIRED_PYTHON:
        ok(f"{label}  (>= {'.'.join(map(str, REQUIRED_PYTHON))})")
        return True
    fail(f"{label}  — need >= {'.'.join(map(str, REQUIRED_PYTHON))}")
    return False


def check_packages() -> bool:
    passed = True
    for import_name, pip_name in REQUIRED_PACKAGES:
        try:
            __import__(import_name)
            ok(f"Package '{import_name}'")
        except ImportError:
            fail(f"Package '{import_name}' missing  -->  pip install {pip_name}")
            passed = False
    for import_name, pip_name in OPTIONAL_PACKAGES:
        try:
            __import__(import_name)
            ok(f"Package '{import_name}'  (optional)")
        except ImportError:
            warn(f"'{import_name}' not found — Pi-mic disabled  "
                 f"(pip install {pip_name} to enable)")
    return passed


def check_ssl() -> bool:
    if SSL_CERT.exists() and SSL_KEY.exists():
        ok(f"SSL certs  ({SSL_CERT}, {SSL_KEY})")
        return True
    fail(
        f"SSL certs missing ({SSL_CERT} / {SSL_KEY})\n"
        "    Run:  openssl req -x509 -newkey rsa:2048 -keyout key.pem "
        "-out cert.pem -days 365 -nodes -subj \"/CN=localhost\""
    )
    return False


def check_vosk_model() -> bool:
    if VOSK_DIR.is_dir():
        ok(f"Vosk model  ({VOSK_DIR})")
    else:
        warn(f"Vosk model not found at '{VOSK_DIR}' — Pi-mic disabled")
    return True   # non-fatal


def check_index_html() -> bool:
    if INDEX_HTML.exists():
        ok(f"index.html  ({INDEX_HTML.resolve()})")
        return True
    fail(f"index.html not found in {Path.cwd()}")
    return False


def _port_free(port: int, kind: str = "tcp") -> bool:
    family = socket.SOCK_DGRAM if kind == "udp" else socket.SOCK_STREAM
    with socket.socket(socket.AF_INET, family) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("0.0.0.0", port))
            return True
        except OSError:
            return False


def check_ports() -> bool:
    passed = True
    for p in UDP_PORTS:
        if _port_free(p, "udp"):
            ok(f"UDP {p}  free")
        else:
            fail(f"UDP {p}  already in use")
            passed = False
    for p in TCP_PORTS:
        if _port_free(p, "tcp"):
            ok(f"TCP {p}  free")
        else:
            if p == 443:
                fail(f"TCP {p}  already in use — another web server running?")
                passed = False
            else:
                warn(f"TCP {p}  already in use — voice HTTP may fail to bind")
    return passed


def check_mavsdk() -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(MAVSDK_TIMEOUT)
        try:
            s.connect((MAVSDK_HOST, MAVSDK_GRPC))
            ok(f"MAVSDK gRPC port {MAVSDK_GRPC}  reachable")
        except (ConnectionRefusedError, OSError):
            warn(
                f"MAVSDK gRPC port {MAVSDK_GRPC}  not responding\n"
                "    Simulation: start mavsdk_server or ArduCopter SITL first\n"
                "    Hardware:   ensure flight-controller is connected"
            )
    return True   # non-fatal — master.py retries internally


def run_preflight() -> bool:
    header("==============================================")
    print(  "   S.A.F.E.  PRE-FLIGHT CHECKLIST           ")
    print(  "==============================================")

    steps = [
        ("[ 1 / 7 ]  Python runtime",           check_python),
        ("[ 2 / 7 ]  Python packages",           check_packages),
        ("[ 3 / 7 ]  SSL certificates",          check_ssl),
        ("[ 4 / 7 ]  Vosk model  (optional)",    check_vosk_model),
        ("[ 5 / 7 ]  index.html",                check_index_html),
        ("[ 6 / 7 ]  Network ports",             check_ports),
        ("[ 7 / 7 ]  MAVSDK flight controller",  check_mavsdk),
    ]
    results = []
    for label, fn in steps:
        header(label)
        results.append(fn())

    print()
    if all(results):
        print(f"{BOLD}{GREEN}  ALL SYSTEMS GO — cleared for launch{RESET}\n")
        return True
    print(f"{BOLD}{RED}  PRE-FLIGHT FAILED — fix errors above before launching.{RESET}\n")
    return False


# ══════════════════════════════════════════════════════════════════════════════
#  SHARED STOP EVENT
# ══════════════════════════════════════════════════════════════════════════════
_stop_event = threading.Event()


# ══════════════════════════════════════════════════════════════════════════════
#  THREAD 1 — FLIGHT CONTROL
#  Coroutines: receive_telemetry_loop() + run_master()
#  Own asyncio event loop on a dedicated OS thread.
# ══════════════════════════════════════════════════════════════════════════════
def thread_flight_control():
    """
    Owns the master drone's MAVSDK connection.

    receive_telemetry_loop()  — 100 Hz UDP drain; keeps slaves{} up to date
                                so Thread 2 always has fresh positions.
    run_master()              — waits for voice 'start', then arms, takes off,
                                and follows waypoints in offboard mode.
    """
    try:
        from master import receive_telemetry_loop, run_master
    except ImportError as e:
        fail(f"[T1-FLIGHT] Cannot import from master.py: {e}")
        _stop_event.set()
        return

    async def _run():
        print("[T1-FLIGHT] Connecting to MAVSDK flight controller ...")
        await asyncio.gather(
            receive_telemetry_loop(),   # keeps slaves{} fresh for Thread 2
            run_master(),               # arming / offboard waypoint follower
        )

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(_run())
    except Exception as e:
        fail(f"[T1-FLIGHT] Crashed: {e}")
        _stop_event.set()
    finally:
        loop.close()


# ══════════════════════════════════════════════════════════════════════════════
#  THREAD 2 — SWARM INTELLIGENCE
#  Coroutine: master_ui_loop()
#    Inside that 50 ms tick:
#      evaluate_swarm_state() — collision avoidance, comms-tether, anomaly
#      plan_path_chunk()      — incremental A* whenever swarm advances
#      cv2 map render         — OpenCV live-map window
# ══════════════════════════════════════════════════════════════════════════════
def thread_swarm_intelligence():
    """
    Runs the 50 ms swarm logic tick on its own event loop.

    master_ui_loop() internally calls:
      evaluate_swarm_state()
        - Collision avoidance: if two drones are within COLLISION_DIST_M,
          the rearward one gets CMD_HOLD.
        - Comms tether: any slave further than COMMS_TETHER_M from master
          gets CMD_HOLD until master catches up.
        - Anomaly handling: slave reports mine -> CMD_DESCEND or CMD_IGNORE
          (redundancy check), mine coordinates appended to known_mines.

      plan_path_chunk()
        - Every CHUNK_SIZE metres of swarm forward progress, re-runs A*
          to produce a mine-safe waypoint list for master_path.

      OpenCV map render
        - Live top-down view: drones, mines, planned path, comms-tether ring.
    """
    try:
        from master import master_ui_loop
    except ImportError as e:
        fail(f"[T2-SWARM] Cannot import from master.py: {e}")
        _stop_event.set()
        return

    async def _run():
        print("[T2-SWARM] Swarm intelligence loop starting ...")
        await master_ui_loop()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(_run())
    except Exception as e:
        fail(f"[T2-SWARM] Crashed: {e}")
        _stop_event.set()
    finally:
        loop.close()


# ══════════════════════════════════════════════════════════════════════════════
#  THREAD 3 — WEB / VOICE SERVER
#  Sub-thread 3a: Flask HTTPS ground server (app.py, port 443)
#  Sub-thread 3b: Voice command HTTP server (master.py port 9000)
# ══════════════════════════════════════════════════════════════════════════════
def thread_web_server():
    """
    Supervisor that owns both web-facing servers.

    Sub-thread 3a — Flask HTTPS (app.py, port 443)
        Serves the operator map UI.
        POST /api/drone_update    <- mine positions and drone telemetry
        POST /api/voice_command   <- phone Web Speech API intents
        GET  /api/status          <- full JSON status for phone UI
        GET  /api/pending_commands<- polled by master.py for queued intents
        Vosk Pi-mic daemon starts automatically inside app.py.

    Sub-thread 3b — Voice HTTP (master.py _run_voice_server, port 9000)
        Minimal HTTP POST /command endpoint.
        Directly flips master.py mission_flags (start/pause/resume/land/scan).
        Backup voice path if phone connects to master Pi WiFi hotspot.
    """

    def _flask_sub():
        try:
            import app as ground_server
        except ImportError as e:
            fail(f"[T3a-FLASK] Cannot import app.py: {e}")
            _stop_event.set()
            return
        print("[T3a-FLASK] Starting Flask HTTPS server on :443 ...")
        # Replicate app.py __main__ block: start Vosk daemon then Flask.
        vosk_t = threading.Thread(
            target=ground_server._vosk_thread, daemon=True, name="vosk-mic"
        )
        vosk_t.start()
        ground_server.app.run(
            host="0.0.0.0",
            port=443,
            ssl_context=("cert.pem", "key.pem"),
            debug=False,
            use_reloader=False,
        )

    def _voice_sub():
        try:
            from master import _run_voice_server
        except ImportError as e:
            fail(f"[T3b-VOICE] Cannot import from master.py: {e}")
            _stop_event.set()
            return
        print("[T3b-VOICE] Starting voice command HTTP server on :9000 ...")
        _run_voice_server()   # HTTPServer.serve_forever — blocks intentionally

    sub_threads = []
    for name, target in [("Flask-HTTPS", _flask_sub), ("Voice-HTTP", _voice_sub)]:
        t = threading.Thread(target=target, name=name, daemon=True)
        t.start()
        sub_threads.append(t)
        time.sleep(0.5)   # stagger so ports have time to bind

    # Watchdog for sub-threads
    while not _stop_event.is_set():
        for t in sub_threads:
            if not t.is_alive():
                warn(f"[T3-WEB] Sub-thread '{t.name}' stopped — check logs above.")
                _stop_event.set()
                break
        time.sleep(3)


# ══════════════════════════════════════════════════════════════════════════════
#  LAUNCH
# ══════════════════════════════════════════════════════════════════════════════
THREAD_DEFS = [
    ("T1-FlightControl",     thread_flight_control),
    ("T2-SwarmIntelligence", thread_swarm_intelligence),
    ("T3-WebServer",         thread_web_server),
]


def launch():
    def _sighandler(sig, frame):
        print(f"\n{YELLOW}[LAUNCH] Signal {sig} — requesting shutdown ...{RESET}")
        _stop_event.set()

    signal.signal(signal.SIGINT,  _sighandler)
    signal.signal(signal.SIGTERM, _sighandler)

    banner("==  LAUNCH SEQUENCE  ==================================================")

    live_threads: list[threading.Thread] = []
    for name, target in THREAD_DEFS:
        info(f"Starting {name} ...")
        t = threading.Thread(target=target, name=name, daemon=True)
        t.start()
        live_threads.append(t)
        time.sleep(0.8)   # stagger startup so logs stay readable

    print()
    print(f"{BOLD}{'─'*60}{RESET}")
    print(f"  {GREEN}T1 Flight Control    {RESET}  MAVSDK offboard + UDP telemetry recv")
    print(f"  {GREEN}T2 Swarm Intelligence{RESET}  Collision avoidance + A* path + OpenCV map")
    print(f"  {GREEN}T3 Web Server        {RESET}  Flask HTTPS :443  |  Voice HTTP :9000")
    print(f"{BOLD}{'─'*60}{RESET}")
    print(f"  Send voice 'start' from the phone app to begin the mission.")
    print(f"  Press  Ctrl-C  here to stop all threads.\n")

    # Main watchdog loop
    while not _stop_event.is_set():
        for t in live_threads:
            if not t.is_alive():
                fail(f"[LAUNCH] Thread '{t.name}' stopped unexpectedly!")
                _stop_event.set()
                break
        time.sleep(2)

    print(f"\n{YELLOW}[LAUNCH] Shutting down (daemon threads will exit with main) ...{RESET}")
    time.sleep(2)
    print(f"{YELLOW}[LAUNCH] Done. Goodbye.{RESET}")
    sys.exit(0)


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="S.A.F.E. swarm system launcher")
    parser.add_argument(
        "--skip-checks",
        action="store_true",
        help="Skip pre-flight checks (dev / SITL simulation mode)",
    )
    args = parser.parse_args()

    if args.skip_checks:
        warn("Pre-flight checks SKIPPED  (--skip-checks flag set)\n")
    else:
        if not run_preflight():
            sys.exit(1)

    launch()


if __name__ == "__main__":
    main()
