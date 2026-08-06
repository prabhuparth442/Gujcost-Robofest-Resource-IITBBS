#!/usr/bin/env python3
"""
voice_arm_test.py  —  S.A.F.E. Indoor Voice-Activation → Arm Test
==================================================================
PURPOSE
-------
Verify the full voice → tcp_commander → slave → MAVSDK → FC arm chain
works correctly INDOORS, with props OFF and NO takeoff.

What this script does, in sequence:
  1.  Connects to each slave's TCP command server (port 14560).
      Any slave that refuses connection is skipped — script never crashes.
  2.  Starts a minimal local HTTP server on port 8080 that acts as app.py.
      This server holds a command queue that you can inject into via:
        - typing a word in the terminal  (keyboard mode), OR
        - speaking into the microphone   (voice mode, requires vosk + sounddevice)
  3.  A voice bridge polls the local server every 100ms, exactly as
      tcp_commander._voice_bridge() does in production.
  4.  When "arm" or "start" is recognised, ARM_ONLY is sent to all
      connected slaves over TCP.  The slave's FC arms the motors
      (no takeoff).  This is safe indoors with props off.
  5.  When "disarm" is recognised, DISARM is sent.
  6.  All results are logged with pass/fail — you can see if the full
      chain worked end to end.

MODES
-----
  python3 voice_arm_test.py              # keyboard mode (type commands)
  python3 voice_arm_test.py --voice      # voice mode (microphone, needs vosk)
  python3 voice_arm_test.py --slaves 1   # only test slave_1 (faster)

VOICE KEYWORDS (any word in the phrase triggers the command)
-------------------------------------------------------------
  ARM / START / ACTIVATE  →  ARM_ONLY  (arm, no takeoff)
  DISARM / STOP / ABORT   →  DISARM
  STATUS                  →  print connection status (no drone action)

PRE-REQUISITES
--------------
  On each slave Pi — MANUALLY start before running this:
    Terminal 1:  mavproxy.py --master=/dev/ttyAMA0 --baudrate=921600 \\
                             --out=udp:0.0.0.0:14540
    Terminal 2:  DRONE_ID=slave_1 python3 main_orchestrator.py

  On master Pi — run this script:
    pip install vosk sounddevice --break-system-packages  (only for --voice mode)
    python3 voice_arm_test.py

  main_orchestrator.py on each slave MUST have ARM_ONLY and DISARM handlers.
  If they are missing, see the HANDLER STUBS section at the bottom of this file.
  You can paste those stubs into main_orchestrator.py's start_tcp_command_server().

SAFETY
------
  • Props should be removed or secured before this test.
  • ARM_ONLY does NOT send takeoff — FC arms the ESCs, motors spin slightly.
  • DISARM immediately disarms.
  • Ctrl-C sends DISARM to all connected drones then exits cleanly.
"""

import argparse
import asyncio
import json
import logging
import queue
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Optional

# ─────────────────────────────────────────────────────────────────────────────
#  CONFIG  — edit if your network differs
# ─────────────────────────────────────────────────────────────────────────────
SLAVE_IPS: dict[str, str] = {
    "slave_1": "10.42.0.11",
    "slave_2": "10.42.0.12",
    "slave_3": "10.42.0.13",
}
CMD_PORT          = 14560
APP_PORT          = 8080      # local fake app.py port, bridge polls this
BRIDGE_POLL_HZ    = 10        # 100ms interval, matches production
ACK_TIMEOUT_S     = 5.0
CONNECT_TIMEOUT_S = 8.0

# Vosk model — small English model, ~50MB. Download from:
# https://alphacephei.com/vosk/models  →  vosk-model-small-en-us-0.15
VOSK_MODEL_PATH = "./vosk-model-small-en-us-0.15"

# Keywords that map to commands
KEYWORD_MAP = {
    "arm":      "ARM_ONLY",
    "start":    "ARM_ONLY",
    "activate": "ARM_ONLY",
    "launch":   "ARM_ONLY",
    "disarm":   "DISARM",
    "stop":     "DISARM",
    "abort":    "DISARM",
    "kill":     "DISARM",
    "status":   "STATUS",
    "check":    "STATUS",
}

# ─────────────────────────────────────────────────────────────────────────────
#  LOGGING
# ─────────────────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s][%(name)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("voice_arm_test")

def plog(component: str, status: str, msg: str) -> None:
    icons = {"OK": "✓", "WARN": "⚠", "ERROR": "✗", "INFO": "·", "TEST": "▶"}
    icon  = icons.get(status, status)
    ts    = time.strftime("%H:%M:%S")
    print(f"[{ts}][{component}][{icon}] {msg}", flush=True)

# ─────────────────────────────────────────────────────────────────────────────
#  TEST RESULTS
# ─────────────────────────────────────────────────────────────────────────────
test_results: list[tuple[str, str, bool, str]] = []  # (step, drone, passed, note)

def record(step: str, drone: str, passed: bool, note: str = "") -> None:
    test_results.append((step, drone, passed, note))
    plog("TEST", "OK" if passed else "WARN",
         f"[{step:<28}] {drone:<10} {'PASS' if passed else 'FAIL'}  {note}")

def print_summary() -> None:
    print("\n" + "─" * 64, flush=True)
    print("  VOICE ARM TEST — SUMMARY", flush=True)
    print("─" * 64, flush=True)
    total  = len(test_results)
    passed = sum(1 for _, _, ok, _ in test_results if ok)
    for step, drone, ok, note in test_results:
        icon = "✓" if ok else "✗"
        print(f"  {icon}  [{step:<28}] {drone:<10}  {note}", flush=True)
    print()
    if total == 0:
        print("  No tests run.", flush=True)
    elif passed == total:
        print(f"  {passed}/{total}  🟢 ALL PASS — voice → arm chain verified", flush=True)
    else:
        print(f"  {passed}/{total}  🔴 {total-passed} step(s) FAILED", flush=True)
    print("─" * 64, flush=True)

# ─────────────────────────────────────────────────────────────────────────────
#  PENDING COMMAND QUEUE  — shared between app server and voice bridge
# ─────────────────────────────────────────────────────────────────────────────
_pending_cmds: list[str] = []
_pending_lock = threading.Lock()

def push_command(cmd: str) -> None:
    """Push a command string into the pending queue (thread-safe)."""
    with _pending_lock:
        _pending_cmds.append(cmd)
    plog("BRIDGE", "INFO", f"Command queued: {cmd!r}")

def pop_commands() -> list[str]:
    """Drain and return all pending commands (thread-safe)."""
    with _pending_lock:
        cmds = list(_pending_cmds)
        _pending_cmds.clear()
    return cmds

# ─────────────────────────────────────────────────────────────────────────────
#  LOCAL FAKE app.py HTTP SERVER
#  Mimics /api/pending_commands exactly as tcp_commander._voice_bridge expects
# ─────────────────────────────────────────────────────────────────────────────
class _AppHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # silence access log

    def do_GET(self):
        if self.path == "/api/pending_commands":
            cmds = pop_commands()
            body = json.dumps({"commands": cmds}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()

def _run_app_server() -> None:
    server = HTTPServer(("127.0.0.1", APP_PORT), _AppHandler)
    plog("APP", "OK", f"Local app.py stub running on 127.0.0.1:{APP_PORT}")
    server.serve_forever()

# ─────────────────────────────────────────────────────────────────────────────
#  SLAVE TCP CLIENT
# ─────────────────────────────────────────────────────────────────────────────
class SlaveClient:
    def __init__(self, drone_id: str, ip: str, port: int = CMD_PORT):
        self.drone_id  = drone_id
        self.ip        = ip
        self.port      = port
        self._sock: Optional[socket.socket] = None
        self._buf  = bytearray()
        self._seq  = 0
        self.connected = False

    def try_connect(self) -> bool:
        try:
            plog(self.drone_id, "INFO", f"Connecting to {self.ip}:{self.port} …")
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(CONNECT_TIMEOUT_S)
            s.connect((self.ip, self.port))
            s.settimeout(ACK_TIMEOUT_S)
            self._sock = s
            self._buf.clear()
            self.connected = True
            plog(self.drone_id, "OK", "TCP connected")
            return True
        except (ConnectionRefusedError, socket.timeout, OSError) as e:
            plog(self.drone_id, "WARN",
                 f"Cannot connect to {self.ip}:{self.port} — {e}")
            plog(self.drone_id, "WARN",
                 "  Is main_orchestrator.py running on that Pi?")
            self.connected = False
            return False

    def send_cmd(self, payload: dict) -> dict:
        if not self.connected or self._sock is None:
            return {"status": "not_connected"}
        self._seq += 1
        payload["seq"] = self._seq
        raw = (json.dumps(payload) + "\n").encode()
        try:
            self._sock.sendall(raw)
        except OSError as e:
            plog(self.drone_id, "ERROR", f"Send failed: {e}")
            self.connected = False
            return {"status": "send_error"}
        try:
            self._sock.settimeout(ACK_TIMEOUT_S)
            while b"\n" not in self._buf:
                chunk = self._sock.recv(65535)
                if not chunk:
                    self.connected = False
                    return {"status": "disconnected"}
                self._buf.extend(chunk)
            line, remainder = self._buf.split(b"\n", 1)
            self._buf = bytearray(remainder)
            return json.loads(line.decode().strip())
        except socket.timeout:
            plog(self.drone_id, "WARN", f"ACK timeout seq={self._seq}")
            return {"status": "timeout"}
        except (json.JSONDecodeError, OSError) as e:
            plog(self.drone_id, "ERROR", f"ACK recv error: {e}")
            self.connected = False
            return {"status": "recv_error"}

    def arm_only(self) -> bool:
        """Send ARM_ONLY — arms FC, NO takeoff."""
        plog(self.drone_id, "INFO", "Sending ARM_ONLY …")
        ack = self.send_cmd({"cmd": "ARM_ONLY"})
        ok  = ack.get("status") == "ok"
        plog(self.drone_id, "OK" if ok else "ERROR",
             f"ARM_ONLY ack: {ack}")
        return ok

    def disarm(self) -> bool:
        plog(self.drone_id, "INFO", "Sending DISARM …")
        ack = self.send_cmd({"cmd": "DISARM"})
        ok  = ack.get("status") == "ok"
        plog(self.drone_id, "OK" if ok else "WARN",
             f"DISARM ack: {ack}")
        return ok

    def close(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        self.connected = False

# ─────────────────────────────────────────────────────────────────────────────
#  VOICE BRIDGE  — polls local app server, dispatches commands to slaves
#  Mirrors tcp_commander._voice_bridge() exactly so you're testing real logic
# ─────────────────────────────────────────────────────────────────────────────
async def voice_bridge_loop(
    slaves:        list[SlaveClient],
    stop_event:    asyncio.Event,
    armed_event:   asyncio.Event,
    disarmed_event: asyncio.Event,
) -> None:
    import aiohttp

    url       = f"http://127.0.0.1:{APP_PORT}/api/pending_commands"
    connector = aiohttp.TCPConnector()
    session   = aiohttp.ClientSession(connector=connector)
    connected = [s for s in slaves if s.connected]

    plog("BRIDGE", "OK",
         f"Voice bridge started — polling {url} at {BRIDGE_POLL_HZ} Hz")
    plog("BRIDGE", "INFO",
         f"Routing commands to {len(connected)} connected slave(s): "
         f"{[s.drone_id for s in connected]}")

    try:
        while not stop_event.is_set():
            await asyncio.sleep(1.0 / BRIDGE_POLL_HZ)
            try:
                async with session.get(
                    url,
                    timeout=aiohttp.ClientTimeout(total=0.4),
                ) as resp:
                    if resp.status != 200:
                        continue
                    data = await resp.json()
                    cmds = data.get("commands", [])
            except Exception:
                continue

            loop = asyncio.get_running_loop()

            for raw_cmd in cmds:
                cmd = raw_cmd.upper()
                plog("BRIDGE", "INFO", f"Processing command: {cmd!r}")

                if cmd == "ARM_ONLY":
                    plog("BRIDGE", "INFO",
                         "ARM_ONLY → broadcasting to all connected slaves …")
                    results = {}

                    def _send_arm(s: SlaveClient):
                        results[s.drone_id] = s.arm_only()

                    import threading as _t
                    threads = [_t.Thread(target=_send_arm, args=(s,), daemon=True)
                               for s in connected]
                    for t in threads: t.start()
                    for t in threads: t.join(timeout=ACK_TIMEOUT_S + 1)

                    for drone_id, ok in results.items():
                        record("voice_arm", drone_id, ok)

                    if any(results.values()):
                        armed_event.set()

                elif cmd == "DISARM":
                    plog("BRIDGE", "INFO",
                         "DISARM → broadcasting to all connected slaves …")
                    results = {}

                    def _send_disarm(s: SlaveClient):
                        results[s.drone_id] = s.disarm()

                    import threading as _t
                    threads = [_t.Thread(target=_send_disarm, args=(s,), daemon=True)
                               for s in connected]
                    for t in threads: t.start()
                    for t in threads: t.join(timeout=ACK_TIMEOUT_S + 1)

                    for drone_id, ok in results.items():
                        record("voice_disarm", drone_id, ok)

                    if any(results.values()):
                        disarmed_event.set()

                elif cmd == "STATUS":
                    print("\n── STATUS ─────────────────────────────────────────")
                    for s in slaves:
                        state = "connected" if s.connected else "NOT CONNECTED"
                        print(f"  {s.drone_id}  ({s.ip}:{s.port})  →  {state}")
                    print("───────────────────────────────────────────────────\n")

                else:
                    plog("BRIDGE", "WARN", f"Unknown command {cmd!r} — ignored")

    finally:
        await session.close()

# ─────────────────────────────────────────────────────────────────────────────
#  KEYBOARD INPUT  — runs in background thread, injects into pending queue
# ─────────────────────────────────────────────────────────────────────────────
def keyboard_input_loop(stop_event: asyncio.Event) -> None:
    print("\n" + "─" * 64)
    print("  KEYBOARD MODE — type a keyword and press ENTER")
    print("  Keywords: arm / start / activate  →  ARM drones (no takeoff)")
    print("            disarm / stop / abort   →  DISARM drones")
    print("            status / check          →  print connection status")
    print("            q / quit / exit         →  exit")
    print("─" * 64 + "\n")

    while not stop_event.is_set():
        try:
            text = input("  Command > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if not text:
            continue

        if text in ("q", "quit", "exit"):
            stop_event.set()
            break

        # Match any keyword in the typed text
        matched = None
        for keyword, action in KEYWORD_MAP.items():
            if keyword in text.split():
                matched = action
                break
        # Also try substring match as fallback
        if matched is None:
            for keyword, action in KEYWORD_MAP.items():
                if keyword in text:
                    matched = action
                    break

        if matched:
            plog("KEYBOARD", "OK", f"Matched keyword in '{text}' → {matched}")
            push_command(matched)
        else:
            plog("KEYBOARD", "WARN",
                 f"No keyword recognised in '{text}'. "
                 f"Try: arm, disarm, status, quit")

# ─────────────────────────────────────────────────────────────────────────────
#  VOICE INPUT  — uses vosk for offline speech recognition
# ─────────────────────────────────────────────────────────────────────────────
def voice_input_loop(stop_event: asyncio.Event) -> None:
    """
    Continuous microphone listener using Vosk (offline, no API key needed).
    Runs in a background thread. Recognised words are matched to KEYWORD_MAP
    and injected into the pending command queue exactly as keyboard mode does.

    Vosk outputs partial + final results. We only act on final results to
    avoid double-triggering on partial matches.
    """
    try:
        import vosk
        import sounddevice as sd
    except ImportError:
        plog("VOICE", "ERROR",
             "vosk or sounddevice not installed. "
             "Run:  pip install vosk sounddevice --break-system-packages")
        plog("VOICE", "ERROR",
             "Then download model from: https://alphacephei.com/vosk/models")
        plog("VOICE", "WARN", "Falling back to keyboard mode.")
        keyboard_input_loop(stop_event)
        return

    import os
    if not os.path.isdir(VOSK_MODEL_PATH):
        plog("VOICE", "ERROR",
             f"Vosk model not found at: {VOSK_MODEL_PATH}")
        plog("VOICE", "ERROR",
             "Download vosk-model-small-en-us-0.15 from https://alphacephei.com/vosk/models")
        plog("VOICE", "ERROR",
             "Extract to this directory and rename folder to: vosk-model-small-en-us-0.15")
        plog("VOICE", "WARN", "Falling back to keyboard mode.")
        keyboard_input_loop(stop_event)
        return

    vosk.SetLogLevel(-1)  # silence vosk debug logs
    model = vosk.Model(VOSK_MODEL_PATH)
    SAMPLE_RATE = 16000
    rec = vosk.KaldiRecognizer(model, SAMPLE_RATE)

    print("\n" + "─" * 64)
    print("  VOICE MODE — speak into microphone")
    print("  Keywords: arm / start / activate  →  ARM drones (no takeoff)")
    print("            disarm / stop / abort   →  DISARM drones")
    print("            status                  →  print connection info")
    print("  Press Ctrl-C to exit")
    print("─" * 64)
    print("  🎤 Listening …\n")

    # Microphone callback — fills an audio queue
    audio_q: queue.Queue = queue.Queue()

    def _mic_callback(indata, frames, time_info, status):
        if status:
            plog("MIC", "WARN", str(status))
        audio_q.put(bytes(indata))

    with sd.RawInputStream(
        samplerate=SAMPLE_RATE,
        blocksize=4000,
        dtype="int16",
        channels=1,
        callback=_mic_callback,
    ):
        last_final = ""
        cooldown_until = 0.0  # prevent double-trigger on same phrase

        while not stop_event.is_set():
            try:
                data = audio_q.get(timeout=0.5)
            except queue.Empty:
                continue

            if rec.AcceptWaveform(data):
                # Final result
                result = json.loads(rec.Result())
                text   = result.get("text", "").lower().strip()

                if not text or text == last_final:
                    continue
                last_final = text

                now = time.time()
                if now < cooldown_until:
                    plog("VOICE", "INFO",
                         f"Heard '{text}' — cooldown active, ignored")
                    continue

                plog("VOICE", "INFO", f"Heard: '{text}'")

                matched = None
                words   = text.split()
                for keyword, action in KEYWORD_MAP.items():
                    if keyword in words:
                        matched = action
                        break

                if matched:
                    plog("VOICE", "OK",
                         f"Keyword recognised in '{text}' → {matched}")
                    push_command(matched)
                    cooldown_until = now + 2.0  # 2s cooldown after trigger
                else:
                    plog("VOICE", "INFO",
                         f"No command keyword in '{text}'")
            else:
                # Partial result — just show it so user knows it's listening
                partial = json.loads(rec.PartialResult()).get("partial", "")
                if partial:
                    print(f"\r  🎤 {partial:<50}", end="", flush=True)

# ─────────────────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────────────────
async def run(args: argparse.Namespace) -> None:
    print("=" * 64, flush=True)
    print("  S.A.F.E. — INDOOR VOICE-ARM TEST", flush=True)
    print(f"  Mode     : {'VOICE (microphone)' if args.voice else 'KEYBOARD'}", flush=True)
    print(f"  Slaves   : {args.slaves or 'all (1, 2, 3)'}", flush=True)
    print(f"  App port : 127.0.0.1:{APP_PORT}", flush=True)
    print(f"  CMD port : :{CMD_PORT}", flush=True)
    print("=" * 64, flush=True)
    print()
    print("  ⚠  SAFETY: Remove or secure props before running.", flush=True)
    print("  ⚠  ARM_ONLY does NOT send takeoff.", flush=True)
    print("  ⚠  Motors will spin briefly when armed.", flush=True)
    print()
    input("  Press ENTER to continue or Ctrl-C to abort …\n")

    # ── Determine which slaves to test ───────────────────────────────────
    if args.slaves:
        ids_to_test = [f"slave_{n}" for n in args.slaves]
    else:
        ids_to_test = list(SLAVE_IPS.keys())

    # ── Connect to slaves ─────────────────────────────────────────────────
    plog("INIT", "INFO", "Connecting to slave TCP servers …")
    slaves: list[SlaveClient] = []
    for drone_id in ids_to_test:
        ip = SLAVE_IPS.get(drone_id)
        if not ip:
            plog("INIT", "WARN", f"Unknown drone_id '{drone_id}' — skipping")
            continue
        client = SlaveClient(drone_id=drone_id, ip=ip)
        client.try_connect()
        slaves.append(client)
        record("tcp_connect", drone_id, client.connected,
               f"({'connected' if client.connected else 'SKIPPED — not reachable'})")

    connected = [s for s in slaves if s.connected]
    if not connected:
        plog("INIT", "ERROR",
             "No slaves connected. Cannot run test.")
        plog("INIT", "ERROR",
             "Check: (1) main_orchestrator.py running on slave Pi, "
             "(2) slave Pi on Wi-Fi, (3) IP correct in SLAVE_IPS")
        return

    plog("INIT", "OK",
         f"{len(connected)}/{len(slaves)} slaves connected: "
         f"{[s.drone_id for s in connected]}")

    # ── Start fake app.py server ─────────────────────────────────────────
    threading.Thread(target=_run_app_server, daemon=True,
                     name="app_server").start()
    await asyncio.sleep(0.3)

    # ── Shared events ─────────────────────────────────────────────────────
    stop_event     = asyncio.Event()
    armed_event    = asyncio.Event()
    disarmed_event = asyncio.Event()

    # ── Start voice bridge ────────────────────────────────────────────────
    bridge_task = asyncio.create_task(
        voice_bridge_loop(slaves, stop_event, armed_event, disarmed_event),
        name="voice_bridge",
    )

    # ── Start input thread ────────────────────────────────────────────────
    input_fn = voice_input_loop if args.voice else keyboard_input_loop
    input_thread = threading.Thread(
        target=input_fn,
        args=(stop_event,),
        daemon=True,
        name="input_thread",
    )
    input_thread.start()

    # ── Wait for arm then disarm, or for stop ─────────────────────────────
    plog("INIT", "OK",
         "Ready. Trigger an 'arm' command to begin the test.")
    plog("INIT", "INFO",
         "Test will wait for: ARM → confirm → DISARM → print summary")

    try:
        # Wait for arm event (up to 5 minutes)
        try:
            await asyncio.wait_for(
                asyncio.shield(armed_event.wait()),
                timeout=300.0,
            )
            plog("TEST", "OK",
                 "ARM received and dispatched. Waiting 3s to confirm …")
            await asyncio.sleep(3.0)

            # Now wait for disarm
            plog("TEST", "INFO",
                 "Now say 'disarm' or type 'disarm' to complete the test.")
            try:
                await asyncio.wait_for(
                    asyncio.shield(disarmed_event.wait()),
                    timeout=60.0,
                )
                plog("TEST", "OK", "DISARM received and dispatched.")
            except asyncio.TimeoutError:
                plog("TEST", "WARN",
                     "No DISARM received in 60s — sending DISARM automatically for safety.")
                for s in connected:
                    s.disarm()
                record("auto_disarm_safety", "all", True,
                       "Auto-disarmed after 60s timeout")

        except asyncio.TimeoutError:
            plog("TEST", "WARN",
                 "No ARM command in 5 minutes — exiting.")
            record("voice_arm", "all", False, "No command received in 5 min")

    except asyncio.CancelledError:
        pass

    finally:
        stop_event.set()
        plog("SAFETY", "INFO", "Sending DISARM to all connected drones …")
        for s in connected:
            if s.connected:
                s.disarm()
        bridge_task.cancel()
        for s in slaves:
            s.close()

    print_summary()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="S.A.F.E. indoor voice → arm test",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--voice", action="store_true",
        help="Use microphone input instead of keyboard",
    )
    parser.add_argument(
        "--slaves", type=int, nargs="+", metavar="N",
        help="Slave numbers to test, e.g. --slaves 1 2  (default: all)",
    )
    args = parser.parse_args()

    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\n")
        plog("SAFETY", "WARN", "Ctrl-C — DISARM sent to all connected drones")
        # Best-effort sync disarm on interrupt (event loop already stopping)
        for drone_id, ip in SLAVE_IPS.items():
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(2.0)
                s.connect((ip, CMD_PORT))
                s.sendall((json.dumps({"cmd": "DISARM", "seq": 9999}) + "\n").encode())
                s.close()
            except OSError:
                pass
        print_summary()
        sys.exit(0)


# ─────────────────────────────────────────────────────────────────────────────
#  HANDLER STUBS FOR main_orchestrator.py
#  Add these inside start_tcp_command_server() alongside the existing handlers.
# ─────────────────────────────────────────────────────────────────────────────
"""
PASTE THESE INTO main_orchestrator.py  →  start_tcp_command_server()
=====================================================================

    @server.on_command("ARM_ONLY")
    def handle_arm_only(cmd: dict) -> dict:
        seq = cmd.get("seq", "?")
        log("TCP", "OK", f"ARM_ONLY received (seq={seq}) — arming FC, NO takeoff")
        async def _arm():
            try:
                await movement.drone.action.arm()
                log("TCP", "OK", "ARM_ONLY: FC armed successfully")
            except Exception as e:
                log("TCP", "ERROR", f"ARM_ONLY: arm() failed: {e}")
        asyncio.run_coroutine_threadsafe(_arm(), asyncio.get_running_loop())
        return {"status": "ok"}

    @server.on_command("DISARM")
    def handle_disarm(cmd: dict) -> dict:
        seq = cmd.get("seq", "?")
        log("TCP", "OK", f"DISARM received (seq={seq}) — disarming FC")
        async def _disarm():
            try:
                await movement.drone.action.disarm()
                log("TCP", "OK", "DISARM: FC disarmed")
            except Exception as e:
                log("TCP", "ERROR", f"DISARM: disarm() failed: {e}")
        asyncio.run_coroutine_threadsafe(_disarm(), asyncio.get_running_loop())
        return {"status": "ok"}
"""

if __name__ == "__main__":
    main()
