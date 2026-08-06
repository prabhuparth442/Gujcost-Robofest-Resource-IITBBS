"""
app.py  —  S.A.F.E. Ground Server  (phone-mic → Pi Vosk edition)
=================================================================

Architecture
------------
Thread 1  Flask web server
  • Serves the map UI to the phone browser
  • Handles GPS, mine overlay, drone telemetry, mission state
  • Exposes  POST /api/audio_chunk  — phone streams raw PCM here

Thread 2  Vosk recognition worker
  • Pure offline, no internet, no physical mic on the Pi
  • Reads raw PCM chunks from a thread-safe queue (audio_queue)
  • Runs Vosk KaldiRecognizer on each chunk
  • On a recognised intent → calls _enqueue_command()

Phone browser (index.html)
  • getUserMedia({ audio: true })  — captures mic at 16 kHz mono
  • ScriptProcessorNode collects 4096-sample frames
  • Converts Float32 → Int16 PCM
  • POSTs binary blob to  /api/audio_chunk  every ~0.5 s
  • Polls  /api/status  at 1 Hz to display latest transcript

No sounddevice, no physical mic, no internet required after page load.

─────────────────────────────────────────────────────────────────────
Setup (run once inside drone_env):
  sudo apt-get install -y portaudio19-dev libatomic1
  pip install flask vosk

Model (already present at ~/webserver/vosk-model-small-en-us-0.15/):
  ✅  confirmed present — no download needed

TLS cert (so browser allows getUserMedia over HTTPS):
  openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem \
    -days 365 -nodes -subj "/CN=localhost"

Run:
  cd ~/webserver
  sudo python3 app.py
─────────────────────────────────────────────────────────────────────
"""

import heapq
import json
import math
import queue
import threading
import time
from pathlib import Path

from flask import Flask, jsonify, render_template_string, request

# ════════════════════════════════════════════════════════════════════════════
#  CONFIG
# ════════════════════════════════════════════════════════════════════════════
VOSK_MODEL_PATH = "./vosk-model-small-en-us-0.15"
SAMPLE_RATE     = 16000   # must match browser AudioContext sampleRate

# ════════════════════════════════════════════════════════════════════════════
#  SHARED LOCK  — every write to shared state must hold this
# ════════════════════════════════════════════════════════════════════════════
_lock = threading.Lock()

# ════════════════════════════════════════════════════════════════════════════
#  SHARED STATE
# ════════════════════════════════════════════════════════════════════════════
origin_lat:     float | None = None
origin_lng:     float | None = None
origin_heading: float        = 0.0

drone_positions: dict[str, dict] = {}
real_mines_gps:  list[dict]      = []

mission_state = {
    "started":    False,
    "ended":      False,
    "start_time": None,
}

pending_commands: list[str] = []

# Written by Vosk thread, read by /api/status so the phone can display it
# {"heard": raw text Vosk heard, "cmd": matched intent or "", "ts": unix timestamp}
latest_transcript: dict = {"heard": "", "cmd": "", "ts": 0.0}

# Raw PCM chunks arrive from the phone → Vosk worker reads from here
audio_queue: queue.Queue[bytes] = queue.Queue(maxsize=60)

# ════════════════════════════════════════════════════════════════════════════
#  INTENT MAP
# ════════════════════════════════════════════════════════════════════════════
INTENTS: dict[str, list[str]] = {
    "start":  ["start", "go", "begin", "launch", "fly"],
    "pause":  ["pause", "stop", "hold", "wait", "freeze"],
    "resume": ["resume", "continue", "proceed", "go ahead"],
    "scan":   ["scan", "check", "rescan", "search"],
    "land":   ["land", "abort", "emergency", "down", "descend"],
    "status": ["status", "report", "where"],
}

def find_intent(text: str) -> str | None:
    lower = text.lower()
    for intent, triggers in INTENTS.items():
        for trigger in triggers:
            if trigger in lower:
                return intent
    return None


def _enqueue_command(cmd: str, heard: str = "") -> None:
    """Thread-safe — called from Vosk thread and Flask voice_command route."""
    global latest_transcript
    with _lock:
        pending_commands.append(cmd)
        latest_transcript = {"heard": heard or cmd, "cmd": cmd, "ts": time.time()}
        if cmd == "start" and not mission_state["started"]:
            mission_state["started"]    = True
            mission_state["start_time"] = mission_state["start_time"] or time.time()
        elif cmd == "land":
            mission_state["ended"] = True
    print(f"[CMD] {cmd.upper()} (heard: \'{heard}\')")


# ════════════════════════════════════════════════════════════════════════════
#  COORDINATE MATH
# ════════════════════════════════════════════════════════════════════════════
EARTH_R = 6_378_137.0

def gps_to_local(lat: float, lng: float) -> dict:
    if origin_lat is None:
        return {"x": 0.0, "y": 0.0}
    d_lat = math.radians(lat - origin_lat)
    d_lng = math.radians(lng - origin_lng)
    y_raw = d_lat * EARTH_R
    x_raw = d_lng * EARTH_R * math.cos(math.radians(origin_lat))
    h     = math.radians(origin_heading)
    return {
        "x": round(x_raw * math.cos(h) - y_raw * math.sin(h), 3),
        "y": round(x_raw * math.sin(h) + y_raw * math.cos(h), 3),
    }


# ════════════════════════════════════════════════════════════════════════════
#  A* PATH PLANNER
# ════════════════════════════════════════════════════════════════════════════
STEP        = 0.5
SAFE_RADIUS = 1.0
FIELD_W     = 20.0   # FIX 4: was 10.0 — full 20m competition field width
FIELD_L     = 100.0

def _is_safe(x, y, mines):
    return all(math.hypot(x - m["x"], y - m["y"]) > SAFE_RADIUS for m in mines)

def calculate_path(start_x, start_y, goal_y, mines):
    sx = round(start_x / STEP) * STEP
    sy = round(start_y / STEP) * STEP
    heap = [(0.0, (sx, sy))]
    came_from: dict = {}
    g: dict = {(sx, sy): 0.0}
    moves = [(STEP,0),(-STEP,0),(0,STEP),(0,-STEP),
             (STEP,STEP),(STEP,-STEP),(-STEP,STEP),(-STEP,-STEP)]
    while heap:
        _, cur = heapq.heappop(heap)
        cx, cy = cur
        if cy >= goal_y:
            path = []
            while cur in came_from:
                path.append({"x": round(cur[0],2), "y": round(cur[1],2)})
                cur = came_from[cur]
            path.reverse()
            return path
        for dx, dy in moves:
            nx, ny = cx+dx, cy+dy
            if not (-FIELD_W <= nx <= FIELD_W and 0 <= ny <= FIELD_L):
                continue
            if not _is_safe(nx, ny, mines):
                continue
            new_g = g[cur] + math.hypot(dx, dy)
            if (nx, ny) not in g or new_g < g[(nx, ny)]:
                came_from[(nx, ny)] = cur
                g[(nx, ny)] = new_g
                heapq.heappush(heap, (new_g + (goal_y - ny), (nx, ny)))
    return []


# ════════════════════════════════════════════════════════════════════════════
#  STATUS HELPER
# ════════════════════════════════════════════════════════════════════════════
def _build_status() -> dict:
    with _lock:
        elapsed = None
        if mission_state["start_time"]:
            elapsed = round(time.time() - mission_state["start_time"], 1)
        drones_local = {
            did: {
                "x":       gps_to_local(d["lat"], d["lng"])["x"],
                "y":       gps_to_local(d["lat"], d["lng"])["y"],
                "heading": d.get("heading", 0),
                "alt":     d.get("altitude", 0),
            }
            for did, d in drone_positions.items()
        }
        mines_local = [
            {"id": i, **gps_to_local(m["lat"], m["lng"]),
             "detected_by": m.get("detected_by", "?")}
            for i, m in enumerate(real_mines_gps)
        ]
        return {
            "mission":    dict(mission_state),
            "elapsed":    elapsed,
            "drones":     drones_local,
            "mines":      mines_local,
            "mine_count": len(mines_local),
            "origin_set": origin_lat is not None,
            "transcript": dict(latest_transcript),   # latest Vosk result → phone UI
        }


# ════════════════════════════════════════════════════════════════════════════
#  FLASK APP  (Thread 1)
# ════════════════════════════════════════════════════════════════════════════
app = Flask(__name__)


@app.route("/")
def index():
    return render_template_string(Path("index.html").read_text())


@app.route("/api/audio_stream", methods=["POST"])
@app.route("/api/audio_chunk", methods=["POST"])   # keep old name as alias
def audio_stream():
    """
    Phone browser POSTs raw 16-bit LE mono PCM at 16 kHz.
    We push it into audio_queue for the Vosk worker (Thread 2).
    Drops chunk silently if the queue is full (backpressure).
    """
    data = request.get_data()   # raw bytes, no JSON parsing
    if data:
        try:
            audio_queue.put_nowait(data)
        except queue.Full:
            pass   # Vosk can't keep up — drop newest is fine
    return "", 204   # No Content — fastest possible response


@app.route("/api/set_origin", methods=["POST"])
def set_origin():
    global origin_lat, origin_lng, origin_heading
    d = request.json
    with _lock:
        origin_lat     = d["lat"]
        origin_lng     = d["lng"]
        origin_heading = float(d.get("heading", 0.0))
        real_mines_gps.clear()
        drone_positions.clear()
        mission_state.update(started=False, ended=False, start_time=None)
    print(f"[ORIGIN] {origin_lat:.6f}, {origin_lng:.6f}, hdg={origin_heading:.1f}°")
    return jsonify({"status": "ok"})


@app.route("/api/drone_update", methods=["POST"])
def drone_update():
    pkt = request.json
    t   = pkt.get("type")
    with _lock:
        if t == "mine_detected":
            real_mines_gps.append({
                "lat": pkt["lat"], "lng": pkt["lng"],
                "detected_by": pkt.get("drone_id", "?"),
            })
            print(f"[MINE] #{len(real_mines_gps)} at {pkt['lat']:.6f},{pkt['lng']:.6f}")
        elif t == "drone_position":
            drone_positions[pkt["drone_id"]] = {
                "lat":      pkt["lat"], "lng":      pkt["lng"],
                "heading":  float(pkt.get("heading", 0)),
                "altitude": float(pkt.get("altitude", 0)),
            }
        elif t == "mission_start":
            if not mission_state["started"]:
                mission_state["started"]    = True
                mission_state["start_time"] = time.time()
                print("[MISSION] Started")
        elif t == "mission_end":
            mission_state["ended"] = True
            print("[MISSION] Ended")
    return jsonify({"status": "ok"})


@app.route("/api/voice_command", methods=["POST"])
def voice_command():
    """Fallback: browser sends a pre-parsed command string directly."""
    cmd = str(request.json.get("cmd", "")).lower().strip()
    if cmd not in {"start", "pause", "resume", "scan", "land", "status"}:
        return jsonify({"error": f"unknown: {cmd}"}), 422
    _enqueue_command(cmd)
    return jsonify({"status": "ok", "cmd": cmd})


@app.route("/api/pending_commands", methods=["GET"])
def pending_commands_get():
    """master.py polls this — commands consumed on each poll."""
    with _lock:
        cmds = pending_commands.copy()
        pending_commands.clear()
    return jsonify({"commands": cmds})


@app.route("/api/get_my_location", methods=["POST"])
def get_my_location():
    d        = request.json
    user_pos = gps_to_local(d["lat"], d["lng"])
    with _lock:
        mines_local = [
            {**gps_to_local(m["lat"], m["lng"]), "id": i}
            for i, m in enumerate(real_mines_gps)
        ]
    safe_path = calculate_path(user_pos["x"], user_pos["y"], FIELD_L, mines_local)
    return jsonify({"pos": user_pos, "path": safe_path, "mines": mines_local})


@app.route("/api/status", methods=["GET"])
def status():
    return jsonify(_build_status())


# ════════════════════════════════════════════════════════════════════════════
#  VOSK WORKER  (Thread 2)
# ════════════════════════════════════════════════════════════════════════════
def vosk_worker():
    """
    Offline speech recognition — no internet, no Pi mic.
    Reads raw 16-bit mono PCM from audio_queue (filled by /api/audio_chunk).
    Recognises intents and injects them into pending_commands.
    """
    global latest_transcript

    try:
        from vosk import KaldiRecognizer, Model, SetLogLevel
    except ImportError:
        print("[VOSK] ❌  vosk not installed.  Run: pip install vosk")
        return

    if not Path(VOSK_MODEL_PATH).exists():
        print(f"[VOSK] ❌  Model not found at {VOSK_MODEL_PATH!r}")
        return

    SetLogLevel(-1)
    model = Model(VOSK_MODEL_PATH)
    rec   = KaldiRecognizer(model, SAMPLE_RATE)
    rec.SetWords(True)

    print("[VOSK] ✅  Offline recognition ready — waiting for phone audio")

    while True:
        chunk = audio_queue.get()   # blocks until phone sends audio

        if rec.AcceptWaveform(chunk):
            result = json.loads(rec.Result())
            text   = result.get("text", "").strip()
            if not text:
                continue

            print(f"[VOSK] Heard: '{text}'")
            intent = find_intent(text)
            if intent:
                _enqueue_command(intent, heard=text)
            else:
                with _lock:
                    latest_transcript = {"heard": text, "cmd": "", "ts": time.time()}
                print(f"[VOSK] No match for: '{text}'")


# ════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    # Thread 2 — Vosk worker (daemon: dies automatically when Flask exits)
    t = threading.Thread(target=vosk_worker, name="VoskWorker", daemon=True)
    t.start()

    # Thread 1 — Flask
    # use_reloader=False is mandatory when running extra threads
    ssl_ctx = ("cert.pem", "key.pem") if Path("cert.pem").exists() else None
    if not ssl_ctx:
        print("[WARN] No cert.pem — running HTTP. Browser may block getUserMedia.")
        print("       Generate: openssl req -x509 -newkey rsa:2048 -keyout key.pem "
              "-out cert.pem -days 365 -nodes -subj '/CN=localhost'")

    app.run(
        host         = "0.0.0.0",
        port         = 443 if ssl_ctx else 5000,
        ssl_context  = ssl_ctx,
        debug        = False,
        use_reloader = False,
    )
