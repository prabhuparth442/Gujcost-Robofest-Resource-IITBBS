"""
app.py  —  S.A.F.E. Ground Server  (multi-client edition)
==========================================================

Multi-client support
--------------------
Multiple phones / laptops can open https://10.42.0.1 at the same time.
Each visitor must enter the 4-digit PIN (default: 1234, change ACCESS_PIN).

How it works:
  • GET  /         → serves login page if no valid session, else map UI
  • POST /login    → check PIN, set secure cookie, redirect to map
  • GET  /logout   → clear cookie
  • GET  /stream   → Server-Sent Events — pushes live status to ALL
                     connected browsers every second.  No polling needed.
  • All /api/*     → unchanged, but protected — reject if no valid cookie.

Session security:
  Cookies are signed with a random SECRET_KEY generated at startup.
  The PIN is never stored in the cookie — only a signed "authenticated"
  flag.  Changing ACCESS_PIN or restarting the server invalidates all
  existing sessions.

Architecture
------------
Thread 1  Flask web server  (handles HTTP, SSE, API)
Thread 2  Vosk recognition worker
Thread 3  SSE broadcaster — wakes every second, pushes status to all
          connected SSE clients via a queue per client.

Change the PIN:
  Edit ACCESS_PIN below before competition.

Run:
  sudo python3 app.py
"""

import heapq
import json
import math
import queue
import secrets
import threading
import time
from pathlib import Path

from flask import (Flask, Response, jsonify, redirect,
                   render_template_string, request,
                   session, url_for)

# ════════════════════════════════════════════════════════════════════════════
#  CONFIG
# ════════════════════════════════════════════════════════════════════════════
VOSK_MODEL_PATH = "./vosk-model-small-en-us-0.15"
SAMPLE_RATE     = 16000

ACCESS_PIN  = "1234"          # ← Change this before competition!
SESSION_KEY = "authenticated"

# ════════════════════════════════════════════════════════════════════════════
#  SHARED LOCK
# ════════════════════════════════════════════════════════════════════════════
_lock = threading.Lock()

# ════════════════════════════════════════════════════════════════════════════
#  MASTER GRID
# ════════════════════════════════════════════════════════════════════════════
try:
    from grid_map import GridMap as _GridMap
    _master_grid = _GridMap()
    _GRID_AVAILABLE = True
except ImportError:
    _master_grid = None
    _GRID_AVAILABLE = False

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
latest_transcript: dict = {"heard": "", "cmd": "", "ts": 0.0}
audio_queue: queue.Queue[bytes] = queue.Queue(maxsize=60)

# ════════════════════════════════════════════════════════════════════════════
#  SSE CLIENT REGISTRY
#  Each connected browser gets its own queue. The broadcaster thread
#  pushes the same status JSON to every queue every second.
# ════════════════════════════════════════════════════════════════════════════
_sse_clients: list[queue.Queue] = []
_sse_lock = threading.Lock()


def _sse_register() -> queue.Queue:
    q = queue.Queue(maxsize=10)
    with _sse_lock:
        _sse_clients.append(q)
    return q


def _sse_unregister(q: queue.Queue) -> None:
    with _sse_lock:
        try:
            _sse_clients.remove(q)
        except ValueError:
            pass


def _sse_broadcast(data: str) -> None:
    """Push the same SSE message to every connected client."""
    dead = []
    with _sse_lock:
        clients = list(_sse_clients)
    for q in clients:
        try:
            q.put_nowait(data)
        except queue.Full:
            dead.append(q)
    for q in dead:
        _sse_unregister(q)


def _broadcaster_thread() -> None:
    """Runs forever — pushes live status to all SSE clients every second."""
    while True:
        time.sleep(1.0)
        try:
            payload = json.dumps(_build_status())
            _sse_broadcast(f"data: {payload}\n\n")
        except Exception as e:
            print(f"[SSE] Broadcast error: {e}")


def _client_count() -> int:
    with _sse_lock:
        return len(_sse_clients)


# ════════════════════════════════════════════════════════════════════════════
#  INTENT MAP
# ════════════════════════════════════════════════════════════════════════════
INTENTS: dict[str, list[str]] = {
    "start":   ["start", "go", "begin", "launch", "fly"],
    "pause":   ["pause", "stop", "hold", "wait", "freeze"],
    "resume":  ["resume", "continue", "proceed", "go ahead"],
    "scan":    ["scan", "check", "rescan", "search"],
    "forward": ["forward", "advance", "move forward", "next"],
    "land":    ["land", "abort", "emergency", "down", "descend"],
    "status":  ["status", "report", "where"],
}


def find_intent(text: str) -> str | None:
    lower = text.lower()
    for intent, triggers in INTENTS.items():
        for trigger in triggers:
            if trigger in lower:
                return intent
    return None


def _enqueue_command(cmd: str, heard: str = "") -> None:
    global latest_transcript
    with _lock:
        pending_commands.append(cmd)
        latest_transcript = {"heard": heard or cmd, "cmd": cmd, "ts": time.time()}
        if cmd == "start" and not mission_state["started"]:
            mission_state["started"]    = True
            mission_state["start_time"] = mission_state["start_time"] or time.time()
        elif cmd == "land":
            mission_state["ended"] = True
    print(f"[CMD] {cmd.upper()} (heard: '{heard}')")


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
#  FIELD MAP
# ════════════════════════════════════════════════════════════════════════════
try:
    from fieldmap import FIELD as _FIELD, BURIED_MINES as _BURIED_MINES, \
                         FORBIDDEN_ZONES as _FORBIDDEN_ZONES
    _FIELD_AWARE = True
    print(f"[FIELDMAP] Loaded — {len(_BURIED_MINES)} buried mines, "
          f"{len(_FORBIDDEN_ZONES)} forbidden zones")
except ImportError:
    _FIELD_AWARE = False
    _FIELD = None
    _BURIED_MINES = []
    _FORBIDDEN_ZONES = []
    print("[FIELDMAP] fieldmap.py not found — running in generic mode")


# ════════════════════════════════════════════════════════════════════════════
#  A* PATH PLANNER
# ════════════════════════════════════════════════════════════════════════════
STEP        = 0.5
SAFE_RADIUS = 1.0
FIELD_W     = 32.0
FIELD_S     = 24.0
FIELD_L     = 100.0


def _all_hazard_circles(runtime_mines: list[dict]) -> list[dict]:
    circles = []
    if _FIELD_AWARE:
        for m in _BURIED_MINES:
            circles.append({"x": m.x, "y": m.y, "r": m.radius_m})
        for fz in _FORBIDDEN_ZONES:
            circles.append({"x": fz.x, "y": fz.y, "r": fz.radius_m + 0.5})
    for m in runtime_mines:
        circles.append({"x": m["x"], "y": m["y"], "r": SAFE_RADIUS})
    return circles


def _is_safe(x, y, hazards):
    if _FIELD_AWARE and not _FIELD.is_safe(x, y, drone_margin=0.0):
        return False
    return all(math.hypot(x - h["x"], y - h["y"]) > h["r"] for h in hazards)


def calculate_path(start_x, start_y, goal_y, mines):
    hazards = _all_hazard_circles(mines)
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
            if not (-FIELD_W <= nx <= FIELD_W and -FIELD_S <= ny <= FIELD_L):
                continue
            if not _is_safe(nx, ny, hazards):
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
        preknown_mines = []
        forbidden_zones_ui = []
        if _FIELD_AWARE:
            preknown_mines = [
                {"id": m.name, "x": m.x, "y": m.y,
                 "r": m.radius_m, "type": "buried_known"}
                for m in _BURIED_MINES
            ]
            forbidden_zones_ui = [
                {"id": fz.name, "x": fz.x, "y": fz.y,
                 "r": fz.radius_m, "type": "forbidden"}
                for fz in _FORBIDDEN_ZONES
            ]
        covered_cells = []
        coverage_pct  = 0.0
        if _GRID_AVAILABLE and _master_grid is not None:
            covered_cells = _master_grid.snapshot()
            coverage_pct  = _master_grid.coverage_pct()

        return {
            "mission":           dict(mission_state),
            "elapsed":           elapsed,
            "drones":            drones_local,
            "mines":             mines_local,
            "mine_count":        len(mines_local),
            "preknown_mines":    preknown_mines,
            "forbidden_zones":   forbidden_zones_ui,
            "covered_cells":     covered_cells,
            "coverage_pct":      coverage_pct,
            "origin_set":        origin_lat is not None,
            "transcript":        dict(latest_transcript),
            "connected_clients": _client_count(),
        }


# ════════════════════════════════════════════════════════════════════════════
#  LOGIN PAGE  (inline HTML — no extra template file needed)
# ════════════════════════════════════════════════════════════════════════════
_LOGIN_HTML = """<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>S.A.F.E. — Login</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;
         background:#0d1117;color:#e6edf3;display:flex;align-items:center;
         justify-content:center;min-height:100vh}
    .card{background:#161b22;border:1px solid #30363d;border-radius:12px;
          padding:40px 36px;width:340px;text-align:center}
    .logo{font-size:2.4rem;margin-bottom:6px}
    h1{font-size:1.3rem;font-weight:600;color:#58a6ff;margin-bottom:4px}
    .sub{font-size:.82rem;color:#8b949e;margin-bottom:28px}
    label{display:block;text-align:left;font-size:.82rem;color:#8b949e;margin-bottom:6px}
    input[type=password]{width:100%;padding:10px 14px;background:#0d1117;
      border:1px solid #30363d;border-radius:8px;color:#e6edf3;font-size:1.4rem;
      letter-spacing:8px;text-align:center;margin-bottom:18px;outline:none}
    input[type=password]:focus{border-color:#58a6ff}
    button{width:100%;padding:11px;background:#238636;border:none;
           border-radius:8px;color:#fff;font-size:1rem;font-weight:600;cursor:pointer}
    button:hover{background:#2ea043}
    .error{background:#3d1f1f;border:1px solid #f85149;color:#f85149;
           border-radius:6px;padding:8px 12px;margin-bottom:16px;font-size:.85rem}
    .clients{margin-top:20px;font-size:.78rem;color:#484f58}
  </style>
</head>
<body>
  <div class="card">
    <div class="logo">&#x1F6E1;&#xFE0F;</div>
    <h1>S.A.F.E. Ground Control</h1>
    <div class="sub">Enter access PIN to connect</div>
    {% if error %}<div class="error">&#x26A0; {{ error }}</div>{% endif %}
    <form method="POST" action="/login">
      <label>Access PIN</label>
      <input type="password" name="pin" maxlength="8"
             autofocus autocomplete="off" inputmode="numeric" placeholder="&bull;&bull;&bull;&bull;">
      <button type="submit">Connect &#x2192;</button>
    </form>
    <div class="clients">{{ clients }} device(s) currently connected</div>
  </div>
</body>
</html>"""


# ════════════════════════════════════════════════════════════════════════════
#  FLASK APP
# ════════════════════════════════════════════════════════════════════════════
app = Flask(__name__)
app.secret_key = secrets.token_hex(32)   # random at startup — cookies reset on restart


def _is_authenticated() -> bool:
    return session.get(SESSION_KEY) is True


def _require_auth():
    """Return redirect if not authenticated, else None."""
    if not _is_authenticated():
        return redirect(url_for("login_page"))
    return None


# ── Auth ──────────────────────────────────────────────────────────────────────

@app.route("/login", methods=["GET"])
def login_page():
    if _is_authenticated():
        return redirect(url_for("index"))
    return render_template_string(_LOGIN_HTML, error=None, clients=_client_count())


@app.route("/login", methods=["POST"])
def login_submit():
    pin = request.form.get("pin", "").strip()
    if pin == ACCESS_PIN:
        session[SESSION_KEY] = True
        session.permanent    = True
        print(f"[AUTH] Client authenticated: {request.remote_addr}")
        return redirect(url_for("index"))
    print(f"[AUTH] Wrong PIN from {request.remote_addr}")
    return render_template_string(_LOGIN_HTML,
                                  error="Wrong PIN — try again",
                                  clients=_client_count()), 401


@app.route("/logout")
def logout():
    session.clear()
    return redirect(url_for("login_page"))


# ── Main UI ───────────────────────────────────────────────────────────────────

@app.route("/")
def index():
    redir = _require_auth()
    if redir:
        return redir
    return render_template_string(Path("index.html").read_text())


# ── SSE — live push to ALL connected clients ──────────────────────────────────

@app.route("/stream")
def stream():
    """
    Server-Sent Events endpoint.
    Every authenticated browser connects here and gets live status
    pushed every second — all devices stay in sync automatically.
    index.html should replace its setInterval status poll with:

        const es = new EventSource('/stream');
        es.onmessage = e => updateUI(JSON.parse(e.data));
    """
    redir = _require_auth()
    if redir:
        return redir, 401

    client_q = _sse_register()
    print(f"[SSE] +1 client {request.remote_addr} (total: {_client_count()})")

    def event_stream():
        # Send immediately on connect — no 1s wait
        try:
            yield f"data: {json.dumps(_build_status())}\n\n"
        except Exception:
            pass
        try:
            while True:
                try:
                    msg = client_q.get(timeout=25.0)
                    yield msg
                except queue.Empty:
                    yield ": keepalive\n\n"   # prevent proxy/browser timeout
        finally:
            _sse_unregister(client_q)
            print(f"[SSE] -1 client {request.remote_addr} (total: {_client_count()})")

    return Response(
        event_stream(),
        mimetype="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


# ── API ───────────────────────────────────────────────────────────────────────

@app.route("/api/audio_stream", methods=["POST"])
@app.route("/api/audio_chunk",  methods=["POST"])
def audio_stream():
    if not _is_authenticated():
        return "", 401
    data = request.get_data()
    if data:
        try:
            audio_queue.put_nowait(data)
        except queue.Full:
            pass
    return "", 204


@app.route("/api/set_origin", methods=["POST"])
def set_origin():
    if not _is_authenticated():
        return jsonify({"error": "unauthorized"}), 401
    global origin_lat, origin_lng, origin_heading
    d = request.json
    with _lock:
        origin_lat     = d["lat"]
        origin_lng     = d["lng"]
        origin_heading = float(d.get("heading", 0.0))
        real_mines_gps.clear()
        drone_positions.clear()
        mission_state.update(started=False, ended=False, start_time=None)
    print(f"[ORIGIN] {origin_lat:.6f}, {origin_lng:.6f}, hdg={origin_heading:.1f}")
    return jsonify({"status": "ok"})


@app.route("/api/drone_update", methods=["POST"])
def drone_update():
    # Called by udp_telementry (localhost) — no browser session needed
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

    if _GRID_AVAILABLE and _master_grid is not None:
        grid_cells = pkt.get("grid")
        if grid_cells:
            try:
                _master_grid.merge_from(grid_cells)
            except Exception as e:
                print(f"[GRID] merge error: {e}")

    return jsonify({"status": "ok"})


@app.route("/api/voice_command", methods=["POST"])
def voice_command():
    if not _is_authenticated():
        return jsonify({"error": "unauthorized"}), 401
    cmd = str(request.json.get("cmd", "")).lower().strip()
    if cmd not in {"start", "pause", "resume", "scan", "forward", "land", "status"}:
        return jsonify({"error": f"unknown: {cmd}"}), 422
    _enqueue_command(cmd)
    return jsonify({"status": "ok", "cmd": cmd})


@app.route("/api/pending_commands", methods=["GET"])
def pending_commands_get():
    # Polled by test_voice.py / tcp_commander — internal, no session needed
    with _lock:
        cmds = pending_commands.copy()
        pending_commands.clear()
    return jsonify({"commands": cmds})


@app.route("/api/get_my_location", methods=["POST"])
def get_my_location():
    if not _is_authenticated():
        return jsonify({"error": "unauthorized"}), 401
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
    # Allow unauthenticated — used by internal test scripts
    return jsonify(_build_status())


@app.route("/api/clients", methods=["GET"])
def clients():
    return jsonify({"connected": _client_count()})


# ════════════════════════════════════════════════════════════════════════════
#  VOSK WORKER  (Thread 2)
# ════════════════════════════════════════════════════════════════════════════
def vosk_worker():
    global latest_transcript
    try:
        from vosk import KaldiRecognizer, Model, SetLogLevel
    except ImportError:
        print("[VOSK] vosk not installed — pip install vosk")
        return
    if not Path(VOSK_MODEL_PATH).exists():
        print(f"[VOSK] Model not found at {VOSK_MODEL_PATH!r}")
        return
    SetLogLevel(-1)
    model = Model(VOSK_MODEL_PATH)
    rec   = KaldiRecognizer(model, SAMPLE_RATE)
    rec.SetWords(True)
    print("[VOSK] Offline recognition ready")
    while True:
        chunk = audio_queue.get()
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
    print(f"[AUTH] Access PIN: {ACCESS_PIN}  — change ACCESS_PIN in app.py")

    # Thread 2 — Vosk
    threading.Thread(target=vosk_worker,        name="VoskWorker",    daemon=True).start()
    # Thread 3 — SSE broadcaster
    threading.Thread(target=_broadcaster_thread, name="SSEBroadcaster", daemon=True).start()

    ssl_ctx = ("cert.pem", "key.pem") if Path("cert.pem").exists() else None
    if not ssl_ctx:
        print("[WARN] No cert.pem — HTTP only. Browser may block mic.")
        print("  openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem"
              " -days 365 -nodes -subj '/CN=10.42.0.1'")

    app.run(
        host         = "0.0.0.0",
        port         = 443 if ssl_ctx else 5000,
        ssl_context  = ssl_ctx,
        debug        = False,
        use_reloader = False,
    )
