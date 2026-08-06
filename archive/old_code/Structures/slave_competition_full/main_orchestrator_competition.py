#!/usr/bin/env python3
"""
main_orchestrator_competition.py  v2  —  S.A.F.E. Slave Competition Edition
============================================================================

What this file does differently from the generic orchestrator
-------------------------------------------------------------
1. STEPWISE MOVEMENT
   Each step: fly to next cell → hover STEP_HOVER_S → capture ROLLING_WINDOW
   frames → run surface-disc vision → continue.  The drone pauses briefly at
   every 0.5 m cell so the sensor integrates before moving on.
   Vision runs on the captured stack synchronously at each step, not in a
   parallel consumer coroutine.

   Grid marking is PASSIVE — it happens from the UDP telemetry position as
   the drone moves, NOT from exact step coordinates.  GPS drift is fine.

2. VIRTUAL GRID  (grid_map.py)
   mark_position() is called from udp_broadcast_loop every telemetry tick.
   The grid paints a fuzzy footprint disc around the reported position.
   Flight loop never reads or depends on the grid.

3. SIDEWAYS MOVEMENT ORDERING
   Between passes the drone waits for a SIDE_MOVE TCP command from the master.
   Master sends SIDE_MOVE to slaves in lead→mid→tail order (direction-aware).
   Each slave unblocks, repositions to new lane X, then continues scanning.

4. TF LUNA LIDAR FAILSAFE  (tf_luna_failsafe.py)
   Background asyncio task.  If obstacle < 1.0 m for 3 consecutive frames:
     x < 19.85 m → sidestep WEST  (clear of pole)
     x ≥ 19.85 m → sidestep EAST  (clear of statue)
   3-second cooldown.  Mission continues after sidestep.

5. MAP SHOWS ONLY COVERED AREA
   UDP packets include a grid snapshot every 5 packets (~1 Hz).
   Master merges snapshots from all three slaves.
   Frontend renders only cells present in the merged snapshot.
"""

import asyncio
import heapq
import importlib
import json
import math
import os
import subprocess
import sys
import threading
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
#  LOGGING
# ─────────────────────────────────────────────────────────────────────────────
def log(component, status, msg):
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}][{component}][{status}] {msg}", flush=True)

# ─────────────────────────────────────────────────────────────────────────────
#  PATHS
# ─────────────────────────────────────────────────────────────────────────────
BASE_DIR   = Path(__file__).resolve().parent
CONFIG_DIR = BASE_DIR / "config"
FPN_PATH   = CONFIG_DIR / "fpn_pattern.npy"
A4_PATH    = CONFIG_DIR / "A4_map.json"
MLX_BIN    = BASE_DIR / "bin" / "mlx_stdout"

# ─────────────────────────────────────────────────────────────────────────────
#  IDENTITY
# ─────────────────────────────────────────────────────────────────────────────
DRONE_ID  = os.environ.get("DRONE_ID", "slave_1")
MASTER_IP = os.environ.get("MASTER_IP", "10.42.0.1")

# ─────────────────────────────────────────────────────────────────────────────
#  FLIGHT PARAMETERS
# ─────────────────────────────────────────────────────────────────────────────
CRUISE_ALT_M         = 1.5
STEP_M               = 0.5    # one grid cell forward
STEP_HOVER_S         = 0.6    # pause at each step before capturing
MAVSDK_ADDRESS       = os.environ.get("MAVSDK_ADDR", "tcp://127.0.0.1:5760")
CONFIDENCE_THRESHOLD = 0.45
ROLLING_WINDOW       = 10     # frames captured per step
PERSIST_WINDOW       = 12     # frames for persistence re-check

# ─────────────────────────────────────────────────────────────────────────────
#  NETWORK PORTS
# ─────────────────────────────────────────────────────────────────────────────
UDP_TELEMETRY_PORT = 14550
TCP_COMMAND_PORT   = 14560
MINE_REPORT_PORT   = 5000

# ─────────────────────────────────────────────────────────────────────────────
#  MODULE IMPORTS
# ─────────────────────────────────────────────────────────────────────────────
sys.path.insert(0, str(BASE_DIR))

try:
    surface_filter = importlib.import_module("06_surface_filter")
    coord_math     = importlib.import_module("04_coordinate_math")
    verifier       = importlib.import_module("05_map_verifier")
    comms_mod      = importlib.import_module("08_comms_link")
    persistence    = importlib.import_module("03_persistence")
    log("INIT", "OK", "Pipeline modules imported")
except Exception as e:
    log("INIT", "ERROR", f"Module import failed: {e}"); sys.exit(1)

try:
    from fieldmap import (FIELD, SCAN_Y_START, SCAN_Y_END, SCAN_STEP_M,
                          generate_a4_map_json, local_to_gps, gps_to_local,
                          lane_x_for_pass, NUM_PASSES, PASS_LANES)
    log("INIT", "OK", "fieldmap imported")
except Exception as e:
    log("INIT", "ERROR", f"fieldmap import failed: {e}"); sys.exit(1)

try:
    from grid_map import GRID
    _GRID_OK = True
    log("INIT", "OK", "grid_map imported")
except Exception as e:
    GRID = None; _GRID_OK = False
    log("INIT", "WARN", f"grid_map unavailable: {e}")

try:
    from tf_luna_failsafe import LidarFailsafe
    _LIDAR_OK = True
    log("INIT", "OK", "tf_luna_failsafe imported")
except Exception as e:
    LidarFailsafe = None; _LIDAR_OK = False
    log("INIT", "WARN", f"tf_luna_failsafe unavailable: {e}")

try:
    from udp_channel import UDPSender
    from tcp_channel  import TCPCommandServer
    log("INIT", "OK", "udp_channel + tcp_channel imported")
except Exception as e:
    log("INIT", "ERROR", f"Comms import failed: {e}"); sys.exit(1)

# ─────────────────────────────────────────────────────────────────────────────
#  DATA CLASSES
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class DiscCandidate:
    drone_lat: float; drone_lon: float; altitude: float
    dx: int; dy: int; conf: float; raw_stack: np.ndarray

# ─────────────────────────────────────────────────────────────────────────────
#  GLOBAL MISSION STATE
# ─────────────────────────────────────────────────────────────────────────────
mission_paused   = False
mission_land     = False
confirmed_discs: list[tuple[float, float]] = []

_flag_counter  = 0
_flag_last_len = 0

def _recent_detection_flag():
    global _flag_counter, _flag_last_len
    cur = len(confirmed_discs)
    if cur > _flag_last_len:
        _flag_last_len = cur; _flag_counter = 15
    if _flag_counter > 0:
        _flag_counter -= 1; return True
    return False

# Set in main() after event loop starts — lets flight_loop await SIDE_MOVE
_side_move_event: Optional[asyncio.Event] = None

# ─────────────────────────────────────────────────────────────────────────────
#  A* PATH PLANNER  (returns every 0.5 m step, unsimplified)
# ─────────────────────────────────────────────────────────────────────────────
def plan_lane_steps(lane_x, y_start, y_end, step=STEP_M):
    """Returns every 0.5 m step cell along a hazard-free South-going lane."""
    log("PLAN", "INFO", f"x={lane_x:.1f}m  y=[{y_start:.1f}→{y_end:.1f}]m")
    sx = round(lane_x / step) * step
    sy = round(y_start / step) * step
    ey = round(y_end   / step) * step

    heap = [(0.0, (sx, sy))]
    came: dict = {}
    g: dict = {(sx, sy): 0.0}
    moves = [(0,-step),(-step,-step),(step,-step),
             (-step,0),(step,0),(0,step),(-step,step),(step,step)]
    itr = 0

    while heap:
        itr += 1
        if itr > 200_000:
            log("PLAN", "WARN", "A* iteration limit"); break
        _, cur = heapq.heappop(heap)
        cx, cy = cur
        if cy <= ey:
            path = []
            node = cur
            while node in came:
                path.append(node); node = came[node]
            path.reverse()
            log("PLAN", "OK", f"{len(path)} steps  (iters={itr})")
            return path
        for dx, dy in moves:
            nx, ny = round(cx+dx,3), round(cy+dy,3)
            if not FIELD.is_safe(nx, ny, drone_margin=0.3):
                continue
            ng = g[cur] + math.hypot(dx, dy)
            h  = abs(ny - ey) + 0.5 * abs(nx - lane_x)
            if (nx, ny) not in g or ng < g[(nx, ny)]:
                g[(nx, ny)] = ng; came[(nx, ny)] = cur
                heapq.heappush(heap, (ng + h, (nx, ny)))

    raise ValueError(f"A* failed: lane x={lane_x:.1f} y=[{y_start:.1f}→{y_end:.1f}]")

# ─────────────────────────────────────────────────────────────────────────────
#  HELPERS
# ─────────────────────────────────────────────────────────────────────────────
def _haversine(lat1, lon1, lat2, lon2):
    R = 6_378_137.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = math.sin(math.radians(lat2-lat1)/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(math.radians(lon2-lon1)/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

# ─────────────────────────────────────────────────────────────────────────────
#  C++ CAMERA PIPE
# ─────────────────────────────────────────────────────────────────────────────
class PipeCamera:
    FRAME_BYTES = 768 * 4
    def __init__(self):
        if not MLX_BIN.exists():
            raise FileNotFoundError(f"MLX binary missing: {MLX_BIN}")
        self.proc = subprocess.Popen([str(MLX_BIN)],
                                      stdout=subprocess.PIPE, stderr=sys.stderr)
        log("CAMERA", "OK", f"C++ pipe PID {self.proc.pid}")

    def read_frame(self):
        raw = self.proc.stdout.read(self.FRAME_BYTES)
        if len(raw) != self.FRAME_BYTES: return None
        return np.frombuffer(raw, dtype=np.float32).reshape((24,32))

    def capture_stack_sync(self, n=ROLLING_WINDOW):
        frames = [f for _ in range(n) if (f := self.read_frame()) is not None]
        return np.array(frames) if frames else None

    def terminate(self):
        self.proc.terminate(); self.proc.wait()

# ─────────────────────────────────────────────────────────────────────────────
#  MOVEMENT BLOCK
# ─────────────────────────────────────────────────────────────────────────────
class MovementBlock:
    def __init__(self, addr=MAVSDK_ADDRESS):
        from mavsdk import System
        self.drone           = System()
        self._address        = addr
        self.current_lat     = 0.0
        self.current_lon     = 0.0
        self.current_alt     = 0.0      # AGL relative
        self.current_alt_amsl = 0.0     # AMSL absolute — needed for goto_location
        self.current_heading = 0.0
        self.current_east_m  = 0.0
        self.current_north_m = 0.0
        self.is_armed        = False
        self.is_airborne     = False
        self._telem_ready    = asyncio.Event()

    async def connect(self):
        log("MOVE", "INFO", f"Connecting FC @ {self._address}")
        await self.drone.connect(system_address=self._address)
        async for s in self.drone.core.connection_state():
            if s.is_connected: log("MOVE","OK","FC connected"); break
        asyncio.create_task(self._start_telem())
        asyncio.create_task(self._safety_monitor())
        await self._telem_ready.wait()
        log("MOVE","OK",f"GPS: ({self.current_lat:.6f},{self.current_lon:.6f})")

    async def _start_telem(self):
        async def _pos():
            async for p in self.drone.telemetry.position():
                self.current_lat      = p.latitude_deg
                self.current_lon      = p.longitude_deg
                self.current_alt      = p.relative_altitude_m   # AGL
                self.current_alt_amsl = p.absolute_altitude_m   # AMSL for goto_location
                self._telem_ready.set()
        async def _ned():
            async for n in self.drone.telemetry.position_velocity_ned():
                self.current_east_m  = n.position.east_m
                self.current_north_m = n.position.north_m
        async def _hdg():
            async for h in self.drone.telemetry.heading():
                self.current_heading = h.heading_deg
        async def _landed():
            from mavsdk.telemetry import LandedState
            async for s in self.drone.telemetry.landed_state():
                self.is_airborne = (s != LandedState.ON_GROUND)
        async def _armed():
            async for a in self.drone.telemetry.armed():
                self.is_armed = a
        for coro in (_pos, _ned, _hdg, _landed, _armed):
            asyncio.create_task(coro())

    async def _safety_monitor(self):
        while True:
            await asyncio.sleep(0.5)
            if self.is_airborne and 0 < self.current_alt < 0.3:
                log("SAFETY","WARN",f"Alt low: {self.current_alt:.2f}m")

    def get_telemetry(self):
        return self.current_lat, self.current_lon, self.current_alt

    def _agl_to_amsl(self, desired_agl: float) -> float:
        """Convert desired AGL altitude to AMSL for goto_location().
        ArduPilot's goto_location() requires AMSL, not AGL.
        Computes: ground_amsl = current_amsl - current_agl, then adds desired_agl."""
        if self.current_alt_amsl == 0.0 or self.current_alt == 0.0:
            return desired_agl + 10.0   # fallback if telemetry not ready yet
        ground_amsl = self.current_alt_amsl - self.current_alt
        return ground_amsl + desired_agl

    async def takeoff_to_hover(self, alt=CRUISE_ALT_M):
        await self.drone.action.set_takeoff_altitude(alt)  # AGL — correct for takeoff
        await self.drone.action.arm()
        log("MOVE","OK","Armed")
        await self.drone.action.takeoff()
        while self.current_alt < alt * 0.85:
            await asyncio.sleep(0.2)
        log("MOVE","OK",f"Hover at {self.current_alt:.1f}m")

    async def move_to(self, lat, lon, alt=CRUISE_ALT_M, tol=0.4):
        """Fly to lat/lon at alt metres AGL. Converts to AMSL internally."""
        amsl = self._agl_to_amsl(alt)
        await self.drone.action.goto_location(lat, lon, amsl, float("nan"))
        timeout = 0
        while True:
            dist = _haversine(self.current_lat, self.current_lon, lat, lon)
            if dist < tol: break
            if mission_land: await self.land(); return
            await asyncio.sleep(0.15)
            timeout += 1
            if timeout >= 400:
                log("MOVE","WARN",f"Timeout — dist={dist:.1f}m, continuing"); break

    async def force_hover(self, duration=1.0):
        try: await self.drone.action.hold()
        except Exception: pass
        if duration > 0: await asyncio.sleep(duration)

    async def land(self):
        log("MOVE","INFO","LAND"); await self.drone.action.land()

    async def emergency_sidestep(self, direction, dist_m=2.0):
        """Immediate lateral escape. direction = 'west' or 'east'."""
        log("MOVE","WARN",f"SIDESTEP {direction.upper()} {dist_m:.1f}m")
        await self.force_hover(0)
        dx_m = dist_m if direction == "east" else -dist_m
        lat_ref = math.radians(self.current_lat)
        delta_lon = math.degrees(dx_m / (6_378_137.0 * math.cos(lat_ref)))
        await self.move_to(self.current_lat, self.current_lon + delta_lon,
                           self.current_alt, tol=0.5)
        await self.force_hover(1.5)
        log("MOVE","OK","Sidestep complete")

# ─────────────────────────────────────────────────────────────────────────────
#  DISC CANDIDATE HANDLER
# ─────────────────────────────────────────────────────────────────────────────
async def _handle_candidate(candidate, cam, movement, tunnel, fpn_pattern, loop):
    log("DISC","INFO",f"Verifying conf={candidate.conf*100:.1f}%")
    await movement.force_hover(1.0)
    restack = await loop.run_in_executor(None, cam.capture_stack_sync, PERSIST_WINDOW)
    if restack is None:
        log("DISC","WARN","Re-capture failed"); return

    def _check():
        dx, dy, fc = surface_filter.process_surface_stack(
            restack, fpn_pattern, altitude_m=candidate.altitude)
        ok = persistence.PersistenceFilter(max_drift_meters=1.5).verify(
            dx, dy, candidate.altitude)
        return ok, dx, dy, fc

    try:
        ok, new_dx, new_dy, final_conf = await loop.run_in_executor(None, _check)
    except Exception as e:
        log("DISC","ERROR",f"Persistence check: {e}"); return

    if not ok or final_conf < CONFIDENCE_THRESHOLD:
        log("DISC","INFO",f"GHOST persist={ok} conf={final_conf*100:.1f}%"); return

    lat, lon, alt = movement.get_telemetry()
    try:
        lx, ly = coord_math.get_pixels_to_meters(new_dx, new_dy, alt)
        tlat, tlon = coord_math.compute_global_gps(lat, lon, lx, ly)
    except Exception as e:
        log("DISC","ERROR",f"GPS math: {e}"); return

    for i, (dlat, dlon) in enumerate(confirmed_discs):
        if _haversine(tlat, tlon, dlat, dlon) < 1.5:
            log("DISC","INFO",f"Duplicate — disc #{i}"); return

    confirmed_discs.append((tlat, tlon))
    log("DISC","OK",
        f"*** DISC #{len(confirmed_discs)} CONFIRMED ***  "
        f"GPS=({tlat:.6f},{tlon:.6f})  conf={final_conf*100:.1f}%")

    # Mark grid detection at confirmed position
    if GRID is not None:
        det_x, det_y = gps_to_local(tlat, tlon)
        GRID.mark_detection(det_x, det_y)

    def _report():
        try:
            if verifier.verify_and_log(tlat, tlon, final_conf):
                tunnel.send_anomaly_data(tlat, tlon, restack)
                log("DISC","OK","Reported to master")
        except Exception as e:
            log("DISC","ERROR",f"Report: {e}\n{traceback.format_exc()}")
    threading.Thread(target=_report, daemon=True).start()

# ─────────────────────────────────────────────────────────────────────────────
#  FLIGHT LOOP  — stepwise multi-pass
# ─────────────────────────────────────────────────────────────────────────────
async def flight_loop(movement, cam, tunnel, udp_sender, fpn_pattern):
    loop = asyncio.get_running_loop()

    # Write A4 map of pre-known buried mines
    try:
        a4 = generate_a4_map_json()
        A4_PATH.parent.mkdir(parents=True, exist_ok=True)
        A4_PATH.write_text(json.dumps(a4, indent=2))
        log("FLIGHT","OK",f"A4 map: {len(a4['mines'])} mines")
    except Exception as e:
        log("FLIGHT","WARN",f"A4 map: {e}")

    # Pre-plan all passes before takeoff
    all_passes: list[list[tuple[float,float]]] = []
    for pn in range(1, NUM_PASSES+1):
        lx = lane_x_for_pass(DRONE_ID, pn)
        try:
            steps = plan_lane_steps(lx, SCAN_Y_START, SCAN_Y_END, STEP_M)
            all_passes.append(steps)
            log("FLIGHT","INFO",f"Pass {pn}/{NUM_PASSES}  x={lx:.1f}m  steps={len(steps)}")
        except ValueError as e:
            log("FLIGHT","ERROR",f"Pass {pn} planning failed: {e}"); return

    log("FLIGHT","OK",
        f"All {NUM_PASSES} passes planned  "
        f"total_steps={sum(len(p) for p in all_passes)}")

    await movement.takeoff_to_hover(CRUISE_ALT_M)
    log("FLIGHT","OK","Airborne — stepwise scan starting")

    detection_queue: asyncio.Queue[DiscCandidate] = asyncio.Queue(maxsize=4)
    steps_done = 0

    for pass_num, steps in enumerate(all_passes, start=1):
        lane_x = lane_x_for_pass(DRONE_ID, pass_num)
        log("FLIGHT","INFO",
            f"=== PASS {pass_num}/{NUM_PASSES}  x={lane_x:.1f}m  {len(steps)} steps ===")

        # ── Wait for SIDE_MOVE command before repositioning (pass > 1) ────
        if pass_num > 1:
            log("FLIGHT","INFO","Awaiting SIDE_MOVE from master…")
            if _side_move_event is not None:
                _side_move_event.clear()
                await _side_move_event.wait()

            # Fly to start of new lane
            start_lat, start_lon = local_to_gps(lane_x, SCAN_Y_START)
            await movement.move_to(start_lat, start_lon, CRUISE_ALT_M, tol=0.4)
            log("FLIGHT","OK",f"Repositioned to lane x={lane_x:.1f}m")

        # ── Step through each cell in this pass ───────────────────────────
        for step_x, step_y in steps:

            if mission_land:
                log("FLIGHT","WARN","LAND flag"); await movement.land(); return

            while mission_paused:
                await asyncio.sleep(0.1)

            if FIELD.is_forbidden(step_x, step_y):
                log("FLIGHT","WARN",f"({step_x:.1f},{step_y:.1f}) forbidden — skip")
                continue

            # 1. Fly to step cell centre
            wp_lat, wp_lon = local_to_gps(step_x, step_y)
            await movement.move_to(wp_lat, wp_lon, CRUISE_ALT_M, tol=0.35)

            # 2. Hover briefly so sensor settles
            await movement.force_hover(STEP_HOVER_S)

            # 3. Capture frame stack at this position
            stack = await loop.run_in_executor(
                None, cam.capture_stack_sync, ROLLING_WINDOW)
            steps_done += 1

            # 4. Run surface-disc vision on captured frames
            if stack is not None:
                def _run_vision(s=stack):
                    return surface_filter.process_surface_stack(
                        s, fpn_pattern, altitude_m=CRUISE_ALT_M)
                try:
                    dx, dy, conf = await loop.run_in_executor(None, _run_vision)
                except Exception as e:
                    log("SCAN","ERROR",f"Vision: {e}"); dx = None

                if dx is not None and conf >= CONFIDENCE_THRESHOLD:
                    lat, lon, alt = movement.get_telemetry()
                    log("SCAN","OK",
                        f"DISC CANDIDATE  conf={conf*100:.1f}%  "
                        f"step=({step_x:.1f},{step_y:.1f})")
                    cand = DiscCandidate(
                        drone_lat=lat, drone_lon=lon, altitude=alt,
                        dx=dx, dy=dy, conf=conf, raw_stack=stack)
                    if not detection_queue.full():
                        await detection_queue.put(cand)

            # 5. Handle any queued detection (non-blocking)
            try:
                cand = detection_queue.get_nowait()
                await _handle_candidate(cand, cam, movement, tunnel,
                                        fpn_pattern, loop)
            except asyncio.QueueEmpty:
                pass

            if steps_done % 20 == 0:
                log("FLIGHT","INFO",
                    f"Pass {pass_num}/{NUM_PASSES}  "
                    f"pos=({step_x:.1f},{step_y:.1f})  "
                    f"discs={len(confirmed_discs)}")

        log("FLIGHT","OK",f"Pass {pass_num} done  discs={len(confirmed_discs)}")

    log("FLIGHT","OK",
        f"ALL {NUM_PASSES} PASSES DONE  steps={steps_done}  discs={len(confirmed_discs)}")
    await movement.land()

# ─────────────────────────────────────────────────────────────────────────────
#  UDP BROADCAST  — 5 Hz telemetry + grid snapshot at ~1 Hz
#  Grid is marked here from real GPS position (drift-tolerant, passive)
# ─────────────────────────────────────────────────────────────────────────────
async def udp_broadcast_loop(movement, sender):
    log("UDP","OK",f"→ {MASTER_IP}:{UDP_TELEMETRY_PORT} @ 5 Hz")
    tick = 0
    while not mission_land:
        sensor = 0.9 if _recent_detection_flag() else 0.0
        pkt = {
            "lat":      movement.current_lat,
            "lng":      movement.current_lon,
            "altitude": movement.current_alt,
            "heading":  movement.current_heading,
            "speed":    0.5,
            "armed":    movement.is_armed,
            "airborne": movement.is_airborne,
            "bat_pct":  100,
            "sensor":   sensor,
        }

        # Passively mark the grid from real GPS position every tick
        # This is the ONLY place mark_position is called — driven by actual
        # telemetry, not by planned step coordinates.
        if GRID is not None and movement.current_lat != 0.0:
            try:
                lx, ly = gps_to_local(movement.current_lat, movement.current_lon)
                GRID.mark_position(lx, ly)
            except Exception:
                pass

        # Attach grid snapshot ~1 Hz (every 5th packet)
        tick += 1
        if tick % 5 == 0 and GRID is not None:
            pkt["grid"] = GRID.snapshot()

        sender.send(pkt)
        await asyncio.sleep(0.2)

# ─────────────────────────────────────────────────────────────────────────────
#  TCP COMMAND SERVER
# ─────────────────────────────────────────────────────────────────────────────
def start_tcp_command_server(movement, loop: asyncio.AbstractEventLoop):
    """
    Starts the TCP command server in a background thread.
    `loop` must be the running asyncio event loop — passed in from async main()
    because asyncio.get_running_loop() raises RuntimeError when called from
    the TCPCommandServer's background thread.
    """
    server = TCPCommandServer(drone_id=DRONE_ID, port=TCP_COMMAND_PORT)

    @server.on_command("GOTO")
    def h_goto(cmd):
        lat, lon = float(cmd["lat"]), float(cmd["lng"])
        alt = float(cmd.get("alt", CRUISE_ALT_M))
        asyncio.run_coroutine_threadsafe(
            movement.move_to(lat, lon, alt), loop)
        return {"status": "ok"}

    @server.on_command("PAUSE")
    def h_pause(cmd):
        global mission_paused; mission_paused = True
        asyncio.run_coroutine_threadsafe(
            movement.force_hover(0), loop)
        log("TCP","OK","PAUSE"); return {"status": "ok"}

    @server.on_command("RESUME")
    def h_resume(cmd):
        global mission_paused; mission_paused = False
        log("TCP","OK","RESUME"); return {"status": "ok"}

    @server.on_command("LAND")
    def h_land(cmd):
        global mission_land; mission_land = True
        asyncio.run_coroutine_threadsafe(
            movement.land(), loop)
        log("TCP","OK","LAND"); return {"status": "ok"}

    @server.on_command("SIDE_MOVE")
    def h_side_move(cmd):
        """Master has decided it's this drone's turn to shift lanes.
        Sets the asyncio event so flight_loop unblocks and repositions."""
        log("TCP","OK",f"SIDE_MOVE seq={cmd.get('seq','?')} — unblocking")
        if _side_move_event is not None:
            asyncio.run_coroutine_threadsafe(
                _trigger_side_move(), loop)
        return {"status": "ok", "drone_id": DRONE_ID}

    @server.on_command("ARM_TAKEOFF")
    def h_arm_takeoff(cmd):
        alt = float(cmd.get("alt", 1.5))
        asyncio.run_coroutine_threadsafe(
            movement.takeoff_to_hover(alt), loop)
        return {"status": "ok"}

    @server.on_command("ARM_ONLY")
    def h_arm_only(cmd):
        async def _a(): await movement.drone.action.arm()
        asyncio.run_coroutine_threadsafe(_a(), loop)
        return {"status": "ok"}

    @server.on_command("DISARM")
    def h_disarm(cmd):
        async def _d(): await movement.drone.action.disarm()
        asyncio.run_coroutine_threadsafe(_d(), loop)
        return {"status": "ok"}

    server.start(blocking=False)
    log("TCP","OK",f"Command server :{TCP_COMMAND_PORT}")
    return server

async def _trigger_side_move():
    if _side_move_event is not None:
        _side_move_event.set()

# ─────────────────────────────────────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────
async def main():
    global _side_move_event
    _side_move_event = asyncio.Event()

    print("=" * 64, flush=True)
    print(f"  S.A.F.E. COMPETITION SLAVE v2  —  {DRONE_ID}", flush=True)
    print(f"  Passes: {NUM_PASSES}  Step hover: {STEP_HOVER_S}s  Alt: {CRUISE_ALT_M}m", flush=True)
    print(f"  Master: {MASTER_IP}", flush=True)
    print(f"  Grid tracking: {'ON' if _GRID_OK else 'OFF (import failed)'}  "
          f"Lidar: {'ON' if _LIDAR_OK else 'OFF (import failed)'}", flush=True)
    print("=" * 64, flush=True)
    FIELD.print_summary()

    fpn_pattern = None
    if FPN_PATH.exists():
        fpn_pattern = np.load(str(FPN_PATH))
        log("INIT","OK",f"FPN shape={fpn_pattern.shape}")
    else:
        log("INIT","WARN","FPN missing — run 00_preflight_calib.py")

    try:
        cam = PipeCamera()
    except FileNotFoundError:
        log("INIT","ERROR","Camera binary missing"); return

    tunnel   = comms_mod.DroneTunnel(target_ip=MASTER_IP, target_port=MINE_REPORT_PORT)
    movement = MovementBlock(addr=MAVSDK_ADDRESS)

    try:
        await movement.connect()
    except Exception as e:
        log("INIT","ERROR",f"MAVSDK: {e}"); cam.terminate(); return

    try:
        udp_sender = UDPSender(
            drone_id=DRONE_ID, master_ip=MASTER_IP, master_port=UDP_TELEMETRY_PORT)
    except ValueError as e:
        log("INIT","ERROR",f"UDPSender: {e}"); cam.terminate(); return

    try:
        start_tcp_command_server(movement, asyncio.get_running_loop())
    except Exception as e:
        log("INIT","ERROR",f"TCP server: {e}"); cam.terminate(); return

    # TF Luna lidar failsafe — optional, graceful if absent
    lidar_task = None
    if LidarFailsafe is not None:
        lidar = LidarFailsafe(movement, drone_id=DRONE_ID)
        lidar_task = asyncio.create_task(lidar.monitor_loop(), name="lidar")
        log("INIT","OK","Lidar failsafe active")

    log("INIT","OK","Launching coroutines")
    try:
        await asyncio.gather(
            flight_loop(movement, cam, tunnel, udp_sender, fpn_pattern),
            udp_broadcast_loop(movement, udp_sender),
        )
    except KeyboardInterrupt:
        log("INIT","INFO","Operator abort")
    except Exception as e:
        log("INIT","ERROR",f"{e}\n{traceback.format_exc()}")
    finally:
        if lidar_task: lidar_task.cancel()
        cam.terminate()
        log("INIT","OK","Clean shutdown")

if __name__ == "__main__":
    asyncio.run(main())
