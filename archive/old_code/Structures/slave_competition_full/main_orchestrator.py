#!/usr/bin/env python3
"""
main_orchestrator.py  —  S.A.F.E. Slave Drone Mission Controller
=================================================================
Three concurrent async coroutines — all blocking work in executors.

Log format:  [COMPONENT][STATUS] message
  Components: INIT, CAMERA, MOVE, FEEDER, VISION, FLIGHT, UDP, TCP, MINE, SAFETY
  Status:     OK, WARN, ERROR, INFO
"""

import asyncio
import os
import subprocess
import sys
import time
import threading
import importlib
import math
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
#  LOGGING HELPER  — every print in this file uses this
# ─────────────────────────────────────────────────────────────────────────────
def log(component: str, status: str, msg: str) -> None:
    """
    Prints a timestamped, tagged log line to stdout.
    status: OK | WARN | ERROR | INFO
    Example:  [12:34:56][VISION][OK] Target conf=92.1% at (20.296, 85.824)
    """
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}][{component}][{status}] {msg}", flush=True)


# ─────────────────────────────────────────────────────────────────────────────
#  PATHS  — dynamic, works on any username (drone1 / drone2 / drone3)
# ─────────────────────────────────────────────────────────────────────────────
BASE_DIR    = Path(__file__).resolve().parent
CONFIG_DIR  = BASE_DIR / "config"
LOG_DIR     = BASE_DIR / "logs"
FPN_PATH    = CONFIG_DIR / "fpn_pattern.npy"
ORIGIN_PATH = CONFIG_DIR / "origin_state.json"
MLX_BIN     = BASE_DIR / "bin" / "mlx_stdout"

# ─────────────────────────────────────────────────────────────────────────────
#  IDENTITY  — set via env var before running
#    drone1:  export DRONE_ID=slave_1
#    drone2:  export DRONE_ID=slave_2
#    drone3:  export DRONE_ID=slave_3
# ─────────────────────────────────────────────────────────────────────────────
DRONE_ID  = os.environ.get("DRONE_ID", "slave_1")
MASTER_IP = os.environ.get("MASTER_IP", "10.42.0.1")

STRIP_Y_M = {"slave_1": -4.45, "slave_2": 0.0, "slave_3": +4.45}.get(DRONE_ID, 0.0)

# ─────────────────────────────────────────────────────────────────────────────
#  FLIGHT + VISION PARAMETERS
# ─────────────────────────────────────────────────────────────────────────────
CRUISE_ALT_M         = 1.5
FORWARD_SPEED_MS     = 0.5
FIELD_LENGTH_M       = 100.0
HOVER_BEFORE_CHECK   = 1.0
MAVSDK_ADDRESS       = os.environ.get("MAVSDK_ADDR", "tcp://127.0.0.1:5760")
CONFIDENCE_THRESHOLD = 0.40   # FIXED: was 0.80 — new 02_vision_filter scores mines at 0.40-0.65.
                               # The old filter used circularity×0.95 which saturated near 0.95,
                               # making 0.80 work. The new composite scorer (circ+size+delta)
                               # never reaches 0.80 — 0.80 silently rejected every detection.
                               # See INTEGRATION NOTE in 02_vision_filter.py.
ROLLING_WINDOW       = 10
PERSIST_WINDOW       = 10

# ─────────────────────────────────────────────────────────────────────────────
#  NETWORK PORTS
# ─────────────────────────────────────────────────────────────────────────────
UDP_TELEMETRY_PORT = 14550   # master listens here (udp_telemetry.py)
TCP_COMMAND_PORT   = 14560   # this slave listens here (tcp_commander connects)
MINE_REPORT_PORT   = 5000    # master mine listener

# ─────────────────────────────────────────────────────────────────────────────
#  MODULE IMPORTS
# ─────────────────────────────────────────────────────────────────────────────
sys.path.insert(0, str(BASE_DIR))

try:
    vision      = importlib.import_module("02_vision_filter")
    coord_math  = importlib.import_module("04_coordinate_math")
    verifier    = importlib.import_module("05_map_verifier")
    comms       = importlib.import_module("08_comms_link")
    persistence = importlib.import_module("03_persistence")
    log("INIT", "OK", "Vision/logic modules imported")
except Exception as e:
    log("INIT", "ERROR", f"Failed to import pipeline modules: {e}")
    log("INIT", "ERROR", f"BASE_DIR={BASE_DIR}  — check filenames match exactly")
    sys.exit(1)

try:
    from udp_channel import UDPSender
    from tcp_channel import TCPCommandServer
    log("INIT", "OK", "udp_channel + tcp_channel imported")
except Exception as e:
    log("INIT", "ERROR", f"Failed to import swarm comms layer: {e}")
    log("INIT", "ERROR", "Ensure udp_channel.py and tcp_channel.py are in BASE_DIR")
    sys.exit(1)

# ─────────────────────────────────────────────────────────────────────────────
#  DATA CLASSES
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class MineCandidate:
    drone_lat:  float
    drone_lon:  float
    altitude:   float
    dx:         int
    dy:         int
    conf:       float
    raw_stack:  np.ndarray

# ─────────────────────────────────────────────────────────────────────────────
#  GLOBAL MISSION STATE
# ─────────────────────────────────────────────────────────────────────────────
mission_paused  = False
mission_land    = False
confirmed_mines: list[tuple[float, float]] = []

_mine_flag_counter  = 0
_mine_flag_last_len = 0

def _recent_mine_flag() -> bool:
    global _mine_flag_counter, _mine_flag_last_len
    current_len = len(confirmed_mines)
    if current_len > _mine_flag_last_len:
        _mine_flag_last_len = current_len
        _mine_flag_counter  = 15   # pulse sensor=0.9 for 3 seconds (15 × 200ms)
    if _mine_flag_counter > 0:
        _mine_flag_counter -= 1
        return True
    return False


# ─────────────────────────────────────────────────────────────────────────────
#  C++ CAMERA PIPE
# ─────────────────────────────────────────────────────────────────────────────
class PipeCamera:
    FRAME_BYTES = 768 * 4   # 768 floats × 4 bytes = 3072

    def __init__(self):
        if not MLX_BIN.exists():
            log("CAMERA", "ERROR", f"C++ binary not found: {MLX_BIN}")
            log("CAMERA", "ERROR", "Compile mlx_stdout.cpp first and place in bin/")
            raise FileNotFoundError(f"MLX binary missing: {MLX_BIN}")
        log("CAMERA", "INFO", f"Starting C++ pipe: {MLX_BIN}")
        self.proc = subprocess.Popen(
            [str(MLX_BIN)],
            stdout=subprocess.PIPE,
            stderr=sys.stderr,
        )
        log("CAMERA", "OK", f"C++ pipe started (PID {self.proc.pid})")

    def read_frame(self) -> Optional[np.ndarray]:
        """BLOCKING — must be called inside run_in_executor."""
        raw = self.proc.stdout.read(self.FRAME_BYTES)
        if len(raw) != self.FRAME_BYTES:
            log("CAMERA", "WARN",
                f"Short read: got {len(raw)} bytes, expected {self.FRAME_BYTES}. "
                f"Pipe may have died (returncode={self.proc.poll()})")
            return None
        return np.frombuffer(raw, dtype=np.float32).reshape((24, 32))

    def capture_stack_sync(self, num_frames: int = ROLLING_WINDOW) -> Optional[np.ndarray]:
        """BLOCKING — call via run_in_executor only."""
        frames = []
        for i in range(num_frames):
            f = self.read_frame()
            if f is not None:
                frames.append(f)
            else:
                log("CAMERA", "WARN", f"Null frame {i+1}/{num_frames} during stack capture")
        if not frames:
            log("CAMERA", "ERROR", "capture_stack_sync returned 0 valid frames")
            return None
        log("CAMERA", "OK", f"Captured stack of {len(frames)} frames")
        return np.array(frames)

    def terminate(self):
        log("CAMERA", "INFO", "Terminating C++ pipe...")
        self.proc.terminate()
        self.proc.wait()
        log("CAMERA", "OK", "C++ pipe terminated")


# ─────────────────────────────────────────────────────────────────────────────
#  MOVEMENT BLOCK (MAVSDK)
# ─────────────────────────────────────────────────────────────────────────────
class MovementBlock:
    def __init__(self, system_address: str = MAVSDK_ADDRESS):
        from mavsdk import System
        self.drone    = System()
        self._address = system_address
        self._connected = False

        self.current_lat:      float = 0.0
        self.current_lon:      float = 0.0
        self.current_alt:      float = 0.0    # AGL relative
        self.current_alt_amsl: float = 0.0    # AMSL absolute — required for goto_location
        self.current_heading:  float = 0.0
        self.current_north_m:  float = 0.0
        self.current_east_m:   float = 0.0
        self.current_down_m:   float = 0.0
        self.is_armed:    bool = False
        self.is_airborne: bool = False
        self._telem_ready = asyncio.Event()

    async def connect(self):
        log("MOVE", "INFO", f"Connecting to FC at {self._address}")
        log("MOVE", "INFO", "If this hangs, check MAVProxy is running and FC is powered")
        await self.drone.connect(system_address=self._address)
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                log("MOVE", "OK", "FC connected via MAVSDK")
                self._connected = True
                break
        asyncio.create_task(self._update_telemetry_loop(), name="telem_loop")
        asyncio.create_task(self._safety_monitor(),        name="safety_monitor")
        log("MOVE", "INFO", "Waiting for first GPS fix...")
        await self._telem_ready.wait()
        log("MOVE", "OK",
            f"GPS fix acquired: lat={self.current_lat:.6f} "
            f"lon={self.current_lon:.6f} alt={self.current_alt:.1f}m")

    async def _update_telemetry_loop(self):
        """
        Spawns 4 independent tasks immediately — one per telemetry stream.
        Each runs forever as its own coroutine so none can block the others.

        BUG FIXED: previously _ned/_hdg/_flags tasks were defined and created
        AFTER the position() async for loop — but that loop is infinite and
        never exits, so those tasks were never started.

        BUG FIXED: _flags had two sequential async for loops — the second
        (armed) could never run while the first (landed_state) was alive.
        Now each stream is its own independent task.
        """

        async def _pos():
            try:
                async for pos in self.drone.telemetry.position():
                    self.current_lat      = pos.latitude_deg
                    self.current_lon      = pos.longitude_deg
                    self.current_alt      = pos.relative_altitude_m   # AGL
                    self.current_alt_amsl = pos.absolute_altitude_m   # AMSL for goto_location
                    self._telem_ready.set()
            except Exception as e:
                log("MOVE", "ERROR", f"Position stream died: {e}")

        async def _ned():
            try:
                async for ned in self.drone.telemetry.position_velocity_ned():
                    self.current_north_m = ned.position.north_m
                    self.current_east_m  = ned.position.east_m
                    self.current_down_m  = ned.position.down_m
            except Exception as e:
                log("MOVE", "WARN", f"NED stream error: {e}")

        async def _hdg():
            try:
                async for h in self.drone.telemetry.heading():
                    self.current_heading = h.heading_deg
            except Exception as e:
                log("MOVE", "WARN", f"Heading stream error: {e}")

        async def _landed():
            # BUG FIXED: was one function with two sequential async for loops.
            # Second loop (armed) never ran while first (landed_state) was live.
            # Now each is its own task.
            try:
                from mavsdk.telemetry import LandedState
                async for status in self.drone.telemetry.landed_state():
                    self.is_airborne = (status != LandedState.ON_GROUND)
            except Exception as e:
                log("MOVE", "WARN", f"LandedState stream error: {e}")

        async def _armed():
            try:
                async for armed in self.drone.telemetry.armed():
                    self.is_armed = armed
            except Exception as e:
                log("MOVE", "WARN", f"Armed stream error: {e}")

        # All 5 tasks start concurrently — none blocks the others
        asyncio.create_task(_pos(),    name="telem_pos")
        asyncio.create_task(_ned(),    name="telem_ned")
        asyncio.create_task(_hdg(),    name="telem_hdg")
        asyncio.create_task(_landed(), name="telem_landed")
        asyncio.create_task(_armed(),  name="telem_armed")
        log("MOVE", "OK", "5 telemetry stream tasks started (pos/ned/hdg/landed/armed)")

    async def _safety_monitor(self):
        """Runs every 500ms — logs anything alarming."""
        check_count = 0
        while True:
            await asyncio.sleep(0.5)
            check_count += 1
            if self.is_airborne and self.current_alt < 0.3 and self.current_alt > 0.0:
                log("SAFETY", "WARN",
                    f"Altitude critically low: {self.current_alt:.2f}m — check terrain!")
            # Log position every 30 seconds so you can see it's alive
            if check_count % 60 == 0:
                log("SAFETY", "INFO",
                    f"Heartbeat — pos=({self.current_lat:.5f},{self.current_lon:.5f}) "
                    f"alt={self.current_alt:.1f}m armed={self.is_armed} air={self.is_airborne}")

    def get_current_telemetry(self) -> tuple[float, float, float]:
        return self.current_lat, self.current_lon, self.current_alt

    async def takeoff_to_hover(self, altitude: float = CRUISE_ALT_M):
        log("MOVE", "INFO", f"Setting takeoff altitude to {altitude}m")
        await self.drone.action.set_takeoff_altitude(altitude)
        log("MOVE", "INFO", "Arming...")
        try:
            await self.drone.action.arm()
            log("MOVE", "OK", "Armed")
        except Exception as e:
            log("MOVE", "ERROR", f"Arm failed: {e}  — is the drone in a safe state?")
            raise
        log("MOVE", "INFO", "Taking off...")
        await self.drone.action.takeoff()
        wait_count = 0
        while self.current_alt < altitude * 0.85:
            await asyncio.sleep(0.2)
            wait_count += 1
            if wait_count % 25 == 0:   # log every 5 seconds
                log("MOVE", "INFO",
                    f"Climbing... current alt={self.current_alt:.1f}m target={altitude}m")
        log("MOVE", "OK", f"Hover reached at {self.current_alt:.1f}m")

    async def move_to_coordinate(self, lat: float, lon: float,
                                  alt: float = CRUISE_ALT_M,
                                  tol_m: float = 0.5):
        dist_start = _haversine(self.current_lat, self.current_lon, lat, lon)
        log("MOVE", "INFO",
            f"Flying to ({lat:.6f},{lon:.6f})  alt={alt:.1f}m  "
            f"dist={dist_start:.1f}m from current position")
        try:
            await self.drone.action.goto_location(lat, lon, alt + self._amsl_offset(), 0.0)
        except Exception as e:
            log("MOVE", "ERROR", f"goto_location failed: {e}")
            raise
        wait_count = 0
        timeout_count = 0
        MAX_WAIT_S = 60   # give up after 60s — avoids infinite hang if GPS drifts
        while True:
            dist = _haversine(self.current_lat, self.current_lon, lat, lon)
            if dist < tol_m:
                log("MOVE", "OK", f"Waypoint reached (dist={dist:.2f}m < tol={tol_m}m)")
                break
            if mission_land:
                log("MOVE", "WARN", "LAND flag set mid-flight — aborting move")
                await self.land()
                return
            await asyncio.sleep(0.2)
            wait_count  += 1
            timeout_count += 1
            if wait_count % 25 == 0:
                log("MOVE", "INFO", f"En route... {dist:.1f}m remaining")
            if timeout_count >= MAX_WAIT_S * 5:   # 5 checks/sec × 60s
                log("MOVE", "WARN",
                    f"Waypoint timeout after {MAX_WAIT_S}s — dist still {dist:.1f}m. "
                    f"Continuing anyway (GPS drift or wind).")
                break

    async def force_hover(self, duration: float = 1.0):
        log("MOVE", "INFO", f"Holding position for {duration}s")
        try:
            await self.drone.action.hold()
        except Exception as e:
            log("MOVE", "WARN", f"Hold command failed: {e}")
        await asyncio.sleep(duration)

    async def land(self):
        log("MOVE", "INFO", "LAND command sent to FC")
        try:
            await self.drone.action.land()
            log("MOVE", "OK", "Landing initiated")
        except Exception as e:
            log("MOVE", "ERROR", f"Land command failed: {e}")

    def _amsl_offset(self) -> float:
        """Return ground AMSL so goto_location() gets absolute altitude, not AGL.
        ArduPilot's goto_location() requires AMSL. Caller passes desired AGL;
        we add ground_amsl = current_amsl - current_agl to get target AMSL."""
        if self.current_alt_amsl == 0.0 or self.current_alt == 0.0:
            return 10.0   # safe fallback if telemetry not ready yet
        return self.current_alt_amsl - self.current_alt   # = ground AMSL

    async def fly_forward_step(self, step_m: float = 1.0):
        delta_lat = step_m / 111_320.0
        new_lat = self.current_lat + delta_lat
        await self.move_to_coordinate(new_lat, self.current_lon, tol_m=0.4)


# ─────────────────────────────────────────────────────────────────────────────
#  HELPER
# ─────────────────────────────────────────────────────────────────────────────
def _haversine(lat1, lon1, lat2, lon2) -> float:
    R = 6_378_137.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a  = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))


# ─────────────────────────────────────────────────────────────────────────────
#  COROUTINE 1 — Frame feeder
# ─────────────────────────────────────────────────────────────────────────────
async def frame_feeder(cam: PipeCamera, frame_queue: asyncio.Queue):
    loop = asyncio.get_running_loop()
    log("FEEDER", "OK", f"Frame feed loop started for {DRONE_ID}")
    frames_read  = 0
    frames_dropped = 0
    null_streak  = 0

    while True:
        if mission_land:
            log("FEEDER", "INFO", "LAND flag — stopping frame feeder")
            break
        frame = await loop.run_in_executor(None, cam.read_frame)
        if frame is not None:
            null_streak = 0
            frames_read += 1
            if frame_queue.full():
                try:
                    frame_queue.get_nowait()
                    frames_dropped += 1
                except asyncio.QueueEmpty:
                    pass
            await frame_queue.put(frame)
            # Log stats every 320 frames (~10 seconds at 32Hz)
            if frames_read % 320 == 0:
                log("FEEDER", "INFO",
                    f"Frames read={frames_read}  dropped={frames_dropped}  "
                    f"queue_size={frame_queue.qsize()}")
        else:
            null_streak += 1
            if null_streak == 1:
                log("FEEDER", "WARN", "C++ pipe returned null frame — pipe may be slow or dead")
            if null_streak >= 50:
                log("FEEDER", "ERROR",
                    f"50 consecutive null frames — C++ pipe is dead! "
                    f"Check {MLX_BIN} is running and sensor is connected")
                null_streak = 0   # reset so we don't spam every frame
            await asyncio.sleep(0.05)


# ─────────────────────────────────────────────────────────────────────────────
#  COROUTINE 2 — Vision consumer
# ─────────────────────────────────────────────────────────────────────────────
async def vision_consumer(
    frame_queue:     asyncio.Queue,
    detection_queue: asyncio.Queue,
    movement:        MovementBlock,
    fpn_pattern:     Optional[np.ndarray],
):
    loop    = asyncio.get_running_loop()
    window: list[np.ndarray] = []
    batches_processed = 0
    log("VISION", "OK", f"Consumer started for {DRONE_ID}")
    if fpn_pattern is None:
        log("VISION", "WARN",
            "No FPN pattern — thermal noise won't be corrected. "
            "Run 00_preflight_calib.py before flight!")

    while True:
        if mission_land:
            log("VISION", "INFO", "LAND flag — stopping vision consumer")
            break

        try:
            frame = frame_queue.get_nowait()
            window.append(frame)
        except asyncio.QueueEmpty:
            await asyncio.sleep(0.01)
            continue

        if len(window) < ROLLING_WINDOW:
            continue

        stack = np.array(window[-ROLLING_WINDOW:])
        window.clear()
        batches_processed += 1

        def _run_vision():
            return vision.process_memory_stack(stack, fpn_pattern)

        try:
            dx, dy, conf = await loop.run_in_executor(None, _run_vision)
        except Exception as e:
            log("VISION", "ERROR", f"process_memory_stack crashed: {e}")
            log("VISION", "ERROR", traceback.format_exc())
            continue

        if dx is None:
            # Log every 50 batches so we know vision is alive even when clear
            if batches_processed % 50 == 0:
                log("VISION", "INFO",
                    f"Batch {batches_processed}: sector clear (no blob detected)")
            continue

        if conf < CONFIDENCE_THRESHOLD:
            log("VISION", "INFO",
                f"Batch {batches_processed}: blob detected but below threshold "
                f"({conf*100:.1f}% < {CONFIDENCE_THRESHOLD*100:.0f}%) — ignored")
            continue

        if mission_paused:
            log("VISION", "WARN",
                f"Detection conf={conf*100:.1f}% discarded — mission is PAUSED")
            continue

        lat, lon, alt = movement.get_current_telemetry()
        log("VISION", "OK",
            f"TARGET ACQUIRED  conf={conf*100:.1f}%  "
            f"dx={dx}px dy={dy}px  "
            f"drone=({lat:.6f},{lon:.6f}) alt={alt:.1f}m")

        candidate = MineCandidate(
            drone_lat=lat, drone_lon=lon, altitude=alt,
            dx=dx, dy=dy, conf=conf, raw_stack=stack,
        )

        if detection_queue.full():
            log("VISION", "WARN",
                "Detection queue full — flight loop is busy. "
                "This candidate will be dropped.")
        else:
            await detection_queue.put(candidate)
            log("VISION", "INFO",
                f"Candidate queued (queue depth now {detection_queue.qsize()})")


# ─────────────────────────────────────────────────────────────────────────────
#  COROUTINE 3 — Flight loop
# ─────────────────────────────────────────────────────────────────────────────
async def flight_loop(
    movement:        MovementBlock,
    cam:             PipeCamera,
    detection_queue: asyncio.Queue,
    tunnel:          "comms.DroneTunnel",
    udp_sender:      UDPSender,
    fpn_pattern:     Optional[np.ndarray],
):
    loop = asyncio.get_running_loop()
    log("FLIGHT", "INFO", f"Strip Y={STRIP_Y_M:.2f}m  length={FIELD_LENGTH_M}m  "
                           f"alt={CRUISE_ALT_M}m  step=0.5m")

    await movement.takeoff_to_hover(CRUISE_ALT_M)

    distance_covered = 0.0
    STEP_M = 0.5
    steps_since_log = 0

    log("FLIGHT", "OK", "Airborne — beginning strip scan")

    while distance_covered < FIELD_LENGTH_M:

        if mission_land:
            log("FLIGHT", "WARN", "LAND flag received — landing immediately")
            await movement.land()
            return

        if mission_paused:
            if steps_since_log == 0:
                log("FLIGHT", "INFO", "Mission PAUSED — holding position")
            steps_since_log = 1
            await asyncio.sleep(0.1)
            continue
        steps_since_log = 0

        # Check for pending mine candidate
        try:
            candidate: MineCandidate = detection_queue.get_nowait()
            log("FLIGHT", "INFO",
                f"Processing mine candidate at {distance_covered:.1f}m into strip")
            await _handle_candidate(candidate, movement, cam, tunnel,
                                    udp_sender, fpn_pattern, loop)
        except asyncio.QueueEmpty:
            pass

        # Advance one step
        await movement.fly_forward_step(STEP_M)
        distance_covered += STEP_M

        # Progress log every 10m
        if int(distance_covered) % 10 == 0 and distance_covered > 0:
            log("FLIGHT", "INFO",
                f"Progress: {distance_covered:.0f}/{FIELD_LENGTH_M:.0f}m  "
                f"mines_found={len(confirmed_mines)}")

    log("FLIGHT", "OK",
        f"Strip complete. Total distance={distance_covered:.1f}m  "
        f"mines_confirmed={len(confirmed_mines)}")
    await movement.land()


async def _handle_candidate(
    candidate:   MineCandidate,
    movement:    MovementBlock,
    cam:         PipeCamera,
    tunnel:      "comms.DroneTunnel",
    udp_sender:  UDPSender,
    fpn_pattern: Optional[np.ndarray],
    loop:        asyncio.AbstractEventLoop,
):
    log("MINE", "INFO",
        f"Hovering {HOVER_BEFORE_CHECK}s for persistence check  "
        f"initial_conf={candidate.conf*100:.1f}%")
    await movement.force_hover(HOVER_BEFORE_CHECK)

    log("MINE", "INFO", f"Re-capturing {PERSIST_WINDOW} frames over target...")
    target_stack = await loop.run_in_executor(
        None, cam.capture_stack_sync, PERSIST_WINDOW
    )
    if target_stack is None:
        log("MINE", "WARN", "Re-capture failed — no frames from pipe. Resuming strip.")
        return

    log("MINE", "INFO", "Running persistence check...")
    def _check():
        new_dx, new_dy, final_conf = vision.process_memory_stack(
            target_stack, fpn_pattern
        )
        p = persistence.PersistenceFilter(max_drift_meters=1.5)
        ok = p.verify(new_dx, new_dy, candidate.altitude)
        return ok, new_dx, new_dy, final_conf

    try:
        is_persistent, new_dx, new_dy, final_conf = await loop.run_in_executor(
            None, _check
        )
    except Exception as e:
        log("MINE", "ERROR", f"Persistence check crashed: {e}")
        return

    if not is_persistent:
        log("MINE", "INFO",
            f"GHOST — target not persistent (conf={final_conf*100:.1f}%). "
            f"Pixel drift too large. Resuming.")
        return

    if final_conf < CONFIDENCE_THRESHOLD:
        log("MINE", "INFO",
            f"GHOST — persistence passed but conf dropped to {final_conf*100:.1f}% "
            f"(threshold {CONFIDENCE_THRESHOLD*100:.0f}%). Resuming.")
        return

    # Compute GPS of mine
    lat, lon, alt = movement.get_current_telemetry()
    log("MINE", "INFO",
        f"Computing GPS from pixel offset dx={new_dx} dy={new_dy} alt={alt:.1f}m")
    try:
        local_x, local_y = coord_math.get_pixels_to_meters(new_dx, new_dy, alt)
        target_lat, target_lon = coord_math.compute_global_gps(lat, lon, local_x, local_y)
    except Exception as e:
        log("MINE", "ERROR", f"GPS coordinate math failed: {e}")
        log("MINE", "ERROR", "Check origin_state.json exists and has valid locked_yaw_rad")
        return

    log("MINE", "INFO",
        f"Computed mine GPS: ({target_lat:.6f},{target_lon:.6f})  "
        f"local_offset=({local_x:.2f}m,{local_y:.2f}m)")

    # Deduplication
    for i, (mlat, mlon) in enumerate(confirmed_mines):
        dist = _haversine(target_lat, target_lon, mlat, mlon)
        if dist < 1.5:
            log("MINE", "INFO",
                f"Duplicate — matches mine #{i} at ({mlat:.6f},{mlon:.6f}) "
                f"dist={dist:.2f}m < 1.5m. Skipping broadcast.")
            return

    confirmed_mines.append((target_lat, target_lon))
    log("MINE", "OK",
        f"*** MINE #{len(confirmed_mines)} CONFIRMED ***  "
        f"GPS=({target_lat:.6f},{target_lon:.6f})  conf={final_conf*100:.1f}%")

    # Report in background thread so we don't stall the flight loop
    def _report():
        log("MINE", "INFO", "Reporting to master — running verifier + DroneTunnel...")
        try:
            is_new = verifier.verify_and_log(target_lat, target_lon, final_conf)
            log("MINE", "OK" if is_new else "INFO",
                f"Verifier result: {'NEW_DISCOVERY — broadcasting' if is_new else 'already known — skipping broadcast'}")
            if is_new:
                tunnel.send_anomaly_data(target_lat, target_lon, target_stack)
                log("MINE", "OK",
                    f"DroneTunnel anomaly_report sent to {MASTER_IP}:{MINE_REPORT_PORT}")
        except Exception as e:
            log("MINE", "ERROR", f"Report thread failed: {e}")
            log("MINE", "ERROR", traceback.format_exc())

    threading.Thread(target=_report, daemon=True, name="mine_report").start()


# ─────────────────────────────────────────────────────────────────────────────
#  UDP TELEMETRY BROADCAST  (5 Hz)
# ─────────────────────────────────────────────────────────────────────────────
async def udp_broadcast_loop(movement: MovementBlock, sender: UDPSender):
    log("UDP", "OK",
        f"Telemetry broadcast started → {MASTER_IP}:{UDP_TELEMETRY_PORT} @ 5 Hz")
    packets_sent  = 0
    packets_failed = 0

    while True:
        if mission_land:
            log("UDP", "INFO", f"LAND flag — stopping. Sent={packets_sent} Failed={packets_failed}")
            break
        sensor_val = 0.9 if _recent_mine_flag() else 0.0

        ok = sender.send({
            "lat":      movement.current_lat,
            "lng":      movement.current_lon,
            "altitude": movement.current_alt,
            "heading":  movement.current_heading,
            "speed":    FORWARD_SPEED_MS,
            "armed":    movement.is_armed,
            "airborne": movement.is_airborne,
            "bat_pct":  100,
            "sensor":   sensor_val,
        })

        if ok:
            packets_sent += 1
        else:
            packets_failed += 1
            if packets_failed <= 3 or packets_failed % 25 == 0:
                log("UDP", "WARN",
                    f"UDP send failed (total failures={packets_failed}) — "
                    f"is master at {MASTER_IP}:{UDP_TELEMETRY_PORT} reachable?")

        # Stats every 50 packets (10 seconds)
        if packets_sent % 50 == 0 and packets_sent > 0:
            log("UDP", "INFO",
                f"Telemetry stats: sent={packets_sent} failed={packets_failed} "
                f"pos=({movement.current_lat:.5f},{movement.current_lon:.5f})")

        await asyncio.sleep(0.2)


# ─────────────────────────────────────────────────────────────────────────────
#  TCP COMMAND SERVER
# ─────────────────────────────────────────────────────────────────────────────
def start_tcp_command_server(movement: MovementBlock,
                             loop: asyncio.AbstractEventLoop):
    """
    Starts the TCP command server in a background thread.
    `loop` must be the running asyncio event loop — passed in from async main()
    because asyncio.get_running_loop() raises RuntimeError when called from
    the TCPCommandServer's background thread.
    """
    server = TCPCommandServer(drone_id=DRONE_ID, port=TCP_COMMAND_PORT)

    @server.on_command("GOTO")
    def handle_goto(cmd: dict) -> dict:
        lat = float(cmd.get("lat", 0))
        lon = float(cmd.get("lng", 0))
        alt = float(cmd.get("alt", CRUISE_ALT_M))
        seq = cmd.get("seq", "?")
        log("TCP", "OK", f"GOTO seq={seq} → ({lat:.6f},{lon:.6f}) alt={alt:.1f}m")
        asyncio.run_coroutine_threadsafe(
            movement.move_to_coordinate(lat, lon, alt),
            loop,
        )
        return {"status": "ok"}

    @server.on_command("PAUSE")
    def handle_pause(cmd: dict) -> dict:
        global mission_paused
        mission_paused = True
        log("TCP", "OK", f"PAUSE received (seq={cmd.get('seq','?')}) — holding")
        asyncio.run_coroutine_threadsafe(
            movement.force_hover(0), loop
        )
        return {"status": "ok"}

    @server.on_command("RESUME")
    def handle_resume(cmd: dict) -> dict:
        global mission_paused
        mission_paused = False
        log("TCP", "OK", f"RESUME received (seq={cmd.get('seq','?')}) — continuing")
        return {"status": "ok"}

    @server.on_command("LAND")
    def handle_land(cmd: dict) -> dict:
        global mission_land
        mission_land = True
        log("TCP", "OK", f"LAND received (seq={cmd.get('seq','?')}) — emergency land")
        asyncio.run_coroutine_threadsafe(
            movement.land(), loop
        )
        return {"status": "ok"}

    @server.on_command("ARM_TAKEOFF")
    def handle_arm_takeoff(cmd: dict) -> dict:
        alt = float(cmd.get("alt", 1.0))
        log("TCP", "OK",
            f"ARM_TAKEOFF received (seq={cmd.get('seq','?')}) alt={alt}m — "
            f"used by hover_test.py pre-flight check")
        asyncio.run_coroutine_threadsafe(
            movement.takeoff_to_hover(alt),
            loop,
        )
        return {"status": "ok"}

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
        asyncio.run_coroutine_threadsafe(_arm(), loop)
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
        asyncio.run_coroutine_threadsafe(_disarm(), loop)
        return {"status": "ok"}

    try:
        server.start(blocking=False)
        log("TCP", "OK", f"TCPCommandServer listening on port {TCP_COMMAND_PORT}")
    except Exception as e:
        log("TCP", "ERROR", f"Failed to start TCPCommandServer: {e}")
        log("TCP", "ERROR", f"Is port {TCP_COMMAND_PORT} already in use? Try: lsof -i :{TCP_COMMAND_PORT}")
        raise
    return server


# ─────────────────────────────────────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────
async def main():
    print("=" * 64, flush=True)
    print(f"  S.A.F.E. SLAVE  —  {DRONE_ID}", flush=True)
    print(f"  Set identity:  export DRONE_ID=slave_1|slave_2|slave_3", flush=True)
    print(f"  Set master IP: export MASTER_IP=10.42.0.1  (current: {MASTER_IP})", flush=True)
    print(f"  Strip Y     :  {STRIP_Y_M:.2f} m", flush=True)
    print(f"  Altitude    :  {CRUISE_ALT_M} m AGL", flush=True)
    print(f"  UDP telem   :  → {MASTER_IP}:{UDP_TELEMETRY_PORT}", flush=True)
    print(f"  TCP cmds    :  ← listening on :{TCP_COMMAND_PORT}", flush=True)
    print(f"  Mine report :  → {MASTER_IP}:{MINE_REPORT_PORT}", flush=True)
    print(f"  BASE_DIR    :  {BASE_DIR}", flush=True)
    print("=" * 64, flush=True)

    # ── Validate config files exist ──────────────────────────────────────────
    log("INIT", "INFO", f"FPN path: {FPN_PATH}")
    fpn_pattern = None
    if FPN_PATH.exists():
        fpn_pattern = np.load(str(FPN_PATH))
        log("INIT", "OK", f"FPN pattern loaded — shape={fpn_pattern.shape}")
    else:
        log("INIT", "WARN",
            "FPN pattern missing — thermal noise uncorrected. "
            "Run 00_preflight_calib.py!")

    log("INIT", "INFO", f"Origin config path: {ORIGIN_PATH}")
    if not ORIGIN_PATH.exists():
        log("INIT", "WARN",
            "origin_state.json missing — GPS coordinate math will use yaw=0. "
            "Run 00_preflight_calib.py!")

    # ── Start hardware ───────────────────────────────────────────────────────
    log("INIT", "INFO", "Starting C++ camera pipe...")
    try:
        cam = PipeCamera()
    except FileNotFoundError:
        log("INIT", "ERROR", "Cannot start without camera. Exiting.")
        return

    log("INIT", "INFO", f"Creating DroneTunnel → {MASTER_IP}:{MINE_REPORT_PORT}")
    tunnel = comms.DroneTunnel(target_ip=MASTER_IP, target_port=MINE_REPORT_PORT)

    log("INIT", "INFO", "Connecting to MAVSDK...")
    movement = MovementBlock(system_address=MAVSDK_ADDRESS)
    try:
        await movement.connect()
    except Exception as e:
        log("INIT", "ERROR", f"MAVSDK connect failed: {e}")
        log("INIT", "ERROR",
            "Check: (1) MAVProxy is running, (2) FC is powered, "
            "(3) MAVSDK_ADDR env var is correct")
        cam.terminate()
        return

    log("INIT", "INFO", f"Creating UDPSender (drone_id={DRONE_ID})")
    try:
        udp_sender = UDPSender(
            drone_id=DRONE_ID,
            master_ip=MASTER_IP,
            master_port=UDP_TELEMETRY_PORT,
        )
        log("INIT", "OK", "UDPSender created")
    except ValueError as e:
        log("INIT", "ERROR", f"UDPSender init failed: {e}")
        log("INIT", "ERROR",
            f"DRONE_ID='{DRONE_ID}' must be one of: slave_1, slave_2, slave_3")
        cam.terminate()
        return

    log("INIT", "INFO", "Starting TCP command server...")
    try:
        start_tcp_command_server(movement, asyncio.get_running_loop())
    except Exception as e:
        log("INIT", "ERROR", f"TCP server failed to start: {e}")
        cam.terminate()
        return

    frame_queue     = asyncio.Queue(maxsize=128)
    detection_queue = asyncio.Queue(maxsize=4)
    log("INIT", "OK", "All systems go — launching coroutines")

    try:
        await asyncio.gather(
            frame_feeder(cam, frame_queue),
            vision_consumer(frame_queue, detection_queue, movement, fpn_pattern),
            flight_loop(movement, cam, detection_queue, tunnel,
                        udp_sender, fpn_pattern),
            udp_broadcast_loop(movement, udp_sender),
        )
    except KeyboardInterrupt:
        log("INIT", "INFO", "Operator abort (Ctrl+C)")
    except Exception as e:
        log("INIT", "ERROR", f"Unhandled exception in gather: {e}")
        log("INIT", "ERROR", traceback.format_exc())
    finally:
        log("INIT", "INFO", "Shutdown — terminating camera pipe")
        cam.terminate()
        log("INIT", "OK", "Clean shutdown complete")


if __name__ == "__main__":
    asyncio.run(main())
