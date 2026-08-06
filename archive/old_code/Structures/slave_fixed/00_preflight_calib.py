#!/usr/bin/env python3
"""
00_preflight_calib.py  —  S.A.F.E. Pre-flight Calibration
==========================================================
Run ONCE on each slave before mission, drone at field start position.

Step 1 — FPN calibration  (thermal sensor noise pattern)
Step 2 — Origin lock      (reads REAL heading from FC via MAVSDK)

Log format: [HH:MM:SS][COMPONENT][STATUS] message
"""

import asyncio
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np


def log(component: str, status: str, msg: str) -> None:
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}][{component}][{status}] {msg}", flush=True)


# ─────────────────────────────────────────────────────────────────────────────
#  PATHS
# ─────────────────────────────────────────────────────────────────────────────
BASE_DIR    = Path(__file__).resolve().parent
CONFIG_DIR  = BASE_DIR / "config"
FPN_FILE    = CONFIG_DIR / "fpn_pattern.npy"
ORIGIN_FILE = CONFIG_DIR / "origin_state.json"
RAM_FPN     = Path("/dev/shm/fpn_raw.dat")
CALIB_BIN   = BASE_DIR / "bin" / "camera_bridge"
MAVSDK_ADDR = os.environ.get("MAVSDK_ADDR", "udp://:14540")
FLIGHT_ALT_M = 1.5

CONFIG_DIR.mkdir(parents=True, exist_ok=True)


# ─────────────────────────────────────────────────────────────────────────────
#  STEP 1 — FPN CALIBRATION
# ─────────────────────────────────────────────────────────────────────────────
def calibrate_fpn():
    log("FPN", "INFO", "Starting Fixed Pattern Noise calibration...")
    log("FPN", "INFO",
        "Sensor must face a UNIFORM surface (lens cap or flat ground) — no heat sources!")

    if not CALIB_BIN.exists():
        log("FPN", "WARN",
            f"camera_bridge binary not found at {CALIB_BIN}. Skipping FPN calibration.")
        return

    log("FPN", "INFO", f"Running: {CALIB_BIN} 30 calib")
    ret = os.system(f"{CALIB_BIN} 30 calib")
    if ret != 0:
        log("FPN", "ERROR",
            f"camera_bridge exited with code {ret}. "
            "Check sensor connection and binary permissions.")
        return

    if not RAM_FPN.exists():
        log("FPN", "ERROR",
            f"{RAM_FPN} not found after calibration run. "
            "C++ binary ran but wrote no data to /dev/shm.")
        return

    try:
        raw_bytes = RAM_FPN.read_bytes()
        expected  = 24 * 32 * 4
        if len(raw_bytes) != expected:
            log("FPN", "ERROR",
                f"RAM file size {len(raw_bytes)}B != expected {expected}B — data corrupt.")
            return

        raw_avg     = np.frombuffer(raw_bytes, dtype=np.float32).reshape(24, 32)
        fpn_pattern = raw_avg - np.mean(raw_avg)
        np.save(str(FPN_FILE), fpn_pattern)
        RAM_FPN.unlink()

        log("FPN", "OK",
            f"Saved to {FPN_FILE}  "
            f"shape={fpn_pattern.shape}  "
            f"min={fpn_pattern.min():.3f}  max={fpn_pattern.max():.3f}")

    except Exception as e:
        log("FPN", "ERROR", f"Failed to process FPN data: {e}")


# ─────────────────────────────────────────────────────────────────────────────
#  STEP 2 — ORIGIN LOCK  (reads REAL heading from FC)
# ─────────────────────────────────────────────────────────────────────────────
async def lock_orientation_async():
    """
    FIX 3: Reads actual compass heading from FC instead of hardcoding 90 degrees.

    Old code wrote locked_yaw_rad=1.5708 always.
    This version reads 5 heading samples from MAVSDK and averages them.
    If MAVSDK fails, writes a fallback file with a loud warning instead
    of silently using wrong coordinates.
    """
    log("ORIENT", "INFO", f"Connecting to FC via MAVSDK at {MAVSDK_ADDR}...")
    log("ORIENT", "INFO",
        "Drone must be at its FIELD START POSITION facing the scan direction!")

    try:
        from mavsdk import System
    except ImportError:
        log("ORIENT", "ERROR", "mavsdk not installed: pip install mavsdk")
        _write_fallback_origin()
        return

    drone = System()
    try:
        await drone.connect(system_address=MAVSDK_ADDR)
    except Exception as e:
        log("ORIENT", "ERROR", f"drone.connect() raised: {e}")
        _write_fallback_origin()
        return

    # Wait for FC connection
    log("ORIENT", "INFO", "Waiting for FC (timeout 15s)...")
    try:
        async def _wait():
            async for state in drone.core.connection_state():
                if state.is_connected:
                    return True
        await asyncio.wait_for(_wait(), timeout=15.0)
        log("ORIENT", "OK", "FC connected")
    except asyncio.TimeoutError:
        log("ORIENT", "ERROR",
            "FC did not connect in 15s. "
            "Check MAVProxy is running and FC is powered.")
        _write_fallback_origin()
        return

    # Read GPS origin
    log("ORIENT", "INFO", "Waiting for GPS fix (up to 30s)...")
    gps_lat = gps_lon = None
    try:
        async def _gps():
            async for pos in drone.telemetry.position():
                if abs(pos.latitude_deg) > 0.001 or abs(pos.longitude_deg) > 0.001:
                    return pos.latitude_deg, pos.longitude_deg
        gps_lat, gps_lon = await asyncio.wait_for(_gps(), timeout=30.0)
        log("ORIENT", "OK", f"GPS origin: {gps_lat:.6f}, {gps_lon:.6f}")
    except asyncio.TimeoutError:
        log("ORIENT", "ERROR",
            "GPS fix not acquired in 30s. Move outdoors with clear sky view.")
        _write_fallback_origin()
        return
    except Exception as e:
        log("ORIENT", "ERROR", f"GPS read error: {e}")
        _write_fallback_origin()
        return

    # Read compass heading — 5 samples, circular mean
    log("ORIENT", "INFO", "Reading compass heading (5 samples)...")
    heading_deg = 0.0
    try:
        async def _hdg():
            samples = []
            async for h in drone.telemetry.heading():
                samples.append(h.heading_deg)
                log("ORIENT", "INFO",
                    f"  Heading sample {len(samples)}/5: {h.heading_deg:.1f} deg")
                if len(samples) >= 5:
                    break
            return samples
        samples = await asyncio.wait_for(_hdg(), timeout=10.0)

        # Circular mean handles 359 -> 1 degree wraparound correctly
        sin_sum = sum(math.sin(math.radians(s)) for s in samples)
        cos_sum = sum(math.cos(math.radians(s)) for s in samples)
        heading_deg = math.degrees(math.atan2(sin_sum, cos_sum)) % 360
        log("ORIENT", "OK",
            f"Heading locked: {heading_deg:.1f} deg  "
            f"(avg of {[round(s,1) for s in samples]})")
        log("ORIENT", "INFO",
            "Reference: 0=North  90=East  180=South  270=West")
        log("ORIENT", "INFO",
            "This value will be used for ALL mine GPS coordinate calculations.")

    except asyncio.TimeoutError:
        log("ORIENT", "WARN",
            "Heading stream timed out — using 0.0 deg. "
            "Coordinates will be correct only if drone faces True North!")
        heading_deg = 0.0
    except Exception as e:
        log("ORIENT", "WARN", f"Heading read failed: {e}  — using 0.0 deg fallback")
        heading_deg = 0.0

    heading_rad = math.radians(heading_deg)

    origin = {
        "local_origin_x":    0.0,
        "local_origin_y":    0.0,
        "locked_yaw_deg":    round(heading_deg, 2),
        "locked_yaw_rad":    round(heading_rad, 6),
        "start_lat":         round(gps_lat, 7),
        "start_lon":         round(gps_lon, 7),
        "flight_altitude_m": FLIGHT_ALT_M,
        "status":            "LOCKED",
        "locked_at":         time.strftime("%Y-%m-%dT%H:%M:%S"),
    }

    try:
        with open(ORIGIN_FILE, "w") as f:
            json.dump(origin, f, indent=4)
        log("ORIENT", "OK", f"Origin saved to {ORIGIN_FILE}")
        log("ORIENT", "OK", f"  lat={gps_lat:.6f}  lon={gps_lon:.6f}  heading={heading_deg:.1f} deg")
    except Exception as e:
        log("ORIENT", "ERROR", f"Failed to write {ORIGIN_FILE}: {e}")


def _write_fallback_origin():
    """Writes a clearly-labelled fallback so the system doesn't crash."""
    log("ORIENT", "WARN",
        "Writing FALLBACK origin — heading=0 deg, GPS=(0,0). "
        "Mine coordinates WILL BE WRONG. Fix MAVSDK and rerun!")
    origin = {
        "local_origin_x":    0.0,
        "local_origin_y":    0.0,
        "locked_yaw_deg":    0.0,
        "locked_yaw_rad":    0.0,
        "start_lat":         0.0,
        "start_lon":         0.0,
        "flight_altitude_m": FLIGHT_ALT_M,
        "status":            "FALLBACK — MAVSDK unavailable at calibration time",
        "locked_at":         time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    with open(ORIGIN_FILE, "w") as f:
        json.dump(origin, f, indent=4)
    log("ORIENT", "WARN", f"Fallback written to {ORIGIN_FILE}")


# ─────────────────────────────────────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────
async def execute_full_preflight():
    print("=" * 64, flush=True)
    print("  S.A.F.E. PRE-FLIGHT CALIBRATION", flush=True)
    print(f"  BASE_DIR   : {BASE_DIR}", flush=True)
    print(f"  MAVSDK_ADDR: {MAVSDK_ADDR}", flush=True)
    print(f"  FPN file   : {FPN_FILE}", flush=True)
    print(f"  Origin file: {ORIGIN_FILE}", flush=True)
    print("=" * 64, flush=True)

    log("PREFLIGHT", "INFO", "Step 1/2 — FPN calibration")
    calibrate_fpn()

    log("PREFLIGHT", "INFO", "Step 2/2 — Origin + heading lock from FC")
    await lock_orientation_async()

    # Final check — confirm both files exist and look valid
    print("=" * 64, flush=True)
    log("PREFLIGHT", "INFO", "Verification:")

    if FPN_FILE.exists():
        fpn = np.load(str(FPN_FILE))
        log("PREFLIGHT", "OK",
            f"FPN file OK  shape={fpn.shape}  max={fpn.max():.3f}")
    else:
        log("PREFLIGHT", "WARN", "FPN file missing — thermal detections may have false positives")

    if ORIGIN_FILE.exists():
        with open(ORIGIN_FILE) as f:
            origin = json.load(f)
        status = origin.get("status", "?")
        yaw    = origin.get("locked_yaw_deg", "?")
        lat    = origin.get("start_lat", "?")
        lon    = origin.get("start_lon", "?")
        at     = origin.get("locked_at", "?")
        if "FALLBACK" in str(status):
            log("PREFLIGHT", "WARN",
                f"Origin is FALLBACK — heading={yaw} deg GPS=({lat},{lon}). "
                "Rerun with working MAVSDK connection!")
        else:
            log("PREFLIGHT", "OK",
                f"Origin LOCKED — heading={yaw} deg  GPS=({lat},{lon})  at {at}")
    else:
        log("PREFLIGHT", "ERROR", "Origin file missing — coordinate math will fail!")

    print("=" * 64, flush=True)
    log("PREFLIGHT", "INFO",
        "Done. Next: start MAVProxy, then python3 main_orchestrator.py")


if __name__ == "__main__":
    asyncio.run(execute_full_preflight())
