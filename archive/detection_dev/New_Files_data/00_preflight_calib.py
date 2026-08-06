#!/usr/bin/env python3
"""
00_preflight_calib.py  (v2)
===========================
FPN calibration for the MLX90640.

Bug fixed in v2: the original code averaged ALL 30 frames from the sensor
pipe, including frames 0–9 which are corrupted by two sensor startup artefacts:

  ARTEFACT A — Subpage stale data (frame 0 only)
    The MLX90640 reads pixels in two interleaved subpages (0 = checkerboard of
    even pixels, 1 = checkerboard of odd pixels).  On the very first call after
    SetRefreshRate, subpage-1 pixels still hold data from before the mode change
    — up to ±0.8°C wrong.  This creates a full-image checkerboard error.

  ARTEFACT B — On-chip NUC thermal settling (frames 0–4)
    The sensor's Non-Uniformity Correction circuit takes ~5 frames to reach
    thermal equilibrium.  During this window odd rows are offset from even rows
    by up to ±1.2°C (decaying exponentially to 0 by frame 5).  This is the
    horizontal grid-line pattern that was appearing in the calibration data.

Effect when both artefacts are averaged into fpn_pattern:
  - Row-stripe amplitude in FPN: ~0.23°C  (should be <0.05°C)
  - Residual grid lines in every flight frame after subtraction: 0.12°C std
  - Mine signal for 3cm depth: ~0.23°C → grid lines of equal magnitude mask it

Fix: spawn mlx_stdout directly from Python, read-and-discard SKIP_FRAMES frames
     for sensor settle, then average the next CALIB_FRAMES for a clean FPN.
     Falls back to the original camera_bridge path if mlx_stdout is absent.
"""

import json
import os
import subprocess
import numpy as np

BASE_DIR   = os.path.expanduser("~/drone_swarm")
CONFIG_DIR = os.path.join(BASE_DIR, "config")
FPN_FILE   = os.path.join(CONFIG_DIR, "fpn_pattern.npy")
RAM_DISK_FPN = "/dev/shm/fpn_raw.dat"

os.makedirs(CONFIG_DIR, exist_ok=True)

MLX_STDOUT_BIN = os.path.join(BASE_DIR, "bin", "mlx_stdout")

# Frames to DISCARD so the sensor NUC/subpage artefacts are gone.
# Frame 0   : subpage-1 stale data (checkerboard artefact)
# Frames 1-4: NUC not settled (row-stripe artefact)
# Frame 5+  : clean
# We skip 10 for a comfortable margin (costs only 0.3 s at 32 Hz).
SKIP_FRAMES  = 10
CALIB_FRAMES = 30   # frames averaged after skip: σ_residual ≈ NETD/√30 ≈ 18 mK


def calibrate_fpn_via_pipe():
    """
    Primary calibration path.

    Spawns mlx_stdout, silently discards the first SKIP_FRAMES, then averages
    CALIB_FRAMES to build a clean, artefact-free FPN pattern.

    Returns True on success, False on failure.
    """
    if not os.path.exists(MLX_STDOUT_BIN):
        return False

    total_needed    = SKIP_FRAMES + CALIB_FRAMES
    bytes_per_frame = 768 * 4

    print(f"[PRE-FLIGHT] Pipe calibration: capturing {total_needed} frames "
          f"(discarding first {SKIP_FRAMES} for NUC settle)...")

    try:
        proc = subprocess.Popen(
            [MLX_STDOUT_BIN],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )
    except OSError as e:
        print(f"[PRE-FLIGHT] Cannot launch {MLX_STDOUT_BIN}: {e}")
        return False

    stack       = []
    frames_read = 0

    try:
        while frames_read < total_needed:
            raw = proc.stdout.read(bytes_per_frame)
            if len(raw) != bytes_per_frame:
                print("\n[PRE-FLIGHT] Pipe ended early — sensor disconnected?")
                return False

            frames_read += 1

            if frames_read <= SKIP_FRAMES:
                # Silently consume the settling frames
                continue

            frame = np.frombuffer(raw, dtype=np.float32).reshape(24, 32)
            stack.append(frame)
            print(f"[PRE-FLIGHT] Calibration frame "
                  f"{len(stack)}/{CALIB_FRAMES}", end="\r")

    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()

    if len(stack) < CALIB_FRAMES:
        print(f"\n[PRE-FLIGHT] Only got {len(stack)} frames, need {CALIB_FRAMES}")
        return False

    avg_frame   = np.mean(np.array(stack, dtype=np.float32), axis=0)
    fpn_pattern = (avg_frame - avg_frame.mean()).astype(np.float32)
    np.save(FPN_FILE, fpn_pattern)

    row_stripe = float(np.std(fpn_pattern.mean(axis=1)))
    print(f"\n[PRE-FLIGHT] SUCCESS: FPN saved.  "
          f"Row-stripe residual = {row_stripe:.4f}°C  "
          f"(good if <0.05°C)")
    return True


def calibrate_fpn_via_bridge():
    """
    Fallback path using the pre-compiled camera_bridge binary.
    This does NOT skip early frames so some grid-line artefact may remain.
    Only used when mlx_stdout is unavailable.
    """
    print("[PRE-FLIGHT] Fallback: camera_bridge path "
          "(note: NUC artefact suppression unavailable on this path).")

    bridge_bin = os.path.join(BASE_DIR, "bin", "camera_bridge")
    os.system(f"{bridge_bin} 30 calib")

    if os.path.exists(RAM_DISK_FPN):
        raw_avg     = np.fromfile(RAM_DISK_FPN, dtype=np.float32).reshape(24, 32)
        fpn_pattern = (raw_avg - raw_avg.mean()).astype(np.float32)
        np.save(FPN_FILE, fpn_pattern)
        os.remove(RAM_DISK_FPN)
        print("[PRE-FLIGHT] SUCCESS (bridge path): FPN pattern locked and saved.")
    else:
        print("[PRE-FLIGHT] ERROR: No data from camera_bridge. "
              "Check hardware connection.")


def calibrate_fpn():
    """
    Run FPN calibration.  Tries the clean pipe path first (skip-10),
    falls back to the bridge path if mlx_stdout is not available.
    """
    print("[PRE-FLIGHT] Calibrating Fixed Pattern Noise (FPN)...")
    if not calibrate_fpn_via_pipe():
        calibrate_fpn_via_bridge()


def lock_orientation():
    origin_state = {
        "local_origin_x":    0.0,
        "local_origin_y":    0.0,
        "locked_yaw_rad":    1.5708,
        "locked_yaw_deg":    90.0,
        "start_lat":         20.296000,
        "start_lon":         85.824000,
        "flight_altitude_m": 1.5,
        "status":            "LOCKED"
    }
    with open(os.path.join(CONFIG_DIR, "origin_state.json"), "w") as f:
        json.dump(origin_state, f, indent=4)
    print(f"[PRE-FLIGHT] Origin locked. Yaw: {origin_state['locked_yaw_deg']}°")


def execute_full_preflight():
    print("\n========================================")
    print(" INITIATING ROBOFEST SWARM PRE-FLIGHT")
    print("========================================")
    calibrate_fpn()
    lock_orientation()
    print("========================================\n")


if __name__ == "__main__":
    execute_full_preflight()
