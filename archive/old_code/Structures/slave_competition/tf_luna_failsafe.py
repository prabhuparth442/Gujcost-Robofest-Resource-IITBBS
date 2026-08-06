#!/usr/bin/env python3
"""
tf_luna_failsafe.py  —  TF Luna LiDAR Proximity Failsafe
==========================================================
Reads the TF Luna (TFmini-S compatible) distance sensor over UART.
If any reading falls below TRIGGER_DIST_M the drone executes an emergency
sidestep.

Sidestep direction logic
-------------------------
The drone knows its current X position in field coordinates.  Two obstacles
require opposite escape directions:

  Pole   at x ≈ +13.19 m  →  sidestep WEST  (−X, towards open ground)
  Statue at x ≈ +26.51 m  →  sidestep EAST  (+X, away from statue base)
  Default / unknown        →  sidestep WEST  (generally safer for our field)

The boundary between "pole zone" and "statue zone" is midway between them:
  MID_X = (13.19 + 26.51) / 2 ≈ 19.85 m

If current_x < MID_X → we are near the pole → sidestep WEST
If current_x ≥ MID_X → we are near the statue → sidestep EAST

Sidestep magnitude: SIDESTEP_M = 2.0 m  (clears both obstacle radii with margin)

Hardware connection
-------------------
  TF Luna UART wired to Raspberry Pi UART:
    TX (Luna pin 4) → RX (Pi GPIO 15 / pin 10)
    RX (Luna pin 5) → TX (Pi GPIO 14 / pin 8)
    +5V (Luna pin 1) → Pi 5V rail
    GND (Luna pin 2) → Pi GND

  Default baud rate: 115200  (TF Luna factory default)
  Default serial port: /dev/serial0  (Pi hardware UART, alias for ttyAMA0)
  Override via LUNA_PORT env var:  export LUNA_PORT=/dev/ttyUSB0

TF Luna frame format (9 bytes, little-endian)
----------------------------------------------
  Byte 0: 0x59  (sync)
  Byte 1: 0x59  (sync)
  Byte 2: Dist_L  (distance low byte, cm)
  Byte 3: Dist_H  (distance high byte, cm)
  Byte 4: Strength_L
  Byte 5: Strength_H
  Byte 6: Temp_L
  Byte 7: Temp_H
  Byte 8: Checksum (sum of bytes 0–7, low byte only)

Integration with orchestrator
------------------------------
    from tf_luna_failsafe import LidarFailsafe

    lidar = LidarFailsafe(movement_block)
    asyncio.create_task(lidar.monitor_loop())

The monitor loop runs as a background coroutine.  When triggered it calls
movement_block.emergency_sidestep(direction) which is a new method added
to MovementBlock in the competition orchestrator.

Simulation / no-hardware mode
------------------------------
If pyserial is not installed or the port fails to open, the module
logs a warning and the monitor loop becomes a no-op.  Mission continues
unaffected — the lidar is a failsafe, not a hard requirement.
"""

import asyncio
import math
import os
import time
from typing import Optional

# ── Configuration ──────────────────────────────────────────────────────────
LUNA_PORT      = os.environ.get("LUNA_PORT", "/dev/serial0")
LUNA_BAUD      = 115_200
TRIGGER_DIST_M = 1.0      # metres — emergency sidestep if distance < this
COOLDOWN_S     = 3.0      # seconds between consecutive sidestep triggers
SIDESTEP_M     = 2.0      # metres — how far to move sideways
POLL_HZ        = 20       # readings per second (TF Luna runs at 100 Hz default)

# X-coordinate boundary between pole-zone and statue-zone
POLE_X   = 13.19   # m East (from fieldmap)
STATUE_X = 26.51   # m East
MID_X    = (POLE_X + STATUE_X) / 2.0   # ≈ 19.85 m


class LidarFailsafe:
    """
    Background asyncio coroutine that monitors TF Luna distance.
    Triggers emergency_sidestep() on MovementBlock if obstacle < 1 m.
    """

    def __init__(self, movement, drone_id: str = "slave_1"):
        self.movement  = movement
        self.drone_id  = drone_id
        self._ser      = None
        self._last_trigger_time = 0.0
        self._enabled  = False
        self._last_dist_m: Optional[float] = None

        # Try to open the serial port
        try:
            import serial   # type: ignore
            self._ser = serial.Serial(
                port=LUNA_PORT, baudrate=LUNA_BAUD,
                timeout=0.1)
            self._enabled = True
            print(f"[LIDAR][OK] TF Luna opened on {LUNA_PORT} @ {LUNA_BAUD} baud",
                  flush=True)
        except ImportError:
            print("[LIDAR][WARN] pyserial not installed — lidar failsafe disabled. "
                  "Run: pip install pyserial", flush=True)
        except Exception as e:
            print(f"[LIDAR][WARN] Cannot open {LUNA_PORT}: {e} — failsafe disabled",
                  flush=True)

    # ── Frame reader ───────────────────────────────────────────────────────

    def _read_frame_sync(self) -> Optional[float]:
        """
        Blocking: read one valid TF Luna frame from serial.
        Returns distance in metres, or None on parse error.
        Runs inside run_in_executor so it never blocks the event loop.
        """
        if not self._enabled or self._ser is None:
            return None

        try:
            # Sync to frame header (0x59 0x59)
            while True:
                b = self._ser.read(1)
                if not b:
                    return None
                if b[0] == 0x59:
                    b2 = self._ser.read(1)
                    if b2 and b2[0] == 0x59:
                        break   # found header

            # Read remaining 7 bytes
            rest = self._ser.read(7)
            if len(rest) < 7:
                return None

            dist_l, dist_h = rest[0], rest[1]
            checksum_byte  = rest[6]

            # Verify checksum
            raw = [0x59, 0x59] + list(rest[:6])
            expected = sum(raw) & 0xFF
            if expected != checksum_byte:
                return None   # corrupted frame

            dist_cm = dist_l | (dist_h << 8)
            dist_m  = dist_cm / 100.0
            return dist_m

        except Exception:
            return None

    # ── Sidestep direction logic ───────────────────────────────────────────

    def _sidestep_direction(self) -> str:
        """
        Determine which way to escape based on current X position.
        Returns "west" (−X) or "east" (+X).
        """
        current_x = getattr(self.movement, "current_east_m", None)
        if current_x is None:
            return "west"   # safe default
        # current_east_m is NED east, approximately same as our local X
        return "west" if current_x < MID_X else "east"

    # ── Main coroutine ─────────────────────────────────────────────────────

    async def monitor_loop(self):
        """
        Runs forever as an asyncio task.
        Polls TF Luna at POLL_HZ, triggers sidestep on proximity event.
        """
        if not self._enabled:
            print("[LIDAR][INFO] Monitor loop running in no-op mode (no hardware)",
                  flush=True)
            while True:
                await asyncio.sleep(1.0)
            return

        loop = asyncio.get_running_loop()
        interval = 1.0 / POLL_HZ
        consecutive_triggers = 0
        CONFIRM_FRAMES = 3   # must see < 1 m for 3 consecutive frames before acting

        print(f"[LIDAR][OK] Proximity monitor active — trigger < {TRIGGER_DIST_M} m",
              flush=True)

        while True:
            await asyncio.sleep(interval)

            dist = await loop.run_in_executor(None, self._read_frame_sync)
            if dist is None:
                consecutive_triggers = 0
                continue

            self._last_dist_m = dist

            if dist >= TRIGGER_DIST_M:
                consecutive_triggers = 0
                continue

            # Obstacle within range
            consecutive_triggers += 1
            if consecutive_triggers < CONFIRM_FRAMES:
                continue   # wait for confirmation

            consecutive_triggers = 0
            now = time.time()
            if now - self._last_trigger_time < COOLDOWN_S:
                continue   # still in cooldown from last trigger

            self._last_trigger_time = now
            direction = self._sidestep_direction()

            print(
                f"[LIDAR][WARN] OBSTACLE {dist:.2f} m — "
                f"EMERGENCY SIDESTEP {direction.upper()}",
                flush=True)

            # Execute sidestep — runs in the event loop
            try:
                await self.movement.emergency_sidestep(direction, SIDESTEP_M)
            except Exception as e:
                print(f"[LIDAR][ERROR] Sidestep failed: {e}", flush=True)

    def last_distance(self) -> Optional[float]:
        """Thread-safe read of the most recent distance measurement."""
        return self._last_dist_m

    def close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
