#!/usr/bin/env python3
"""
grid_map.py  —  Best-Effort Coverage Grid
==========================================
Paints a visual coverage map from real drone GPS positions as they arrive.
Never blocks flight, never enforces position accuracy, never creates gaps
from drift.  If the drone drifts 0.3 m, the adjacent cell also gets painted —
which is exactly correct: the sensor footprint covered that area anyway.

Design principles
-----------------
• PASSIVE  — flight loop calls mark_position(x, y) after each move.
             The grid does not control where the drone goes.
• FUZZY    — each position paints a disc matching the sensor ground footprint
             (radius ≈ 0.78 m at 1.5 m alt).  GPS noise fills in naturally.
• ADDITIVE — cells only ever go unvisited → scanned → detection.
             Nothing erases or downgrades a cell.
• SPARSE   — stored as a dict of only written cells.
• MERGEABLE — master unions grids from all 3 slaves.
              Detection flag always wins over scanned flag.

Cell states (int flags, OR-combinable)
  0 UNVISITED  absent from dict
  1 SCANNED    drone flew nearby, sensor covered this cell
  2 DETECTION  surface disc detected here
  4 HAZARD     pre-known buried mine avoidance disc
  8 FORBIDDEN  pole / statue no-fly zone
"""

import math
import threading

try:
    from fieldmap import (FIELD_X_MIN, FIELD_X_MAX, FIELD_Y_MIN, FIELD_Y_MAX,
                          BURIED_MINES, FORBIDDEN_ZONES)
except ImportError:
    FIELD_X_MIN, FIELD_X_MAX = -2.0, 32.0
    FIELD_Y_MIN, FIELD_Y_MAX = -24.0,  2.0
    BURIED_MINES, FORBIDDEN_ZONES = [], []

CELL = 0.5
FOOTPRINT_RADIUS_M = 0.78   # sensor ground half-width at 1.5 m alt, 55 deg FOV

UNVISITED = 0
SCANNED   = 1
DETECTION = 2
HAZARD    = 4
FORBIDDEN = 8


class GridMap:
    def __init__(self, cell=CELL, footprint_radius=FOOTPRINT_RADIUS_M):
        self._cell = cell
        self._fp   = footprint_radius
        self._cols = math.ceil((FIELD_X_MAX - FIELD_X_MIN) / cell)
        self._rows = math.ceil((FIELD_Y_MAX - FIELD_Y_MIN) / cell)
        self._data: dict[tuple[int, int], int] = {}
        self._lock = threading.Lock()
        self._pre_mark_static()

    def _to_cell(self, x, y):
        ci = int((x - FIELD_X_MIN) / self._cell)
        cj = int((y - FIELD_Y_MIN) / self._cell)
        return (max(0, min(self._cols-1, ci)),
                max(0, min(self._rows-1, cj)))

    def _centre(self, ci, cj):
        return (FIELD_X_MIN + (ci + 0.5) * self._cell,
                FIELD_Y_MIN + (cj + 0.5) * self._cell)

    def _or_flag(self, ci, cj, flag):
        k = (ci, cj)
        self._data[k] = self._data.get(k, UNVISITED) | flag

    def _pre_mark_static(self):
        for m in BURIED_MINES:
            for ci in range(self._cols):
                for cj in range(self._rows):
                    cx, cy = self._centre(ci, cj)
                    if math.hypot(cx - m.x, cy - m.y) <= m.radius_m + self._cell:
                        self._or_flag(ci, cj, HAZARD)
        for fz in FORBIDDEN_ZONES:
            for ci in range(self._cols):
                for cj in range(self._rows):
                    cx, cy = self._centre(ci, cj)
                    if math.hypot(cx - fz.x, cy - fz.y) <= fz.radius_m + self._cell:
                        self._or_flag(ci, cj, FORBIDDEN)

    # ── Write ──────────────────────────────────────────────────────────────

    def mark_position(self, x, y):
        """Paint SCANNED onto cells within sensor footprint of (x, y).
        Drift-tolerant: adjacent cells painted = sensor covered them anyway."""
        ci0, cj0 = self._to_cell(x, y)
        n = math.ceil(self._fp / self._cell) + 1
        with self._lock:
            for di in range(-n, n+1):
                for dj in range(-n, n+1):
                    ci, cj = ci0+di, cj0+dj
                    if 0 <= ci < self._cols and 0 <= cj < self._rows:
                        cx, cy = self._centre(ci, cj)
                        if math.hypot(cx-x, cy-y) <= self._fp:
                            self._or_flag(ci, cj, SCANNED)

    def mark_detection(self, x, y):
        ci, cj = self._to_cell(x, y)
        with self._lock:
            self._or_flag(ci, cj, DETECTION | SCANNED)

    def merge_from(self, cells):
        """Union a slave's snapshot into this grid. Safe to call from any thread."""
        with self._lock:
            for c in cells:
                k = (c["ci"], c["cj"])
                self._data[k] = self._data.get(k, UNVISITED) | c.get("flags", SCANNED)

    # ── Read ───────────────────────────────────────────────────────────────

    def snapshot(self):
        """Return list of all non-UNVISITED cells as dicts for JSON.
        ONLY cells the drone has actually visited are included (no empty ground)."""
        with self._lock:
            snap = list(self._data.items())
        result = []
        for (ci, cj), flags in snap:
            if flags == UNVISITED:
                continue
            cx, cy = self._centre(ci, cj)
            parts = []
            if flags & FORBIDDEN: parts.append("forbidden")
            if flags & HAZARD:    parts.append("hazard")
            if flags & DETECTION: parts.append("detection")
            elif flags & SCANNED: parts.append("scanned")
            result.append({
                "ci": ci, "cj": cj,
                "cx": round(cx, 2), "cy": round(cy, 2),
                "state": "+".join(parts) or "unknown",
                "flags": flags,
            })
        return result

    def coverage_pct(self):
        with self._lock:
            snap = dict(self._data)
        total = self._cols * self._rows
        scanned = sum(1 for f in snap.values()
                      if (f & SCANNED) and not (f & FORBIDDEN))
        return round(100.0 * scanned / max(1, total), 1)


GRID = GridMap()
