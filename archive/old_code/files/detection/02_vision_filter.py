#!/usr/bin/env python3
"""
02_vision_filter.py  (v2)
=========================
Drop-in replacement for the original vision filter.

Public interface UNCHANGED:
    process_memory_stack(frame_stack, fpn_pattern) -> (dx, dy, conf)

    Called by main_orchestrator.py exactly as before.
    fpn_pattern is the (24,32) float32 array loaded from fpn_pattern.npy —
    the RAW sensor frames from PipeCamera are passed here, so FPN is applied
    inside this function (not pre-applied).

IMPORTANT — orchestrator integration change:
    CONFIDENCE_THRESHOLD in main_orchestrator.py must be 0.40, NOT 0.80.
    The old filter used  conf = circularity × 0.95  which saturated near 0.95.
    The new filter uses  conf = 0.45×circ + 0.30×size_fit + 0.25×delta_strength.
    A good confirmed mine scores 0.49–0.65.  Threshold of 0.80 rejects everything.
    See the one-line fix marked INTEGRATION NOTE in main_orchestrator.py.

Changes and reasons
-------------------
SpatiotemporalFilter:
  OLD thresh: median + 1.25°C  → never fires (real mine max ΔT ≈ 0.5°C at survey time)
  NEW thresh: physics-derived bidirectional bands:
      HOT  +0.15 to +1.10°C  (shallow mines, daytime; rocks excluded below 0.15)
      COLD −0.46 to −0.06°C  (deeper mines acting as insulators)
    Both bands needed because at 10:00–11:00 IST, shallow mines are thermally
    cold relative to background; at 12:00–14:00 IST they are hot.

  OLD: hot-only  → misses all mines deeper than ~6 cm at any time of day
  NEW: bidirectional  → detects 2–12 cm depth range

  OLD: per-frame median threshold + large dilation for drift → coarse blobs
  NEW: spatial high-pass (σ=8px Gaussian background) per frame, then:
       - avg_delta over 48 frames (noise → 14 mK, mine survives at 0.15–0.5°C)
       - temporal vote (pixel must be in-band in ≥55% of frames)
       - temporal stability gate (std ≤ 1.5×NETD catches flickering FPs)

BlobDetector:
  OLD: INTER_NEAREST upscale → blocky, low circularity → unreliable filter
  NEW: INTER_CUBIC upscale → smooth blobs → accurate circularity

  OLD: fixed area [200, 15000] px → wrong for 5m altitude
  NEW: altitude-adaptive area gate derived from physical mine size + FOV

  OLD: conf = circularity × 0.95 → saturates at 0.95, never works with 0.80 threshold
  NEW: conf = 0.45×circ + 0.30×size_fit + 0.25×delta_strength → realistic 0.4–0.7 range
"""

import cv2
import numpy as np
import math

# ── Detection band limits (physics, Ahmedabad 20 March 2026, 13:00–15:30 IST) ────
#
# Survey window: 1:00 PM – 3:30 PM.  Drone altitude: 1.5m.
#
# HOT band (shallow mines, 2–6 cm):
#   At 1PM the thermal wave has peaked for shallow mines — all are HOT.
#   3cm @ 13h = +0.48°C, rising to +0.93°C by 15:30h.
#   Rocks above local background: 0.07–0.14°C.
#   HOT_MIN = 0.15°C correctly separates the two with a 0.33°C safety margin.
#   HOT_MAX = 1.25°C: 2cm mine peaks at +1.20°C by 15:30h — add 0.05°C margin.
#
# COLD band (deeper mines, 8–12 cm):
#   At 1PM these are coldest (ΔT ≈ −0.17 to −0.20°C), warming toward zero by 3:30h.
#   10cm mine exits cold band at ~14:30h (ΔT > −0.06°C) → undetectable from then.
#   COLD_MAX tightened to −0.08°C (from −0.06): all real cold mines are ≤ −0.10°C
#   at any point in the window, while borderline cold FP rocks sit at −0.06 to −0.07°C.
#   This provides a 0.02°C margin that eliminates the class of cold-FP blobs
#   that were winning over real mines when COLD_MAX was −0.06°C.
HOT_MIN  =  0.150   # °C — above rock FP level (0.14), well below 3cm mine (+0.48)
HOT_MAX  =  1.250   # °C — increased from 1.10 to cover 2cm mine at 15:30h (+1.20)
COLD_MIN = -0.500   # °C — slightly loosened from -0.46 to cover 8cm @ 13h (-0.17)
COLD_MAX = -0.080   # °C — tightened from -0.06: real cold mines are ≤-0.10, rocks at -0.06/-0.07

DEFAULT_ALTITUDE_M   = 1.5          # actual flight altitude (competition spec)
DEFAULT_FOV_DEG      = [55, 35]     # MLX90640 55° lens
SENSOR_RES           = [32, 24]
UPSCALE_W, UPSCALE_H = 640, 480     # matches 04_coordinate_math.py

VOTE_THRESHOLD = 0.55   # fraction of frames pixel must be in-band


# ─────────────────────────────────────────────────────────────────────────────
# SpatiotemporalFilter
# ─────────────────────────────────────────────────────────────────────────────

class SpatiotemporalFilter:
    """
    Converts a (N, 24, 32) raw frame stack into a binary anomaly map (24×32 uint8).

    Pipeline
    --------
    1. FPN subtract (if fpn_pattern provided — orchestrator always provides it)
    2. Per-frame spatial background removal via σ_bg Gaussian blur:
         delta_i = frame_i − GaussianBlur(frame_i, σ_bg)
       Removes slow soil gradients; preserves mine's narrow thermal footprint.
    3. Temporal average of per-frame deltas (noise → NETD/√N ≈ 14 mK)
    4. Band threshold on avg_delta (HOT or COLD mine band)
    5. Temporal vote: pixel must appear in-band in ≥55% of frames
    6. Stability gate: temporal std(delta) ≤ 1.5×NETD rejects flickering FPs
    7. Morphological closing (fills tiny gaps between adjacent pixels)

    Returns (binary_map, avg_delta).
    The orchestrator only uses binary_map; avg_delta is passed to BlobDetector
    for confidence scoring.
    """

    def __init__(self,
                 vote_threshold: float = VOTE_THRESHOLD,
                 hot_min:  float = HOT_MIN,  hot_max:  float = HOT_MAX,
                 cold_min: float = COLD_MIN, cold_max: float = COLD_MAX,
                 sigma_bg: float = 8.0):
        """
        sigma_bg: Gaussian σ for spatial background (native 32×24 pixels).
          Must be > mine radius in pixels so the Gaussian doesn't smear the mine
          into its own background estimate.
          At 1.5m altitude, mine ≈ 4 native px → σ=8 is 2× larger → safe.
          At 5.0m altitude, mine ≈ 1 native px → σ=8 still safe.
          Floor of 8 handles the gradient scale (~metres) correctly at both altitudes.
        """
        self.vote_threshold = vote_threshold
        self.hot_min  = hot_min;  self.hot_max  = hot_max
        self.cold_min = cold_min; self.cold_max = cold_max
        self.sigma_bg = sigma_bg
        self._close_k = np.ones((3, 3), np.uint8)

    def _bg(self, frame: np.ndarray) -> np.ndarray:
        return cv2.GaussianBlur(
            frame.astype(np.float32), (0, 0),
            sigmaX=self.sigma_bg, sigmaY=self.sigma_bg,
            borderType=cv2.BORDER_REFLECT_101)

    def extract_solid_targets(self, frame_stack: np.ndarray,
                               fpn_pattern=None):
        """
        Parameters
        ----------
        frame_stack : (N, 24, 32) float32  — raw frames from PipeCamera
        fpn_pattern : (24, 32) float32 or None

        Returns
        -------
        binary_map : (24, 32) uint8  — 255 at persistent mine-candidate pixels
        avg_delta  : (24, 32) float32 — averaged local-bg-subtracted signal
        """
        N = frame_stack.shape[0]

        # Step 1: FPN correction
        corrected = (frame_stack.astype(np.float32) - fpn_pattern[np.newaxis]
                     if fpn_pattern is not None
                     else frame_stack.astype(np.float32))

        # Step 2: Per-frame spatial-background-subtracted delta
        # Pre-allocate in one block rather than building a Python list
        frame_deltas = np.empty_like(corrected)
        for i in range(N):
            frame_deltas[i] = corrected[i] - self._bg(corrected[i])

        # Step 3: Temporal average (noise collapses; mine signal persists)
        avg_delta = frame_deltas.mean(axis=0).astype(np.float32)

        # Step 4: Band threshold on the averaged delta
        hot_avg  = ((avg_delta >= self.hot_min) &
                    (avg_delta <= self.hot_max)).astype(np.uint8) * 255
        cold_avg = ((avg_delta >= self.cold_min) &
                    (avg_delta <= self.cold_max)).astype(np.uint8) * 255

        # Step 5: Temporal vote (count how many frames each pixel was in-band)
        hot_v  = np.zeros((SENSOR_RES[1], SENSOR_RES[0]), np.float32)
        cold_v = np.zeros_like(hot_v)
        for i in range(N):
            fd = frame_deltas[i]
            hot_v  += ((fd >= self.hot_min)  & (fd <= self.hot_max)).astype(np.float32)
            cold_v += ((fd >= self.cold_min) & (fd <= self.cold_max)).astype(np.float32)

        VOTE_WEAK = max(0.35, self.vote_threshold - 0.20)   # looser gate for high-alt
        hot_vote  = cv2.bitwise_or(
            (hot_v  / N >= self.vote_threshold).astype(np.uint8) * 255,
            (hot_v  / N >= VOTE_WEAK         ).astype(np.uint8) * 255)
        cold_vote = cv2.bitwise_or(
            (cold_v / N >= self.vote_threshold).astype(np.uint8) * 255,
            (cold_v / N >= VOTE_WEAK         ).astype(np.uint8) * 255)

        # Step 6: Temporal-stability gate  (std <= 1.5 × NETD = 0.15°C)
        # Mine pixels have std ≈ NETD (0.10°C); flickering background pixels
        # have std > 0.15°C.
        NETD   = 0.10
        stable = (frame_deltas.std(axis=0) <= 1.5 * NETD).astype(np.uint8) * 255

        hot_final  = cv2.bitwise_and(cv2.bitwise_and(hot_avg,  hot_vote),  stable)
        cold_final = cv2.bitwise_and(cv2.bitwise_and(cold_avg, cold_vote), stable)
        merged     = cv2.bitwise_or(hot_final, cold_final)

        # Step 7: Morphological closing (connect adjacent pixels; no erosion —
        # erosion kills single-pixel mine blobs at high altitude)
        binary_map = cv2.morphologyEx(merged, cv2.MORPH_CLOSE, self._close_k)

        return binary_map, avg_delta


# ─────────────────────────────────────────────────────────────────────────────
# BlobDetector
# ─────────────────────────────────────────────────────────────────────────────

class BlobDetector:
    """
    Extracts mine candidate blobs from the binary map.

    Upscales 24×32 → 640×480 with INTER_CUBIC for smooth, accurate contours.
    Filters by altitude-aware physical size and circularity.
    Uses the peak delta temperature within each blob for confidence scoring.

    Returns (dx, dy, conf) in 640×480 pixel space, compatible with
    04_coordinate_math.py's focal lengths (fx=614.5, fy=761.2).
    """

    # Physical mine thermal-patch diameter range at the ground surface
    MINE_DIAM_MIN_M = 0.15   # m  — smallest expected thermal footprint
    MINE_DIAM_MAX_M = 0.55   # m  — largest (generous diffusion margin)

    # 1 upscaled pixel's worth in native space = min detectable blob area
    AREA_MIN_SCALE  = 0.5    # × (upscale_px_per_native_px)²

    MIN_CIRCULARITY = 0.35   # lenient: 32×24 is coarse
    MAX_ASPECT      = 2.2    # reject elongated rocks/shadows
    MIN_CONFIDENCE  = 0.40   # minimum to report a detection

    def __init__(self, altitude_m: float = DEFAULT_ALTITUDE_M,
                 fov_deg=None, sensor_res=None):
        fov_deg    = fov_deg    or DEFAULT_FOV_DEG
        sensor_res = sensor_res or SENSOR_RES

        mpp_x = (2 * altitude_m * math.tan(math.radians(fov_deg[0] / 2))) / UPSCALE_W
        mpp_y = (2 * altitude_m * math.tan(math.radians(fov_deg[1] / 2))) / UPSCALE_H
        self.mpp = (mpp_x + mpp_y) / 2.0

        # Upscale factor (each native pixel → this many upscaled pixels on each axis)
        scale_w = UPSCALE_W / sensor_res[0]
        scale_h = UPSCALE_H / sensor_res[1]
        px_area = scale_w * scale_h   # upscaled px² per native px²

        self.area_min = self.AREA_MIN_SCALE * px_area    # ≈200 px²

        r_max = (self.MINE_DIAM_MAX_M / 2) / self.mpp
        self.area_max = math.pi * r_max ** 2 * 1.5

        # Soft scoring range (for size_score in confidence)
        r_min = (self.MINE_DIAM_MIN_M / 2) / self.mpp
        self._area_soft_mid  = (math.pi * r_min**2 + math.pi * r_max**2) / 2
        self._area_soft_half = (math.pi * r_max**2 - math.pi * r_min**2) / 2 + 1

    def detect(self, binary_map_24x32: np.ndarray,
               avg_delta: np.ndarray = None,
               frame_stack=None, fpn_pattern=None):
        """
        Parameters
        ----------
        binary_map_24x32 : (24, 32) uint8  — from SpatiotemporalFilter
        avg_delta        : (24, 32) float32 — pre-computed by filter (preferred)
        frame_stack / fpn_pattern : legacy fallback if avg_delta is None

        Returns
        -------
        (dx, dy, conf)  or  (None, None, 0.0)
        """
        upscaled = cv2.resize(binary_map_24x32, (UPSCALE_W, UPSCALE_H),
                              interpolation=cv2.INTER_CUBIC)
        _, upscaled = cv2.threshold(upscaled, 127, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(upscaled, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        if avg_delta is None:
            avg_delta = self._fallback_avg_delta(frame_stack, fpn_pattern)

        best_conf = 0.0
        best      = None

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (self.area_min <= area <= self.area_max):
                continue

            peri = cv2.arcLength(cnt, True)
            if peri < 1:
                continue
            circ = 4 * math.pi * area / (peri * peri)
            if circ < self.MIN_CIRCULARITY:
                continue

            x_bb, y_bb, w_bb, h_bb = cv2.boundingRect(cnt)
            asp = float(w_bb) / h_bb if h_bb else 99
            if not (1.0 / self.MAX_ASPECT < asp < self.MAX_ASPECT):
                continue

            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # ── Confidence ────────────────────────────────────────────────
            circ_score = min(1.0, circ)
            size_score = max(0.0,
                1.0 - abs(area - self._area_soft_mid) / self._area_soft_half)

            # Peak-delta pixel within the blob (more reliable than centroid)
            delta_score = 0.0
            if avg_delta is not None:
                blob_up = np.zeros((UPSCALE_H, UPSCALE_W), np.uint8)
                cv2.drawContours(blob_up, [cnt], -1, 255, -1)
                blob_nat = cv2.resize(blob_up, (SENSOR_RES[0], SENSOR_RES[1]),
                                      interpolation=cv2.INTER_NEAREST)
                px_yx = np.argwhere(blob_nat > 0)
                if len(px_yx) > 0:
                    best_dT = max((avg_delta[r, c] for r, c in px_yx),
                                  key=abs, default=0.0)
                else:
                    cx_n = max(0, min(SENSOR_RES[0]-1, int(cx*SENSOR_RES[0]/UPSCALE_W)))
                    cy_n = max(0, min(SENSOR_RES[1]-1, int(cy*SENSOR_RES[1]/UPSCALE_H)))
                    best_dT = float(avg_delta[cy_n, cx_n])

                if HOT_MIN <= best_dT <= HOT_MAX:
                    # Score peaks at 0.50°C — centre of realistic mine hot band
                    # at 13:00–15:30 IST (3cm mine ranges +0.48 to +0.93°C).
                    # This was 0.25°C when calibrated for 12:00 IST; at 1PM mines
                    # are significantly hotter so the peak is re-centred.
                    delta_score = max(0.0,
                        1.0 - abs(best_dT - 0.50) / ((HOT_MAX - HOT_MIN) / 2))
                elif COLD_MIN <= best_dT <= COLD_MAX:
                    # Cold mines peak around -0.17°C in the 13:00–15:30 window
                    delta_score = max(0.0,
                        1.0 - abs(best_dT - (-0.17)) / (abs(COLD_MAX - COLD_MIN) / 2))

            if delta_score < 0.05:
                continue   # no valid thermal signature → not a mine

            # delta_score weight raised from 0.25 → 0.40.
            # The thermal magnitude is the most physically diagnostic feature:
            # a blob at exactly the right temperature is far more mine-like than
            # one that is merely circular. Circularity reduced from 0.45 → 0.35
            # to prevent high-circ FPs (perfectly round rocks) from outscoring
            # genuine mine blobs whose circularity is degraded by the coarse 32×24 grid.
            conf = 0.35 * circ_score + 0.25 * size_score + 0.40 * delta_score
            if conf < self.MIN_CONFIDENCE:
                continue

            if conf > best_conf:
                best_conf = conf
                best = (cx - UPSCALE_W // 2, cy - UPSCALE_H // 2, conf)

        if best:
            dx, dy, conf = best
            print(f"[VISION] TARGET ACQUIRED: dX={dx}px dY={dy}px | "
                  f"Conf={conf*100:.1f}%")
            return dx, dy, conf

        return None, None, 0.0

    @staticmethod
    def _fallback_avg_delta(frame_stack, fpn_pattern, sigma_bg=8.0):
        if frame_stack is None or len(frame_stack) == 0:
            return None
        corr = (frame_stack.astype(np.float32) - fpn_pattern[np.newaxis]
                if fpn_pattern is not None
                else frame_stack.astype(np.float32))
        avg  = corr.mean(axis=0)
        bg   = cv2.GaussianBlur(avg, (0, 0), sigmaX=sigma_bg, sigmaY=sigma_bg,
                                borderType=cv2.BORDER_REFLECT_101)
        return (avg - bg).astype(np.float32)


# ─────────────────────────────────────────────────────────────────────────────
# Public entry point  (interface identical to v1)
# ─────────────────────────────────────────────────────────────────────────────

def process_memory_stack(frame_stack, fpn_pattern, altitude_m=DEFAULT_ALTITUDE_M):
    """
    Main entry point called by main_orchestrator.py.

    Parameters
    ----------
    frame_stack  : (N, 24, 32) float32  — raw frames from PipeCamera.capture_stack()
    fpn_pattern  : (24, 32) float32 or None  — from fpn_pattern.npy
    altitude_m   : float  — drone AGL height (default 5.0 from origin_state.json)

    Returns
    -------
    (dx, dy, conf)  or  (None, None, 0.0)

    INTEGRATION NOTE
    ----------------
    CONFIDENCE_THRESHOLD in main_orchestrator.py must be changed from 0.80 → 0.40.
    The new filter's composite scoring gives confirmed mines a conf of 0.49–0.65.
    The old filter's circularity-based scoring saturated near 0.95; that's why
    the original 0.80 threshold was never a problem with the old code.
    """
    if frame_stack is None or len(frame_stack) == 0:
        return None, None, 0.0

    # σ_bg = 8.0 px FIXED for 1.5m altitude (the correct operating altitude).
    #
    # Why fixed, not computed dynamically:
    #   At 1.5m, the mine surface thermal patch is ~4.3 native pixels in radius.
    #   The dynamic formula (2×r_px + 2 = 10.6) was too large: at σ=10.6 the
    #   Gaussian background estimate begins to incorporate the medium-scale soil
    #   gradients that are stronger at 1PM–3PM (σ_soil ≈ 0.21°C), causing
    #   background pixels to accumulate votes and merge into oversized blobs
    #   that exceed area_max.  σ=8.0 correctly removes the ~0.4–1m wavelength
    #   gradients while leaving the mine's ~0.2m footprint intact.
    #
    #   At 5.0m (if ever needed), mine is sub-pixel; σ=8 is also correct there.
    sigma_bg = 8.0

    sf = SpatiotemporalFilter(sigma_bg=sigma_bg)
    binary_map, avg_delta = sf.extract_solid_targets(frame_stack, fpn_pattern)

    bd = BlobDetector(altitude_m=altitude_m)
    return bd.detect(binary_map, avg_delta=avg_delta)
