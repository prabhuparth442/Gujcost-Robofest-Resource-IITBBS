#!/usr/bin/env python3
"""
06_surface_filter.py  —  Surface Disc Detector
===============================================
Detects above-ground composite discs (MDF/hardboard top, HDPE flying discs)
against wet-soil background, 1–3:30 PM IST, 1.5 m altitude.

WHY a separate filter from 02_vision_filter.py
-----------------------------------------------
The buried-mine filter (02_vision_filter.py) is tuned for SMALL thermal
contrasts: HOT_MIN = +0.15°C, HOT_MAX = +1.25°C.  Surface discs produce
ΔT = +3 to +36°C against post-rain wet soil — they massively saturate the
mine filter's band and score zero on delta_score (which peaks at 0.50°C).

This filter uses a HIGH-SIGNAL band (+3°C to +40°C) matched to the
disc physics.  Everything else (blob detection, circularity, size gate,
confidence scoring) is the same architecture as the buried-mine filter
but re-calibrated for this target type.

Key physics differences
-----------------------
  Buried mine   : ΔT +0.15 to +1.25°C  (thermal wave attenuated by soil)
  HDPE on surface : ΔT +3 to +9°C      (direct solar, wet soil background)
  MDF  on surface : ΔT +9 to +36°C     (extreme insulator vs wet soil)

Both disc types are HOT — no cold channel needed.

HOT_MAX is set to 40°C which effectively has no upper cap because
the filter's delta_score peaks at 15°C (centre of realistic range).
MDF at +25°C still scores non-zero; only cosmic-ray dead pixels (stuck at
100°C) would be excluded, and those are masked by FPN correction.

Confidence scoring
------------------
  conf = W_CIRC × circ_score + W_SIZE × size_score + W_DELTA × delta_score
  W_CIRC  = 0.30   (discs are circular but FPN artifacts can be too)
  W_SIZE  = 0.35   (physical size gate is the strongest discriminator)
  W_DELTA = 0.35   (thermal magnitude separates discs from warm rocks)

  delta_score peaks at DELTA_PEAK = 15°C (between HDPE ~6°C and MDF ~20°C).
  Any blob with avg_delta < 3°C scores zero → filtered out.

Public interface (same as 02_vision_filter.py)
----------------------------------------------
    process_surface_stack(frame_stack, fpn_pattern,
                          altitude_m=1.5) -> (dx, dy, conf)

    Returns (None, None, 0.0) when no surface disc is detected.
    Returns (dx, dy, conf) in 640×480 upscaled pixel space.
"""

import cv2
import math
import numpy as np

# ── Detection band (post-rain wet soil, 1–3:30 PM IST, 1.5 m) ──────────────
HOT_MIN   =  3.0    # °C — well above rock FP (~0.1–0.5°C) and buried mines
HOT_MAX   = 40.0    # °C — no practical upper cap; dead pixels masked by FPN
DELTA_PEAK = 15.0   # °C — delta_score peaks here (mid-range between disc types)

# Temporal voting
VOTE_THRESHOLD = 0.50   # pixel must be in-band in ≥50% of frames
                         # (lower than mine filter because surface blobs are stable)

# Upscale resolution (must match 04_coordinate_math.py)
UPSCALE_W, UPSCALE_H = 640, 480
SENSOR_RES            = [32, 24]

DEFAULT_ALTITUDE_M = 1.5
DEFAULT_FOV_DEG    = [55, 35]

# Background subtraction sigma.
# At 1.5 m altitude on wet soil, spatial gradient σ ≈ 0.35°C (larger than dry).
# σ_bg = 6 px removes ~0.3 m scale gradients while preserving 0.2 m disc footprint.
SIGMA_BG = 6.0

# Confidence weights
W_CIRC  = 0.30
W_SIZE  = 0.35
W_DELTA = 0.35

MIN_CONFIDENCE  = 0.45   # minimum to report (higher than mine filter)
MIN_CIRCULARITY = 0.30   # lenient: 32×24 is coarse


# ─────────────────────────────────────────────────────────────────────────────
class SurfaceFilter:
    """
    Converts (N, 24, 32) raw frame stack → binary anomaly map (24×32 uint8).

    Pipeline:
      1. FPN subtract
      2. Per-frame spatial background removal (σ=6px Gaussian)
      3. Temporal average of per-frame deltas
      4. HOT band threshold on avg_delta (3°C to 40°C)
      5. Temporal vote (pixel in-band ≥50% of frames)
      6. Temporal stability gate (std ≤ 3°C — surface blobs are very stable)
      7. Morphological closing

    Note: no COLD channel — surface discs are always hot vs wet soil.
    """

    def __init__(self,
                 hot_min:  float = HOT_MIN,
                 hot_max:  float = HOT_MAX,
                 vote_thr: float = VOTE_THRESHOLD,
                 sigma_bg: float = SIGMA_BG):
        self.hot_min  = hot_min
        self.hot_max  = hot_max
        self.vote_thr = vote_thr
        self.sigma_bg = sigma_bg
        self._close_k = np.ones((3, 3), np.uint8)

    def _bg(self, frame: np.ndarray) -> np.ndarray:
        return cv2.GaussianBlur(
            frame.astype(np.float32), (0, 0),
            sigmaX=self.sigma_bg, sigmaY=self.sigma_bg,
            borderType=cv2.BORDER_REFLECT_101)

    def extract(self, frame_stack: np.ndarray,
                fpn_pattern=None):
        """
        Parameters
        ----------
        frame_stack : (N, 24, 32) float32
        fpn_pattern : (24, 32) float32 or None

        Returns
        -------
        binary_map : (24, 32) uint8
        avg_delta  : (24, 32) float32
        """
        N = frame_stack.shape[0]

        # Step 1: FPN correction
        corrected = (frame_stack.astype(np.float32) - fpn_pattern[np.newaxis]
                     if fpn_pattern is not None
                     else frame_stack.astype(np.float32))

        # Step 2: Per-frame background-subtracted delta
        frame_deltas = np.empty_like(corrected)
        for i in range(N):
            frame_deltas[i] = corrected[i] - self._bg(corrected[i])

        # Step 3: Temporal average
        avg_delta = frame_deltas.mean(axis=0).astype(np.float32)

        # Step 4: Band threshold on average
        hot_avg = ((avg_delta >= self.hot_min) &
                   (avg_delta <= self.hot_max)).astype(np.uint8) * 255

        # Step 5: Temporal vote
        hot_v = np.zeros((SENSOR_RES[1], SENSOR_RES[0]), np.float32)
        for i in range(N):
            fd   = frame_deltas[i]
            hot_v += ((fd >= self.hot_min) & (fd <= self.hot_max)).astype(np.float32)

        hot_vote = (hot_v / N >= self.vote_thr).astype(np.uint8) * 255

        # Step 6: Stability gate
        # Surface disc ΔT is very stable; background noise has higher variance.
        # std ≤ 3°C selects persistent hot blobs over transient reflections.
        SURF_STD_MAX = 3.0
        stable = (frame_deltas.std(axis=0) <= SURF_STD_MAX).astype(np.uint8) * 255

        merged = cv2.bitwise_and(
            cv2.bitwise_and(hot_avg, hot_vote), stable)

        # Step 7: Morphological close
        binary_map = cv2.morphologyEx(merged, cv2.MORPH_CLOSE, self._close_k)

        return binary_map, avg_delta


# ─────────────────────────────────────────────────────────────────────────────
class SurfaceBlobDetector:
    """
    Extracts surface disc candidate blobs from binary map.

    Physical disc size at 1.5 m altitude, 55°/35° FOV, 640×480 upscale:
      HDPE frisbee  : ~175 mm diameter → ~100 upscaled px radius
      MDF disc      : ~200 mm diameter → ~115 upscaled px radius
      Thermal halo  : 1.5× physical size → max ~175 px radius

    Area gate is generous because wet soil amplifies the apparent thermal
    footprint slightly beyond the physical disc edge.
    """

    DISC_DIAM_MIN_M = 0.10   # m — smallest disc (10 cm Skimmer)
    DISC_DIAM_MAX_M = 0.50   # m — largest expected thermal footprint
    AREA_MIN_SCALE  = 0.5

    MAX_ASPECT      = 2.5    # discs can appear elongated at sensor edge
    MIN_CIRCULARITY = MIN_CIRCULARITY

    def __init__(self, altitude_m: float = DEFAULT_ALTITUDE_M,
                 fov_deg=None, sensor_res=None):
        fov_deg    = fov_deg    or DEFAULT_FOV_DEG
        sensor_res = sensor_res or SENSOR_RES

        mpp_x = (2 * altitude_m * math.tan(math.radians(fov_deg[0] / 2))) / UPSCALE_W
        mpp_y = (2 * altitude_m * math.tan(math.radians(fov_deg[1] / 2))) / UPSCALE_H
        self.mpp = (mpp_x + mpp_y) / 2.0

        scale_w = UPSCALE_W / sensor_res[0]
        scale_h = UPSCALE_H / sensor_res[1]
        px_area = scale_w * scale_h

        self.area_min = self.AREA_MIN_SCALE * px_area

        r_max = (self.DISC_DIAM_MAX_M / 2) / self.mpp
        self.area_max = math.pi * r_max ** 2 * 1.8   # generous multiplier

        r_min = (self.DISC_DIAM_MIN_M / 2) / self.mpp
        self._area_soft_mid  = (math.pi * r_min**2 + math.pi * r_max**2) / 2
        self._area_soft_half = (math.pi * r_max**2 - math.pi * r_min**2) / 2 + 1

    def detect(self, binary_map: np.ndarray, avg_delta: np.ndarray):
        """
        Returns (dx, dy, conf) in 640×480 pixel space, or (None, None, 0.0).
        dx, dy are pixel offsets from image centre.
        """
        upscaled = cv2.resize(binary_map, (UPSCALE_W, UPSCALE_H),
                              interpolation=cv2.INTER_CUBIC)
        _, upscaled = cv2.threshold(upscaled, 127, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(upscaled, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
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

            # ── Peak delta temperature inside blob ────────────────────────
            blob_up = np.zeros((UPSCALE_H, UPSCALE_W), np.uint8)
            cv2.drawContours(blob_up, [cnt], -1, 255, -1)
            blob_nat = cv2.resize(blob_up, (SENSOR_RES[0], SENSOR_RES[1]),
                                  interpolation=cv2.INTER_NEAREST)
            px_yx = np.argwhere(blob_nat > 0)
            if len(px_yx) > 0:
                best_dT = float(max(
                    avg_delta[r, c] for r, c in px_yx))
            else:
                cx_n = max(0, min(SENSOR_RES[0]-1,
                                  int(cx * SENSOR_RES[0] / UPSCALE_W)))
                cy_n = max(0, min(SENSOR_RES[1]-1,
                                  int(cy * SENSOR_RES[1] / UPSCALE_H)))
                best_dT = float(avg_delta[cy_n, cx_n])

            if best_dT < HOT_MIN:
                continue   # not actually hot — skip

            # ── Confidence ────────────────────────────────────────────────
            circ_score = min(1.0, circ)

            size_score = max(0.0,
                1.0 - abs(area - self._area_soft_mid) / self._area_soft_half)

            # delta_score: peaks at DELTA_PEAK, falls off on both sides
            # Half-width = (HOT_MAX - HOT_MIN) / 2
            half_bw = (HOT_MAX - HOT_MIN) / 2.0
            delta_score = max(0.0,
                1.0 - abs(best_dT - DELTA_PEAK) / half_bw)

            if delta_score < 0.02:
                continue   # signal far outside expected disc range

            conf = W_CIRC * circ_score + W_SIZE * size_score + W_DELTA * delta_score
            if conf < MIN_CONFIDENCE:
                continue

            if conf > best_conf:
                best_conf = conf
                best = (cx - UPSCALE_W // 2, cy - UPSCALE_H // 2, conf)

        if best:
            dx, dy, conf = best
            print(f"[SURFACE] DISC DETECTED: dX={dx}px dY={dy}px | "
                  f"Conf={conf*100:.1f}%")
            return dx, dy, conf

        return None, None, 0.0


# ─────────────────────────────────────────────────────────────────────────────
#  Public entry point
# ─────────────────────────────────────────────────────────────────────────────

def process_surface_stack(frame_stack,
                           fpn_pattern,
                           altitude_m: float = DEFAULT_ALTITUDE_M):
    """
    Main entry point — called by the orchestrator for surface disc detection.

    Parameters
    ----------
    frame_stack  : (N, 24, 32) float32
    fpn_pattern  : (24, 32) float32 or None
    altitude_m   : float (default 1.5)

    Returns
    -------
    (dx, dy, conf)  or  (None, None, 0.0)
    """
    if frame_stack is None or len(frame_stack) == 0:
        return None, None, 0.0

    sf = SurfaceFilter()
    binary_map, avg_delta = sf.extract(frame_stack, fpn_pattern)

    bd = SurfaceBlobDetector(altitude_m=altitude_m)
    return bd.detect(binary_map, avg_delta)
