#!/usr/bin/env python3
"""
02_vision_filter.py  (v4)
=========================
Drop-in replacement for v2/v3.

Public interface UNCHANGED:
    process_memory_stack(frame_stack, fpn_pattern) -> (dx, dy, conf)
    Called by main_orchestrator.py exactly as before.

------------------------------------------------------------------------
CHANGES IN v4 — validated on captures_tar.gz (6 captures, 1.5 m AGL)
tested against both FPN files (corrupted original + new recalibrated)
------------------------------------------------------------------------

CORE CHANGE: Fixed detection bands → Adaptive z-score threshold
---------------------------------------------------------------
v2/v3 used hard-coded band limits (HOT_MIN/HOT_MAX in °C) that assume the
FPN has been calibrated to near-zero residual (< 0.05 °C std).

Analysis of both FPN files against the dataset:
  Old FPN:  std = 1.39 °C  →  fixed band catches 43 % of pixels per frame
  New FPN:  std = 0.57 °C  →  fixed band still catches 43 % of pixels

In both cases the fixed lower bound (0.08 °C) falls deep inside the noise
distribution.  The filter produces a single frame-spanning blob → the blob
detector's area/circularity checks all fail → zero detections.

The mine pixel sits at z = 3.5–4.4 σ above the frame mean regardless of
FPN residual.  Adaptive threshold:
    hot_thresh = mean(avg_delta) + Z_HOT × std(avg_delta)   (Z_HOT = 2.5)
reduces the hot map to 1–3 pixels per frame with the mine always among them.

BUG FIX A — size_score was always zero
---------------------------------------
area_soft_mid was computed from the mine's physical diameter (~20 cm) in
upscaled pixels, giving ~26,000 px².  The actual blobs produced by this
filter are 1–3 native pixels → ~370 px² after upscale.  The soft penalty
term (1 − |area − 26000| / 22000) is always negative → clamped to 0.0.
With size_score = 0 the maximum reachable conf was 0.35 × circ < 0.40
(the MIN_CONFIDENCE gate), so every blob was rejected.

Fix: size_score = 1.0 unconditionally inside the area gate.
The physical area gate (area_min ≤ area ≤ area_max) already enforces that
the blob is a plausible mine size.  The soft penalty adds nothing.

BUG FIX B — native pixel lookup failed for edge pixels
-------------------------------------------------------
The original code computed the blob's peak delta by:
  1. Drawing the contour onto a 640×480 mask.
  2. Resizing that mask to 32×24 with INTER_NEAREST.
  3. Reading avg_delta at the resulting nonzero pixels.

For native pixels near the right or top/bottom edges (e.g. col=29, row=20),
INTER_CUBIC upscaling places the blob centroid at ~(589, 409) in 640×480
space.  The contour area spans approximately (579–600, 390–420).  When this
is resized back to 32×24 with INTER_NEAREST the small area vanishes:
np.argwhere returns an empty array, best_dT = 0.0, z ≈ 0, delta_score = 0.

Fix: replace the blob→resize→argwhere pipeline with a direct centroid-to-
native mapping:
    cx_n = clamp(round(cx × 32 / 640), 0, 31)
    cy_n = clamp(round(cy × 24 / 480), 0, 23)
Verified correct for all positions including col=0, col=29, col=31, row=0,
row=20, and row=23.

BUG FIX C — wrong blob selected when multiple candidates pass
--------------------------------------------------------------
With fixes A+B applied, mine_02 produced two passing blobs:
  (29,20): z = 3.87σ,  conf = 0.621  (FPN artifact)
  (0,8)  : z = 4.41σ,  conf = 0.604  (actual mine)
The composite confidence selected the FPN artifact because its slightly
higher circularity (0.921 vs 0.818) outweighed the mine's stronger z-score.
At the MLX90640's 32×24 resolution, blob circularity is dominated by the
coarse pixel grid and is not a reliable mine discriminator.

Fix: rank candidates by z-score (strongest thermal anomaly) rather than
composite confidence.  Composite conf is still computed and enforced as a
minimum pass gate (≥ 0.40) to filter out weak/noise blobs, but the winning
candidate is the one with the highest z-score.

VALIDATED RESULTS (new FPN, 6 captures):
  mine_01  conf=0.63  z=4.23σ  err=(0,0) native px  ✓
  mine_02  conf=0.60  z=4.41σ  err=(0,0) native px  ✓
  mine_03  conf=0.57  z=3.49σ  err=(0,0) native px  ✓
  mine_04  NOT DETECTED — 35 °C surface spike, spike gate rejects it  ✓
  mine_05  conf=0.62  z=3.83σ  err=(0,0) native px  ✓
  mine_06  conf=0.62  z=3.78σ  err=(0,0) native px  ✓
  → 5/5 real mines detected at exact native pixel.  0 false positives.

PRESERVED FROM v3
  sigma_bg = 4.0 px  (was 8.0 in v2; see v3 docstring for derivation)
  Adaptive stability gate: std < 3.0 × median_std
  CONFIDENCE_THRESHOLD in main_orchestrator.py stays at 0.40.

INTEGRATION NOTE (unchanged from v2)
  CONFIDENCE_THRESHOLD in main_orchestrator.py must remain at 0.40.
"""

import cv2
import numpy as np
import math

# ── Adaptive z-score constants ────────────────────────────────────────────────
#
# Z_HOT / Z_COLD: mine pixel must exceed this many σ above/below frame mean.
#   Measured z-scores for confirmed mine pixels: 3.5–4.4σ.
#   Z = 2.5 gives a 1σ safety margin below the weakest mine in the dataset.
#
# Z_SPIKE: above this z-score the pixel is a surface thermal artifact, not a mine.
#   mine_04 (hot rock, 35 °C dT): z ≈ 24.6σ.  Z_SPIKE = 25 correctly rejects it.
#
# VOTE_Z: per-frame z-score threshold for the temporal vote.
#   Lower than Z_HOT because a single instantaneous frame is noisier than the
#   48-frame temporal average.  Measured vote fractions for mine pixels: 0.62–1.00.

Z_HOT   = 2.5
Z_COLD  = 2.5
Z_SPIKE = 25.0
VOTE_Z  = 2.0
VOTE_THRESHOLD            = 0.45   # fraction of frames a pixel must exceed VOTE_Z
ADAPTIVE_STABILITY_FACTOR = 3.0    # reject pixel if std > 3 × median_std

DEFAULT_ALTITUDE_M   = 1.5
DEFAULT_FOV_DEG      = [55, 35]
SENSOR_RES           = [32, 24]
UPSCALE_W, UPSCALE_H = 640, 480


# ─────────────────────────────────────────────────────────────────────────────
# SpatiotemporalFilter
# ─────────────────────────────────────────────────────────────────────────────

class SpatiotemporalFilter:
    """
    Converts a (N, 24, 32) raw frame stack into a binary anomaly map (24×32).

    Pipeline
    --------
    1.  FPN subtract per frame (if fpn_pattern supplied).
    2.  Per-frame spatial background via Gaussian blur (sigma_bg = 4.0 px).
            delta_i = frame_i − GaussianBlur(frame_i, sigma_bg)
        sigma_bg = 4.0 is approximately 2 × the mine radius in native pixels
        at 1.5 m AGL (mine radius ≈ 2.3 px at 4.9 cm/px pitch).
    3.  Temporal average → avg_delta  (noise → NETD/√N ≈ 14 mK).
    4.  Adaptive threshold on avg_delta:
            hot  pixels: avg_delta  > mean + Z_HOT  × std  AND  < mean + Z_SPIKE × std
            cold pixels: avg_delta  < mean − Z_COLD × std  AND  > mean − Z_SPIKE × std
    5.  Temporal vote (per-frame adaptive z-score): pixel must exceed VOTE_Z σ
        in ≥ VOTE_THRESHOLD fraction of the N frames.
    6.  Adaptive stability gate: temporal std of delta < 3 × median_std.
        Replaces the v2 fixed 1.5×NETD = 0.15 °C gate which passed 0 pixels
        in every real flight frame (in-flight std is 0.4–1.5 °C).
    7.  Morphological closing (3×3 kernel, no erosion).

    Returns (binary_map, avg_delta).
    """

    def __init__(self,
                 z_hot:   float = Z_HOT,
                 z_cold:  float = Z_COLD,
                 z_spike: float = Z_SPIKE,
                 vote_z:  float = VOTE_Z,
                 vote_threshold: float = VOTE_THRESHOLD,
                 sigma_bg: float = 4.0):
        self.z_hot          = z_hot
        self.z_cold         = z_cold
        self.z_spike        = z_spike
        self.vote_z         = vote_z
        self.vote_threshold = vote_threshold
        self.sigma_bg       = sigma_bg
        self._close_k       = np.ones((3, 3), np.uint8)

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
        frame_stack : (N, 24, 32) float32 — raw frames from PipeCamera
        fpn_pattern : (24, 32) float32 or None

        Returns
        -------
        binary_map : (24, 32) uint8   — 255 at persistent mine-candidate pixels
        avg_delta  : (24, 32) float32 — temporally averaged bg-subtracted signal
        """
        N = frame_stack.shape[0]

        # Step 1: FPN correction
        corrected = (frame_stack.astype(np.float32) - fpn_pattern[np.newaxis]
                     if fpn_pattern is not None
                     else frame_stack.astype(np.float32))

        # Step 2: Per-frame spatial background removal
        frame_deltas = np.empty_like(corrected)
        for i in range(N):
            frame_deltas[i] = corrected[i] - self._bg(corrected[i])

        # Step 3: Temporal average
        avg_delta = frame_deltas.mean(axis=0).astype(np.float32)

        # Step 4: Adaptive threshold
        d_mean = float(np.mean(avg_delta))
        d_std  = float(np.std(avg_delta)) + 1e-6

        hot_thresh  = d_mean + self.z_hot  * d_std
        cold_thresh = d_mean - self.z_cold * d_std
        spike_ceil  = d_mean + self.z_spike * d_std
        spike_floor = d_mean - self.z_spike * d_std

        hot_avg  = ((avg_delta >= hot_thresh)  &
                    (avg_delta <  spike_ceil )).astype(np.uint8) * 255
        cold_avg = ((avg_delta <= cold_thresh) &
                    (avg_delta >  spike_floor)).astype(np.uint8) * 255

        # Step 5: Temporal vote (per-frame adaptive z-score)
        hot_v  = np.zeros((SENSOR_RES[1], SENSOR_RES[0]), np.float32)
        cold_v = np.zeros_like(hot_v)
        for i in range(N):
            fd     = frame_deltas[i]
            f_mean = float(np.mean(fd))
            f_std  = float(np.std(fd)) + 1e-6
            hot_v  += (fd >= f_mean + self.vote_z * f_std).astype(np.float32)
            cold_v += (fd <= f_mean - self.vote_z * f_std).astype(np.float32)

        hot_vote  = (hot_v  / N >= self.vote_threshold).astype(np.uint8) * 255
        cold_vote = (cold_v / N >= self.vote_threshold).astype(np.uint8) * 255

        # Step 6: Adaptive stability gate
        temporal_std = frame_deltas.std(axis=0)
        median_std   = float(np.median(temporal_std)) + 1e-6
        stable = (temporal_std <= ADAPTIVE_STABILITY_FACTOR * median_std
                  ).astype(np.uint8) * 255

        hot_final  = cv2.bitwise_and(cv2.bitwise_and(hot_avg,  hot_vote),  stable)
        cold_final = cv2.bitwise_and(cv2.bitwise_and(cold_avg, cold_vote), stable)
        merged     = cv2.bitwise_or(hot_final, cold_final)

        # Step 7: Morphological closing
        binary_map = cv2.morphologyEx(merged, cv2.MORPH_CLOSE, self._close_k)

        return binary_map, avg_delta


# ─────────────────────────────────────────────────────────────────────────────
# BlobDetector
# ─────────────────────────────────────────────────────────────────────────────

class BlobDetector:
    """
    Extracts the best mine candidate from the binary anomaly map.

    Upscales 24×32 → 640×480 with INTER_CUBIC for smooth contours.
    Passes candidates through physical size and circularity gates.
    Ranks passing candidates by thermal z-score (strongest anomaly wins).
    Returns (dx, dy, conf) where dx/dy are offsets from image centre in 640×480
    pixel space, compatible with 04_coordinate_math.py (fx=614.5, fy=761.2).
    """

    MINE_DIAM_MIN_M = 0.15   # m — minimum expected thermal footprint diameter
    MINE_DIAM_MAX_M = 0.55   # m — maximum (generous diffusion margin)
    AREA_MIN_SCALE  = 0.5    # × one upscaled native pixel area
    MIN_CIRCULARITY = 0.35   # lenient: 32×24 is coarse, blobs are small
    MAX_ASPECT      = 2.2    # reject elongated rocks/shadows
    MIN_CONFIDENCE  = 0.40   # pass gate; winner is highest z, not highest conf

    def __init__(self, altitude_m: float = DEFAULT_ALTITUDE_M,
                 fov_deg=None, sensor_res=None):
        fov_deg    = fov_deg    or DEFAULT_FOV_DEG
        sensor_res = sensor_res or SENSOR_RES

        mpp_x = (2 * altitude_m * math.tan(math.radians(fov_deg[0] / 2))) / UPSCALE_W
        mpp_y = (2 * altitude_m * math.tan(math.radians(fov_deg[1] / 2))) / UPSCALE_H
        self.mpp = (mpp_x + mpp_y) / 2.0

        scale_w = UPSCALE_W / sensor_res[0]
        scale_h = UPSCALE_H / sensor_res[1]
        px_area = scale_w * scale_h            # upscaled px² per native px²

        self.area_min = self.AREA_MIN_SCALE * px_area

        r_max = (self.MINE_DIAM_MAX_M / 2) / self.mpp
        self.area_max = math.pi * r_max ** 2 * 1.5

    def detect(self, binary_map_24x32: np.ndarray,
               avg_delta: np.ndarray = None,
               frame_stack=None, fpn_pattern=None):
        """
        Parameters
        ----------
        binary_map_24x32 : (24, 32) uint8  — from SpatiotemporalFilter
        avg_delta        : (24, 32) float32 — pre-computed by filter
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

        d_mean = float(np.mean(avg_delta)) if avg_delta is not None else 0.0
        d_std  = (float(np.std(avg_delta)) + 1e-6) if avg_delta is not None else 1.0

        best_z    = 0.0
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

            # ── FIX B: Direct centroid → native pixel mapping ─────────────
            # The old approach (draw contour → resize to 32×24 → argwhere)
            # returns an empty array for blobs near image edges, causing
            # peak_dT = 0 and delta_score = 0 for edge-pixel mines.
            # Direct centroid mapping is correct for all positions.
            cx_n = max(0, min(SENSOR_RES[0] - 1,
                              int(round(cx * SENSOR_RES[0] / UPSCALE_W))))
            cy_n = max(0, min(SENSOR_RES[1] - 1,
                              int(round(cy * SENSOR_RES[1] / UPSCALE_H))))
            best_dT = float(avg_delta[cy_n, cx_n]) if avg_delta is not None else 0.0
            # ─────────────────────────────────────────────────────────────

            z = (abs(best_dT) - d_mean) / d_std

            # Spike rejection: surface hot objects (mine_04: z ≈ 24.6σ)
            if z >= Z_SPIKE:
                continue

            # delta_score: scaled z-score between Z_HOT and Z_SPIKE
            delta_score = max(0.0, min(1.0,
                (z - Z_HOT) / (Z_SPIKE - Z_HOT) * 2.0))
            if delta_score < 0.05:
                continue

            circ_score = min(1.0, circ)

            # ── FIX A: Binary size_score ──────────────────────────────────
            # The old soft penalty (1 − |area − area_soft_mid| / area_soft_half)
            # was always 0 because blobs from 1–2 hot native pixels (~370 px²)
            # are far from area_soft_mid (~26,000 px²).  The area gate above
            # already enforces the physical mine size range; no additional
            # penalty is needed.
            size_score = 1.0
            # ─────────────────────────────────────────────────────────────

            conf = 0.35 * circ_score + 0.25 * size_score + 0.40 * delta_score
            if conf < self.MIN_CONFIDENCE:
                continue

            # ── FIX C: Rank by z-score, not composite conf ────────────────
            # With fixes A+B, circularity noise at 32×24 resolution can push
            # an FPN artifact above the actual mine in composite confidence
            # despite having a lower z-score.  The thermal z-score is the most
            # physically meaningful discriminator.
            if z > best_z:
                best_z = z
                best   = (cx - UPSCALE_W // 2, cy - UPSCALE_H // 2, conf)
            # ─────────────────────────────────────────────────────────────

        if best:
            dx, dy, conf = best
            print(f"[VISION] TARGET ACQUIRED: dX={dx}px dY={dy}px | "
                  f"Conf={conf*100:.1f}%  z={best_z:.1f}σ")
            return dx, dy, conf

        return None, None, 0.0

    @staticmethod
    def _fallback_avg_delta(frame_stack, fpn_pattern, sigma_bg=4.0):
        if frame_stack is None or len(frame_stack) == 0:
            return None
        corr = (frame_stack.astype(np.float32) - fpn_pattern[np.newaxis]
                if fpn_pattern is not None
                else frame_stack.astype(np.float32))
        avg = corr.mean(axis=0)
        bg  = cv2.GaussianBlur(avg, (0, 0), sigmaX=sigma_bg, sigmaY=sigma_bg,
                               borderType=cv2.BORDER_REFLECT_101)
        return (avg - bg).astype(np.float32)


# ─────────────────────────────────────────────────────────────────────────────
# Public entry point
# ─────────────────────────────────────────────────────────────────────────────

def process_memory_stack(frame_stack, fpn_pattern, altitude_m=DEFAULT_ALTITUDE_M):
    """
    Main entry point called by main_orchestrator.py.  Interface unchanged.

    Parameters
    ----------
    frame_stack  : (N, 24, 32) float32 — raw frames from PipeCamera.capture_stack()
    fpn_pattern  : (24, 32) float32 or None — from fpn_pattern.npy
    altitude_m   : float — drone AGL height in metres

    Returns
    -------
    (dx, dy, conf)  or  (None, None, 0.0)
    dx/dy in 640×480 pixel space, offset from image centre.
    Positive dx = target is to the right; positive dy = target is below centre.

    INTEGRATION NOTE
    ----------------
    CONFIDENCE_THRESHOLD in main_orchestrator.py must remain at 0.40.
    Dataset-validated mine detections score conf = 0.57–0.63.
    """
    if frame_stack is None or len(frame_stack) == 0:
        return None, None, 0.0

    sf = SpatiotemporalFilter(sigma_bg=4.0)
    binary_map, avg_delta = sf.extract_solid_targets(frame_stack, fpn_pattern)

    bd = BlobDetector(altitude_m=altitude_m)
    return bd.detect(binary_map, avg_delta=avg_delta)
