# 01 — Thermal Infrared Mine Detection

## Why this matters for us

Our entire detection pipeline (`02_vision_filter.py`, `06_surface_filter.py`) is built on the
physics of thermal contrast between mines and surrounding soil. Understanding *why* the contrast
exists, when it's strongest, and what degrades it is critical for tuning thresholds and for
deciding **when in the day to fly**.

---

## Key Paper: UAV-based detection of landmines using infrared thermography

**Reference:** arXiv:2410.23998 (2024)  
**URL:** https://arxiv.org/abs/2410.23998

### What it says

- End-to-end system: UAV + thermal camera → base computer → image processing algorithm
- Best accuracy: **~88% detection** — achieved during **evening hours**
- Daytime detection degrades significantly due to solar heating of surface soil masking the buried object's thermal signature
- Uses high-resolution thermal imaging (not 32×24 like our MLX90640) but the physics is the same

### Why evening/night is optimal

Buried mines have higher **thermal inertia** than loose soil:
- During the day: solar radiation heats the surface soil faster than the mine → contrast washes out
- At dusk and night: surface soil cools faster than the mine → mine appears **warmer** than surroundings
- The optimal window is typically **1–3 hours after sunset**

### Connection to our code

Our `02_vision_filter.py` detects buried mines as a **bidirectional** anomaly (ΔT ±0.15–1.25°C).
The bidirectional threshold (both warm and cold anomalies) is intentional: at different times of
day, the same mine can appear warmer OR cooler depending on solar cycle phase.

**For Robofest 6.0:** If the competition schedule allows evening flight windows, detection
confidence will be significantly higher. Document the time of day during field calibration.

---

## Key Dataset: Thermographic Images of Buried Landmines

**Reference:** Data in Brief, 49, 109312 (2023) — PMC10403701  
**DOI:** https://doi.org/10.1016/j.dib.2023.109312

### What it says

- Platform: DJI Matrice 100 with Zenmuse XT thermal camera
- Field conditions: controlled burial depths 0–10 cm
- Dataset: ~2700 thermographic images with ground truth
- Key finding: detection contrast is highest at **5–10 cm depth**, drops sharply below 15 cm

### Our mine depth context

The Robofest problem statement specifies two mine types:
- **Buried mines** — detected by `02_vision_filter.py` (ΔT ±0.15–1.25°C bidirectional)
- **Surface discs** — detected by `06_surface_filter.py` (ΔT +3–40°C, one-directional hot)

Surface discs are trivially easy to detect thermally. Buried mines are the hard case. The dataset
confirms that at typical contest burial depths (<10 cm), a 32×24 sensor *can* detect them in good
conditions — but the contrast is marginal.

**For Robofest 6.0:** Tune `BURIED_MINE_DELTA_T_LOW` (currently 0.15°C) against real field
captures in `slave/test_data/captures/`. Consider whether the current threshold is too aggressive
for shallow burials in high-sun conditions.

---

## Fixed Pattern Noise (FPN) — what it is and why we correct for it

**Reference:** Barral et al., WACV 2024 — https://openaccess.thecvf.com/content/WACV2024/papers/Barral_Fixed_Pattern_Noise_Removal_for_Multi-View_Single-Sensor_Infrared_Camera_WACV_2024_paper.pdf  
**Also:** IEEE Xplore 10122709 — https://ieeexplore.ieee.org/document/10122709/

### What FPN is

Every pixel in the MLX90640's 32×24 focal plane array has a slightly different:
- Dark current (the signal it reads even with no light)
- Gain (how much it amplifies the actual signal)

This results in a fixed, repeatable spatial pattern overlaid on every frame — like a permanent
"fingerprint" of hot and cold pixels that is always there regardless of what the camera is
looking at.

### The effect on mine detection

Without FPN correction, a cold or warm pixel might fire our vision filter even when pointing at
bare soil — because that pixel *always* reads slightly higher/lower than its neighbours.

### How our code handles it

`00_preflight_calib.py` captures 30 frames with the lens cap on (or pointing at a uniform
warm surface), then saves the mean as `slave/config/fpn_pattern.npy`. The vision filter
subtracts this from every live frame before computing ΔT.

The FPN pattern stored in the repo (`slave/config/fpn_pattern.npy`) is from a real field
calibration — **do not replace it without running a fresh calibration on the actual sensor**.

### FPN changes over time

Importantly, FPN is not perfectly stable — it drifts with:
- Sensor temperature (heats up during flight)
- Integration time changes
- Age of the sensor

**For Robofest 6.0:** Run `00_preflight_calib.py` before *every* competition flight, not just
once per season. The calibration takes ~30 seconds.

---

## Detection Thresholds — tuning guide

| Parameter | Current value | Located in | What it controls |
|-----------|--------------|------------|-----------------|
| `BURIED_DELTA_T_LOW` | 0.15°C | `02_vision_filter.py` | Min ΔT to flag as possible buried mine |
| `BURIED_DELTA_T_HIGH` | 1.25°C | `02_vision_filter.py` | Max ΔT (above this = surface object, not buried) |
| `SURFACE_DELTA_T_LOW` | 3.0°C | `06_surface_filter.py` | Min ΔT for surface disc |
| `SURFACE_DELTA_T_HIGH` | 40.0°C | `06_surface_filter.py` | Sanity cap (above this = fire/person, not mine) |
| `PERSIST_DRIFT_M` | 1.5 m | `03_persistence.py` | Max pixel drift allowed during re-hover confirmation |
| `PERSIST_FRAMES` | 12 | `03_persistence.py` | Frames to re-capture during confirmation |

**How to retune:** Use the captured data in `slave/test_data/captures/` and the code snippet
in `tools/README.md` to run offline against known mine captures. Adjust thresholds until all
6 known captures are detected without false positives on the bare-soil frames.

---

## Further reading

- Melexis application note: "MLX90640 in non-uniformity correction mode" — ask Melexis support
- NATO ITEP (International Test & Evaluation Program) mine detection studies — not publicly available in full, but summaries exist
- FLIR Lepton 3.5 (alternative 160×120 sensor) — significantly better resolution but ≈3× cost and ≈2× power, worth evaluating for Robofest 7.0
