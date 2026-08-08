# 04 — Ultra-Wideband (UWB) Ranging

## Centimetre-accurate relative positioning without GPS

---

## The layman version

Wi-Fi and Bluetooth tell you "you're near the router" — they give signal strength, not
real distance. **Ultra-Wideband (UWB)** sends a very short radio pulse and measures
*exactly* how long it takes to arrive. Since radio travels at the speed of light, time
= distance to a few centimetres.

Think of it like sonar (PING → echo timing), but with radio waves instead of sound,
and accurate to ~10 cm instead of ~10 cm with sonar but without line-of-sight
constraints (UWB goes through thin walls, plastic, and wooden structures).

---

## How UWB positioning works

### Time of Flight (ToF) — one-way ranging

```
Anchor A ──pulse─────────────────────→ Tag (drone)
         t_send                     t_receive

distance = (t_receive - t_send) × c
         = Δt × 3×10⁸ m/s
```

Problem: clocks on both devices must be perfectly synchronised (1 ns error = 30 cm error).

### Two-Way Ranging (TWR) — eliminates clock sync requirement

```
Anchor A ──MSG──────────────────→ Tag
Tag      ──RESP─────────────────→ Anchor A
Anchor A ──FINAL────────────────→ Tag

distance = c × (T_round - T_reply) / 2
```

This is the method used by DW1000 / DW3000 chipsets (Decawave/Qorvo). Clock errors
cancel in the calculation.

### Trilateration — from distances to positions

With 3 known anchor positions A₁, A₂, A₃ and measured distances d₁, d₂, d₃:

```
(x - x₁)² + (y - y₁)² = d₁²
(x - x₂)² + (y - y₂)² = d₂²
(x - x₃)² + (y - y₃)² = d₃²

→ Solve for (x, y) of the tag
```

4 anchors gives an overdetermined system → least-squares solution, more robust to
measurement noise. In 3D (altitude needed), 4 anchors minimum for full XYZ.

---

## Accuracy numbers for micro-UAV swarms

| Study / Paper | Setup | Accuracy (RMSE) | Notes |
|---|---|---|---|
| Nguyen et al. 2021 | 4 anchors, indoor | **0.167 m** | Micro-UAV swarm, TWR |
| Guo et al. 2020 | 6 anchors, outdoor | **0.12 m** | 20 m × 20 m field |
| Queralta et al. 2022 | 4 anchors, mixed | 0.08–0.25 m | Varies by multipath |
| DWM1001 dev kit docs | Standard TWR | 0.10 m typical | Manufacturer spec |
| Our expected (4 corners) | 4 anchors, 15×60 m | **~0.15–0.25 m** | Larger field → slightly worse |

For our 15 m × 60 m Robofest field, placing anchors at the 4 corners and measuring:
- Range: 0–15 m wide, 0–60 m long
- Expected accuracy: 15–25 cm (geometry dilutes precision at field corners)

This is better than optical flow (~30 cm) and far better than dead reckoning (~300 cm).

---

## Hardware options

### DW1000/DW3000 — the industry standard chipset

| Module | Chipset | Weight | Cost | Notes |
|---|---|---|---|---|
| **Decawave MDEK1001** | DW1000 | ~15 g each | ~$30/node | Most documented, easy setup |
| **Nooploop LinkTrack P** | Custom | ~10 g tag | ~$80/tag | Plug-and-play, good docs |
| **Bitcraze LPS node** | DW1000 | ~20 g anchor | ~$35/anchor | Works with Crazyflie, open SW |
| **Makerfabs UWB kit** | DW3000 | ~8 g tag | ~$20/tag | Cheaper, newer chip |
| **Sewio RTLS anchor** | Custom | 200 g anchor | ~$200 | Commercial grade, overkill |

**Best ROI for Robofest:** Makerfabs DW3000 tags on drones (~8 g, ~$20 each) + 4
DW3000 anchors at field corners placed by team before flight (~$80 total for anchors).

### Weight budget for our drones

```
Current drone budget: 750 g max, currently ~620 g estimated
UWB tag (DW3000 module): ~8 g
PCB + connectors: ~5 g
Total UWB addition: ~13 g per drone ✓ (well within budget)
```

---

## Setup for Robofest

### Anchor placement

```
Field layout (top view, 15 m × 60 m):

A1 ──────────────────────── A2
│  ←────── 15 m ────────→  │
│                            │
│           FIELD            │  60 m
│                            │
│   drones fly inside here   │
│                            │
A3 ──────────────────────── A4

A1: (0, 0)
A2: (15, 0)
A3: (0, 60)
A4: (15, 60)
```

Each anchor is a small battery-powered node (DW3000) on a tripod ~1.5 m high. The
team sets them up during the 10-minute preparation window before flight starts.

The drone tags range to all 4 anchors simultaneously at ~50 Hz update rate.

### ArduPilot integration

ArduPilot does NOT natively support UWB position input. Two approaches:

**Option A: VISION_POSITION_ESTIMATE (recommended)**
Run UWB → position solver on the Raspberry Pi, then send as external position:

```python
import serial
import json
from pymavlink import mavutil

# UWB module sends ranges via serial (JSON format for many modules)
uwb = serial.Serial('/dev/ttyUSB0', 115200)

# Anchor positions (measured with tape measure at field setup)
ANCHORS = {
    'A1': (0.0,  0.0,  1.5),
    'A2': (15.0, 0.0,  1.5),
    'A3': (0.0,  60.0, 1.5),
    'A4': (15.0, 60.0, 1.5),
}

mav = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)

def trilaterate(ranges):
    """Least-squares 2D trilateration from anchor ranges."""
    import numpy as np
    A = []
    b = []
    anchors = list(ANCHORS.values())
    for i in range(1, len(anchors)):
        x0, y0, _ = anchors[0]
        xi, yi, _ = anchors[i]
        d0 = ranges[0]
        di = ranges[i]
        A.append([2*(xi - x0), 2*(yi - y0)])
        b.append([di**2 - d0**2 - xi**2 + x0**2 - yi**2 + y0**2])
    A, b = np.array(A), np.array(b)
    pos = np.linalg.lstsq(A, b, rcond=None)[0]
    return float(pos[0]), float(pos[1])

while True:
    line = uwb.readline().decode().strip()
    data = json.loads(line)  # e.g. {"A1": 3.21, "A2": 14.1, "A3": 58.4, "A4": 45.2}
    ranges = [data['A1'], data['A2'], data['A3'], data['A4']]
    x, y = trilaterate(ranges)
    
    # Send to ArduPilot EKF3
    mav.mav.vision_position_estimate_send(
        usec=int(time.time() * 1e6),
        x=x, y=y, z=0,  # altitude from barometer
        roll=0, pitch=0, yaw=0,  # heading from compass
        covariance=[0.04] * 21   # σ = 0.2 m → σ² = 0.04
    )
```

**Option B: AP_Proximity (for obstacle avoidance, not positioning)**
Some UWB implementations work as proximity sensors. Not useful for our use case.

ArduPilot EKF3 parameters (same as VIO):
```
EK3_SRC1_POSXY = 6   # ExternalNav for XY
EK3_SRC1_VELXY = 0   # Dead reckoning for velocity (UWB gives position, not velocity)
VISO_TYPE      = 1   # Enable external nav input
```

---

## Critical problem: Robofest rules and pre-placement

**Competition rule constraint:** Can we place anchors before the drones fly?

The Robofest 6.0 problem statement says:
- "no GPS inside the minefield" → implies no pre-existing infrastructure inside
- Teams typically have a setup/preparation window (usually 10–15 minutes)

**If anchors are allowed:** UWB gives ~0.15 m accuracy, better than any other option.

**If anchors are NOT allowed (strict no-infrastructure interpretation):**
- Drone-to-drone UWB ranging is still valid (relative positioning within swarm)
- Position within field can't be known absolutely, but relative formation can be
- One drone starts from a known position and the swarm maintains relative geometry

### Swarm-relative positioning (no fixed anchors)

With 3 drones all carrying UWB, they range to each other:
```
Drone 1 ←──d12──→ Drone 2
    ↑ d13            ↑ d23
    └──────────── Drone 3
```

This gives relative positions accurate to ~0.15 m. Combined with initial absolute
position from the origin lock (GPS averaged before entering field), the swarm can
track absolute positions for several minutes before drift accumulates.

---

## Multipath and outdoor considerations

UWB is much less susceptible to multipath (signal reflections) than Wi-Fi, but:

- **Ground reflection:** At 1.5 m anchor height over flat grass, ground reflection
  adds ~0.05 m bias. Compensate by calibrating anchor height in trilateration.

- **Body shadowing:** A drone can shadow UWB signal between itself and an anchor.
  With 4 anchors, losing one still leaves 3 — enough for 2D positioning.

- **Antenna orientation:** DW3000 has directional gain patterns. Mounting horizontally
  on the drone and anchors vertically facing outward is recommended.

- **Interference:** UWB (3.1–10.6 GHz) can interfere with 5.8 GHz FPV video.
  Use 6.5 GHz UWB centre frequency band, not 5.8 GHz sub-band.

---

## Power consumption

| Component | Current draw | Notes |
|---|---|---|
| DW3000 ranging mode | ~70 mA at 3.3V | ~230 mW |
| DW3000 listen mode | ~35 mA at 3.3V | When waiting for range request |
| 50 Hz update rate | ~70 mA typical | Continuous two-way ranging |
| Impact on flight time | ~0.3% | Negligible vs. motors |

---

## Summary for Robofest

| Factor | Assessment |
|---|---|
| Accuracy | ✅ ~0.15–0.25 m — best of all approaches |
| Weight | ✅ ~13 g per drone |
| Cost | ✅ ~$20/tag + ~$80 for 4 anchors |
| Compute | ✅ Trilateration is near-zero CPU (microseconds) |
| Infrastructure | ⚠️ Requires 4 anchors placed at field corners |
| Rules compliance | ❓ Depends on whether pre-placed infrastructure is permitted |
| Outdoor suitability | ✅ Good, minor multipath from ground only |
| Integration effort | 🔶 Medium — no native ArduPilot driver, need Pi bridge |

---

## References

1. **Nguyen, T.H., et al. (2021).** *Centimeter-Level Localization Using Ultra-Wideband
   Communication for Micro-Aerial Vehicle Swarms.*
   https://arxiv.org/abs/2109.11899

2. **Queralta, J.P., et al. (2022).** *UWB-Based System for UAV Localization in
   GPS-Denied Environments.* IROS 2022.

3. **DW3000 datasheet — Qorvo:**
   https://www.qorvo.com/products/p/DW3000

4. **Bitcraze LPS (Loco Positioning System) — open source UWB for drones:**
   https://www.bitcraze.io/products/loco-positioning-system/

5. **ArduPilot external position source documentation:**
   https://ardupilot.org/copter/docs/common-non-gps-navigation.html
