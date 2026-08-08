# 02 — Optical Flow Navigation

## The cheapest meaningful upgrade to our current system

---

## The layman version

Look at the ground below a moving drone through a downward-facing camera. As the drone
drifts sideways, the ground texture shifts in the opposite direction in the image.

Optical flow is the process of **measuring how fast the image pixels are moving**, then
converting that pixel velocity into a physical velocity estimate using the known altitude.

It is the same principle a fly uses to navigate — not knowing its absolute position, but
knowing how fast the world is passing beneath it.

---

## The physics

### From pixels to metres

If a drone is at altitude **h** metres and pixels are moving at **v_px** pixels/second
across a camera with focal length **f** pixels, then the drone's physical velocity is:

```
v_physical (m/s) = v_px × h / f
```

This means:
- Higher altitude → same pixel speed = faster physical motion
- You MUST know altitude accurately for optical flow to work
- Our barometer provides altitude — this is why altitude accuracy matters

### The flow field

In a video stream, each pixel has a flow vector (dx, dy) telling you how far it moved
between frames. Computing this across the entire image gives a **flow field**. The
average flow across the image (after removing rotation effects from the gyroscope) gives
the camera's translational velocity.

The rotation correction is critical — if the drone rotates slightly, pixels appear to
move even when the drone isn't translating. We subtract the rotational component using
gyroscope readings:

```
v_translational = v_observed_flow - (gyro_rate × h)
```

---

## Hardware options

### Option A — PMW3901 (recommended for Robofest 6.0)

The PMW3901 is a tiny optical mouse sensor chip originally designed for gaming mice,
repurposed for drone navigation.

| Spec | Value |
|---|---|
| Weight | ~3 g (sensor only) |
| Size | 22 × 22 mm |
| Output | XY pixel velocity at up to 100 Hz |
| Range | Works 80 mm to infinity (best: 0.3–2 m) |
| Interface | SPI |
| Cost | ~$15–30 |

Modules available: **Holybro PMW3901**, **Matek 3901-L0X** (includes TF-Luna-equivalent
ranging), **ThoneFlow-3901** (standalone).

**Matek 3901-L0X** is the best choice for our setup — it combines PMW3901 optical flow
with an L0X laser rangefinder on one board (~5 g, ~$25). The L0X provides altitude
independently of the barometer.

### Option B — HereFlow (CAN bus)

CubePilot's HereFlow combines PMW3901 + lidar on a CAN bus module.

| Spec | Value |
|---|---|
| Weight | ~16 g |
| Interface | CAN (UAVCAN/DroneCAN) |
| Altitude sensor | Included laser (2 m range) |
| ArduPilot support | Native, well-tested |
| Cost | ~$60 |

HereFlow is more robust but heavier and costs more. Suitable if you want
a polished plug-and-play solution.

### Option C — PiCam + Lucas-Kanade (software on RPi)

Use our existing Raspberry Pi camera to compute optical flow in software.

| Spec | Value |
|---|---|
| Weight | 0 g extra (camera already exists) |
| Compute cost | ~30% of one Pi4 core at 30 Hz |
| Accuracy | Lower than dedicated chip (vibration-sensitive) |
| Setup complexity | Higher — needs calibration and MAVLink bridge |

The Lucas-Kanade algorithm tracks sparse feature points between frames. OpenCV has
it built in (`cv2.calcOpticalFlowPyrLK`). The output can be sent to ArduPilot via
`OPTICAL_FLOW` MAVLink messages.

This is zero-hardware-cost but requires a vibration-damped camera mount and careful
calibration.

---

## ArduPilot EKF3 integration

ArduPilot EKF3 can fuse optical flow natively. Parameters to set:

```
# Enable optical flow
FLOW_TYPE = 10       # CXOF / PMW3901-style sensor on UART
# or
FLOW_TYPE = 5        # DroneCAN (HereFlow)

# EKF3 source configuration (GPS-denied mode)
EK3_SRC1_POSXY = 0   # No GPS for XY position
EK3_SRC1_VELXY = 5   # Optical flow for XY velocity
EK3_SRC1_POSZ  = 1   # Barometer for Z
EK3_SRC1_VELZ  = 0   # No Z velocity source
EK3_SRC1_YAW   = 1   # Compass for heading

# Flow sensor orientation (if mounted rotated)
FLOW_ORIENT_YAW = 0  # degrees — adjust to match your mounting

# Flow quality threshold
FLOW_FXSCALER = 0    # Scale factor — calibrate empirically
FLOW_FYSCALER = 0
```

### How EKF3 uses flow

EKF3 treats optical flow as a **velocity measurement**, not a position measurement.
This is important:
- Flow corrects **velocity drift** (slows down position drift)
- But does NOT give absolute position
- After 10 minutes, some drift still accumulates, but much slower

Typical drift with optical flow + barometer: **~0.05–0.2 m/minute** vs
**0.3–0.5 m/minute** without it — roughly 3× improvement.

---

## Limitations and failure modes

### 1. Feature-poor ground
Optical flow needs **texture** to track. If the ground is:
- A uniform colour (sand, concrete) → tracking fails
- Wet (reflective) → reflection patterns confuse the algorithm
- Moving (grass blowing in wind) → false velocity readings

Robofest 6.0 fields are typically grass or mixed terrain outdoors — generally
workable but needs validation.

### 2. Height dependence
Flow velocity converts to physical speed using altitude. If altitude estimate is wrong:
```
v_actual = v_measured × h_actual / h_estimated
```
A 10% altitude error → 10% velocity scale error → integrates into position error.

### 3. Sensor vibration
Motor vibration corrupts optical flow readings. Solutions:
- Soft mounting (foam/rubber standoffs)
- Hardware vibration damping on flight controller
- High IMU pre-filter (SpeedyBee has hardware filtering)

### 4. Low light / high speed
PMW3901 works best in daylight. At >3 m/s horizontal speed, pixel motion may exceed
the sensor's trackable range. Our scan speed of ~0.5–1 m/s is well within limits.

### 5. Wind-induced body oscillation
If wind pushes the drone and it oscillates, the flow sensor oscillates too,
and the averaged velocity may not represent true translation.

---

## Practical accuracy numbers

| Condition | Drift over 10 min | Position error at 60 m |
|---|---|---|
| No flow (current) | ~3–5 m total | ±5 m at field far end |
| PMW3901 optical flow | ~0.5–1.5 m total | ±1.5 m at field far end |
| PMW3901 + height sensor | ~0.3–0.8 m total | ±0.8 m at field far end |
| HereFlow (high quality) | ~0.2–0.5 m total | ±0.5 m at field far end |

These numbers assume good lighting, textured ground, and proper vibration damping.

---

## Weight and cost budget

For Robofest 6.0 (750 g limit per drone):

| Component | Weight | Cost |
|---|---|---|
| Matek 3901-L0X | ~5 g | ~$25 |
| Mounting hardware | ~2 g | ~$2 |
| Wiring to FC UART | ~1 g | — |
| **Total addition** | **~8 g** | **~$27** |

This is the **best ROI upgrade** available — 8 g and ~$27 per drone, 3×
improvement in navigation accuracy.

---

## Implementation checklist

- [ ] Purchase Matek 3901-L0X or PMW3901-based module
- [ ] Mount under drone, lens pointing downward, vibration-isolated
- [ ] Connect to SpeedyBee F405 UART (UART1 or UART4 recommended)
- [ ] Set `FLOW_TYPE` and `SERIAL_x_PROTOCOL = 18` (optical flow)
- [ ] Set `EK3_SRC1_VELXY = 5` to enable flow fusion
- [ ] Calibrate in a textured area (measure actual vs GPS to find scale errors)
- [ ] Test LOITER mode without GPS — drone should hold position within 0.5 m

---

## References

1. **ArduPilot Optical Flow setup:**
   https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html

2. **HereFlow documentation:**
   https://ardupilot.org/copter/docs/common-hereflow.html

3. **Walczak et al. (2025).** *Fusion of Optical Flow and Dead Reckoning Algorithms
   for UAV Navigation Without GPS.* TransNav Journal.
   https://www.transnav.eu/Article_Fusion_of_Optical_Flow_and_Dead_Walczak,76,1592.html

4. **ArduPilot EKF3 optical flow drift issue (community data):**
   https://github.com/ArduPilot/ardupilot/issues/28242

5. **Matek 3901-L0X product page:**
   https://www.mateksys.com/?portfolio=3901-l0x
