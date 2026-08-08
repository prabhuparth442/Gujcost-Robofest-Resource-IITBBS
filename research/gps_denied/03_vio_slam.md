# 03 — Visual Inertial Odometry (VIO) & SLAM

## Computer vision fused with IMU — the modern high-accuracy approach

---

## The layman version

Imagine walking through an unfamiliar building with your eyes open. You notice a chair
in the corner, a crack in the wall, a painting. When you walk back past them, you
recognise them and think: "I've looped around — I'm back near the entrance." You're
building an internal map of landmarks and using them to know where you are.

**SLAM (Simultaneous Localization and Mapping)** is exactly this — a drone uses a
camera to detect and track visual landmarks, builds a map of them, and uses that map
to estimate its position.

**VIO (Visual Inertial Odometry)** is a simpler, lighter version: instead of building a
full map, it just tracks how features move between frames and fuses that with IMU data
to estimate motion. Less memory, faster computation, but no loop closure.

---

## VIO — how it works

### Feature tracking pipeline

```
Frame N        Frame N+1
  ┌───┐          ┌───┐
  │ × │          │  ×│  ← same feature, moved right
  │   │  ----→   │   │
  │   ×│          │  ×│  ← another feature, moved right
  └───┘          └───┘

Apparent pixel motion = camera translation + camera rotation
                      - rotation (from gyroscope) = translation only
```

Steps:
1. Detect image features (corners, edges, blobs) in frame N
2. Track those same features in frame N+1 using Lucas-Kanade or similar
3. Compute the camera's essential/fundamental matrix from matched features
4. Decompose into rotation and translation (scale is unknown without depth)
5. Fuse with IMU to resolve scale and smooth out estimation

### Scale problem in monocular VIO

A single camera cannot tell the difference between:
- A small object close up moving slowly
- A large object far away moving fast

This ambiguity means monocular VIO cannot recover absolute scale without external
input. Solutions:
- **Stereo camera** — baseline between two lenses gives depth directly
- **IMU fusion** — gravity vector and known acceleration provides scale
- **Height sensor** — barometer or lidar gives absolute altitude → resolves scale

**VINS-Mono** (Monocular Visual-Inertial Nonlinear System) is a landmark paper
(Qin, Li, Shen — 2018, IEEE TRO) that solved scale recovery for monocular systems by
tightly coupling camera + IMU in a nonlinear optimization.

---

## SLAM — extending VIO with map and loop closure

VIO drifts because small errors accumulate frame by frame. SLAM adds:

### 1. A keyframe map
Every N frames, save a "keyframe" with the features detected. Build a growing database
of "this place looks like this."

### 2. Loop closure detection
When a new frame resembles a keyframe from long ago, the system recognises it:
"I've been here before — my current position estimate must match that keyframe's
position." This provides a **global correction** that resets accumulated drift.

Without loop closure: drift = O(t) — grows linearly with time
With loop closure: drift ≈ 0 at loop points, bounded between closures

### 3. Bundle adjustment
After loop closure, SLAM re-optimises all keyframe poses simultaneously to be
globally consistent. This is computationally expensive but only runs periodically.

### Major open-source SLAM systems

| System | Type | CPU on Pi4 | Accuracy | Field tested |
|---|---|---|---|---|
| **ORB-SLAM3** | Stereo/Mono-Inertial | ~80–150% 1 core | ±1.7 m outdoor stereo | Yes |
| **VINS-Mono** | Monocular-Inertial | ~60–80% 1 core | ±0.5–2 m outdoor | Yes |
| **VINS-Fusion** | Stereo/Mono + GPS fusion | ~100–180% 1 core | ±0.3–0.8 m | Yes |
| **OpenVINS** | Monocular-Inertial (EKF) | ~40–60% 1 core | ±0.4–1.0 m | Yes |
| **Kimera-VIO** | Stereo-Inertial + mesh | ~200%+ 2 cores | ±0.2–0.6 m | Research |

**For Raspberry Pi 4 (4 GB) with our existing drone code:**
ORB-SLAM3 stereo or VINS-Mono are the most practical. We have significant CPU budget
left on the Pi since our detection pipeline runs intermittently.

---

## ArduPilot EKF3 integration via VISION_POSITION_ESTIMATE

ArduPilot supports external vision position input via MAVLink:

```python
# On Raspberry Pi — send VIO position to ArduPilot EKF3
from pymavlink import mavutil

mav = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)

def send_vision_position(x, y, z, roll, pitch, yaw, covariance=0.1):
    """
    Send VIO pose estimate to ArduPilot EKF3.
    x, y, z in metres (NED frame)
    roll, pitch, yaw in radians
    """
    mav.mav.vision_position_estimate_send(
        usec=int(time.time() * 1e6),  # timestamp microseconds
        x=x,
        y=y,
        z=z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        covariance=[covariance] * 21  # 6×6 upper triangular
    )
```

ArduPilot EKF3 parameters for vision input:
```
EK3_SRC1_POSXY = 6   # ExternalNav for XY position
EK3_SRC1_VELXY = 6   # ExternalNav for XY velocity
EK3_SRC1_POSZ  = 6   # ExternalNav for Z (or 1=baro)
EK3_SRC1_YAW   = 6   # ExternalNav for yaw (or 1=compass)
VISO_TYPE      = 1   # Enable visual odometry input
```

When EKF3 receives VISION_POSITION_ESTIMATE at >5 Hz with low covariance, it will
trust it heavily and significantly reduce dead-reckoning drift.

---

## Practical VIO implementation on our system

### Hardware needed

| Component | Weight | Cost | Notes |
|---|---|---|---|
| Pi Camera v2 (already have) | 3 g | — | May work for VIO |
| OV9281 global shutter mono | 3–5 g | ~$15 | Much better for VIO (no rolling shutter) |
| ArduCam stereo hat | ~20 g | ~$35 | Two OV9281 sensors on one board |
| IMU (already on SpeedyBee) | — | — | Shared via MAVLink |

**Key requirement:** A **global shutter** camera (like OV9281) is strongly preferred
for VIO. Standard Pi cameras use rolling shutter — when the drone vibrates, each row
of the image is captured at a slightly different time, distorting the image and
corrupting feature tracking.

### Software stack

```
┌──────────────────────────────────────────────────────┐
│  Raspberry Pi 4                                       │
│                                                       │
│  Camera (OV9281) ──→ OpenCV frame capture            │
│  IMU data ←────────── MAVLink from SpeedyBee        │
│                                                       │
│  VIO system (VINS-Mono or OpenVINS):                  │
│    - Feature detection (ORB or FAST)                  │
│    - IMU pre-integration                              │
│    - Nonlinear optimisation (Ceres/g2o)               │
│    - Pose output (X, Y, Z, roll, pitch, yaw)          │
│                              │                        │
│                              ↓                        │
│  MAVLink bridge:                                      │
│    VISION_POSITION_ESTIMATE → SpeedyBee EKF3         │
│                                                       │
│  Our detection pipeline (parallel thread):            │
│    MLX90640 → detection → mine reports                │
└──────────────────────────────────────────────────────┘
```

### VINS-Mono installation on Raspberry Pi 4

```bash
# Install ROS Noetic (VINS-Mono requires ROS)
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install ros-noetic-ros-base

# Install dependencies
sudo apt install libceres-dev libeigen3-dev libopencv-dev

# Clone and build VINS-Mono
cd ~/catkin_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git
cd ~/catkin_ws && catkin_make -j2  # -j2 to avoid Pi4 RAM exhaustion

# Alternative: OpenVINS (lighter, no ROS required for non-ROS version)
git clone https://github.com/rpng/open_vins.git
```

**CPU budget on Pi4:** VINS-Mono uses ~60–100% of one core. Our detection pipeline
uses ~30–50% of one core when scanning. With 4 cores, there is headroom, but careful
thread management is needed.

---

## Accuracy vs our system

| Approach | 10-min drift (60 m field) | Notes |
|---|---|---|
| Current (EKF3 only) | 3–5 m | Baseline |
| + Optical flow | 0.5–1.5 m | 3× better |
| + Monocular VIO (VINS-Mono) | 0.3–0.8 m | 5–10× better |
| + Stereo VIO (ORB-SLAM3) | 0.1–0.3 m | 15–30× better |
| + Stereo VIO + loop closure | <0.1 m | Near-GPS accuracy |

For our 15 m wide field, 0.3 m drift means mine positions are reported within 0.3 m
of true — well within the thermal sensor's detection footprint (~0.5 m radius).

---

## The hard parts

### 1. Texture in the outdoor field
VIO needs **visual features** to track. Outdoor grass is generally textured enough,
but:
- Motion blur at higher drone speeds degrades features
- Overcast vs. bright sun changes apparent texture
- If the drone looks down (thermal camera) and forward (VIO camera) simultaneously,
  the two cameras must not interfere

### 2. Vibration and synchronisation
IMU readings and camera frames must be synchronised within ~1–2 ms. The Pi's USB
camera timestamp jitter can be problematic. Global shutter + hardware trigger is ideal.

### 3. Initialisation
VINS-Mono needs a few seconds of motion to initialise — it has to observe enough
parallax to estimate the scale. Our preflight calibration routine can include a
deliberate 3-second motion prior to the scan.

### 4. Computational headroom
If VIO + detection pipeline both max out the Pi during a re-hover detection check,
the VIO estimate may lag. Watchdog thread priorities must be set carefully.

---

## References

1. **Qin, T., Li, P., Shen, S. (2018).** *VINS-Mono: A Robust and Versatile Monocular
   Visual-Inertial State Estimator.* IEEE Transactions on Robotics.
   https://arxiv.org/abs/1708.03852

2. **Campos, C., et al. (2021).** *ORB-SLAM3: An Accurate Open-Source Library for
   Visual, Visual–Inertial, and Multimap SLAM.* IEEE TRO.
   https://arxiv.org/abs/2007.11898

3. **ArduPilot VIO integration (community):**
   https://discuss.ardupilot.org/t/a-small-vio-system-using-raspberry-pi-and-arducam-ov9281/103299

4. **Geneva, P., et al. (2020).** *OpenVINS: A Research Platform for Visual-Inertial
   Estimation.* ICRA 2020. https://github.com/rpng/open_vins

5. **NavCore-Pixhawk (VIO → EKF3 bridge reference):**
   https://github.com/ARYA-mgc/NavCore-Pixhawk

6. **Robust Visual SLAM for UAV Navigation (2025):**
   https://arxiv.org/html/2605.03678v1
