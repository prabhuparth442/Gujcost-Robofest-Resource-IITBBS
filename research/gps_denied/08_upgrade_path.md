# 08 — Concrete Upgrade Path

## What to actually build, in what order, with SpeedyBee F405 + ArduCopter

---

## Starting point — what we have right now

```
Hardware:
  SpeedyBee F405 Wing Mini (ArduCopter)
  Raspberry Pi 4 (4 GB)
  MLX90640 thermal sensor (mine detection)
  3S/4S LiPo, ~620 g all-up weight
  Pi Camera v2 (present, currently underused)

Software:
  ArduCopter with GUIDED mode
  pymavlink bridge (Pi ↔ SpeedyBee via UART6)
  MAVSDK for preflight checks
  Detection pipeline (thermal → mine reports)

Navigation accuracy: ~3–5 m drift over 10 minutes (dead reckoning only)
```

**Gap to mission-success:** Need <0.5 m mine position error.
**Gap size:** 6–10× improvement needed.

---

## Phase 1 — Optical Flow (1–2 days, ~₹2,200 per drone)

### What to buy

**Option A (budget):** PMW3901 optical flow breakout
- Matek 3901-L0X: has optical flow + integrated sonar in one unit, ~8 g, ~₹1,800
- Alternative: HereFlow (official ArduPilot), ~16 g, ~₹4,500

**Recommended:** Matek 3901-L0X — lightweight, has sonar for height assist, well-documented with ArduPilot.

### Physical installation

```
Mount under the drone, facing straight down.
Orientation: flow sensor's X axis pointing forward (same as drone nose).
Connection: UART on SpeedyBee F405

SpeedyBee F405 UART assignments (check your specific board):
  UART1: GPS (may already be used)
  UART2: available
  UART3: available
  UART6 (T6/R6 pads): Raspberry Pi companion
  
Connect Matek 3901-L0X to UART2 or UART3:
  TX (Matek) → RX pad
  RX (Matek) → TX pad
  GND → GND
  5V → 5V
```

### ArduCopter parameters

Open Mission Planner → Full Parameters or connect via MAVProxy:
```
FLOW_TYPE     = 10    # Matek 3901 (CXOF protocol)
FLOW_FXSCALER = 0     # Set after calibration (see below)
FLOW_FYSCALER = 0     # Set after calibration
EK3_SRC1_VELXY = 5   # Optical flow for XY velocity
EK3_SRC1_VELZ  = 0   # Barometer for Z velocity (keep)
EK3_SRC1_POSZ  = 1   # Barometer for altitude (keep)
RNGFND1_TYPE   = 10  # Matek sonar (if using 3901-L0X)
RNGFND1_MAX_CM = 400 # 4 m range
```

Via MAVProxy (from Raspberry Pi):
```bash
mavproxy.py --master=/dev/ttyAMA0 --baudrate=921600
param set FLOW_TYPE 10
param set EK3_SRC1_VELXY 5
param set RNGFND1_TYPE 10
param set RNGFND1_MAX_CM 400
```

### Calibration procedure (once per sensor)

1. Arm the drone and hover at ~1.5 m altitude on a textured surface (grass works)
2. Open Mission Planner → Setup → Optional Hardware → Optical Flow
3. Follow the calibration wizard: fly small circles/rectangles and it auto-tunes FLOW_FXSCALER and FLOW_FYSCALER
4. Land, check that EKF3 is accepting flow data (Mission Planner → Status → flowX/Y should be non-zero in hover)

**Validation test:**
- Hover at 2 m altitude for 5 minutes
- Land and measure displacement from takeoff point
- Should be <0.3 m instead of ~1.5 m (dead reckoning baseline)

### Expected improvement after Phase 1

| Metric | Before | After |
|---|---|---|
| 5-min position drift | ~1.5 m | ~0.2–0.4 m |
| 10-min position drift | ~3–5 m | ~0.4–0.8 m |
| Mine reporting accuracy | ±3–5 m | ±0.4–0.8 m |
| Competition mine score | ~20–30% | ~70–85% |

---

## Phase 1b — ArUco Markers (1 day, ~₹300 printing cost)

### Only if rules allow pre-placed markers on the field

**Materials:**
- Print 12 × ArUco markers (DICT_4X4_50, IDs 0–11) at 20 cm size on A3 paper
- Laminate (prevents wind flipping)
- Small tent pegs or cable stakes to hold flat on ground

### Camera calibration (one-time, 30 minutes)

```bash
# Run on Raspberry Pi
python3 scripts/calibrate_camera.py
# Point camera at a 9×6 checkerboard pattern, capture 20 frames from different angles
# Saves camera_matrix.npy and dist_coeffs.npy to scripts/
```

```python
# scripts/calibrate_camera.py
import cv2
import numpy as np

CHECKERBOARD = (9, 6)
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= 0.025  # 25 mm squares

objpoints, imgpoints = [], []
cap = cv2.VideoCapture(0)

while len(objpoints) < 20:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD)
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)
        print(f"Captured {len(objpoints)}/20")
    cv2.imshow('Calibration', frame)
    cv2.waitKey(100)

ret, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
np.save('scripts/camera_matrix.npy', mtx)
np.save('scripts/dist_coeffs.npy', dist)
print(f"Calibration complete. RMS reprojection error: {ret:.3f} px")
```

### Marker placement at competition

Before flight, measure and record each marker's position:
```python
# Update this dict at the competition field
MARKER_POSITIONS = {
    0:  (0.0,  0.0,  0.0),   # corner A
    1:  (15.0, 0.0,  0.0),   # corner B
    2:  (0.0,  60.0, 0.0),   # corner C
    3:  (15.0, 60.0, 0.0),   # corner D
    4:  (7.5,  0.0,  0.0),   # midpoint AB
    5:  (7.5,  60.0, 0.0),   # midpoint CD
    6:  (0.0,  20.0, 0.0),   # third point on A side
    7:  (15.0, 20.0, 0.0),   # third point on B side
    8:  (0.0,  40.0, 0.0),   # two-thirds on A side
    9:  (15.0, 40.0, 0.0),   # two-thirds on B side
    10: (7.5,  20.0, 0.0),   # centre grid
    11: (7.5,  40.0, 0.0),   # centre grid
}
```

Markers on the ground face upward; camera faces downward at 2–3 m altitude.
At 3 m altitude, 20 cm markers are detectable within ~2 m horizontal offset.

---

## Phase 2 — Monocular VIO (2–3 weeks, ~₹1,200)

### What to buy

- OV9281 global shutter camera (USB or CSI for Pi)
- Mount pointing forward and slightly downward (~20° below horizon)

**Or:** Use Pi Camera v2 (already have) as a temporary test. Rolling shutter will
cause some feature corruption during vibrations but is sufficient for proof-of-concept.

### Software setup

```bash
# On Raspberry Pi 4
# Install ROS2 Humble (one-time setup, 30 min)
sudo apt install ros-humble-ros-base

# Install OpenVINS (lighter than VINS-Mono, no Ceres dependency)
cd ~/ros2_ws/src
git clone https://github.com/rpng/open_vins.git
cd ~/ros2_ws && colcon build --packages-select ov_msckf --parallel-workers 2
```

**Configuration for our camera:**
```yaml
# config/drone_cam.yaml
camera_model: pinhole_radtan
distortion_coeffs: [k1, k2, p1, p2]  # from calibration
intrinsics: [fx, fy, cx, cy]          # from calibration
T_imu_cam:                             # camera-to-IMU transform (measure physically)
  - [0.0, -1.0, 0.0, 0.05]
  - [1.0,  0.0, 0.0, 0.0]
  - [0.0,  0.0, 1.0, -0.03]
  - [0.0,  0.0, 0.0, 1.0]
```

### Bridge to ArduPilot

```python
# scripts/vio_bridge.py — runs alongside existing detection pipeline
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from pymavlink import mavutil
import time

class VIOBridge(Node):
    def __init__(self):
        super().__init__('vio_bridge')
        self.mav = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600)
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/ov_msckf/poseimu',
            self.on_pose,
            10
        )
    
    def on_pose(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        # Convert quaternion to euler for MAVLink (simplified)
        import math
        yaw = math.atan2(2*(o.w*o.z + o.x*o.y), 1 - 2*(o.y**2 + o.z**2))
        
        self.mav.mav.vision_position_estimate_send(
            usec=int(time.time() * 1e6),
            x=float(p.x), y=float(p.y), z=float(p.z),
            roll=0.0, pitch=0.0, yaw=yaw,
            covariance=[0.04] * 21
        )

rclpy.init()
node = VIOBridge()
rclpy.spin(node)
```

Launch order:
```bash
# Terminal 1: Start VIO
ros2 launch ov_msckf subscribe.launch.py config:=drone_cam

# Terminal 2: Bridge to ArduPilot
python3 scripts/vio_bridge.py

# Terminal 3: Existing detection pipeline
python3 scripts/launch.py
```

### Expected improvement after Phase 2

| Metric | Phase 1 (flow) | Phase 2 (flow + VIO) |
|---|---|---|
| 10-min drift | 0.4–0.8 m | 0.1–0.4 m |
| Mine accuracy | ±0.5–0.8 m | ±0.2–0.4 m |
| Competition score | ~75% | ~90%+ |

---

## Phase 3 — UWB (if competition rules allow anchor placement)

### Buy list

- 4× Makerfabs DW3000 UWB anchors (~$20 each = ₹6,600 total)
- 3× DW3000 UWB tags for drones (~$15 each = ₹3,700 total)
- 4 tripods or tent stakes for anchor mounting

### Setup at competition

```python
# Before flight: measure anchor positions precisely with measuring tape
# Update anchors.json:
{
  "A1": [0.0, 0.0, 1.5],
  "A2": [15.0, 0.0, 1.5],
  "A3": [0.0, 60.0, 1.5],
  "A4": [15.0, 60.0, 1.5]
}
```

```bash
# Run on Pi during flight
python3 scripts/uwb_bridge.py --anchors scripts/anchors.json
```

This adds a third position source. ArduPilot EKF3 fuses all three (flow velocity,
VIO position, UWB position) using their respective covariance weights.

---

## Integration checklist

### Before every competition flight

- [ ] Check optical flow sensor is clean (no dust on lens)
- [ ] Verify EKF3 is accepting flow: `mavproxy.py` → `status EK3_SRC1` = flow
- [ ] Hover test: 60-second hover, drift <0.1 m (Phase 1 requirement)
- [ ] Marker positions updated in `marker_positions.py` if new field layout
- [ ] VIO initialisation: fly a slow 3-second figure-8 before starting scan
- [ ] UWB anchor check: all 4 anchors responding in `uwb_bridge.py --test`

### Parameter backup

Save current ArduCopter params:
```bash
mavproxy.py --master=/dev/ttyAMA0 --baudrate=921600
param save params/competition_params.parm
```

---

## Cost and weight summary

| Phase | Component | Weight added | Cost added |
|---|---|---|---|
| Phase 1 | Matek 3901-L0X (×3 drones) | +8 g/drone | ~₹5,400 |
| Phase 1b | ArUco markers (print) | 0 | ~₹300 |
| Phase 2 | OV9281 global shutter cam | +5 g/drone | ~₹3,500 |
| Phase 3 | DW3000 UWB setup | +13 g/drone + 4 anchors | ~₹10,300 |

**After Phase 1+2:** 633 g per drone (117 g under limit) ✅
**After Phase 1+2+3:** 646 g per drone (104 g under limit) ✅

---

## Timeline recommendation

```
Week 1: Buy and install Matek 3901-L0X sensors
        Calibrate optical flow
        Run hover drift test
        → Expected: mine accuracy 0.5–0.8 m

Week 2: Print ArUco markers + camera calibration script
        Test marker detection on existing Pi cam
        → If markers allowed: accuracy 0.2–0.4 m

Week 3–5: Install OpenVINS
           Build VIO bridge
           Test VIO initialisation and drift test
           → Expected: mine accuracy 0.2–0.3 m

Competition prep: Test complete integration (flow + markers + VIO)
                  Backup: ensure Phase 1 works standalone as fallback
```

---

## Final recommendation

**Ship Phase 1 (optical flow) immediately.** It is the highest ROI improvement
available and takes 1–2 days to implement. Everything else builds on top of it.

Do not skip to Phase 2 or 3 without Phase 1 working reliably — optical flow is the
foundation that the other approaches supplement.

The competition is winnable with Phase 1 + markers alone.
Phases 2 and 3 are margins of excellence, not minimum requirements.
