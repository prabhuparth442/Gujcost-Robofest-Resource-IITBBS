# 05 — Marker-Based Absolute Position References

## Visual landmarks that tell the drone exactly where it is

---

## The layman version

Imagine you're navigating a building with no GPS. If someone puts a sign at the door
saying "You are at Room 204", you instantly know exactly where you are — no estimation,
no drift. The sign is an **absolute position reference**.

ArUco and AprilTag are the robot equivalent: printed patterns (like QR codes but
designed for pose estimation) placed at known locations. When a drone's camera sees one,
it can compute its own position in 3D space to within a few centimetres — resetting any
accumulated drift in one camera frame.

---

## ArUco vs AprilTag — which one?

Both are widely used fiducial marker systems:

| Property | ArUco | AprilTag |
|---|---|---|
| Detection speed | Faster | Slightly slower |
| Detection range | Good | Better at long range |
| False positive rate | ~1% | <0.1% |
| ROS2 support | `ros2-aruco` package | `apriltag_ros` package |
| OpenCV built-in | ✅ Yes (`cv2.aruco`) | ❌ External lib needed |
| Recommended for UAV | ✅ Standard choice | ✅ Preferred for far detection |

**For Robofest:** ArUco is simpler (built into OpenCV which we already use), while
AprilTag is better at detecting markers from further away. Either works. We'll use
ArUco below as it requires no extra dependencies.

---

## How pose estimation works

### Marker geometry

An ArUco marker is a square black-and-white pattern printed at a **known physical size**
(e.g. 20 cm × 20 cm). The camera image of this marker tells us:

1. Where the 4 corners appear in the image (pixel coordinates)
2. We know where those 4 corners are in the real world (20 cm × 20 cm)
3. OpenCV `solvePnP` finds the rotation and translation that maps 3D → 2D

```python
import cv2
import numpy as np

# Camera calibration (from calibration step)
camera_matrix = np.array([[fx, 0, cx],
                           [0, fy, cy],
                           [0,  0,  1]])
dist_coeffs = np.array([k1, k2, p1, p2])

# Detect ArUco markers
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, params)

corners, ids, _ = detector.detectMarkers(frame)

if ids is not None:
    marker_size = 0.20  # 20 cm marker
    for i, marker_id in enumerate(ids.flatten()):
        # Estimate pose
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners[i:i+1], marker_size, camera_matrix, dist_coeffs)
        
        # tvec = [x, y, z] in metres relative to marker centre
        x_cam, y_cam, z_cam = tvec[0][0]
        
        # Drone position = marker_position - camera_offset_from_drone - tvec_rotated
        # (See full transform below)
        drone_x = MARKER_POSITIONS[marker_id][0] - x_cam
        drone_y = MARKER_POSITIONS[marker_id][1] - y_cam
        drone_z = MARKER_POSITIONS[marker_id][2] + z_cam  # z is altitude
```

### Coordinate transform chain

```
Marker frame → Camera frame → Drone body frame → World (NED) frame

tvec from solvePnP gives: camera position relative to marker
We want: drone position relative to world origin

Steps:
1. tvec_world = R_marker_to_world × tvec
2. drone_pos = marker_pos_world - R_drone_to_world × camera_offset_from_drone
                                 - tvec_world
```

This transform is ~20 lines of numpy. Many ROS2 packages (aruco_ros2) handle it
automatically if you configure the marker positions in a YAML file.

---

## Marker placement strategy for Robofest

### Option 1: Boundary markers (field perimeter)

Place ArUco markers along the field boundary at known positions:

```
Field (15 m × 60 m, top view):

│M1│ ─────── M2 ─────── │M3│
│                          │
│    (drones scan here)    │
│                          │
│M4│ ─────── M5 ─────── │M6│

M1: (0, 0)
M3: (15, 0)
M4: (0, 60)
M6: (15, 60)
M2, M5: midpoints at (7.5, 0) and (7.5, 60)
```

**Detection range for 20 cm markers:** ~3–5 m from a downward-facing camera at 3 m altitude,
or ~8–12 m from a forward-facing camera.

**Coverage:** If the drone stays within 3 m of a wall, it can always see a boundary
marker. Fine for a sweep pattern but leaves the centre of the 15 m field uncovered.

### Option 2: Grid markers (fixed positions throughout field)

Place markers every 5 m across the field:

```
Field (15 m × 60 m):
Every cell = 5 m × 5 m → 3 × 12 = 36 marker positions

For Robofest: reduce to corners only + every 10 m centres
→ ~12 markers total on the ground, facing upward
```

Downward-facing camera at 3 m altitude can see a 20 cm marker up to ~2 m horizontal
offset from directly below. So the drone needs to fly within 2 m of each marker.

**For a systematic sweep path, this is achievable.**

### Option 3: Checkpoint markers + dead reckoning between

The practical hybrid approach:
1. Place markers at 6–8 positions (start zone, corners, midpoints)
2. Drone uses dead reckoning between markers
3. When a marker is detected, position is corrected to exact known value

This matches how real survey robots work. Error resets to ~5 cm at every marker,
then drifts until the next one.

---

## Accuracy

| Configuration | Accuracy at marker | Max drift between |
|---|---|---|
| 20 cm marker at 2 m | ±3–8 cm | Dead reckoning drift |
| 20 cm marker at 3 m | ±5–12 cm | Between markers: 0.3–0.5 m/min |
| 50 cm marker at 5 m | ±8–20 cm | Long-range detection |
| Landing pad (50 cm) | ±3–5 cm | Used for precision landing |

Precision landing accuracy for our case: **0.91 m** (reported in ArUco landing studies
with unoptimised code). With careful camera calibration, <0.3 m is achievable.

---

## Integration with ArduPilot

Two approaches:

### Approach A: VISION_POSITION_ESTIMATE (same as VIO)

When a marker is detected:
```python
# Compute drone position from marker detection
drone_x, drone_y, drone_z = compute_drone_pose(corners, ids)

# Send to ArduPilot EKF3 with high confidence (covariance = 0.01 → σ = 10 cm)
mav.mav.vision_position_estimate_send(
    usec=int(time.time() * 1e6),
    x=drone_x, y=drone_y, z=drone_z,
    roll=0, pitch=0, yaw=current_yaw,
    covariance=[0.01] * 21  # High confidence — we just saw a marker
)
```

EKF3 treats this as a very confident position fix and snaps its estimate.

### Approach B: MAVLink LANDING_TARGET (for precision landing)

ArduPilot natively supports precision landing via:
```
PLND_ENABLED = 1
PLND_TYPE    = 1   # MAVLink LANDING_TARGET message
```

```python
mav.mav.landing_target_send(
    time_usec=int(time.time() * 1e6),
    target_num=0,
    frame=8,             # MAV_FRAME_BODY_NED
    angle_x=marker_x_angle,  # angle to target in radians
    angle_y=marker_y_angle,
    distance=marker_distance,
    size_x=0.20,
    size_y=0.20
)
```

This activates ArduPilot's built-in precision landing controller.

---

## Practical implementation on our Pi

We already have a Pi camera. Marker detection adds minimal overhead:

```
Current camera use: thermal (MLX90640, I2C — no camera frames used for thermal)
VGA camera: currently used for optional detection
```

**Running ArUco detection in a thread:**
```python
import cv2
import threading
import time

class MarkerDetector(threading.Thread):
    def __init__(self, mav_conn):
        super().__init__(daemon=True)
        self.mav = mav_conn
        self.cap = cv2.VideoCapture(0)  # Pi camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
            cv2.aruco.DetectorParameters()
        )
    
    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue
            corners, ids, _ = self.detector.detectMarkers(frame)
            if ids is not None:
                self.process_detection(corners, ids)
            time.sleep(0.05)  # 20 Hz marker detection
    
    def process_detection(self, corners, ids):
        for i, mid in enumerate(ids.flatten()):
            if mid not in MARKER_POSITIONS:
                continue
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i:i+1], 0.20,
                CAMERA_MATRIX, DIST_COEFFS
            )
            pos = self.compute_drone_position(mid, tvec[0][0])
            self.send_vision_estimate(*pos)
```

**CPU cost:** ~15–25% of one Pi4 core at 20 Hz with 640×480 images.
Combined with our thermal processing: ~40–60% total — well within 4-core budget.

---

## Setup required at competition

1. **Print markers** (ArUco DICT_4X4_50, IDs 0–11) on A3 or A4 paper, laminate.
   20 cm markers require A3 print.
2. **Measure and record** exact positions of each marker when placed on field.
3. **Camera calibration** — 20-frame checkerboard calibration, takes ~5 minutes.
   Run `python scripts/calibrate_camera.py` (we need to write this utility).
4. **MARKER_POSITIONS dict** — update with field measurements before flight.

This adds ~15 minutes of pre-flight prep, feasible within the competition setup window.

---

## References

1. **ArUco markers in OpenCV:**
   https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html

2. **AprilTag library (Python bindings):**
   https://github.com/duckietown/lib-dt-apriltags

3. **ArduPilot precision landing with ArUco:**
   https://ardupilot.org/copter/docs/precision-landing-with-irlock.html

4. **Navarro-Galvez et al. (2023).** *ArUco marker accuracy for UAV positioning:*
   field measurements show 0.91 m landing accuracy with default parameters.

5. **Garrido-Jurado, S., et al. (2014).** *Automatic generation and detection of highly
   reliable fiducial markers under occlusion.* Pattern Recognition.
