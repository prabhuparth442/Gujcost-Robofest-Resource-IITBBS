# 06 — LiDAR SLAM

## 3D mapping with laser ranging — the most accurate, heaviest option

---

## The layman version

A LiDAR (Light Detection and Ranging) sensor spins a laser beam in all directions and
measures the time each pulse takes to bounce back off surfaces. In one rotation (~100 ms),
it builds a point cloud — a 3D map of everything around it, accurate to a few centimetres.

SLAM with LiDAR uses these point clouds to both **build a map** of the environment and
**localise** within that map simultaneously. Think of it as GPS built entirely from geometry:
"I can see these walls/trees/features at these exact angles and distances — therefore I
know exactly where I am."

---

## Types of LiDAR for drones

### 3D spinning LiDAR (highest accuracy, heaviest)

```
Velodyne VLP-16 ("Puck"):
- 16 laser beams, 360° × ±15° vertical FOV
- 100 m range, ±3 cm accuracy
- Weight: 830 g ❌ too heavy for our 750 g total weight limit
- Cost: ~$4,000 ❌

Livox Mid-360:
- Solid state, non-repetitive scan pattern
- 40 m range, ±2 cm accuracy
- Weight: 265 g ❌ still too heavy
- Cost: ~$1,000 ❌
```

### 2D spinning LiDAR (lighter, 2D map only)

```
RPLIDAR A1:
- 360° horizontal scan, 6 m range
- Weight: 170 g — marginal (drones lose 170g of payload)
- Cost: ~$100
- Resolution: 1° angular

YDLIDAR X4:
- 360° horizontal scan, 10 m range
- Weight: 190 g ❌
- Cost: ~$80
```

**Problem with 2D LiDAR for drones:** Only scans a horizontal plane. A drone hovering
at 3 m altitude scanning horizontally sees a 2D ring at 3 m height — misses ground
features, only useful if scanning at the same height as walls/obstacles.

### Solid-state LiDAR (lightweight, limited FOV)

```
Benewake TF-Luna (ToF, forward-facing):
- 8 m range, ±6 cm
- Weight: 5 g ✅
- Cost: ~$20
- Only single-point — useful for obstacle avoidance, not SLAM

Benewake TF Mini Plus:
- 12 m range, ±6 cm
- Weight: 35 g ✅
- Cost: ~$35
- Single-point — same limitation

Livox HAP (automotive, expensive):
- 100 m range, ±2 cm, 3D
- Weight: 695 g ❌
```

**For our weight budget (<130 g remaining after optical flow), no 3D LiDAR fits.**

---

## SLAM algorithms for LiDAR

Even if we accept the weight penalty, here's how LiDAR SLAM works:

### FAST-LIO2 (current state-of-the-art, 2022)

**"Fast LiDAR-Inertial Odometry"** — fuses LiDAR point clouds with IMU in a tight
coupling.

Key innovation: iterative Kalman filter that incrementally updates a k-d tree map
instead of rebuilding it each frame. Result: runs at LiDAR scan rate (10 Hz) even on
ARM processors.

```
LiDAR scan (point cloud)
         ↓
Preprocess (downsample to 1/4 points)
         ↓
IMU pre-integration (400 Hz between scans)
         ↓
Iterative Extended Kalman Filter:
  - State: position, velocity, orientation, IMU bias
  - Observation: closest point in existing map to each new scan point
  - Update: rotate + translate new scan to minimise point-to-plane distance
         ↓
Incremental k-d tree map update
         ↓
Pose estimate: X, Y, Z, roll, pitch, yaw at 10 Hz
```

**Accuracy:** <0.1 m position error in typical indoor/outdoor environments.
**CPU:** ~150% of one ARM Cortex-A72 core (Pi4 can run it, barely).
**RAM:** ~800 MB for a 100 m² map.

**Problem for us:** Requires a 3D LiDAR. With only ToF single-point sensors, can't run FAST-LIO2.

### Hector SLAM (2D, lightweight)

Designed for 2D LiDAR without IMU. Uses scan-matching to find the transformation
between consecutive laser scans.

```
Scan N-1  →  Scan N
Find transformation T such that scan N best overlaps scan N-1
Accumulate T over time → position estimate
```

**Accuracy:** 0.1–0.5 m in corridors, worse in open fields (few features to match).
**CPU:** ~30–50% of one core.
**Weight penalty:** Requires RPLIDAR A1 (~170 g) — exceeds our budget.

### Cartographer (Google's 2D/3D SLAM)

- Very accurate, with loop closure
- Requires 2D or 3D LiDAR
- CPU-heavy: ~200% of two cores for 3D, ~100% for 2D
- Memory: ~2 GB for large maps
- **Not practical on Pi4 in real-time**

---

## The lightweight alternative: ToF altitude + SLAM-lite

For our 750 g weight budget, the practical LiDAR contribution is:

### Downward-facing ToF (already useful)

```
VL53L1X ToF sensor (I2C):
- Range: 4 m, ±3 cm accuracy
- Weight: 1 g ✅
- Cost: ~$5
- Already built into SpeedyBee F405 or easily added
```

ArduPilot parameter: `RNGFND1_TYPE = 2` (I2C rangefinder)

This gives precise altitude — removes barometric error (~0.5 m) and helps EKF3 hold
exact height during the scan. NOT a SLAM system, but a useful sensor addition.

### Side-facing single-beam ToF for wall following

Place one ToF forward + one ToF side on each drone. Use wall distance to constrain
one axis of position:

```python
# If the drone is flying along the left wall at 2 m:
# side_tof_distance = measured distance to wall (should be 2 m)
# If it drifts → correct yaw to bring side distance back to 2 m

error = side_tof_reading - target_wall_distance
yaw_correction = Kp * error  # simple proportional control
```

This is **not SLAM**, but it's a 5 g, $10 addition that constrains one axis of drift.

---

## Weight and feasibility summary

| LiDAR option | Weight | Cost | Accuracy | Feasible for us? |
|---|---|---|---|---|
| Velodyne VLP-16 | 830 g | $4,000 | ±2 cm, 3D | ❌ Over weight limit |
| Livox Mid-360 | 265 g | $1,000 | ±2 cm, 3D | ❌ Over budget weight |
| RPLIDAR A1 (2D) | 170 g | $100 | ±2 cm, 2D | ❌ Marginal, 2D only |
| VL53L1X ToF | 1 g | $5 | ±3 cm, 1D | ✅ altitude only |
| Benewake TF Mini | 35 g | $35 | ±6 cm, 1D | ✅ 1D ranging |

**Conclusion for Robofest:** Full LiDAR SLAM is not feasible under the 750 g constraint.
Use ToF for altitude precision only. If the weight budget allows after optical flow
(~13 g remaining ~117 g headroom), a single RPLIDAR A1 *could* be added for 2D
positioning — but only if indoor or near-wall flight.

---

## What LiDAR SLAM looks like on a real competition robot

For completeness, here's a full stack that winning teams with bigger drones use:

```
Hardware:
  Livox Mid-360 (3D LiDAR, 265 g)
  + Pixhawk 6C or SpeedyBee F405 (FC)
  + Jetson Nano or Xavier NX (companion computer — much more powerful than Pi)

Software:
  FAST-LIO2 (point cloud → pose at 10 Hz)
  → ROS2 /tf topic → ArduPilot via AP_DDS
  → EK3_SRC1_POSXY=6 (ExternalNav)

Result:
  <5 cm position error across a 50 m field, even with loops
```

For Robofest Aerial Robotics specifically, this is what a "top-tier" team runs.
We can achieve 80% of that accuracy with optical flow + VIO for 1/10th the weight.

---

## References

1. **Xu, W., et al. (2022).** *FAST-LIO2: Fast Direct LiDAR-Inertial Odometry.*
   IEEE Transactions on Robotics. https://arxiv.org/abs/2107.06829

2. **Kohlbrecher, S., et al. (2011).** *A Flexible and Scalable SLAM System with
   Full 3D Motion Estimation.* SSRR 2011. (Hector SLAM paper)

3. **Hess, W., et al. (2016).** *Real-Time Loop Closure in 2D LIDAR SLAM.*
   ICRA 2016. (Cartographer paper)

4. **RPLIDAR A1 documentation:**
   https://www.slamtec.com/en/lidar/A1

5. **VL53L1X with ArduPilot:**
   https://ardupilot.org/copter/docs/common-rangefinder-vl53l0x.html
