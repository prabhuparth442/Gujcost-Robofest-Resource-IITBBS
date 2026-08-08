# 05 — Sensor Hardware Reference

## Sensors on each slave drone

| Sensor | Interface | Purpose | Driver |
|--------|-----------|---------|--------|
| MLX90640 | I²C @ 0x33 | 32×24 thermal IR camera | `slave/src/mlx_stdout.cpp` (C++ subprocess) |
| TF-Luna | UART (serial) | 1D LIDAR obstacle detection | `tf_luna_failsafe.py` |
| RPi Camera / GPS | (via ArduPilot) | pymavlink/MAVSDK flight control | `pymavlink`, `mavsdk` Python libraries |

---

## MLX90640 — 32×24 Thermal IR Sensor

**Datasheet:** https://www.melexis.com/en/documents/documentation/datasheets/datasheet-mlx90640  
**Library (C++):** https://github.com/pimoroni/mlx90640-library  
**Our driver:** `slave/src/mlx_stdout.cpp`

### Key specs

| Parameter | Value |
|-----------|-------|
| Resolution | 32 × 24 pixels (768 total) |
| Field of View | 55° × 35° (with our lens) |
| Temperature range | -40°C to +300°C |
| Accuracy | ±1°C (absolute), but relative ΔT resolution ~0.1°C |
| Refresh rate (our setting) | 2 Hz |
| Interface | I²C at up to 1 MHz |
| I²C address | 0x33 |
| Supply voltage | 3.3 V |

### Why 2 Hz refresh rate

The drone moves at ≤0.3 m/s with 0.5 m step dwell time at each waypoint. At 2 Hz we get
~4 frames per dwell position — enough for the temporal filter but not so fast that we
overwhelm the I²C bus.

Higher rates (4 Hz, 8 Hz) are available but produce noisier data at low light levels.

### Ground footprint at 1.5 m altitude

```
FOV 55° horizontal → width  = 2 × 1.5 × tan(27.5°) = 1.56 m
FOV 35° vertical   → height = 2 × 1.5 × tan(17.5°) = 0.95 m

At 32 px wide: pixel pitch = 1.56 / 32 = 0.049 m ≈ 4.9 cm/pixel
```

This matches `FOOTPRINT_RADIUS_M = 0.78 m` in `grid_map.py` (half of 1.56 m).

### How our driver works

The C++ binary (`mlx_stdout.cpp`) runs as a subprocess of Python:

```python
proc = subprocess.Popen(["sudo", "./bin/mlx_stdout"], stdout=subprocess.PIPE)
raw = proc.stdout.read(768 * 4)   # 3072 bytes = one frame
frame = np.frombuffer(raw, dtype=np.float32).reshape((24, 32))
```

Debug messages (I²C init, EEPROM dump, etc.) go to **stderr** — do not redirect stderr
to the Python pipe or the float stream will be corrupted.

### Common failure modes

| Symptom | Cause | Fix |
|---------|-------|-----|
| `EEPROM read failed: -1` | I²C wiring or permissions | Check I²C enabled in raspi-config; run with sudo |
| All pixels read 0.0 | Process started without sudo | Restart with `sudo ./bin/mlx_stdout` |
| Frozen frame (same data repeating) | I²C bus lockup | Power cycle sensor; add 100Ω pull-up to SDA/SCL |
| Striping pattern visible | FPN not calibrated | Run `00_preflight_calib.py` again |
| `GetFrameData timeout: -3` | I²C bus busy / EMI | Shorten I²C cable; add 10nF bypass caps |

### Wiring (Raspberry Pi 4)

| MLX90640 pin | RPi pin | BCM | Note |
|-------------|---------|-----|------|
| VCC | Pin 1 | 3.3V | Do NOT use 5V |
| GND | Pin 6 | GND | |
| SDA | Pin 3 | GPIO2 | 4.7kΩ pull-up to 3.3V |
| SCL | Pin 5 | GPIO3 | 4.7kΩ pull-up to 3.3V |

Enable I²C in raspi-config → Interface Options → I2C.

### Building the C++ driver

```bash
# 1. Install BCM2835
cd drone_swarm_folder/drone_swarm/bcm2835-1.71
./configure && make && sudo make install

# 2. Build MLX90640 library
cd drone_swarm_folder/drone_swarm/lib/mlx90640-library
make

# 3. Build our binary
cd slave/src
bash build.sh
# Output: slave/bin/mlx_stdout
```

---

## TF-Luna — Single-Point LIDAR

**Datasheet:** https://en.benewake.com/TFLuna  
**Our driver:** `slave/tf_luna_failsafe.py`

### Key specs

| Parameter | Value |
|-----------|-------|
| Range | 0.2 – 8 m |
| Accuracy | ±2 cm |
| Frame rate | up to 250 Hz (we use 10 Hz) |
| Interface | UART (default 115200 baud) or I²C |
| Supply | 5V, ~80 mA |
| Size | 35 × 21.5 × 13.5 mm, 5g |

### How our failsafe works

`tf_luna_failsafe.py` runs as an independent asyncio task alongside the main flight loop:

```python
async def tf_luna_monitor(drone, serial_port="/dev/ttyAMA0"):
    consecutive = 0
    while True:
        dist = read_tf_luna(serial_port)
        if dist < 1.0:
            consecutive += 1
            if consecutive >= 3:
                # Emergency: sidestep 1 m East
                await emergency_sidestep(drone)
                consecutive = 0
        else:
            consecutive = 0
        await asyncio.sleep(0.1)  # 10 Hz polling
```

Three consecutive readings below 1.0 m → emergency sidestep. This prevents false triggers
from momentary EMI or sensor noise.

### Wiring

| TF-Luna pin | RPi pin | Note |
|-------------|---------|------|
| 5V | Pin 4 | |
| GND | Pin 14 | |
| TX | GPIO15 (RXD, Pin 10) | RPi receives here |
| RX | GPIO14 (TXD, Pin 8) | RPi sends here |

Enable UART in raspi-config → Interface Options → Serial Port  
Disable serial console (`serial console = no`, `serial hardware = yes`).

---

## BCM2835 C Library

**URL:** http://www.airspayce.com/mikem/bcm2835/

Low-level C library for direct GPIO, SPI, I²C hardware access on Raspberry Pi.
Used by the MLX90640 C++ driver for I²C communication.

Version in repo: `bcm2835-1.71` (in `drone_swarm_folder/drone_swarm/bcm2835-1.71/`)

**Do not update BCM2835** without re-testing the MLX90640 build — the library API is
stable but the build flags change between major versions.

---

## Camera Intrinsics — for pixel-to-metre projection

The `04_coordinate_math.py` module converts a thermal pixel offset to a physical ground
offset using similar triangles (no full camera calibration matrix — the MLX90640 has
no lens distortion worth correcting at 32×24 resolution).

```python
# From 04_coordinate_math.py
ALTITUDE_M = 1.5  # nominal flight altitude above ground

# Focal lengths derived from FOV
fx = 614.5  # pixels per metre at 1 m distance  (55° hFOV, 640 px virtual)
fy = 761.2  # pixels per metre at 1 m distance  (35° vFOV, 480 px virtual)

# Convert pixel offset to body-frame metres
body_x = (pixel_dx / fx) * ALTITUDE_M   # forward/back
body_y = (pixel_dy / fy) * ALTITUDE_M   # left/right

# Then rotate by drone heading to get NED, then add to GPS origin
```

The 640×480 "virtual" size is because the MLX90640 frame (32×24) is upscaled ×20 for
OpenCV processing. Intrinsics scale proportionally — the same fx/fy apply.

---

## Alternative Sensors Worth Evaluating for 6.0

### FLIR Lepton 3.5

| Spec | MLX90640 | Lepton 3.5 |
|------|----------|-----------|
| Resolution | 32×24 | 160×120 |
| FOV | 55°×35° | 57°×45° |
| Interface | I²C | SPI |
| Frame rate | 2–64 Hz | 8.7 Hz |
| Price | ~₹3,000 | ~₹15,000 |
| Library | C++ (Melexis) | Python-lepton |

Higher resolution = better localisation accuracy for buried mines. The pixel-to-metre
calculation becomes 4× more precise. Downside: 5× more expensive.

### Boson+ 320

Resolution 320×256 at 60 Hz — significantly better but ~₹80,000+. Overkill for Robofest
budget constraints but the reference standard in professional mine-detection research.
