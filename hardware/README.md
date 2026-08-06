# Hardware Libraries

Libraries needed to build and interface the physical sensors on the Raspberry Pi.

---

## 1. MLX90640 — 32×24 Thermal Camera

**What it does:** The primary mine-detection sensor. Streams a 32×24 array of temperature readings at up to 16 Hz. Each pixel is a temperature in °C. At 1.5 m altitude with the 55°×35° lens, each pixel covers ~2.8 cm × 2.8 cm of ground.

**Library source:** `../drone_swarm_folder/drone_swarm/lib/mlx90640-library/`  
**Reference PDF:** `../drone_swarm_folder/drone_swarm/lib/mlx90640-library/MLX90640 driver.pdf`

### Build the C++ library

```bash
cd ../drone_swarm_folder/drone_swarm/lib/mlx90640-library/
make
# Produces: libMLX90640_API.so and libMLX90640_API.a
```

### Build the mlx_stdout binary (used by slave)

The slave's `bin/mlx_stdout` binary reads from the MLX90640 over I²C and streams raw `float32` frames to stdout. Python reads from this pipe:

```bash
# Enable I²C on Raspberry Pi:
sudo raspi-config  # → Interface Options → I2C → Enable

# Check sensor is detected (address 0x33):
sudo i2cdetect -y 1

# Build (source in drone_swarm_folder or your own):
g++ -o bin/mlx_stdout mlx_stdout.cpp \
    -I../drone_swarm_folder/drone_swarm/lib/mlx90640-library/headers \
    -L../drone_swarm_folder/drone_swarm/lib/mlx90640-library \
    -lMLX90640_API -lbcm2835
```

### Lens options

The MLX90640 comes with two lens options:
- **55°×35°** (wider) — used in this project; better for low-altitude scanning
- **110°×75°** (ultra-wide) — useful at higher altitudes but less resolution per cell

The focal length constants in `04_coordinate_math.py` assume the 55°×35° lens:
```python
fx = 320 / tan(27.5°) = 614.5   # horizontal
fy = 240 / tan(17.5°) = 761.2   # vertical
```
If you switch to the 110° lens, update both constants.

### Frame format

The C++ binary outputs raw `float32` at 24×32 = 768 floats per frame:
```python
FRAME_BYTES = 768 * 4  # 3072 bytes per frame
raw = proc.stdout.read(FRAME_BYTES)
frame = np.frombuffer(raw, dtype=np.float32).reshape((24, 32))
```

---

## 2. BCM2835 — Raspberry Pi GPIO/SPI/I²C Library

**What it does:** Low-level C library for direct hardware access on Raspberry Pi. Used internally by the MLX90640 C++ driver for I²C communication. You don't call this directly from Python.

**Source:** `../drone_swarm_folder/drone_swarm/bcm2835-1.71/`  
**Tarball:** `../drone_swarm_folder/drone_swarm/bcm2835-1.71.tar.gz`

### Build

```bash
cd ../drone_swarm_folder/drone_swarm/bcm2835-1.71/
./configure
make
sudo make install
# Installs to /usr/local/lib/libbcm2835.a
```

### Note

BCM2835 requires running as root (`sudo`) for direct hardware access, which is why the slave orchestrator is started with `sudo python3 main_orchestrator_competition.py` or the binary is setuid.

---

## 3. TF-Luna — I²C / UART LIDAR

**What it does:** Short-range LIDAR used for obstacle detection (the `tf_luna_failsafe.py` module). Returns distance in cm at up to 250 Hz. Threshold: 1.0 m — if anything is closer than this for 3 consecutive frames, the drone sidestepped immediately.

**Interface:** I²C (default address 0x10) or UART (115200 baud)

**Python library:** `pyserial` (already in `requirements.txt`)

**Port:** Set via `LUNA_PORT` environment variable (default: `/dev/serial0`)

```bash
# UART mode (recommended for RPi):
export LUNA_PORT=/dev/serial0
# or
export LUNA_PORT=/dev/ttyUSB0
```

---

## 4. Flight Controller + MAVSDK

**What it does:** The flight controller (FC) runs PX4 or ArduPilot and handles low-level stabilisation. MAVSDK is the Python library that lets the Pi send commands (arm, takeoff, goto, land) and receive telemetry (GPS, heading, altitude) from the FC.

**Connection string:** `udp://:14540` (via MAVProxy bridge)

```bash
# MAVProxy bridge (run in a separate terminal before starting the slave):
# USB connection:
mavproxy.py --master=/dev/ttyACM0 --out=udp:127.0.0.1:14540
# UART connection (GPIO pins 14/15):
mavproxy.py --master=/dev/serial0,57600 --out=udp:127.0.0.1:14540
```

**Install MAVSDK:**
```bash
pip3 install mavsdk
```

**Recommended FC settings (PX4):**
- `MPC_XY_VEL_MAX`: 1.0 m/s (slow, controlled scan)
- `MPC_Z_VEL_MAX_UP`: 0.5 m/s (gentle takeoff)
- `COM_RC_OVERRIDE`: 0 (disable RC override so MAVSDK has full control)
- Emergency kill switch wired to dedicated RC channel

---

## Wiring summary (Raspberry Pi 4)

| Sensor | Interface | Pi Pins |
|--------|-----------|---------|
| MLX90640 | I²C | SDA=GPIO2 (pin 3), SCL=GPIO3 (pin 5), 3.3V, GND |
| TF-Luna | UART | TX=GPIO14 (pin 8), RX=GPIO15 (pin 10), 5V, GND |
| Flight controller | UART | via USB or GPIO14/15 (via MAVProxy) |

> Enable I²C with `sudo raspi-config` → Interface Options → I2C → Enable  
> Check I²C bus: `sudo i2cdetect -y 1` (MLX90640 should appear at 0x33, TF-Luna at 0x10)
