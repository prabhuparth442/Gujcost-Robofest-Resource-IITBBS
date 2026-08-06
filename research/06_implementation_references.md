# 06 — Implementation References

Quick-reference for every external tool, SDK, and library the system depends on.
Use this when setting up a new drone from scratch or when something breaks.

---

## MAVSDK-Python

**Docs:** https://mavsdk.mavlink.io/main/en/python/  
**GitHub:** https://github.com/mavlink/MAVSDK-Python  
**Install:** `pip install mavsdk`  
**Minimum version:** `mavsdk>=1.4.0` (see `slave/requirements.txt`)

### What we use

| API | Where in code | Purpose |
|-----|--------------|---------|
| `drone.action.arm()` | `main_orchestrator_competition.py` | Arm motors |
| `drone.action.takeoff()` | same | Altitude hold takeoff |
| `drone.action.land()` | same | Controlled landing |
| `drone.offboard.set_position_global()` / `goto_location()` | same | Fly to GPS coordinate |
| `drone.telemetry.position()` | `udp_channel.py` | Subscribe to GPS position stream |
| `drone.telemetry.heading()` | `00_preflight_calib.py` | Get compass heading |
| `drone.telemetry.in_air()` | `main_orchestrator_competition.py` | Check airborne status |
| `drone.telemetry.battery()` | telemetry | Battery voltage monitoring |

### Offboard mode

MAVSDK requires the drone to be in **offboard mode** before `goto_location()` works.
Our code enables this automatically — but if PX4 rejects offboard mode, check:
1. RC transmitter is off (PX4 requires RC off or RC override for offboard)
2. SITL is running (for testing) or real flight controller connected
3. MAVProxy bridge is active (port 14540)

### Connecting to PX4

```bash
# On the drone (replaces direct USB with UDP bridge)
mavproxy.py --master=/dev/ttyAMA0,57600 --out=udp:127.0.0.1:14540
```

MAVSDK connects to `udp://:14540` — this is the default in our code.

### Common errors

| Error | Fix |
|-------|-----|
| `ConnectionError: no heartbeat received` | MAVProxy not running, or wrong port |
| `ActionError: command denied` | Drone not in correct flight mode; check RC override |
| `OffboardError: rejected` | Send at least 1 setpoint before enabling offboard mode |
| `TelemetryError: no GPS fix` | Wait for GPS lock; do not arm indoors without SITL |

---

## PX4 Autopilot — Offboard Control

**Official docs:** https://docs.px4.io/main/en/flight_modes/offboard.html  
**Parameters reference:** https://docs.px4.io/main/en/advanced_config/parameter_reference.html

### Offboard mode rules

- PX4 must receive setpoints at >2 Hz or it will exit offboard and fall back to hold mode
- Our code sends waypoints sequentially — if `goto_location()` awaits completion (>500 ms
  between waypoints), the drone may exit offboard mid-flight
- **Fix:** Call `drone.offboard.set_velocity_body()` as a keep-alive if waypoint dwell is long

### Key PX4 parameters to check before flight

| Parameter | Recommended | Purpose |
|-----------|-------------|---------|
| `COM_RC_LOSS_T` | 5.0 s | RC loss timeout before failsafe |
| `EKF2_AID_MASK` | 1 (GPS) | Which sensors EKF fuses |
| `MPC_XY_VEL_MAX` | 3.0 m/s | Max horizontal velocity in offboard |
| `MPC_LAND_SPEED` | 0.5 m/s | Descent rate during landing |
| `RTL_RETURN_ALT` | 3.0 m | Return-to-launch altitude (above home) |

---

## MAVProxy

**Docs:** https://ardupilot.org/mavproxy/  
**Install:** `pip install MAVProxy`

MAVProxy acts as a serial-to-UDP bridge between the Pixhawk serial port and MAVSDK.

```bash
# Standard launch command (on RPi drone):
mavproxy.py --master=/dev/ttyAMA0,57600 \
            --out=udp:127.0.0.1:14540 \
            --out=udp:<ground_station_ip>:14550
```

The second `--out` sends MAVLink telemetry to a ground station (QGroundControl or Mission Planner)
for monitoring — remove this for competition flights to avoid broadcast traffic.

---

## Vosk — Offline Speech Recognition

**Website:** https://alphacephei.com/vosk/  
**GitHub:** https://github.com/alphacep/vosk-api  
**Install:** `pip install vosk`  
**Model download:** https://alphacephei.com/vosk/models

### What we use

Voice commands processed by the master drone's Flask server:

| Voice command | Action triggered |
|--------------|-----------------|
| "start mission" | Begin scan sequence |
| "pause" | PAUSE TCP to all slaves |
| "resume" | RESUME TCP to all slaves |
| "land all" | LAND TCP to all slaves |
| "abort" | Emergency land all |

### Setup on master Pi

```bash
# Download model (~50 MB, English small)
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip -d master/vosk_model/

# Confirm path in app.py:
MODEL_PATH = "vosk_model/vosk-model-small-en-us-0.15"
```

### How it works in our system

1. Phone browser (on WiFi) opens the Flask HTTPS page
2. Browser uses `getUserMedia()` to capture mic audio
3. Audio streamed as 16-bit mono PCM to `/api/audio_chunk` via POST
4. Master Pi runs Vosk recogniser, converts PCM to text
5. Text matched against command table → flight action

**Why offline Vosk instead of cloud STT:** Competition rules forbid external data links.
Vosk runs entirely on the Pi with no internet connection required.

### Why HTTPS is required

Browser security policy requires HTTPS for `getUserMedia()` (microphone access).
Flask by itself is HTTP — a self-signed TLS certificate is needed:

```bash
# Generate self-signed cert (run once on master Pi):
openssl req -x509 -newkey rsa:4096 -keyout master/key.pem \
    -out master/cert.pem -days 365 -nodes \
    -subj "/CN=drone-master"

# Flask launch with TLS:
app.run(host='0.0.0.0', port=443, ssl_context=('cert.pem', 'key.pem'))
```

Browser will show "insecure connection" warning — accept it. This is expected with
a self-signed cert and is fine for competition use.

---

## Flask

**Docs:** https://flask.palletsprojects.com/  
**Install:** `pip install flask`

### Master API endpoints

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/` | GET | Serve `index.html` (phone control UI) |
| `/api/audio_chunk` | POST | Receive PCM audio from phone |
| `/api/drone_update` | POST | Receive mine report from slave |
| `/api/status` | GET | Return JSON status: drones online, mine count, coverage % |
| `/api/command` | POST | Manual override: send command to specific drone |
| `/api/grid` | GET | Return current global grid snapshot as JSON |

---

## Python Dependencies

Full list from `slave/requirements.txt`:

```
mavsdk>=1.4.0
numpy>=1.21
opencv-python>=4.5
pyserial>=3.5
```

Master additionally needs:
```
flask
vosk
openssl (system)
```

Install all on Raspberry Pi:
```bash
pip3 install mavsdk numpy opencv-python pyserial flask vosk --break-system-packages
```

---

## Network Setup

The master Pi runs a WiFi access point. All slaves and the phone connect to it.

| Device | IP | Connects to |
|--------|----| ------------|
| Master Pi (AP) | 10.42.0.1 | — |
| Slave 1 (lead) | 10.42.0.101 | Master |
| Slave 2 (mid) | 10.42.0.102 | Master |
| Slave 3 (tail) | 10.42.0.103 | Master |
| Phone | 10.42.0.201 | Master |

Set static IPs on each Pi in `/etc/dhcpcd.conf`:
```
interface wlan0
static ip_address=10.42.0.101/24
static routers=10.42.0.1
```

Configure master as AP using `hostapd` + `dnsmasq`. The `master/SETUP.md` file has the
exact config.

---

## Useful Tools for Development

| Tool | Purpose |
|------|---------|
| QGroundControl | Monitor drone positions, check PX4 params, view SITL |
| MAVLink Inspector (in QGC) | Debug raw MAVLink messages from PX4 |
| `python -m mavsdk.server` | Start MAVSDK server manually for testing |
| `python slave/test_suite.py` | Run offline unit tests for vision pipeline |
| `python slave/test_mission.py` | Run a SITL simulated flight |
| `tools/pc_thermal_viewer.py` | View live thermal feed from a slave over WiFi |
| `tools/pc_visualizer.py` | Top-down field view with drone positions and mines |

---

## SITL (Software-in-the-Loop) Testing

PX4 SITL lets you run the full flight code without a physical drone.

```bash
# Install PX4 SITL (on development machine, not Pi):
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
make px4_sitl gazebo-classic_iris

# SITL connects to MAVSDK on UDP:14540 automatically
# Run slave code as normal — it won't know it's simulated
```

Our `slave/test_mission.py` was written to run against SITL with a pre-defined waypoint
sequence matching the competition field geometry.
