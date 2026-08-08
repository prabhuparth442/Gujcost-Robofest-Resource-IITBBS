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
Our code enables this automatically — but if ArduPilot rejects GUIDED mode, check:
1. RC is in a position that allows GUIDED mode (set MODE switch or use Mission Planner)
2. SITL is running (for testing) or real flight controller connected
3. MAVProxy bridge is active (port 14540)

### Connecting to ArduCopter (SpeedyBee F405)

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

## ArduPilot — GUIDED Mode

**Official docs:** https://ardupilot.org/copter/docs/ac2_guidedmode.html  
**Parameters reference:** https://ardupilot.org/copter/docs/parameters.html

### GUIDED mode rules (ArduCopter)

- ArduCopter must receive setpoints at >2 Hz in GUIDED mode or it will hold position
- Our code sends waypoints sequentially — if `goto_location()` awaits completion (>500 ms
  between waypoints), the drone may stop responding to waypoints mid-flight
- **Fix:** Call `a velocity keep-alive MAVLink message via pymavlink` as a keep-alive if waypoint dwell is long

### Key ArduCopter parameters to check before flight (SpeedyBee F405)

| Parameter | Recommended | Purpose |
|-----------|-------------|---------|
| `FS_THR_ENABLE` | 1 (RTL) | RC failsafe action on signal loss |
| `EK3_SRC1_POSXY` | 3 (GPS) | Which sensors EKF3 uses for XY position |
| `WPNAV_SPEED` | 300 cm/s | Max horizontal speed in GUIDED/AUTO mode |
| `LAND_SPEED` | 50 cm/s | Descent rate during landing |
| `RTL_ALT` | 300 cm | Return-to-launch altitude above home (cm) |
| `GUID_TIMEOUT` | 3.0 s | GUIDED mode target timeout |

---

## MAVProxy

**Docs:** https://ardupilot.org/mavproxy/  
**Install:** `pip install MAVProxy`

MAVProxy acts as a serial-to-UDP bridge between the SpeedyBee F405 serial port and MAVSDK/pymavlink.

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
| Mission Planner / QGroundControl | Monitor drone positions, check ArduCopter params, view SITL |
| MAVLink Inspector (in Mission Planner or QGC) | Debug raw MAVLink messages from ArduCopter |
| `python -m mavsdk.server` | Start MAVSDK server manually for testing |
| `python slave/test_suite.py` | Run offline unit tests for vision pipeline |
| `python slave/test_mission.py` | Run a SITL simulated flight |
| `tools/pc_thermal_viewer.py` | View live thermal feed from a slave over WiFi |
| `tools/pc_visualizer.py` | Top-down field view with drone positions and mines |

---

## SITL (Software-in-the-Loop) Testing — ArduCopter

ArduCopter SITL lets you run the full flight code without a physical drone.

```bash
# Install ArduCopter SITL (on development machine, not Pi):
pip install --user mavproxy
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# Run ArduCopter SITL with Gazebo (or standalone):
cd ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map

# SITL listens on UDP:14550 (MAVProxy), MAVSDK connects to UDP:14540
# mavproxy.py bridges between SITL and MAVSDK automatically
# Run slave code as normal — it won't know it's simulated
```

Our `slave/test_mission.py` was written to run against SITL with a pre-defined waypoint
sequence matching the competition field geometry.
