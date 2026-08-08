# Getting Started — Complete Beginner's Guide

If you just joined the team and have never worked with drones or embedded Python before,
start here. This document explains everything from scratch: what the system does, what
each piece of hardware is, and how to run your first test.

---

## What are we building?

We are building a **swarm of 3 drones** that fly over a minefield and find landmines
automatically — without any human flying them and without GPS during the mission.

The drones use a **thermal (heat) camera** to spot buried mines, because buried objects
retain heat differently from surrounding soil. When a mine is found, the drone reports
its GPS coordinates to a central "master" drone, which keeps a map.

A person controls the mission by **speaking voice commands** into their phone browser —
the system has offline speech recognition so it works without internet.

---

## Hardware on each drone

You don't need to buy or build anything to start learning — but understanding what's
physically on the drone helps.

### Raspberry Pi 4
A small credit-card sized computer (not Windows, not macOS — it runs Linux). This is
the "brain" of the drone. All our Python code runs here.

Think of it like a very small laptop attached to the drone.

### SpeedyBee F405 (ArduCopter)
A dedicated All-in-One flight controller + ESC board that handles the actual flying —
keeping the drone stable, reading motor speeds, handling the gyroscope. It talks to
the Raspberry Pi over a UART serial connection.

You write Python code on the Pi, and the Pi tells the SpeedyBee where to fly.

### MLX90640 Thermal Camera
A tiny sensor (roughly the size of a postage stamp) that measures temperature at
768 points arranged in a 32×24 grid — like a camera but instead of colour it shows
heat. It connects to the Pi over I²C (a simple 2-wire communication protocol).

### TF-Luna LIDAR
A laser rangefinder that measures distance to the closest object in front of it.
We use it as an emergency obstacle detector — if something is within 1 metre,
the drone automatically swerves.

---

## Software stack — what runs where

```
Your Phone (browser)
    │  speaks voice commands → WiFi → Master Pi
    ▼
Master Drone (Raspberry Pi 4)
    │  runs Flask web server + Vosk speech recognition + A* path planner
    │  sends flight commands over WiFi → Slave Drones
    ▼
Slave Drones (Raspberry Pi 4 × 3)
    │  each runs the detection pipeline + pymavlink/MAVSDK to fly
    │  sends mine reports back → Master
```

Everything communicates over a **local WiFi network** that the master drone creates
(it acts as a WiFi hotspot). There is no internet connection during a mission.

---

## Key concepts explained simply

### What is MAVSDK / pymavlink?
MAVSDK and pymavlink are Python libraries that let you control an ArduPilot flight
controller from code. Instead of manually setting motor speeds, you say things like:

```python
# MAVSDK style:
await drone.action.arm()
await drone.action.takeoff()
await drone.goto_location(lat, lon, altitude)

# pymavlink style (lower-level, more control):
mav.arducopter_arm()
mav.mav.command_long_send(...)  # MAV_CMD_NAV_TAKEOFF
```

Both translate into **MAVLink** messages that the SpeedyBee F405 understands.

**MAVLink** is the protocol ArduPilot uses to receive commands — think of it like
the language the SpeedyBee speaks. MAVSDK/pymavlink handle the translation so you
don't need to learn raw MAVLink directly.

**MAVProxy** is a ground-station bridge that sits between the Pi and the SpeedyBee,
forwarding MAVLink over UDP so both MAVSDK and ground control apps can connect.

### What is GUIDED mode?
By default, the SpeedyBee listens to an RC transmitter (a remote control). **GUIDED mode**
is ArduCopter's equivalent of offboard control — the FC instead accepts waypoint commands
from the companion computer (our Pi). MAVSDK/pymavlink request this mode automatically
before sending flight commands.

In Mission Planner this shows as "Mode: GUIDED" in the status bar.

### What is asyncio?
Python code normally runs one line at a time. `asyncio` lets Python do multiple things
"at the same time" — for example, reading thermal frames AND listening for TCP commands
AND running the path planner simultaneously.

You'll see `async def` and `await` throughout the code. If you're new to this:
- `async def` means "this function can pause and let other things run"
- `await` means "pause here until this finishes, but let other code run while waiting"

A good beginner introduction: https://realpython.com/async-io-python/

### What is TCP vs UDP?
Both are ways to send data over a network.
- **UDP** — fast, fires-and-forgets, no guarantee of delivery. We use this for telemetry
  (drone positions) because it's okay if one packet is lost.
- **TCP** — slower, guaranteed delivery, re-sends if packet is lost. We use this for mine
  reports because we can't afford to lose a detection.

### What is a Flask server?
Flask is a Python library for building web servers. The master drone runs a Flask server
so that a phone browser can connect to it and show a control interface. When you open
`https://10.42.0.1` on your phone (connected to the drone's WiFi), Flask serves the page.

---

## Setting up your development environment

### Step 1 — Install Python 3.10+

```bash
# Check your Python version:
python3 --version

# If below 3.10, install it:
sudo apt install python3.10  # on Ubuntu/Raspberry Pi OS
```

### Step 2 — Clone / copy the repo

```bash
# If using git:
git clone <your-repo-url> Gujcost_Files
cd Gujcost_Files
```

### Step 3 — Install Python dependencies

```bash
# For slave drones:
pip3 install -r slave/requirements.txt --break-system-packages

# For master drone:
pip3 install flask vosk mavsdk --break-system-packages
```

### Step 4 — Test the vision pipeline offline (no drone needed)

This runs the thermal detection filters against pre-recorded mine data:

```bash
cd Gujcost_Files
python3 slave/test_suite.py
```

You should see output like:
```
[TEST] mine_01 ... DETECTED (conf=0.87)
[TEST] mine_02 ... DETECTED (conf=0.91)
[TEST] bare_soil_01 ... no detection ✓
```

### Step 5 — Run the PC visualiser

With no drone connected, you can still launch the PC viewer to see what the system looks like:

```bash
python3 tools/pc_visualizer.py
```

---

## File map — where is what

If you're looking for something specific:

| I want to understand... | Read this file |
|------------------------|----------------|
| How a slave drone flies from point A to B | `slave/main_orchestrator_competition.py` |
| How thermal detection works | `slave/02_vision_filter.py` |
| How mine positions are confirmed (not false positives) | `slave/03_persistence.py` |
| How GPS pixel offset is computed | `slave/04_coordinate_math.py` |
| How the master coordinates all drones | `master/app.py` |
| How path planning works (A*) | `master/app.py` — search for `a_star` |
| How the coverage map is built | `slave/grid_map.py` |
| How the field geometry is defined | `slave/fieldmap.py` |
| How voice commands work | `master/app.py` — search for `audio_chunk` |
| How to set up the hardware from scratch | `hardware/README.md` |
| Competition rules | `docs/competition_brief.md` |
| Full system architecture diagram | `docs/architecture.md` |

---

## Running a simulated mission (SITL)

SITL stands for **Software In The Loop** — it simulates a real ArduCopter flight
controller on your laptop.
You can run the full slave drone code without any physical drone.

### Install ArduCopter SITL (on your laptop, not the Pi)

```bash
# Install prerequisites
pip install --user mavproxy
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# Run SITL (opens a simulated ArduCopter)
cd ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```

This opens MAVProxy console + a map view with a simulated drone. MAVSDK or pymavlink
connects to it over UDP port 14540 (via MAVProxy) — exactly the same as the real drone.

### Run the slave code against SITL

```bash
# Terminal 1: ArduCopter SITL (leave running)
cd ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map

# Terminal 2: slave flight test
cd Gujcost_Files
python3 slave/test_mission.py
```

The simulated drone will take off, fly a scan pattern, and land — with thermal frames
coming from the pre-recorded test data.

---

## Common first-time mistakes

| Mistake | Fix |
|---------|-----|
| `Permission denied` running mlx_stdout | Run with `sudo` — BCM2835 needs root for GPIO |
| `No heartbeat received` from MAVSDK | MAVProxy is not running; start it first |
| Flask shows "http" but phone mic doesn't work | Flask must use HTTPS; generate TLS cert (see `master/SETUP.md`) |
| `ModuleNotFoundError: mavsdk` | Run `pip3 install mavsdk pymavlink --break-system-packages` |
| Drone arms but doesn't move | ArduCopter rejected GUIDED mode; check RC mode switch and GUID param |
| Thermal frame is all zeros | MLX binary crashed; check I²C wiring and run with sudo |

---

## Glossary

| Term | Plain English |
|------|-------------|
| EKF | Extended Kalman Filter — the maths inside the SpeedyBee (ArduPilot EKF3) that fuses all sensor readings into a position estimate |
| FPN | Fixed Pattern Noise — systematic hot/cold pixel errors baked into the thermal sensor that we subtract out |
| NED | North-East-Down coordinate frame — standard aviation coordinate system |
| GPS | Global Positioning System — satellite-based location. Banned during mission; only used to set origin |
| I²C | A simple 2-wire bus for connecting chips to a microcontroller. The thermal camera uses this |
| UART | Universal Asynchronous Receiver-Transmitter — serial communication. The LIDAR uses this |
| A* | "A-star" — a graph search algorithm for finding the shortest path around obstacles |
| asyncio | Python library for writing code that can do multiple things concurrently |
| MAVLink | The message protocol drones use to communicate |
| DDS | Data Distribution Service — the publish-subscribe middleware ROS2 is built on |
| ROS2 | Robot Operating System 2 — a framework for robot software (see `migration/` if interested) |
