# ArduPilot & Gazebo Integration: Comprehensive Technical Report

A detailed troubleshooting guide for establishing communication between ArduPilot and Gazebo for drone simulation.

---

## Table of Contents

- [Executive Summary](#executive-summary)
- [Detailed Error Analysis](#detailed-error-analysis)
  - [Error A: The "One-Way Mirror" (Link 1 Down)](#error-a-the-one-way-mirror-link-1-down)
  - [Error B: "Duplicate Input Frame"](#error-b-duplicate-input-frame)
  - [Error C: "Incorrect Protocol Magic"](#error-c-incorrect-protocol-magic)
  - [Error D: Compilation Failure (Anaconda Conflict)](#error-d-compilation-failure-the-anaconda-conflict)
  - [Error E: "ActionError: FAILED" (Safety Veto)](#error-e-actionerror-failed-safety-veto)
- [The "Golden" Configuration](#the-golden-configuration)
- [Basic Hovering Script](#basic-hovering-script)
- [How to Run](#how-to-run)

---

## Executive Summary

The objective was to establish a communication link between **ArduPilot** (the flight controller software) and **Gazebo** (the physics simulator) to simulate a drone. Initial attempts following a 2020 tutorial failed due to software version mismatches, network security updates in modern Linux, and environment conflicts (Anaconda).

After a systematic "First Principles" troubleshooting approach, we successfully:

1. Restored the network bridge (Multicast)
2. Eliminated "Ghost" processes and port conflicts
3. Cleaned the build environment of Anaconda interference
4. Established a stable JSON-based telemetry link
5. Automated the safety bypass to allow script-based flight

---

## Detailed Error Analysis

This section breaks down the five critical errors we faced, explaining the technical "Why" behind each one.

### Error A: The "One-Way Mirror" (Link 1 Down)

**Symptom:** Gazebo was running, and ArduPilot was running, but MAVProxy showed `Link 1 down`. Gazebo logs sometimes showed `[Wrn] Duplicate input frame`.

**Root Causes:**

1. **Network:** Modern Linux distributions (Ubuntu 22.04+) often disable **Multicast on Localhost** by default for security. The simulation bridge relies on Multicast to find itself. Gazebo could "hear" ArduPilot, but the firewall/network settings blocked the reply.

2. **Configuration:** The 2020 tutorial instructed us to manually set `<fdm_port_out>`. In the modern plugin, this parameter is **deprecated**. Hardcoding it broke the plugin's ability to "Auto-Detect" where ArduPilot was listening.

**The Fix:**

1. Enabled multicast:
   ```bash
   sudo ip link set lo multicast on
   ```

2. Removed `<fdm_port_out>` from the SDF file to allow auto-negotiation.

---

### Error B: "Duplicate Input Frame"

**Symptom:** The Gazebo terminal flooded with warnings about duplicate frames, and the drone would not respond to controls.

**Root Cause:** This occurred when we attempted to create the Swarm. We duplicated the `iris` folder to create `iris_2` but **failed to edit `model.config`**.

- Gazebo uses `model.config` as the "ID Card" for a model.
- Since both folders claimed to be named "iris", Gazebo loaded two physical drones but treated them as the **same network entity**. It routed data from Drone 2 to Drone 1, causing a collision.

**The Fix:** We reverted to a clean "Single Drone" setup to isolate the issue. For future swarms, every copied folder must have its `model.config` `<name>` tag updated.

---

### Error C: "Incorrect Protocol Magic"

**Symptom:** `[Wrn] Incorrect protocol magic 0 should be 18458`.

**Root Cause:** We attempted to run the simulation without the `--model JSON` flag to test the legacy Binary protocol.

- **Magic 0** = Raw/Binary Data
- **Magic 18458** = JSON Data
- The modern `ardupilot_gazebo` plugin is compiled to expect JSON. When we sent it Binary, it rejected the connection.

**The Fix:** We confirmed that `--model JSON` is mandatory for this specific plugin version.

---

### Error D: Compilation Failure (The Anaconda Conflict)

**Symptom:** `fatal error: google/protobuf/message_lite.h`.

**Root Cause:** The user environment had **Anaconda** installed. Anaconda includes its own version of system libraries (like Google Protobuf).

- When compiling, CMake found Anaconda's *headers* but tried to link against the System's *library files*. This mismatch caused the build to crash.

**The Fix:** We performed a "Nuclear Clean":

1. Deleted the `build` folder
2. Removed Anaconda from `$PATH`
3. Recompiled strictly using `/usr/bin/` system libraries

---

### Error E: "ActionError: FAILED" (Safety Veto)

**Symptom:** The Python script crashed when trying to arm the drone.

**Root Cause:** ArduPilot mimics a real drone. Real drones have a **Pre-Arm Safety Check**.

- The simulated drone detected "No RC Receiver" (because we had no joystick connected) and triggered a Failsafe, refusing to arm.

**The Fix:** We implemented the `ready.parm` method to "brainwash" the drone on startup, forcing it to disable all safety checks and RC requirements.

---

## The "Golden" Configuration

To successfully run this simulation in the future, this is the exact setup you must use.

### A. The Startup Parameter File (`ready.parm`)

**Location:** `~/ardupilot/ArduCopter/ready.parm`

**Purpose:** Disables safety latches so Python can fly the drone.

```text
ARMING_CHECK 0
FS_THR_ENABLE 0
FS_GCS_ENABLE 0
DISARM_DELAY 0
```

### B. The Launch Command

You must launch ArduPilot with these specific flags to connect to MAVSDK and load the parameters.

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console -I0 --add-param-file=ready.parm --out=udp:127.0.0.1:14540
```

**Flag Explanations:**
- `--model JSON`: Required for the plugin
- `--add-param-file`: Loads the safety bypass
- `--out=udp:127.0.0.1:14540`: Opens the port for your Python script

---

## Basic Hovering Script

This is the final, clean Python script. It uses **MAVSDK** to connect to the drone, arm it, take off to 2.5 meters, hover for 10 seconds, and land.

**File Name:** `basic_hover.py`

```python
import asyncio
from mavsdk import System

async def run():
    # --- 1. INITIALIZATION ---
    drone = System()
    
    # We use 'udpin://' to listen for the drone's heartbeat.
    # Port 14540 was specified in the sim_vehicle.py launch command.
    print("Waiting for drone to connect...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    # Wait for the async connection to fully establish
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    # --- 2. HEALTH CHECKS ---
    # ArduPilot requires a valid Global Position (GPS) before it allows
    # Guided/Auto modes. We wait for the simulated GPS to warm up.
    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    # --- 3. MODE SWITCHING ---
    # Critical Step: ArduPilot starts in 'STABILIZE' mode by default.
    # It will reject 'Takeoff' commands unless it is in 'GUIDED' mode.
    # MAVSDK's 'action.hold()' is the safest way to trigger GUIDED mode.
    print("-- Setting Mode to GUIDED")
    await drone.action.hold()

    # --- 4. ARMING ---
    print("-- Arming")
    try:
        await drone.action.arm()
    except Exception as e:
        # If this fails, it's usually because 'ready.parm' wasn't loaded
        print(f"\nERROR: Arming Failed: {e}")
        print("CHECK: Did you run sim_vehicle.py with '--add-param-file=ready.parm'?")
        return

    # --- 5. TAKEOFF ---
    # Standard takeoff rises to approx 2.5 meters
    print("-- Taking off")
    await drone.action.takeoff()

    # --- 6. HOVER ---
    print("-- Hovering for 10 seconds")
    await asyncio.sleep(10)

    # --- 7. LAND ---
    print("-- Landing")
    await drone.action.land()

# Entry point for Python 3.7+
if __name__ == "__main__":
    asyncio.run(run())
```

---

## How to Run

### Step 1: Open Terminal 1 (Gazebo)

```bash
gz sim -v4 -r iris_runway.sdf
```

### Step 2: Open Terminal 2 (ArduPilot)

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console -I0 --add-param-file=ready.parm --out=udp:127.0.0.1:14540
```

*Wait for the console to say `GPS Lock`*

### Step 3: Open Terminal 3 (Python)

```bash
source ~/drone_env/bin/activate
python basic_hover.py
```

### Expected Result

The drone will automatically arm, ascend, hold position for 10 seconds, and descend to a landing.

---

## License

This guide is provided as-is for educational purposes.

## Contributing

Feel free to submit issues or pull requests if you encounter additional errors or have improvements to suggest.
