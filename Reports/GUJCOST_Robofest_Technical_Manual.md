# Aerial Minefield Navigation Challenge
## GUJCOST Robofest 5.0 - Technical Manual

---

## 1. Abstract

This document serves as the comprehensive technical manual for the Aerial Minefield Navigation Challenge (GUJCOST Robofest 5.0), detailing the implementation of a high-fidelity Software-In-The-Loop (SITL) simulation. It integrates the ArduPilot flight control stack with the Gazebo physics simulator to create a testbed for autonomous drone swarms tasked with thermal mine detection.

Beyond standard flight simulation, this handbook bridges the gap between robotics and thermodynamics. It specifies the "First Principles" configuration of the 1D Heat Diffusion Engine, which mathematically models the thermal signatures of buried plastic plates versus surface clutter based on soil energy balance and diurnal solar heating. Additionally, it provides a "Golden Configuration" for networking and troubleshooting, addressing critical integration barriers such as multicast blocking, protocol mismatches, and Anaconda environment conflicts.

---

## 2. Environment Installation & Setup

### 2.1 Prerequisites

Ensure the host machine is running a compatible Linux distribution:

- **OS:** Ubuntu 20.04 (Focal) or 22.04 (Jammy)
- **Core Tools:** Git, CMake, GCC, and Python 3

### 2.2 Network Configuration (Critical Fix)

Modern Linux distributions often disable multicast on localhost, which breaks the bridge connection ("Link 1 down"). You must enable this feature.

**Run this command to enable multicast:**

```bash
sudo ip link set lo multicast on
```

### 2.3 Installing the Bridge Plugin

ArduPilot requires a custom plugin to interface with Gazebo.

**Step 1: Clone the Repository**

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
```

**Step 2: Build the Plugin**

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

### 2.4 Environment Variables

Register the plugin and model paths in your shell configuration.

```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
source ~/.bashrc
```

### 2.5 Safety Configuration (ready.parm)

To allow Python scripts to control the drone without an RC controller, you must disable the pre-arm safety checks.

**Create the file:** `~/ardupilot/ArduCopter/ready.parm`

**Content:**

```
ARMING_CHECK 0
FS_THR_ENABLE 0
FS_GCS_ENABLE 0
DISARM_DELAY 0
```

### 2.6 Swarm Implementation (Deep Configuration)

To simulate a swarm, you must configure unique models, unique ports, and a world file to separate them physically.

**Step 1: Create Unique Models**

Duplicate the default iris model for the second drone.

```bash
cd $GZ_SIM_RESOURCE_PATH
cp -r iris iris_2
```

**Step 2: Modify `iris_2/model.config`**

**Crucial:** You must change the name tag to prevent the "Duplicate Input Frame" error.

```xml
<name>iris_2</name>
```

**Step 3: Modify `iris_2/model.sdf` (Port Offsets)**

Open `iris_2/model.sdf` and locate the `<plugin name="arducopter_plugin">`. You must offset the ports by 10 for every new drone.

| Drone   | Port In (fdm_port_in) | Port Out (fdm_port_out) | Instance Flag |
|---------|----------------------|-------------------------|---------------|
| Drone 1 | 9002                 | 9003                    | -I0           |
| Drone 2 | 9012                 | 9013                    | -I1           |

**Step 4: Create the Swarm World File**

Create a new file `swarm.world`. You must add a `<pose>` offset (e.g., `0 3 0`) to the second drone so they do not spawn inside each other.

```xml
<sdf version="1.6">
  <world name="swarm_world">
    <include>
      <uri>model://iris</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
    <include>
      <uri>model://iris_2</uri>
      <pose>0 3 0 0 0 0</pose>
    </include>
  </world>
</sdf>
```

### 2.7 Launching the Swarm

You need three separate terminals.

**Terminal 1: Gazebo (Load the Swarm World)**

```bash
gz sim -v4 -r swarm.world
```

**Terminal 2: Drone 1 (Instance 0)**

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console -I0 --add-param-file=ready.parm
```

**Terminal 3: Drone 2 (Instance 1)**

- `-I1`: Automatically shifts MAVLink ports to 14560
- `--model JSON`: Mandatory for the bridge plugin

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console -I1 --add-param-file=ready.parm
```

---

## 3. Thermal Physics Engine

### 3.1 The Energy Balance Equation

The core physics engine relies on the Energy Balance Method to compute temperature evolution across the voxel grid. For any subsurface voxel $(i, j, k)$, the change in thermal energy is equal to the sum of heat fluxes from its six neighbors.

#### The 3D Diffusion Equation (Finite Difference)

For a uniform grid, the temperature at the next time step $T_{i,j,k}^{t+1}$ is derived as:

$$T_{i,j,k}^{t+1} = T_{i,j,k}^t + \frac{\Delta t}{\rho C_p} \left[ \frac{k_x (T_{i+1} - 2T_i + T_{i-1})}{\Delta x^2} + \frac{k_y (T_{j+1} - 2T_j + T_{j-1})}{\Delta y^2} + \frac{k_z (T_{k+1} - 2T_k + T_{k-1})}{\Delta z^2} \right]$$

**Where:**

- $\rho C_p$ is the volumetric heat capacity ($J/m^3K$)
- $k$ is the thermal conductivity ($W/m\cdot K$). This variable is spatially dependent:
  - If voxel $(i,j,k)$ is soil, $k = k_{soil} \approx 1.0$
  - If voxel $(i,j,k)$ is a mine, $k = k_{mine} \approx 0.28$ (acting as a thermal insulator)

#### Surface Energy Balance ($z=0$)

The surface voxels are driven by environmental boundary conditions. The net heat flux $Q_{net}$ determines the surface temperature "seen" by the drone's IR camera:

$$Q_{net} = Q_{sun} - Q_{conv} - Q_{rad} - Q_{evap}$$

**Solar Input ($Q_{sun}$):**

$$Q_{sun} = (1 - \text{Albedo}) \cdot I_{solar} \cdot \cos(\theta)$$

- $I_{solar}$: Incident solar irradiance ($W/m^2$)
- $\text{Albedo}$: Reflectivity of the material ($\approx 0.25$ for soil, $\approx 0.15$ for rock)

**Convective Cooling ($Q_{conv}$):**

$$Q_{conv} = h \cdot (T_{surf} - T_{air})$$

- $h = 5.7 + 3.8 \cdot v_{wind}$: Heat transfer coefficient dependent on wind speed $v_{wind}$

**Sky Radiation ($Q_{rad}$):**

$$Q_{rad} = \epsilon \sigma (T_{surf}^4 - T_{sky}^4)$$

- $\sigma$: Stefan-Boltzmann constant
- $T_{sky}$: Effective sky temperature (typically $10-20°C$ lower than $T_{air}$)

### 3.2 Obstruction Signatures (Mines vs. Rocks)

The simulation distinguishes targets based on their thermal impedance relative to the surrounding soil. The specific signatures generated by the physics engine are defined below:

| Feature | Equation Logic | Physical Mechanism | Visual Result (IR Camera) |
|---------|---------------|-------------------|---------------------------|
| **Buried Mine** | $k_{mine} \ll k_{soil}$ | Reflective Boundary: The plastic insulator blocks downward heat flow, causing thermal energy to "pile up" in the soil layer above it. | Warm Blob: A diffuse circular anomaly ($\Delta T \approx +1°C$ to $+4°C$) |
| **Buried Rock** | $k_{rock} \gg k_{soil}$ | Conductive Sink: The rock conducts heat deeper into the ground faster than the soil, "wicking" heat away from the surface. | Cold Blob: A diffuse irregular anomaly (Surface is cooler than surroundings) |
| **Surface Rock** | $\text{Albedo}_{rock} < \text{Albedo}_{soil}$ | Absorption Dominant: Darker color absorbs more $Q_{sun}$, combined with high thermal mass. | Extreme Hot Spot: High contrast ($+10°C$), sharp edges, no diffusion blur |
| **Hollow Cavity** | $k_{air} \approx 0.026$ | Super-Insulator: An almost total block of heat transfer, creating stronger contrast than plastic. | High Contrast Blob: Similar to a mine but with higher $\Delta T$ |

### 3.3 Stability & Simulation Parameters

To prevent numerical instability (where the simulation diverges or crashes), the finite difference solver must adhere to the Courant–Friedrichs–Lewy (CFL) condition for heat diffusion.

**Time Step Limit (Δt):** The time step must satisfy the following inequality to ensure stability:

$$\Delta t < \frac{\rho C_p \Delta z^2}{2k}$$

**Recommendation:** For a grid resolution of $\Delta z = 1$ cm, a safe time step is $\Delta t = 1$ second.

**Convergence Initialization:** The simulation cannot start "cold." To generate a realistic deep-earth temperature gradient, you must run the physics engine for 2 full diurnal cycles (48 hours) prior to the mission start time.

**Detector Definition:** The virtual IR camera reading at pixel coordinates $(x,y)$ is strictly defined as the temperature of the surface voxel:

$$\text{Pixel}(x,y) = T(x,y,z=0)$$

---

## 4. Troubleshooting & Error Repository

This section documents critical errors encountered during the ArduPilot-Gazebo integration and their validated solutions.

### 4.1 Error A: The "One-Way Mirror" (Link 1 Down)

**Symptom:** Gazebo and ArduPilot are running, but the MAVProxy console persistently shows `Link 1 down`. Gazebo logs may occasionally show `[Wrn] Duplicate input frame`.

**Root Cause:**

- **Network Blocking:** Modern Linux distributions (Ubuntu 22.04+) often disable Multicast on Localhost by default for security, preventing the simulation bridge from discovering itself.
- **Configuration:** Hardcoding the `<fdm_port_out>` parameter in the SDF file prevents the plugin from auto-negotiating the connection.

**Fix:**

1. Enable multicast on the loopback interface:
   ```bash
   sudo ip link set lo multicast on
   ```

2. Remove the `<fdm_port_out>` line entirely from your `model.sdf` file.

### 4.2 Error B: "Duplicate Input Frame" (Swarm Collision)

**Symptom:** The Gazebo terminal floods with warnings about duplicate frames, and the drone fails to respond to controls.

**Root Cause:** Occurs during swarm creation when the model folder is duplicated (e.g., `iris` to `iris_2`) but the `model.config` file is not updated. Gazebo treats both folders as the same network entity because they share the same `<name>` ID.

**Fix:**

Open `model.config` in the duplicated folder and ensure the name tag is unique:

```xml
<name>iris_2</name>
```

### 4.3 Error C: "Incorrect Protocol Magic"

**Symptom:** Console output shows `[Wrn] Incorrect protocol magic 0 should be 18458`.

**Root Cause:** The simulation was launched without the `--model JSON` flag. "Magic 0" indicates raw binary data, while "Magic 18458" indicates the JSON format required by the modern ardupilot_gazebo plugin.

**Fix:**

Always include the JSON model flag in your launch command:

```bash
sim_vehicle.py ... --model JSON ...
```

### 4.4 Error D: Compilation Failure (The Anaconda Conflict)

**Symptom:** The build process fails with `fatal error: google/protobuf/message_lite.h`.

**Root Cause:** The host environment has Anaconda installed. CMake detects Anaconda's header files but attempts to link against the system's library files, creating a version mismatch that crashes the build.

**Fix:** Perform a "Nuclear Clean" of the build environment:

1. Delete the build folder
2. Temporarily remove Anaconda from your `$PATH`
3. Recompile strictly using system libraries (`/usr/bin/`)

### 4.5 Error E: "ActionError: FAILED" (Safety Veto)

**Symptom:** The Python control script crashes immediately when attempting to arm the drone.

**Root Cause:** The simulated drone mimics a real vehicle's Pre-Arm Safety Check. It detects "No RC Receiver" (since no joystick is connected) and triggers a failsafe that vetoes the arming command.

**Fix:**

1. Create a parameter file `ready.parm` containing:
   ```
   ARMING_CHECK 0
   FS_THR_ENABLE 0
   FS_GCS_ENABLE 0
   DISARM_DELAY 0
   ```

2. Launch ArduPilot with the parameter file loaded:
   ```bash
   sim_vehicle.py ... --add-param-file=ready.parm ...
   ```

---

*Document Version: 1.0*  
*GUJCOST Robofest 5.0 - Aerial Minefield Navigation Challenge*
