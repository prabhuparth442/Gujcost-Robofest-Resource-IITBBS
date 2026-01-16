# Implementation Guide: ArduPilot & Gazebo Simulation
## Single Vehicle and Swarm Configuration

**Engineering Technical Documentation**

---

## Abstract

This document serves as a technical manual for setting up a software-in-the-loop (SITL) simulation using ArduPilot and Gazebo. It covers the installation of necessary plugins, the configuration of a single drone, and the advanced networking steps required to simulate a drone swarm. This guide is designed to be a standalone resource for implementation.

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Part 1: Single Drone Configuration](#part-1-single-drone-configuration)
   - [Install the Bridge Plugin](#1-install-the-bridge-plugin)
   - [Launching Single Simulation](#2-launching-single-simulation)
3. [Part 2: Drone Swarm Implementation](#part-2-drone-swarm-implementation)
   - [Creating Unique Models](#1-creating-unique-models)
   - [Creating the Swarm World](#2-creating-the-swarm-world)
   - [Launching the Swarm](#3-launching-the-swarm)
4. [References & Resources](#references--resources)

---

## Prerequisites

Before proceeding, ensure the host machine (preferably running Ubuntu 20.04 or 22.04) has the following core components installed:

- **ArduPilot Source Code:** The flight controller firmware.
- **Gazebo:** The physics simulator (Gazebo Garden/Harmonic or Gazebo Classic 11).
- **Git & Build Tools:** CMake, GCC, and Python 3.

---

## Part 1: Single Drone Configuration

### 1. Install the Bridge Plugin

ArduPilot communicates with Gazebo via a specific plugin.

**Step 1: Clone the official repository**

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
```

**Step 2: Build the plugin (Example for Gazebo Harmonic)**

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

**Step 3: Configure environment variables in `.bashrc`**

```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
source ~/.bashrc
```

### 2. Launching Single Simulation

Run the following in two separate terminals:

**Terminal 1 (Gazebo):**

```bash
gz sim -v4 -r iris_runway.sdf
```

**Terminal 2 (ArduPilot SITL):**

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console
```

---

## Part 2: Drone Swarm Implementation

To simulate a swarm, we must ensure that each drone instance communicates on unique network ports so they do not conflict.

### 1. Creating Unique Models

We must duplicate the standard drone model (e.g., `iris`) and assign unique UDP ports for the SITL connection.

**Step 1: Duplicate the Model Folder**

Navigate to your models directory and copy the `iris` folder to create `iris_2`.

**Step 2: Modify `model.config`**

Open `iris_2/model.config` and change the name tag:

```xml
<name>iris_2</name>
```

**Step 3: Modify `model.sdf` (Critical Step)**

Open `iris_2/model.sdf`. Locate the `ArduCopter Plugin` section and offset the ports by 10 for every new drone.

#### Port Configuration Table

| Drone | Port In (fdm_port_in) | Port Out (fdm_port_out) | Instance Flag |
|-------|----------------------|------------------------|---------------|
| Drone 1 (Default) | 9002 | 9003 | -I0 |
| Drone 2 | 9012 | 9013 | -I1 |
| Drone 3 | 9022 | 9023 | -I2 |

**Example SDF modification for Drone 2:**

```xml
<plugin name="arducopter_plugin" filename="libArduPilotPlugin.so">
    <fdm_addr>127.0.0.1</fdm_addr>
    <fdm_port_in>9012</fdm_port_in>
    <fdm_port_out>9013</fdm_port_out>
    ...
</plugin>
```

### 2. Creating the Swarm World

Create a new world file (e.g., `swarm.world`) and include your modified models. Offset their poses so they do not spawn inside one another.

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

### 3. Launching the Swarm

You need one terminal for Gazebo, and one unique terminal for *each* drone's flight controller.

**Terminal 1 (Gazebo):**

```bash
gz sim -v4 -r swarm.world
```

**Terminal 2 (Drone 1):**

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 --console
```

**Terminal 3 (Drone 2):**

```bash
# -I1 automatically shifts MAVLink ports to 14560
# --model JSON ensures it talks to the new plugin structure
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I1 --console
```

---

## References & Resources

For further troubleshooting and advanced configurations, consult the following:

- **ArduPilot Wiki (SITL):** https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html
- **ArduPilot Gazebo Plugin Repo:** https://github.com/ArduPilot/ardupilot_gazebo
- **Intelligent Quads (Video Tutorial):** https://www.youtube.com/watch?v=r15Tc6e2K7Y
- **Gazebo Documentation:** https://gazebosim.org/docs