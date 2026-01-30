
# 📂 Project Reports & Technical Documentation

This directory contains the complete research, simulation data, and engineering guides for the Aerial Minefield Navigation Challenge.

## 🌟 Primary Strategic Reports
*State-of-the-art detection strategies and physics simulations.*

| File | Description |
| :--- | :--- |
| **[Report_Updated_Robofest_Mine_Simulant_Detection_v2.html](Report_Updated_Robofest_Mine_Simulant_Detection_v2.html)** | **Current Master Strategy.** Details the "One-Pass" sweep strategy, onboard detection pipeline, and sensor fusion approach (Thermal + RGB). |
| **[Report_19+21_Nov_Visual.htm](Report_19+21_Nov_Visual.htm)** | **Disturbed Soil Study.** Visual explanation of why we detect "disturbed soil" rather than plastic, including diagrams of the drone footprint and thermal cues. |
| **[Surface_Temp_Simulation.html](Surface_Temp_Simulation.html)** | **Interactive Physics Engine.** A browser-based simulator to tune parameters (burial depth, time of day, soil type) and predict the exact $\Delta T$ surface contrast. |
  
  
  `Report_19+21_Nov_Visual.htm`  
  => https://prabhuparth442.github.io/Gujcost-Robofest-Resource-IITBBS/Reports/Report_19+21_Nov_Visual.htm
  `Report_Updated_Robofest_Mine_Simulant_Detection_v2.html`  
  => https://prabhuparth442.github.io/Gujcost-Robofest-Resource-IITBBS/Reports/Report_Updated_Robofest_Mine_Simulant_Detection_v2.html
  `According to the report Surface_Temperature_due_to_Mines.pdf`  
  => https://prabhuparth442.github.io/Gujcost-Robofest-Resource-IITBBS/Reports/Surface_Temp_Simulation.html

---

## 🛠️ Simulation & Engineering Guides
*Guides for setting up the SITL (Software-In-The-Loop) environment and simulating the swarm.*

| File | Description |
| :--- | :--- |
| **[ardupilot_gazebo_guide.md](ardupilot_gazebo_guide.md)** | **The "How-To" Manual.** Step-by-step guide to installing ArduPilot, Gazebo, and the bridge plugin. Includes instructions for spawning single drones and swarms. |
| **[ArduPilot_Gazebo_Integration_Errors_Report.md](ArduPilot_Gazebo_Integration_Errors_Report.md)** | **Troubleshooting Handbook.** Solutions for the 5 critical integration errors (Multicast blocking, Protocol Magic, Anaconda conflicts, Safety Veto, etc.). |
| **[Developer_notes.md](Developer_notes.md)** | **Physics Math.** The mathematical specs for the thermal simulation engine, including the 3D diffusion equations and obstruction signatures (Mines vs. Rocks). |

---

## 🚁 Hardware & Swarm Setup
*Physical wiring, networking, and companion computer configuration.*

| File | Description |
| :--- | :--- |
| **[Rpi_Drone_Setup.md](Rpi_Drone_Setup.md)** | **Companion Computer Guide.** Complete setup for Raspberry Pi 4 (Ubuntu 24.04), including headless Wi-Fi (India domain), brownout prevention, and MAVSDK installation. |
| **[drone_swarm_readme.md](drone_swarm_readme.md)** | **Swarm Networking.** Guide for setting up Master-Slave coordination, static IP assignment, and the Python coordination script. |

---


