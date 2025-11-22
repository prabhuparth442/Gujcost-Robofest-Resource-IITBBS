# 🛰 Aerial Robotics – Minefield Navigation Challenge (GUJCOST Robofest)

This repository collects **design notes, reports, and reference material** for our swarm-drone solution to the GUJCOST Robofest *Aerial Robotics: Minefield Navigation* challenge.

The repo is meant to be a single place where the team can track:

- the **problem understanding** and constraints,
- our **swarm architecture & information-flow ideas**, and
- the **sensor / detection reasoning** for underground mines.

---

## 🔍 What our solution is about (short)

Based on the ideation and design documents in this repo:

- We use a **small swarm of quadrotors** (one master + multiple scanning drones) to escort a human across a minefield.
- The swarm first performs a **field-scan phase** with the human waiting at the start line.  
  Each scanning drone:
  - flies low over the ground,
  - collects sensor data (thermal + RGB, and possibly other lightweight sensors),
  - builds a **local occupancy / risk map** around itself, and
  - sends compressed map tiles and health info to the master drone.
- The master drone fuses:
  - local sub-maps from all drones,
  - any prior information provided in the problem statement,
  - and our own disturbed-soil / anomaly detections
  into a **global risk map**.
- Instead of trying to “see” deep plastic mines directly from 2 m altitude (which is physically unrealistic), we focus on detecting **recently disturbed soil patches of mine-like size** using:
  - **thermal IR anomalies** (1–2 °C contrast after burial under good conditions),
  - **RGB texture + micro-topography** (slight mounds/depressions and soil texture changes).
- From the global risk map the master plans a **safe corridor** with a margin around all suspicious patches.  
  The swarm then forms a moving formation around the human, continuously updating the corridor and giving clear “go / stop / turn” cues.

Overall aim: a detection-agnostic, safety-first escort system that honestly respects the physics limits while still using the best available cues (disturbed soil, prior map, sensor fusion).

---

## 📁 Repository Structure

- `Reports/`  
  Notes and working documents:
  - `18Nov.md`, `21Nov.md`, … – Recorded Research and other details we found.
  - `README.md` – short explanations per report (if needed).

- `Resources/`  
  Local copies of **papers, articles, and tutorials** that we found useful during ideation:
  - UAV and autopilot basics
  - Swarm robotics and multi-UAV coordination
  - Path-planning and occupancy-grid mapping
  - Sensing and detection methods relevant to landmines / underground targets  
  (File names are descriptive; see the folder listing in GitHub.)

- `LICENSE`  
  Repository license.



---

## 🤝 Contributing
- Add **only open-access** resources.  
- Follow the naming format: `##_<ShortTitle>.pdf`.

---

## 📜 License
This repository is for **educational and research purposes only**.  
Linked resources remain property of their respective authors and publishers.
