# Research Reference Library — Gujcost Robofest 6.0

This folder contains literature summaries, technical references, and implementation notes
organised by topic for the 2026 Aerial Robotics: Minefield Navigation team.

Every document is a **digest** — enough to understand the idea, know where to look deeper,
and connect the paper's findings to our actual codebase. Full PDFs are available at the
arxiv / DOI links inside each file.

---

## Topic Index

| File | Topic | Most relevant to |
|------|-------|-----------------|
| [01_thermal_mine_detection.md](01_thermal_mine_detection.md) | Infrared thermography for landmine detection | `02_vision_filter.py`, `06_surface_filter.py`, FPN calibration |
| [02_gps_denied_navigation.md](02_gps_denied_navigation.md) | Localisation without GPS | `00_preflight_calib.py`, coordinate system, origin_state.json |
| [03_swarm_coordination.md](03_swarm_coordination.md) | Multi-UAV formation and task sharing | `master/app.py`, SIDE_MOVE protocol, UDP telemetry |
| [04_path_planning.md](04_path_planning.md) | Coverage path planning and A* variants | `fieldmap.py`, A* in `master/app.py` |
| [05_sensor_hardware.md](05_sensor_hardware.md) | MLX90640, TF-Luna, ArduPilot/pymavlink | `slave/src/mlx_stdout.cpp`, `tf_luna_failsafe.py` |
| [06_implementation_references.md](06_implementation_references.md) | pymavlink, MAVProxy, Vosk, ArduCopter GUIDED mode — official docs | All drone flight code |

---

## Full Bibliography (APA)

### Thermal / Mine Detection

1. **[arXiv:2410.23998]** Anonymous. (2024). *UAV-based detection of landmines using infrared thermography*. arXiv. https://arxiv.org/abs/2410.23998

2. **[Dataset]** Authors not listed. (2023). *Dataset of thermographic images for the detection of buried landmines*. Data in Brief, 49, 109312. https://doi.org/10.1016/j.dib.2023.109312  — NCBI PMC10403701

3. **[FPN / NUC]** Barral, V., et al. (2024). *Fixed Pattern Noise Removal For Multi-View Single-Sensor Infrared Camera*. WACV 2024. https://openaccess.thecvf.com/content/WACV2024/papers/Barral_Fixed_Pattern_Noise_Removal_for_Multi-View_Single-Sensor_Infrared_Camera_WACV_2024_paper.pdf

4. **[FPN semi-calib]** Anonymous. (2023). *Fixed Pattern Noise Removal Based on a Semi-Calibration Method*. IEEE Xplore. https://ieeexplore.ieee.org/document/10122709/

### GPS-Denied Navigation

5. **[arXiv:2409.10193]** Anonymous. (2024). *Relative Positioning for Aerial Robot Path Planning in GPS Denied Environment*. arXiv. https://arxiv.org/abs/2409.10193

6. **[PMC7256583]** Anonymous. (2020). *Autonomous Navigation for Drone Swarms in GPS-Denied Environments Using Structured Learning*. PMC. https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7256583/

7. **[Drones 9(6):442, 2025]** Anonymous. *UAV Autonomous Navigation System Based on Air-Ground Collaboration in GPS-Denied Environments*. MDPI Drones. https://www.mdpi.com/2504-446X/9/6/442

### Swarm Coordination

8. **[arXiv:2409.17798]** Zhu, F., Ren, Y., Yin, L., Kong, F., Liu, Q., Xue, R., Liu, W., Cai, Y., Lu, G., Li, H., & Zhang, F. (2024). *Swarm-LIO2: Decentralized, Efficient LiDAR-inertial Odometry for UAV Swarms*. arXiv. https://arxiv.org/abs/2409.17798

9. **[Drones 8(7):320, 2024]** Anonymous. *Advancement Challenges in UAV Swarm Formation Control: A Comprehensive Review*. MDPI Drones. https://www.mdpi.com/2504-446X/8/7/320

### Path Planning

10. **[arXiv:2505.08060]** Anonymous. (2025). *Land-Coverage Aware Path-Planning for Multi-UAV Swarms in Search and Rescue Scenarios*. arXiv. https://arxiv.org/abs/2505.08060

11. **[ACM doi/10.1145/3737280]** Anonymous. *Comprehensive Review of Path Planning Techniques for UAVs*. ACM. https://doi.org/10.1145/3737280

12. **[Remote Sensing 16(21):4019, 2024]** Anonymous. *A Review of UAV Path-Planning Algorithms and Obstacle Avoidance Methods*. MDPI Remote Sensing. https://www.mdpi.com/2072-4292/16/21/4019

13. **[Wiley 10.1155/2024/5114696]** Anonymous. (2024). *Enhanced Multi-UAV Path Planning with Voronoi-Based Obstacle Modelling and Q-Learning*. Wiley. https://doi.org/10.1155/2024/5114696

14. **[SagePub 2024]** Yang, Z., Yang, Y., He, X., & Qi, W. (2024). *Incremental coverage path planning method for UAV ground mapping in unknown area*. International Journal of Micro Air Vehicles. https://journals.sagepub.com/doi/10.1177/17568293241262323

### Sensor Hardware

15. **[MLX90640 datasheet]** Melexis. (2019). *MLX90640 32×24 IR Array Datasheet*. https://www.melexis.com/en/documents/documentation/datasheets/datasheet-mlx90640

16. **[MLX90640 library]** pimoroni/mlx90640-library. GitHub. https://github.com/pimoroni/mlx90640-library

17. **[TF-Luna]** Benewake. *TF-Luna Single-Point Ranging LiDAR — Product Manual*. https://en.benewake.com/TFLuna

18. **[BCM2835]** Hall, M. *BCM2835 C library for Raspberry Pi*. http://www.airspayce.com/mikem/bcm2835/

### Implementation / SDK

19. **[MAVSDK-Python]** MAVSDK. *MAVSDK-Python Documentation*. https://mavsdk.mavlink.io/main/en/python/

20. **[ArduPilot GUIDED Mode]** ArduPilot Dev Team. *ArduCopter GUIDED Mode*. https://ardupilot.org/copter/docs/ac2_guidedmode.html

21. **[MAVProxy]** ArduPilot. *MAVProxy Documentation*. https://ardupilot.org/mavproxy/

22. **[Vosk API]** Alpha Cephei. *Vosk Offline Speech Recognition API*. https://alphacephei.com/vosk/ — GitHub: https://github.com/alphacep/vosk-api

23. **[Flask HTTPS]** Pallets. *Flask TLS / HTTPS documentation*. https://flask.palletsprojects.com/en/latest/deploying/

---

## How to use this folder

Each topic file explains:
- What the research says (the findings that matter for us)
- How it connects to our current code
- What to try / improve for Robofest 6.0

Start with `01_thermal_mine_detection.md` if you're working on the vision pipeline.
Start with `04_path_planning.md` if you're improving coverage efficiency.
Start with `06_implementation_references.md` if you're setting up a new drone from scratch.
