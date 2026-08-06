# Migration Guide — MAVSDK/PX4 → ROS2

## What is this folder?

This folder explains how to migrate our current drone system (built on MAVSDK + direct
Python scripts) to **ROS2** — the standard framework used in professional and research
robotics worldwide.

This is not required for Robofest 6.0. The current MAVSDK system works. But for
Robofest 7.0 and beyond, ROS2 provides:
- Better modularity (swap out components independently)
- Ready-made packages (navigation, SLAM, computer vision)
- Industry-standard tools for debugging and visualisation
- Easier collaboration with other teams and researchers

---

## Files in this folder

| File | What it covers |
|------|---------------|
| [01_ros2_concepts.md](01_ros2_concepts.md) | What ROS2 is, its core concepts, how it differs from what we have |
| [02_mavsdk_to_ros2.md](02_mavsdk_to_ros2.md) | Side-by-side code: our MAVSDK code → equivalent ROS2 code |
| [03_px4_ros2_bridge.md](03_px4_ros2_bridge.md) | How to connect PX4 to ROS2 using uXRCE-DDS (replacing MAVProxy) |
| [04_swarm_in_ros2.md](04_swarm_in_ros2.md) | How to port our 3-drone swarm architecture to ROS2 namespaces |
| [05_migration_plan.md](05_migration_plan.md) | Step-by-step migration plan with effort estimates |

---

## Recommended reading order

**If you're brand new to ROS2:** Read `01_ros2_concepts.md` first. It explains everything
from scratch. Then read `02_mavsdk_to_ros2.md` to see concrete code comparisons.

**If you know ROS2 already:** Jump straight to `03_px4_ros2_bridge.md` for the PX4
integration specifics, then `04_swarm_in_ros2.md` for swarm architecture.

**If you're planning the migration sprint:** Read `05_migration_plan.md`.

---

## Current system summary (what we're migrating FROM)

```
Slave drone:
  Python scripts → MAVSDK → MAVProxy (serial bridge) → Pixhawk (PX4)
  Manual asyncio tasks for each pipeline step
  Manual UDP/TCP sockets for inter-drone communication

Master drone:
  Flask web server
  Manual A* implementation
  Manual TCP command dispatch
  Manual grid merge
```

---

## Target system summary (what we're migrating TO)

```
Slave drone:
  ROS2 nodes → px4_msgs topics → uXRCE-DDS agent → Pixhawk (PX4)
  Each pipeline step = one ROS2 node
  Inter-node communication via ROS2 topics/services/actions

Master drone:
  ROS2 nodes: planner, grid_server, mine_db, voice_interface
  Nav2 for path planning (optional, replaces our A*)
  ROS2 topics for inter-drone communication (via swarm_ros_bridge or DDS)
```

---

## Key external references

- PX4 ROS2 User Guide: https://docs.px4.io/main/en/ros2/user_guide
- uXRCE-DDS bridge docs: https://docs.px4.io/main/en/middleware/uxrce_dds
- ROS2 Humble docs: https://docs.ros.org/en/humble/
- px4-offboard Python example: https://github.com/Jaeyoung-Lim/px4-offboard
- px4_ros2_interface_lib: https://docs.px4.io/main/en/ros2/px4_ros2_control_interface
- swarm_ros_bridge: https://github.com/carryowen/swarm_ros_bridge
