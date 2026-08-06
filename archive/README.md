# Archive — Development History

This folder collects older and intermediate versions of the codebase.
**For competition use, always use the code in `slave/` and `master/` at the repo root.**

---

## Version history (newest → oldest)

| Folder | Stage | What changed |
|--------|-------|-------------|
| `Files/Structures/slave_competition_full (1)/` | **LATEST** (v2) | Two-stage detection (02 + 06 filters), 4-pass scan, SIDE_MOVE protocol, TF-Luna lidar failsafe, passive grid marking, A* per-pass planner |
| `Files/Structures/master_competition_full (1)/` | **LATEST** (v2) | Vosk offline voice recognition, grid merge from slaves, field-aware A* path planner |
| `Files/Structures/slave_competition_full/` | v1 | Single-pass scan, no SIDE_MOVE, no lidar failsafe |
| `Files/Structures/master_competition_full/` | v1 | Master without grid merge |
| `Files/Structures/slave_competition/` | dev | Vision pipeline under development, missing surface filter |
| `Files/Structures/slave_fixed/` | dev | Bug-fixed intermediate slave |
| `Files/Structures/master_fixed/` | dev | Bug-fixed intermediate master |
| `Files/files/` | dev | Early integrated files + detection subfolder |
| `Files/master/safe_swarm/` | prototype | First swarm prototype with sim harness |
| `Files/slave/safe_swarm/` | prototype | First slave prototype |

---

## Backup/ — early test scripts

`Backup/` contains early standalone flight tests written before the full swarm architecture:

| File | Purpose |
|------|---------|
| `takeoff.py` | Minimal takeoff/land test |
| `hover_test.py` | Hover stability test |
| `indoor_test.py` | Indoor short-range test |
| `helix.py` | Helix trajectory test |
| `spiral.py` | Spiral scan trajectory |
| `t_shape.py` | T-shape formation test |
| `debug_battery.py` | Battery telemetry debug |
| `mav.parm` | ArduPilot parameter file |
| `mav.tlog` | MAVLink telemetry log |

`Backup/Thermal_Sensor/` — MLX90640 Python library and standalone thermal viewer.

`Backup/thermal_env/` — Python virtual environment for thermal testing (large, ~several hundred MB). Not needed for competition — dependencies are in `requirements.txt`.

---

## drone_swarm_folder/ — hardware libraries

| Path | Contents |
|------|---------|
| `drone_swarm_folder/drone_swarm/bcm2835-1.71/` | BCM2835 C library source (Raspberry Pi GPIO/SPI/I²C low-level driver) |
| `drone_swarm_folder/drone_swarm/bcm2835-1.71.tar.gz` | BCM2835 library tarball |
| `drone_swarm_folder/drone_swarm/lib/mlx90640-library/` | MLX90640 32×24 thermal camera C++ library |

See `hardware/README.md` for build instructions.

---

## Complete Specification/

| File | Purpose |
|------|---------|
| `ROBOFEST 5 complete specification guidelines.pdf` | Old Robofest 5 spec (reference only — not the current competition) |
| `Signatures/` | Team + faculty signatures for project submissions |

---

## Why so many versions?

The slave code went through several major iterations:
1. **Prototype** — simple takeoff + hover, no detection
2. **safe_swarm** — first attempt at TCP comms + UDP telemetry
3. **slave_fixed** — fixed coordinate math (was using wrong focal lengths)
4. **slave_competition** — added 02_vision_filter, 03_persistence
5. **slave_competition_full v1** — added surface disc filter (06), A* planner
6. **slave_competition_full v2 (1)** — added SIDE_MOVE protocol, TF-Luna failsafe, 4-pass scan, passive grid marking, lidar-aware sidestep logic

Only v2 (`(1)` folders) is competition-ready. The others are here for reference.
