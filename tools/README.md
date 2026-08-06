# Tools — PC-side Visualisation and Debugging

These scripts run on a **laptop/desktop** (not on the drones) during testing and development.
They receive data from the drones over the local network and display thermal images, mine maps, and sector results.

---

## pc_thermal_viewer.py

Real-time display of the MLX90640 thermal camera feed.
Connects to the slave drone's raw frame stream and shows it as a false-colour image.

```bash
python3 pc_thermal_viewer.py --ip <slave-drone-ip>
```

---

## pc_sector_viewer.py

Displays a sector result packet sent by `08_comms_link.DroneTunnel.send_sector_result()`.
Shows three images side by side:
1. Raw thermal average
2. Binary detection mask (where the filter fired)
3. Final annotated image (mine circle + confidence)

```bash
python3 pc_sector_viewer.py
# Listens on TCP port 5000 (same as mine report port)
```

---

## pc_viewer.py

Generic packet viewer — shows any JSON packet received from the drones.

```bash
python3 pc_viewer.py
```

---

## pc_visualizer.py

More advanced visualisation with field overlay.
Draws drone positions, mine markers, and scan coverage on a top-down field map.

```bash
python3 pc_visualizer.py
```

---

## pc_binary_viewer.py

Shows the binary (thresholded) output of the vision filter.
Useful for tuning detection band parameters without flying.

```bash
python3 pc_binary_viewer.py
```

---

## raw_listener.py

Listens for raw MLX90640 frame bytes (float32 stream) and saves them to disk.
Used to capture real-world mine data for offline testing.

```bash
python3 raw_listener.py --output captures/
```

---

## Using captured data for testing

The `slave/test_data/captures/` directory contains real mine captures from field tests:
```
mine_01_20260319_162211/
    raw_stack.npy     ← (N, 24, 32) float32 frames
    corrected.npy     ← FPN-corrected version
    metadata.json     ← GPS, confidence, timestamp
```

Run the vision filter against captured data:
```python
import numpy as np
from slave import vision_filter  # or 06_surface_filter
stack = np.load("slave/test_data/captures/mine_01_20260319_162211/raw_stack.npy")
fpn   = np.load("slave/config/fpn_pattern.npy")
dx, dy, conf = vision_filter.process_memory_stack(stack, fpn)
print(f"Detection: ({dx}, {dy}) conf={conf:.2f}")
```
