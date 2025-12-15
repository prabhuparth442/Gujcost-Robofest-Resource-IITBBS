import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
collections.Iterable = collections.abc.Iterable

import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from dronekit import connect
from field_simulation import Thermal_map
import numpy as np
import random

# 1. SETUP MAP
print("Generating Tactical Map...")
# Sync settings with mission.py
sim = Thermal_map(seed=42, n_rocks=30) 
for i in range(15):
    random.seed(i*99)
    sim.add_mine(int(random.uniform(2,38)), int(random.uniform(2,38)))
sim.add_mine(20, 30)

# 2. CONNECT
print("Connecting to Telemetry stream...")
vehicle = connect('127.0.0.1:14552', wait_ready=True)

# 3. SETUP PLOT
fig, ax = plt.subplots(figsize=(8, 8))
fig.canvas.manager.set_window_title("Tactical Thermal Feed")

ax.imshow(sim.grid, cmap='inferno', origin='lower', vmin=22, vmax=30, 
          extent=[0, 40, 0, 40]) 

drone_marker, = ax.plot([], [], 'g^', markersize=15, markeredgecolor='white', label='Drone')
path_line, = ax.plot([], [], 'g-', alpha=0.5, linewidth=1)

x_data, y_data = [], []

def update(frame):
    if vehicle.location.global_relative_frame.alt is None: return
    loc = vehicle.location.local_frame
    if loc.north is None: return
    
    # Map Transformation
    map_x = 20.0 + loc.east
    map_y = 20.0 + loc.north 
    
    x_data.append(map_x)
    y_data.append(map_y)
    
    drone_marker.set_data([map_x], [map_y])
    path_line.set_data(x_data, y_data)
    ax.set_title(f"Alt: {vehicle.location.global_relative_frame.alt:.2f}m")

ani = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
plt.legend()
plt.grid(color='white', alpha=0.2)
plt.show()
