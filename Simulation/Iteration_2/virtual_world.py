from collections import deque
import datetime
import time
import bisect
import threading
import random

class VirtualThermalCamera:
    def __init__(self, latency=0.15, fps=8):
        self.latency = latency
        self.min_frame_interval = 1.0 / fps
        self.history = deque(maxlen=50)
        self.last_read_time = 0
        self.record_interval = 0.01
        self.last_record_time = 0

        # Time setup
        now_init = datetime.datetime.now()
        time_curr = now_init.hour + now_init.minute/60 + 5.5

        # --- FIX 1: Set Length to 100m (Not 10m) ---
        self.Map = ThermalMap(length=10, width=12, time=13)

        self.add_anomalies(mines = 8, length = 10, width = 12)

        self.lock = threading.Lock()



    def add_anomalies(self, surface_rocks = 5, underground_rocks = 27,
                      mines = 5, length = 100,width = 20):

        print("Initializing Field ...")
        locations = np.array([[0,0]])
        # Fail safe if current number of mines cannot be added
        tries = 0
        iter = 0
        while True:
            mx = random.uniform(2.0, width  - 2.0)
            my = random.uniform(2.0, length - 2.0)
            new_loc = [mx, my]
            # Minimum Distance between two mines
            d = 0.5
            # Fail safe if current number of mines cannot be added
            if tries > 100 : break
            tries += 1
            distances = np.linalg.norm(locations - new_loc, axis=1)
            if np.any(distances < d):
                continue

            self.Map.add_mine(mx, my, 0.1)
            locations = np.vstack([locations, new_loc])

            iter += 1
            tries = 0
            if iter >= mines : break

        print(f" -> Seeding {surface_rocks+underground_rocks}Anomalies...")
        self.Map.add_anomaly(count_surface=surface_rocks,
                             count_underground=underground_rocks)



    def update_state(self, x, y, h):
        now = time.time()
        if (now - self.last_record_time) < self.record_interval:
            return
        self.last_record_time = now
        with self.lock:
            self.history.append((now, x, y, h))

    def _get_lagged_position(self, target_time):
        with self.lock:
            if len(self.history) < 2: return 0, 0, 0
            current_list = list(self.history)

        time_list = [row[0] for row in current_list]
        idx = bisect.bisect_right(time_list, target_time)

        if idx == 0: return current_list[0][1:]
        elif idx >= len(current_list): return current_list[-1][1:]

        t1, x1, y1, h1 = current_list[idx-1]
        t2, x2, y2, h2 = current_list[idx]

        if t2 - t1 == 0: return x1, y1, h1
        alpha = (target_time - t1)/(t2 - t1)
        x = x1 + (x2 - x1) * alpha
        y = y1 + (y2 - y1) * alpha
        h = h1 + (h2 - h1) * alpha
        return x, y, h

    def read(self):
        now = time.time()
        if (now - self.last_read_time) <= self.min_frame_interval:
            return None, None # Correctly returns tuple

        self.last_read_time = now
        target_time = now - self.latency
        x, y, h = self._get_lagged_position(target_time)

        # DEBUG PRINT
        print(f"[CAM DEBUG] Read Pos: {x:.2f}, {y:.2f} | Time: {target_time:.2f}")

        return self.Map.get_view(drone_height=h, drone_x=x, drone_y=y)
