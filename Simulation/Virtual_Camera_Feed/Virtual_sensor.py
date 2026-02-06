import time
import bisect
import threading
import numpy as np
import datetime

from collections import deque

# Importing the field
from ThermalField import ThermalMap

class VirtualThermalCamera:
    def __init__(self, latency=0.15, fps=8):
        """
        Args:
            latency: Time delay in seconds (e.g., 0.15s).
            fps: Max frames per second the sensor can handle.
        """
        self.latency = latency
        
        self.min_frame_interval = 1.0 / fps
        
        # History Buffer: List to store tuples of (timestamp, x, y, h)
        self.history = deque(maxlen=50)
        
        # Track when we last successfully gave an image
        self.last_read_time = 0
        self.record_interval = 0.01
        self.last_record_time = 0

        #Calling generation of Field so values doesn't change over time
        now = datetime.datetime.now()
        minutes = now.minute
        hours = now.hour
        time_curr = hours + minutes/60 + 5.5
        self.Map = ThermalMap(length=30, width= 10,time=time_curr)
        
        # Lock for thread safety (Since update_state and read run in parallel)
        self.lock = threading.Lock()

    def update_state(self, x, y, h):
        
        now = time.time()
        
        if (now - self.last_record_time) < self.record_interval:
            return
        
        self.last_record_time = now

        with self.lock:
            now = time.time()
            self.history.append((now, x, y, h))

    def _get_lagged_position(self, target_time):
        
        with self.lock:
            if len(self.history) < 2:
                return 0, 0, 0
            
            current_list = list(self.history)

        time_list = [row[0] for row in current_list]
        idx = bisect.bisect_right(time_list, target_time)


        
        # TODO 2: Handle Edge Cases
        if idx == 0 : return current_list[0][1:]
        elif idx == len(self.history) : return current_list[-1][1:]
        
        # TODO 3: Interpolate (Math time!)
        t1, x1, y1, h1 = current_list[idx-1]
        t2, x2, y2, h2 = current_list[idx]

        if t2 - t1 == 0:
            return x1, y1, h1
        
        alpha = (target_time - t1)/(t2 - t1)
        
        x = x1 + (x2 - x1) * alpha
        y = y1 + (y2 - y1) * alpha
        h = h1 + (h2 - h1) * alpha

        return x, y, h

    def read(self):
        
        now = time.time()
        
        # TODO 1: FPS Throttling
        if (now - self.last_read_time) <= self.min_frame_interval: return None, None

        self.last_read_time = now
        
        # TODO 2: Calculate the 'Target Time'
        target_time = now - self.latency
        
        # TODO 3: Get the position from the past
        # (Use the lock here because _get_lagged_position reads the shared list)
        x, y, h = self._get_lagged_position(target_time)
            
        # TODO 4: Generate the image using your generator class
        image, clear_image = self.Map.get_view(drone_height=h,drone_x=x,drone_y=y)
        
        return image, clear_image