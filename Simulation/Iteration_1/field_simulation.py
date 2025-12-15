import numpy as np 
import random

class Thermal_map:
    def __init__(self, seed=42, n_rocks=50):
        self.meters_total = 40
        self.pixels_total = 1280
        self.scale = self.pixels_total / self.meters_total

        random.seed(seed)
        np.random.seed(seed)
        
        print("Initializing Thermal Map ...")
        self.grid = np.full((self.pixels_total, self.pixels_total), 22.0)

        # Background noise
        noise = np.random.normal(0, 0.6, (self.pixels_total, self.pixels_total))
        self.grid += noise

        # Optional rocks
        if n_rocks > 0:
            self.scatter_rocks(count=n_rocks)

    def scatter_rocks(self, count):
        print(f"Scattering {count} rocks...")
        for _ in range(count):
            rx = random.uniform(0, self.meters_total)
            ry = random.uniform(0, self.meters_total)
            
            base_size = random.uniform(1.5, 5.0)
            stretch_ratio = random.uniform(1.0, 3.0) 
            
            if random.choice([True, False]):
                spread_x = base_size * stretch_ratio
                spread_y = base_size
            else:
                spread_x = base_size
                spread_y = base_size * stretch_ratio

            theta = random.uniform(0, 360)

            self._add_gaussian_blob(rx, ry, 
                                    peak_delta_T=random.uniform(2.0, 5.0), 
                                    spread_x=spread_x, 
                                    spread_y=spread_y, 
                                    rotation_deg=theta)

    def add_mine(self, x_meter, y_meter):
        # Perfect Circles
        self._add_gaussian_blob(x_meter, y_meter, peak_delta_T=8, spread_x=80, spread_y=80, rotation_deg=0)
        print(f"Mine added at ({x_meter},{y_meter})")
        
    def _add_gaussian_blob(self, x_meter, y_meter, peak_delta_T, spread_x, spread_y, rotation_deg, radius=20):
        center_x = int(x_meter * self.scale)
        center_y = int(y_meter * self.scale)

        y, x = np.ogrid[-radius:radius, -radius:radius]
        
        theta = np.radians(rotation_deg)
        x_rot = x * np.cos(theta) - y * np.sin(theta)
        y_rot = x * np.sin(theta) + y * np.cos(theta)

        heat_map = peak_delta_T * np.exp( - ( (x_rot**2)/spread_x + (y_rot**2)/spread_y ) )

        # Calculate bounds
        start_x = max(0, center_x - radius)
        start_y = max(0, center_y - radius)
        end_x = min(self.pixels_total, center_x + radius)
        end_y = min(self.pixels_total, center_y + radius)

        # Handle Clipping
        stamp_start_x = 0 if start_x == center_x - radius else (radius - (center_x - start_x))
        stamp_start_y = 0 if start_y == center_y - radius else (radius - (center_y - start_y))
        stamp_end_x = stamp_start_x + (end_x - start_x)
        stamp_end_y = stamp_start_y + (end_y - start_y)

        # Use consistent [y, x] indexing (Row=North, Col=East)
        self.grid[start_y:end_y, start_x:end_x] += heat_map[stamp_start_y:stamp_end_y, stamp_start_x:stamp_end_x]

    def get_view(self, drone_x, drone_y, height, resolution=32):
        px_x = int(drone_x * self.scale)
        px_y = int(drone_y * self.scale)
        
        Sensor_resolution = resolution
        
        # Clamp zoom factor
        zoom_factor = int(max(1, 2 * height))
        
        slice_size = Sensor_resolution * zoom_factor
        half_slice = slice_size // 2

        x_min = max(0, px_x - half_slice)
        x_max = min(self.pixels_total, px_x + half_slice)
        y_min = max(0, px_y - half_slice)
        y_max = min(self.pixels_total, px_y + half_slice)

        raw_view = self.grid[y_min:y_max, x_min:x_max]

        # Improved Padding: If we hit an edge, don't pad with flat 22.0.
        # Pad with random noise to prevent edge detection artifacts.
        if raw_view.shape != (slice_size, slice_size):
            h, w = raw_view.shape
            padded = np.full((slice_size, slice_size), 22.0)
            noise_pad = np.random.normal(0, 0.6, (slice_size, slice_size))
            padded += noise_pad
            
            padded[:h, :w] = raw_view
            raw_view = padded

        view_resized = raw_view.reshape(Sensor_resolution, zoom_factor, Sensor_resolution, zoom_factor).mean(axis=(1, 3))
        return view_resized
