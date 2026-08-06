#!/usr/bin/env python3
"""
04_coordinate_math.py  —  Pixel Offset → GPS Coordinate
=========================================================
Pipeline step 4 of 6.  Converts a thermal-camera pixel offset (from
vision filter) into a real-world GPS latitude/longitude of the mine.

Two functions, called in sequence by main_orchestrator_competition.py:

  Step A:  get_pixels_to_meters(dx, dy, altitude_m)
               Pixel offset in 640×480 frame
               → Physical offset in metres relative to drone body frame
               Uses pinhole camera projection with MLX90640 FOV constants.

  Step B:  compute_global_gps(drone_lat, drone_lon, local_x_m, local_y_m)
               Body-frame metres + drone's locked heading
               → Absolute GPS (lat, lon) of the mine
               Reads heading from config/origin_state.json (written by 00_preflight_calib.py).

Coordinate conventions used here
---------------------------------
  CAMERA / PIXEL space (OpenCV):
    Origin = top-left of 640×480 image
    +dx = right    −dx = left
    +dy = DOWN     −dy = up  ← OpenCV Y axis grows downward

  BODY frame (drone):
    +X = Right of drone body
    +Y = FORWARD  ← note: camera +dy (DOWN) maps to BACKWARD, so we invert dy

  GLOBAL NED frame:
    +North, +East  (standard navigation)

  GPS output:
    Absolute decimal degrees (lat, lon)

Called from
-----------
    main_orchestrator_competition.py → _handle_candidate():
        lx, ly = coord_math.get_pixels_to_meters(new_dx, new_dy, alt)
        tlat, tlon = coord_math.compute_global_gps(lat, lon, lx, ly)
"""
import math
import json
import os
from pathlib import Path

# Earth's radius in meters for GPS Haversine approximation
R_EARTH = 6378137.0

# Using absolute paths to prevent sudo environment bugs
BASE_DIR    = str(Path(__file__).resolve().parent)  # FIX 5: dynamic, was hardcoded
CONFIG_DIR  = os.path.join(BASE_DIR, "config")
ORIGIN_FILE = os.path.join(CONFIG_DIR, "origin_state.json")

def get_pixels_to_meters(dx_pixels, dy_pixels, altitude_m=5.0):
    """
    Converts OpenCV pixel offsets to physical meters relative to the drone's body.
    Assumes standard MLX90640 55x35 degree Field of View upscaled to 640x480.
    """
    # Optical Center Math for 55°x35° FOV at 640x480 resolution
    # fx = 320 / tan(27.5°) = 614.5
    # fy = 240 / tan(17.5°) = 761.2
    fx, fy = 614.5, 761.2
    
    # Similar triangles: physical_offset = (pixel_offset * altitude) / focal_length
    raw_x_m = (dx_pixels * altitude_m) / fx
    raw_y_m = (dy_pixels * altitude_m) / fy
    
    # BODY FRAME TRANSLATION:
    # +X is Right (matches +dx)
    # +Y is Forward (OpenCV +dy is DOWN, meaning physically BEHIND the drone. Must invert.)
    local_x_m = raw_x_m
    local_y_m = -raw_y_m
    
    return local_x_m, local_y_m

def compute_global_gps(drone_lat, drone_lon, local_x_m, local_y_m):
    """
    Rotates the local body coordinates to Global North/East using the drone's locked yaw,
    then translates those meters into GPS decimal degrees.
    """
    try:
        with open(ORIGIN_FILE, 'r') as f:
            origin = json.load(f)
        yaw_rad = origin.get("locked_yaw_rad", 0.0)
    except Exception:
        print("[MATH] Warning: No origin file found. Assuming Yaw=0.0 (Facing True North).")
        yaw_rad = 0.0

    # 1. 2D Rotation Matrix: Body Frame (+Y Forward, +X Right) to NED Frame
    delta_north = (local_y_m * math.cos(yaw_rad)) - (local_x_m * math.sin(yaw_rad))
    delta_east  = (local_x_m * math.cos(yaw_rad)) + (local_y_m * math.sin(yaw_rad))
    
    # 2. Convert physical meters to GPS decimal degrees
    # Latitude scales directly with Earth's radius.
    delta_lat = (delta_north / R_EARTH) * (180.0 / math.pi)
    
    # Longitude shrinks as you move away from the equator, requiring a cosine compensation.
    lat_rad = drone_lat * (math.pi / 180.0)
    delta_lon = (delta_east / (R_EARTH * math.cos(lat_rad))) * (180.0 / math.pi)
    
    target_lat = drone_lat + delta_lat
    target_lon = drone_lon + delta_lon
    
    return target_lat, target_lon

# Example usage for testing
if __name__ == "__main__":
    # Simulate a target 100 pixels Right and 100 pixels Down from center at 5m hover
    test_lx, test_ly = get_pixels_to_meters(100, 100, altitude_m=5.0)
    print(f"Local Offsets: Right {test_lx:.2f}m, Forward {test_ly:.2f}m")
    
    # Translate to GPS assuming facing East (Yaw = 90 deg = 1.5708 rad)
    # The target is behind and to the right, so it should move South and West.
    t_lat, t_lon = compute_global_gps(20.296, 85.824, test_lx, test_ly)
    print(f"Target GPS: {t_lat:.6f}, {t_lon:.6f}")
