import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
collections.Iterable = collections.abc.Iterable

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import numpy as np
import cv2
from field_simulation import Thermal_map
import random

# --- CONFIGURATION ---
SEARCH_ALT = 2.0
DESCEND_ALT = 0.5
HEAT_THRESH = 29.0
FIELD_LENGTH = 40       
CAMERA_RES = 64         
FOV_AT_2M = 2.0
SWEEP_STEP = 2.0        # Shift 2m sideways for next pass

# --- 1. SETUP ---
print("Connecting...")
vehicle = connect('127.0.0.1:14551', wait_ready=True)

print("Generating Map...")
# Add Random Mines & Rocks back
sim = Thermal_map(seed=42, n_rocks=30) 

# Add 15 Random Mines
for i in range(15):
    random.seed(i*99) # Consistent seed per mine
    # Keep mines within [2, 38] to avoid edge clipping issues
    sim.add_mine(int(random.uniform(2,38)), int(random.uniform(2,38)))

# Ensure one mine is on our first path for immediate gratification
# Path starts at East=20 (Sim X=40) and sweeps West.
# We place a mine at East=18 (Sim X=38), North=30 (Sim Y=30) so it hits it soon.
sim.add_mine(38, 30) 

confirmed_mines = []

# --- 2. CORE MATH FUNCTIONS ---

def get_relative_meters(home, current):
    """ Returns (dNorth, dEast) relative to Home. """
    dLat = current.lat - home.lat
    dLon = current.lon - home.lon
    dNorth = dLat * 111319.5
    dEast = dLon * 111319.5 * math.cos(math.radians(home.lat))
    return dNorth, dEast

def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
    return LocationGlobalRelative(original_location.lat + (dLat * 180/math.pi),
                                  original_location.lon + (dLon * 180/math.pi),
                                  original_location.alt)

def get_dist(loc1, loc2):
    return math.sqrt(((loc1.lat-loc2.lat)*111319.5)**2 + ((loc1.lon-loc2.lon)*111319.5 * math.cos(math.radians(loc1.lat)))**2)

# --- 3. CV LOGIC ---

def get_pixel_scale(altitude):
    # Scale = Meters per pixel
    return (FOV_AT_2M / CAMERA_RES) * (max(0.1, altitude) / 2.0)

def find_blob_offset(thermal_image, current_alt):
    """ Calculates meters to move to center the target """
    y_idx, x_idx = np.unravel_index(np.argmax(thermal_image), thermal_image.shape)
    
    center = CAMERA_RES / 2
    px_dx = x_idx - center
    
    # Y Axis: Index increases downwards (North in our map view, if viewing as image)
    # But wait, in get_view: raw_view = grid[y_min:y_max]
    # y_min is South. y_max is North.
    # So index 0 is South. Index Max is North.
    # If target is at Index Max (North), we want to move North (+).
    # So (y_idx - center) is positive. Correct.
    px_dy = y_idx - center 
    
    scale = get_pixel_scale(current_alt)
    return px_dy * scale, px_dx * scale

def check_hough_circle(thermal_image):
    norm_img = np.clip(thermal_image, 22.0, 35.0)
    norm_img = (norm_img - 22.0) * (255.0 / (35.0 - 22.0))
    norm_img = np.uint8(norm_img)
    
    blurred = cv2.GaussianBlur(norm_img, (5, 5), 0)
    
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=10,
                               param1=50, param2=15, minRadius=4, maxRadius=24)
    return circles is not None

def arm_and_takeoff(aTargetAltitude):
    print("Pre-arm checks...")
    while not vehicle.is_armable: time.sleep(1)
    print("Arming...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed: time.sleep(1)
    print("Taking off...")
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95: break
        time.sleep(1)

def simple_move(target):
    vehicle.simple_goto(target)
    start_time = time.time()
    while True:
        # Check distance (Tolerance 1.0m)
        if get_dist(vehicle.location.global_relative_frame, target) < 1.0: 
            break
        # Timeout after 15 seconds
        if time.time() - start_time > 15:
            print("   (Move Timeout - Continuing)")
            break
        time.sleep(0.5)

# ==========================================
# MAIN MISSION
# ==========================================

arm_and_takeoff(SEARCH_ALT)
time.sleep(2)

home_loc = vehicle.location.global_relative_frame

# --- LAWNMOWER PATTERN SETUP ---
# Start at East Edge (+20m relative to Home)
# Home is at Sim(20, 20).
# East Edge is Sim X=40 (East=+20).
current_lane_x = 20.0  # Start at East Edge
sweep_direction = 1    # 1 = North, -1 = South
limit_west = -20.0     # Stop when we reach West Edge (Sim X=0, East=-20)

print(f"Moving to Start Position (South-East Corner: {current_lane_x}m East)...")
# Start at South Edge (-20m North) of the East-most lane
start_pt = get_location_metres(home_loc, -20, current_lane_x)
vehicle.groundspeed = 3.0
simple_move(start_pt)

print("Starting Search Sweep...")
vehicle.groundspeed = 0.5

while True:
    # 1. Define End Point for this lane
    target_north = 20.0 if sweep_direction == 1 else -20.0
    lane_target = get_location_metres(home_loc, target_north, current_lane_x)
    
    vehicle.simple_goto(lane_target)
    print(f"--> Sweeping Lane at East={current_lane_x}m towards North={target_north}m")

    # 2. Flight Loop for this Lane
    while True:
        current_pos = vehicle.location.global_relative_frame
        
        # Check if reached end of lane
        if get_dist(current_pos, lane_target) < 1.0:
            print("   Lane Finished.")
            break

        # --- MAPPING & SENSOR ---
        dN, dE = get_relative_meters(home_loc, current_pos)
        
        # Home(0,0) = Sim(20, 20)
        sim_y = dN + 20.0
        sim_x = dE + 20.0 
        
        if 0 <= sim_x <= 40 and 0 <= sim_y <= 40:
            view = sim.get_view(sim_x, sim_y, height=current_pos.alt, resolution=CAMERA_RES)
            max_t = np.max(view)
            
            if max_t > HEAT_THRESH:
                print(f">>> HEAT ({max_t:.1f}C) at Map Y={sim_y:.1f}m, X={sim_x:.1f}m")
                vehicle.mode = VehicleMode("GUIDED")
                saved_spot = vehicle.location.global_relative_frame
                
                # 1. Center
                n_off, e_off = find_blob_offset(view, current_pos.alt)
                target_center = get_location_metres(current_pos, n_off, e_off)
                simple_move(target_center)
                
                # 2. Descend
                descend_loc = LocationGlobalRelative(target_center.lat, target_center.lon, DESCEND_ALT)
                simple_move(descend_loc)
                time.sleep(1)
                
                # 3. Verify
                dN_low, dE_low = get_relative_meters(home_loc, vehicle.location.global_relative_frame)
                low_view = sim.get_view(dE_low + 20.0, dN_low + 20.0, height=DESCEND_ALT, resolution=CAMERA_RES)
                
                if check_hough_circle(low_view):
                    print(f"   [+] MINE CONFIRMED at {sim_y:.1f}m North")
                    confirmed_mines.append((sim_y, sim_x))
                else:
                    print("   [-] False Alarm")
                
                # 4. Recovery
                print("   Returning to search path...")
                climb_loc = LocationGlobalRelative(vehicle.location.global_relative_frame.lat, 
                                                 vehicle.location.global_relative_frame.lon, 
                                                 SEARCH_ALT)
                simple_move(climb_loc)
                simple_move(saved_spot)
                
                # Nudge forward in direction of sweep
                nudge_dist = 3.0 * sweep_direction # +3 if North, -3 if South
                print(f"   Nudging {nudge_dist}m...")
                nudge_pt = get_location_metres(saved_spot, nudge_dist, 0)
                simple_move(nudge_pt)
                
                print("   Resuming...")
                vehicle.simple_goto(lane_target)
                
        time.sleep(0.2)

    # 3. Lane Done - Shift West (Left)
    current_lane_x -= SWEEP_STEP # Move West by 2m
    
    # Check if we hit the edge of the map
    if current_lane_x < limit_west:
        print("All Lanes Complete.")
        break
        
    # Shift Drone
    print(f"Shifting Left to East={current_lane_x}m...")
    next_start_pt = get_location_metres(home_loc, target_north, current_lane_x)
    simple_move(next_start_pt)
    
    # Flip direction for next pass
    sweep_direction *= -1

print(f"Mines Found: {len(confirmed_mines)}")
print(confirmed_mines)
vehicle.mode = VehicleMode("RTL")
