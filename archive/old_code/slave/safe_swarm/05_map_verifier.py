#!/usr/bin/env python3
import json
import math
import os
from pathlib import Path

# FIX 5: dynamic paths — works on drone1, drone2, drone3 (was /home/drone3/)
BASE_DIR         = str(Path(__file__).resolve().parent)
CONFIG_DIR       = os.path.join(BASE_DIR, "config")
LOG_DIR          = os.path.join(BASE_DIR, "logs")
A4_MAP_FILE      = os.path.join(CONFIG_DIR, "A4_map.json")
VIRTUAL_MAP_FILE = os.path.join(LOG_DIR, "virtual_map.json")

# Physical matching limits
KNOWN_MINE_RADIUS_M = 2.5    # How close it must be to A4 map intel to be considered "Known"
DEDUPLICATION_RADIUS_M = 1.5 # How close it must be to a previously scanned mine to merge them

os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(CONFIG_DIR, exist_ok=True)

def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculates the exact physical distance in meters between two GPS coordinates."""
    R = 6378137.0 # Earth radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def load_json_map(filepath, default_content):
    if not os.path.exists(filepath):
        with open(filepath, 'w') as f:
            json.dump(default_content, f, indent=4)
        return default_content
    with open(filepath, 'r') as f:
        return json.load(f)

def verify_and_log(target_lat, target_lon, confidence):
    print(f"\n[VERIFIER] Processing Target: {target_lat:.6f}, {target_lon:.6f} (Conf: {confidence*100:.1f}%)")
    
    a4_map = load_json_map(A4_MAP_FILE, {"mines": []})
    virtual_map = load_json_map(VIRTUAL_MAP_FILE, {"mines": []})
    
    # 1. DEDUPLICATION: Did we already find this exact mine during this flight?
    for idx, logged_mine in enumerate(virtual_map.get("mines", [])):
        dist_to_logged = haversine_distance(target_lat, target_lon, logged_mine["lat"], logged_mine["lon"])
        
        if dist_to_logged <= DEDUPLICATION_RADIUS_M:
            print(f"[VERIFIER] DEDUPLICATION: Target matches previously scanned mine ({dist_to_logged:.1f}m away).")
            
            # Coordinate Fusion: Average the coordinates for higher precision
            fused_lat = (logged_mine["lat"] + target_lat) / 2.0
            fused_lon = (logged_mine["lon"] + target_lon) / 2.0
            
            virtual_map["mines"][idx]["lat"] = fused_lat
            virtual_map["mines"][idx]["lon"] = fused_lon
            virtual_map["mines"][idx]["confidence"] = 1.0 # Confirmed twice, 100% confidence
            virtual_map["mines"][idx]["scans"] = virtual_map["mines"][idx].get("scans", 1) + 1
            
            with open(VIRTUAL_MAP_FILE, 'w') as f:
                json.dump(virtual_map, f, indent=4)
            
            # Return False so the orchestrator doesn't broadcast the same mine to the Master twice
            return False 

    # 2. INTEL CHECK: Is this a known mine from our prior A4 sheet?
    is_on_map = False
    for mine in a4_map.get("mines", []):
        dist = haversine_distance(target_lat, target_lon, mine["lat"], mine["lon"])
        if dist <= KNOWN_MINE_RADIUS_M:
            is_on_map = True
            print(f"[VERIFIER] INTEL MATCH: Target perfectly aligns with A4 map entity {mine.get('id', 'Unknown')}.")
            break

    # 3. LOGGING: It's a brand new valid target. Record it.
    target_type = "KNOWN_MINE" if is_on_map else "NEW_DISCOVERY"
    
    if not is_on_map:
        print("[VERIFIER] SELF-DETECTION: Unknown anomaly found! Trusting internal vision pipeline.")

    new_entry = {
        "lat": target_lat,
        "lon": target_lon,
        "confidence": confidence,
        "type": target_type,
        "scans": 1
    }
    
    virtual_map.setdefault("mines", []).append(new_entry)
    
    with open(VIRTUAL_MAP_FILE, 'w') as f:
        json.dump(virtual_map, f, indent=4)
        
    print(f"[VERIFIER] Logged as {target_type}. Triggering network broadcast.")
    return True

if __name__ == "__main__":
    # Test Haversine distance tracking
    print("Testing verifier logic...")
    verify_and_log(20.296000, 85.824000, 0.95) # Initial discovery
    verify_and_log(20.296005, 85.824005, 0.88) # Duplicate scan (should fuse coordinates)
