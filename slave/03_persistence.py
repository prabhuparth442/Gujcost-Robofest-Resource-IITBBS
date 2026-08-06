#!/usr/bin/env python3
"""
03_persistence.py  —  Detection Persistence / Confirmation Gate
===============================================================
Pipeline step 3 of 6.  Called by main_orchestrator_competition.py
AFTER the initial vision filter fires.

WHY this step exists
--------------------
The thermal vision filter (02 or 06) spots a suspicious blob in a
rolling window of frames.  But rocks, wet patches, and sensor noise can
all produce momentary blobs that look like mines.

The persistence check works by making the drone hover directly over the
candidate position and re-capturing a fresh frame stack.  Because the
drone is now centred above the target, the blob should appear very close
to (dx=0, dy=0) in the thermal image — within the GPS/wind drift tolerance.

If it's not close to centre, it means the original blob was GPS drift or
a false positive from a different physical location.

Coordinate math reminder
------------------------
dx, dy are pixel offsets from the 640×480 frame centre.
Positive dx = target is to the RIGHT  of the drone camera centre.
Positive dy = target is BELOW          (OpenCV convention: Y grows downward).

Physical drift is computed using the same focal-length constants as
04_coordinate_math.py:  fx=614.5  fy=761.2  (55°×35° FOV at 640×480 px).

Called from
-----------
    main_orchestrator_competition.py → _handle_candidate():
        ok = persistence.PersistenceFilter(max_drift_meters=1.5).verify(
            dx, dy, candidate.altitude)
        if not ok:
            return  # ghost — discard, continue scanning
"""
import math

class PersistenceFilter:
    def __init__(self, max_drift_meters=1.5):
        # Allowable error radius from dead center. 
        # 1.5 meters accounts for GPS inaccuracy and wind drift during hover.
        self.max_drift_meters = max_drift_meters

    def verify(self, new_dx, new_dy, altitude_m, fx=614.5, fy=761.2):
        """
        Since the drone moved over the target, it should be at dead center (dx=0, dy=0).
        This verifies the new anomaly is physically close enough to the center to be the same target.

        fx=614.5, fy=761.2 match 04_coordinate_math.py (55°×35° FOV at 640×480).
        Old defaults were fx=fy=800 which over-estimated drift by ~23%, causing
        real mines to be rejected as "too far off centre".
        """
        if new_dx is None or new_dy is None:
            print("[PERSISTENCE] REJECTED: Target vanished completely.")
            return False

        # Calculate how far the target is from the dead center (0, 0) in pixels
        pixel_distance = math.sqrt(new_dx**2 + new_dy**2)
        
        # Convert that pixel error into physical meters using similar triangles
        error_x_m = (new_dx * altitude_m) / fx
        error_y_m = (new_dy * altitude_m) / fy
        
        # Total physical drift in meters from the drone's center axis
        physical_drift = math.sqrt(error_x_m**2 + error_y_m**2)

        print(f"[PERSISTENCE] Target offset from center: {physical_drift:.2f}m (Limit: {self.max_drift_meters}m)")

        if physical_drift <= self.max_drift_meters:
            print("[PERSISTENCE] CONFIRMED: Target successfully tracked through movement.")
            return True
        else:
            print("[PERSISTENCE] REJECTED: Anomaly is too far from center. Likely a different rock.")
            return False

# Example usage for testing
if __name__ == "__main__":
    p_filter = PersistenceFilter(max_drift_meters=1.5)
    # Target is exactly in the center (0, 0) at 5 meters high -> Should Confirm
    p_filter.verify(0, 0, altitude_m=5.0) 
    
    # Target is way off at the edge of the screen (300, 200) -> Should Reject
    p_filter.verify(300, 200, altitude_m=5.0)
