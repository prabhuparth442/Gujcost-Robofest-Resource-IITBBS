#!/usr/bin/env python3
import sys
import os
import time
import threading
import subprocess
import importlib
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Module Imports
vision      = importlib.import_module("vision.02_vision_filter")
coord_math  = importlib.import_module("logic.04_coordinate_math")
verifier    = importlib.import_module("logic.05_map_verifier")
comms       = importlib.import_module("logic.08_comms_link")
movement    = importlib.import_module("logic.07_movement")
persistence = importlib.import_module("logic.03_persistence")

# CHANGED 0.80 → 0.40 for new vision filter (v2).
# Old filter scored confidence as circularity × 0.95 which reached 0.9+.
# New filter uses composite scoring (circ + size_fit + delta_strength)
# which gives confirmed mine detections a confidence of 0.49–0.65.
# 0.40 is the new validated threshold; anything below is noise.
CONFIDENCE_THRESHOLD = 0.40
scanned_mines = []

# --- 1. IN-MEMORY HARDWARE PIPELINE ---
class PipeCamera:
    """Runs C++ binary in background and pipes float arrays directly to RAM."""
    def __init__(self):
        print("[SYSTEM] Booting C++ 32Hz IPC Pipeline...")
        # FIX 1: Point to the compiled C++ binary, not the Python stub.
        self.proc = subprocess.Popen(
            ['/home/drone3/drone_swarm/bin/mlx_stdout'],
            stdout=subprocess.PIPE,
            stderr=sys.stderr
        )

    def read_frame(self):
        """Reads exactly one 768-float frame (3072 bytes) from the C++ pipe."""
        raw_bytes = self.proc.stdout.read(3072)
        if len(raw_bytes) != 3072:
            return None
        return np.frombuffer(raw_bytes, dtype=np.float32).reshape((24, 32))

    def capture_stack(self, num_frames=48):
        """Blocks until num_frames valid frames are collected, then returns the stack."""
        stack = []
        for _ in range(num_frames):
            frame = self.read_frame()
            if frame is not None:
                stack.append(frame)
        if len(stack) > 0:
            return np.array(stack)
        return None


# --- 2. THREAD B: FINAL VERIFICATION & COMMS ---
# FIX 2: Added raw_stack as a proper parameter — it was referenced but never passed in.
def thread_b_final_logging(target_lat, target_lon, final_conf, raw_stack, tunnel):
    print("[THREAD B] Executing Final Verifier Module...")
    is_valid = verifier.verify_and_log(target_lat, target_lon, final_conf)

    if is_valid:
        scanned_mines.append((target_lat, target_lon))
        # FIX 3: raw_stack is now correctly in scope via the function parameter.
        tunnel.send_anomaly_data(target_lat, target_lon, raw_stack)


# --- 3. MAIN STATE MACHINE ---
def run_mission():
    tunnel = comms.DroneTunnel()
    cam = PipeCamera()

    try:
        # Load FPN Pattern into memory once at startup
        fpn_path = "/home/drone3/drone_swarm/config/fpn_pattern.npy"
        fpn_pattern = np.load(fpn_path) if os.path.exists(fpn_path) else None
        if fpn_pattern is None:
            print("[WARN] No FPN pattern found. Run preflight calibration first.")

        movement.takeoff_to_hover(altitude=1.5)

        print("\n========================================")
        print("      STARTING ASYNC SECTOR PATROL      ")
        print("========================================\n")

        for sector in range(1, 10):
            print(f"\n[NAV] Scanning Sector {sector}...")

            # STATE 1: DISCRETE CAPTURE (HOME POSITION)
            # 48 frames @ 32Hz = 1.5 seconds of thermal data
            home_stack = cam.capture_stack(num_frames=48)
            if home_stack is None:
                print("[ERROR] Camera pipe returned no frames. Skipping sector.")
                continue

            # STATE 2: INITIAL DETECTION
            dx, dy, conf = vision.process_memory_stack(home_stack, fpn_pattern)

            if dx is not None and conf >= CONFIDENCE_THRESHOLD:
                print(f"[LOGIC] High-probability target detected (conf={conf*100:.1f}%). Committing to move...")

                # STATE 3: CALCULATE PHYSICAL TARGET COORDINATES
                drone_lat, drone_lon, alt = movement.get_current_telemetry()
                local_x, local_y = coord_math.get_pixels_to_meters(dx, dy, alt)
                target_lat, target_lon = coord_math.compute_global_gps(
                    drone_lat, drone_lon, local_x, local_y
                )

                # STATE 4: FLY TO TARGET
                movement.move_to_coordinate(target_lat, target_lon)
                movement.force_hover(1.0)

                # STATE 5: DISCRETE CAPTURE (OVER TARGET)
                target_stack = cam.capture_stack(num_frames=48)
                if target_stack is None:
                    print("[ERROR] Camera pipe failed over target. Returning to sector.")
                    movement.return_to_sector_center()
                    continue

                # STATE 6: PERSISTENCE CHECK
                print("[LOGIC] Confirming target persistence...")
                new_dx, new_dy, final_conf = vision.process_memory_stack(target_stack, fpn_pattern)

                p_filter = persistence.PersistenceFilter(max_drift_meters=1.5)
                # fx=614.5, fy=761.2 matches 04_coordinate_math.py (55°×35° FOV at 640×480)
                # The old PersistenceFilter defaulted to fx=fy=800 which was wrong
                is_persistent = p_filter.verify(new_dx, new_dy, alt,
                                                 fx=614.5, fy=761.2)

                # Re-run the filter just for the probe (zero cost, same logic)
                # extract_solid_targets now returns (binary_map, avg_delta) tuple
                try:
                    sector_binary, _ = vision.SpatiotemporalFilter().extract_solid_targets(
                        home_stack, fpn_pattern)
                except Exception:
                    sector_binary = None  # viewer will show blank binary slot

                if is_persistent and final_conf >= CONFIDENCE_THRESHOLD:
                    print("[LOGIC] Persistence confirmed. Forking threads.")

                    verification_thread = threading.Thread(
                        target=thread_b_final_logging,
                        args=(target_lat, target_lon, final_conf, target_stack, tunnel)
                    )
                    verification_thread.daemon = True
                    verification_thread.start()

                    # Send confirmed mine result to PC viewer
                    tunnel.send_sector_result(
                        sector_id=sector, raw_stack=home_stack,
                        binary_map=sector_binary, mine_found=True,
                        dx=dx, dy=dy, conf=final_conf,
                        mine_lat=target_lat, mine_lon=target_lon
                    )

                    print("[THREAD A] Returning to sector center...")
                    movement.return_to_sector_center()
                    verification_thread.join()

                else:
                    print("[LOGIC] Persistence failed. Ghost anomaly dropped.")
                    # Send ghost-rejected result to PC viewer
                    tunnel.send_sector_result(
                        sector_id=sector, raw_stack=home_stack,
                        binary_map=sector_binary, mine_found=False,
                        dx=dx, dy=dy, conf=conf
                    )
                    movement.return_to_sector_center()

            elif dx is not None:
                print(f"[LOGIC] Target below confidence threshold ({conf*100:.1f}%). Ignoring.")
                tunnel.send_sector_result(sector_id=sector, raw_stack=home_stack, mine_found=False, conf=conf)
            else:
                print("[SCAN] Sector clear. No targets found.")
                tunnel.send_sector_result(sector_id=sector, raw_stack=home_stack, mine_found=False)

            # Advance to next sector
            print("[NAV] Moving to next sector...")
            movement.move_to_next_sector(overlap_factor=0.95)

    except KeyboardInterrupt:
        print("\n[SYSTEM] Mission aborted by operator.")
    finally:
        print("[SYSTEM] Terminating C++ camera pipe...")
        cam.proc.terminate()
        cam.proc.wait()


if __name__ == "__main__":
    run_mission()
