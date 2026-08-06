#!/usr/bin/env python3
"""
main_orchestrator.py — with DebugProbe injections at every pipeline stage.
Start the dashboard: python3 debug_visualizer.py --serve
Then run this file normally. Open http://<drone-ip>:8765 on any browser.
Remove the probe.* lines for production — everything else is identical.
"""
import sys
import os
import time
import threading
import subprocess
import importlib
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

vision      = importlib.import_module("vision.02_vision_filter")
coord_math  = importlib.import_module("logic.04_coordinate_math")
verifier    = importlib.import_module("logic.05_map_verifier")
comms       = importlib.import_module("logic.08_comms_link")
movement    = importlib.import_module("logic.07_movement")
persistence = importlib.import_module("logic.03_persistence")

# ── DEBUG: import and start the visualizer server ────────────────────────────
from debug_visualizer import DebugProbe, start_server
start_server(port=8765)   # Dashboard at http://<drone-ip>:8765
probe = DebugProbe()
# ─────────────────────────────────────────────────────────────────────────────

# CHANGED 0.80 → 0.40: new filter composite scoring gives mines 0.49–0.65
CONFIDENCE_THRESHOLD = 0.40
scanned_mines = []


class PipeCamera:
    def __init__(self):
        print("[SYSTEM] Booting C++ 32Hz IPC Pipeline...")
        self.proc = subprocess.Popen(
            ['/home/drone3/drone_swarm/bin/mlx_stdout'],
            stdout=subprocess.PIPE, stderr=sys.stderr
        )

    def read_frame(self):
        raw_bytes = self.proc.stdout.read(3072)
        if len(raw_bytes) != 3072:
            return None
        frame = np.frombuffer(raw_bytes, dtype=np.float32).reshape((24, 32))

        # ── PROBE 1: raw frame straight from C++ pipe ─────────────────────
        probe.raw_frame(frame)
        # ─────────────────────────────────────────────────────────────────

        return frame

    def capture_stack(self, num_frames=48):
        stack = []
        for _ in range(num_frames):
            frame = self.read_frame()
            if frame is not None:
                stack.append(frame)
        return np.array(stack) if stack else None


def thread_b_final_logging(target_lat, target_lon, final_conf, raw_stack, tunnel):
    print("[THREAD B] Executing Final Verifier Module...")
    is_valid = verifier.verify_and_log(target_lat, target_lon, final_conf)
    if is_valid:
        scanned_mines.append((target_lat, target_lon))
        try:
            tunnel.send_anomaly_data(target_lat, target_lon, raw_stack)
            # ── PROBE 8: comms broadcast succeeded ───────────────────────
            probe.comms(True, target_lat, target_lon)
            # ─────────────────────────────────────────────────────────────
        except Exception as e:
            # ── PROBE 8: comms broadcast failed ──────────────────────────
            probe.comms(False, error_msg=str(e))
            # ─────────────────────────────────────────────────────────────


def run_mission():
    tunnel = comms.DroneTunnel()
    cam = PipeCamera()

    try:
        fpn_path = "/home/drone3/drone_swarm/config/fpn_pattern.npy"
        fpn_pattern = np.load(fpn_path) if os.path.exists(fpn_path) else None
        if fpn_pattern is None:
            probe.log("preflight", "No FPN pattern found — run preflight first", "WARN")

        movement.takeoff_to_hover(altitude=1.5)

        for sector in range(1, 10):
            print(f"\n[NAV] Scanning Sector {sector}...")
            probe.log("nav", f"Entering sector {sector}", "INFO")

            # STATE 1: CAPTURE
            home_stack = cam.capture_stack(num_frames=48)
            if home_stack is None:
                probe.log("capture", "capture_stack returned None — pipe dead?", "ERROR")
                continue

            # ── PROBE 2: FPN-corrected view of the averaged frame ─────────
            if fpn_pattern is not None:
                probe.fpn_corrected(np.mean(home_stack, axis=0) - fpn_pattern)
            else:
                probe.fpn_corrected(np.mean(home_stack, axis=0))
            # ─────────────────────────────────────────────────────────────

            # STATE 2: DETECTION
            dx, dy, conf = vision.process_memory_stack(home_stack, fpn_pattern)

            # ── PROBE 3+4: binary map and detection result ────────────────
            # Re-run the filter just for the probe (zero cost, same logic)
            from vision import _02_vision_filter as vf_mod  # adjust import if needed
            try:
                st = vf_mod.SpatiotemporalFilter()
                bmap, _ = st.extract_solid_targets(home_stack, fpn_pattern)
                probe.binary_map(bmap)
                probe.detection(bmap, dx, dy, conf)
            except Exception:
                probe.log("detection",
                          f"dx={dx} dy={dy} conf={conf*100:.1f}%" if dx else "No target",
                          "OK" if dx else "INFO")
            # ─────────────────────────────────────────────────────────────

            if dx is not None and conf >= CONFIDENCE_THRESHOLD:

                # STATE 3: TELEMETRY + GPS MATH
                drone_lat, drone_lon, alt = movement.get_current_telemetry()

                # ── PROBE 5: telemetry ────────────────────────────────────
                probe.telemetry(drone_lat, drone_lon, alt)
                # ─────────────────────────────────────────────────────────

                local_x, local_y = coord_math.get_pixels_to_meters(dx, dy, alt)
                target_lat, target_lon = coord_math.compute_global_gps(
                    drone_lat, drone_lon, local_x, local_y
                )

                # ── PROBE 6: computed target GPS ──────────────────────────
                probe.target_gps(target_lat, target_lon, local_x, local_y)
                # ─────────────────────────────────────────────────────────

                # STATE 4: FLY
                movement.move_to_coordinate(target_lat, target_lon)
                movement.force_hover(1.0)

                # STATE 5: RE-CAPTURE OVER TARGET
                target_stack = cam.capture_stack(num_frames=48)
                if target_stack is None:
                    probe.log("capture", "target_stack returned None over target", "ERROR")
                    movement.return_to_sector_center()
                    continue

                # STATE 6: PERSISTENCE
                new_dx, new_dy, final_conf = vision.process_memory_stack(target_stack, fpn_pattern)
                p_filter = persistence.PersistenceFilter(max_drift_meters=1.5)
                # fx=614.5, fy=761.2 matches 04_coordinate_math.py (55°×35° at 640×480)
                is_persistent = p_filter.verify(new_dx, new_dy, alt,
                                                 fx=614.5, fy=761.2)

                # ── PROBE 7: persistence result ───────────────────────────
                from logic import _03_persistence as pers_mod
                try:
                    drift_m = 0.0
                    import math
                    if new_dx and new_dy:
                        fx, fy = 614.5, 761.2
                        drift_m = math.sqrt(
                            ((new_dx * alt) / fx) ** 2 +
                            ((new_dy * alt) / fy) ** 2
                        )
                    probe.persistence(is_persistent, drift_m)
                except Exception:
                    probe.log("persistence",
                              f"{'CONFIRMED' if is_persistent else 'REJECTED'}",
                              "OK" if is_persistent else "WARN")
                # ─────────────────────────────────────────────────────────

                if is_persistent and final_conf >= CONFIDENCE_THRESHOLD:
                    verification_thread = threading.Thread(
                        target=thread_b_final_logging,
                        args=(target_lat, target_lon, final_conf, target_stack, tunnel)
                    )
                    verification_thread.daemon = True
                    verification_thread.start()
                    movement.return_to_sector_center()
                    verification_thread.join()
                else:
                    probe.log("persistence", "Ghost anomaly dropped", "WARN")
                    movement.return_to_sector_center()

            elif dx is not None:
                probe.log("detection", f"Below threshold ({conf*100:.1f}% < 40%)", "WARN")
            else:
                probe.log("detection", f"Sector {sector} clear", "INFO")

            movement.move_to_next_sector(overlap_factor=0.95)

    except KeyboardInterrupt:
        print("\n[SYSTEM] Mission aborted.")
    finally:
        cam.proc.terminate()
        cam.proc.wait()


if __name__ == "__main__":
    run_mission()
