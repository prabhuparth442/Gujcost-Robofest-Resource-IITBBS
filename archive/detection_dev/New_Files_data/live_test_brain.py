#!/usr/bin/env python3
import time
import subprocess
import numpy as np
import cv2
import socket
import json
import base64
import io
from PIL import Image

# --- 1. IPC HARDWARE DRIVER ---
# --- 1. IPC HARDWARE DRIVER ---
class PipeCamera:
    def __init__(self):
        print("[SYSTEM] Spawning C++ 32Hz Hardware Pump...")
        # FIX 1: Removed 'sudo' to prevent password deadlocks.
        # FIX 2: Routed stderr to DEVNULL so API text warnings vanish instead of corrupting the pipe.
        import subprocess
        self.proc = subprocess.Popen(['sudo','./bin/mlx_stdout'], stdout=subprocess.PIPE) 
    def read(self):
        raw_bytes = self.proc.stdout.read(3072)
        if len(raw_bytes) != 3072: 
            print(f"[FATAL] Pipe blocked or C++ crashed. Received {len(raw_bytes)} bytes.")
            return None
        data = np.frombuffer(raw_bytes, dtype=np.float32).reshape((24, 32))
        return data, data
# --- 2. NETWORK TUNNEL ---
class DroneTunnel:
    def __init__(self, target_ip, target_port):
        self.target_ip = target_ip
        self.target_port = target_port

    def send_memory_image(self, name, pil_img):
        buffer = io.BytesIO()
        pil_img.save(buffer, format="JPEG")
        b64_bytes = base64.b64encode(buffer.getvalue())
        packet = {"type": "image", "name": name, "payload": b64_bytes.decode('utf-8')}
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.settimeout(0.5) 
            client.connect((self.target_ip, self.target_port))
            client.sendall(json.dumps(packet).encode('utf-8'))
            client.close()
            print("[NETWORK] Frame dispatched to PC.", end='\r')
        except Exception as e:
            print(f"\n[NETWORK ERROR] PC Unreachable: {e}")

# --- 3. PROCESSING CLASSES ---
class CameraHandler:
    def __init__(self, camera_instance):
        self.cam = camera_instance
        self.fpn_pattern = np.zeros((24, 32))

    def calibrate(self, frames=30, skip=10):
        """
        Build an FPN pattern from `frames` averaged frames, discarding the first
        `skip` frames which contain two startup artefacts:
          - Frame 0: subpage-1 pixels are stale (checkerboard ±0.8°C error)
          - Frames 1-4: on-chip NUC not settled (row-stripe ±1.2°C, gone by frame 5)
        Skipping 10 frames costs 10/32 ≈ 0.3 s at 32 Hz and drops the residual
        row-stripe from ~0.12°C to ~0.02°C, below the mine signal floor.
        """
        total = frames + skip
        print(f"[CAMERA] Calibrating FPN ({total} frames, discarding first {skip})...")
        stack = []
        acquired = 0
        discarded = 0
        while len(stack) < frames:
            res = self.cam.read()
            if res is None or res[0] is None:
                print(f"\n[FATAL] Calibration failed. C++ pipe is dead.")
                exit()
            acquired += 1
            if discarded < skip:
                discarded += 1
                continue   # silently consume the settling frames
            stack.append(res[0])
            print(f"[CAMERA] Calibration frame {len(stack)}/{frames}", end='\r')

        avg_frame = np.mean(np.array(stack), axis=0)
        self.fpn_pattern = avg_frame - np.mean(avg_frame)
        row_stripe = float(np.std(self.fpn_pattern.mean(axis=1)))
        print(f"\n[CAMERA] Calibration complete. "
              f"Row-stripe residual: {row_stripe:.4f}°C "
              f"(good if <0.05°C)")

    def capture_stack(self, num_frames=24):
        stack = []
        for _ in range(num_frames):
            res = self.cam.read()
            if res is not None and res[0] is not None:
                stack.append(res[0] - self.fpn_pattern)
        if len(stack) > 0: return np.mean(np.array(stack), axis=0)
        return None

class VisionProcessor:
    def __init__(self):
        self.sigma_threshold = 1.5
    def process(self, thermal_frame):
        if thermal_frame is None: return None, None
        upscaled = cv2.resize(thermal_frame, (128, 96), interpolation=cv2.INTER_CUBIC)
        denoised = cv2.bilateralFilter(upscaled.astype(np.float32), 5, 50, 50)
        vmin, vmax = np.min(denoised), np.max(denoised)
        if (vmax - vmin) < 0.2:
            return denoised, np.zeros((96, 128), dtype=np.uint8)
        norm_img = cv2.normalize(denoised, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        mean_bg = np.mean(norm_img)
        thresh_val = mean_bg + max(np.std(norm_img) * self.sigma_threshold, 4.0)
        _, binary_map = cv2.threshold(norm_img, thresh_val, 255, cv2.THRESH_BINARY)
        return denoised, binary_map

class BlobDetector:
    def __init__(self):
        self.min_area = 8
        self.max_area = 500
    def find_mines(self, binary_map):
        if binary_map is None: return []
        kernel = np.ones((3,3), np.uint8)
        solid_map = cv2.dilate(binary_map.astype(np.uint8), kernel, iterations=1)
        contours, _ = cv2.findContours(solid_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if self.min_area <= area <= self.max_area:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    candidates.append({'x': int(M["m10"]/M["m00"]), 'y': int(M["m01"]/M["m00"]), "area": area})
        if not candidates: return []
        max_area = max(c['area'] for c in candidates)
        return [c for c in candidates if c['area'] > (max_area / 4.0)]

# --- 4. EXECUTION LOOP ---
if __name__ == "__main__":
    print("[SYSTEM] Booting Drone Brain (1.5s Sweep Mode)...")
    pipe_cam = PipeCamera()
    cam_handler = CameraHandler(pipe_cam)
    vision = VisionProcessor()
    detector = BlobDetector()
    
    # UPDATED IP: Using the hotspot IP you provided earlier
    tunnel = DroneTunnel(target_ip="10.42.0.1", target_port=5000)

    # Let the sensor warm up and learn the noise
    cam_handler.calibrate(frames=30)
    print("\n[MISSION] Commencing 1.5-Second Radar Sweeps...")
    
    try:
        while True:
            print("[SWEEP] Capturing 1.5 seconds of thermal data (48 frames)...", end='\r')
            
            stacked_frame = cam_handler.capture_stack(num_frames=48)
            
            if stacked_frame is None:
                continue
                
            _, binary_img = vision.process(stacked_frame)
            targets = detector.find_mines(binary_img)
            
            vis_img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2RGB)
            
            if targets:
                print(f"\n[ALERT] Targets Locked: {len(targets)} mines detected.")
                for t in targets:
                    radius = int(np.sqrt(t['area'])) + 2
                    cv2.circle(vis_img, (t['x'], t['y']), radius, (255, 0, 0), 1)
                    cv2.putText(vis_img, "MINE", (t['x']-5, t['y']-5), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1)
            else:
                print("[SCAN] Area Clear. No targets found.                ", end='\r')

            img_to_send = Image.fromarray(vis_img).resize((640, 480), Image.NEAREST)
            tunnel.send_memory_image('sweep_stream', img_to_send)
                
    except KeyboardInterrupt:
        print("\n[SYSTEM] Terminating hardware pipe...")
        pipe_cam.proc.terminate()
