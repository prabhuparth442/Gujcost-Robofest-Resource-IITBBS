import numpy as np
import cv2
import time

class CameraHandler:
    def __init__(self, camera_instance):
        self.cam = camera_instance
        print("[Driver] Connected to Virtual Camera.")

    def calibrate(self, frames=30):
        """
        Captures 30 frames to learn the 'Fixed Pattern Noise' (FPN).
        Call this ONCE before the mission starts.
        """
        print("[Camera] Calibrating Sensor Noise... (Please wait)")
        stack = []
        for _ in range(frames):
            res = self.cam.read()
            if res is not None and res[0] is not None:
                # We use the noisy image to learn the noise
                stack.append(res[0])
            time.sleep(0.02)

        if len(stack) > 0:
            # Average the frames to isolate the static noise pattern
            avg_frame = np.mean(np.array(stack), axis=0)

            # Center the noise around 0 (so we don't shift the whole temp down)
            self.fpn_pattern = avg_frame - np.mean(avg_frame)
            print(f"[Camera] Calibration Complete. Noise Pattern Captured.")
        else:
            print("[Camera] Calibration Failed! No data.")
            self.fpn_pattern = np.zeros((24, 32))

    def capture_and_process_debug(self, duration=2.0):
        """
        Captures stack, SUBTRACTS NOISE, and returns debug data.
        """
        noisy_stack = []
        clean_stack = []
        start_time = time.time()

        # 1. CAPTURE LOOP
        while (time.time() - start_time) < duration:
            result = self.cam.read()

            if result is not None and result[0] is not None:
                raw_noisy, raw_truth = result

                # --- CRITICAL FIX: APPLY CALIBRATION ---
                # Subtract the Fixed Pattern Noise we learned earlier
                if self.fpn_pattern is not None:
                    corrected_frame = raw_noisy - self.fpn_pattern
                else:
                    corrected_frame = raw_noisy

                # Store the CORRECTED frame in the stack
                noisy_stack.append(corrected_frame)

                # Store truth for comparison
                clean_stack.append(raw_truth)

            time.sleep(0.02)

        # 2. PROCESS (STACKING)
        if len(noisy_stack) > 0:
            # A. Raw Sample (For debug, we return the very first uncorrected frame)
            # We want to see how bad it was BEFORE we fixed it.
            # We need to grab a raw frame from the camera again or cache it,
            # but since 'noisy_stack' is already corrected, let's just return the
            # first corrected frame as 'raw' or we can accept the corrected one is what we process.
            # Actually, let's return the Stacked result.

            # B. The Stacked Result (Averaging the CORRECTED frames)
            # This kills the random noise (NETD)
            processed_result = np.mean(np.array(noisy_stack), axis=0)

            # C. Ground Truth
            ground_truth = np.mean(np.array(clean_stack), axis=0)

            # D. Raw Snapshot (reconstructing the noisy one for visualization)
            # This is just for the "Raw" plot in the graph
            raw_snapshot = processed_result + self.fpn_pattern if self.fpn_pattern is not None else processed_result

            return processed_result, 1, raw_snapshot, ground_truth
        else:
            return None, 0, None, None



class VisionProcessor:
    def __init__(self):
        self.upscale_factor = 4
        self.sigma_threshold = 1.5  # Sensitivity (Keep this low/1.5 as it works)

        # AREA FILTER (Keep this to ignore single-pixel static)
        self.min_blob_area = 3
        self.max_blob_area = 500    # Increased max just in case

    def process(self, thermal_frame):
        if thermal_frame is None: return [], None, None

        # 1. Upscale
        upscaled = cv2.resize(thermal_frame, (128, 96), interpolation=cv2.INTER_CUBIC)

        # 2. Denoise
        upscaled = cv2.bilateralFilter(upscaled.astype(np.float32), 5, 50, 50)

        # 3. Safety Check
        vmin, vmax = np.min(upscaled), np.max(upscaled)
        if (vmax - vmin) < 0.2:
            blank = np.zeros_like(upscaled, dtype=np.uint8)
            return [], blank, blank

        # 4. Normalize & Threshold
        norm_img = cv2.normalize(upscaled, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        mean_bg = np.mean(norm_img)
        std_bg  = np.std(norm_img)

        # Using the settings that successfully showed you the white blobs
        noise_floor = max(std_bg * self.sigma_threshold, 4.0)
        thresh_val = mean_bg + noise_floor

        _, binary_map = cv2.threshold(norm_img, thresh_val, 255, cv2.THRESH_BINARY)

        # 5. BLOB DETECTION (Shape Filters REMOVED)
        contours, _ = cv2.findContours(binary_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        for cnt in contours:
            area = cv2.contourArea(cnt)

            # --- ONLY CHECK AREA ---
            # We removed Aspect Ratio and Solidity checks.
            # If it's big enough to be real, we take it.
            if self.min_blob_area <= area <= self.max_blob_area:
                x, y, w, h = cv2.boundingRect(cnt)

                # Calculate Center
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    detections.append({"x_px": cX, "y_px": cY, "area": area})

        return detections, norm_img, binary_map
