#!/usr/bin/env python3
"""
08_comms_link.py  —  TCP Mine Report Channel  (Slave → Master)
==============================================================
Pipeline step 6 of 6 (the final step).

What it does
------------
Sends confirmed mine detections from a slave drone to the master drone
over TCP (port 5000).  Two separate packet types are supported:

  send_anomaly_data(gps_lat, gps_lon, raw_frame_stack)
      ─ The primary inter-drone channel used by main_orchestrator_competition.py.
      ─ Sends a compact JSON packet with the mine GPS + a JPEG thermal image.
      ─ Master receives it at /api/drone_update and adds it to the mine list.

  send_sector_result(sector_id, raw_stack, binary_map, mine_found, ...)
      ─ Extended packet used by PC-side analysis tools (pc_sector_viewer.py).
      ─ Sends three visualisation images (raw thermal, binary mask, annotated).
      ─ Useful for debugging the vision pipeline on a laptop during testing.

Wire format
-----------
Every packet is length-prefixed:
    4 bytes  big-endian uint32 = total length of JSON body
    N bytes  UTF-8 JSON body

This prevents packet fragmentation on TCP: the receiver reads the 4-byte
header first, then reads exactly that many bytes for the JSON payload.

Why TCP instead of UDP?
-----------------------
Mine reports are critical events — we cannot afford to lose them.
Telemetry (positions, grid snapshots) uses UDP because occasional packet
loss is acceptable there.  Mine detections use TCP for guaranteed delivery.

Called from
-----------
    main_orchestrator_competition.py → _handle_candidate() → _report():
        tunnel.send_anomaly_data(tlat, tlon, restack)
"""
import socket
import json
import base64
import time
import cv2
import numpy as np

class DroneTunnel:
    def __init__(self, target_ip="10.42.0.1", target_port=5000):
        self.target_ip   = target_ip
        self.target_port = target_port
        self.drone_id    = "drone3"

    def _encode_img(self, img_bgr, quality=82):
        _, buf = cv2.imencode('.jpg', img_bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return base64.b64encode(buf).decode('utf-8')

    def _render_thermal(self, frame_2d, colormap=cv2.COLORMAP_INFERNO):
        vmin, vmax = np.min(frame_2d), np.max(frame_2d)
        if vmax > vmin:
            norm = ((frame_2d - vmin) / (vmax - vmin) * 255).astype(np.uint8)
        else:
            norm = np.zeros_like(frame_2d, dtype=np.uint8)
        up = cv2.resize(norm, (320, 240), interpolation=cv2.INTER_NEAREST)
        return cv2.applyColorMap(up, colormap)

    def _render_binary(self, binary_map_24x32):
        up = cv2.resize(binary_map_24x32, (320, 240), interpolation=cv2.INTER_AREA)
        return cv2.cvtColor(up, cv2.COLOR_GRAY2BGR)

    def _render_detection(self, frame_2d, dx, dy, conf):
        vis = self._render_thermal(frame_2d, cv2.COLORMAP_INFERNO)
        h, w = vis.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.drawMarker(vis, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 30, 1)
        if dx is not None and dy is not None:
            tx = max(8, min(w - 8, cx + dx // 2))
            ty = max(8, min(h - 8, cy + dy // 2))
            cv2.circle(vis, (tx, ty), 14, (0, 60, 255), 2)
            cv2.circle(vis, (tx, ty),  3, (0, 60, 255), -1)
            cv2.line(vis, (cx, cy), (tx, ty), (0, 200, 255), 1)
            cv2.putText(vis, f"MINE {conf*100:.0f}%", (tx + 6, ty - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 60, 255), 1)
        return vis

    def _transmit(self, packet):
        """Send JSON packet over TCP with a 4-byte length prefix."""
        raw = json.dumps(packet).encode('utf-8')
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Sector result packets with 3 images can be ~150-200KB.
        # Give enough time for WiFi to push the full payload.
        client.settimeout(8.0)
        try:
            client.connect((self.target_ip, self.target_port))
            client.sendall(len(raw).to_bytes(4, 'big'))
            client.sendall(raw)
            print(f"[COMMS] Sent {len(raw)/1024:.1f} KB")
        finally:
            client.close()

    # ── NEW: send full sector result to PC viewer ────────────────────────────
    def send_sector_result(self, sector_id,
                           raw_stack,
                           binary_map=None,
                           mine_found=False,
                           dx=None, dy=None, conf=0.0,
                           mine_lat=None, mine_lon=None):
        """
        Call once per sector after scanning completes — mine found or not.
        Sends three images + metadata to pc_sector_viewer.py on the PC.

        Args:
            sector_id    : int
            raw_stack    : ndarray (N, 24, 32) float32
            binary_map   : ndarray (24, 32) uint8  — from SpatiotemporalFilter
            mine_found   : bool
            dx, dy       : pixel offsets from center in 640x480 space (or None)
            conf         : float 0..1
            mine_lat/lon : GPS of confirmed mine (or None)
        """
        try:
            avg_frame  = np.mean(raw_stack, axis=0)
            img_raw    = self._encode_img(self._render_thermal(avg_frame))
            img_binary = self._encode_img(
                self._render_binary(binary_map) if binary_map is not None
                else np.zeros((240, 320, 3), dtype=np.uint8)
            )
            img_final  = self._encode_img(self._render_detection(avg_frame, dx, dy, conf))

            hot_pct = 0.0
            if binary_map is not None:
                hot_pct = 100.0 * float(np.sum(binary_map > 0)) / binary_map.size

            packet = {
                "type":            "sector_result",
                "drone_id":        self.drone_id,
                "sector_id":       sector_id,
                "mine_found":      mine_found,
                "confidence":      round(float(conf), 4),
                "hot_pixel_pct":   round(hot_pct, 2),
                "frames_captured": int(len(raw_stack)),
                "mine_lat":        mine_lat,
                "mine_lon":        mine_lon,
                "timestamp":       time.strftime("%H:%M:%S"),
                "img_raw":         img_raw,
                "img_binary":      img_binary,
                "img_final":       img_final,
            }
            self._transmit(packet)
            status = f"MINE at {mine_lat:.6f},{mine_lon:.6f}" if mine_found else "clear"
            print(f"[COMMS] Sector {sector_id} → {status}")

        except Exception as e:
            print(f"[COMMS] send_sector_result failed: {e}")

    # ── KEPT: original anomaly broadcast for Master drone comms ─────────────
    def send_anomaly_data(self, gps_lat, gps_lon, raw_frame_stack):
        """Sends confirmed mine alert to Master drone. Original behaviour preserved."""
        try:
            avg_frame = np.mean(raw_frame_stack, axis=0)
            vmin, vmax = np.min(avg_frame), np.max(avg_frame)
            norm = ((avg_frame - vmin) / (vmax - vmin) * 255).astype(np.uint8) \
                   if vmax > vmin else np.zeros_like(avg_frame, dtype=np.uint8)
            upscaled  = cv2.resize(norm, (640, 480), interpolation=cv2.INTER_NEAREST)
            color_img = cv2.applyColorMap(upscaled, cv2.COLORMAP_INFERNO)
            h, w = color_img.shape[:2]
            cv2.drawMarker(color_img, (w//2, h//2), (0,255,0), cv2.MARKER_CROSS, 40, 2)
            cv2.circle(color_img, (w//2, h//2), 50, (0,255,0), 2)
            _, buf = cv2.imencode('.jpg', color_img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            packet = {
                "type": "anomaly_report",
                "latitude": gps_lat, "longitude": gps_lon,
                "payload": base64.b64encode(buf).decode('utf-8')
            }
            self._transmit(packet)
            print(f"[COMMS] Anomaly broadcast: {gps_lat:.6f}, {gps_lon:.6f}")
        except Exception as e:
            print(f"[COMMS] send_anomaly_data failed: {e}")


if __name__ == "__main__":
    tunnel = DroneTunnel(target_ip="127.0.0.1")
    fake_stack  = np.random.uniform(20.0, 35.0, (48, 24, 32)).astype(np.float32)
    fake_stack[:, 10:14, 14:18] += 4.0
    fake_binary = np.zeros((24, 32), dtype=np.uint8)
    fake_binary[10:14, 14:18] = 255
    tunnel.send_sector_result(3, fake_stack, fake_binary, True, 40, -20, 0.88, 20.296045, 85.824031)
    tunnel.send_sector_result(4, np.random.uniform(20,28,(48,24,32)).astype(np.float32), mine_found=False)
