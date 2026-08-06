import socket
import numpy as np
import cv2

HOST = '0.0.0.0'
PORT = 5000

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
server.listen(1)

print(f"[SYSTEM] Listening for RAW C++ 32Hz Telemetry on port {PORT}...")

while True:
    conn, addr = server.accept()
    print(f"[CONNECTED] Drone linked at {addr}")
    try:
        while True:
            # We expect exactly 3072 bytes per frame (768 floats * 4 bytes)
            packet = b""
            while len(packet) < 3072:
                chunk = conn.recv(3072 - len(packet))
                if not chunk:
                    raise ConnectionError("Stream ended by drone.")
                packet += chunk

            # Decode raw bytes directly to 24x32 matrix
            data = np.frombuffer(packet, dtype=np.float32).reshape((24, 32))

            # Normalize for OpenCV
            d_min, d_max = np.min(data), np.max(data)
            if d_max > d_min:
                norm_data = ((data - d_min) / (d_max - d_min) * 255).astype(np.uint8)
            else:
                norm_data = np.zeros((24, 32), dtype=np.uint8)

            img_upscaled = cv2.resize(norm_data, (640, 480), interpolation=cv2.INTER_NEAREST)
            colored_img = cv2.applyColorMap(img_upscaled, cv2.COLORMAP_INFERNO)

            cv2.imshow("32Hz C++ Live Feed", colored_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except Exception as e:
        print(f"[DISCONNECTED] {e}")
    finally:
        conn.close()

cv2.destroyAllWindows()
server.close()
