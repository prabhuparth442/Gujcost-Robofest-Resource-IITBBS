import socket
import json
import base64
import numpy as np
import cv2

HOST = '0.0.0.0'
PORT = 5000

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((HOST, PORT))
# CRITICAL: Prevent socket from freezing the OpenCV GUI thread
server.settimeout(0.1) 
server.listen(1)

print(f"[SYSTEM] Listening for Binary Map on port {PORT}...")
cv2.namedWindow("Drone Swarm: Live Binary Map", cv2.WINDOW_NORMAL)

while True:
    try:
        conn, addr = server.accept()
        data = b""
        while True:
            packet = conn.recv(65536)
            if not packet: break
            data += packet
            
        if data:
            payload = json.loads(data.decode('utf-8'))
            if payload['type'] == 'image':
                img_data = base64.b64decode(payload['payload'])
                nparr = np.frombuffer(img_data, np.uint8)
                img = cv2.imdecode(nparr, cv2.IMREAD_GRAYSCALE)
                
                cv2.imshow("Drone Swarm: Live Binary Map", img)
                print("[RECEIVER] Frame rendered.", end='\r')
        conn.close()
        
    except socket.timeout:
        # No packet this fraction of a second. Just pass and let the GUI refresh.
        pass
    except KeyboardInterrupt:
        break
    except Exception as e:
        print(f"\n[ERROR] {e}")

    # Keep the window alive regardless of network state
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
server.close()
