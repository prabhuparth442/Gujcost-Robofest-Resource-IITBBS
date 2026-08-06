import socket
import cv2
import numpy as np

# Bind to all interfaces on port 5000
HOST = '0.0.0.0'
PORT = 5000

def main():
    print(f"[PC VIEWER] Listening for UDP UDP thermal feed on port {PORT}...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    
    cv2.namedWindow("Drone Swarm: Live Thermal Feed", cv2.WINDOW_NORMAL)

    while True:
        try:
            # 65535 is the max UDP packet size. A compressed 640x480 JPEG easily fits.
            data, addr = sock.recvfrom(65535)
            
            # Decode the raw JPEG bytes back into an image
            np_arr = np.frombuffer(data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if img is not None:
                cv2.imshow("Drone Swarm: Live Thermal Feed", img)
            
            # 1ms wait key to keep the OpenCV GUI thread alive
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n[PC VIEWER] Shutting down.")
                break
                
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"[PC VIEWER] Error: {e}")

    cv2.destroyAllWindows()
    sock.close()

if __name__ == "__main__":
    main()
