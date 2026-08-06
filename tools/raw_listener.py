# Save as raw_listener.py
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 5000))
print("[PC] Listening for raw UDP packets on port 5000...")

while True:
    data, addr = sock.recvfrom(1024)
    
    # OPTION 1: Print raw bytes (Good for debugging)
    print(f"[PC] Received from {addr}: {data}")
    
    # OPTION 2: Print Hexadecimal representation (Good for analyzing binary protocols)
    # print(f"[PC] Received from {addr}: {data.hex()}")
