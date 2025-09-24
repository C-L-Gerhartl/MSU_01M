# mit Adminrechten ausführen #####

import socket
import os

UDP_IP = "0.0.0.0"  	# <-- Bitte anpassen
UDP_PORT = 5005		# <-- ggf. ebenfalls anpassen
SHUTDOWN_KEYWORD = "shutdown"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening on UDP port {UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(1024)
    message = data.decode().strip()
    print(f"Received: {message} from {addr}")
    if message == SHUTDOWN_KEYWORD:
        print("Shutdown command received. Shutting down...")
        os.system("shutdown now")