import socket
import threading
import json
import time
import sys

sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')
from AI.Vehicle_Brain_Module import Vehicle_Brain_Module
from LOG.Logger import IndustrialLogger

SERVER_IP = "192.168.0.10"  # replace with your server IP
SERVER_PORT = 9999

# ---------------- CLIENT LOGGER ----------------
client_logger = IndustrialLogger("Client_Log.json")

vehicle = Vehicle_Brain_Module(log_callback=lambda data: client_logger.info("Vehicle event", **data))

connected = False
start_driving = False
sock = None

# ---------------- NETWORK THREAD ----------------
def network_thread():
    global connected, start_driving, sock
    while True:
        if not connected:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((SERVER_IP, SERVER_PORT))
                connected = True
                print(f"[Client] Connected to server {SERVER_IP}:{SERVER_PORT}")
                client_logger.info("Connected to server")
            except Exception as e:
                print(f"[Client] Connection failed: {e}. Retrying in 3s...")
                time.sleep(3)
                continue

        try:
            data = sock.recv(1024).decode().strip()
            if data == "START":
                print("[Client] Received START command")
                start_driving = True
            elif data == "STOP":
                print("[Client] Received STOP command")
                start_driving = False
            time.sleep(0.1)
        except Exception as e:
            print(f"[Client] Network error: {e}")
            connected = False
            start_driving = False
            sock.close()
            time.sleep(2)

# ---------------- LOG SENDER ----------------
def send_logs():
    global connected, sock
    while True:
        if connected:
            while not vehicle.log_callback is None:
                time.sleep(0.1)
        else:
            time.sleep(1)

# ---------------- MAIN LOOP ----------------
threading.Thread(target=network_thread, daemon=True).start()
threading.Thread(target=send_logs, daemon=True).start()

print("[Client] Waiting for START command from server...")

while True:
    if start_driving:
        vehicle.drive()  # This will run until stopped
    else:
        time.sleep(0.1)