import socket
import threading
import json
import time
import queue
import sys

sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')
from LOG.Logger import IndustrialLogger
from Vehicle_Brain_Module import Vehicle_Brain_Module

SERVER_IP = "192.168.1.100"  # Replace with your server IP
SERVER_PORT = 9999

# ---------------- QUEUE & LOGGER ----------------
log_queue = queue.Queue()
logger = IndustrialLogger("Client_Log.json")

# ---------------- NETWORK SENDER ----------------
def send_logs():
    connected = False  # Initialize to avoid UnboundLocalError
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print(f"[Client] Connecting to {SERVER_IP}:{SERVER_PORT}...")
            sock.connect((SERVER_IP, SERVER_PORT))
            print("[Client] Connected to server")
            connected = True

            while connected:
                while not log_queue.empty():
                    log_entry = log_queue.get()
                    sock.sendall((json.dumps(log_entry) + "\n").encode())
                time.sleep(0.05)

        except Exception as e:
            if connected:
                print(f"[Client] Connection lost: {e}")
            else:
                print(f"[Client] Connection failed, retrying: {e}")
            connected = False
            time.sleep(2)

# ---------------- VEHICLE LOG CALLBACK ----------------
def log_callback(data):
    logger.info("Vehicle event", **data)
    log_queue.put(data)

# ---------------- MAIN ----------------
if __name__ == "__main__":
    threading.Thread(target=send_logs, daemon=True).start()

    # Start vehicle brain
    vehicle = Vehicle_Brain_Module(log_callback=log_callback)
    vehicle.drive()