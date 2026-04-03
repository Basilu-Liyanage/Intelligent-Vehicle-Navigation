import socket
import threading
import json
import time
from vehicle_brain_module import Vehicle_Brain_Module
from LOG.Logger import IndustrialLogger

SERVER_IP = "SERVER_IP_HERE"  # Replace with server IP
SERVER_PORT = 9999

client_logger = IndustrialLogger("Client_Log.json")
vehicle = Vehicle_Brain_Module(log_callback=client_logger.info)

# ---------------- NETWORK THREAD ----------------
def network_thread():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((SERVER_IP, SERVER_PORT))
    print("[Client] Connected to server")

    # Send logs in background
    def log_sender():
        while True:
            try:
                log_entry = client_logger.network_queue.get()
                sock.sendall((json.dumps(log_entry) + "\n").encode())
            except:
                break

    threading.Thread(target=log_sender, daemon=True).start()

    # Receive commands
    try:
        buffer = ""
        while True:
            data = sock.recv(1024)
            if not data:
                break
            buffer += data.decode()
            while "\n" in buffer:
                cmd, buffer = buffer.split("\n", 1)
                cmd = cmd.strip().upper()
                print(f"[Client] Received command: {cmd}")
                if cmd == "START":
                    threading.Thread(target=vehicle.drive, daemon=True).start()
                elif cmd == "STOP":
                    vehicle.running = False
                elif cmd == "EXIT":
                    vehicle.running = False
                    sock.close()
                    return
    except Exception as e:
        print(f"[Client] Network error: {e}")

if __name__ == "__main__":
    network_thread()