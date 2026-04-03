import socket
import threading
import json
import time
import sys
sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')
from AI.Vehicle_Brain_Module import Vehicle_Brain_Module
from LOG.Logger import IndustrialLogger

SERVER_IP = "SERVER_IP_HERE"  # replace with your server IP
SERVER_PORT = 9999

client_logger = IndustrialLogger("Client_Log.json")
vehicle = Vehicle_Brain_Module(log_callback=client_logger.info)

connected = False
sock = None

def connect_to_server():
    global sock, connected
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((SERVER_IP, SERVER_PORT))
            connected = True
            print("[Client] CONNECTED to server")
            break
        except Exception as e:
            print(f"[Client] Connection failed, retrying in 3s... ({e})")
            time.sleep(3)

def network_thread():
    global connected
    connect_to_server()

    # Log sender
    def send_logs():
        while True:
            if not connected:
                time.sleep(0.5)
                continue
            try:
                log_entry = client_logger.network_queue.get()
                sock.sendall((json.dumps(log_entry) + "\n").encode())
            except:
                connected = False
                print("[Client] LOST connection, reconnecting...")
                connect_to_server()

    threading.Thread(target=send_logs, daemon=True).start()

    # Receive commands
    buffer = ""
    while True:
        if not connected:
            time.sleep(0.5)
            continue
        try:
            data = sock.recv(1024)
            if not data:
                connected = False
                print("[Client] DISCONNECTED from server, reconnecting...")
                connect_to_server()
                continue
            buffer += data.decode()
            while "\n" in buffer:
                cmd, buffer = buffer.split("\n", 1)
                cmd = cmd.strip().upper()
                print(f"[Client] Command received: {cmd}")
                if cmd == "START":
                    if not vehicle.running:
                        threading.Thread(target=vehicle.drive, daemon=True).start()
                elif cmd == "STOP":
                    vehicle.running = False
                elif cmd == "EXIT":
                    vehicle.running = False
                    sock.close()
                    return
        except Exception as e:
            connected = False
            print(f"[Client] Network error: {e}, reconnecting...")
            connect_to_server()

if __name__ == "__main__":
    network_thread()