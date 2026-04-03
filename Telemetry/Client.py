import socket
import threading
import json
import time
from AI.Vehicle_Brain_Module import Vehicle_Brain_Module
from LOG.Logger import IndustrialLogger

SERVER_IP = "192.168.1.4"  # Put your server IP here
SERVER_PORT = 9999

# ---------------- CLIENT LOGGER ----------------
client_logger = IndustrialLogger("Client_Log.json")

# ---------------- NETWORK THREAD ----------------
def network_sender(sock, queue):
    while True:
        try:
            log_entry = queue.get()
            data_str = json.dumps(log_entry)
            sock.sendall(data_str.encode() + b"\n")
        except Exception as e:
            print(f"[Network Error] {e}")
            time.sleep(1)


# ---------------- MAIN CLIENT ----------------
def main():
    # Connect to server
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((SERVER_IP, SERVER_PORT))
    print(f"[Client] Connected to server at {SERVER_IP}:{SERVER_PORT}")

    # Create vehicle with logger callback
    vehicle = Vehicle_Brain_Module(logger=client_logger)

    # Start network sender thread
    network_thread = threading.Thread(target=network_sender, args=(sock, vehicle.logger.network_queue), daemon=True)
    network_thread.start()

    # Listen for server commands (start/stop driving)
    def server_listener():
        while True:
            try:
                msg = sock.recv(1024).decode().strip()
                if msg == "START":
                    print("[Client] Received START command")
                    vehicle.running = True
                    threading.Thread(target=vehicle.drive, daemon=True).start()
                elif msg == "STOP":
                    print("[Client] Received STOP command")
                    vehicle.running = False
            except Exception as e:
                print(f"[Listener Error] {e}")
                time.sleep(1)

    listener_thread = threading.Thread(target=server_listener, daemon=True)
    listener_thread.start()

    # Keep main thread alive
    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()