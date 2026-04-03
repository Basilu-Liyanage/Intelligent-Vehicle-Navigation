import socket
import threading
import json

import sys
sys.path.append(r'C:\Users\user_\OneDrive\Documents\Intelligent-Vehicle-Navigation')
from LOG.Logger import IndustrialLogger

SERVER_PORT = 9999
server_logger = IndustrialLogger("Server_Log.json")

def handle_client(conn, addr):
    print(f"[Server] Connected by {addr}")
    buffer = ""
    while True:
        try:
            data = conn.recv(1024)
            if not data:
                break

            buffer += data.decode()
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                try:
                    log_entry = json.loads(line)
                    server_logger.info("Client log", **log_entry)
                except json.JSONDecodeError:
                    print(f"[Server] Invalid JSON from client: {line}")
        except Exception as e:
            print(f"[Server] Error with {addr}: {e}")
            break

    conn.close()
    print(f"[Server] Connection closed {addr}")

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(("", SERVER_PORT))
    sock.listen()
    print(f"[Server] Waiting for connections on port {SERVER_PORT}...")

    while True:
        conn, addr = sock.accept()
        threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()

if __name__ == "__main__":
    main()