import socket
import threading
import json
import sys
import time

sys.path.append(r'C:\Users\user_\OneDrive\Documents\Intelligent-Vehicle-Navigation')
from LOG.Logger import IndustrialLogger

SERVER_PORT = 9999

# ---------------- SERVER LOGGER ----------------
server_logger = IndustrialLogger("Server_Log.json")

# ---------------- CLIENT HANDLER ----------------
def handle_client(conn, addr, clients):
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
            print(f"[Server] Error: {e}")
            break

    conn.close()
    clients.remove(conn)
    print(f"[Server] Connection closed {addr}")

# ---------------- COMMAND SENDER ----------------
def command_sender(clients):
    """
    Allows typing START/STOP in server console and sends to all clients
    """
    while True:
        cmd = input("Enter command for clients (START/STOP): ").strip().upper()
        if cmd not in ("START", "STOP"):
            print("Invalid command")
            continue

        for conn in clients.copy():  # use copy to avoid modifying list during iteration
            try:
                conn.sendall((cmd + "\n").encode())
                print(f"[Server] Sent {cmd} to client {conn.getpeername()}")
            except Exception as e:
                print(f"[Server] Error sending to client: {e}")
                clients.remove(conn)

# ---------------- MAIN SERVER ----------------
def main():
    clients = []
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(("", SERVER_PORT))
    sock.listen()
    print(f"[Server] Listening on port {SERVER_PORT}")

    # Start command sender thread
    threading.Thread(target=command_sender, args=(clients,), daemon=True).start()

    while True:
        conn, addr = sock.accept()
        clients.append(conn)
        threading.Thread(target=handle_client, args=(conn, addr, clients), daemon=True).start()

if __name__ == "__main__":
    main()