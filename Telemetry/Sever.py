import socket
import threading
import json

import sys
sys.path.append(r'C:\Users\user_\OneDrive\Documents\Intelligent-Vehicle-Navigation')
from LOG.Logger import IndustrialLogger

SERVER_IP = ""  # empty = all interfaces
SERVER_PORT = 9999

# ---------------- SERVER LOGGER ----------------
server_logger = IndustrialLogger("Server_Log.json")

clients = []

# ---------------- CLIENT HANDLER ----------------
def handle_client(conn, addr):
    print(f"[Server] Connected by {addr}")
    server_logger.info("Client connected", client=str(addr))
    clients.append(conn)

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
    clients.remove(conn)
    print(f"[Server] Connection closed {addr}")
    server_logger.info("Client disconnected", client=str(addr))


def send_command_to_all(command):
    """Send START/STOP to all connected clients"""
    message = command + "\n"
    for c in clients:
        try:
            c.send(message.encode())
        except Exception as e:
            print(f"[Server] Failed to send to client: {e}")


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((SERVER_IP, SERVER_PORT))
    sock.listen()
    print(f"[Server] Waiting for connections on port {SERVER_PORT}...")

    threading.Thread(target=lambda: command_input_loop(), daemon=True).start()

    while True:
        conn, addr = sock.accept()
        threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()


def command_input_loop():
    """Command line to send START/STOP"""
    while True:
        cmd = input("Enter command (START/STOP/EXIT): ").strip().upper()
        if cmd in ["START", "STOP"]:
            send_command_to_all(cmd)
            print(f"[Server] Sent {cmd} to all clients")
        elif cmd == "EXIT":
            print("[Server] Exiting...")
            for c in clients:
                try: c.close()
                except: pass
            exit(0)
        else:
            print("Unknown command. Use START, STOP, or EXIT.")


if __name__ == "__main__":
    main()