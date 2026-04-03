import socket
import threading
import json
import sys

sys.path.append(r'C:\Users\user_\OneDrive\Documents\Intelligent-Vehicle-Navigation')
from LOG.Logger import IndustrialLogger

SERVER_PORT = 9999
server_logger = IndustrialLogger("Server_Log.json")
clients = {}  # addr -> socket

lock = threading.Lock()

def handle_client(conn, addr):
    print(f"[Server] CONNECTED: {addr}")
    with lock:
        clients[addr] = conn

    buffer = ""
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            buffer += data.decode()
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                try:
                    log_entry = json.loads(line)
                    log_entry["client"] = str(addr)
                    server_logger.info("Client log", **log_entry)
                    server_logger.flush()
                except json.JSONDecodeError:
                    print(f"[Server] Invalid JSON from {addr}: {line}")
    except Exception as e:
        print(f"[Server] Error with {addr}: {e}")
    finally:
        conn.close()
        with lock:
            clients.pop(addr, None)
        print(f"[Server] DISCONNECTED: {addr}")

def command_sender():
    while True:
        cmd = input("[Server Command] START / STOP / EXIT: ").strip().upper()
        if cmd not in ["START", "STOP", "EXIT"]:
            print("Invalid command")
            continue
        with lock:
            for addr, c in list(clients.items()):
                try:
                    c.sendall((cmd + "\n").encode())
                except:
                    print(f"[Server] Failed to send to {addr}")
        if cmd == "EXIT":
            print("Server shutting down...")
            break

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(("", SERVER_PORT))
    sock.listen()
    print(f"[Server] Listening on port {SERVER_PORT}")

    threading.Thread(target=command_sender, daemon=True).start()

    try:
        while True:
            conn, addr = sock.accept()
            threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
    except KeyboardInterrupt:
        print("Server stopped")
    finally:
        sock.close()
        server_logger.flush()

if __name__ == "__main__":
    main()