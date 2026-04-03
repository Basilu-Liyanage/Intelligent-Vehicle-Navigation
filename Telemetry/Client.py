import socket
import threading
import json
from LOG.Logger import IndustrialLogger

SERVER_PORT = 9999
server_logger = IndustrialLogger("Server_Log.json")

# Connected clients
clients = []

def handle_client(conn, addr):
    print(f"[Server] Connected by {addr}")
    clients.append(conn)
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
                    # Avoid duplicate message keys
                    entry_data = log_entry.copy()
                    entry_data["client_message"] = entry_data.pop("message", "")
                    server_logger.info("Client log", **entry_data)
                    server_logger.flush()
                except json.JSONDecodeError:
                    print(f"[Server] Invalid JSON: {line}")

    except Exception as e:
        print(f"[Server] Error: {e}")
    finally:
        conn.close()
        clients.remove(conn)
        print(f"[Server] Connection closed {addr}")

# Command sender thread
def command_sender():
    while True:
        cmd = input("[Server Command] START / STOP / EXIT: ").strip().upper()
        if cmd not in ["START", "STOP", "EXIT"]:
            print("Invalid command")
            continue
        for c in clients:
            try:
                c.sendall((cmd + "\n").encode())
            except:
                pass
        if cmd == "EXIT":
            print("Shutting down server...")
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