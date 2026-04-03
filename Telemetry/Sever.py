import socket
import threading
import json
import logging
from logging.handlers import RotatingFileHandler

HOST = "0.0.0.0"
PORT = 5000

# -------- LOGGER --------
logger = logging.getLogger("ServerLogger")
logger.setLevel(logging.INFO)

handler = RotatingFileHandler("Server_Log.json", maxBytes=2_000_000, backupCount=5)
handler.setFormatter(logging.Formatter('%(message)s'))
logger.addHandler(handler)

# -------- SERVER --------
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(1)

print("🟢 Waiting for Pi...")
conn, addr = server.accept()
print(f"✅ Connected: {addr}")

last_id = 0  # 🔥 SYNC CONTROL

def receive_logs():
    global last_id
    buffer = ""

    while True:
        try:
            data = conn.recv(1024).decode()
            if not data:
                break

            buffer += data

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)

                if line.strip():
                    log = json.loads(line)

                    # 🔥 SYNC: ignore duplicates
                    if log["id"] <= last_id:
                        continue

                    last_id = log["id"]

                    logger.info(json.dumps(log))
                    print(f"[{log['level']}] {log['message']}")

        except Exception as e:
            print("❌ Error:", e)
            break

threading.Thread(target=receive_logs, daemon=True).start()

# -------- COMMAND LOOP --------
while True:
    cmd = input("Command (START/STOP): ").strip().upper()

    if cmd in ["START", "STOP"]:
        conn.sendall(cmd.encode())
        print("📤 Sent:", cmd)