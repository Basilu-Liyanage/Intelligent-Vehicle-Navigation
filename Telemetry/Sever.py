import socket

HOST = "0.0.0.0"
PORT = 5000

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(1)

print("🟢 Server started, waiting for Pi...")

conn, addr = server.accept()
print(f"✅ Connected to {addr}")

while True:
    cmd = input("Enter command (START/STOP): ").strip().upper()

    if cmd in ["START", "STOP"]:
        conn.sendall(cmd.encode())
        print(f"📤 Sent: {cmd}")
    else:
        print("❌ Invalid command")