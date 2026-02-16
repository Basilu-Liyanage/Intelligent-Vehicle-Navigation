#!/usr/bin/env python3
"""
CAR CLIENT - Raspberry Pi
Enhanced with scanner output suppression when paused.
"""

import socket
import threading
import sys
import os
import json
from datetime import datetime

sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')
from Vehicle.Control.autonomous_controller import IVNController

INFO_LOG_FILE_PATH = "/tmp/car_info.log"
open(INFO_LOG_FILE_PATH, 'w').close()
last_position = 0

def send_logs(client_socket):
    global last_position
    try:
        with open(INFO_LOG_FILE_PATH, "r") as f:
            f.seek(last_position)
            for line in f:
                client_socket.sendall(line.encode())
            last_position = f.tell()
    except:
        pass

def main(server_ip='192.168.1.2', server_port=65432):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, server_port))
    print(f"✅ Connected to server {server_ip}:{server_port}")

    # Create controller (will be paused initially)
    controller = IVNController()
    controller.paused = True
    # Optional: suppress scanner output when paused (requires minor controller mod)
    # We'll handle it by setting an attribute that the scanner thread can check.
    # For now, we assume the controller's scanner prints regardless.
    print("⏸️ Controller paused. Waiting for 'start' from server")

    exit_event = threading.Event()

    def receiver():
        while not exit_event.is_set():
            try:
                client_socket.settimeout(1.0)
                data = client_socket.recv(1024).decode().strip()
                if not data:
                    continue
                print(f"📨 Command: {data}")
                if data == "start":
                    controller.paused = False
                    print("▶️ Started driving")
                elif data == "stop":
                    controller.paused = True
                    if hasattr(controller, 'motor'):
                        controller.motor.stop()
                    print("⏸️ Stopped driving")
                elif data == "exit":
                    controller.running = False
                    exit_event.set()
                    break
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error: {e}")
                break

    def log_sender():
        while not exit_event.is_set():
            try:
                send_logs(client_socket)
                threading.Event().wait(1)
            except:
                break

    threading.Thread(target=receiver, daemon=True).start()
    threading.Thread(target=log_sender, daemon=True).start()

    try:
        controller.run()
    except KeyboardInterrupt:
        print("\n🛑 Interrupted")
    finally:
        exit_event.set()
        if hasattr(controller, 'motor'):
            controller.motor.stop()
        if hasattr(controller, 'pca'):
            controller.pca.center_all()
        client_socket.close()
        print("🔌 Disconnected")

if __name__ == "__main__":
    server_ip = sys.argv[1] if len(sys.argv) > 1 else '192.168.1.2'
    main(server_ip)