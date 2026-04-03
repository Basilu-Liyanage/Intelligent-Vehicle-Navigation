import socket
import threading
import time
import sys
sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

from AI.Vehicle_Brain_Module import Vehicle_Brain_Module

driver = Vehicle_Brain_Module()
SERVER_IP = "192.168.1.4"   # 🔁 CHANGE to your laptop IP
PORT = 5000


class VehicleClient:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.driver = driver
        self.running = False
        self.drive_thread = None

    def start_drive(self):
        if not self.running:
            print("🚀 START command received")
            self.running = True
            self.vehicle.running = True

            self.drive_thread = threading.Thread(target=self.driver.drive)
            self.drive_thread.daemon = True
            self.drive_thread.start()

    def stop_drive(self):
        print("🛑 STOP command received")
        self.running = False
        self.vehicle.running = False
        self.vehicle.stop()

    def connect(self):
        while True:
            try:
                print("🔌 Connecting to server...")
                client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client.connect((SERVER_IP, PORT))
                print("✅ Connected to server")

                while True:
                    data = client.recv(1024).decode().strip()

                    if not data:
                        break

                    print(f"📩 Command: {data}")

                    if data == "START":
                        self.start_drive()

                    elif data == "STOP":
                        self.stop_drive()

            except Exception as e:
                print(f"❌ Connection error: {e}")
                print("🔄 Reconnecting in 3s...")
                time.sleep(3)

if __name__ == "__main__":
    client = VehicleClient(driver)
    client.connect()