import sys
import time
import socket
import threading
import json

sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

from industrial_logger import IndustrialLogger
from Hardware.Lidar_Sensor import TFLuna
from Hardware.DC_Motor import DCMotor
from Hardware.PCA_Board import PCA9685
from Hardware.Units.CarSteering import SteeringController
from Hardware.Units.CarEye import MultiAngleLiDAR


class Vehicle_Brain_Module:
    def __init__(self):
        self.logger = IndustrialLogger()

        self.PCABoard = PCA9685()
        self.CarEngine = DCMotor(4, 17, 27, 22, "MainDrive")
        self.CarSteering = SteeringController(self.PCABoard, 0, 0, 60)
        self.CarEye = MultiAngleLiDAR()
        self.LiDAR = TFLuna()

        self.running = False
        self.turning = False
        self.reversing = False

        self.MINIMUM_GAP = 20
        self.MID_GAP = 50
        self.HIGH_GAP = 100

        self.eye_center_angle = 125
        self.driver_center_angle = 35

        self.current_speed = 0
        self.target_speed = 0

    # -------- SPEED --------
    def apply_smooth_speed(self):
        diff = self.target_speed - self.current_speed
        step = 6 if diff > 0 else -10
        self.current_speed += step
        self.current_speed = max(0, min(100, self.current_speed))

        if self.current_speed > 0:
            self.CarEngine.move_forward(self.current_speed)
        else:
            self.CarEngine.stop()

        self.logger.info("Speed update", speed=self.current_speed)

    # -------- STEERING --------
    def center_wheels(self):
        self.CarSteering.set_angle(self.driver_center_angle)

    def eye_to_steering(self, eye):
        return max(0, min(60, 35 - (eye - 125) * (35 / 105)))

    # -------- DRIVE --------
    def drive(self):
        self.logger.info("Drive started")

        while self.running:
            self.CarEye.set_servo(125)
            dist = self.LiDAR.read_distance()

            self.logger.info("Distance read", distance=dist)

            if dist < self.MINIMUM_GAP:
                self.logger.warning("Obstacle detected", distance=dist)

                self.CarEngine.stop()
                self.current_speed = 0

                best_angle, best_distance = self.CarEye.scan_row()

                self.logger.info("Scan result", angle=best_angle, distance=best_distance)

                if best_distance > self.MINIMUM_GAP:
                    self.turning = True

                    steer = self.eye_to_steering(best_angle)
                    self.logger.info("Turning", steering=steer)

                    self.CarSteering.set_angle(steer)
                    self.CarEngine.move_forward(30)

                    time.sleep(1)

                    self.center_wheels()
                    self.turning = False

            else:
                if dist < self.MID_GAP:
                    self.target_speed = 30
                elif dist < self.HIGH_GAP:
                    self.target_speed = 60
                else:
                    self.target_speed = 100

                self.apply_smooth_speed()

            time.sleep(0.05)

    def stop(self):
        self.logger.info("Vehicle stopped")
        self.running = False
        self.CarEngine.stop()
        self.center_wheels()


# -------- CLIENT NETWORK --------
class Client:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def send_logs(self, sock):
        while True:
            try:
                log = self.vehicle.logger.network_queue.get()
                sock.sendall((json.dumps(log) + "\n").encode())
            except:
                break

    def connect(self):
        SERVER_IP = "192.168.1.100"  # CHANGE
        PORT = 5000

        while True:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((SERVER_IP, PORT))

                print("✅ Connected to server")

                threading.Thread(target=self.send_logs, args=(sock,), daemon=True).start()

                while True:
                    cmd = sock.recv(1024).decode().strip()

                    if cmd == "START":
                        self.vehicle.running = True
                        threading.Thread(target=self.vehicle.drive, daemon=True).start()

                    elif cmd == "STOP":
                        self.vehicle.stop()

            except Exception as e:
                print("Reconnect...", e)
                time.sleep(3)


# -------- MAIN --------
if __name__ == "__main__":
    vehicle = Vehicle_Brain_Module()
    client = Client(vehicle)
    client.connect()