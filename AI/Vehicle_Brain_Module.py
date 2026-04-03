import sys
import time
import math
import threading

sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

from Hardware.Lidar_Sensor import TFLuna
from Hardware.DC_Motor import DCMotor
from Hardware.PCA_Board import PCA9685
from Hardware.Units.CarSteering import SteeringController
from Hardware.Units.CarEye import MultiAngleLiDAR
from LOG.Logger import IndustrialLogger  # Use your logger class


class Vehicle_Brain_Module:
    def __init__(self, logger=None):
        self.PCABoard = PCA9685()
        self.CarEngine = DCMotor(rpwm_pin=4, lpwm_pin=17, ren_pin=27, len_pin=22, motor_name="MainDrive")
        self.CarSteering = SteeringController(self.PCABoard, servo_channel=0, min_angle=0, max_angle=60)
        self.CarEye = MultiAngleLiDAR()
        self.LiDAR = TFLuna()

        self.MINIMUM_GAP = 20
        self.MID_GAP = 50
        self.HIGH_GAP = 100

        self.running = False
        self.turning = False
        self.reversing = False

        self.eye_center_angle = 125
        self.driver_center_angle = 35
        self.STEERING_CENTER_SERVO = self.driver_center_angle

        self.current_speed = 0
        self.target_speed = 0
        self.distance_center = 0.0

        self.logger = logger or IndustrialLogger("client_log.json")

    # ---------------- LOGGING ----------------
    def log_event(self, message, **kwargs):
        self.logger.info(message, speed=self.current_speed, distance=self.distance_center, **kwargs)

    # ---------------- SMOOTH SPEED ----------------
    def apply_smooth_speed(self):
        ACCEL_STEP = 6
        DECEL_STEP = 10
        effective_target = self.target_speed
        if self.turning:
            effective_target = min(self.target_speed, 40)

        if self.current_speed < effective_target:
            self.current_speed += ACCEL_STEP
        elif self.current_speed > effective_target:
            self.current_speed -= DECEL_STEP

        self.current_speed = max(0, min(100, self.current_speed))

        if self.current_speed > 0:
            self.CarEngine.move_forward(self.current_speed)
        else:
            self.CarEngine.stop()

        self.log_event("Speed applied", target_speed=self.target_speed)

    # ---------------- STEERING ----------------
    def center_wheels(self):
        self.CarSteering.set_angle(self.STEERING_CENTER_SERVO)
        time.sleep(0.1)

    def set_wheels_turned(self, driver_angle):
        self.CarSteering.set_angle(driver_angle)
        time.sleep(0.1)

    # ---------------- SPEED CONTROL ----------------
    def smart_speed_control(self):
        self.CarEye.set_servo(self.eye_center_angle)
        time.sleep(0.03)
        self.distance_center = self.LiDAR.read_distance()

        if self.distance_center < self.MINIMUM_GAP:
            self.target_speed = 0
        elif self.distance_center < self.MID_GAP:
            self.target_speed = 30
        elif self.distance_center < self.HIGH_GAP:
            self.target_speed = 60
        else:
            self.target_speed = 100

        self.log_event("Distance checked")
        return self.distance_center

    # ---------------- ANGLE CONVERSION ----------------
    def eye_to_steering(self, eye_angle):
        if eye_angle <= 125:
            driver_angle = 35 + (125 - eye_angle) / 3
        else:
            driver_angle = 35 - (eye_angle - 125) * 7 / 15
        return max(0, min(60, driver_angle))

    def estimate_turn_duration(self, driver_angle):
        angle_diff = abs(driver_angle - self.driver_center_angle)
        duration = 1 + (angle_diff / 60.0) * 1.5
        return duration

    # ---------------- REVERSE ----------------
    def reverse_until_clear(self):
        self.center_wheels()
        self.reversing = True
        self.CarEngine.move_reverse(40)

        start = time.time()
        while time.time() - start < 2:
            d = self.LiDAR.read_distance()
            if d > self.MINIMUM_GAP * 1.5:
                break
            time.sleep(0.05)

        self.CarEngine.stop()
        self.center_wheels()
        self.reversing = False
        self.log_event("Reversed until clear")

    # ---------------- TURN ----------------
    def execute_turn(self, best_angle, best_distance):
        driver_angle = self.eye_to_steering(best_angle)
        turn_time = self.estimate_turn_duration(driver_angle)

        self.set_wheels_turned(driver_angle)
        self.current_speed = 30
        self.CarEngine.move_forward(30)

        start = time.time()
        while time.time() - start < turn_time:
            time.sleep(0.05)

        self.center_wheels()
        self.turning = False
        self.log_event("Turn executed", best_angle=best_angle, driver_angle=driver_angle)

    # ---------------- MAIN LOOP ----------------
    def drive(self):
        try:
            self.running = True
            self.center_wheels()

            while self.running:
                dist = self.smart_speed_control()

                if not self.turning and not self.reversing:
                    self.apply_smooth_speed()

                if dist < self.MINIMUM_GAP and not self.turning and not self.reversing:
                    self.log_event("Obstacle detected")
                    self.CarEngine.stop()
                    self.current_speed = 0

                    self.reverse_until_clear()
                    best_angle, best_distance = self.CarEye.scan_row()
                    self.log_event("Best path scanned", best_angle=best_angle, best_distance=best_distance)

                    if best_distance > self.MINIMUM_GAP:
                        self.turning = True
                        self.execute_turn(best_angle, best_distance)
                    else:
                        self.log_event("No path, reversing more")
                        self.CarEngine.move_reverse(50)
                        time.sleep(1.5)
                        self.CarEngine.stop()
                        self.center_wheels()

                time.sleep(0.03)

        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            self.stop()

    # ---------------- STOP ----------------
    def stop(self):
        self.CarEngine.stop()
        self.current_speed = 0
        for _ in range(2):
            self.CarSteering.set_angle(self.STEERING_CENTER_SERVO)
            time.sleep(0.05)
        self.CarEye.set_servo(self.eye_center_angle)
        self.log_event("Vehicle stopped")


if __name__ == "__main__":
    logger = IndustrialLogger("Client_Log.json")
    vehicle = Vehicle_Brain_Module(logger=logger)
    vehicle.drive()