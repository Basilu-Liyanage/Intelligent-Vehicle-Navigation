import sys
import time
import math

sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

import threading
from Hardware.Lidar_Sensor import TFLuna
from Hardware.DC_Motor import DCMotor
from Hardware.PCA_Board import PCA9685
from Hardware.Units.CarSteering import SteeringController
from Hardware.Units.CarEye import MultiAngleLiDAR


class Vehicle_Brain_Module:
    def __init__(self):
        self.PCABoard = PCA9685()
        self.CarEngine = DCMotor(rpwm_pin=4, lpwm_pin=17, ren_pin=27, len_pin=22, motor_name="MainDrive")
        self.CarSteering = SteeringController(self.PCABoard, servo_channel=0, min_angle=0, max_angle=60)
        self.CarEye = MultiAngleLiDAR()
        self.LiDAR = TFLuna()

        # Distance thresholds
        self.MINIMUM_GAP = 20
        self.MID_GAP = 50
        self.HIGH_GAP = 100

        # Control flags
        self.running = False
        self.turning = False
        self.reversing = False

        # Angles
        self.eye_center_angle = 125
        self.driver_center_angle = 35
        self.STEERING_CENTER_SERVO = self.driver_center_angle

        # Speed tracking
        self.current_speed = 0
        self.target_speed = 0
        self.distance_center = 0.0

    # ================= SMOOTH SPEED =================
    def apply_smooth_speed(self):
        """Smooth acceleration + deceleration"""

        ACCEL_STEP = 6
        DECEL_STEP = 10

        # Reduce speed while turning
        effective_target = self.target_speed
        if self.turning:
            effective_target = min(self.target_speed, 40)

        # Smooth ramp
        if self.current_speed < effective_target:
            self.current_speed += ACCEL_STEP
        elif self.current_speed > effective_target:
            self.current_speed -= DECEL_STEP

        # Clamp
        self.current_speed = max(0, min(100, self.current_speed))

        # Apply
        if self.current_speed > 0:
            self.CarEngine.move_forward(self.current_speed)
        else:
            self.CarEngine.stop()

    # ================= STEERING =================
    def center_wheels(self):
        self.CarSteering.set_angle(self.STEERING_CENTER_SERVO)
        time.sleep(0.1)

    def set_wheels_turned(self, driver_angle):
        self.CarSteering.set_angle(driver_angle)
        time.sleep(0.1)

    # ================= SPEED CONTROL =================
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

        return self.distance_center

    # ================= ANGLE CONVERSION =================
    def eye_to_steering(self, eye_angle):
        if eye_angle <= 125:
            driver_angle = 35 + (125 - eye_angle) / 3
        else:
            driver_angle = 35 - (eye_angle - 125) * 7 / 15

        return max(0, min(60, driver_angle))

    def estimate_turn_duration(self, driver_angle):
        angle_diff = abs(driver_angle - self.driver_center_angle)
        duration = 0.5 + (angle_diff / 60.0) * 1.5
        return duration

    # ================= REVERSE =================
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

    # ================= TURN =================
    def execute_turn(self, best_angle, best_distance):
        driver_angle = self.eye_to_steering(best_angle)
        turn_time = self.estimate_turn_duration(driver_angle)

        print(f"Turning → Eye {best_angle} → Driver {driver_angle}")

        # Turn wheels
        self.set_wheels_turned(driver_angle)

        # Move while turning (IMPORTANT FIX)
        self.current_speed = 30
        self.CarEngine.move_forward(30)

        start = time.time()
        while time.time() - start < turn_time:
            time.sleep(0.05)

        # Straighten
        self.center_wheels()
        self.turning = False

    # ================= MAIN LOOP =================
    def drive(self):
        try:
            self.running = True
            self.center_wheels()

            while self.running:
                dist = self.smart_speed_control()

                # NORMAL DRIVING
                if not self.turning and not self.reversing:
                    self.apply_smooth_speed()

                # OBSTACLE DETECTED
                if dist < self.MINIMUM_GAP and not self.turning and not self.reversing:
                    print(f"\n🛑 Obstacle at {dist:.1f}cm")

                    self.CarEngine.stop()
                    self.current_speed = 0

                    self.reverse_until_clear()

                    best_angle, best_distance = self.CarEye.scan_row()

                    if best_distance > self.MINIMUM_GAP:
                        self.turning = True
                        self.execute_turn(best_angle, best_distance)
                    else:
                        print("❌ No path, reversing more")
                        self.CarEngine.move_reverse(50)
                        time.sleep(1.5)
                        self.CarEngine.stop()
                        self.center_wheels()

                time.sleep(0.03)  # FASTER LOOP (important)

        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            self.stop()

    # ================= STOP =================
    def stop(self):
        self.CarEngine.stop()
        self.current_speed = 0
        for _ in range(2):
            self.CarSteering.set_angle(self.STEERING_CENTER_SERVO)
            time.sleep(0.05)
        self.CarEye.set_servo(self.eye_center_angle)


if __name__ == "__main__":
    vehicle_brain = Vehicle_Brain_Module()
    vehicle_brain.drive()