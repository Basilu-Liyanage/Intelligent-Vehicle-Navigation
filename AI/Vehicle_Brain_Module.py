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
        self.CarSteering = SteeringController(self.PCABoard, servo_channel=0, min_angle=60, max_angle=120)
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
        self.STEERING_CENTER_SERVO = 60 + self.driver_center_angle

        # Speed tracking
        self.current_speed = 0
        self.target_speed = 0
        self.distance_center = 0.0

        # Turn timing
        self.turn_start_time = 0
        self.turn_duration = 0.0  # Will calculate dynamically

    def center_wheels(self):
        """Force wheels to center position"""
        self.CarSteering.set_angle(self.STEERING_CENTER_SERVO)
        time.sleep(0.15)
        self.CarSteering.set_angle(self.STEERING_CENTER_SERVO)
        time.sleep(0.1)

    def set_wheels_turned(self, driver_angle):
        """Turn wheels to specific angle"""
        servo_angle = 60 + driver_angle
        self.CarSteering.set_angle(servo_angle)
        time.sleep(0.15)

    def smart_speed_control(self):
        """Control speed based on center distance"""
        self.CarEye.set_servo(self.eye_center_angle)
        time.sleep(0.05)

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

    def eye_to_steering(self, eye_angle):
        """Convert eye angle to steering angle"""
        if eye_angle <= 125:
            driver_angle = 35 + (125 - eye_angle) / 3
        else:
            driver_angle = 35 - (eye_angle - 125) * 7 / 15

        driver_angle = max(0, min(60, driver_angle))
        return round(driver_angle, 2)

    def estimate_turn_duration(self, driver_angle, speed_percent):
        """Dynamic turn duration based on angle and speed"""
        angle_diff = abs(driver_angle - self.driver_center_angle)
        speed_factor = max(10, speed_percent)
        duration = (angle_diff / 60.0) * (50.0 / speed_factor) + 0.6
        return round(duration, 2)

    def reverse_until_clear(self):
        """Reverse with straight wheels"""
        self.center_wheels()
        self.reversing = True
        self.CarEngine.move_reverse(45)

        reverse_start = time.time()
        while time.time() - reverse_start < 2.5:
            distance = self.LiDAR.read_distance()
            if distance > self.MINIMUM_GAP * 1.5:
                break
            time.sleep(0.05)

        self.CarEngine.stop()
        self.center_wheels()
        self.reversing = False

    def execute_turn_and_straighten(self, best_angle, best_distance):
        """Turn, drive, then straighten wheels"""
        driver_angle = self.eye_to_steering(best_angle)
        self.turn_duration = self.estimate_turn_duration(driver_angle, self.current_speed)

        # Determine turn direction
        if best_angle > 125:
            direction = "RIGHT"
        elif best_angle < 125:
            direction = "LEFT"
        else:
            direction = "STRAIGHT"

        # Stop before turn
        self.CarEngine.stop()
        time.sleep(0.1)

        # Turn wheels
        self.set_wheels_turned(driver_angle)

        # Move forward through turn slowly
        self.CarEngine.move_forward(max(30, self.current_speed // 2))
        turn_start = time.time()
        while time.time() - turn_start < self.turn_duration:
            time.sleep(0.05)

        # Stop and straighten
        self.CarEngine.stop()
        self.center_wheels()
        time.sleep(0.1)
        self.turning = False

    def drive(self):
        """Main driving loop"""
        try:
            self.running = True
            self.center_wheels()

            while self.running:
                center_distance = self.smart_speed_control()

                # Forward motion if not turning or reversing
                if self.target_speed > 0 and not self.turning and not self.reversing:
                    self.CarEngine.move_forward(self.target_speed)
                    self.current_speed = self.target_speed

                elif self.target_speed == 0 and not self.turning and not self.reversing:
                    self.CarEngine.stop()
                    if center_distance < self.MINIMUM_GAP:
                        self.reverse_until_clear()
                        best_angle, best_distance = self.CarEye.scan_row()

                        if best_distance > self.MINIMUM_GAP:
                            self.turning = True
                            self.execute_turn_and_straighten(best_angle, best_distance)
                        else:
                            self.CarEngine.move_reverse(50)
                            time.sleep(1.5)
                            self.CarEngine.stop()
                            self.center_wheels()
                            self.reversing = False

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\nStopped by user")
        except Exception as e:
            print(f"\nError: {e}")
        finally:
            self.stop()

    def stop(self):
        """Emergency stop - guarantee straight wheels"""
        self.CarEngine.stop()
        for _ in range(3):
            self.CarSteering.set_angle(self.STEERING_CENTER_SERVO)
            time.sleep(0.1)
        self.CarEye.set_servo(self.eye_center_angle)


if __name__ == "__main__":
    vehicle_brain = Vehicle_Brain_Module()
    try:
        vehicle_brain.drive()
    except KeyboardInterrupt:
        print("\nDemo terminated")
    finally:
        vehicle_brain.stop()