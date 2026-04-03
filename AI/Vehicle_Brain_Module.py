import sys
import time

# Append your project path
sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

import threading
from enum import Enum
from Hardware.Lidar_Sensor import TFLuna
from Hardware.DC_Motor import DCMotor, MotorDirection
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

        self.MINIMUM_GAP = 20
        self.MID_GAP = 50
        self.HIGH_GAP = 100
        
        self.running = False
        self.lock = threading.Lock()
        self.eye_channel = 1
        self.eye_center_angle = 125
        
        self.current_speed = 0
        self.target_speed = 0
        self.current_steering = 90  # Straight
        self.distance_center  = 0.0
        self.distance_array = list()
        self.turning = False
        
        self.driver_center_angle = 35  # Corresponds to eye_center_angle of 125
        self.driver_left_limit = 0
        self.driver_right_limit = 60
        
        self.eye_center_angle = 125
        self.eye_left_limit = 200
        self.eye_right_limit = 50
        
    
    def smart_speed_control(self, MIN_GAP, MID_GAP, HIGH_GAP):
        while True:
            self.CarEye.set_servo(self.eye_center_angle)
            
            distances = self.LiDAR.read_distance()

            with self.lock:
                if distances < MIN_GAP:
                     self.target_speed = 0
                elif distances < MID_GAP:
                     self.target_speed = 30
                elif distances < HIGH_GAP:
                    self.target_speed = 60
                else:
                    self.target_speed = 100

            time.sleep(0.1)
    
    def eye_to_steering(eye_angle):
        # Main formula
        if eye_angle <= 125:
            driver_angle = 35 + (125 - eye_angle) / 3
        else:
            driver_angle = 35 - (eye_angle - 125) * 7 / 15
        
        # Keep the result between 0 and 60
        driver_angle = max(0, min(60, driver_angle))
        
        return round(driver_angle, 2)
    import math

    def estimate_turn_wait_time(steering_angle: float, speed_percent: float = 50) -> float:
        if speed_percent < 10:
            return 0.8
        
        # Base time increases when steering is sharper
        sharpness_factor = abs(steering_angle - 35) / 25.0 + 0.6   # center = 35
        
        base_time = 1.8 * sharpness_factor
        
        # Speed affects time (faster = less time)
        speed_factor = 60.0 / max(20, speed_percent)
        
        wait_time = base_time * speed_factor
        
        return round(wait_time, 2)
    
    def drive(self):
        try:
            with self.lock:
                self.running = True
                
            while True:
                with self.lock:
                    if not self.running:
                        break
                self.smart_speed_control(self.MINIMUM_GAP, self.MID_GAP, self.HIGH_GAP)
                if speed is not 0:
                    self.CarEngine.set_speed(self.target_speed)
                if speed is 0:
                    self.CarEngine.stop()
                    self.distance_center = self.LiDAR.read_distance()
                    
                    if self.distance_center < self.MINIMUM_GAP:
                        while self.distance_center < self.MINIMUM_GAP:
                            self.CarEngine.reverse()
                            self.distance_center = self.LiDAR.read_distance()
                            time.sleep(0.01)
                        self.CarEngine.stop()
                        best_angle, best_distance = self.CarEye.scan_row()
                        driver_turning_angle = self.eye_to_steering(best_angle)
                        if best_distance < self.MINIMUM_GAP:
                            temp_angle = self.driver_center_angle + (self.driver_center_angle - driver_turning_angle)
                            self.CarSteering.set_angle(temp_angle)
                            self.CarEye.set_servo(self.eye_center_angle)
                            distance = self.LiDAR.read_distance()
                            while distance < self.MINIMUM_GAP:
                                self.CarEngine.reverse()
                                distance = self.LiDAR.read_distance()
                                time.sleep(0.01)
                            self.CarEngine.stop()
                            self.CarSteering.set_angle(self.driver_center_angle)
                            self.CarEye.set_servo(self.eye_center_angle)
                            temp_angle = 0
                        else:
                            self.turning = True
                            self.CarSteering.set_angle(driver_turning_angle)
                            start_time = time.time()
                            wait_time = self.estimate_turn_wait_time(driver_turning_angle, self.current_speed)
                
                if self.turning:
                    if time.time() - start_time >= wait_time:
                        self.CarSteering.set_angle(self.driver_center_angle)
                        self.turning = False
                   
                    
        finally:
            with self.lock:
                self.running = False
            self.CarEngine.stop()
            self.CarSteering.set_angle(self.driver_center_angle)
            self.CarEye.set_servo(self.eye_center_angle)
            