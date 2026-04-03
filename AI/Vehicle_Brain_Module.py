import sys
import time
import threading

sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

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

        # Config
        self.MINIMUM_GAP = 25
        self.MID_GAP = 50
        self.HIGH_GAP = 100

        self.driver_center_angle = 35
        self.eye_center_angle = 125

        self.MAX_STEER_CHANGE = 7
        self.SPEED_RAMP_TIME = 0.8
        self.SCAN_EVERY = 9

        self.running = False
        self.lock = threading.Lock()

        self.current_speed = 0
        self.target_speed = 0
        self.current_steering = self.driver_center_angle
        self.target_steering = self.driver_center_angle

        self.scan_counter = 0

        # Safe start
        self._set_steering(self.current_steering)
        self.CarEye.set_servo(self.eye_center_angle)

    def _logical_to_physical(self, logical: float) -> int:
        """Convert logical driver angle (0-60) to physical servo angle (60-120)"""
        physical = 60 + logical
        return max(60, min(120, round(physical)))

    def _set_steering(self, logical_angle: float):
        """=== STEERING HAPPENS HERE ==="""
        physical_angle = self._logical_to_physical(logical_angle)
        self.CarSteering.set_angle(physical_angle)
        print(f"   [STEERING] Logical {logical_angle:.1f}° → Physical Servo {physical_angle}°")

    @staticmethod
    def eye_to_steering(eye_angle: float) -> float:
        if eye_angle <= 125:
            driver_angle = 35 + (125 - eye_angle) / 3
        else:
            driver_angle = 35 - (eye_angle - 125) * 7 / 15
        return max(0, min(60, round(driver_angle, 2)))

    def _ramp_steering(self):
        """Smoothly moves steering toward target"""
        diff = self.target_steering - self.current_steering
        if abs(diff) < 0.5:
            return

        change = max(-self.MAX_STEER_CHANGE, min(self.MAX_STEER_CHANGE, diff))
        self.current_steering += change
        self._set_steering(self.current_steering)        # ← Steering command

    def _apply_speed(self):
        if self.target_speed <= 5:
            if self.current_speed > 0:
                self.CarEngine.stop()
                self.current_speed = 0
        else:
            if abs(self.target_speed - self.current_speed) > 20:
                self.CarEngine.ramp_to_speed(self.target_speed, MotorDirection.FORWARD, self.SPEED_RAMP_TIME)
            else:
                self.CarEngine.move_forward(self.target_speed)
            self.current_speed = self.target_speed

    def drive(self):
        with self.lock:
            self.running = True
            print("🚗 Started - Scanning & Steering to best path")

        try:
            while self.running:
                self.CarEye.set_servo(self.eye_center_angle)
                time.sleep(0.07)

                distance_center = self.LiDAR.read_distance()
                self.target_speed = self.smart_speed_from_distance(distance_center)

                print(f"📡 Center Dist: {distance_center:.1f} cm | Speed: {self.target_speed}%")

                # Emergency
                if distance_center < 12:
                    self.CarEngine.emergency_stop()
                    self._emergency_reverse(1.3)
                    continue

                # === SCAN & DECIDE NEW DIRECTION ===
                self.scan_counter += 1
                if self.scan_counter >= self.SCAN_EVERY or distance_center < self.MINIMUM_GAP:
                    best_eye_angle, best_distance = self.CarEye.scan_row()

                    if best_distance > 35:
                        self.target_steering = self.eye_to_steering(best_eye_angle)
                        print(f"🎯 TARGET UPDATED → Eye {best_eye_angle}° | Logical Driver {self.target_steering}°")

                    self.scan_counter = 0

                # === STEERING HAPPENS HERE EVERY LOOP ===
                self._ramp_steering()
                self._apply_speed()

                time.sleep(0.06)

        finally:
            self.stop()

    def smart_speed_from_distance(self, distance: float) -> int:
        if distance < self.MINIMUM_GAP:   return 0
        elif distance < self.MID_GAP:     return 35
        elif distance < self.HIGH_GAP:    return 65
        else:                             return 100

    def _emergency_reverse(self, duration=1.2):
        print("🚨 EMERGENCY REVERSE")
        self.CarEngine.stop()
        self.CarEngine.move_reverse(45)
        time.sleep(duration)
        self.CarEngine.stop()

    def stop(self):
        with self.lock:
            self.running = False
        self.CarEngine.emergency_stop()
        self._set_steering(self.driver_center_angle)
        self.CarEye.set_servo(self.eye_center_angle)
        print("🛑 Stopped safely")

    def cleanup(self):
        self.stop()
        self.CarEngine.cleanup()


if __name__ == "__main__":
    brain = Vehicle_Brain_Module()
    try:
        brain.drive()
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        brain.cleanup()