import sys
import time
import threading
import math

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
        self.MINIMUM_GAP = 20
        self.MID_GAP = 50
        self.HIGH_GAP = 100

        self.driver_center_angle = 35   # Logical (0-60)
        self.eye_center_angle = 125

        self.running = False
        self.lock = threading.Lock()

        self.current_speed = 0
        self.target_speed = 0
        self.current_steering = self.driver_center_angle
        self.target_steering = self.driver_center_angle

        self.turning = False
        self.turn_start_time = 0
        self.turn_wait_time = 0

    # ====================== STEERING MAPPING ======================
    def _logical_to_physical(self, logical_angle: float) -> int:
        """Convert logical driver angle (0-60) to physical servo angle (60-120)"""
        physical = 60 + logical_angle
        return max(60, min(120, round(physical)))

    def _set_steering(self, logical_angle: float):
        physical_angle = self._logical_to_physical(logical_angle)
        self.CarSteering.set_angle(physical_angle)
        print(f"[STEERING] Logical: {logical_angle:.1f}° → Physical: {physical_angle}°")

    # ====================== TURN TIME CALCULATION ======================
    def estimate_turn_wait_time(self, steering_angle: float, speed_percent: float = 50) -> float:
        """Calculate how long to wait for the vehicle to turn"""
        if speed_percent < 10:
            return 0.8
        
        sharpness_factor = abs(steering_angle - 35) / 25.0 + 0.6
        base_time = 1.8 * sharpness_factor
        speed_factor = 60.0 / max(20, speed_percent)
        
        wait_time = base_time * speed_factor
        return round(wait_time, 2)

    # ====================== EYE TO STEERING ======================
    @staticmethod
    def eye_to_steering(eye_angle: float) -> float:
        if eye_angle <= 125:
            driver_angle = 35 + (125 - eye_angle) / 3
        else:
            driver_angle = 35 - (eye_angle - 125) * 7 / 15
        return max(0, min(60, round(driver_angle, 2)))

    # ====================== MAIN DRIVE LOOP ======================
    def drive(self):
        with self.lock:
            self.running = True
            print("🚗 Vehicle Brain Started")

        try:
            while self.running:
                # Read center distance
                self.CarEye.set_servo(self.eye_center_angle)
                time.sleep(0.08)
                
                distance_center = self.LiDAR.read_distance()

                # Update target speed
                if distance_center < self.MINIMUM_GAP:
                    self.target_speed = 0
                elif distance_center < self.MID_GAP:
                    self.target_speed = 35
                elif distance_center < self.HIGH_GAP:
                    self.target_speed = 65
                else:
                    self.target_speed = 100

                # Apply speed
                if self.target_speed <= 5:
                    self.CarEngine.stop()
                    self.current_speed = 0
                else:
                    self.CarEngine.move_forward(self.target_speed)
                    self.current_speed = self.target_speed

                # === PATH BLOCKED → SCAN AND TURN ===
                if distance_center < self.MINIMUM_GAP:
                    print(f"🛑 Blocked! ({distance_center:.1f}cm) - Scanning...")
                    self.CarEngine.stop()
                    self.current_speed = 0

                    best_eye_angle, best_distance = self.CarEye.scan_row()

                    if best_distance < self.MINIMUM_GAP * 1.5:
                        print("❌ No safe path → Emergency Reverse")
                        self.CarEngine.move_reverse(40)
                        time.sleep(1.2)
                        self.CarEngine.stop()
                    else:
                        driver_angle = self.eye_to_steering(best_eye_angle)
                        print(f"🎯 Best direction: Eye {best_eye_angle}° → Driver {driver_angle}°")

                        # Start turning
                        self.target_steering = driver_angle
                        self._set_steering(driver_angle)
                        
                        self.turning = True
                        self.turn_start_time = time.time()
                        self.turn_wait_time = self.estimate_turn_wait_time(driver_angle, self.current_speed)

                        print(f"⏳ Waiting {self.turn_wait_time} seconds for turn...")

                # === CHECK IF TURN IS COMPLETE ===
                if self.turning:
                    if time.time() - self.turn_start_time >= self.turn_wait_time:
                        print("✅ Turn completed - Centering steering")
                        self._set_steering(self.driver_center_angle)
                        self.turning = False

                time.sleep(0.07)

        except Exception as e:
            print(f"Error: {e}")
        finally:
            self.stop()

    def stop(self):
        with self.lock:
            self.running = False
        self.CarEngine.stop()
        self._set_steering(self.driver_center_angle)
        self.CarEye.set_servo(self.eye_center_angle)
        print("🛑 Vehicle Stopped Safely")


# ====================== RUN ======================
if __name__ == "__main__":
    vehicle = Vehicle_Brain_Module()
    try:
        vehicle.drive()
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        vehicle.stop()