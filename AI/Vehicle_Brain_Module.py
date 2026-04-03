import sys
import time
import threading
from enum import Enum

sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

from Hardware.Lidar_Sensor import TFLuna
from Hardware.DC_Motor import DCMotor, MotorDirection
from Hardware.PCA_Board import PCA9685
from Hardware.Units.CarSteering import SteeringController
from Hardware.Units.CarEye import MultiAngleLiDAR


class Vehicle_Brain_Module:
    def __init__(self):
        self.PCABoard = PCA9685()
        
        # Use your new professional DCMotor
        self.CarEngine = DCMotor(
            rpwm_pin=4, 
            lpwm_pin=17, 
            ren_pin=27, 
            len_pin=22, 
            motor_name="MainDrive"
        )
        
        self.CarSteering = SteeringController(self.PCABoard, servo_channel=0, 
                                            min_angle=60, max_angle=120)
        self.CarEye = MultiAngleLiDAR()
        self.LiDAR = TFLuna()

        # === CONFIG ===
        self.MINIMUM_GAP = 20      # cm
        self.MID_GAP = 50
        self.HIGH_GAP = 100

        self.driver_center_angle = 35
        self.eye_center_angle = 125

        # Smooth control
        self.MAX_STEER_CHANGE = 6          # degrees per update
        self.SPEED_RAMP_TIME = 0.8         # seconds for speed ramp

        # State
        self.running = False
        self.lock = threading.Lock()

        self.current_speed = 0
        self.target_speed = 0
        self.current_steering = self.driver_center_angle
        self.target_steering = self.driver_center_angle

        # Initialize hardware to safe state
        self.CarSteering.set_angle(self.current_steering)
        self.CarEye.set_servo(self.eye_center_angle)

    @staticmethod
    def eye_to_steering(eye_angle: float) -> float:
        if eye_angle <= 125:
            driver_angle = 35 + (125 - eye_angle) / 3
        else:
            driver_angle = 35 - (eye_angle - 125) * 7 / 15
        return max(0, min(60, round(driver_angle, 2)))

    def _ramp_steering(self):
        """Smooth steering movement"""
        diff = self.target_steering - self.current_steering
        if abs(diff) < 0.5:
            return
        
        change = max(-self.MAX_STEER_CHANGE, min(self.MAX_STEER_CHANGE, diff))
        self.current_steering += change
        self.CarSteering.set_angle(round(self.current_steering, 1))

    def _apply_speed(self):
        """Apply target speed using new DCMotor with ramping"""
        if self.target_speed <= 5:
            if self.current_speed > 0:
                self.CarEngine.stop()
                self.current_speed = 0
        else:
            # Smooth ramp using built-in ramp_to_speed when big change
            if abs(self.target_speed - self.current_speed) > 15:
                direction = MotorDirection.FORWARD
                self.CarEngine.ramp_to_speed(self.target_speed, direction, 
                                           ramp_time=self.SPEED_RAMP_TIME)
            else:
                self.CarEngine.move_forward(self.target_speed)
            
            self.current_speed = self.target_speed

    def _emergency_reverse(self, duration: float = 1.0):
        print("🚨 EMERGENCY REVERSE")
        self.CarEngine.stop()
        self.CarEngine.move_reverse(50)
        time.sleep(duration)
        self.CarEngine.stop()

    def smart_speed_from_distance(self, distance: float) -> int:
        if distance < self.MINIMUM_GAP:
            return 0
        elif distance < self.MID_GAP:
            return 35
        elif distance < self.HIGH_GAP:
            return 65
        else:
            return 100

    def drive(self):
        with self.lock:
            self.running = True
            print("🚗 Autonomous Driving Started")

        try:
            while self.running:
                # Center eye for forward monitoring
                self.CarEye.set_servo(self.eye_center_angle)
                time.sleep(0.05)

                distance_center = self.LiDAR.read_distance()

                # Update target speed
                self.target_speed = self.smart_speed_from_distance(distance_center)

                # === EMERGENCY SITUATIONS ===
                if distance_center < 12:                                 # Very close
                    self.CarEngine.emergency_stop()
                    self._emergency_reverse(1.2)
                    continue

                if distance_center < self.MINIMUM_GAP:
                    print("🛑 Path blocked - Scanning alternative route...")
                    self.CarEngine.stop()
                    self.current_speed = 0
                    self.target_speed = 0

                    best_eye_angle, best_distance, _ = self.CarEye.scan_row()

                    if best_distance < self.MINIMUM_GAP * 1.5:
                        print("❌ All directions blocked → Reversing")
                        self._emergency_reverse(1.5)
                        self.target_steering = self.driver_center_angle
                    else:
                        self.target_steering = self.eye_to_steering(best_eye_angle)
                        print(f"✅ Best direction → Eye: {best_eye_angle}° | Driver: {self.target_steering}°")
                        
                        # Smooth big turn
                        self._smooth_turn_to_target()

                    self.CarEye.set_servo(self.eye_center_angle)
                    self.target_speed = 40  # Start slowly after turn

                # === Normal Operation ===
                self._ramp_steering()
                self._apply_speed()

                time.sleep(0.08)   # ~12.5 Hz control loop

        except Exception as e:
            print(f"💥 Error: {e}")
        finally:
            self.stop()

    def _smooth_turn_to_target(self):
        """Extra smooth steering for direction changes"""
        steps = 0
        while abs(self.current_steering - self.target_steering) > 1.0 and steps < 35:
            self._ramp_steering()
            time.sleep(0.03)
            steps += 1
        self.current_steering = self.target_steering
        self.CarSteering.set_angle(self.current_steering)

    def stop(self):
        with self.lock:
            self.running = False
        self.CarEngine.emergency_stop()
        self.CarSteering.set_angle(self.driver_center_angle)
        self.CarEye.set_servo(self.eye_center_angle)
        print("🛑 Vehicle fully stopped")

    def cleanup(self):
        self.stop()
        self.CarEngine.cleanup()


# ====================== RUN ======================
if __name__ == "__main__":
    brain = Vehicle_Brain_Module()
    try:
        brain.drive()
    except KeyboardInterrupt:
        print("\n\n🛑 Stopped by user")
    finally:
        brain.cleanup()