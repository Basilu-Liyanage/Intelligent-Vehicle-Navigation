import sys
import time
import threading
from enum import Enum

# Append your project path
sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

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

        # === CONFIG ===
        self.MINIMUM_GAP = 20      # cm - emergency
        self.MID_GAP = 50          # cm - slow down
        self.HIGH_GAP = 100        # cm - normal speed

        self.driver_center_angle = 35
        self.eye_center_angle = 125

        # Smooth control parameters
        self.MAX_SPEED_CHANGE = 25      # units per loop (smooth accel/decel)
        self.MAX_STEER_CHANGE = 6       # degrees per loop (smooth turning)

        # State
        self.running = False
        self.lock = threading.Lock()

        self.current_speed = 0
        self.target_speed = 0
        self.current_steering = self.driver_center_angle   # 0-60 logical driver angle
        self.target_steering = self.driver_center_angle

        # Start centered
        self.CarSteering.set_angle(self.current_steering)
        self.CarEye.set_servo(self.eye_center_angle)

    @staticmethod
    def eye_to_steering(eye_angle: float) -> float:
        """Convert eye servo angle → driver steering angle (0-60)"""
        if eye_angle <= 125:
            driver_angle = 35 + (125 - eye_angle) / 3
        else:
            driver_angle = 35 - (eye_angle - 125) * 7 / 15

        # Clamp to hardware limits
        return max(0, min(60, driver_angle))

    def _ramp_speed(self):
        """Smoothly accelerate or decelerate"""
        diff = self.target_speed - self.current_speed
        change = max(-self.MAX_SPEED_CHANGE, min(self.MAX_SPEED_CHANGE, diff))

        self.current_speed += change

        if self.current_speed <= 0:
            self.CarEngine.stop()
            self.current_speed = 0
        else:
            self.CarEngine.set_speed(int(self.current_speed))

    def _ramp_steering(self):
        """Smoothly turn toward target steering angle"""
        diff = self.target_steering - self.current_steering
        if abs(diff) < 0.5:
            return

        change = max(-self.MAX_STEER_CHANGE, min(self.MAX_STEER_CHANGE, diff))
        self.current_steering += change
        self.CarSteering.set_angle(round(self.current_steering, 1))

    def _emergency_reverse(self, duration: float = 0.8):
        """Quick reverse when stuck"""
        print("🚨 EMERGENCY REVERSE")
        self.CarEngine.stop()
        self.current_speed = 0
        self.CarEngine.reverse()          # assumes your DCMotor.reverse() starts reversing
        time.sleep(duration)
        self.CarEngine.stop()

    def smart_speed_from_distance(self, distance: float) -> int:
        """Convert front distance → target speed"""
        if distance < self.MINIMUM_GAP:
            return 0
        elif distance < self.MID_GAP:
            return 30
        elif distance < self.HIGH_GAP:
            return 60
        else:
            return 100

    def drive(self):
        """Main autonomous driving loop - smooth, safe, reactive"""
        with self.lock:
            self.running = True
            print("🚗 Vehicle Brain started - autonomous mode")

        try:
            while self.running:
                # 1. Always keep eye looking straight for monitoring
                self.CarEye.set_servo(self.eye_center_angle)
                time.sleep(0.04)  # servo settle

                # 2. Read center distance
                distance_center = self.LiDAR.read_distance()

                # 3. Update target speed (smooth accel/decel handled below)
                self.target_speed = self.smart_speed_from_distance(distance_center)

                # 4. Emergency stop + reverse if dangerously close
                if distance_center < self.MINIMUM_GAP / 2 and self.current_speed > 0:
                    self._emergency_reverse(duration=1.0)
                    continue

                # 5. If blocked but not emergency, stop and scan for best path
                if distance_center < self.MINIMUM_GAP and self.current_speed > 5:
                    self.CarEngine.stop()
                    self.current_speed = 0
                    self.target_speed = 0

                    print("🛑 Obstacle ahead - scanning best direction...")

                    # Scan with eye servo (uses the improved MultiAngleLiDAR)
                    best_eye_angle, best_distance, _ = self.CarEye.scan_row()

                    # If all directions are still blocked
                    if best_distance < self.MINIMUM_GAP:
                        print("❌ All directions blocked - reversing")
                        self._emergency_reverse(duration=1.5)
                        self.target_steering = self.driver_center_angle
                        continue

                    # Calculate target driver angle from best eye angle
                    self.target_steering = self.eye_to_steering(best_eye_angle)
                    print(f"✅ Best path found → Eye {best_eye_angle}° | Driver {self.target_steering}°")

                    # Smoothly steer to new direction
                    self._smooth_steer_to_target()

                    # Reset eye to center for next cycle
                    self.CarEye.set_servo(self.eye_center_angle)

                    # Start moving slowly in new direction
                    self.target_speed = 40

                # 6. Apply smooth speed and steering
                self._ramp_speed()
                self._ramp_steering()

                # Small loop delay for smooth control (~10 Hz)
                time.sleep(0.08)

        except Exception as e:
            print(f"💥 Error in drive loop: {e}")
        finally:
            self.CarEngine.stop()
            self.CarSteering.set_angle(self.driver_center_angle)
            self.CarEye.set_servo(self.eye_center_angle)
            print("🛑 Vehicle stopped safely")

    def _smooth_steer_to_target(self):
        """Extra-smooth big steering move when changing direction"""
        steps = 0
        while abs(self.current_steering - self.target_steering) > 1.0 and self.running and steps < 30:
            self._ramp_steering()
            time.sleep(0.025)   # very smooth visual feel
            steps += 1

        # Final exact position
        self.current_steering = self.target_steering
        self.CarSteering.set_angle(self.current_steering)

    def stop(self):
        """Emergency stop from any thread"""
        with self.lock:
            self.running = False
        self.CarEngine.stop()
        self.target_speed = 0
        self.current_speed = 0
        self.target_steering = self.driver_center_angle
        print("🛑 Full emergency stop activated")


# ====================== USAGE ======================
if __name__ == "__main__":
    brain = Vehicle_Brain_Module()
    try:
        brain.drive()          # starts autonomous driving
    except KeyboardInterrupt:
        brain.stop()
        print("Goodbye!")