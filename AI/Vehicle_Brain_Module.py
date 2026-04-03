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

        # === CONFIG ===
        self.MINIMUM_GAP = 25
        self.MID_GAP = 50
        self.HIGH_GAP = 100

        self.driver_center_angle = 35          # logical 0-60
        self.eye_center_angle = 125

        self.MAX_STEER_CHANGE = 7
        self.SPEED_RAMP_TIME = 0.8
        self.SCAN_EVERY = 9                    # scan ~every 0.7-0.8 seconds

        self.running = False
        self.lock = threading.Lock()

        self.current_speed = 0
        self.target_speed = 0
        self.current_steering = self.driver_center_angle   # logical 0-60
        self.target_steering = self.driver_center_angle

        self.scan_counter = 0

        # Safe start with correct physical servo angle
        self._set_steering(self.current_steering)
        self.CarEye.set_servo(self.eye_center_angle)

    def _logical_to_physical(self, logical_angle: float) -> int:
        """CRITICAL FIX: logical driver (0-60) → physical servo (60-120)"""
        physical = 60 + logical_angle          # 0 → 60, 35 → 95, 60 → 120
        return max(60, min(120, round(physical)))

    def _set_steering(self, logical_angle: float):
        """Helper to always send the correct physical angle to servo"""
        physical = self._logical_to_physical(logical_angle)
        self.CarSteering.set_angle(physical)
        print(f"   → Steering command: logical {logical_angle:.1f}° → physical {physical}°")

    @staticmethod
    def eye_to_steering(eye_angle: float) -> float:
        if eye_angle <= 125:
            driver_angle = 35 + (125 - eye_angle) / 3
        else:
            driver_angle = 35 - (eye_angle - 125) * 7 / 15
        return max(0, min(60, round(driver_angle, 2)))

    def _ramp_steering(self):
        diff = self.target_steering - self.current_steering
        if abs(diff) < 0.5:
            return

        change = max(-self.MAX_STEER_CHANGE, min(self.MAX_STEER_CHANGE, diff))
        self.current_steering += change

        self._set_steering(self.current_steering)   # now sends correct physical angle

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

    def _emergency_reverse(self, duration=1.2):
        print("🚨 EMERGENCY REVERSE")
        self.CarEngine.stop()
        self.CarEngine.move_reverse(45)
        time.sleep(duration)
        self.CarEngine.stop()

    def smart_speed_from_distance(self, distance: float) -> int:
        if distance < self.MINIMUM_GAP:   return 0
        elif distance < self.MID_GAP:     return 35
        elif distance < self.HIGH_GAP:    return 65
        else:                             return 100

    def drive(self):
        with self.lock:
            self.running = True
            print("🚗 Autonomous Driving Started - Scanning + Steering to best path")

        try:
            while self.running:
                self.CarEye.set_servo(self.eye_center_angle)
                time.sleep(0.07)

                distance_center = self.LiDAR.read_distance()
                self.target_speed = self.smart_speed_from_distance(distance_center)

                print(f"📡 Center: {distance_center:.1f} cm | Target Speed: {self.target_speed}% | "
                      f"Current Steering (logical): {self.current_steering:.1f}°")

                # Critical emergency
                if distance_center < 12:
                    print("🚨 TOO CLOSE → Emergency Reverse")
                    self.CarEngine.emergency_stop()
                    self._emergency_reverse(1.3)
                    self.scan_counter = 0
                    continue

                # === PERIODIC SCAN + STEER ===
                self.scan_counter += 1
                if self.scan_counter >= self.SCAN_EVERY or distance_center < self.MINIMUM_GAP:
                    print("🔍 Scanning best open direction...")
                    best_eye_angle, best_distance = self.CarEye.scan_row()

                    if best_distance > 30:                     # only trust good readings
                        new_target = self.eye_to_steering(best_eye_angle)
                        self.target_steering = new_target
                        print(f"🎯 BEST DIRECTION FOUND → Eye {best_eye_angle}° | "
                              f"Logical Driver {new_target}°")

                    self.scan_counter = 0

                # Normal smooth steering
                self._ramp_steering()
                self._apply_speed()

                time.sleep(0.06)

        except Exception as e:
            print(f"💥 Error: {e}")
        finally:
            self.stop()

    def stop(self):
        with self.lock:
            self.running = False
        self.CarEngine.emergency_stop()
        self._set_steering(self.driver_center_angle)      # center again
        self.CarEye.set_servo(self.eye_center_angle)
        print("🛑 Vehicle stopped safely")

    def cleanup(self):
        self.stop()
        self.CarEngine.cleanup()


# ====================== START ======================
if __name__ == "__main__":
    brain = Vehicle_Brain_Module()
    try:
        brain.drive()
    except KeyboardInterrupt:
        print("\n\n🛑 Stopped by user")
    finally:
        brain.cleanup()