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
        self.CarSteering = SteeringController(self.PCABoard, servo_channel=0, 
                                            min_angle=60, max_angle=120)
        
        self.CarEye = MultiAngleLiDAR()
        self.LiDAR = TFLuna()

        # === CONFIG ===
        self.MINIMUM_GAP = 25
        self.MID_GAP = 50
        self.HIGH_GAP = 100

        self.driver_center = 35          # Logical 0-60
        self.eye_center_angle = 125

        self.MAX_STEER_CHANGE = 8
        self.SCAN_EVERY = 8              # Scan quite often

        self.running = False
        self.lock = threading.Lock()

        self.current_speed = 0
        self.target_speed = 0
        self.current_driver_angle = self.driver_center   # 0-60 logical
        self.target_driver_angle = self.driver_center

        self.scan_counter = 0

        # Initialize steering to center
        self._set_steering(self.driver_center)

    @staticmethod
    def eye_to_steering(eye_angle: float) -> float:
        if eye_angle <= 125:
            d = 35 + (125 - eye_angle) / 3
        else:
            d = 35 - (eye_angle - 125) * 7 / 15
        return max(0, min(60, round(d, 2)))

    def _driver_to_servo(self, driver_angle: float) -> float:
        """Convert logical driver angle (0-60) → servo angle (60-120)"""
        # Linear mapping: 0→60, 35→90, 60→120
        return 60 + (driver_angle * 1.0)   # 60/60 = 1.0 multiplier

    def _set_steering(self, driver_angle: float):
        """Safely set steering"""
        self.current_driver_angle = driver_angle
        servo_angle = self._driver_to_servo(driver_angle)
        self.CarSteering.set_angle(servo_angle)
        print(f"   → Steering set to {servo_angle:.1f}° (driver {driver_angle:.1f})")

    def _ramp_steering(self):
        diff = self.target_driver_angle - self.current_driver_angle
        if abs(diff) < 0.5:
            return
        
        change = max(-self.MAX_STEER_CHANGE, min(self.MAX_STEER_CHANGE, diff))
        new_angle = self.current_driver_angle + change
        self._set_steering(new_angle)

    def _apply_speed(self):
        if self.target_speed <= 5:
            if self.current_speed > 0:
                self.CarEngine.stop()
                self.current_speed = 0
        else:
            if abs(self.target_speed - self.current_speed) > 20:
                self.CarEngine.ramp_to_speed(self.target_speed, MotorDirection.FORWARD, 0.8)
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
            print("🚗 Autonomous Driving Started - Scanning + Steering Active")

        try:
            while self.running:
                self.CarEye.set_servo(self.eye_center_angle)
                time.sleep(0.07)

                distance_center = self.LiDAR.read_distance()
                self.target_speed = self.smart_speed_from_distance(distance_center)

                print(f"📡 Center: {distance_center:.1f}cm | Speed: {self.current_speed}% | Driver: {self.current_driver_angle:.1f}°")

                # Emergency
                if distance_center < 12:
                    self.CarEngine.emergency_stop()
                    self._emergency_reverse(1.3)
                    self.scan_counter = 0
                    continue

                # === SCAN PERIODICALLY ===
                self.scan_counter += 1
                if self.scan_counter >= self.SCAN_EVERY or distance_center < self.MINIMUM_GAP + 10:
                    print("🔍 Scanning...")
                    best_eye_angle, best_distance = self.CarEye.scan_row()

                    if best_distance > 25:
                        self.target_driver_angle = self.eye_to_steering(best_eye_angle)
                        print(f"🎯 Targeting best direction → Driver {self.target_driver_angle}°")

                    self.scan_counter = 0

                # Smooth steering & speed
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
        self._set_steering(self.driver_center)
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