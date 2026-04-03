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
        self.MINIMUM_GAP = 25      # Increased a bit for safety
        self.MID_GAP = 50
        self.HIGH_GAP = 100

        self.driver_center_angle = 35
        self.eye_center_angle = 125

        self.MAX_STEER_CHANGE = 7
        self.SPEED_RAMP_TIME = 0.8

        self.running = False
        self.lock = threading.Lock()

        self.current_speed = 0
        self.target_speed = 0
        self.current_steering = self.driver_center_angle
        self.target_steering = self.driver_center_angle

        # Safe start
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
        diff = self.target_steering - self.current_steering
        if abs(diff) < 0.5:
            return
        change = max(-self.MAX_STEER_CHANGE, min(self.MAX_STEER_CHANGE, diff))
        self.current_steering += change
        self.CarSteering.set_angle(round(self.current_steering, 1))

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
            print("🚗 Autonomous Driving Started - Press Ctrl+C to stop")

        try:
            while self.running:
                # === CENTER LOOK ===
                self.CarEye.set_servo(self.eye_center_angle)
                time.sleep(0.08)

                distance_center = self.LiDAR.read_distance()
                
                print(f"📡 Center Distance: {distance_center:.1f} cm | Speed: {self.current_speed}% | Steering: {self.current_steering}°")

                self.target_speed = self.smart_speed_from_distance(distance_center)

                # === CRITICAL EMERGENCY ===
                if distance_center < 12:
                    print("🚨 TOO CLOSE! Emergency Reverse")
                    self.CarEngine.emergency_stop()
                    self._emergency_reverse(1.3)
                    continue

                # === PATH BLOCKED → SCAN ===
                if distance_center < self.MINIMUM_GAP:
                    print(f"🛑 PATH BLOCKED ({distance_center:.1f}cm) → Starting Scan...")
                    self.CarEngine.stop()
                    self.current_speed = 0
                    self.target_speed = 0

                    # Call your original scan_row (no extra parameters)
                    best_eye_angle, best_distance = self.CarEye.scan_row()

                    print(f"📊 Scan Result → Best Eye: {best_eye_angle}° | Distance: {best_distance:.1f} cm")

                    if best_distance < self.MINIMUM_GAP * 1.5:
                        print("❌ No safe path found → Reversing")
                        self._emergency_reverse(1.8)
                        self.target_steering = self.driver_center_angle
                    else:
                        self.target_steering = self.eye_to_steering(best_eye_angle)
                        print(f"🎯 TURNING TO BEST DIRECTION → Driver Angle: {self.target_steering}°")

                        self._smooth_turn_to_target()
                        self.target_speed = 45   # Start moving slowly after turn

                    # Reset eye to center
                    self.CarEye.set_servo(self.eye_center_angle)

                # === NORMAL DRIVING ===
                self._ramp_steering()
                self._apply_speed()

                time.sleep(0.07)   # Control loop speed

        except Exception as e:
            print(f"💥 Error in drive loop: {e}")
        finally:
            self.stop()

    def _smooth_turn_to_target(self):
        """Smooth steering to new direction"""
        print("🔄 Smoothing steering...")
        for _ in range(40):
            if abs(self.current_steering - self.target_steering) < 1.0:
                break
            self._ramp_steering()
            time.sleep(0.03)
        self.current_steering = self.target_steering
        self.CarSteering.set_angle(self.current_steering)
        print("✅ Steering completed")

    def stop(self):
        with self.lock:
            self.running = False
        self.CarEngine.emergency_stop()
        self.CarSteering.set_angle(self.driver_center_angle)
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