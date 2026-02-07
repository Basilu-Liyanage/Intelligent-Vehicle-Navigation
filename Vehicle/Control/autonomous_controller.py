#!/usr/bin/env python3
"""
INDUSTRIAL-PURE AI TESLA CONTROLLER
- Throttle AI only
- Rule-based steering fully uses CarEye async scan
- Smooth acceleration/deceleration
- Failsafe emergency stops
- High-frequency loop (~10Hz)
- Thread-safe and robust
"""

import time
import numpy as np
import sys
import os
import signal
import threading
import torch
import torch.nn as nn
from collections import deque

# ========================= HARDWARE IMPORTS ==============================
sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

try:
    from Hardware.PCA_Board import PCA9685Controller
    from Hardware.DC_Motor import DCMotor, MotorDirection
    from Hardware.Lidar_Sensor import TFLuna as TFLunaInterface
    from Hardware.Units.CarEye import CarEye
    from Config.config import DRIVER_CENTER, EYE_CENTER, EYE_MIN, EYE_MAX
    print("\033[1;32m✅ Hardware modules loaded\033[0m")
except Exception as e:
    print(f"\033[1;31m❌ Hardware import error: {e}\033[0m")
    sys.exit(1)

# ========================= AI MODEL =====================================
class PureAIActor(nn.Module):
    def __init__(self, input_dim=6, output_dim=6):
        super().__init__()
        self.fc0 = nn.Linear(input_dim, 128)
        self.fc2 = nn.Linear(128, 128)
        self.fc4_mean = nn.Linear(128, output_dim)
        self.fc4_log_std = nn.Linear(128, output_dim)
        self.apply(self._init_weights)

    def _init_weights(self, m):
        if isinstance(m, nn.Linear):
            nn.init.orthogonal_(m.weight, gain=np.sqrt(2))
            nn.init.constant_(m.bias, 0.0)

    def forward(self, x, deterministic=True):
        x = torch.relu(self.fc0(x))
        x = torch.relu(self.fc2(x))
        mean = self.fc4_mean(x)
        if deterministic:
            return torch.tanh(mean)
        else:
            log_std = torch.clamp(self.fc4_log_std(x), -20, 2)
            std = torch.exp(log_std)
            pi = torch.distributions.Normal(mean, std).rsample()
            return torch.tanh(pi)

class PureAILoader:
    @staticmethod
    def load(model_path):
        if not os.path.exists(model_path):
            print(f"\033[1;33m⚠️ Model not found: {model_path}\033[0m")
            return None
        try:
            checkpoint = torch.load(model_path, map_location='cpu')
            actor_state_dict = checkpoint.get('actor_state_dict', None)
            if actor_state_dict is None:
                print("\033[1;31m❌ No actor_state_dict in checkpoint\033[0m")
                return None
            input_dim = actor_state_dict['fc0.weight'].shape[1]
            output_dim = actor_state_dict['fc4_mean.weight'].shape[0]
            actor = PureAIActor(input_dim=input_dim, output_dim=output_dim)
            actor.load_state_dict(actor_state_dict)
            actor.eval()
            print(f"\033[1;32m✅ PURE AI Model loaded: {input_dim}→128→128→{output_dim}\033[0m")
            return actor
        except Exception as e:
            print(f"\033[1;31m❌ AI model loading failed: {e}\033[0m")
            import traceback
            traceback.print_exc()
            return None

# ========================= PURE AI CONTROLLER =============================
class PureAIController:
    def __init__(self):
        print("\033[1;36m🏭 INITIALIZING INDUSTRIAL-PURE AI CONTROLLER...\033[0m")

        # Hardware
        self.pca = PCA9685Controller()
        self.CarEye = CarEye(self.pca)
        time.sleep(0.5)
        self.motor = DCMotor(4, 17, 27, 22, "PureAI")
        self.MotorDirection = MotorDirection.FORWARD
        self.lidar = TFLunaInterface('/dev/serial0')

        # AI
        model_path = "/home/pi/Desktop/Intelligent-Vehicle-Navigation/AI/spatial_rl_model.pth"
        self.ai = PureAILoader.load(model_path)

        # State
        self.driver_position = DRIVER_CENTER
        self.eye_position = EYE_CENTER
        self.current_speed = 0
        self.running = True
        self.cycle_count = 0
        self.target_driver_angle = DRIVER_CENTER
        self.is_turning = False
        self.turn_start_time = 0
        self.turn_duration = 0
        self.last_distance = 100.0
        self.distance_history = deque(maxlen=5)
        self.speed_history = deque(maxlen=5)
        self.scan_lock = threading.Lock()

        # Timing
        self.scan_interval = 0.1  # faster scanning for smoother turns

        # Center servos
        self.pca.center_all()
        time.sleep(0.2)

        # Start async CarEye scanning
        threading.Thread(target=self.scan_loop, daemon=True).start()
        print("\033[1;32m✅ INDUSTRIAL-PURE AI CONTROLLER READY\033[0m")

    # --------------------- ASYNC CarEye SCANNING ------------------------------
    def scan_loop(self):
        while self.running:
            try:
                best_distance, best_angle = self.CarEye.get_moving_direction()
                if best_distance is not None and best_distance > 0:
                    driver_angle = self.eye_to_driver_accurate(best_angle)
                    with self.scan_lock:
                        self.target_driver_angle = driver_angle
                        self.is_turning = True
                        self.turn_start_time = time.time()
                        self.turn_duration = self.calculate_turn_duration(driver_angle, self.current_speed)
            except Exception as e:
                print(f"\033[1;31m❌ Scan loop error: {e}\033[0m")
            time.sleep(self.scan_interval)

    # --------------------- LIDAR ---------------------------------------------
    def get_lidar_distance(self):
        try:
            raw_distance = self.lidar.read_distance()
            if raw_distance is None:
                return self.last_distance
            distance_cm = raw_distance * 100
            if 0 <= distance_cm <= 800:
                self.distance_history.append(distance_cm)
                filtered = float(np.median(list(self.distance_history)))
                self.last_distance = filtered
                return filtered
        except:
            return self.last_distance
        return self.last_distance

    # --------------------- AI THROTTLE ONLY ---------------------------------
    def create_ai_state(self, distance_cm):
        state = np.zeros(6, dtype=np.float32)
        state[0] = min(distance_cm / 200.0, 1.0)
        state[1] = max(0, self.current_speed) / 100.0
        state[2] = (self.driver_position - DRIVER_CENTER) / 40.0
        state[3] = (self.eye_position - EYE_CENTER) / ((EYE_MAX - EYE_MIN)/2)
        state[4] = np.sin(self.cycle_count * 0.1)
        state[5] = 1.0 if distance_cm < 30 else 0.0
        return state

    def get_ai_throttle(self, distance_cm):
        if distance_cm < 20:  # emergency stop
            return 0, f"⚠️ EMERGENCY STOP: {distance_cm:.1f}cm"
        if self.ai is None:
            return 30 if distance_cm > 50 else 10, "❌ NO AI: Fallback"
        state = self.create_ai_state(distance_cm)
        with torch.no_grad():
            action = self.ai(torch.FloatTensor(state).unsqueeze(0), deterministic=True)
        throttle = max(0, action.squeeze(0).numpy()[0])
        target_speed = max(0, min(80, throttle*100))
        reason = f"🤖 AI DECISION: Speed {target_speed:.0f}%"
        return target_speed, reason

    # --------------------- MOTOR CONTROL -----------------------------------
    def apply_motor_control(self, target_speed):
        # Smooth acceleration / deceleration
        speed_diff = target_speed - self.current_speed
        if speed_diff > 0:
            change = min(5, speed_diff)
        else:
            change = max(-8, speed_diff)
        new_speed = max(0, min(75, self.current_speed + change))

        if new_speed < 5:
            self.motor.stop()
        else:
            self.motor.move_forward(int(new_speed))
            self.MotorDirection = MotorDirection.FORWARD

        self.current_speed = new_speed
        self.speed_history.append(new_speed)
        return new_speed

    # --------------------- RULE-BASED STEERING -----------------------------
    def apply_rule_steering(self, distance_cm):
        with self.scan_lock:
            angle = getattr(self, 'target_driver_angle', DRIVER_CENTER)
        # Fallback: if obstacle extremely close
        if distance_cm < 25:
            angle += 25 if angle < DRIVER_CENTER else -25
        self.pca.driver.set_angle(angle)
        self.driver_position = angle
        return angle

    # --------------------- HELPERS -----------------------------------------
    def eye_to_driver_accurate(self, eye_angle: float) -> float:
        scale = (130-50)/(46-15)
        return 50 + (eye_angle-15)*scale

    def calculate_turn_duration(self, driver_angle: float, speed: float) -> float:
        angle_diff = abs(driver_angle - DRIVER_CENTER)
        base = 0.5 if angle_diff < 10 else 1.5 if angle_diff < 25 else 2.5 if angle_diff < 40 else 3.5
        speed_factor = max(20, min(50, speed))/35.0
        return base * speed_factor

    # --------------------- DISPLAY -----------------------------------------
    def display_status(self, distance_cm, speed, steering, reason):
        speed_icon = "🚗" if speed>30 else "🐢" if speed>10 else "⏹️"
        dist_icon = "📏" if distance_cm<100 else "🟢"
        steer_icon = "↖️" if steering<80 else "↗️" if steering>100 else "⬆️"
        return (f"\033[2K\033[1G[{self.cycle_count:04d}] "
                f"{dist_icon}{distance_cm:5.1f}cm "
                f"{speed_icon}{speed:3.0f}% "
                f"🧭{steer_icon}{steering:3.0f}° "
                f"👁️{self.eye_position:4.1f} "
                f"{reason}")

    # --------------------- DRIVE LOOP -------------------------------------
    def drive(self):
        print("\033[1;36m🚀 STARTING INDUSTRIAL-PURE AI AUTONOMOUS MODE\033[0m")
        try:
            while self.running:
                self.cycle_count += 1
                start_time = time.time()

                distance_cm = self.get_lidar_distance()
                target_speed, reason = self.get_ai_throttle(distance_cm)
                actual_speed = self.apply_motor_control(target_speed)

                steering_angle = self.apply_rule_steering(distance_cm)

                # Reset async turn if done
                if self.is_turning and (time.time() - self.turn_start_time) >= self.turn_duration:
                    self.is_turning = False

                # Display status
                if self.cycle_count % 3 == 0:
                    print(self.display_status(distance_cm, actual_speed, self.driver_position, reason), end="\r")

                # Loop at ~10Hz
                cycle_time = time.time() - start_time
                time.sleep(max(0, 0.1 - cycle_time))

        except KeyboardInterrupt:
            print("\n\033[1;33m🛑 User requested stop\033[0m")
        finally:
            self.shutdown()

    # --------------------- SAFE SHUTDOWN ----------------------------------
    def shutdown(self):
        self.running = False
        self.motor.stop()
        self.pca.center_all()
        time.sleep(0.2)
        print("\n\033[1;32m✅ SYSTEM SAFE - Goodbye!\033[0m")


# ========================= MAIN ==========================================
def signal_handler(sig, frame):
    print("\n\033[1;31m🛑 EMERGENCY STOP SIGNAL RECEIVED\033[0m")
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    controller = PureAIController()
    controller.drive()
