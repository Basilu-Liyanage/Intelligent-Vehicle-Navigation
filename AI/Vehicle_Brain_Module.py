#!/usr/bin/env python3
"""
Industrial AI Controller - Fully Compatible with current hardware
- Uses existing DC motor controller, PCA9685 servos, TF-Luna LiDAR
- Smooth AI throttle and steering
- Threaded scanning via CarEye
"""

import sys
import time
import threading
from collections import deque
import numpy as np
import torch
import torch.nn as nn

# Add your project path
sys.path.append('C:/Users/user_/OneDrive/Documents/Intelligent-Vehicle-Navigation')

# ====================== IMPORT HARDWARE ===============================
from Hardware.DC_Motor import DCMotor, MotorDirection
from Hardware.PCA_Board import PCA9685
from Hardware.Lidar_Sensor import TFLuna
from Hardware.Units.CarSteering import SteeringController
from Hardware.Units.CarEye import MultiAngleLiDAR

# ====================== AI MODEL ======================================
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

def load_ai_model(path):
    checkpoint = torch.load(path, map_location='cpu')
    actor_state = checkpoint.get('actor_state_dict', None)
    if actor_state is None:
        raise RuntimeError("Actor state dict not found in checkpoint")
    input_dim = actor_state['fc0.weight'].shape[1]
    output_dim = actor_state['fc4_mean.weight'].shape[0]
    actor = PureAIActor(input_dim, output_dim)
    actor.load_state_dict(actor_state)
    actor.eval()
    print("✅ AI model loaded")
    return actor

# ====================== INDUSTRIAL AI CONTROLLER ======================
class IndustrialAIController:
    def __init__(self, model_path):
        print("Initializing Industrial AI Controller...")

        # Hardware
        self.motor = DCMotor(rpwm_pin=4, lpwm_pin=17, ren_pin=27, len_pin=22)
        self.pca_board = PCA9685()
        self.steering = SteeringController(self.pca_board, servo_channel=0, min_angle=60, max_angle=120)
        self.lidar = TFLuna(port="/dev/serial0")
        self.car_eye = MultiAngleLiDAR()

        # AI
        self.ai = load_ai_model(model_path)

        # State
        self.distance_history = deque(maxlen=5)
        self.running = True
        self.current_speed = 0
        self.target_angle = self.steering.get_current_angle()
        self.scan_lock = threading.Lock()
        self.cycle_count = 0

        # Start scanning thread
        threading.Thread(target=self.scan_loop, daemon=True).start()
        print("✅ Controller ready")

    # --------------------- SCAN LOOP --------------------------------------
    def scan_loop(self):
        while self.running:
            try:
                distance_cm = self.get_lidar_distance()
                if distance_cm < 40:
                    best_distance, best_angle = self.car_eye.scan_row()[len(self.car_eye.angles)//2], 90
                else:
                    best_distance, best_angle = distance_cm, 90

                with self.scan_lock:
                    self.target_angle = best_angle
            except Exception as e:
                print(f"Scan error: {e}")
            time.sleep(0.1)

    # --------------------- LIDAR READ -------------------------------------
    def get_lidar_distance(self):
        d = self.lidar.read_distance()
        if d <= 0:
            d = 100.0
        self.distance_history.append(d)
        return float(np.median(list(self.distance_history)))

    # --------------------- AI THROTTLE ------------------------------------
    def compute_throttle(self, distance_cm):
        if distance_cm < 20:
            return 0
        state = np.array([
            min(distance_cm / 200.0, 1.0),
            self.current_speed / 100.0,
            (self.steering.get_current_angle() - 90)/40.0,
            0.0,  # placeholder
            np.sin(self.cycle_count*0.1),
            1.0 if distance_cm < 30 else 0.0
        ], dtype=np.float32)
        with torch.no_grad():
            action = self.ai(torch.FloatTensor(state).unsqueeze(0), deterministic=True)
        raw_throttle = action.squeeze(0).numpy()[0]
        throttle = (raw_throttle + 1)/2 * 0.65 + 0.15
        return np.clip(throttle, 0.15, 0.8) * 100

    # --------------------- MOTOR CONTROL ----------------------------------
    def apply_motor(self, target_speed, distance_cm):
        if distance_cm < 20:
            self.motor.stop()
            self.current_speed = 0
            return
        diff = target_speed - self.current_speed
        change = np.clip(diff, -8, 5)
        new_speed = np.clip(self.current_speed + change, 0, 75)
        self.motor.move_forward(int(new_speed))
        self.current_speed = new_speed

    # --------------------- STEERING CONTROL -------------------------------
    def apply_steering(self, distance_cm):
        with self.scan_lock:
            angle = self.target_angle
        if distance_cm < 25:
            angle += 15 if angle >= 90 else -15
        elif distance_cm < 40:
            angle += 8 if angle >= 90 else -8
        self.steering.set_angle(angle)

    # --------------------- DRIVE LOOP -------------------------------------
    def drive(self):
        print("🚀 Starting Industrial AI Drive Mode")
        try:
            while self.running:
                self.cycle_count += 1
                start_time = time.time()
                distance = self.get_lidar_distance()
                throttle = self.compute_throttle(distance)
                self.apply_motor(throttle, distance)
                self.apply_steering(distance)
                dt = time.time() - start_time
                time.sleep(max(0, 0.1 - dt))
        except KeyboardInterrupt:
            print("🛑 Stopping AI Controller")
        finally:
            self.shutdown()

    # --------------------- SHUTDOWN --------------------------------------
    def shutdown(self):
        self.running = False
        self.motor.stop()
        self.steering.center()
        print("✅ System safe")

# ====================== MAIN ===========================================
if __name__ == "__main__":
    model_path = "C:/Users/user_/OneDrive/Documents/Intelligent-Vehicle-Navigation/checkpoints/sac_demo_120000_steps.zip"
    controller = IndustrialAIController(model_path)
    controller.drive()