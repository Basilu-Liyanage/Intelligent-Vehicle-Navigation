#!/usr/bin/env python3
"""
IVN CONTROLLER - Intelligent Vehicle Navigation
- Quick scan before startup
- Live updating dashboard (single line refresh)
- Pixhawk speed sensor integration
"""

import time
import numpy as np
import sys
import os
import signal
import threading
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal
from collections import deque
import glob
import subprocess
import queue

# ========================= QUICK SCAN ==============================
def quick_scan():
    """Fast hardware check - returns True if ready"""
    print("\n🔍 IVN QUICK SCAN")
    print("-" * 30)
    
    issues = []
    
    # 1. Quick I2C check (PCA9685)
    try:
        import smbus2
        bus = smbus2.SMBus(3)
        bus.read_byte_data(0x40, 0x00)
        bus.close()
        print("✅ PCA9685: OK")
    except:
        issues.append("PCA9685 not found")
        print("❌ PCA9685: FAIL")
    
    # 2. Quick LiDAR check
    try:
        import serial
        ser = serial.Serial('/dev/serial0', 115200, timeout=0.5)
        if ser.is_open:
            ser.close()
            print("✅ LiDAR port: OK")
        else:
            issues.append("LiDAR port not accessible")
            print("❌ LiDAR port: FAIL")
    except:
        issues.append("LiDAR serial error")
        print("❌ LiDAR port: FAIL")
    
    # 3. Quick Pixhawk check (optional)
    try:
        from Vehicle.perception.sensor_fusion.pixhawk_reader import PixhawkReader
        pixhawk = PixhawkReader()
        if pixhawk.connected or pixhawk.simulation_mode:
            print("✅ Pixhawk: OK" + (" (simulation)" if pixhawk.simulation_mode else ""))
        else:
            print("⚠️ Pixhawk: Not connected (using simulation)")
    except:
        print("⚠️ Pixhawk: Not available (using simulation)")
    
    # 4. Quick GPIO check
    try:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(4, GPIO.IN)
        GPIO.cleanup()
        print("✅ GPIO: OK")
    except:
        issues.append("GPIO error")
        print("❌ GPIO: FAIL")
    
    # 5. Check AI model exists
    model_path = "/home/pi/Desktop/Intelligent-Vehicle-Navigation/AI/spatial_rl_model.pth"
    if os.path.exists(model_path):
        print("✅ AI Model: OK")
    else:
        issues.append("AI model not found")
        print("⚠️ AI Model: Missing (fallback mode)")
    
    print("-" * 30)
    return len(issues) == 0

# ========================= AI MODEL ================================
class CheckpointActor(nn.Module):
    """Matches the exact architecture from your checkpoint"""
    def __init__(self):
        super().__init__()
        
        # Input layer
        self.input_layer = nn.Linear(8, 256)
        
        # Residual block 1
        self.res_block1 = nn.Sequential(
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.LayerNorm(256),
            nn.Linear(256, 256)
        )
        
        # Residual block 2
        self.res_block2 = nn.Sequential(
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.LayerNorm(256),
            nn.Linear(256, 256)
        )
        
        # Output heads
        self.mean_head = nn.Linear(256, 6)
        self.log_std_head = nn.Linear(256, 6)

    def forward(self, x):
        x = F.relu(self.input_layer(x))
        
        residual = x
        x = self.res_block1(x)
        x = F.relu(x + residual)
        
        residual = x
        x = self.res_block2(x)
        x = F.relu(x + residual)
        
        mean = self.mean_head(x)
        log_std = self.log_std_head(x).clamp(-20, 2)
        
        return mean, log_std

class ModelLoader:
    @staticmethod
    def find_model():
        model_path = "/home/pi/Desktop/Intelligent-Vehicle-Navigation/AI/spatial_rl_model.pth"
        if os.path.exists(model_path):
            return model_path
        
        patterns = [
            "/home/pi/Desktop/Intelligent-Vehicle-Navigation/AI/Models/reverse_ready_model_final_*.pth",
            "/home/pi/Desktop/Intelligent-Vehicle-Navigation/AI/Models/reverse_ready_model_latest.pth",
        ]
        for pattern in patterns:
            matches = glob.glob(pattern)
            if matches:
                return max(matches, key=os.path.getctime)
        return None

    @staticmethod
    def load():
        model_path = ModelLoader.find_model()
        if not model_path:
            print("⚠️ No model found")
            return None

        print(f"📁 Loading model: {model_path}")
        checkpoint = torch.load(model_path, map_location='cpu')
        
        actor_sd = checkpoint.get('actor_state_dict', None)
        if actor_sd is None:
            print("❌ actor_state_dict missing")
            return None

        model = CheckpointActor()
        try:
            model.load_state_dict(actor_sd)
            print("✅ Model loaded successfully!")
        except Exception as e:
            print(f"❌ Error loading: {e}")
            return None
        
        model.eval()
        return model

# ========================= CONTROLLER ==============================
class IVNController:
    def __init__(self):
        # Clear screen for clean start
        os.system('clear')
        
        print("\n" + "="*60)
        print("🚗 IVN CONTROLLER v2.0 - Intelligent Vehicle Navigation")
        print("="*60)
        
        # Run quick scan first
        print("\n🔍 Running pre-flight check...")
        if not quick_scan():
            print("\n❌ Pre-flight failed! Fix hardware issues first.")
            response = input("Continue anyway? (y/N): ")
            if response.lower() != 'y':
                sys.exit(1)
        
        print("\n" + "="*60)
        print("🔄 Initializing systems...")
        print("="*60)
        
        # Import hardware
        sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')
        
        try:
            from Hardware.PCA_Board import PCA9685Controller
            from Hardware.DC_Motor import DCMotor, MotorDirection
            from Hardware.Lidar_Sensor import TFLuna as TFLunaInterface
            from Hardware.Units.CarEye import CarEye
            from Config.config import DRIVER_CENTER, EYE_CENTER, EYE_MIN, EYE_MAX
            print("✅ Hardware modules loaded")
        except Exception as e:
            print(f"❌ Hardware import error: {e}")
            sys.exit(1)
        
        # Initialize Pixhawk for speed
        try:
            from Vehicle.perception.sensor_fusion.pixhawk_reader import PixhawkReader
            self.pixhawk = PixhawkReader()
            print("✅ Pixhawk initialized")
        except:
            self.pixhawk = None
            print("⚠️ Pixhawk not available (using motor speed)")
        
        # Hardware
        self.pca = PCA9685Controller()
        self.car_eye = CarEye(self.pca)
        time.sleep(0.5)
        self.motor = DCMotor(4, 17, 27, 22, "IVN")
        self.lidar = TFLunaInterface('/dev/serial0')

        # AI Model
        self.ai = ModelLoader.load()
        self.ai_enabled = self.ai is not None
        self.cmd_queue = queue.Queue()
        self.paused = True  # Start paused 
        if not self.ai_enabled:
            print("⚠️ Running in fallback mode")

        # State
        self.steering = DRIVER_CENTER
        self.eye_pos = EYE_CENTER
        self.motor_speed = 0  # Motor command speed
        self.actual_speed = 0  # Actual speed from Pixhawk
        self.running = True
        self.cycle = 0
        
        # Safety
        self.emergency_stop_distance = 15
        self.reverse_power = 40
        self.forward_power = 30
        
        # Statistics
        self.stats = {
            'forward': 0,
            'reverse': 0,
            'stop': 0,
            'emergency': 0,
            'ai_calls': 0
        }
        
        # For scanning
        self.scan_lock = threading.Lock()
        self.best_angle = DRIVER_CENTER
        self.scan_paused = False
        self.last_scan_readings = []
        self.best_distance = 0
        
        # Distance history
        self.distance_history = deque(maxlen=5)
        
        self.DRIVER_CENTER = DRIVER_CENTER
        self.EYE_CENTER = EYE_CENTER
        self.EYE_MIN = EYE_MIN
        self.EYE_MAX = EYE_MAX
        # Center everything
        self.pca.center_all()
        time.sleep(0.2)
        
        # Start scanner
        threading.Thread(target=self.scanner, daemon=True).start()
        
        # Hide cursor for clean display
        print("\033[?25l", end='')  # Hide cursor
        
        print("\n" + "="*60)
        print("✅ READY - Press Ctrl+C to stop")
        print("="*60 + "\n")
        time.sleep(1)

    # --------------------- SCANNING ---------------------------------
    def scanner(self):
        while self.running:
            if not self.scan_paused:
                try:
                    best_dist, best_angle = self.car_eye.get_moving_direction()
                    if best_dist and best_dist > 0:
                        with self.scan_lock:
                            self.best_angle = self.eye_to_steering(best_angle)
                            self.eye_pos = best_angle
                            self.best_distance = best_dist
                            self.last_scan_readings = [(best_angle, best_dist)]
                except:
                    pass
            time.sleep(0.1)

    # --------------------- SENSORS ----------------------------------
    def get_distance(self):
        try:
            d = self.lidar.read_distance()
            if d:
                distance_cm = d * 100
                self.distance_history.append(distance_cm)
                return distance_cm
        except:
            pass
        return 100.0

    def update_actual_speed(self):
        """Get actual speed from Pixhawk"""
        if self.pixhawk:
            try:
                self.pixhawk.update()
                state = self.pixhawk.get_state()
                # Pixhawk speed is in m/s, convert to percentage for display
                # Assuming max speed ~2 m/s = 100%
                self.actual_speed = state['velocity_x'] * 50  # Scale to 0-100%
            except:
                self.actual_speed = self.motor_speed  # Fallback to motor command
        else:
            self.actual_speed = self.motor_speed

    # --------------------- STATE CREATION --------------------------
    def create_state(self, distance):
        state = np.zeros(8, dtype=np.float32)
        
        # [0] Normalized distance (0-1)
        state[0] = min(distance / 200.0, 1.0)
        
        # [1] Normalized speed (-1 to 1) - use actual speed from Pixhawk
        state[1] = self.actual_speed / 100.0
        
        # [2] Normalized steering (-1 to 1)
        state[2] = (self.steering - self.DRIVER_CENTER) / 40.0
        
        # [3] Normalized eye position (-1 to 1)
        eye_range = (self.EYE_MAX - self.EYE_MIN) / 2
        state[3] = (self.eye_pos - self.EYE_CENTER) / eye_range
        
        # [4] Distance trend
        if len(self.distance_history) >= 2:
            trend = (distance - self.distance_history[-2]) / 10.0
            state[4] = np.clip(trend, -1.0, 1.0)
        else:
            state[4] = 0.0
        
        # [5] Danger indicator
        state[5] = 1.0 if distance < 40.0 else 0.0
        
        # [6] Collision risk
        if len(self.distance_history) >= 3:
            risk = (self.distance_history[-1] - distance) / 3.0
            state[6] = np.clip(risk, -1.0, 1.0)
        else:
            state[6] = 0.0
        
        # [7] Time feature
        state[7] = np.sin(self.cycle * 0.05)
        
        return state

    # --------------------- AI DECISION -----------------------------
    def get_ai_command(self, distance):
        if distance < 15:
            self.stats['emergency'] += 1
            self.scan_paused = True
            return -self.reverse_power, "🚨 EMERGENCY"
        
        if not self.ai_enabled:
            self.scan_paused = False
            if distance < 30:
                return -20, "REVERSE"
            elif distance < 50:
                return 15, "SLOW"
            elif distance < 80:
                return 25, "FORWARD"
            else:
                return 30, "FAST"
        
        try:
            self.stats['ai_calls'] += 1
            
            state = self.create_state(distance)
            tensor = torch.FloatTensor(state).unsqueeze(0)
            
            with torch.no_grad():
                mean, log_std = self.ai(tensor)
            
            std = log_std.exp()
            normal = Normal(mean, std)
            raw_action = normal.sample()
            action = torch.tanh(raw_action).squeeze().numpy()
            
            throttle = action[0]
            
            if throttle >= 0:
                speed = throttle * 50 + 10
            else:
                speed = throttle * 40
            
            if distance < 20:
                if speed > 0:
                    speed = 15
                else:
                    speed = max(speed, -40)
            elif distance < 30 and speed > 25:
                speed = 25
            elif distance < 50 and speed > 40:
                speed = 40
            
            speed = np.clip(speed, -40, 60)
            self.scan_paused = (speed < 0)
            
            return speed, f"AI"
            
        except Exception as e:
            return 0, "ERROR"

    # --------------------- MOTOR CONTROL ---------------------------
    def set_motor(self, target_speed):
        diff = target_speed - self.motor_speed
        if abs(diff) > 3:
            change = 3 if diff > 0 else -3
            target_speed = self.motor_speed + change
        
        self.motor_speed = target_speed
        
        if self.motor_speed > 2:
            self.motor.move_forward(int(abs(self.motor_speed)))
            self.stats['forward'] += 1
        elif self.motor_speed < -2:
            self.motor.move_reverse(int(abs(self.motor_speed)))
            self.stats['reverse'] += 1
        else:
            self.motor.stop()
            self.stats['stop'] += 1

    # --------------------- STEERING --------------------------------
    def set_steering(self, distance):
        with self.scan_lock:
            angle = self.best_angle
        
        if self.motor_speed < 0:
            angle = self.DRIVER_CENTER
        else:
            if distance < 30:
                if angle >= self.DRIVER_CENTER:
                    angle = min(angle + 10, 140)
                else:
                    angle = max(angle - 10, 40)
        
        self.pca.driver.set_angle(angle)
        self.steering = angle

    def eye_to_steering(self, eye_angle):
        steering = 90 - (eye_angle * 0.9)
        steering = max(35, min(145, steering))
        if abs(steering - 90) < 4:
            steering = 90
        return steering

    # --------------------- LIVE DASHBOARD --------------------------
    def show_dashboard(self, distance, reason):
        """Single line updating dashboard"""
        
        # Update actual speed from Pixhawk
        self.update_actual_speed()
        
        # Clear line and return cursor to start
        print('\r\033[K', end='')
        
        # Build dashboard
        # [CYCLE] MODE  DISTANCE  SPEED  STEERING  REASON
        
        # Mode icon
        if self.motor_speed > 5:
            mode = "🚗"
        elif self.motor_speed < -5:
            mode = "🔙"
        else:
            mode = "⏹️"
        
        # Distance icon
        if distance < 20:
            dist_icon = "💀"
        elif distance < 40:
            dist_icon = "⚠️"
        elif distance < 80:
            dist_icon = "📏"
        else:
            dist_icon = "🟢"
        
        # Scan icon
        scan = "⏸️" if self.scan_paused else "🔍"
        
        # Steering icon
        if self.steering < 80:
            steer_icon = "↖️"
        elif self.steering > 100:
            steer_icon = "↗️"
        else:
            steer_icon = "⬆️"
        
        # Speed display (motor command vs actual)
        if self.pixhawk:
            speed_display = f"{self.motor_speed:+3.0f}%({self.actual_speed:3.0f}%)"
        else:
            speed_display = f"{self.motor_speed:+3.0f}%"
        
        # AI confidence bar (simulated)
        if self.ai_enabled and self.motor_speed > 0:
            confidence = min(100, int(distance / 2))
            conf_bar = "█" * (confidence // 10)
        else:
            conf_bar = "▒" * 5
        
        # Build the line
        dashboard = (f"[{self.cycle:04d}] {scan}{mode} "
                    f"{dist_icon}{distance:3.0f}cm "
                    f"{steer_icon}{self.steering:3.0f}° "
                    f"👁️{self.eye_pos:3.0f}° "
                    f"⚡{speed_display} "
                    f"[{conf_bar:10s}] {reason}")
        
        # Color based on danger
        if distance < 20:
            print(f"\033[1;31m{dashboard}\033[0m", end='', flush=True)  # Red
        elif distance < 40:
            print(f"\033[1;33m{dashboard}\033[0m", end='', flush=True)  # Yellow
        else:
            print(f"\033[1;32m{dashboard}\033[0m", end='', flush=True)  # Green

    def show_stats_line(self):
        """Show statistics on separate line"""
        total = sum(self.stats.values()) - self.stats['ai_calls']
        if total > 0:
            rev_pct = (self.stats['reverse'] / total) * 100
            stats = (f"\n📊 Fwd:{self.stats['forward']} Rev:{self.stats['reverse']} "
                    f"({rev_pct:.0f}%) Stop:{self.stats['stop']} "
                    f"Emerg:{self.stats['emergency']} AI:{self.stats['ai_calls']}")
            print(stats)

    # --------------------- MAIN LOOP -------------------------------
    def run(self):
        try:
            while self.running:
                try:
                    cmd, value = self.cmd_queue.get_nowait()
                    if cmd == 'pause':
                        self.paused = True
                        if hasattr(self, 'motor'):
                            self.motor.stop()
                    elif cmd == 'resume':
                        self.paused = False
                except queue.Empty:
                    pass

                if self.paused:
                    time.sleep(0.1)
                    continue
                    
                self.cycle += 1
                start = time.time()
                
                # Get distance
                distance = self.get_distance()
                
                # Get AI command
                target_speed, reason = self.get_ai_command(distance)
                
                # Apply motor control
                self.set_motor(target_speed)
                
                # Apply steering
                self.set_steering(distance)
                
                # Update dashboard (single line)
                self.show_dashboard(distance, reason)
                
                # Show stats every 50 cycles (new line)
                if self.cycle % 50 == 0:
                    self.show_stats_line()
                
                # Maintain 10Hz
                elapsed = time.time() - start
                if elapsed < 0.1:
                    time.sleep(0.1 - elapsed)
                    
        except KeyboardInterrupt:
            self.shutdown()

    def shutdown(self):
        print("\n\n" + "="*60)
        print("🛑 SHUTTING DOWN")
        print("="*60)
        
        # Show final stats
        self.show_stats_line()
        
        self.running = False
        self.motor.stop()
        self.pca.center_all()
        
        # Show cursor again
        print("\033[?25h", end='')
        
        if self.pixhawk:
            self.pixhawk.close()
        
        time.sleep(0.2)
        print("\n✅ System safe - Goodbye!\n")

# ========================= MAIN =====================================
def signal_handler(sig, frame):
    print("\n\n🛑 Emergency stop signal received")
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    controller = IVNController()
    controller.run()