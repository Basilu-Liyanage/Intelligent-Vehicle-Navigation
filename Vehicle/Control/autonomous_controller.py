#!/usr/bin/env python3
"""
TESLA CONTROLLER - FIXED VERSION
- Uses the exact architecture from your checkpoint
- AI controls throttle (action[0])
- Steering from CarEye
- When reversing: wheels straight, scanning paused
- Emergency: if all directions blocked, reverse hard
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

# ========================= HARDWARE IMPORTS ==============================
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

# ========================= AI MODEL (EXACT CHECKPOINT ARCHITECTURE) ========
class CheckpointActor(nn.Module):
    """Matches the exact architecture from your checkpoint"""
    def __init__(self):
        super().__init__()
        
        # Input layer
        self.input_layer = nn.Linear(8, 256)
        
        # Residual block 1 - matches checkpoint structure
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
        # Input layer
        x = F.relu(self.input_layer(x))
        
        # Residual block 1 with skip connection
        residual = x
        x = self.res_block1(x)
        x = F.relu(x + residual)
        
        # Residual block 2 with skip connection
        residual = x
        x = self.res_block2(x)
        x = F.relu(x + residual)
        
        # Outputs
        mean = self.mean_head(x)
        log_std = self.log_std_head(x).clamp(-20, 2)
        
        return mean, log_std


class ModelLoader:
    @staticmethod
    def find_model():
        """Find the model file"""
        # Use the specific path you provided
        model_path = "/home/pi/Desktop/Intelligent-Vehicle-Navigation/AI/spatial_rl_model.pth"
        if os.path.exists(model_path):
            return model_path
        
        # Fallback patterns
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
        """Load the model with exact checkpoint architecture"""
        model_path = ModelLoader.find_model()
        if not model_path:
            print("⚠️ No model found")
            return None

        print(f"📁 Loading model: {model_path}")

        checkpoint = torch.load(model_path, map_location='cpu')
        
        # Print available keys for debugging
        print(f"📊 Checkpoint keys: {list(checkpoint.keys())}")

        # Get actor state dict
        actor_sd = checkpoint.get('actor_state_dict', None)
        if actor_sd is None:
            print("❌ actor_state_dict missing in checkpoint")
            return None

        # Print first few keys to verify structure
        print("📊 First few keys in state_dict:")
        for i, (key, _) in enumerate(list(actor_sd.items())[:5]):
            print(f"   {key}")

        # Create model with matching architecture
        model = CheckpointActor()
        
        # Load the state dict directly - should work now
        try:
            model.load_state_dict(actor_sd)
            print("✅ Model loaded successfully!")
        except Exception as e:
            print(f"❌ Error loading state_dict: {e}")
            return None
        
        model.eval()
        
        # Test the model
        test_tensor = torch.randn(1, 8)
        with torch.no_grad():
            mean, log_std = model(test_tensor)
            print(f"✅ Test forward pass - mean shape: {mean.shape}, log_std shape: {log_std.shape}")
            
            # Sample an action
            std = log_std.exp()
            normal = Normal(mean, std)
            action = torch.tanh(normal.sample())
            print(f"✅ Sample action: {action.squeeze().numpy()}")
        
        return model

# ========================= CONTROLLER ===================================
class TeslaController:
    def __init__(self):
        print("\n" + "="*50)
        print("TESLA CONTROLLER - PRODUCTION VERSION")
        print("="*50)

        # Hardware
        self.pca = PCA9685Controller()
        self.car_eye = CarEye(self.pca)
        time.sleep(0.5)
        self.motor = DCMotor(4, 17, 27, 22, "Tesla")
        self.lidar = TFLunaInterface('/dev/serial0')

        # AI Model
        self.ai = ModelLoader.load()
        self.ai_enabled = self.ai is not None
        if not self.ai_enabled:
            print("⚠️ Running in fallback mode")

        # State
        self.steering = DRIVER_CENTER
        self.eye_pos = EYE_CENTER
        self.speed = 0
        self.running = True
        self.cycle = 0
        
        # Safety
        self.emergency_stop_distance = 15  # cm
        self.reverse_power = 40  # % when reversing
        self.forward_power = 30  # % normal forward
        
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
        
        # Distance history for trend
        self.distance_history = deque(maxlen=5)
        
        # Center everything
        self.pca.center_all()
        time.sleep(0.2)
        
        # Start scanner
        threading.Thread(target=self.scanner, daemon=True).start()
        print("✅ Controller ready\n")

    # --------------------- SCANNING --------------------------------------
    def scanner(self):
        """Background scanner - pauses when reversing"""
        while self.running:
            if not self.scan_paused:
                try:
                    # Get best direction
                    best_dist, best_angle = self.car_eye.get_moving_direction()
                    
                    if best_dist and best_dist > 0:
                        with self.scan_lock:
                            self.best_angle = self.eye_to_steering(best_angle)
                            self.eye_pos = best_angle
                            self.best_distance = best_dist
                            
                            # For emergency detection, store current reading
                            self.last_scan_readings = [(best_angle, best_dist)]
                            
                except Exception as e:
                    # Silently ignore scanner errors
                    pass
            
            time.sleep(0.1)

    # --------------------- LIDAR -----------------------------------------
    def get_distance(self):
        """Get current distance"""
        try:
            d = self.lidar.read_distance()
            if d:
                distance_cm = d * 100  # meters to cm
                self.distance_history.append(distance_cm)
                return distance_cm
        except:
            pass
        return 100.0  # Default safe distance

    # --------------------- STATE CREATION -------------------------------
    def create_state(self, distance):
        """Create 8-dim state vector for the AI"""
        state = np.zeros(8, dtype=np.float32)
        
        # [0] Normalized distance (0-1)
        state[0] = min(distance / 200.0, 1.0)
        
        # [1] Normalized speed (-1 to 1)
        state[1] = self.speed / 100.0
        
        # [2] Normalized steering (-1 to 1)
        state[2] = (self.steering - DRIVER_CENTER) / 40.0
        
        # [3] Normalized eye position (-1 to 1)
        eye_range = (EYE_MAX - EYE_MIN) / 2
        state[3] = (self.eye_pos - EYE_CENTER) / eye_range
        
        # [4] Distance trend (positive = moving away)
        if len(self.distance_history) >= 2:
            trend = (distance - self.distance_history[-2]) / 10.0
            state[4] = np.clip(trend, -1.0, 1.0)
        else:
            state[4] = 0.0
        
        # [5] Danger indicator (0 or 1)
        state[5] = 1.0 if distance < 40.0 else 0.0
        
        # [6] Collision risk (-1 to 1)
        if len(self.distance_history) >= 3:
            risk = (self.distance_history[-1] - distance) / 3.0
            state[6] = np.clip(risk, -1.0, 1.0)
        else:
            state[6] = 0.0
        
        # [7] Time feature (-1 to 1)
        state[7] = np.sin(self.cycle * 0.05)
        
        return state

    # --------------------- AI DECISION -----------------------------------
    def get_ai_command(self, distance):
        """Get throttle from AI using proper sampling"""
        
        # EMERGENCY MODE - if too close
        if distance < 15:
            self.stats['emergency'] += 1
            self.scan_paused = True
            return -self.reverse_power, "🚨 EMERGENCY STOP"
        
        # No AI - simple rules
        if not self.ai_enabled:
            self.scan_paused = False
            if distance < 30:
                return -20, "⚠️ OBSTACLE - reverse"
            elif distance < 50:
                return 15, "⚠️ CAUTION - slow"
            elif distance < 80:
                return 25, "✅ CLEAR - forward"
            else:
                return 30, "✅ OPEN - fast"
        
        # Use AI
        try:
            self.stats['ai_calls'] += 1
            
            # Create state
            state = self.create_state(distance)
            tensor = torch.FloatTensor(state).unsqueeze(0)
            
            # Get mean and log_std from model
            with torch.no_grad():
                mean, log_std = self.ai(tensor)
            
            # Create distribution and sample
            std = log_std.exp()
            normal = Normal(mean, std)
            
            # Sample and apply tanh
            raw_action = normal.sample()
            action = torch.tanh(raw_action).squeeze().numpy()
            
            # Use throttle (first action dimension)
            throttle = action[0]
            
            # Convert to speed
            if throttle >= 0:
                speed = throttle * 50 + 10  # 10 to 60
            else:
                speed = throttle * 40        # -40 to 0
            
            # Safety limits based on distance
            if distance < 20:
                if speed > 0:
                    speed = 15  # Limit forward
                else:
                    speed = max(speed, -40)  # Allow full reverse
            elif distance < 30:
                if speed > 25:
                    speed = 25
            elif distance < 50:
                if speed > 40:
                    speed = 40
            
            speed = np.clip(speed, -40, 60)
            
            # Pause scanning when reversing
            self.scan_paused = (speed < 0)
            
            # Show raw throttle for debugging
            if self.cycle % 10 == 0:
                print(f"🎯 Raw throttle: {throttle:.3f} → Speed: {speed:.0f}%")
            
            return speed, f"AI[{throttle:.2f}]"
            
        except Exception as e:
            print(f"❌ AI error: {e}")
            return 0, "AI ERROR"

    # --------------------- MOTOR CONTROL --------------------------------
    def set_motor(self, target_speed):
        """Set motor speed and direction"""
        # Smooth acceleration
        diff = target_speed - self.speed
        if abs(diff) > 3:
            change = 3 if diff > 0 else -3
            target_speed = self.speed + change
        
        self.speed = target_speed
        
        # Apply motor command
        if self.speed > 2:
            self.motor.move_forward(int(abs(self.speed)))
            self.stats['forward'] += 1
            if self.cycle % 10 == 0:
                print(f"➡️ FORWARD at {int(abs(self.speed))}%")
        elif self.speed < -2:
            self.motor.move_reverse(int(abs(self.speed)))
            self.stats['reverse'] += 1
            if self.cycle % 5 == 0:
                print(f"⬅️ REVERSE at {int(abs(self.speed))}% (scan paused)")
        else:
            self.motor.stop()
            self.stats['stop'] += 1

    # --------------------- STEERING CONTROL -----------------------------
    def set_steering(self, distance):
        """Set steering based on CarEye"""
        with self.scan_lock:
            angle = self.best_angle
        
        # When reversing, keep straight
        if self.speed < 0:
            angle = DRIVER_CENTER
        else:
            # Adjust based on distance
            if distance < 30:
                # In danger, turn more
                if angle >= DRIVER_CENTER:
                    angle = min(angle + 10, 140)
                else:
                    angle = max(angle - 10, 40)
        
        # Apply steering
        self.pca.driver.set_angle(angle)
        self.steering = angle

    # --------------------- CONVERSION ------------------------------------
    def eye_to_steering(self, eye_angle):
        """Convert eye angle to steering angle"""
        steering = 90 - (eye_angle * 0.9)
        steering = max(35, min(145, steering))
        if abs(steering - 90) < 4:
            steering = 90
        return steering

    # --------------------- DISPLAY ---------------------------------------
    def show_status(self, distance):
        """Show status line"""
        # Icons
        if self.speed > 5:
            icon = "🚗"
        elif self.speed < -5:
            icon = "🔙"
        else:
            icon = "⏹️"
        
        # Distance indicator
        if distance < 20:
            dist_icon = "💀"
        elif distance < 40:
            dist_icon = "⚠️"
        elif distance < 80:
            dist_icon = "📏"
        else:
            dist_icon = "🟢"
        
        # Scan status
        scan_icon = "⏸️" if self.scan_paused else "🔍"
        
        # Direction indicator
        if self.steering < 80:
            dir_icon = "↖️"
        elif self.steering > 100:
            dir_icon = "↗️"
        else:
            dir_icon = "⬆️"
        
        # Build status line
        status = (f"[{self.cycle:04d}] {scan_icon} {dist_icon}{distance:3.0f}cm "
                 f"{icon}{self.speed:+3.0f}% {dir_icon}{self.steering:3.0f}° "
                 f"👁️{self.eye_pos:3.0f}°")
        
        # Color code
        if distance < 20:
            print(f"\033[1;31m{status}\033[0m")  # Red
        elif distance < 40:
            print(f"\033[1;33m{status}\033[0m")  # Yellow
        else:
            print(f"\033[1;32m{status}\033[0m")  # Green

    # --------------------- STATS DISPLAY --------------------------------
    def show_stats(self):
        """Show statistics periodically"""
        total = sum(self.stats.values()) - self.stats['ai_calls']  # Exclude ai_calls
        if total > 0:
            rev_pct = (self.stats['reverse'] / total) * 100
            print(f"\n📊 Stats: Reverse={rev_pct:.1f}% ({self.stats['reverse']}/{total}) "
                  f"Fwd={self.stats['forward']} Stop={self.stats['stop']} "
                  f"Emerg={self.stats['emergency']} AI calls={self.stats['ai_calls']}")

    # --------------------- MAIN LOOP ------------------------------------
    def run(self):
        """Main control loop"""
        print("🚀 Starting - Press Ctrl+C to stop\n")
        
        try:
            while self.running:
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
                
                # Show status
                self.show_status(distance)
                
                # Show stats every 50 cycles
                if self.cycle % 50 == 0:
                    self.show_stats()
                
                # Maintain 10Hz
                elapsed = time.time() - start
                if elapsed < 0.1:
                    time.sleep(0.1 - elapsed)
                    
        except KeyboardInterrupt:
            print("\n\n🛑 Stopping...")
        finally:
            self.shutdown()

    def shutdown(self):
        """Safe shutdown"""
        print("\n" + "="*50)
        print("SHUTTING DOWN")
        print("="*50)
        
        # Show final stats
        self.show_stats()
        
        self.running = False
        self.motor.stop()
        self.pca.center_all()
        time.sleep(0.2)
        print("\n✅ System safe - Goodbye!")

# ========================= MAIN =========================================
def signal_handler(sig, frame):
    print("\n🛑 Emergency stop signal received")
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    controller = TeslaController()
    controller.run()