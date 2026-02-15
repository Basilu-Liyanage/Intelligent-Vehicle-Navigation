#!/usr/bin/env python3
"""
INDUSTRIAL-PURE AI TESLA CONTROLLER - FIXED VERSION
- Throttle AI only
- Rule-based steering fully uses CarEye async scan
- Smooth acceleration/deceleration
- Failsafe emergency stops
- High-frequency loop (~10Hz)
- Thread-safe and robust
- FIXED: Maintains motion during turns
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
                        # FIXED: Update eye_position for AI state
                        self.eye_position = best_angle
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
        """Create state vector for AI with all dynamic values"""
        state = np.zeros(6, dtype=np.float32)
        state[0] = min(distance_cm / 200.0, 1.0)  # Normalized distance
        state[1] = max(0, self.current_speed) / 100.0  # Normalized speed
        state[2] = (self.driver_position - DRIVER_CENTER) / 40.0  # Normalized steering
        state[3] = (self.eye_position - EYE_CENTER) / ((EYE_MAX - EYE_MIN)/2)  # FIXED: Now updates!
        state[4] = np.sin(self.cycle_count * 0.1)  # Time feature
        state[5] = 1.0 if distance_cm < 30 else 0.0  # Emergency flag
        return state

    def get_ai_throttle(self, distance_cm):
        """Get throttle from AI with safety overrides"""
        # Emergency stop
        if distance_cm < 20:
            return 0, f"⚠️ EMERGENCY STOP: {distance_cm:.1f}cm"
        
        # Fallback if no AI
        if self.ai is None:
            base_speed = 30 if distance_cm > 50 else 15
            return base_speed, "❌ NO AI: Fallback"
        
        # Get AI prediction
        state = self.create_ai_state(distance_cm)
        with torch.no_grad():
            action = self.ai(torch.FloatTensor(state).unsqueeze(0), deterministic=True)
        
        # Extract throttle from first output dimension
        raw_throttle = action.squeeze(0).numpy()[0]
        
        # FIXED: Ensure minimum throttle during normal operation
        if distance_cm > 30:  # Not emergency
            # Scale from [-1,1] to [0.15, 0.8] for 15-80% speed
            throttle = (raw_throttle + 1) / 2 * 0.65 + 0.15
            throttle = np.clip(throttle, 0.15, 0.8)
        else:
            # Emergency zone - allow lower speeds
            throttle = max(0, raw_throttle)
            throttle = np.clip(throttle, 0, 0.5)
        
        # FIXED: Add minimum speed during sharp turns
        steering_angle = self.driver_position
        if abs(steering_angle - DRIVER_CENTER) > 30:  # Sharp turn
            throttle = max(throttle, 0.20)  # At least 20% speed
            turn_msg = " (turn boost)"
        else:
            turn_msg = ""
        
        target_speed = throttle * 100
        reason = f"🤖 AI DECISION: Speed {target_speed:.0f}%{turn_msg}"
        
        # Debug print (remove in production)
        if self.cycle_count % 10 == 0:
            print(f"\nDEBUG - raw: {raw_throttle:.3f}, final: {throttle:.3f}, speed: {target_speed:.0f}%")
            print(f"DEBUG - dist:{state[0]:.2f} spd:{state[1]:.2f} str:{state[2]:.2f} eye:{state[3]:.2f}")
        
        return target_speed, reason

    # --------------------- MOTOR CONTROL -----------------------------------
    def apply_motor_control(self, target_speed, distance_cm):
        """Apply smooth motor control with emergency stops"""
        # Emergency override
        if distance_cm < 20:
            self.motor.stop()
            self.pca.center_all()
            self.current_speed = 0
            return 0
        
        # Smooth acceleration/deceleration
        speed_diff = target_speed - self.current_speed
        if speed_diff > 0:
            change = min(5, speed_diff)  # Accelerate smoothly
        else:
            change = max(-8, speed_diff)  # Decelerate faster
        
        new_speed = self.current_speed + change
        
        # FIXED: Lower stop threshold and ensure minimum speed
        if new_speed < 2:  # Changed from 5 to 2
            # Only stop if really necessary
            if distance_cm < 25 or target_speed < 1:
                self.motor.stop()
                self.pca.center_all()
                new_speed = 0
            else:
                # Maintain minimum crawl speed
                new_speed = 8
                self.motor.move_forward(int(new_speed))
        else:
            # Normal operation
            new_speed = np.clip(new_speed, 0, 75)
            self.motor.move_forward(int(new_speed))
            self.MotorDirection = MotorDirection.FORWARD
        
        self.current_speed = new_speed
        self.speed_history.append(new_speed)
        return new_speed

    # --------------------- RULE-BASED STEERING -----------------------------
    def apply_rule_steering(self, distance_cm):
        """Apply steering based on CarEye scan results"""
        with self.scan_lock:
            angle = getattr(self, 'target_driver_angle', DRIVER_CENTER)
        
        # FIXED: Better obstacle avoidance steering
        if distance_cm < 25:  # Very close obstacle
            # Steer more aggressively away
            if angle >= DRIVER_CENTER:
                angle = min(angle + 15, 145)  # Turn right more
            else:
                angle = max(angle - 15, 35)   # Turn left more
        elif distance_cm < 40:  # Close obstacle
            # Gentle steering adjustment
            if angle >= DRIVER_CENTER:
                angle = min(angle + 8, 140)
            else:
                angle = max(angle - 8, 40)
        
        # Apply steering
        self.pca.driver.set_angle(angle)
        self.driver_position = angle
        return angle

    # --------------------- HELPERS -----------------------------------------
    def eye_to_driver_accurate(self, eye: float) -> float:
        driver = 90 - (eye * 0.9)
        
        # Clamp safe servo range
        driver = max(35, min(145, driver))
        
        # Deadzone near center (prevents micro jitter)
        if abs(driver - 90) < 4:
            driver = 90
        
        return driver

    def calculate_turn_duration(self, driver_angle: float, speed: float) -> float:
        """Calculate how long to maintain turn"""
        angle_diff = abs(driver_angle - DRIVER_CENTER)
        if angle_diff < 10:
            base = 0.5
            
        elif angle_diff < 25:
            base = 1.5
        elif angle_diff < 40:
            base = 2.5
        else:
            base = 3.5
        
        speed_factor = max(20, min(50, speed)) / 35.0
        return base * speed_factor

    # --------------------- DISPLAY -----------------------------------------
    def display_status(self, distance_cm, speed, steering, reason):
        """Create colorful status display"""
        speed_icon = "🚗" if speed > 30 else "🐢" if speed > 10 else "⏹️"
        dist_icon = "⚠️" if distance_cm < 30 else "📏" if distance_cm < 60 else "🟢"
        steer_icon = "↖️" if steering < 80 else "↗️" if steering > 100 else "⬆️"
        
        status = (f"\033[2K\033[1G[{self.cycle_count:04d}] "
                 f"{dist_icon}{distance_cm:5.1f}cm "
                 f"{speed_icon}{speed:3.0f}% "
                 f"{steer_icon}{steering:3.0f}° "
                 f"👁️{self.eye_position:4.1f} "
                 f"{reason}")
        
        # Color code based on distance
        if distance_cm < 30:
            return f"\033[1;31m{status}\033[0m"  # Red for danger
        elif distance_cm < 60:
            return f"\033[1;33m{status}\033[0m"  # Yellow for caution
        else:
            return f"\033[1;32m{status}\033[0m"  # Green for safe

    # --------------------- DRIVE LOOP -------------------------------------
    def drive(self):
        """Main autonomous driving loop"""
        print("\033[1;36m🚀 STARTING INDUSTRIAL-PURE AI AUTONOMOUS MODE\033[0m")
        print("\033[1;33mPress Ctrl+C to stop\033[0m\n")
        
        try:
            while self.running:
                self.cycle_count += 1
                start_time = time.time()

                # Get sensor data
                distance_cm = self.get_lidar_distance()
                
                # Get AI throttle
                target_speed, reason = self.get_ai_throttle(distance_cm)
                
                # Apply motor control with distance override
                actual_speed = self.apply_motor_control(target_speed, distance_cm)

                # Apply steering
                steering_angle = self.apply_rule_steering(distance_cm)

                # Reset async turn if done
                if self.is_turning and (time.time() - self.turn_start_time) >= self.turn_duration:
                    self.is_turning = False

                # Display status (every 3 cycles for readability)
                if self.cycle_count % 3 == 0:
                    print(self.display_status(distance_cm, actual_speed, 
                                            self.driver_position, reason), end="\r")

                # Maintain ~10Hz loop
                cycle_time = time.time() - start_time
                time.sleep(max(0, 0.1 - cycle_time))

        except KeyboardInterrupt:
            print("\n\033[1;33m🛑 User requested stop\033[0m")
        finally:
            self.shutdown()

    # --------------------- SAFE SHUTDOWN ----------------------------------
    def shutdown(self):
        """Safe system shutdown"""
        print("\n\033[1;36m🛑 Shutting down...\033[0m")
        self.running = False
        self.motor.stop()
        self.pca.center_all()
        time.sleep(0.2)
        print("\033[1;32m✅ SYSTEM SAFE - Goodbye!\033[0m")


# ========================= MAIN ==========================================
def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\n\033[1;31m🛑 EMERGENCY STOP SIGNAL RECEIVED\033[0m")
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    controller = PureAIController()
    controller.drive()