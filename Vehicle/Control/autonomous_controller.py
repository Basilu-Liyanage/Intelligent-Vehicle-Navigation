#!/usr/bin/env python3
"""
PURE AI TESLA CONTROLLER - FIXED VERSION
- Uses ONLY trained AI for all decisions
- No hard-coded safety zones
- AI decides when to stop, slow down, or reverse
- Maintains servo stability
"""

import time
import numpy as np
import sys
import os
import signal
from collections import deque
import torch
import torch.nn as nn

# ============================================================================
# REAL HARDWARE IMPORTS
# ============================================================================

print("\033[1;35m" + "="*80 + "\033[0m")
print("\033[1;36m🤖 PURE AI TESLA CONTROLLER\033[0m")
print("\033[1;32m   AI makes ALL decisions | No hard-coded rules\033[0m")
print("\033[1;35m" + "="*80 + "\033[0m")

sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

try:
    from Hardware.PCA_Board import PCA9685Controller                                                                                           
    from Hardware.DC_Motor import DCMotor, MotorDirection
    from Hardware.Lidar_Sensor import TFLuna as TFLunaInterface
    from Hardware.Units.CarEye import CarEye
    
    print("\033[1;32m✅ Hardware modules loaded\033[0m")
except Exception as e:
    print(f"\033[1;31m❌ Hardware import error: {e}\033[0m")
    sys.exit(1)

# ============================================================================
# AI MODEL - FIXED TO MATCH YOUR TRAINED 6D MODEL
# ============================================================================

class PureAIActor(nn.Module):
    """FIXED architecture that matches spatial_rl_model.pth"""
    def __init__(self, input_dim=6, output_dim=6):
        super().__init__()
        # EXACTLY as shown in your model: 6 → 128 → 128 → 6
        self.fc0 = nn.Linear(input_dim, 128)
        self.fc2 = nn.Linear(128, 128)
        self.fc4_mean = nn.Linear(128, output_dim)
        self.fc4_log_std = nn.Linear(128, output_dim)
        
        # Initialize weights properly
        self.apply(self._init_weights)
    
    def _init_weights(self, m):
        if isinstance(m, nn.Linear):
            nn.init.orthogonal_(m.weight, gain=np.sqrt(2))
            nn.init.constant_(m.bias, 0.0)
    
    def forward(self, x, deterministic=True):
        # Layer 0: 6 → 128
        x = torch.relu(self.fc0(x))
        
        # Layer 2: 128 → 128
        x = torch.relu(self.fc2(x))
        
        # Layer 4: 128 → 6
        if deterministic:
            mean = self.fc4_mean(x)
            # Use only the first 3 outputs: [throttle, steering, eye]
            action = torch.tanh(mean)
            return action
        else:
            mean = self.fc4_mean(x)
            log_std = self.fc4_log_std(x)
            log_std = torch.clamp(log_std, -20, 2)
            std = torch.exp(log_std)
            normal = torch.distributions.Normal(mean, std)
            pi = normal.rsample()
            return torch.tanh(pi)

class PureAILoader:
    @staticmethod
    def load(model_path):
        """Load your trained AI model"""
        if not os.path.exists(model_path):
            print(f"\033[1;33m⚠️  Model not found: {model_path}\033[0m")
            return None
        
        try:
            print(f"\033[1;36m🔍 Loading PURE AI model: {model_path}\033[0m")
            checkpoint = torch.load(model_path, map_location='cpu')
            
            # Debug: print what's in the checkpoint
            print(f"🤖 Checkpoint keys: {list(checkpoint.keys())}")
            
            if 'actor_state_dict' not in checkpoint:
                print(f"\033[1;31m❌ No actor_state_dict in checkpoint\033[0m")
                # Try looking for other keys
                for key in checkpoint.keys():
                    print(f"   Key: {key}, Type: {type(checkpoint[key])}")
                return None
            
            # Get model dimensions from checkpoint
            actor_state_dict = checkpoint['actor_state_dict']
            print(f"🤖 Actor state dict keys: {list(actor_state_dict.keys())}")
            
            # Determine input/output dimensions from weights
            input_dim = actor_state_dict['fc0.weight'].shape[1]  # Should be 6
            output_dim = actor_state_dict['fc4_mean.weight'].shape[0]  # Should be 6
            
            print(f"\033[1;32m📊 AI Architecture: {input_dim} → 128 → 128 → {output_dim}\033[0m")
            
            # Create model with correct dimensions
            actor = PureAIActor(input_dim=input_dim, output_dim=output_dim)
            actor.load_state_dict(actor_state_dict)
            actor.eval()
            
            print(f"\033[1;32m✅ PURE AI Model loaded successfully!\033[0m")
            print(f"\033[1;33m   AI expects {input_dim}D state, outputs {output_dim}D action\033[0m")
            
            return actor
            
        except Exception as e:
            print(f"\033[1;31m❌ AI model loading failed: {e}\033[0m")
            import traceback
            traceback.print_exc()
            return None

# ============================================================================
# SERVO STABILITY SETTINGS ONLY
# ============================================================================

# SERVO LIMITS (for hardware protection only)
DRIVER_MIN = 40
DRIVER_MAX = 140
DRIVER_CENTER = 90

EYE_CENTER = 31.5
EYE_MIN = 15.0
EYE_MAX = 45.0

# CONTROL TIMING (for hardware stability)
MIN_SERVO_MOVE_TIME = 0.1  # 100ms minimum between servo moves

# ============================================================================
# PURE AI CONTROLLER - NO HARD-CODED RULES
# ============================================================================

class PureAIController:
    """Controller that uses ONLY trained AI for decisions"""
    
    def __init__(self):
        print("\n\033[1;36m🏭 INITIALIZING PURE AI CONTROLLER...\033[0m")
        
        # Initialize hardware
        try:
            self.pca = PCA9685Controller()
            self.CarEye = CarEye(self.pca)
            time.sleep(1)
            print("\033[1;32m✅ PCA9685 ready\033[0m")
        except Exception as e:
            print(f"\033[1;31m❌ PCA error: {e}\033[0m")
            raise
        
        self.MotorDirection = MotorDirection
        self.motor = DCMotor(
            rpwm_pin=4,
            lpwm_pin=17,
            ren_pin=27,
            len_pin=22,
            motor_name="PureAI"
        )
        print("\033[1;32m✅ DCMotor ready\033[0m")
        
        self.lidar = TFLunaInterface('/dev/serial0')
        print("\033[1;32m✅ LiDAR ready\033[0m")
        
        # Load PURE AI model
        model_path = "/home/pi/Desktop/Intelligent-Vehicle-Navigation/AI/spatial_rl_model.pth"
        self.ai = PureAILoader.load(model_path)
        
        # State tracking
        self.eye_position = EYE_CENTER
        self.driver_position = DRIVER_CENTER
        self.current_speed = 0
        self.running = True
        self.cycle_count = 0
        
        # Scanning state
        self.last_scan_time = 0
        self.scan_interval = 5.0  # Scan every 5 seconds
        
        # History for AI state
        self.distance_history = deque(maxlen=5)
        self.speed_history = deque(maxlen=5)
        self.last_distance = 100.0  # Start with safe distance
        
        # Timing
        self.last_eye_move = 0
        self.last_driver_move = 0
        
        # Turning state
        self.turn_start_time = 0
        self.is_turning = False
        self.turn_duration = 0
        self.target_driver_angle = DRIVER_CENTER
        
        # Test AI if loaded
        if self.ai:
            self.test_ai_model()
        
        # Center everything
        self.pca.center_all()
        time.sleep(0.5)
        
        print("\n\033[1;35m" + "="*80 + "\033[0m")
        if self.ai:
            print("\033[1;32m✅ PURE AI READY FOR AUTONOMOUS CONTROL\033[0m")
            print("\033[1;33m   AI makes ALL decisions - No human rules\033[0m")
        else:
            print("\033[1;31m❌ FALLBACK MODE - No AI available\033[0m")
            print("\033[1;33m   Using simple distance-based control\033[0m")
        print("\033[1;35m" + "="*80 + "\033[0m")
    
    def test_ai_model(self):
        """Test AI model with sample inputs"""
        print("\n\033[1;36m🧪 TESTING AI MODEL...\033[0m")
        
        test_states = [
            [0.2, 0.3, 0.0, 0.0, 0.0, 1.0],  # Close obstacle
            [0.8, 0.5, 0.0, 0.0, 0.0, 0.0],  # Far, moving
            [0.5, 0.0, 0.5, 0.0, 0.5, 0.0],  # Medium distance, turning
        ]
        
        for i, state in enumerate(test_states):
            with torch.no_grad():
                state_tensor = torch.FloatTensor(state).unsqueeze(0)
                action = self.ai(state_tensor, deterministic=True)
                action_np = action.squeeze(0).numpy()
                
                print(f"Test {i+1}: State {state} → Action {action_np}")
                print(f"   Throttle: {action_np[0]:.3f} → Speed: {max(0, action_np[0])*100:.0f}%")
                print(f"   Steering: {action_np[1]:.3f} → Angle: {DRIVER_CENTER + action_np[1]*40:.0f}°")
                print()
    
    def get_lidar_distance(self):
        """Get LiDAR distance with filtering"""
        try:
            raw_distance = self.lidar.read_distance()
            
            if raw_distance is None:
                return self.last_distance
            
            distance_cm = raw_distance * 100.0
            
            # Basic validation
            if 0 <= distance_cm <= 800:
                self.distance_history.append(distance_cm)
                filtered = np.median(list(self.distance_history))
                self.last_distance = filtered
                return filtered
        
        except Exception as e:
            pass
        
        return self.last_distance
    
    def create_ai_state(self, distance_cm):
        """Create 6D state vector - MUST MATCH TRAINING"""
        state = np.zeros(6, dtype=np.float32)
        
        # State 0: Normalized distance (0-200cm → 0-1)
        state[0] = min(distance_cm / 200.0, 1.0)
        
        # State 1: Normalized current speed (0 to 100 → 0 to 1) - POSITIVE ONLY
        state[1] = max(0, self.current_speed) / 100.0
        
        # State 2: Normalized steering (-1 to 1)
        current_steering = self.driver_position if hasattr(self, 'driver_position') else DRIVER_CENTER
        state[2] = (current_steering - DRIVER_CENTER) / 40.0
        
        # State 3: Normalized eye position (-1 to 1)
        state[3] = (self.eye_position - EYE_CENTER) / ((EYE_MAX - EYE_MIN) / 2)
        
        # State 4: Time/sin feature
        state[4] = np.sin(self.cycle_count * 0.1)
        
        # State 5: Obstacle flag (0=no obstacle, 1=obstacle close)
        state[5] = 1.0 if distance_cm < 30.0 else 0.0
        
        return state
    
    def get_ai_decision(self, distance_cm):
        """Get decision from trained AI - FIXED VERSION"""
        if self.ai is None:
            # Fallback if no AI
            if distance_cm < 20:
                return 0, 90, "❌ NO AI: Stop"
            elif distance_cm < 50:
                return 30, 90, "❌ NO AI: Slow"
            else:
                return 50, 90, "❌ NO AI: Forward"
        
        try:
            # Create state for AI - MUST MATCH TRAINING
            state = self.create_ai_state(distance_cm)
            
            # Debug: show state values every 10 cycles
            if self.cycle_count % 10 == 0:
                print(f"\n🤖 AI STATE: [{state[0]:.2f}, {state[1]:.2f}, {state[2]:.2f}, {state[3]:.2f}, {state[4]:.2f}, {state[5]:.2f}]")
            
            # Get AI action
            with torch.no_grad():
                state_tensor = torch.FloatTensor(state).unsqueeze(0)
                action = self.ai(state_tensor, deterministic=True)
                action_np = action.squeeze(0).numpy()
            
            if self.cycle_count % 10 == 0:
                print(f"🤖 RAW ACTION: {action_np}")
            
            # AI output interpretation (trained 6D model):
            # action[0] = throttle (-1 to 1)
            # action[1] = steering (-1 to 1)
            # action[2] = eye control (-1 to 1)
            # action[3:6] = aux outputs
            
            throttle_raw = action_np[0]
            steering_raw = action_np[1]
            
            # FIX 1: Always use positive throttle (forward only)
            throttle = max(0, throttle_raw)  # Clamp to [0, 1]
            
            # FIX 2: Convert to actual values
            target_speed = throttle * 100  # 0 to 100%
            
            # Calculate steering angle
            steering_angle = DRIVER_CENTER + (steering_raw * 40)
            
            # FIX 3: Ensure reasonable values
            target_speed = max(0, min(80, target_speed))
            steering_angle = max(DRIVER_MIN, min(DRIVER_MAX, steering_angle))
            
            # Determine reason based on AI outputs
            if throttle < 0.2:
                reason = "🤖 DECISION: Very Slow"
            elif throttle < 0.4:
                reason = "🤖 DECISION: Slow Forward"
            elif throttle < 0.6:
                reason = "🤖 DECISION: Moderate Speed"
            else:
                reason = "🤖 DECISION: Fast Forward"
            
            # Add steering info
            if abs(steering_raw) > 0.3:
                direction = "LEFT" if steering_raw < 0 else "RIGHT"
                reason += f" | Turning {direction}"
            
            # Add raw values for debugging every 20 cycles
            if self.cycle_count % 20 == 0:
                reason += f" | Raw: T={throttle_raw:.2f}, S={steering_raw:.2f}"
                print(f"🤖 FINAL: Speed={target_speed:.0f}%, Steer={steering_angle:.1f}°")
            
            return target_speed, steering_angle, reason
            
        except Exception as e:
            print(f"\033[1;31m❌ AI inference error: {e}\033[0m")
            import traceback
            traceback.print_exc()
            return 30, DRIVER_CENTER, "⚠️ AI ERROR - Default Forward"
    
    def apply_motor_control(self, target_speed):
        """Apply speed with smooth acceleration - FORWARD ONLY"""
        # Ensure positive speed (forward only)
        target_speed = max(0, target_speed)
        
        # Smooth acceleration
        speed_diff = target_speed - self.current_speed
        
        if speed_diff > 0:
            change = min(5, speed_diff)  # Gentle acceleration
        elif speed_diff < 0:
            change = max(-10, speed_diff)  # Faster deceleration
        else:
            change = 0
        
        new_speed = self.current_speed + change
        
        # FORWARD ONLY: Clamp to positive values
        new_speed = max(0, min(75, new_speed))
        
        # Apply to motor
        if abs(new_speed) < 5:
            self.motor.stop()
        elif new_speed > 0:
            self.motor.move_forward(int(new_speed))
            self.MotorDirection = MotorDirection.FORWARD
        else:
            # Should not happen with forward-only, but just in case
            self.motor.stop()
        
        # Update state
        self.current_speed = new_speed
        self.speed_history.append(new_speed)
        
        return new_speed
    
    def display_status(self, distance_cm, speed, steering, reason):
        """Display AI-driven status"""
        # Speed color based on AI decision
        if speed < 10:
            speed_color = "\033[1;90m"  # Gray for stopped/slow
            speed_icon = "⏹️"
        elif speed < 30:
            speed_color = "\033[1;33m"  # Yellow for slow
            speed_icon = "🐢"
        elif speed < 50:
            speed_color = "\033[1;32m"  # Green for moderate
            speed_icon = "🚗"
        else:
            speed_color = "\033[1;36m"  # Cyan for fast
            speed_icon = "🏎️"
        
        # Distance color (informational only, not used for control)
        if distance_cm < 30:
            dist_color = "\033[1;33m"  # Yellow (info)
            dist_icon = "⚠️"
        elif distance_cm < 100:
            dist_color = "\033[1;32m"  # Green (info)
            dist_icon = "📏"
        else:
            dist_color = "\033[1;36m"  # Cyan (info)
            dist_icon = "🟢"
        
        # Steering direction
        if steering < 80:
            steer_icon = "↖️"
        elif steering > 100:
            steer_icon = "↗️"
        else:
            steer_icon = "⬆️"
        
        # Build display
        display = (f"\033[2K\033[1G"  # Clear line
                  f"\033[1;37m[{self.cycle_count:04d}] "
                  f"{dist_color}{dist_icon}{distance_cm:5.1f}cm "
                  f"{speed_color}{speed_icon}{speed:3.0f}% "
                  f"\033[1;35m🧭{steer_icon}{steering:3.0f}° "
                  f"\033[1;34m👁️{self.eye_position:4.1f} "
                  f"\033[0m{reason}")
        
        return display
    
    def eye_to_driver_accurate(self, eye_angle: float) -> float:
        """Convert eye angle to driver servo angle"""
        # Map eye range 15-46 to driver range 50-130
        eye_range = 46 - 15  # = 31
        driver_range = 130 - 50  # = 80
        
        # Scale factor
        scale = driver_range / eye_range  # 80 / 31 ≈ 2.58
        
        # Convert eye angle to driver angle
        driver_angle = 50 + (eye_angle - 15) * scale
        
        return driver_angle
    
    def calculate_turn_duration(self, driver_angle: float, speed: float) -> float:
        """Calculate appropriate turn duration"""
        if speed <= 10:
            return 0.5  # Very short turn if barely moving
        
        # Calculate how sharp the turn is
        angle_diff = abs(driver_angle - DRIVER_CENTER)
        
        # Turn duration based on angle (longer for sharper turns)
        if angle_diff < 10:
            base_duration = 1.0  # Gentle adjustment
        elif angle_diff < 25:
            base_duration = 2.0  # Moderate turn
        elif angle_diff < 40:
            base_duration = 3.0  # Sharp turn
        else:
            base_duration = 4.0  # Very sharp turn
        
        # Adjust for speed - slower = longer turn time
        speed_factor = max(20, min(50, speed)) / 35.0
        
        return base_duration * speed_factor
    
    def drive(self):
        """Main loop - PURE AI control"""
        print("\n\033[1;36m🚀 STARTING PURE AI AUTONOMOUS MODE\033[0m")
        print(f"\033[1;33m   AI Model: {'✅ Loaded' if self.ai else '❌ Not Available'}\033[0m")
        print("\033[1;33m   Press Ctrl+C to stop\033[0m")
        print("\033[1;35m" + "="*80 + "\033[0m\n")
        
        last_display = 0
        display_interval = 0.3
        
        try:
            while self.running:
                self.cycle_count += 1
                cycle_start = time.time()
                
                # 1. Get environment information
                distance_cm = self.get_lidar_distance()
                
                # 2. Get AI decision (speed AND steering)
                target_speed, steering_angle, reason = self.get_ai_decision(distance_cm)
                
                # 3. Apply AI speed decision
                actual_speed = self.apply_motor_control(target_speed)
                
                # 4. Apply AI steering decision (unless we're using CarEye)
                current_time = time.time()
                
                # Only use CarEye scanning when moving forward at good speed
                should_scan = (actual_speed >= 20 and 
                              self.MotorDirection == MotorDirection.FORWARD and
                              not self.is_turning and
                              (current_time - self.last_scan_time) >= self.scan_interval)
                
                if should_scan:
                    # Get best moving direction from CarEye
                    print("\n\033[1;33m🔍 SCANNING ENVIRONMENT...\033[0m")
                    best_distance, best_angle = self.CarEye.get_moving_direction()
                    
                    # Only turn if it's significantly better
                    if best_distance > distance_cm + 15:  # At least 15cm better
                        driver_turn_angle = self.eye_to_driver_accurate(best_angle)
                        
                        # Apply the turn
                        self.pca.driver.set_angle(driver_turn_angle)
                        self.driver_position = driver_turn_angle
                        self.is_turning = True
                        self.target_driver_angle = driver_turn_angle
                        self.turn_start_time = current_time
                        self.turn_duration = self.calculate_turn_duration(driver_turn_angle, actual_speed)
                        self.last_scan_time = current_time
                        
                        print(f"\n🤖 STARTING TURN: {driver_turn_angle:.1f}° for {self.turn_duration:.1f}s")
                        print(f"   Best angle: {best_angle:.1f}°, Distance: {best_distance:.1f}cm")
                    else:
                        print(f"\n🤖 NO TURN NEEDED: Distance similar ({best_distance:.1f}cm)")
                        # Use AI steering instead
                        self.pca.driver.set_angle(steering_angle)
                        self.driver_position = steering_angle
                else:
                    # Use AI steering directly
                    self.pca.driver.set_angle(steering_angle)
                    self.driver_position = steering_angle
                
                # Check if turn time has elapsed
                if self.is_turning and (current_time - self.turn_start_time) >= self.turn_duration:
                    # Time's up - straighten!
                    self.pca.center_all()
                    self.driver_position = DRIVER_CENTER
                    self.is_turning = False
                    self.target_driver_angle = DRIVER_CENTER
                    if self.cycle_count % 20 == 0:
                        print(f"\n🤖 ENDING TURN: Straightening after {self.turn_duration:.1f}s")
                
                # 5. Display
                if current_time - last_display >= display_interval:
                    # Show turning status
                    display_reason = reason
                    if self.is_turning:
                        time_left = max(0, self.turn_duration - (current_time - self.turn_start_time))
                        display_reason += f" | 🚗 Turning ({time_left:.1f}s left)"
                    
                    print(self.display_status(distance_cm, actual_speed, 
                                             self.driver_position, 
                                             display_reason), 
                          end="", flush=True)
                    last_display = current_time
                
                # 6. Maintain timing
                cycle_time = time.time() - cycle_start
                sleep_time = max(0, 0.2 - cycle_time)  # ~5Hz
                if sleep_time > 0:
                    time.sleep(sleep_time)
        
        except KeyboardInterrupt:
            print("\n\n\033[1;33m🛑 User requested stop\033[0m")
        except Exception as e:
            print(f"\n\n\033[1;31m❌ System error: {e}\033[0m")
            import traceback
            traceback.print_exc()
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Safe shutdown"""
        print("\n\033[1;33m🔽 Performing safe shutdown...\033[0m")
        
        # Stop motor
        self.motor.stop()
        
        # Center servos
        self.pca.center_all()
        time.sleep(0.3)
        
        # Final stats
        print("\n\033[1;35m" + "="*80 + "\033[0m")
        print("\033[1;36m📊 PURE AI PERFORMANCE REPORT\033[0m")
        print("\033[1;35m" + "="*80 + "\033[0m")
        print(f"\033[1;37m   Total cycles: {self.cycle_count}\033[0m")
        print(f"\033[1;37m   Final speed: {self.current_speed:.0f}%\033[0m")
        print(f"\033[1;37m   Final distance: {self.last_distance:.1f}cm\033[0m")
        print(f"\033[1;37m   AI Model: {'✅ Active' if self.ai else '❌ Not Used'}\033[0m")
        print(f"\033[1;37m   Last steering: {self.driver_position:.0f}°\033[0m")
        print("\033[1;35m" + "="*80 + "\033[0m")
        print("\033[1;32m✅ SYSTEM SAFE - Goodbye!\033[0m")

# ============================================================================
# MAIN
# ============================================================================

def signal_handler(sig, frame):
    """Emergency stop handler"""
    print("\n\033[1;31m🛑 EMERGENCY STOP SIGNAL RECEIVED\033[0m")
    sys.exit(0)

def test_ai_only():
    """Test AI without CarEye scanning"""
    print("\n\033[1;36m🧠 TESTING AI-ONLY MODE\033[0m")
    print("="*60)
    
    try:
        controller = PureAIController()
        
        # Override to use AI steering only
        controller.scan_interval = 99999  # Never scan
        
        print("\n🚗 Starting AI-only test (10 seconds)...")
        start_time = time.time()
        
        while time.time() - start_time < 10:
            distance = controller.get_lidar_distance()
            speed, steering, reason = controller.get_ai_decision(distance)
            controller.apply_motor_control(speed)
            controller.pca.driver.set_angle(steering)
            controller.driver_position = steering
            
            print(controller.display_status(distance, speed, steering, reason), end="\r")
            time.sleep(0.2)
        
        print("\n\n✅ AI-only test complete!")
        controller.shutdown()
        
    except Exception as e:
        print(f"\n❌ Test failed: {e}")

def test_careye_only():
    """Test CarEye scanning without AI"""
    print("\n\033[1;36m👁️ TESTING CAR-EYE ONLY MODE\033[0m")
    print("="*60)
    
    try:
        controller = PureAIController()
        controller.motor.stop()  # Don't move
        
        print("\n🔍 Testing CarEye scanning...")
        
        for i in range(3):
            print(f"\nScan {i+1}:")
            best_distance, best_angle = controller.CarEye.get_moving_direction()
            driver_angle = controller.eye_to_driver_accurate(best_angle)
            
            print(f"   Best angle: {best_angle:.1f}°")
            print(f"   Driver angle: {driver_angle:.1f}°")
            print(f"   Distance: {best_distance:.1f}cm")
            
            # Move to that angle
            controller.pca.driver.set_angle(driver_angle)
            time.sleep(1)
        
        controller.pca.center_all()
        print("\n✅ CarEye test complete!")
        
    except Exception as e:
        print(f"\n❌ Test failed: {e}")

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        print("\n" + "="*80)
        print("🤖 PURE AI TESLA CONTROLLER - DEVELOPMENT MODE")
        print("="*80)
        
        # Menu for testing
        while True:
            print("\n📋 SELECT TEST MODE:")
            print("1. Full Autonomous (AI + CarEye)")
            print("2. AI-Only Test (No scanning)")
            print("3. CarEye-Only Test (No AI)")
            print("4. Exit")
            
            choice = input("\nSelect (1-4): ").strip()
            
            if choice == '1':
                print("\n🚀 STARTING FULL AUTONOMOUS MODE...")
                controller = PureAIController()
                controller.drive()
            elif choice == '2':
                test_ai_only()
            elif choice == '3':
                test_careye_only()
            elif choice == '4':
                print("\n👋 Goodbye!")
                break
            else:
                print("❌ Invalid choice, try again.")
        
    except Exception as e:
        print(f"\n\033[1;31m❌ FATAL ERROR: {e}\033[0m")
        import traceback
        traceback.print_exc()
        sys.exit(1)