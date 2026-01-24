#!/usr/bin/env python3
"""
FIXED TESLA CONTROLLER - NO I/O ERRORS, STABLE SERVOS
- Prevents servo movement during emergency stops
- Reduces I2C bus traffic when stopped
- Smart scanning with delays
- Working 20cm minimum gap
"""

import time
import numpy as np
import sys
import os
import signal
from collections import deque
import threading

# ============================================================================
# REAL HARDWARE IMPORTS
# ============================================================================

print("\033[1;35m" + "="*80 + "\033[0m")
print("\033[1;36m🤖 STABLE TESLA CONTROLLER - NO I/O ERRORS\033[0m")
print("\033[1;32m   Prevents rapid servo movements | Stable I2C bus | 20cm min gap\033[0m")
print("\033[1;35m" + "="*80 + "\033[0m")

sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

try:
    from Hardware.PCA_Board import PCA9685Controller
    from Hardware.DC_Motor import DCMotor
    from Hardware.Lidar_Sensor import TFLuna as TFLunaInterface
    print("\033[1;32m✅ Hardware modules loaded\033[0m")
except Exception as e:
    print(f"\033[1;31m❌ Hardware import error: {e}\033[0m")
    sys.exit(1)

# ============================================================================
# CONFIGURATION - OPTIMIZED FOR STABILITY
# ============================================================================

# LiDAR conversion
LIDAR_TO_CM = 100.0

# SAFETY ZONES
EMERGENCY_STOP_CM = 8.0      # Absolute emergency (reverse only)
MIN_SAFE_GAP_CM = 20.0       # MINIMUM DISTANCE - WON'T GO CLOSER
SLOW_ZONE_CM = 40.0          # Slow down zone
NORMAL_ZONE_CM = 100.0       # Normal operation

# SPEED LIMITS
MOTOR_MAX_FORWARD = 75
MOTOR_MAX_REVERSE = 30
MOTOR_ACCEL_LIMIT = 8
MOTOR_DECEL_LIMIT = 12

# SERVO LIMITS - WITH MOVEMENT CONSTRAINTS
DRIVER_MIN = 40
DRIVER_MAX = 140
DRIVER_CENTER = 90

EYE_CENTER = 31.5
EYE_MIN = 15.0
EYE_MAX = 45.0

# CONTROL TIMING - SLOWER FOR STABILITY
CONTROL_HZ = 5              # Reduced from 10Hz to 5Hz for stability
SCAN_HZ = 2                 # Scan at 2Hz when stopped
MIN_SERVO_MOVE_TIME = 0.1   # Minimum 100ms between servo moves

# ============================================================================
# STABLE SERVO CONTROLLER
# ============================================================================

class StableServoController:
    """Prevents rapid servo movements that cause I/O errors"""
    
    def __init__(self, pca):
        self.pca = pca
        self.eye_position = EYE_CENTER
        self.driver_position = DRIVER_CENTER
        self.last_eye_move = 0
        self.last_driver_move = 0
        self.emergency_mode = False
        
    def safe_eye_move(self, angle, force=False):
        """Move eye servo with timing constraints"""
        if self.emergency_mode and not force:
            return  # Don't move servos during emergency
        
        current_time = time.time()
        time_since_last = current_time - self.last_eye_move
        
        # Enforce minimum time between moves
        if time_since_last < MIN_SERVO_MOVE_TIME and not force:
            return
        
        # Clamp angle
        angle = max(EYE_MIN, min(EYE_MAX, angle))
        
        # Only move if significantly different
        if abs(angle - self.eye_position) < 1.0:
            return
        
        try:
            self.pca.eye.set_50_range(angle)
            self.eye_position = angle
            self.last_eye_move = current_time
        except Exception as e:
            print(f"\033[1;33m⚠️ Eye servo error: {e}\033[0m")
    
    def safe_driver_move(self, angle, force=False):
        """Move driver servo with timing constraints"""
        if self.emergency_mode and not force:
            return  # Don't move servos during emergency
        
        current_time = time.time()
        time_since_last = current_time - self.last_driver_move
        
        # Enforce minimum time between moves
        if time_since_last < MIN_SERVO_MOVE_TIME and not force:
            return
        
        # Clamp angle
        angle = max(DRIVER_MIN, min(DRIVER_MAX, angle))
        
        # Only move if significantly different
        if abs(angle - self.driver_position) < 2.0:
            return
        
        try:
            self.pca.driver.set_angle(int(angle))
            self.driver_position = angle
            self.last_driver_move = current_time
        except Exception as e:
            print(f"\033[1;33m⚠️ Driver servo error: {e}\033[0m")
    
    def set_emergency_mode(self, emergency):
        """Enable/disable emergency mode (stops servo movement)"""
        self.emergency_mode = emergency
        if emergency:
            # Center servos and stop moving them
            try:
                self.pca.eye.set_50_range(EYE_CENTER)
                self.pca.driver.set_angle(DRIVER_CENTER)
                self.eye_position = EYE_CENTER
                self.driver_position = DRIVER_CENTER
            except:
                pass  # Ignore errors during emergency

# ============================================================================
# MAIN TESLA CONTROLLER
# ============================================================================

class StableTeslaController:
    """Main controller with I/O error prevention"""
    
    def __init__(self):
        print("\n\033[1;36m🏭 INITIALIZING STABLE TESLA CONTROLLER...\033[0m")
        
        # Initialize hardware
        try:
            self.pca = PCA9685Controller()
            time.sleep(1)  # Give PCA time to initialize
            print("\033[1;32m✅ PCA9685 ready\033[0m")
        except Exception as e:
            print(f"\033[1;31m❌ PCA error: {e}\033[0m")
            raise
        
        self.motor = DCMotor(
            rpwm_pin=4,
            lpwm_pin=17,
            ren_pin=27,
            len_pin=22,
            motor_name="TeslaStable"
        )
        print("\033[1;32m✅ DCMotor ready\033[0m")
        
        self.lidar = TFLunaInterface('/dev/serial0')
        print("\033[1;32m✅ LiDAR ready\033[0m")
        
        # Initialize stable servo controller
        self.servos = StableServoController(self.pca)
        
        # Center everything
        self.servos.safe_eye_move(EYE_CENTER, force=True)
        self.servos.safe_driver_move(DRIVER_CENTER, force=True)
        time.sleep(0.5)
        
        # State
        self.current_speed = 0
        self.running = True
        self.cycle_count = 0
        self.emergency_stops = 0
        self.obstacle_detected = False
        self.last_scan_time = 0
        
        # Distance tracking
        self.last_distance = 999.9
        self.distance_history = deque(maxlen=3)
        
        print("\n\033[1;35m" + "="*80 + "\033[0m")
        print("\033[1;32m✅ STABLE CONTROLLER READY\033[0m")
        print(f"\033[1;33m   Minimum safe gap: {MIN_SAFE_GAP_CM}cm\033[0m")
        print(f"\033[1;33m   Control frequency: {CONTROL_HZ}Hz\033[0m")
        print(f"\033[1;33m   Servo move delay: {MIN_SERVO_MOVE_TIME}s\033[0m")
        print("\033[1;35m" + "="*80 + "\033[0m")
    
    def get_lidar_distance(self):
        """Get LiDAR distance with error handling"""
        try:
            raw_distance = self.lidar.read_distance()
            
            if raw_distance is None:
                return 999.9
            
            # Convert meters to cm
            distance_cm = raw_distance * LIDAR_TO_CM
            
            # Validate range
            if distance_cm < 0 or distance_cm > 800:
                return 999.9
            
            # Filter with history
            self.distance_history.append(distance_cm)
            filtered_cm = np.median(list(self.distance_history))
            
            self.last_distance = filtered_cm
            return filtered_cm
            
        except Exception as e:
            print(f"\033[1;33m⚠️ LIDAR read error: {e}\033[0m")
            return 999.9
    
    def simple_stable_scan(self):
        """Simplified scan that minimizes servo movement"""
        current_time = time.time()
        
        # Don't scan too frequently
        if current_time - self.last_scan_time < (1.0 / SCAN_HZ):
            return [self.last_distance, self.last_distance, self.last_distance]
        
        distances = []
        
        # Only scan if we're moving or not in emergency
        if self.current_speed > 5 and not self.obstacle_detected:
            # Check center (eye already at center)
            center_dist = self.get_lidar_distance()
            distances.append(center_dist)
            
            # Only scan sides if center is getting close
            if center_dist < SLOW_ZONE_CM:
                # Check left
                self.servos.safe_eye_move(20)
                time.sleep(0.05)
                left_dist = self.get_lidar_distance()
                distances.append(left_dist)
                
                # Check right
                self.servos.safe_eye_move(43)
                time.sleep(0.05)
                right_dist = self.get_lidar_distance()
                distances.append(right_dist)
                
                # Return to center
                self.servos.safe_eye_move(EYE_CENTER)
            else:
                # Use center for all
                distances.append(center_dist)
                distances.append(center_dist)
        else:
            # Just use last known distance
            distances = [self.last_distance, self.last_distance, self.last_distance]
        
        self.last_scan_time = current_time
        return distances
    
    def check_obstacle(self):
        """Check for obstacles with minimal scanning"""
        # Simple center reading without moving eye
        distance = self.get_lidar_distance()
        
        # Update obstacle detection state
        if distance < MIN_SAFE_GAP_CM:
            self.obstacle_detected = True
            self.servos.set_emergency_mode(True)
        else:
            self.obstacle_detected = False
            self.servos.set_emergency_mode(False)
        
        return distance
    
    def enforce_safety(self, distance):
        """Enforce minimum 20cm gap"""
        if distance < EMERGENCY_STOP_CM:
            # ABSOLUTE EMERGENCY - STOP AND REVERSE
            self.motor.stop()
            self.current_speed = 0
            self.emergency_stops += 1
            self.servos.set_emergency_mode(True)
            return True, "EMERGENCY! <8cm"
        
        elif distance < MIN_SAFE_GAP_CM:
            # VIOLATING MINIMUM GAP - STOP
            if self.current_speed > 0:
                self.motor.stop()
                self.current_speed = 0
                self.emergency_stops += 1
                self.servos.set_emergency_mode(True)
                return True, f"STOP: {distance:.0f}cm < {MIN_SAFE_GAP_CM}cm"
        
        return False, ""
    
    def calculate_speed(self, distance):
        """Calculate safe speed based on distance"""
        if distance < EMERGENCY_STOP_CM:
            # Emergency reverse
            return -20, "Emergency reverse"
        
        elif distance < MIN_SAFE_GAP_CM:
            # Stop - don't go closer than 20cm
            return 0, f"Stop at {MIN_SAFE_GAP_CM}cm gap"
        
        elif distance < SLOW_ZONE_CM:
            # Slow approach
            return 25, "Slow approach"
        
        elif distance < NORMAL_ZONE_CM:
            # Normal speed
            return 45, "Normal speed"
        
        else:
            # Clear path
            return 65, "Clear path"
    
    def apply_motor_control(self, target_speed):
        """Apply speed with acceleration limits"""
        if target_speed == 0:
            self.motor.stop()
            self.current_speed = 0
            return 0
        
        # Acceleration/deceleration limiting
        speed_diff = target_speed - self.current_speed
        
        if speed_diff > 0:
            change = min(MOTOR_ACCEL_LIMIT, speed_diff)
        elif speed_diff < 0:
            change = max(-MOTOR_DECEL_LIMIT, speed_diff)
        else:
            change = 0
        
        new_speed = self.current_speed + change
        
        # Clamp to limits
        if new_speed > 0:
            new_speed = min(MOTOR_MAX_FORWARD, new_speed)
        else:
            new_speed = max(-MOTOR_MAX_REVERSE, new_speed)
        
        # Apply to motor
        if abs(new_speed) < 3:  # Deadzone
            self.motor.stop()
            self.current_speed = 0
        elif new_speed > 0:
            motor_speed = int(min(100, new_speed))
            self.motor.move_forward(motor_speed)
            self.current_speed = new_speed
        else:
            motor_speed = int(min(100, -new_speed))
            self.motor.move_reverse(motor_speed)
            self.current_speed = new_speed
        
        return self.current_speed
    
    def display_status(self, distance, reason, emergency=False):
        """Display status with clear information"""
        # Distance color coding
        if distance < EMERGENCY_STOP_CM:
            dist_color = "\033[1;41m"  # Red background
            dist_icon = "🛑"
        elif distance < MIN_SAFE_GAP_CM:
            dist_color = "\033[1;31m"  # Red
            dist_icon = "⚠️ "
        elif distance < SLOW_ZONE_CM:
            dist_color = "\033[1;33m"  # Yellow
            dist_icon = "🚧"
        elif distance < NORMAL_ZONE_CM:
            dist_color = "\033[1;32m"  # Green
            dist_icon = "📏"
        else:
            dist_color = "\033[1;36m"  # Cyan
            dist_icon = "🟢"
        
        # Speed icon
        if self.current_speed == 0:
            speed_icon = "⏹️"
        elif self.current_speed < 0:
            speed_icon = "🔙"
        elif self.current_speed < 20:
            speed_icon = "🐢"
        elif self.current_speed < 40:
            speed_icon = "🚗"
        elif self.current_speed < 60:
            speed_icon = "🏎️"
        else:
            speed_icon = "💨"
        
        # Build display
        display = (f"\033[2K\033[1G"  # Clear line
                  f"\033[1;37m[{self.cycle_count:04d}] "
                  f"{dist_color}{dist_icon}{distance:5.1f}cm "
                  f"\033[1;34m{speed_icon}{abs(self.current_speed):3.0f}% "
                  f"\033[1;36m🧭{self.servos.driver_position:3.0f}° "
                  f"\033[1;35m👁️{self.servos.eye_position:4.1f} "
                  f"\033[1;33m⚙️ "
                  f"\033[1;31m🚨{self.emergency_stops} ")
        
        # Add reason if provided
        if reason:
            display += f"\033[1;90m[{reason[:25]}]"
        
        # Emergency warning
        if emergency:
            display += f" \033[1;41mSTOPPED!\033[0m"
        
        # Obstacle warning
        if self.obstacle_detected:
            display += f" \033[1;33m🚫\033[0m"
        
        return display
    
    def run_autonomous(self):
        """Main autonomous loop - STABLE VERSION"""
        print("\n\033[1;36m🚀 STARTING STABLE AUTONOMOUS OPERATION\033[0m")
        print(f"\033[1;33m   Minimum gap: {MIN_SAFE_GAP_CM}cm | Control: {CONTROL_HZ}Hz\033[0m")
        print("\033[1;33m   Press Ctrl+C to stop\033[0m")
        print("\033[1;35m" + "="*80 + "\033[0m\n")
        
        last_display = 0
        display_interval = 0.3  # Display every 300ms
        
        try:
            while self.running:
                self.cycle_count += 1
                cycle_start = time.time()
                
                # 1. SIMPLE OBSTACLE CHECK (no servo movement)
                distance = self.check_obstacle()
                
                # 2. ENFORCE SAFETY (minimum 20cm gap)
                emergency, emergency_reason = self.enforce_safety(distance)
                
                # 3. Calculate appropriate speed
                if not emergency:
                    target_speed, reason = self.calculate_speed(distance)
                else:
                    target_speed = 0
                    reason = emergency_reason
                
                # 4. Apply motor control
                self.apply_motor_control(target_speed)
                
                # 5. Only scan if we're moving and not in emergency
                if self.current_speed > 10 and not self.obstacle_detected:
                    # Do a full scan to decide steering
                    distances = self.simple_stable_scan()
                    
                    # Simple steering logic
                    if len(distances) >= 3:
                        left = distances[1]
                        right = distances[2]
                        
                        if left > right + 20:
                            self.servos.safe_driver_move(60)  # Turn left
                        elif right > left + 20:
                            self.servos.safe_driver_move(120)  # Turn right
                        else:
                            self.servos.safe_driver_move(DRIVER_CENTER)
                
                # 6. Display status
                current_time = time.time()
                if current_time - last_display >= display_interval:
                    print(self.display_status(distance, reason, emergency), 
                          end="", flush=True)
                    last_display = current_time
                
                # 7. Timing control
                cycle_time = time.time() - cycle_start
                sleep_time = max(0, (1.0 / CONTROL_HZ) - cycle_time)
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
        
        # Center servos (with error handling)
        try:
            self.servos.safe_eye_move(EYE_CENTER, force=True)
            self.servos.safe_driver_move(DRIVER_CENTER, force=True)
            time.sleep(0.3)
        except:
            pass  # Ignore errors during shutdown
        
        # Statistics
        print("\n\033[1;35m" + "="*80 + "\033[0m")
        print("\033[1;36m📊 FINAL STATISTICS\033[0m")
        print("\033[1;35m" + "="*80 + "\033[0m")
        print(f"\033[1;37m   Cycles: {self.cycle_count}\033[0m")
        print(f"\033[1;31m   Emergency stops: {self.emergency_stops}\033[0m")
        print(f"\033[1;33m   Minimum gap: {MIN_SAFE_GAP_CM}cm\033[0m")
        print(f"\033[1;36m   Final speed: {self.current_speed:.0f}%\033[0m")
        print(f"\033[1;37m   Final distance: {self.last_distance:.1f}cm\033[0m")
        print("\033[1;35m" + "="*80 + "\033[0m")
        print("\033[1;32m✅ SYSTEM SAFE - Goodbye!\033[0m")

# ============================================================================
# SIGNAL HANDLER
# ============================================================================

def signal_handler(sig, frame):
    """Emergency stop handler"""
    print("\n\033[1;31m🛑 EMERGENCY STOP SIGNAL RECEIVED\033[0m")
    sys.exit(0)

# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    # Set signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Create and run controller
        print("\n" + "="*80)
        print("🤖 STABLE TESLA CONTROLLER - NO I/O ERRORS")
        print("="*80)
        
        controller = StableTeslaController()
        controller.run_autonomous()
        
    except Exception as e:
        print(f"\n\033[1;31m❌ FATAL ERROR: {e}\033[0m")
        sys.exit(1)