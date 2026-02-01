#!/usr/bin/env python3
import time
import sys
import os
import signal
import tty
import termios
import select
from threading import Thread
import numpy as np

sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

# ============================================================================
# HARDWARE
# ============================================================================
try:
    from Hardware.PCA_Board import PCA9685Controller
    from Hardware.DC_Motor import DCMotor
    from Hardware.Lidar_Sensor import TFLuna as TFLunaInterface
    print("✅ Hardware modules loaded")
except Exception as e:
    print(f"❌ Hardware import error: {e}")
    sys.exit(1)

# ============================================================================
# MANUAL CONTROL CAR WITH CRUISE CONTROL
# ============================================================================
class ManualControlCar:
    def __init__(self):
        print("\n" + "=" * 60)
        print("🎮 MANUAL CONTROL CAR")
        print("=" * 60)
        print("Controls:")
        print("  W     - Accelerate Forward")
        print("  S     - Accelerate Reverse")
        print("  A     - Turn Left")
        print("  D     - Turn Right")
        print("  SPACE - Brake (stop immediately)")
        print("  E     - Toggle Cruise Control")
        print("  C     - Center steering")
        print("  X     - Emergency stop & shutdown")
        print("  ESC   - Quit")
        print("=" * 60)
        
        # Initialize hardware
        self.pca = PCA9685Controller()
        self.motor = DCMotor(4, 17, 27, 22, "ManualControl")
        self.lidar = TFLunaInterface('/dev/serial0')
        
        # Center everything
        print("\n🎯 Initializing hardware...")
        self.pca.driver.set_angle(90)
        self.pca.eye.set_50_range(31.5)
        time.sleep(0.5)
        
        # Control parameters
        self.steering_angle = 90
        self.throttle = 0
        self.target_throttle = 0
        self.eye_position = 31.5
        
        # Steering settings
        self.STEERING_CENTER = 90
        self.STEERING_STEP = 8  # Degrees per keypress
        
        # Throttle settings
        self.THROTTLE_ACCEL = 10  # Acceleration step
        self.THROTTLE_DECEL = 20  # Braking step
        self.MAX_THROTTLE = 75
        self.MIN_THROTTLE = -75
        
        # Cruise control
        self.cruise_control = False
        self.cruise_speed = 0
        
        # Eye settings
        self.EYE_CENTER = 31.5
        self.EYE_STEP = 5
        
        # Status flags
        self.running = True
        self.braking = False
        self.emergency_stop = False
        
        # Initialize keyboard
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        # Test hardware
        self.test_hardware()
    
    def test_hardware(self):
        """Quick hardware test"""
        print("\n🧪 Testing hardware...")
        
        # Test steering
        print("  Testing steering...")
        for angle in [70, 90, 110]:
            self.pca.driver.set_angle(angle)
            time.sleep(0.2)
        self.pca.driver.set_angle(90)
        
        # Test motor
        print("  Testing motor...")
        self.motor.move_forward(30)
        time.sleep(0.3)
        self.motor.stop()
        time.sleep(0.1)
        self.motor.move_reverse(30)
        time.sleep(0.3)
        self.motor.stop()
        
        # Test eye
        print("  Testing eye...")
        for pos in [25.0, 31.5, 38.0]:
            self.pca.eye.set_50_range(pos)
            time.sleep(0.2)
        self.pca.eye.set_50_range(31.5)
        
        print("✅ Hardware test complete\n")
        time.sleep(0.5)
    
    def get_key(self):
        """Get a single key press without waiting for Enter"""
        try:
            tty.setraw(sys.stdin.fileno())
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                # Handle arrow keys and escape
                if key == '\x1b':  # ESC
                    key += sys.stdin.read(2) if select.select([sys.stdin], [], [], 0.1)[0] else ''
                return key
            return ''
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def apply_controls(self):
        """Apply steering and throttle controls"""
        # Apply steering
        self.pca.driver.set_angle(self.steering_angle)
        
        # Smooth throttle transitions
        if self.braking:
            # Hard brake - go directly to 0
            self.throttle = 0
        elif abs(self.target_throttle - self.throttle) < self.THROTTLE_ACCEL:
            self.throttle = self.target_throttle
        elif self.target_throttle > self.throttle:
            self.throttle += self.THROTTLE_ACCEL
        elif self.target_throttle < self.throttle:
            self.throttle -= self.THROTTLE_ACCEL
        
        # Clamp throttle
        self.throttle = max(self.MIN_THROTTLE, min(self.MAX_THROTTLE, self.throttle))
        
        # Apply motor control
        if abs(self.throttle) < 5:
            self.motor.stop()
        elif self.throttle > 0:
            self.motor.move_forward(int(self.throttle))
        else:
            self.motor.move_reverse(int(-self.throttle))
        
        # Apply eye position
        self.pca.eye.set_50_range(self.eye_position)
    
    def update_display(self):
        """Update status display"""
        # Get distance for safety display
        try:
            dist_m = self.lidar.get_distance_to_obstacle()
            distance_cm = dist_m * 100 if dist_m else 999
        except:
            distance_cm = 999
        
        # Clear line and update
        print(f"\033[2K\033[1G", end="")
        
        # Steering indicator
        steering_bar = "["
        for i in range(50, 131, 10):
            if i <= self.steering_angle:
                steering_bar += "█"
            else:
                steering_bar += " "
        steering_bar += "]"
        
        # Throttle indicator
        throttle_bar = "["
        bar_length = 20
        throttle_pos = int((self.throttle + 75) / 150 * bar_length)
        for i in range(bar_length):
            if i < throttle_pos:
                if self.throttle > 0:
                    throttle_bar += "\033[92m█\033[0m"  # Green for forward
                else:
                    throttle_bar += "\033[91m█\033[0m"  # Red for reverse
            else:
                throttle_bar += " "
        throttle_bar += "]"
        
        # Status indicators
        cruise_indicator = "✅ CRUISE" if self.cruise_control else "❌ CRUISE"
        brake_indicator = "🛑 BRAKING" if self.braking else "✅ READY"
        
        # Display
        print(f"🧭 Steering: {steering_bar} {self.steering_angle:3d}°  ", end="")
        print(f"⚡ Throttle: {throttle_bar} {self.throttle:+4d}%  ", end="")
        print(f"📏 {distance_cm:4.0f}cm  ", end="")
        print(f"{cruise_indicator}  {brake_indicator}", end="")
    
    def handle_brake(self):
        """Handle braking"""
        self.braking = True
        self.target_throttle = 0
        self.cruise_control = False
        self.cruise_speed = 0
        time.sleep(0.2)  # Hold brake for 200ms
        self.braking = False
    
    def toggle_cruise_control(self):
        """Toggle cruise control"""
        self.cruise_control = not self.cruise_control
        if self.cruise_control:
            self.cruise_speed = self.throttle
            print(f"\n✅ Cruise control ON: {self.cruise_speed}%")
        else:
            print(f"\n❌ Cruise control OFF")
    
    def scan_surroundings(self):
        """Scan left, center, right"""
        positions = [25.0, 31.5, 38.0]
        distances = []
        
        print("\n👁️  Scanning...")
        for pos in positions:
            self.pca.eye.set_50_range(pos)
            time.sleep(0.15)
            dist_m = self.lidar.get_distance_to_obstacle()
            dist_cm = dist_m * 100 if dist_m else 0
            distances.append(dist_cm)
            print(f"  {pos:4.1f}° → {dist_cm:5.1f}cm")
        
        # Return to center
        self.pca.eye.set_50_range(31.5)
        self.eye_position = 31.5
        
        print(f"📊 Left: {distances[0]:.0f}cm  Center: {distances[1]:.0f}cm  Right: {distances[2]:.0f}cm")
    
    def emergency_stop_procedure(self):
        """Emergency stop and shutdown"""
        print("\n🛑 EMERGENCY STOP!")
        
        # Hard brake
        self.target_throttle = 0
        self.throttle = 0
        self.motor.stop()
        
        # Center steering
        self.steering_angle = 90
        self.pca.driver.set_angle(90)
        
        # Center eye
        self.eye_position = 31.5
        self.pca.eye.set_50_range(31.5)
        
        # Wait
        time.sleep(1)
        
        print("✅ Vehicle stopped and centered")
        print("   Press any key to continue...")
    
    def run(self):
        """Main control loop"""
        print("\n🚗 READY FOR MANUAL CONTROL")
        print("Press keys to control (no Enter needed)")
        print("Press 'C' to center steering")
        print("-" * 60)
        
        try:
            while self.running:
                # Apply current controls
                self.apply_controls()
                
                # Update display
                self.update_display()
                
                # Check for key press
                key = self.get_key()
                
                if key:
                    # Handle steering
                    if key == 'a' or key == 'A':
                        # Turn left
                        self.steering_angle = max(50, self.steering_angle - self.STEERING_STEP)
                    elif key == 'd' or key == 'D':
                        # Turn right
                        self.steering_angle = min(130, self.steering_angle + self.STEERING_STEP)
                    
                    # Handle throttle
                    elif key == 'w' or key == 'W':
                        # Accelerate forward
                        if not self.braking:
                            self.target_throttle = min(self.MAX_THROTTLE, self.target_throttle + self.THROTTLE_ACCEL)
                            if self.cruise_control:
                                self.cruise_speed = self.target_throttle
                    elif key == 's' or key == 'S':
                        # Accelerate reverse
                        if not self.braking:
                            self.target_throttle = max(self.MIN_THROTTLE, self.target_throttle - self.THROTTLE_ACCEL)
                            if self.cruise_control:
                                self.cruise_speed = self.target_throttle
                    
                    # Handle special controls
                    elif key == ' ':
                        # Space bar - brake
                        self.handle_brake()
                    elif key == 'e' or key == 'E':
                        # Toggle cruise control
                        self.toggle_cruise_control()
                    elif key == 'c' or key == 'C':
                        # Center steering
                        self.steering_angle = 90
                        print("\n✅ Steering centered")
                    elif key == 'x' or key == 'X':
                        # Emergency stop
                        self.emergency_stop_procedure()
                    elif key == '\x1b':  # ESC
                        # Quit
                        print("\n\n🛑 Exiting...")
                        self.running = False
                    elif key == '1':
                        # Look left
                        self.eye_position = max(15.0, self.eye_position - self.EYE_STEP)
                    elif key == '2':
                        # Look right
                        self.eye_position = min(46.0, self.eye_position + self.EYE_STEP)
                    elif key == '3':
                        # Center eye
                        self.eye_position = 31.5
                    elif key == '4':
                        # Scan surroundings
                        self.scan_surroundings()
                    
                    # Print key info (for debugging)
                    # print(f"\nPressed: {repr(key)}")
                
                # Cruise control logic
                if self.cruise_control and not self.braking:
                    self.target_throttle = self.cruise_speed
                
                # Small delay for smooth control
                time.sleep(0.05)
        
        except KeyboardInterrupt:
            print("\n\n🛑 Interrupted by user")
        except Exception as e:
            print(f"\n\n❌ Error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Clean shutdown"""
        print("\n🔻 Shutting down...")
        
        # Stop motor
        self.motor.stop()
        
        # Center servos
        self.pca.driver.set_angle(90)
        self.pca.eye.set_50_range(31.5)
        
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        
        time.sleep(0.5)
        print("✅ System safe")

# ============================================================================
# MAIN
# ============================================================================
if __name__ == "__main__":
    signal.signal(signal.SIGINT, lambda s, f: sys.exit(0))
    
    try:
        car = ManualControlCar()
        car.run()
    except Exception as e:
        print(f"\n💥 FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)