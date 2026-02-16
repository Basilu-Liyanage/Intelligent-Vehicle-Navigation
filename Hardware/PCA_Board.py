import smbus2
import time
import json
from enum import Enum
from typing import Optional, Dict

# ==================== CHANNEL ENUM ====================
class ServoChannel(Enum):
    """Hardware channel assignments"""
    DRIVER = 0
    EYE = 1
    REAR = 2

# ==================== SERVO CALIBRATION ====================
class ServoCalibration:
    """Calibration data for a servo"""
    def __init__(self, channel: ServoChannel, center_position=25.0, center_pulse=1500, 
                 pos_25_pulse=None, pos_38_pulse=None):
        self.channel = channel
        self.center_position = center_position  # In 0-50 range
        self.center_pulse = center_pulse        # µs at center
        self.pos_25_pulse = pos_25_pulse        # µs at position 25
        self.pos_38_pulse = pos_38_pulse        # µs at position 38
        
        if pos_25_pulse and pos_38_pulse:
            self.us_per_step = (pos_38_pulse - pos_25_pulse) / 13.0  # 38-25=13 steps
        else:
            self.us_per_step = 16.0  # Default
    
    def position_to_pulse(self, position):
        """Convert 0-50 position to pulse width"""
        if self.pos_25_pulse and self.pos_38_pulse:
            # Use calibrated linear interpolation
            return int(self.pos_25_pulse + (position - 25) * self.us_per_step)
        else:
            # Fallback to center-based calculation
            pulse_range = 400  # ±400µs from center for full range
            if position < self.center_position:
                ratio = position / self.center_position
                return int(self.center_pulse - (pulse_range * ratio))
            else:
                ratio = (position - self.center_position) / (50 - self.center_position)
                return int(self.center_pulse + (pulse_range * ratio))
    
    def save_to_file(self, filename=None):
        """Save calibration to JSON file"""
        if not filename:
            filename = f"servo_calibration_{self.channel.name.lower()}.json"
        
        data = {
            "channel": self.channel.value,
            "center_position": self.center_position,
            "center_pulse": self.center_pulse,
            "pos_25_pulse": self.pos_25_pulse,
            "pos_38_pulse": self.pos_38_pulse,
            "us_per_step": self.us_per_step,
            "calibration_date": time.strftime("%Y-%m-%d %H:%M:%S")
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"💾 Calibration saved to {filename}")
    
    @classmethod
    def load_from_file(cls, channel: ServoChannel):
        """Load calibration from JSON file"""
        filename = f"servo_calibration_{channel.name.lower()}.json"
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            cal = cls(
                channel=channel,
                center_position=data["center_position"],
                center_pulse=data["center_pulse"],
                pos_25_pulse=data["pos_25_pulse"],
                pos_38_pulse=data["pos_38_pulse"]
            )
            print(f"📂 Loaded calibration from {filename}")
            return cal
        except FileNotFoundError:
            print(f"⚠️  No calibration file found: {filename}")
            return None

# ==================== SERVO CLASS ====================
class Servo:
    """Individual servo with easy access methods"""
    
    def __init__(self, channel: ServoChannel, name: str,
                 controller,  # Will be set after PCA9685Controller is created
                 actuation_min=15, actuation_max=70, 
                 min_pulse=500, max_pulse=2500,
                 calibration: Optional[ServoCalibration] = None):
        
        self.channel = channel
        self.name = name
        self._controller = None  # Will be set by PCA9685Controller
        self.actuation_min = actuation_min
        self.actuation_max = actuation_max
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.current_angle = 90  # Default center (0-180 normalized)
        
        # Calibration
        if calibration:
            self.calibration = calibration
        else:
            # Default calibration
            self.calibration = ServoCalibration(
                channel=channel,
                center_position=25.0,
                center_pulse=1500
            )
        
        # Load saved calibration if exists
        saved_cal = ServoCalibration.load_from_file(channel)
        if saved_cal:
            self.calibration = saved_cal
    
    def set_controller(self, controller):
        """Set the controller reference"""
        self._controller = controller
    
    # ===== PUBLIC INTERFACE =====
    
    def set_angle(self, angle: float):
        """
        Set servo using normalized 0-180° angle
        Maps to servo's physical actuation range
        """
        if self._controller is None:
            raise RuntimeError(f"Servo {self.name} not connected to controller")
        
        angle = max(0, min(180, angle))
        
        # Map 0-180 to actuation_min-actuation_max
        real_angle = (self.actuation_min + 
                     (angle / 180.0) * (self.actuation_max - self.actuation_min))
        
        # Convert to pulse
        pulse_range = self.max_pulse - self.min_pulse
        pulse = self.min_pulse + (real_angle / 180.0) * pulse_range
        pulse = int(pulse)
        
        # Set via controller
        self._controller.set_pulse(self.channel.value, pulse)
        self.current_angle = angle
        
        print(f"{self.name} → {angle}° → {real_angle:.1f}° physical → {pulse}µs")
        return pulse
    
    def set_50_range(self, position: float):
        """
        Set servo using 0-50 range (your preferred control range)
        This uses the CALIBRATED mapping
        """
        if self._controller is None:
            raise RuntimeError(f"Servo {self.name} not connected to controller")
        
        position = max(0, min(50, position))
        
        # Get calibrated pulse
        pulse = self.calibration.position_to_pulse(position)
        
        # Set via controller
        self._controller.set_pulse(self.channel.value, pulse)
        
        # Calculate normalized angle for reference
        pulse_range = self.max_pulse - self.min_pulse
        norm_pulse = (pulse - self.min_pulse) / pulse_range
        self.current_angle = norm_pulse * 180.0
        
        print(f"{self.name} → {position:.1f}/50 → {pulse}µs → {self.current_angle:.0f}°")
        return pulse
    
    # ===== CONVENIENCE METHODS =====
    
    def center(self):
        """Center the servo"""
        if self.name.lower() == "eye":
            # Eye uses 0-50 range with special center
            return self.set_50_range(31.5)
        else:
            # Other servos use 0-180° range
            return self.set_angle(90)
    
    def full_left(self):
        """Move to full left position"""
        if self.name.lower() == "eye":
            return self.set_50_range(0)
        else:
            return self.set_angle(0)
    
    def full_right(self):
        """Move to full right position"""
        if self.name.lower() == "eye":
            return self.set_50_range(50)
        else:
            return self.set_angle(180)
    
    def look_left(self, amount: float = 25):
        """Eye-specific: Look left by amount"""
        if self.name.lower() != "eye":
            print(f"⚠️  look_left() is only for eye servo")
            return self.set_angle(max(0, self.current_angle - amount))
        
        position = max(0, 31.5 - amount)
        return self.set_50_range(position)
    
    def look_right(self, amount: float = 25):
        """Eye-specific: Look right by amount"""
        if self.name.lower() != "eye":
            print(f"⚠️  look_right() is only for eye servo")
            return self.set_angle(min(180, self.current_angle + amount))
        
        position = min(50, 31.5 + amount)
        return self.set_50_range(position)
    
    def save_calibration(self):
        """Save current calibration to file"""
        self.calibration.save_to_file()
    
    @property
    def status(self) -> Dict:
        """Get servo status"""
        return {
            "name": self.name,
            "channel": self.channel.name,
            "current_angle": self.current_angle,
            "calibration": {
                "center_position": self.calibration.center_position,
                "center_pulse": self.calibration.center_pulse
            }
        }

# ==================== PCA9685 CONTROLLER ====================
class PCA9685Controller:
    """PCA9685 PWM controller with predefined servo properties"""
    
    def __init__(self, bus_num=3, address=0x40, frequency=50):
        self.bus_num = bus_num
        self.address = address
        self.frequency = frequency
        
        # Initialize I2C
        self.bus = smbus2.SMBus(self.bus_num)
        
        # PCA9685 registers
        self.MODE1 = 0x00
        self.PRESCALE = 0xFE
        
        # Initialize PCA9685
        self._init_pca()
        
        # ===== PREDEFINED SERVOS =====
        # EYE SERVO - PERFECT CALIBRATION
        eye_calibration = ServoCalibration(
            channel=ServoChannel.EYE,
            center_position=31.50,  # TRUE CENTER in 0-50 range
            center_pulse=1574,      # TRUE CENTER pulse
            pos_25_pulse=1470,      # Position 25 pulse
            pos_38_pulse=1678       # Position 38 pulse
        )
        
        # Create all servos
        self.driver = Servo(
            channel=ServoChannel.DRIVER,
            name="Driver",
            controller=self,
            actuation_min=15,
            actuation_max=70
        )
        
        self.eye = Servo(
            channel=ServoChannel.EYE,
            name="Eye",
            controller=self,
            actuation_min=15,
            actuation_max=70,
            calibration=eye_calibration
        )
        
        self.rear = Servo(
            channel=ServoChannel.REAR,
            name="Rear",
            controller=self,
            actuation_min=15,
            actuation_max=70
        )
        
        # Connect servos to this controller
        self.driver.set_controller(self)
        self.eye.set_controller(self)
        self.rear.set_controller(self)
        
        # Store all servos in a dict for easy access
        self._servos = {
            "driver": self.driver,
            "eye": self.eye,
            "rear": self.rear
        }
        
        print("\n" + "=" * 60)
        print("🤖 PCA9685 CONTROLLER INITIALIZED")
        print("=" * 60)
        print(f"Predefined servos:")
        print(f"  • self.driver   (Channel {ServoChannel.DRIVER.value})")
        print(f"  • self.eye      (Channel {ServoChannel.EYE.value}) - PERFECTLY CALIBRATED!")
        print(f"  • self.rear     (Channel {ServoChannel.REAR.value})")
        print(f"  • self._servos  (Dictionary access)")
        print("=" * 60)
    
    def _init_pca(self):
        """Initialize PCA9685 chip"""
        # Reset
        self.bus.write_byte_data(self.address, self.MODE1, 0x00)
        time.sleep(0.01)
        
        # Set frequency
        prescale_val = int(25000000.0 / (4096 * self.frequency) - 1)
        
        # Put to sleep, set prescaler, wake up
        self.bus.write_byte_data(self.address, self.MODE1, 0x10)  # Sleep
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale_val)
        self.bus.write_byte_data(self.address, self.MODE1, 0x00)  # Wake
        time.sleep(0.01)
    
    def set_pulse(self, channel: int, pulse_us: int):
        """Set servo pulse width in microseconds"""
        # Clamp to safe limits
        pulse_us = max(500, min(2500, pulse_us))
        
        # Convert µs to PCA9685 register value
        pulse_length = 1000000.0 / self.frequency / 4096  # µs per bit
        pulse_value = int(pulse_us / pulse_length)
        
        # Calculate register address for this channel
        LED0_ON_L = 0x06 + 4 * channel
        
        # Set PWM
        self.bus.write_byte_data(self.address, LED0_ON_L, 0)
        time.sleep(0.01)
        self.bus.write_byte_data(self.address, LED0_ON_L + 1, 0)
        time.sleep(0.01)
        self.bus.write_byte_data(self.address, LED0_ON_L + 2, pulse_value & 0xFF)
        time.sleep(0.01)
        self.bus.write_byte_data(self.address, LED0_ON_L + 3, (pulse_value >> 8) & 0xFF)
        time.sleep(0.01)
        return pulse_us
    
    # ===== SERVO ACCESS METHODS =====
    
    def get_servo(self, name: str) -> Optional[Servo]:
        """Get servo by name"""
        return self._servos.get(name.lower())
    
    def center_all(self):
        """Center all servos"""
        print("\n🔄 Centering all servos...")
        self.driver.center()
        time.sleep(0.5)
        self.eye.center()
        time.sleep(0.5)
        self.rear.center()
    
    def test_all(self):
        """Test all servos through their ranges"""
        print("\n" + "=" * 60)
        print("🧪 TESTING ALL SERVOS")
        print("=" * 60)
        
        # Test Driver
        print("\n🚗 DRIVER SERVO:")
        self.driver.full_left()
        time.sleep(1)
        self.driver.center()
        time.sleep(1)
        self.driver.full_right()
        time.sleep(1)
        self.driver.center()
        
        # Test Eye
        print("\n👁️  EYE SERVO:")
        self.eye.full_left()
        time.sleep(1)
        self.eye.center()
        time.sleep(1)
        self.eye.full_right()
        time.sleep(1)
        self.eye.center()
        
        # Test Rear
        print("\n📦 REAR SERVO:")
        self.rear.full_left()
        time.sleep(1)
        self.rear.center()
        time.sleep(1)
        self.rear.full_right()
        time.sleep(1)
        self.rear.center()
        
        print("\n✅ All servos tested!")
    
    @property
    def status(self) -> Dict:
        """Get status of all servos"""
        return {
            "driver": self.driver.status,
            "eye": self.eye.status,
            "rear": self.rear.status
        }

# ==================== USAGE EXAMPLES ====================

if __name__ == "__main__":
    # Create controller (automatically creates all servos)
    pca = PCA9685Controller()
    
    # Center eye immediately (using perfect calibration)
    pca.eye.center()
    time.sleep(1)
    
    # ===== CLEAN ACCESS EXAMPLES =====
    print("\n" + "=" * 60)
    print("✨ CLEAN SERVO ACCESS EXAMPLES")
    print("=" * 60)
    
    # Example 1: Direct access
    print("1. Direct property access:")
    print("   pca.driver.set_angle(0)      # Driver full left")
    print("   pca.eye.set_50_range(31.5)   # Eye perfect center")
    print("   pca.rear.set_angle(180)      # Rear full right")
    
    # Example 2: Convenience methods
    print("\n2. Convenience methods:")
    print("   pca.driver.center()          # Center driver")
    print("   pca.eye.full_left()          # Eye full left")
    print("   pca.rear.full_right()        # Rear full right")
    print("   pca.eye.look_left(15)        # Eye look left 15")
    print("   pca.eye.look_right(15)       # Eye look right 15")
    
    # Example 3: Dictionary access
    print("\n3. Dictionary access:")
    print("   pca.get_servo('driver').set_angle(90)")
    print("   pca._servos['eye'].set_50_range(25)")
    
    print("\n" + "=" * 60)
    
    # Interactive menu
    while True:
        print("\n🤖 SERVO CONTROL MENU")
        print("=" * 30)
        print("1. Driver - Full Left (0°)")
        print("2. Driver - Center (90°)")
        print("3. Driver - Full Right (180°)")
        print("4. Eye - Full Left (0/50)")
        print("5. Eye - Center (31.5/50)")
        print("6. Eye - Full Right (50/50)")
        print("7. Eye - Look Left 15")
        print("8. Eye - Look Right 15")
        print("9. Rear - Full Left (0°)")
        print("10. Rear - Center (90°)")
        print("11. Rear - Full Right (180°)")
        print("12. Center All Servos")
        print("13. Test All Servos")
        print("14. Show Status")
        print("15. Exit")
        
        choice = input("\nSelect (1-15): ").strip()
        
        if choice == '1':
            pca.driver.set_angle(0)
        elif choice == '2':
            pca.driver.set_angle(90)
        elif choice == '3':
            pca.driver.set_angle(180)
        elif choice == '4':
            pca.eye.set_50_range(0)
        elif choice == '5':
            pca.eye.set_50_range(31.5)
        elif choice == '6':
            pca.eye.set_50_range(50)
        elif choice == '7':
            pca.eye.look_left(15)
        elif choice == '8':
            pca.eye.look_right(15)
        elif choice == '9':
            pca.rear.set_angle(0)
        elif choice == '10':
            pca.rear.set_angle(90)
        elif choice == '11':
            pca.rear.set_angle(180)
        elif choice == '12':
            pca.center_all()
        elif choice == '13':
            pca.test_all()
        elif choice == '14':
            import pprint
            pprint.pprint(pca.status, indent=2)
        elif choice == '15':
            print("\n👋 Shutting down...")
            pca.center_all()
            break
        else:
            print("❌ Invalid choice")
        
        time.sleep(0.5)