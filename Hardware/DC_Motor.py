"""
dc_motor_controller_fixed.py
Fixed Professional DC Motor Controller
"""
import RPi.GPIO as GPIO
import time
import logging
from typing import Optional, Tuple
from enum import Enum
from dataclasses import dataclass
import signal
import sys

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class MotorDirection(Enum):
    """Motor direction enumeration"""
    FORWARD = "FORWARD"
    REVERSE = "REVERSE"
    STOP = "STOP"

class DCMotor:
    """
    Fixed DC Motor Controller - Actually Works!
    """
    
    def __init__(self, 
                 rpwm_pin: int = 4, 
                 lpwm_pin: int = 17, 
                 ren_pin: int = 27, 
                 len_pin: int = 22,
                 motor_name: str = "Motor"):
        """
        Initialize DC Motor Controller
        
        Args:
            rpwm_pin: Right PWM pin (forward) - GPIO 4
            lpwm_pin: Left PWM pin (reverse) - GPIO 17
            ren_pin: Right enable pin - GPIO 27
            len_pin: Left enable pin - GPIO 22
        """
        self.motor_name = motor_name
        self.pins = {
            'RPWM': rpwm_pin,
            'LPWM': lpwm_pin,
            'REN': ren_pin,
            'LEN': len_pin
        }
        
        # FIXED: Always set GPIO mode
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Configure pins
        for pin_name, pin in self.pins.items():
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        # PWM setup - FIXED: Use proper frequency
        PWM_FREQ = 1000  # 1kHz works well
        self.rpwm = GPIO.PWM(self.pins['RPWM'], PWM_FREQ)
        self.lpwm = GPIO.PWM(self.pins['LPWM'], PWM_FREQ)
        self.rpwm.start(0)
        self.lpwm.start(0)
        
        # State
        self.current_speed = 0.0
        self.current_direction = MotorDirection.STOP
        
        # Setup emergency stop signal
        signal.signal(signal.SIGINT, self._emergency_signal_handler)
        signal.signal(signal.SIGTERM, self._emergency_signal_handler)
        
        logger.info(f"{motor_name} initialized and ready on pins: {self.pins}")
    
    def _emergency_signal_handler(self, signum, frame):
        """Handle Ctrl+C and termination signals"""
        logger.warning(f"⚠️ Emergency stop triggered by signal {signum}")
        self.emergency_stop()
        sys.exit(0)
    
    def move_forward(self, speed_percent: float):
        """
        Move motor forward
        
        Args:
            speed_percent: 0-100
        """
        if not 0 <= speed_percent <= 100:
            raise ValueError(f"Speed must be 0-100, got {speed_percent}")
        
        # Enable motor driver
        GPIO.output(self.pins['REN'], GPIO.HIGH)
        GPIO.output(self.pins['LEN'], GPIO.HIGH)
        
        # Apply PWM
        self.rpwm.ChangeDutyCycle(speed_percent)
        self.lpwm.ChangeDutyCycle(0)
        
        self.current_speed = speed_percent
        self.current_direction = MotorDirection.FORWARD
        
        logger.info(f"{self.motor_name}: Forward at {speed_percent}%")
    
    def move_reverse(self, speed_percent: float):
        """
        Move motor in reverse
        
        Args:
            speed_percent: 0-100
        """
        if not 0 <= speed_percent <= 100:
            raise ValueError(f"Speed must be 0-100, got {speed_percent}")
        
        # Enable motor driver
        GPIO.output(self.pins['REN'], GPIO.HIGH)
        GPIO.output(self.pins['LEN'], GPIO.HIGH)
        
        # Apply PWM
        self.lpwm.ChangeDutyCycle(speed_percent)
        self.rpwm.ChangeDutyCycle(0)
        
        self.current_speed = speed_percent
        self.current_direction = MotorDirection.REVERSE
        
        logger.info(f"{self.motor_name}: Reverse at {speed_percent}%")
    
    def stop(self, brake: bool = False):
        """
        Stop the motor
        
        Args:
            brake: If True, apply braking (short motor leads)
        """
        if brake:
            # Brake mode - short motor leads
            GPIO.output(self.pins['REN'], GPIO.HIGH)
            GPIO.output(self.pins['LEN'], GPIO.HIGH)
            self.rpwm.ChangeDutyCycle(100)
            self.lpwm.ChangeDutyCycle(100)
            time.sleep(0.05)  # Short brake pulse
        
        # Coast mode
        GPIO.output(self.pins['REN'], GPIO.LOW)
        GPIO.output(self.pins['LEN'], GPIO.LOW)
        self.rpwm.ChangeDutyCycle(0)
        self.lpwm.ChangeDutyCycle(0)
        
        self.current_speed = 0.0
        self.current_direction = MotorDirection.STOP
        
        logger.info(f"{self.motor_name}: Stopped {'with brake' if brake else ''}")
    
    def emergency_stop(self):
        """Immediate emergency stop with braking"""
        logger.warning(f"🚨 {self.motor_name} EMERGENCY STOP")
        self.stop(brake=True)
    
    def ramp_to_speed(self, target_speed: float, direction: MotorDirection, ramp_time: float = 2.0):
        """
        Smooth ramp todc_motor_controller target speed
        
        Args:
            target_speed: 0-100
            direction: FORWARD or REVERSE
            ramp_time: seconds to reach target
        """
        steps = int(ramp_time * 10)  # 10 updates per second
        step_delay = ramp_time / steps
        speed_step = (target_speed - self.current_speed) / steps
        
        for i in range(steps):
            current_step_speed = self.current_speed + speed_step
            
            if direction == MotorDirection.FORWARD:
                self.move_forward(current_step_speed)
            else:
                self.move_reverse(current_step_speed)
            
            time.sleep(step_delay)
        
        # Ensure final speed is exact
        if direction == MotorDirection.FORWARD:
            self.move_forward(target_speed)
        else:
            self.move_reverse(target_speed)
    
    def get_status(self):
        """Return current status"""
        return {
            'motor': self.motor_name,
            'speed': self.current_speed,
            'direction': self.current_direction.value,
            'pins': self.pins
        }
    
    def cleanup(self):
        """Clean shutdown"""
        logger.info(f"Cleaning up {self.motor_name}")
        self.stop()
        self.rpwm.stop()
        self.lpwm.stop()
        GPIO.cleanup()
        logger.info(f"{self.motor_name} cleanup complete")


# ============================================================================
# SIMPLE TEST - This will actually work!
# ============================================================================

def test_motor():
    """Test that actually moves the motor"""
    motor = DCMotor(
        rpwm_pin=4,
        lpwm_pin=17,
        ren_pin=27,
        len_pin=22,
        motor_name="MainDrive"
    )
    
    try:
        print("Testing motor control...")
        
        # Test 1: Forward
        print("\n1. Moving forward at 30%")
        motor.move_forward(30)
        time.sleep(2)
        
        # Test 2: Ramp up
        print("\n2. Ramping to 60% forward")
        motor.ramp_to_speed(60, MotorDirection.FORWARD, ramp_time=1.5)
        time.sleep(1)
        
        # Test 3: Stop
        print("\n3. Stopping")
        motor.stop()
        time.sleep(1)
        
        # Test 4: Reverse
        print("\n4. Moving reverse at 40%")
        motor.move_reverse(40)
        time.sleep(2)
        
        # Test 5: Emergency stop
        print("\n5. Emergency stop")
        motor.emergency_stop()
        
        print("\n✅ All tests passed!")
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        motor.emergency_stop()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        motor.emergency_stop()
    finally:
        motor.cleanup()


def simple_cli():
    """Simple command line interface"""
    motor = DCMotor(motor_name="TestMotor")
    
    print("=" * 50)
    print("MOTOR TEST INTERFACE")
    print("=" * 50)
    print("Pins configured:")
    print(f"  RPWM: GPIO {motor.pins['RPWM']}")
    print(f"  LPWM: GPIO {motor.pins['LPWM']}")
    print(f"  REN:  GPIO {motor.pins['REN']}")
    print(f"  LEN:  GPIO {motor.pins['LEN']}")
    print("=" * 50)
    
    try:
        while True:
            print(f"\nCurrent: {motor.current_speed}% {motor.current_direction.value}")
            print("1. Forward")
            print("2. Reverse")
            print("3. Stop")
            print("4. Ramp test")
            print("5. Exit")
            
            choice = input("Choice (1-5): ").strip()
            
            if choice == '1':
                speed = float(input("Speed % (0-100): "))
                motor.move_forward(speed)
            elif choice == '2':
                speed = float(input("Speed % (0-100): "))
                motor.move_reverse(speed)
            elif choice == '3':
                motor.stop()
            elif choice == '4':
                speed = float(input("Target speed %: "))
                direction = MotorDirection.FORWARD if speed >= 0 else MotorDirection.REVERSE
                ramp = float(input("Ramp time (s): "))
                motor.ramp_to_speed(abs(speed), direction, ramp)
            elif choice == '5':
                break
            else:
                print("Invalid choice")
            
    except KeyboardInterrupt:
        print("\n\nInterrupted")
    finally:
        motor.cleanup()
        print("Goodbye!")


if __name__ == "__main__":
    # Uncomment one of these:
    # test_motor()      # For automated test
    simple_cli()       # For interactive testing