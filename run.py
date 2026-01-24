#!/usr/bin/env python3
# run.py - WITH MOTOR DEBUGGING

import sys
import time
import signal
import os
import traceback

# Add your project paths
sys.path.append('AI')
sys.path.append('Vehicle')
sys.path.append('Vehicle/Perception')
sys.path.append('Vehicle/Control')

print("=" * 80)
print("🚗 DEBUG MODE - AUTONOMOUS VEHICLE")
print("=" * 80)

def debug_motor():
    """Direct motor test - bypass everything else"""
    print("\n🔧 DIRECT MOTOR TEST")
    
    try:
        from Hardware.DC_Motor import DCMotorController, MotorDirection
        
        print("Creating motor controller...")
        motor = DCMotorController(
            rpwm_pin=4,
            lpwm_pin=17,
            ren_pin=27,
            len_pin=22,
            motor_name="DEBUG_MOTOR"
        )
        
        print("\nTEST 1: 30% forward for 2 seconds")
        motor.set_speed(30, MotorDirection.FORWARD)
        motor.update()  # CRITICAL!
        print("Motor command sent. Wheels should move NOW.")
        time.sleep(2)
        
        print("\nTEST 2: Stop")
        motor.set_speed(0, MotorDirection.FORWARD)
        motor.update()
        time.sleep(1)
        
        print("\nTEST 3: 50% forward for 2 seconds")
        motor.set_speed(50, MotorDirection.FORWARD)
        motor.update()
        time.sleep(2)
        
        print("\nEmergency stop")
        motor.emergency_stop()
        
        print("\n✅ Motor test complete. Did wheels move?")
        
    except Exception as e:
        print(f"❌ Motor test failed: {e}")
        traceback.print_exc()

def main():
    """Main function with debug options"""
    print("\nChoose operation:")
    print("1. Direct motor test (bypass everything)")
    print("2. Run normal autonomous mode")
    print("3. Exit")
    
    choice = input("\nSelect (1-3): ").strip()
    
    if choice == "1":
        debug_motor()
    elif choice == "2":
        run_normal_mode()
    else:
        print("Exiting...")

def run_normal_mode():
    """Your original main function with fixes"""
    # ===== CORRECT IMPORTS =====
    try:
        from Vehicle.Control.autonomous_controller import AutonomousController
        from Hardware.Lidar_Sensor import TFLuna
        from Vehicle.perception.sensor_fusion.pixhawk_reader import PixhawkReader
        
        print("✅ All imports successful")
    except Exception as e:
        print(f"❌ Import error: {e}")
        traceback.print_exc()
        return
    
    # ===== CHECK SERVO BRIDGE =====
    try:
        from Vehicle.Control.steering_integration import AIToServoBridge
        servo_bridge_class = AIToServoBridge
        print("✅ Servo bridge available")
    except:
        print("⚠️ No servo bridge available")
        servo_bridge_class = None
    
    # ===== CHECK AI =====
    try:
        from AI.Models.SAC import SACAgent
        ai_loaded = True
        print("✅ AI module available")
    except:
        print("⚠️ AI module not found")
        ai_loaded = False
    
    class SimpleAI:
        """Simple AI for testing"""
        def get_action(self, observation, deterministic=True):
            import numpy as np
            # Simple pattern: slight turn, moderate speed
            steering = 0.1 if time.time() % 4 < 2 else -0.1
            throttle = 0.4  # 40% power
            return np.array([steering, throttle])
    
    # Initialize sensors
    print("\nInitializing sensors...")
    lidar = None
    pixhawk = None
    
    try:
        lidar = TFLuna(port='/dev/serial0')
        print("✅ LiDAR connected")
    except Exception as e:
        print(f"⚠️ LiDAR failed: {e}")
    
    try:
        pixhawk = PixhawkReader(port='/dev/ttyACM0')
        print("✅ Pixhawk connected")
    except Exception as e:
        print(f"⚠️ Pixhawk failed: {e}")
    
    # Initialize AI
    print("\nLoading AI...")
    if ai_loaded:
        try:
            ai_agent = SACAgent(state_dim=10, action_dim=2)
            checkpoint_path = "AI/checkpoints/BEST_300_REWARD_ep15.pth"
            if ai_agent.load(checkpoint_path):
                print("✅ 300-reward AI loaded")
            else:
                print("⚠️ AI load failed, using simple AI")
                ai_agent = SimpleAI()
        except:
            print("⚠️ AI failed, using simple AI")
            ai_agent = SimpleAI()
    else:
        ai_agent = SimpleAI()
        print("✅ Using simple AI")
    
    # ===== FIXED CONTROLLER INITIALIZATION =====
    print("\nCreating controller...")
    try:
        controller = AutonomousController(
            ai_agent=ai_agent,
            lidar_sensor=lidar,
            pixhawk_sensor=pixhawk,
            rpwm_pin=4,
            lpwm_pin=17,
            ren_pin=27,
            len_pin=22,
            motor_name="MainDrive"
        )
        print("✅ Controller created")
        
        # Add servo bridge if available
        if servo_bridge_class is not None:
            if not hasattr(controller, 'servo_bridge') or controller.servo_bridge is None:
                controller.servo_bridge = servo_bridge_class()
                if hasattr(controller.servo_bridge, 'initialize'):
                    controller.servo_bridge.initialize()
                print("✅ Servo bridge added")
        
    except Exception as e:
        print(f"❌ Controller creation failed: {e}")
        traceback.print_exc()
        return
    
    # Signal handling
    def signal_handler(sig, frame):
        print(f"\nSignal {sig} received, shutting down...")
        if hasattr(controller, 'cleanup'):
            controller.cleanup()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Command interface
    print("\n" + "=" * 80)
    print("CONTROL PANEL")
    print("=" * 80)
    print("Commands: s=start, x=stop, e=emergency, r=resume, p=status, t=motor test, q=quit")
    
    while True:
        cmd = input("\nCommand: ").strip().lower()
        
        if cmd == 's':
            if not controller.running:
                print("🚀 Starting autonomous mode...")
                print("SAFETY CHECK: Vehicle in open area?")
                confirm = input("Type 'GO' to confirm: ").strip()
                if confirm == 'GO':
                    # Start with VERY SLOW speed for testing
                    print("Starting with SAFE SLOW speed (20%)")
                    controller.start_autonomous()
                else:
                    print("Cancelled")
            else:
                print("Already running")
        
        elif cmd == 'x':
            if controller.running:
                print("Stopping...")
                controller.stop_autonomous()
            else:
                print("Not running")
        
        elif cmd == 'e':
            print("🚨 EMERGENCY STOP!")
            if hasattr(controller, 'trigger_emergency_stop'):
                controller.trigger_emergency_stop("Manual emergency")
            elif hasattr(controller, 'emergency_stop'):
                controller.emergency_stop()
        
        elif cmd == 'r':
            print("Resuming...")
            if hasattr(controller, 'resume_from_emergency'):
                controller.resume_from_emergency()
        
        elif cmd == 'p':
            if hasattr(controller, 'get_status_report'):
                status = controller.get_status_report()
                print("\nSTATUS:")
                for key, value in status.items():
                    print(f"{key:20}: {value}")
            else:
                print("Status not available")
        
        elif cmd == 't':
            # Quick motor test
            debug_motor()
        
        elif cmd == 'q':
            print("Exiting...")
            break
        
        else:
            print("Unknown command")

if __name__ == "__main__":
    main()