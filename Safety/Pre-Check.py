#!/usr/bin/env python3
"""
IVN QUICK SCAN - Fast hardware check before running
Run this before starting the main controller
"""

import time
import sys
import os
import subprocess

def quick_scan():
    """Fast 2-second hardware check - returns True if ready"""
    
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
    
    # 3. Quick GPIO check (just see if pins are free)
    try:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        # Just test one pin quickly
        GPIO.setup(4, GPIO.IN)
        GPIO.cleanup()
        print("✅ GPIO: OK")
    except:
        issues.append("GPIO error")
        print("❌ GPIO: FAIL")
    
    # 4. Check AI model exists
    model_path = "/home/pi/Desktop/Intelligent-Vehicle-Navigation/AI/spatial_rl_model.pth"
    if os.path.exists(model_path):
        print("✅ AI Model: OK")
    else:
        issues.append("AI model not found")
        print("⚠️ AI Model: Missing (will run in fallback mode)")
    
    # Summary
    print("-" * 30)
    if not issues:
        print("✅ READY - All systems go!")
        return True
    else:
        print(f"⚠️ {len(issues)} issue(s):")
        for issue in issues:
            print(f"   • {issue}")
        return False

if __name__ == "__main__":
    ready = quick_scan()
    if ready:
        print("\n🚀 Starting main controller...")
        # Import and run your main controller here
        # from tesla_controller_fixed import TeslaController
        # controller = TeslaController()
        # controller.run()
    else:
        print("\n❌ Cannot start - fix hardware issues first")
        sys.exit(1)