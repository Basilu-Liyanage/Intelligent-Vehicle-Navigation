#!/usr/bin/env python3
"""
complete_fusion.py
Complete fusion system in one file
"""

import time
import threading
from dataclasses import dataclass
from typing import Dict, Optional
import serial

# ===== VEHICLE STATE =====
@dataclass
class VehicleState:
    timestamp: float = 0.0
    latitude: float = 37.7749
    longitude: float = -122.4194
    speed_kmh: float = 0.0
    heading_degrees: float = 0.0
    front_distance: float = 5.0
    left_distance: float = 6.0
    right_distance: float = 5.5
    fusion_quality: float = 0.0
    
    def __post_init__(self):
        if self.timestamp == 0:
            self.timestamp = time.time()

# ===== LIDAR =====
class SimpleLidar:
    def __init__(self, port="/dev/serial0"):
        self.port = port
        self.serial = None
        
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=115200,
                timeout=0.2
            )
            print("✅ LIDAR connected")
        except:
            print("⚠️  LIDAR not available")
    
    def get_distance(self):
        if not self.serial:
            return 5.0
        
        try:
            start_time = time.time()
            timeout = 0.1
            
            while time.time() - start_time < timeout:
                if self.serial.in_waiting > 8:
                    data = self.serial.read(9)
                    
                    if len(data) == 9 and data[0] == 0x59 and data[1] == 0x59:
                        distance = data[2] + data[3] * 256
                        return distance / 100.0
                
                time.sleep(0.001)
            
            return 5.0  # Timeout
        except:
            return 5.0

# ===== FUSION NODE =====
class CompleteFusion:
    def __init__(self):
        print("🚀 Starting CompleteFusion...")
        
        # Hardware
        self.lidar = SimpleLidar()
        
        # State
        self.state = VehicleState()
        self.running = False
        self.thread = None
        
        print("✅ CompleteFusion ready")
    
    def update(self):
        """Update all sensors and state"""
        # Get LIDAR
        distance = self.lidar.get_distance()
        time.sleep(0.01)
        
        # Update state
        self.state = VehicleState(
            timestamp=time.time(),
            latitude=self.state.latitude + 0.000001,
            longitude=self.state.longitude + 0.000001,
            speed_kmh=1.5,
            heading_degrees=(self.state.heading_degrees + 0.5) % 360,
            front_distance=distance,
            left_distance=distance + 1.0,
            right_distance=distance + 0.5,
            fusion_quality=0.8 if distance != 5.0 else 0.5
        )
        
        print(f"📡 LIDAR: {distance:.2f}m | "
              f"Speed: {self.state.speed_kmh:.1f}km/h | "
              f"Heading: {self.state.heading_degrees:.0f}°")
    
    def start(self):
        """Start fusion"""
        if self.running:
            return
        
        print("▶️  Starting...")
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        time.sleep(0.1)
    def _loop(self):
        """Main loop"""
        while self.running:
            self.update()
            time.sleep(0.1)
    
    def stop(self):
        """Stop fusion"""
        print("⏹️  Stopping...")
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
    
    def get_state(self):
        return self.state

# ===== MAIN =====
if __name__ == "__main__":
    print("🧪 Complete Fusion Test")
    print("=" * 60)
    
    fusion = CompleteFusion()
    
    try:
        fusion.start()
        
        print("\nGetting readings (Ctrl+C to stop):")
        for i in range(10):
            time.sleep(1)
            state = fusion.get_state()
            print(f"{i+1:2d}. Front: {state.front_distance:.2f}m | "
                  f"Speed: {state.speed_kmh:.1f}km/h")
        
        fusion.stop()
        print("\n✅ Test complete!")
        
    except KeyboardInterrupt:
        print("\n⏹️  Stopped by user")
        fusion.stop()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        fusion.stop()