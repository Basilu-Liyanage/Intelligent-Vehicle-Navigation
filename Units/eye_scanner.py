#!/usr/bin/env python3
"""
SMART EYE SCANNER
Turns LiDAR to scan environment and builds a "view map"
"""

import numpy as np
import time

class EyeScanner:
    """Scans environment by moving LiDAR eye"""
    
    def __init__(self, servo_bridge, lidar):
        self.servo = servo_bridge
        self.lidar = lidar
        self.scan_map = {}  # angle -> distance
        self.last_scan_time = 0
        self.scan_interval = 0.5  # seconds
        
        # Scan positions (in 0-50 range)
        self.scan_positions = [15, 25, 31.5, 38, 45]  # Left to right
    
    def perform_scan(self):
        """Perform full scan and update map"""
        current_time = time.time()
        
        if current_time - self.last_scan_time < self.scan_interval:
            return self.scan_map
        
        print("👁️  Performing LiDAR scan...")
        
        for pos in self.scan_positions:
            # Move eye to position
            self.servo.execute_eye_position(pos)
            time.sleep(0.1)  # Let LiDAR stabilize
            
            # Read distance
            distance = self.lidar.read_distance()
            
            # Convert position to angle
            angle = (pos - 31.5) * (20.0 / 50.0)  # -10° to +10°
            
            # Store in map
            self.scan_map[angle] = distance
            
            print(f"  Angle {angle:4.1f}°: {distance:.2f}m")
        
        # Return to center
        self.servo.execute_eye_position(31.5)
        
        self.last_scan_time = current_time
        return self.scan_map
    
    def get_best_direction(self):
        """Find clearest path from scan"""
        if not self.scan_map:
            return 0.0  # Straight ahead
        
        # Find direction with maximum distance
        best_angle = max(self.scan_map, key=self.scan_map.get)
        best_distance = self.scan_map[best_angle]
        
        # Normalize to -1 to 1
        normalized_angle = best_angle / 10.0  # ±10° → ±1.0
        
        print(f"🎯 Best direction: {best_angle:4.1f}° ({best_distance:.2f}m)")
        return normalized_angle

# Add to your servo bridge
class EnhancedServoBridge:
    """Adds scanning capability"""
    
    def __init__(self, original_bridge, lidar):
        self.bridge = original_bridge
        self.lidar = lidar
        self.scanner = EyeScanner(self, lidar)
        
    def execute_eye_position(self, position):
        """Move eye to specific position"""
        eye = self.bridge.servo_controller.eye
        eye.set_50_range(position)
        
    def smart_scan_and_steer(self, ai_steering):
        """
        Smart scanning: Look where we want to go
        """
        # 1. Perform scan
        scan_map = self.scanner.perform_scan()
        
        # 2. Find best direction
        best_direction = self.scanner.get_best_direction()
        
        # 3. Blend AI steering with scan results
        # If scan shows obstacle ahead but clear on side, override AI
        center_dist = scan_map.get(0.0, 5.0)  # Center distance
        
        if center_dist < 1.0:  # Obstacle ahead
            print(f"🚨 Obstacle ahead ({center_dist:.2f}m), using scan guidance")
            final_steering = best_direction
        else:
            # Follow AI but adjust based on scan
            scan_weight = 0.3  # How much to trust scan vs AI
            final_steering = (1 - scan_weight) * ai_steering + scan_weight * best_direction
        
        # 4. Look in the direction we're steering
        look_position = 31.5 + (final_steering * 18.5)  # Map to 0-50
        self.execute_eye_position(look_position)
        
        return final_steering