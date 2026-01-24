#!/usr/bin/env python3
"""
LIDAR SCANNER CLASS - For active scanning with servo-mounted LiDAR
"""

import time
import numpy as np
import serial

class LidarScanner:
    """
    Controls TF-Luna LiDAR with servo for active scanning
    Provides the scan data format that AI expects
    """
    
    def __init__(self, servo_controller, servo_channel=1, lidar_port='/dev/serial0'):
        """
        servo_controller: PCA9685Controller instance
        servo_channel: Channel for eye servo (default 1)
        lidar_port: Serial port for TF-Luna
        """
        self.servo = servo_controller
        self.channel = servo_channel
        self.center = 31.5  # Servo position for straight ahead
        self.current_pos = self.center
        
        # Scanning parameters
        self.scan_range = (0, 50)  # Servo units
        self.scan_angles = [0, 12.5, 25, 37.5, 50]  # 5-point scan
        
        # Initialize LiDAR
        self.lidar_initialized = False
        self.lidar = None
        
        try:
            self.lidar = serial.Serial(
                port=lidar_port,
                baudrate=115200,
                timeout=0.1
            )
            self.lidar_initialized = True
            print(f"✅ LiDAR initialized on {lidar_port}")
        except Exception as e:
            print(f"⚠️ Could not initialize LiDAR: {e}")
            print("⚠️ Using simulated LiDAR data")
        
        # Center the scanner
        self.center_scanner()
        
        print(f"✅ LidarScanner initialized on servo channel {servo_channel}")
    
    def center_scanner(self):
        """Center the LiDAR looking straight ahead"""
        self.servo.set_angle(self.channel, self.center)
        self.current_pos = self.center
        time.sleep(0.3)
        return self.center
    
    def set_servo_angle(self, angle):
        """Set servo to specific angle (0-50 servo units)"""
        angle = max(self.scan_range[0], min(self.scan_range[1], angle))
        self.servo.set_angle(self.channel, angle)
        self.current_pos = angle
        time.sleep(0.08)  # Short settling time
        return angle
    
    def read_lidar_distance(self, samples=3):
        """
        Read distance from TF-Luna LiDAR
        Returns: distance in cm, or None if error
        """
        if not self.lidar_initialized or self.lidar is None:
            # Return simulated data
            base_dist = 100 + (25 - abs(self.current_pos - 25)) * 2
            noise = np.random.uniform(-10, 10)
            return max(10, min(1200, base_dist + noise))
        
        try:
            distances = []
            for _ in range(samples):
                # TF-Luna communication protocol
                self.lidar.reset_input_buffer()
                self.lidar.write(b'\x5a\x05\x00\x01\x60')
                time.sleep(0.01)
                
                if self.lidar.in_waiting >= 9:
                    data = self.lidar.read(9)
                    if len(data) == 9 and data[0] == 0x5A:
                        # Parse distance (bytes 2-3, little endian)
                        dist_cm = (data[2] + data[3] * 256) / 100.0
                        if 1 < dist_cm < 1200:
                            distances.append(dist_cm)
                
                time.sleep(0.01)
            
            if distances:
                return np.median(distances)
            
        except Exception as e:
            print(f"LiDAR read error: {e}")
        
        return None
    
    def quick_scan(self, angles=None):
        """
        Perform 5-point scan
        Returns: List of (angle, distance) tuples
        """
        if angles is None:
            angles = self.scan_angles
        
        scan_results = []
        
        for angle in angles:
            # Move servo to angle
            self.set_servo_angle(angle)
            
            # Read distance
            distance = self.read_lidar_distance(samples=2)
            
            if distance is None:
                distance = 1200  # Max range if no reading
            
            scan_results.append((angle, distance))
        
        # Return to center
        self.center_scanner()
        
        return scan_results
    
    def get_distance_at(self, angle, move_servo=True):
        """
        Get distance at specific angle
        move_servo: If True, move servo to angle. If False, estimate.
        """
        if move_servo:
            old_pos = self.current_pos
            self.set_servo_angle(angle)
            distance = self.read_lidar_distance()
            self.set_servo_angle(old_pos)
            return distance
        else:
            # Estimate based on nearest scan points
            # This is less accurate but faster
            return self.read_lidar_distance()
    
    def get_current_distance(self):
        """Get distance at current servo position"""
        return self.read_lidar_distance()
    
    def full_sweep(self, step=5):
        """Complete sweep from min to max angle"""
        scan_results = []
        
        current = self.scan_range[0]
        while current <= self.scan_range[1]:
            self.set_servo_angle(current)
            distance = self.read_lidar_distance(samples=1)
            scan_results.append((current, distance if distance else 1200))
            current += step
        
        self.center_scanner()
        return scan_results
    
    def test_scanning(self):
        """Test the scanner"""
        print("\n🔍 Testing LidarScanner...")
        print("Performing 5-point scan:")
        
        scan_data = self.quick_scan()
        for angle, dist in scan_data:
            print(f"  Angle {angle:5.1f}° → {dist:6.1f}cm")
        
        print(f"Current position: {self.current_pos}°")
        print(f"Current distance: {self.get_current_distance():.1f}cm")
        
        return scan_data

if __name__ == "__main__":
    # Test the LidarScanner
    print("LidarScanner test - requires servo controller")
    
    # Mock for testing
    class MockServo:
        def set_angle(self, channel, angle):
            print(f"Mock servo {channel} → {angle}°")
    
    scanner = LidarScanner(MockServo(), servo_channel=1)
    scanner.test_scanning()