#!/usr/bin/env python3
"""
CAR EYE CONTROLLER - 4 Degree Scanning
Properly scans left and right in 4-degree increments
"""

import time
from typing import Tuple, List
import math
import sys

sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

from Hardware.PCA_Board import PCA9685Controller
from Hardware.Lidar_Sensor import TFLuna as TFLunaInterface

# ============================================================================
# CONFIGURATION
# ============================================================================

EYE_DEFAULT_ANGLE = 31.5  # Center position (0-50 range)
EYE_MIN_ANGLE = 15.0      # Left limit
EYE_MAX_ANGLE = 46.0      # Right limit
SCAN_STEP = 4.0           # Scan every 4 degrees
SLEEP_TIME = 0.08         # Time between LiDAR readings

# ============================================================================
# CAR EYE CLASS WITH PROPER 4-DEGREE SCANNING
# ============================================================================

class CarEye:
    def __init__(self, pca_controller: PCA9685Controller):
        self.pca = pca_controller
        self.lidar = TFLunaInterface('/dev/serial0')
        
        # Current state
        self.current_angle = EYE_DEFAULT_ANGLE
        self.angle = EYE_DEFAULT_ANGLE  # For compatibility with original code
        
        # Initialize eye to center
        self.reset()
        print(f"✅ CarEye initialized - Center: {EYE_DEFAULT_ANGLE}")
    
    def set_angle(self, angle: float) -> float:
        """
        Set eye to specific angle (0-50 range)
        
        Returns:
            The angle that was actually set
        """
        angle = max(EYE_MIN_ANGLE, min(EYE_MAX_ANGLE, angle))
        
        try:
            self.pca.eye.set_50_range(angle)
            self.current_angle = angle
            self.angle = angle  # Update both for compatibility
            time.sleep(0.05)
            return angle
        except Exception as e:
            print(f"⚠️  Eye servo error: {e}")
            return self.current_angle
    
    def get_center_distance(self) -> float:
        """Get distance measurement from center"""
        try:
            dist_m = self.lidar.get_distance_to_obstacle()
            return dist_m * 100.0 if dist_m else 999.9
        except:
            return 999.9
    
    def turn_right(self, angle_step: float = SCAN_STEP) -> Tuple[bool, float]:
        """
        Turn eye right by angle_step (4 degrees)
        
        Returns:
            (success, new_angle)
        """
        target_angle = self.current_angle - angle_step
        
        # Check if we can move
        if target_angle < EYE_MIN_ANGLE:
            return False, self.current_angle
        
        # Move to target
        old_angle = self.current_angle
        actual_angle = self.set_angle(target_angle)
        
        # Check if we moved at least 2 degrees
        is_moved = abs(actual_angle - old_angle) >= 2.0
        
        return is_moved, actual_angle
    
    def turn_left(self, angle_step: float = SCAN_STEP) -> Tuple[bool, float]:
        """
        Turn eye left by angle_step (4 degrees)
        
        Returns:
            (success, new_angle)
        """
        target_angle = self.current_angle + angle_step
        
        # Check if we can move
        if target_angle > EYE_MAX_ANGLE:
            return False, self.current_angle
        
        # Move to target
        old_angle = self.current_angle
        actual_angle = self.set_angle(target_angle)
        
        # Check if we moved at least 2 degrees
        is_moved = abs(actual_angle - old_angle) >= 2.0
        
        return is_moved, actual_angle
    
    def reset(self):
        """Reset eye to center position"""
        self.set_angle(EYE_DEFAULT_ANGLE)
        print(f"🎯 Eye reset to center: {EYE_DEFAULT_ANGLE}")
    
    def scan_4_degrees(self) -> List[Tuple[float, float]]:
        """
        Scan environment every 4 degrees
        Returns list of (distance, angle) pairs
        """
        print("🔍 Starting 4-degree scan...")
        distances = []
        
        # Start from far right and scan left
        self.set_angle(EYE_MAX_ANGLE)
        time.sleep(0.1)
        
        # Scan from right to left in 4-degree steps
        angle = EYE_MAX_ANGLE
        while angle >= EYE_MIN_ANGLE:
            # Set angle
            self.set_angle(angle)
            time.sleep(SLEEP_TIME)
            
            # Get distance
            distance = self.get_center_distance()
            
            # Only record valid readings
            if 0 < distance < 800:
                distances.append((distance, angle))
                print(f"   Angle: {angle:5.1f}° → Distance: {distance:6.1f}cm")
            
            # Move 4 degrees left
            angle -= SCAN_STEP
        
        # Return to center
        self.reset()
        
        print(f"📊 Scan complete: {len(distances)} readings")
        return distances
    
    def get_moving_direction(self) -> Tuple[float, float]:
        """
        Find best direction to move using 4-degree scanning
        Returns: (best_distance, best_angle)
        """
        print("\n" + "="*60)
        print("🧭 FINDING BEST MOVING DIRECTION (4° scanning)")
        print("="*60)
        
        # Perform full scan
        scan_data = self.scan_4_degrees()
        
        if not scan_data:
            print("⚠️  No valid distance readings")
            return 999.9, EYE_DEFAULT_ANGLE
        
        # Find best direction (maximum distance)
        best_distance, best_angle = max(scan_data, key=lambda x: x[0])
        
        # Find center distance for reference
        center_distances = [d for d, a in scan_data if abs(a - EYE_DEFAULT_ANGLE) < 2.0]
        center_distance = center_distances[0] if center_distances else 0
        
        print(f"\n📊 Scan Results:")
        print(f"   Total readings: {len(scan_data)}")
        print(f"   Center distance: {center_distance:.1f}cm")
        print(f"   Best angle: {best_angle:.1f}°")
        print(f"   Best distance: {best_distance:.1f}cm")
        print(f"   Improvement: +{best_distance - center_distance:.1f}cm")
        print("="*60)
        
        # Visual representation
        self.visualize_scan(scan_data, best_angle)
        
        return best_distance, best_angle
    
    def visualize_scan(self, scan_data: List[Tuple[float, float]], best_angle: float):
        """Create visual representation of the scan"""
        print("\n👁️  ENVIRONMENT MAP:")
        
        # Sort by angle
        scan_data.sort(key=lambda x: x[1])
        
        # Create angle buckets
        angles = [d[1] for d in scan_data]
        distances = [d[0] for d in scan_data]
        
        # Find min and max for scaling
        min_dist = min(distances) if distances else 0
        max_dist = max(distances) if distances else 100
        
        # Print visual map
        for angle, distance in zip(angles, distances):
            # Scale distance to 0-50 for visualization
            scaled = int((distance - min_dist) / (max_dist - min_dist) * 50) if max_dist > min_dist else 0
            bars = "█" * max(1, scaled)
            
            # Mark center and best angle
            if abs(angle - EYE_DEFAULT_ANGLE) < 1.0:
                marker = "🎯"
            elif abs(angle - best_angle) < 1.0:
                marker = "⭐"
            else:
                marker = "│"
            
            print(f"   {angle:5.1f}° {marker} {bars:50s} {distance:6.1f}cm")
    
    def quick_scan_3_points(self) -> Tuple[float, float, float]:
        """Quick scan left, center, right (for faster decisions)"""
        points = []
        
        for angle in [EYE_MIN_ANGLE, EYE_DEFAULT_ANGLE, EYE_MAX_ANGLE]:
            self.set_angle(angle)
            time.sleep(SLEEP_TIME)
            distance = self.get_center_distance()
            points.append(distance)
            
            # Show reading
            position = "LEFT" if angle == EYE_MIN_ANGLE else "RIGHT" if angle == EYE_MAX_ANGLE else "CENTER"
            print(f"   {position:6s}: {angle:4.1f}° → {distance:6.1f}cm")
        
        # Return to center
        self.reset()
        
        return tuple(points)
    
    def smart_scan(self, target_angle: float = None):
        """
        Smart scanning: 
        - If target_angle given, goes directly there
        - Otherwise performs 4-degree scan
        """
        if target_angle is not None:
            # Go directly to target angle
            print(f"🎯 Moving to target: {target_angle:.1f}°")
            self.set_angle(target_angle)
            time.sleep(SLEEP_TIME)
            distance = self.get_center_distance()
            return [(distance, target_angle)]
        else:
            # Full 4-degree scan
            return self.scan_4_degrees()

# ============================================================================
# TEST FUNCTIONS
# ============================================================================

def test_basic_movement():
    """Test basic eye movement"""
    print("\n" + "="*60)
    print("🧪 TESTING BASIC MOVEMENT")
    print("="*60)
    
    pca = PCA9685Controller()
    eye = CarEye(pca)
    
    # Test positions
    positions = [15.0, 20.0, 25.0, 31.5, 38.0, 43.0, 46.0]
    
    for pos in positions:
        print(f"\nMoving to {pos:4.1f}°...")
        eye.set_angle(pos)
        time.sleep(0.3)
        distance = eye.get_center_distance()
        print(f"   Distance: {distance:.1f}cm")
    
    eye.reset()
    print("\n✅ Basic movement test complete")

def test_4_degree_scan():
    """Test 4-degree scanning"""
    print("\n" + "="*60)
    print("🔍 TESTING 4-DEGREE SCANNING")
    print("="*60)
    
    pca = PCA9685Controller()
    eye = CarEye(pca)
    
    # Quick 3-point scan
    print("\n🚀 Quick 3-point scan:")
    left, center, right = eye.quick_scan_3_points()
    print(f"   Left: {left:.1f}cm, Center: {center:.1f}cm, Right: {right:.1f}cm")
    
    # Full 4-degree scan
    print("\n📡 Full 4-degree scan:")
    start_time = time.time()
    scan_data = eye.scan_4_degrees()
    scan_time = time.time() - start_time
    
    print(f"\n📊 Scan Statistics:")
    print(f"   Time taken: {scan_time:.2f}s")
    print(f"   Points scanned: {len(scan_data)}")
    print(f"   Avg time per point: {scan_time/len(scan_data):.3f}s")
    
    # Find best direction
    if scan_data:
        best_dist, best_angle = max(scan_data, key=lambda x: x[0])
        print(f"   Best direction: {best_angle:.1f}° → {best_dist:.1f}cm")
    
    print("\n✅ 4-degree scan test complete")

def test_moving_direction():
    """Test finding best moving direction"""
    print("\n" + "="*60)
    print("🧭 TESTING MOVING DIRECTION FINDER")
    print("="*60)
    
    pca = PCA9685Controller()
    eye = CarEye(pca)
    
    # Get best direction
    best_distance, best_angle = eye.get_moving_direction()
    
    print(f"\n🎯 RECOMMENDED ACTION:")
    print(f"   Move towards: {best_angle:.1f}°")
    print(f"   Expected clearance: {best_distance:.1f}cm")
    
    # Demonstrate moving to best angle
    print(f"\n🚗 Moving to best angle...")
    eye.set_angle(best_angle)
    time.sleep(0.5)
    
    # Verify distance at best angle
    distance = eye.get_center_distance()
    print(f"   Actual distance: {distance:.1f}cm")
    
    eye.reset()
    print("\n✅ Moving direction test complete")

# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    print("\n" + "="*80)
    print("🤖 CAR EYE CONTROLLER - 4° SCANNING SYSTEM")
    print("="*80)
    
    try:
        # Create PCA controller
        pca = PCA9685Controller()
        
        # Create CarEye instance
        eye = CarEye(pca)
        
        # Menu
        while True:
            print("\n" + "="*40)
            print("📋 CAR EYE CONTROL MENU")
            print("="*40)
            print("1. Test Basic Movement")
            print("2. Quick 3-Point Scan (Left-Center-Right)")
            print("3. Full 4-Degree Scan")
            print("4. Find Best Moving Direction")
            print("5. Custom Angle Scan")
            print("6. Reset to Center")
            print("7. Exit")
            print("="*40)
            
            choice = input("\nSelect (1-7): ").strip()
            
            if choice == '1':
                test_basic_movement()
            elif choice == '2':
                eye.quick_scan_3_points()
            elif choice == '3':
                eye.scan_4_degrees()
            elif choice == '4':
                eye.get_moving_direction()
            elif choice == '5':
                angle = float(input("Enter angle (15-46): "))
                eye.set_angle(angle)
                time.sleep(0.3)
                distance = eye.get_center_distance()
                print(f"Distance at {angle}°: {distance:.1f}cm")
            elif choice == '6':
                eye.reset()
            elif choice == '7':
                print("\n👋 Shutting down...")
                eye.reset()
                break
            else:
                print("❌ Invalid choice")
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n\n🛑 Stopped by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()