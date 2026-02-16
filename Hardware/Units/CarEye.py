#!/usr/bin/env python3
"""
CAR EYE CONTROLLER - 4 Degree Scanning with Adaptive Modes
- Mode 1: Full scan (8 points) for stationary/low speed
- Mode 2: Quick scan (3 points) for moving
- Original method names preserved for compatibility
"""

import time
from typing import Tuple, List, Optional
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

# NEW: Mode configurations
QUICK_SCAN_ANGLES = [EYE_MIN_ANGLE, EYE_DEFAULT_ANGLE, EYE_MAX_ANGLE]  # Left, Center, Right
FULL_SCAN_ANGLES = [15.0, 19.0, 23.0, 27.0, 31.5, 35.0, 39.0, 43.0, 46.0]  # 9 points covering full range

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
        
        # NEW: Cache for last scan results
        self.last_full_scan = []
        self.last_full_scan_time = 0
        self.last_quick_scan = []
        self.last_quick_scan_time = 0
        
        # Initialize eye to center
        self.reset()
        print(f"✅ CarEye initialized - Center: {EYE_DEFAULT_ANGLE}")
        print(f"   Full scan: {len(FULL_SCAN_ANGLES)} points")
        print(f"   Quick scan: {len(QUICK_SCAN_ANGLES)} points")
    
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
    
    # ==================== ORIGINAL METHODS (PRESERVED) ====================
    
    def scan_4_degrees(self) -> List[Tuple[float, float]]:
        """
        ORIGINAL METHOD - PRESERVED
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
            time.sleep(.1)
        # Return to center
        self.reset()
        
        # Cache the result
        self.last_full_scan = distances
        self.last_full_scan_time = time.time()
        
        print(f"📊 Scan complete: {len(distances)} readings")
        return distances
    
    def quick_scan_3_points(self) -> Tuple[float, float, float]:
        """
        ORIGINAL METHOD - PRESERVED
        Quick scan left, center, right (for faster decisions)
        """
        print("🚀 Quick 3-point scan:")
        points = []
        
        for angle in [EYE_MIN_ANGLE, EYE_DEFAULT_ANGLE, EYE_MAX_ANGLE]:
            self.set_angle(angle)
            time.sleep(SLEEP_TIME)
            distance = self.get_center_distance()
            points.append(distance)
            
            # Show reading
            position = "LEFT" if angle == EYE_MIN_ANGLE else "RIGHT" if angle == EYE_MAX_ANGLE else "CENTER"
            print(f"   {position:6s}: {angle:4.1f}° → {distance:6.1f}cm")
            time.sleep(.1)
        
        # Return to center
        self.reset()
        
        # Cache the result
        self.last_quick_scan = points
        self.last_quick_scan_time = time.time()
        
        return tuple(points)
    
    def get_moving_direction(self) -> Tuple[float, float]:
        """
        ORIGINAL METHOD - PRESERVED
        Find best direction to move using 4-degree scanning
        Returns: (best_distance, best_angle)
        """
        print("\n" + "="*60)
        print("🧭 FINDING BEST MOVING DIRECTION (4° scanning)")
        print("="*60)
        
        # Perform full scan
        scan_data = self.scan_4_degrees()
        time.sleep(.01)
        
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
    
    def smart_scan(self, target_angle: float = None):
        """
        ORIGINAL METHOD - PRESERVED
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
    
    # ==================== NEW ADAPTIVE SCANNING METHODS ====================
    
    def adaptive_scan(self, speed_percent: float) -> Tuple[List[Tuple[float, float]], str]:
        """
        NEW METHOD: Choose scan mode based on speed
        Args:
            speed_percent: Current speed as percentage (0-100 forward, negative for reverse)
        Returns:
            (scan_results, mode_name)
        """
        # If reversing, no scan
        if speed_percent < 0:
            print("🔙 Reverse mode - scanning paused")
            return [], "REVERSE"
        
        # Choose mode based on speed threshold (20%)
        if speed_percent <= 20:
            # Stationary or slow speed - full scan
            print(f"🐢 Low speed ({speed_percent:.0f}%) - Full scan mode")
            return self.perform_full_scan(), "FULL"
        else:
            # Moving - quick scan
            print(f"🚗 Moving ({speed_percent:.0f}%) - Quick scan mode")
            return self.perform_quick_scan(), "QUICK"
    
    def perform_full_scan(self) -> List[Tuple[float, float]]:
        """
        NEW METHOD: Full scan using predefined angles
        Returns list of (distance, angle) tuples
        """
        print("📡 Performing FULL scan (9 points)...")
        results = []
        
        # Store current position to return later
        original_angle = self.current_angle
        
        # Scan all predefined angles
        for angle in FULL_SCAN_ANGLES:
            self.set_angle(angle)
            time.sleep(.01)
            time.sleep(SLEEP_TIME)
            distance = self.get_center_distance()
            
            if 0 < distance < 800:
                results.append((distance, angle))
                print(f"   Angle: {angle:5.1f}° → Distance: {distance:6.1f}cm")
        
        # Return to original position
        self.set_angle(original_angle)
        time.sleep(.01)
        # Cache results
        self.last_full_scan = results
        self.last_full_scan_time = time.time()
        
        print(f"📊 Full scan complete: {len(results)} readings")
        return results
    
    def perform_quick_scan(self) -> List[Tuple[float, float]]:
        """
        NEW METHOD: Quick scan (left, center, right)
        Returns list of (distance, angle) tuples
        """
        print("⚡ Performing QUICK scan (3 points)...")
        results = []
        
        # Store current position to return later
        original_angle = self.current_angle
        
        # Scan left, center, right
        for angle in QUICK_SCAN_ANGLES:
            self.set_angle(angle)
            time.sleep(SLEEP_TIME)
            distance = self.get_center_distance()
            
            if 0 < distance < 800:
                results.append((distance, angle))
                
                # Show position label
                if angle == EYE_MIN_ANGLE:
                    pos = "LEFT"
                elif angle == EYE_MAX_ANGLE:
                    pos = "RIGHT"
                else:
                    pos = "CENTER"
                print(f"   {pos:6s}: {angle:4.1f}° → {distance:6.1f}cm")
            time.sleep(.1)
        # Return to original position
        self.set_angle(original_angle)
        
        # Cache results
        self.last_quick_scan = [d for d, _ in results]  # Store just distances for quick access
        self.last_quick_scan_time = time.time()
        
        print(f"⚡ Quick scan complete")
        return results
    
    def get_best_direction_from_scan(self, scan_results: List[Tuple[float, float]]) -> Tuple[float, float]:
        """
        NEW METHOD: Find best direction from scan results
        Returns: (best_distance, best_angle)
        """
        if not scan_results:
            return 0, self.current_angle
        
        # Find angle with maximum distance
        best_distance, best_angle = max(scan_results, key=lambda x: x[0])
        return best_distance, best_angle
    
    def get_scan_with_context(self, speed_percent: float) -> dict:
        """
        NEW METHOD: Get scan results with context information
        Returns dict with scan data and metadata
        """
        scan_results, mode = self.adaptive_scan(speed_percent)
        
        if not scan_results:
            return {
                'mode': mode,
                'success': False,
                'best_distance': 0,
                'best_angle': self.current_angle,
                'readings': []
            }
        
        best_distance, best_angle = self.get_best_direction_from_scan(scan_results)
        
        # Find center reading if available
        center_reading = None
        for d, a in scan_results:
            if abs(a - EYE_DEFAULT_ANGLE) < 2.0:
                center_reading = d
                break
        
        return {
            'mode': mode,
            'success': True,
            'best_distance': best_distance,
            'best_angle': best_angle,
            'center_distance': center_reading,
            'readings': scan_results,
            'timestamp': time.time()
        }
    
    # ==================== VISUALIZATION METHODS ====================
    
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
            scaled = int((distance - min_dist) / (max_dist - min_dist + 0.001) * 50) if max_dist > min_dist else 0
            bars = "█" * max(1, scaled)
            
            # Mark center and best angle
            if abs(angle - EYE_DEFAULT_ANGLE) < 1.0:
                marker = "🎯"
            elif abs(angle - best_angle) < 1.0:
                marker = "⭐"
            else:
                marker = "│"
            
            print(f"   {angle:5.1f}° {marker} {bars:50s} {distance:6.1f}cm")

# ============================================================================
# TEST FUNCTIONS (PRESERVED)
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

# ==================== NEW TEST FUNCTIONS ====================

def test_adaptive_scan():
    """Test the new adaptive scanning modes"""
    print("\n" + "="*60)
    print("🔄 TESTING ADAPTIVE SCANNING")
    print("="*60)
    
    pca = PCA9685Controller()
    eye = CarEye(pca)
    
    # Test different speed scenarios
    test_speeds = [-10, 0, 10, 30, 50, 80]
    
    for speed in test_speeds:
        print(f"\n{'='*40}")
        print(f"Speed: {speed}%")
        print(f"{'='*40}")
        
        result = eye.get_scan_with_context(speed)
        
        if result['success']:
            print(f"Mode: {result['mode']}")
            print(f"Best direction: {result['best_angle']:.1f}° → {result['best_distance']:.1f}cm")
            if result['center_distance']:
                print(f"Center distance: {result['center_distance']:.1f}cm")
        else:
            print(f"Mode: {result['mode']} - No scan")
        
        time.sleep(1)
    
    print("\n✅ Adaptive scan test complete")

# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    print("\n" + "="*80)
    print("🤖 CAR EYE CONTROLLER - ADAPTIVE SCANNING SYSTEM")
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
            print("2. Quick 3-Point Scan (Original)")
            print("3. Full 4-Degree Scan (Original)")
            print("4. Find Best Moving Direction (Original)")
            print("5. Custom Angle Scan")
            print("6. Reset to Center")
            print("7. Test Adaptive Scanning (NEW)")
            print("8. Exit")
            print("="*40)
            
            choice = input("\nSelect (1-8): ").strip()
            
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
                test_adaptive_scan()
            elif choice == '8':
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