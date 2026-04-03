import time
import sys
sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

from Hardware.PCA_Board import PCA9685
from Hardware.Lidar_Sensor import TFLuna

class MultiAngleLiDAR:
    def __init__(self):
        self.lidar = TFLuna()
        self.pca = PCA9685()
        self.eye_servo_channel = 1  

        # Safe angles only
        self.angles = [200, 175, 150, 125, 105, 80, 55]

    def set_servo(self, angle):
        safe_angle = max(50, min(200, int(angle)))
        self.pca.channel_map[self.eye_servo_channel].rotate(safe_angle)

    def scan_row(self):
        """Scan and return the angle with the maximum distance (no center bias)"""
        best_angle = None
        best_distance = -float('inf')

        print("🔍 Scanning for best open direction...")

        for angle in self.angles:
            self.set_servo(angle)
            time.sleep(0.10)

            distance = self.lidar.read_distance()

            print(f"   Eye {angle:3}° → Dist {distance:4.1f}cm")

            if distance > best_distance:
                best_distance = distance
                best_angle = angle

        self.set_servo(125)  # Return to center
        time.sleep(0.25)

        print(f"✅ Chosen: {best_angle}° (Widest open path)")
        return best_angle, best_distance
