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

        self.angles = [230, 200, 175, 150, 125, 105, 80, 55, 30]

    def set_servo(self, angle):
        self.pca.channel_map[self.eye_servo_channel].rotate(angle)

    def scan_row(self):
        """Scan all angles and return the angle with the BEST (closest) distance"""
        best_angle = None
        best_distance = float('inf')   # Start with very large number
        distances = []                 # Optional: keep all data

        for angle in self.angles:
            self.set_servo(angle)
            time.sleep(0.09)           # Wait for servo to settle
            
            distance = self.lidar.read_distance()
            distances.append((angle, distance))
            
            # Update best if this distance is closer
            if distance < best_distance:
                best_distance = distance
                best_angle = angle

        print(f"Best angle: {best_angle}° | Distance: {best_distance} cm")
        return best_angle, best_distance 
