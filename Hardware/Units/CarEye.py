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
        row = []
        for angle in self.angles:
            self.set_servo(angle)
            time.sleep(0.1)  
            distance = self.lidar.read_distance()
            row.append(distance)
        return row

# Example usage:
lidar_map = MultiAngleLiDAR()

while True:
    latest = lidar_map.update_map()
    print("Latest scan:", latest)
    time.sleep(0.1) 