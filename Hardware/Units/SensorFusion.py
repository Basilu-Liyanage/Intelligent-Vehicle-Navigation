import time
import sys
import numpy as np
sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

from Hardware.PCA_Board import PCA9685
from Hardware.Lidar_Sensor import TFLuna
from Hardware.Units.CarEye import MultiAngleLiDAR
class MAP:
    def __init__(self):
        self.lidar = TFLuna()
        self.pca = PCA9685()
        self.CarEye = MultiAngleLiDAR()
        self.map = np.zeros((10, 8))
        e1, e2, b = 0, 0, 0
        
    def add_row(self, new_row):
        self.map[1:] = self.map[:-1]   # shift down
        self.map[0] = new_row  

        
    def _map(self, e1, l1, l2,l3, c, r1, r2, r3, e2, b):
        new_row = np.array([e1, l1, l2, l3, c, r1, r2, r3, e2, b])
        self.add_row(new_row)
        
    def sensor_data(self):
        return self.map[0]

    def update(self):
        lidar_data = self.CarEye.scan_row()
        l1, l2, l3, c, r1, r2, r3 = lidar_data
        self._map(e1=0, l1=l1, l2=l2, l3=l3, c=c, r1=r1, r2=r2, r3=r3, e2=0, b=0) 