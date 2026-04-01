import time
import sys
import numpy as np
sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

from Hardware.PCA_Board import PCA9685
from Hardware.Lidar_Sensor import TFLuna

class MAP:
    def __init__(self):
        self.lidar = TFLuna()
        self.pca = PCA9685()
        
        self.map = np.zeros((8, 8))

    def add_row(self, new_row):
        self.map = np.vstack([new_row, self.map[:-1]])

        
    def _map(self, e1, l1, l2, c, r1, r2, e2, b):
        new_row = np.array([e1, l1, l2, c, r1, r2, e2, b])
        self.add_row(new_row)

