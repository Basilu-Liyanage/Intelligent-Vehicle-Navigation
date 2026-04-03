# Vehicle_Brain_Module.py
import sys
import os
import time
import types
import numpy as np

# -------------------- BYPASS MATPLOTLIB --------------------
# SB3 imports matplotlib for logging; Pi environment fails
os.environ["MPLBACKEND"] = "Agg"  # prevent GUI backend
sys.modules['matplotlib'] = types.ModuleType('matplotlib')
sys.modules['matplotlib.pyplot'] = types.ModuleType('pyplot')
sys.modules['matplotlib.figure'] = types.ModuleType('figure')

# -------------------- IMPORTS --------------------
sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

from Hardware.PCA_Board import PCA9685
from Hardware.Lidar_Sensor import TFLuna
from Hardware.DC_Motor import DCMotor, MotorDirection
from Hardware.Units.CarSteering import SteeringController
from Hardware.Units.CarEye import MultiAngleLiDAR

from stable_baselines3 import SAC  # Your model

# -------------------- INIT HARDWARE --------------------
motor = DCMotor(
    rpwm_pin=4,
    lpwm_pin=17,
    ren_pin=27,
    len_pin=22,
    motor_name="MainDrive"
)

pca = PCA9685()
steering = SteeringController(pca, servo_channel=0, min_angle=60, max_angle=120)
eye = MultiAngleLiDAR()

# -------------------- LOAD MODEL --------------------
MODEL_PATH = '/home/pi/Desktop/Intelligent-Vehicle-Navigation/checkpoints/sac_demo_120000_steps.zip'
model = SAC.load(MODEL_PATH)

# -------------------- HELPER FUNCTIONS --------------------
def normalize_scan(scan):
    """Convert distances to normalized 0-1"""
    return [min(d / 800.0, 1.0) for d in scan]  # assuming max lidar 800 cm

def safety_check(front_distance_cm):
    """Stop if obstacle too close"""
    if front_distance_cm < 15:  # 15 cm safe distance
        motor.stop()
        steering.center()
        return False
    return True

def get_observation():
    """Returns model input"""
    scan = eye.scan_row()[:8]          # Take first 8 angles
    scan_norm = normalize_scan(scan)
    steer_norm = (steering.get_current_angle() - 60) / 60  # normalize to 0-1
    obs = np.array(scan_norm + [steer_norm], dtype=np.float32)
    return obs

# -------------------- MAIN LOOP --------------------
try:
    print("🚗 Vehicle AI starting...")
    while True:
        obs = get_observation()
        front_distance = eye.scan_row()[0]  # front-most LiDAR

        if not safety_check(front_distance):
            time.sleep(0.1)
            continue

        action, _states = model.predict(obs, deterministic=True)
        throttle, steer_delta = action

        # -------------------- APPLY ACTIONS --------------------
        # Steering
        current_angle = steering.get_current_angle()
        new_angle = current_angle + int(steer_delta * 10)  # scale small steps
        steering.set_angle(new_angle)

        # Motor
        if throttle > 0:
            motor.move_forward(min(max(throttle * 100, 10), 100))  # scale 0-1 → 10-100%
        else:
            motor.move_reverse(min(max(-throttle * 100, 10), 100))

        time.sleep(0.05)  # 20Hz control loop

except KeyboardInterrupt:
    print("\n🛑 Stopping vehicle...")
    motor.emergency_stop()
    steering.center()
    pca.reset()
    print("Shutdown complete.")

except Exception as e:
    print(f"\n❌ ERROR: {e}")
    motor.emergency_stop()
    steering.center()
    pca.reset()