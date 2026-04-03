import sys
import time
import numpy as np
import signal

# ---------------- PATHS ----------------
sys.path.append('C:/Users/user_/OneDrive/Documents/Intelligent-Vehicle-Navigation')

# ---------------- HARDWARE ----------------
from Hardware.DC_Motor import DCMotor, MotorDirection
from Hardware.PCA_Board import PCA9685
from Hardware.Units.CarSteering import SteeringController
from Hardware.Lidar_Sensor import TFLuna
from Hardware.Units.CarEye import MultiAngleLiDAR

# ---------------- RL MODEL ----------------
from stable_baselines3 import SAC  # Ensure SB3 is installed

MODEL_PATH = 'C:/Users/user_/OneDrive/Documents/Intelligent-Vehicle-Navigation/checkpoints/sac_demo_120000_steps.zip'

# ---------------- SAFETY LIMITS ----------------
MAX_THROTTLE = 30  # in %
MIN_LIDAR_DISTANCE = 15  # cm, minimum safe distance to obstacle
STEERING_MIN = 60
STEERING_MAX = 120

# ---------------- INIT ----------------
print("Initializing hardware...")
motor = DCMotor()
pca_board = PCA9685()
steering = SteeringController(pca_board, servo_channel=0, min_angle=STEERING_MIN, max_angle=STEERING_MAX)
lidar_front = TFLuna()
car_eye = MultiAngleLiDAR()

print("Loading SAC model...")
model = SAC.load(MODEL_PATH)

# ---------------- SAFETY ----------------
def emergency_stop(signum=None, frame=None):
    print("\n🚨 EMERGENCY STOP!")
    motor.emergency_stop()
    steering.center()
    sys.exit(0)

signal.signal(signal.SIGINT, emergency_stop)
signal.signal(signal.SIGTERM, emergency_stop)

# ---------------- OBSERVATION ----------------
def get_observation():
    """
    Observation is an array:
    - Front LiDAR distance (cm)
    - Multi-angle LiDAR distances (list)
    - Steering angle
    """
    front_dist = lidar_front.read_distance()
    front_dist = max(front_dist, MIN_LIDAR_DISTANCE)
    
    eye_scan = car_eye.scan_row()  # 9-angle scan
    obs = np.array([front_dist] + eye_scan + [steering.get_current_angle()], dtype=np.float32)
    return obs

# ---------------- ACTIONS ----------------
def apply_action(action):
    """
    action: np.array([steer_norm, throttle_norm])
    - steer_norm: -1 to 1
    - throttle_norm: -1 to 1
    """
    steer_val = STEERING_MIN + ((action[0] + 1)/2) * (STEERING_MAX - STEERING_MIN)
    steering.set_angle(int(steer_val))
    
    throttle = np.clip(action[1], -1, 1)
    throttle_pct = abs(throttle) * MAX_THROTTLE
    
    if throttle >= 0:
        motor.move_forward(throttle_pct)
    else:
        motor.move_reverse(throttle_pct)

# ---------------- MAIN LOOP ----------------
print("Starting SAC controller...")
try:
    while True:
        obs = get_observation()
        action, _ = model.predict(obs, deterministic=True)
        apply_action(action)
        time.sleep(0.05)  # 20 Hz loop
except KeyboardInterrupt:
    emergency_stop()
except Exception as e:
    print(f"\n❌ Error: {e}")
    emergency_stop()