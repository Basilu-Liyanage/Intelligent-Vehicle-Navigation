# Vehicle_Brain_NoSB3.py
"""
Raspberry Pi Safe Vehicle Brain
No Stable Baselines3 required
"""

import sys
import time
from pathlib import Path
import numpy as np

# ---------------- Add project path ----------------
PROJECT_PATH = "/home/pi/Desktop/Intelligent-Vehicle-Navigation"
sys.path.append(PROJECT_PATH)

# ---------------- Hardware imports ----------------
from Hardware.DC_Motor import DCMotor, MotorDirection
from Hardware.PCA_Board import PCA9685
from Hardware.Units.CarSteering import SteeringController
from Hardware.Units.CarEye import MultiAngleLiDAR

# ---------------- Initialize Hardware ----------------
motor = DCMotor(
    rpwm_pin=4,
    lpwm_pin=17,
    ren_pin=27,
    len_pin=22,
    motor_name="MainDrive"
)

pca = PCA9685()
steering = SteeringController(pca, servo_channel=0, min_angle=60, max_angle=120)
lidar_eye = MultiAngleLiDAR()  # Multi-angle LiDAR

# ---------------- Load precomputed AI actions ----------------
# Example: a NumPy array of [throttle, steering] pairs for simplicity
# You can generate this offline with SAC and save as 'actions.npy'
AI_ACTIONS_FILE = Path(PROJECT_PATH) / "checkpoints" / "actions.npy"
if AI_ACTIONS_FILE.exists():
    actions = np.load(AI_ACTIONS_FILE)
    print(f"✅ Loaded {len(actions)} precomputed actions")
else:
    print("⚠️ No precomputed actions found, running dummy loop")
    actions = np.array([[0.5, 0.0]] * 1000)  # Forward at 50%, center steering

action_index = 0

# ---------------- Control Loop ----------------
def get_state_from_lidar():
    """Return simplified state for AI: front, left, right distances"""
    scan = lidar_eye.scan_row()
    front = scan[len(scan)//2]
    left = scan[0]
    right = scan[-1]
    return [front, left, right]

try:
    print("🚗 Vehicle Brain Online. Starting control loop...")
    while True:
        state = get_state_from_lidar()
        # ---------------- Get AI action ----------------
        action = actions[action_index % len(actions)]
        action_index += 1

        throttle = float(action[0])  # 0-1 forward
        steer = float(action[1])     # -1 to +1

        # ---------------- Motor control ----------------
        if throttle >= 0:
            motor.move_forward(min(throttle * 100, 100))
        else:
            motor.move_reverse(min(-throttle * 100, 100))

        # ---------------- Steering control ----------------
        current_angle = 90 + int(steer * 30)  # -1:+1 -> 60:120 deg
        steering.set_angle(current_angle)

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n🛑 Shutdown requested by user")
finally:
    motor.emergency_stop()
    steering.center()
    pca.reset()
    print("✅ All hardware safely stopped")