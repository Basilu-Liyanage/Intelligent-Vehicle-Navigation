# Vehicle_Brain_Module_pi_safe.py
"""
Pi-safe Vehicle Brain Module
Runs SAC autonomous driving on Raspberry Pi without matplotlib
"""

import sys
import time
import types
from pathlib import Path

# ---------------- Monkeypatch matplotlib for SB3 ----------------
fake_matplotlib = types.ModuleType("matplotlib")
fake_matplotlib.figure = types.SimpleNamespace(Figure=lambda *a, **k: None)
sys.modules["matplotlib"] = fake_matplotlib
sys.modules["matplotlib.pyplot"] = types.SimpleNamespace()

# ---------------- Add project path ----------------
PROJECT_PATH = "/home/pi/Desktop/Intelligent-Vehicle-Navigation"
sys.path.append(PROJECT_PATH)

# ---------------- Hardware imports ----------------
from Hardware.DC_Motor import DCMotor, MotorDirection
from Hardware.PCA_Board import PCA9685
from Hardware.Units.CarSteering import SteeringController
from Hardware.Units.CarEye import MultiAngleLiDAR

# ---------------- Stable Baselines3 ----------------
from stable_baselines3 import SAC

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
lidar_eye = MultiAngleLiDAR()  # Use for multi-angle obstacle scanning

# ---------------- Load SAC model ----------------
MODEL_PATH = Path(PROJECT_PATH) / "checkpoints" / "sac_demo_120000_steps.zip"
model = SAC.load(str(MODEL_PATH))

# ---------------- Control Loop ----------------
def get_state_from_lidar():
    """Return simplified state for AI: e.g., front distance + left/right distances"""
    scan = lidar_eye.scan_row()  # returns distances for all angles
    # Example: front, front-left, front-right
    front = scan[len(scan)//2]
    left = scan[0]
    right = scan[-1]
    return [front, left, right]

try:
    print("🚗 Vehicle Brain Online. Starting autonomous control loop...")
    while True:
        state = get_state_from_lidar()
        action, _ = model.predict(state, deterministic=True)

        # ---------------- Interpret SAC output ----------------
        throttle = float(action[0])  # example: 0-1 forward speed
        steer = float(action[1])     # example: -1 left to +1 right

        # Motor control
        if throttle >= 0:
            motor.move_forward(min(throttle * 100, 100))
        else:
            motor.move_reverse(min(-throttle * 100, 100))

        # Steering control
        current_angle = 90 + int(steer * 30)  # scale -1:+1 -> 60:120 deg
        steering.set_angle(current_angle)

        # Small delay for Pi
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n🛑 Shutdown requested by user")
finally:
    motor.emergency_stop()
    steering.center()
    pca.reset()
    print("✅ All hardware safely stopped")