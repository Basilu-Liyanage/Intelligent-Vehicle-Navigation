import torch
import time
from Hardware.Lidar_Sensor import TFLuna
from Hardware.PCA_Board import PCA9685
from Hardware.Units.CarSteering import SteeringController
from Hardware.DC_Motor import DCMotor, MotorDirection

# Load TorchScript SAC model
model = torch.jit.load("/home/pi/Desktop/Intelligent-Vehicle-Navigation/AI/sac_model_ts.pt")
model.eval()

# Initialize hardware
lidar = TFLuna()
pca = PCA9685()
steering = SteeringController(pca, servo_channel=0, min_angle=0, max_angle=60)
motor = DCMotor(rpwm_pin=4, lpwm_pin=17, ren_pin=27, len_pin=22)

# Constants
SAFE_DISTANCE = 30.0  # cm
MAX_SPEED = 50        # percent
MIN_SPEED = 20        # percent

def get_observation():
    # Front distance only for now
    return torch.tensor([lidar.read_distance()], dtype=torch.float32)

try:
    while True:
        obs = get_observation()
        with torch.no_grad():
            action = model(obs)  # SAC output: tensor([steering, speed])
            steering_angle = float(action[0].item())
            target_speed = float(action[1].item())

        # Clamp and apply
        steering_angle = max(0, min(60, steering_angle))
        target_speed = max(MIN_SPEED, min(MAX_SPEED, target_speed))

        # Apply steering and speed
        steering.set_angle(steering_angle)

        if lidar.read_distance() < SAFE_DISTANCE:
            motor.stop(brake=True)
        else:
            motor.move_forward(target_speed)

        time.sleep(0.05)  # fast loop

except KeyboardInterrupt:
    motor.emergency_stop()
    print("Stopped")