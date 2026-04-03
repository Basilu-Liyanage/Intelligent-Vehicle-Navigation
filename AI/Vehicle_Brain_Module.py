# ---------------- VEHICLE BRAIN MODULE ----------------
import sys
import time
sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

from Hardware.PCA_Board import PCA9685
from Hardware.Lidar_Sensor import TFLuna
from Hardware.Units.CarEye import MultiAngleLiDAR
from Hardware.Units.CarSteering import SteeringController
from Hardware.DC_Motor import DCMotor, MotorDirection

# ---------------- INITIALIZE HARDWARE ----------------
print("🟢 Initializing hardware...")

# Motor
motor = DCMotor(
    rpwm_pin=4,
    lpwm_pin=17,
    ren_pin=27,
    len_pin=22,
    motor_name="MainDrive"
)

# PCA board for servos
pca = PCA9685()

# Steering
steering = SteeringController(pca, servo_channel=0, min_angle=60, max_angle=120)

# Eye (multi-angle lidar)
lidar_eye = MultiAngleLiDAR()
lidar_eye.pca = pca  # Use the same PCA instance

# ---------------- CONTROL PARAMETERS ----------------
FRONT_STOP_DISTANCE = 30  # cm
SCAN_INTERVAL = 0.05      # seconds
MAX_SPEED = 50            # percent
STEERING_CENTER = 90
STEERING_RANGE = (60, 120)

print("🚗 Vehicle Brain Online. Starting reactive control loop...")

# ---------------- MAIN LOOP ----------------
try:
    while True:
        # Scan LiDAR
        scan = lidar_eye.scan_row()
        front = scan[len(scan)//2]
        left = scan[0]
        right = scan[-1]

        # ----- Obstacle avoidance -----
        if front < FRONT_STOP_DISTANCE:
            motor.stop()
            print(f"⚠️ Obstacle ahead ({front:.1f} cm)! Stopping.")
        else:
            # Dynamic speed (slow if close obstacle in front)
            speed = MAX_SPEED
            if front < 60:
                speed = int(MAX_SPEED * front/60)
            motor.move_forward(speed)

            # Dynamic steering
            if left < right:
                # More space on the right → steer right
                steer_angle = STEERING_CENTER + int((right-left)/max(right,left)*30)
            else:
                # More space on the left → steer left
                steer_angle = STEERING_CENTER - int((left-right)/max(left,right)*30)

            # Clamp steering to limits
            steer_angle = max(STEERING_RANGE[0], min(STEERING_RANGE[1], steer_angle))
            steering.set_angle(steer_angle)

        time.sleep(SCAN_INTERVAL)

except KeyboardInterrupt:
    print("\n🛑 Shutdown requested by user")

finally:
    # Safe shutdown
    motor.emergency_stop()
    steering.center()
    pca.reset()
    print("✅ All hardware safely stopped")