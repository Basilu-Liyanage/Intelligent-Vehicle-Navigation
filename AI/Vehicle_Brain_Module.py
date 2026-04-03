import sys, time
sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

from Hardware.PCA_Board import PCA9685
from Hardware.Lidar_Sensor import TFLuna
from Hardware.Units.CarEye import MultiAngleLiDAR
from Hardware.Units.CarSteering import SteeringController
from Hardware.DC_Motor import DCMotor, MotorDirection

# ---------------- INITIALIZE ----------------
motor = DCMotor(rpwm_pin=4, lpwm_pin=17, ren_pin=27, len_pin=22)
pca = PCA9685()
steering = SteeringController(pca, servo_channel=0, min_angle=60, max_angle=120)
lidar_eye = MultiAngleLiDAR()
lidar_eye.pca = pca

# ---------------- PARAMETERS ----------------
FRONT_STOP = 25         # cm, immediate stop
MAX_SPEED = 50          # %
STEERING_CENTER = 90
STEERING_MIN, STEERING_MAX = 60, 120
SCAN_INTERVAL = 0.03    # 30ms → faster reaction

# ---------------- HELPER ----------------
def compute_steering(scan):
    """
    Weighted steering: more distance → more influence
    scan: list of distances from left to right
    returns steering angle
    """
    n = len(scan)
    weights = [i - n//2 for i in range(n)]  # left negative, right positive
    weighted_sum = sum(w*d for w,d in zip(weights, scan))
    # Normalize
    max_dist = max(scan) if max(scan) > 0 else 1
    steering_offset = int((weighted_sum / (max_dist * (n//2))) * 30)
    angle = STEERING_CENTER + steering_offset
    return max(STEERING_MIN, min(STEERING_MAX, angle))

# ---------------- MAIN LOOP ----------------
try:
    while True:
        scan = lidar_eye.scan_row()
        front = scan[len(scan)//2]

        if front < FRONT_STOP:
            motor.stop()
            print(f"⚠️ Obstacle ahead ({front:.1f} cm)! Stopping.")
        else:
            speed = MAX_SPEED
            if front < 60:
                speed = int(MAX_SPEED * front / 60)
            motor.move_forward(speed)

            angle = compute_steering(scan)
            steering.set_angle(angle)

        time.sleep(SCAN_INTERVAL)

except KeyboardInterrupt:
    print("🛑 Shutdown requested")

finally:
    motor.emergency_stop()
    steering.center()
    pca.reset()
    print("✅ Shutdown complete")