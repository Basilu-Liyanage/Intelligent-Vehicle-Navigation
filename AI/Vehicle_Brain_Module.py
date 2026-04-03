import sys
import time

sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

from Hardware.PCA_Board import PCA9685
from Hardware.Lidar_Sensor import TFLuna
from Hardware.DC_Motor import DCMotor
from Hardware.Units.CarSteering import SteeringController

# ================= CONFIG =================
FRONT_CLEAR = 80        # cm
FRONT_STOP = 25         # cm
MAX_SPEED = 100
MIN_SPEED = 20
EYE_CHANNEL = 1
CENTER_EYE_ANGLE = 125
LOOP_DELAY = 0.09       # PCA safe
SCAN_ANGLES = [100, 110, 115, 135, 140]  # Wide scan when obstacle

# ================= INIT =================
pca = PCA9685()
motor = DCMotor(rpwm_pin=4, lpwm_pin=17, ren_pin=27, len_pin=22)
steering = SteeringController(pca, servo_channel=0, min_angle=0, max_angle=60)
lidar = TFLuna()

# Center LiDAR and steering
pca.channel_map[EYE_CHANNEL].rotate(CENTER_EYE_ANGLE)
steering.center()
last_steering = 35
current_speed = 0

# ================= FUNCTIONS =================
def read_front():
    return lidar.read_distance()

def smooth_speed(target, current, step=5):
    """Incrementally adjust speed towards target"""
    if current < target:
        return min(current + step, target)
    elif current > target:
        return max(current - step, target)
    return current

def wide_scan():
    """Perform wide scan when obstacle ahead"""
    left_sum, right_sum = 0, 0
    left_count, right_count = 0, 0

    for angle in SCAN_ANGLES:
        pca.channel_map[EYE_CHANNEL].rotate(angle)
        time.sleep(0.05)
        dist = lidar.read_distance()
        if angle < CENTER_EYE_ANGLE:
            left_sum += dist
            left_count += 1
        else:
            right_sum += dist
            right_count += 1

    pca.channel_map[EYE_CHANNEL].rotate(CENTER_EYE_ANGLE)
    left_avg = left_sum / left_count if left_count else 0
    right_avg = right_sum / right_count if right_count else 0

    # Decide steering 0=left, 60=right
    if left_avg + right_avg == 0:
        return 35
    angle = 35 + int((right_avg - left_avg) / (left_avg + right_avg) * 25)
    return max(0, min(60, angle))

# ================= MAIN LOOP =================
try:
    while True:
        front = read_front()

        # Emergency stop
        if front <= FRONT_STOP:
            print(f"🚨 Stop! Obstacle {front}cm ahead")
            motor.stop(brake=True)
            steering.center()
            last_steering = 35
            current_speed = 0
            while read_front() <= FRONT_STOP:
                time.sleep(0.05)
            continue

        # Speed adapts to front distance
        target_speed = MIN_SPEED + (front - FRONT_STOP) / (FRONT_CLEAR - FRONT_STOP) * (MAX_SPEED - MIN_SPEED)
        target_speed = max(MIN_SPEED, min(MAX_SPEED, target_speed))
        current_speed = smooth_speed(target_speed, current_speed, step=5)
        motor.move_forward(int(current_speed))

        # Steering: wide scan if obstacle close, else go straight
        if front < FRONT_CLEAR:
            angle = wide_scan()
        else:
            angle = 35  # center

        # Smooth steering changes
        if abs(angle - last_steering) > 3:
            steering.set_angle(angle)
            last_steering = angle

        time.sleep(LOOP_DELAY)

except KeyboardInterrupt:
    print("\nStopping vehicle")
finally:
    motor.stop(brake=True)
    steering.center()
    pca.channel_map[EYE_CHANNEL].rotate(CENTER_EYE_ANGLE)
    motor.cleanup()
    print("Vehicle safely stopped")