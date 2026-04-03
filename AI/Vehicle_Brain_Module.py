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
SCAN_RANGE = [100, 110, 115, 135, 140]  # Only scan when needed
LOOP_DELAY = 0.09       # Avoid PCA crashing

# ================= INIT =================
pca = PCA9685()
motor = DCMotor(rpwm_pin=4, lpwm_pin=17, ren_pin=27, len_pin=22)
steering = SteeringController(pca, servo_channel=0, min_angle=0, max_angle=60)
lidar = TFLuna()

# Center LiDAR
pca.channel_map[EYE_CHANNEL].rotate(CENTER_EYE_ANGLE)
steering.center()
last_steering = 35

# ================= FUNCTIONS =================
def read_front():
    return lidar.read_distance()

def adaptive_speed(distance):
    if distance > FRONT_CLEAR:
        return MAX_SPEED
    elif distance < FRONT_STOP:
        return 0
    else:
        return int(MIN_SPEED + (distance - FRONT_STOP) / (FRONT_CLEAR - FRONT_STOP) * (MAX_SPEED - MIN_SPEED))

def weighted_steering():
    """Scan sides only if front is near obstacle"""
    front = read_front()
    if front > FRONT_CLEAR:
        return 35  # center

    left_sum, right_sum = 0, 0
    left_count, right_count = 0, 0

    for angle in SCAN_RANGE:
        pca.channel_map[EYE_CHANNEL].rotate(angle)
        time.sleep(0.05)  # slower per step
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

    # Proportional steering 0=left, 60=right
    if left_avg + right_avg == 0:
        return 35
    steering_angle = 35 + int((right_avg - left_avg) / (left_avg + right_avg) * 25)
    steering_angle = max(0, min(60, steering_angle))
    return steering_angle

# ================= MAIN LOOP =================
try:
    while True:
        front = read_front()
        if front <= FRONT_STOP:
            print(f"🚨 Emergency stop! Front obstacle {front}cm")
            motor.stop(brake=True)
            steering.center()
            last_steering = 35
            while read_front() <= FRONT_STOP:
                time.sleep(0.05)
            continue

        speed = adaptive_speed(front)
        motor.move_forward(speed)

        # Adjust steering smoothly
        angle = weighted_steering()
        if abs(angle - last_steering) > 3:  # smooth change
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