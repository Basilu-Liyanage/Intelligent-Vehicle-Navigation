import time
from Hardware.PCA_Board import PCA9685
from Hardware.Lidar_Sensor import TFLuna
from Hardware.DC_Motor import DCMotor
from Hardware.Units.CarSteering import SteeringController

# ================= CONFIG =================
FRONT_CLEAR = 80        # cm
FRONT_STOP = 25         # cm
MAX_SPEED = 100
MIN_SPEED = 20
LOOP_DELAY = 0.09       # safe for PCA board
EYE_CHANNEL = 1
CENTER_EYE_ANGLE = 125
STEERING_CENTER = 35
STEERING_MIN = 0
STEERING_MAX = 60
SMALL_SCAN_ANGLES = [120, 125, 130]  # small scan while moving
WIDE_SCAN_ANGLES = [100, 110, 115, 135, 140]  # wide scan when obstacle detected

# ================= INIT =================
pca = PCA9685()
motor = DCMotor(rpwm_pin=4, lpwm_pin=17, ren_pin=27, len_pin=22)
steering = SteeringController(pca, servo_channel=0, min_angle=STEERING_MIN, max_angle=STEERING_MAX)
lidar = TFLuna()

pca.channel_map[EYE_CHANNEL].rotate(CENTER_EYE_ANGLE)
steering.center()
last_steering = STEERING_CENTER
current_speed = 0

# ================= FUNCTIONS =================
def read_front():
    return lidar.read_distance()

def smooth_speed(target, current, step=10):
    if current < target:
        return min(current + step, target)
    elif current > target:
        return max(current - step, target)
    return current

def scan_and_decide(angles):
    readings = []
    for angle in angles:
        pca.channel_map[EYE_CHANNEL].rotate(angle)
        time.sleep(0.03)
        readings.append((angle, lidar.read_distance()))
    pca.channel_map[EYE_CHANNEL].rotate(CENTER_EYE_ANGLE)
    
    # Decide steering: angle with max distance is safest
    best_angle, _ = max(readings, key=lambda x: x[1])
    # Map LiDAR angle to servo steering
    if best_angle < CENTER_EYE_ANGLE:
        steering_angle = STEERING_CENTER - (CENTER_EYE_ANGLE - best_angle)//2
    else:
        steering_angle = STEERING_CENTER + (best_angle - CENTER_EYE_ANGLE)//2

    return max(STEERING_MIN, min(STEERING_MAX, steering_angle))

# ================= MAIN LOOP =================
try:
    while True:
        front = read_front()

        # Emergency stop
        if front <= FRONT_STOP:
            print(f"🚨 Stop! Obstacle {front}cm ahead")
            motor.stop(brake=True)
            steering.center()
            last_steering = STEERING_CENTER
            current_speed = 0
            while read_front() <= FRONT_STOP:
                time.sleep(0.05)
            continue

        # Speed adapts to front distance
        target_speed = MIN_SPEED + (front - FRONT_STOP)/(FRONT_CLEAR - FRONT_STOP)*(MAX_SPEED - MIN_SPEED)
        target_speed = max(MIN_SPEED, min(MAX_SPEED, target_speed))
        current_speed = smooth_speed(target_speed, current_speed, step=10)
        motor.move_forward(int(current_speed))

        # Steering
        if front < FRONT_CLEAR:
            # Obstacle detected → wide scan
            angle = scan_and_decide(WIDE_SCAN_ANGLES)
        else:
            # Default → small scan while moving
            angle = scan_and_decide(SMALL_SCAN_ANGLES)

        # Smooth steering updates
        if abs(angle - last_steering) > 2:
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