import sys, time
sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

from Hardware.DC_Motor import DCMotor, MotorDirection
from Hardware.PCA_Board import PCA9685
from Hardware.Units.CarSteering import SteeringController
from Hardware.Lidar_Sensor import TFLuna

# ===== INIT =====
motor = DCMotor(rpwm_pin=4, lpwm_pin=17, ren_pin=27, len_pin=22, motor_name="MainDrive")
pca = PCA9685()
steering = SteeringController(pca, servo_channel=0, min_angle=0, max_angle=60)
lidar = TFLuna(port="/dev/serial0")

steering.center()

# ===== CONFIG =====
FRONT_SAFE_DISTANCE = 60  # cm
EMERGENCY_DISTANCE = 20   # cm
MIN_SPEED = 15
MAX_SPEED = 50
SCAN_ANGLES = [30, 55, 80, 105, 125, 150, 175, 200, 230]
SCAN_DELAY = 0.05

# ===== HELPERS =====
def map_speed(front_distance):
    if front_distance <= EMERGENCY_DISTANCE:
        return 0
    speed = int(MIN_SPEED + (MAX_SPEED - MIN_SPEED) * min(front_distance, FRONT_SAFE_DISTANCE)/FRONT_SAFE_DISTANCE)
    return max(MIN_SPEED, min(MAX_SPEED, speed))

def map_steering(left_dist, right_dist, min_angle=0, max_angle=60):
    total = left_dist + right_dist
    if total == 0:
        return (min_angle + max_angle) // 2
    angle = int(min_angle + (max_angle - min_angle) * (right_dist / total))
    return max(min_angle, min(max_angle, angle))

def quick_scan():
    readings = []
    for angle in SCAN_ANGLES:
        pca.channel_map[1].rotate(angle)
        time.sleep(SCAN_DELAY)
        readings.append((angle, lidar.read_distance()))
    return readings

# ===== MAIN LOOP =====
try:
    print("🚗 Vehicle brain started.")
    while True:
        front_distance = lidar.read_distance()

        # Emergency stop
        if front_distance <= EMERGENCY_DISTANCE:
            print(f"⚠️ Emergency! Front obstacle {front_distance}cm")
            motor.stop(brake=True)
            steering.center()
            while lidar.read_distance() <= EMERGENCY_DISTANCE:
                time.sleep(0.05)
            continue

        # Speed mapping based on front only
        speed = map_speed(front_distance)
        motor.move_forward(speed)

        # If front clear, go straight
        if front_distance > FRONT_SAFE_DISTANCE:
            steering.center()
            continue

        # Front obstacle detected: quick scan
        scan_results = quick_scan()
        left_dist = max(d for a, d in scan_results if a < 125)
        right_dist = max(d for a, d in scan_results if a >= 125)
        angle = map_steering(left_dist, right_dist)
        steering.set_angle(angle)

        # Slow down while maneuvering
        motor.move_forward(int(speed * 0.6))
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n🔴 Exiting safely...")

finally:
    motor.emergency_stop()
    steering.center()
    if hasattr(lidar, 'close'):
        lidar.close()
    print("✅ Vehicle shutdown complete.")