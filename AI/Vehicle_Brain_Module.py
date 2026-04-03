import sys
import time

# Add your project path
sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

from Hardware.DC_Motor import DCMotor, MotorDirection
from Hardware.PCA_Board import PCA9685
from Hardware.Units.CarSteering import SteeringController
from Hardware.Lidar_Sensor import TFLuna

# ===================== INIT HARDWARE =====================
motor = DCMotor(
    rpwm_pin=4,
    lpwm_pin=17,
    ren_pin=27,
    len_pin=22,
    motor_name="MainDrive"
)

pca = PCA9685()
steering = SteeringController(pca, servo_channel=0, min_angle=0, max_angle=60)
lidar = TFLuna(port="/dev/serial0")

# Center servo and steering
steering.center()

# ===================== CONFIG =====================
FRONT_SAFE_DISTANCE = 50  # cm, slow down if below
EMERGENCY_DISTANCE = 20   # cm, stop immediately
SCAN_ANGLES = [230, 200, 175, 150, 125, 105, 80, 55, 30]  # LiDAR scan angles

MIN_SPEED = 20
MAX_SPEED = 80
SCAN_DELAY = 0.05  # 50ms per angle

# ===================== UTILS =====================
def map_speed(front_distance):
    """Map front distance to speed"""
    if front_distance <= EMERGENCY_DISTANCE:
        return 0
    safe_dist = FRONT_SAFE_DISTANCE
    speed = int(MIN_SPEED + (MAX_SPEED - MIN_SPEED) * min(front_distance, safe_dist) / safe_dist)
    return max(MIN_SPEED, min(MAX_SPEED, speed))

def map_steering(left_dist, right_dist, min_angle=0, max_angle=60):
    """Map left/right distances to steering angle"""
    total = left_dist + right_dist
    if total == 0:
        return (min_angle + max_angle) // 2
    # More space on left -> turn left (lower angle)
    angle = int(min_angle + (max_angle - min_angle) * (right_dist / total))
    return max(min_angle, min(max_angle, angle))

def quick_scan():
    """Scan multiple angles to find free path"""
    readings = []
    for angle in SCAN_ANGLES:
        pca.channel_map[1].rotate(angle)  # Eye servo
        time.sleep(SCAN_DELAY)
        distance = lidar.read_distance()
        readings.append((angle, distance))
    return readings

# ===================== MAIN LOOP =====================
try:
    print("🚗 Vehicle brain started. Ctrl+C to stop.")
    
    while True:
        # 1. Check front distance
        front_distance = lidar.read_distance()
        
        # 2. Emergency stop
        if front_distance <= EMERGENCY_DISTANCE:
            print(f"⚠️ Obstacle too close ({front_distance}cm)! Stopping.")
            motor.stop(brake=True)
            steering.center()
            # Optionally: wait until cleared
            while lidar.read_distance() <= EMERGENCY_DISTANCE:
                time.sleep(0.05)
            continue
        
        # 3. Normal speed
        speed = map_speed(front_distance)
        motor.move_forward(speed)
        
        # 4. If front clear, go straight
        if front_distance > FRONT_SAFE_DISTANCE:
            steering.center()
            continue
        
        # 5. If obstacle detected ahead, do a quick scan
        scan_results = quick_scan()
        
        # Map scan results: left (<125°), right (>125°)
        left_dist = max(d for a, d in scan_results if a <= 125)
        right_dist = max(d for a, d in scan_results if a > 125)
        
        # Map distances to steering
        angle = map_steering(left_dist, right_dist)
        steering.set_angle(angle)
        
        # Optional: slow down while turning near obstacle
        motor.move_forward(int(speed * 0.7))
        
        time.sleep(0.05)  # Fast reaction

except KeyboardInterrupt:
    print("\n🔴 Exiting safely...")

finally:
    motor.emergency_stop()
    steering.center()
    if hasattr(lidar, 'close'):
        lidar.close()
    print("✅ Vehicle shutdown complete.")