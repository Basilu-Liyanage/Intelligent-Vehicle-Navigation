import sys
import time

# Append your project path
sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

from Hardware.DC_Motor import DCMotor, MotorDirection
from Hardware.PCA_Board import PCA9685
from Hardware.Units.CarSteering import SteeringController
from Hardware.Units.CarEye import MultiAngleLiDAR

# ===================== INIT HARDWARE =====================
# Motor
motor = DCMotor(rpwm_pin=4, lpwm_pin=17, ren_pin=27, len_pin=22, motor_name="MainDrive")

# PCA Board for servos
pca_board = PCA9685()

# Steering
steering = SteeringController(pca_board, servo_channel=0, min_angle=60, max_angle=120)

# CarEye for LiDAR scanning
car_eye = MultiAngleLiDAR()

# ===================== RULE-BASED PARAMETERS =====================
SAFE_DISTANCE_CM = 40   # Minimum distance to obstacle
FORWARD_SPEED = 40      # Percent
TURN_STEP = 15          # Steering step for obstacle avoidance
SCAN_DELAY = 0.15       # Seconds between servo movements

# ===================== HELPER FUNCTIONS =====================
def detect_obstacle():
    """
    Scans LiDAR at multiple angles and returns True if obstacle is too close.
    """
    row = car_eye.scan_row()
    front_distance = row[len(row)//2]  # Center of the LiDAR sweep
    return front_distance < SAFE_DISTANCE_CM, row

def avoid_obstacle(lidar_row):
    """
    Simple avoidance: if obstacle closer on left, turn right; vice versa.
    """
    left = sum(lidar_row[:len(lidar_row)//2])
    right = sum(lidar_row[len(lidar_row)//2+1:])
    
    if left < right:
        # More space on right, turn right
        steering.step_right(TURN_STEP)
        print("Turning Right to avoid obstacle")
    else:
        # More space on left, turn left
        steering.step_left(TURN_STEP)
        print("Turning Left to avoid obstacle")

# ===================== MAIN LOOP =====================
try:
    print("🚗 Rule-based vehicle started. Press Ctrl+C to stop.")
    
    # Center steering at start
    steering.center()
    
    while True:
        obstacle, lidar_row = detect_obstacle()
        
        if obstacle:
            # Stop and avoid
            motor.stop(brake=True)
            print("Obstacle detected! Stopping...")
            avoid_obstacle(lidar_row)
            time.sleep(0.5)  # Give time for steering to adjust
        else:
            # Move forward
            motor.move_forward(FORWARD_SPEED)
        
        time.sleep(SCAN_DELAY)

except KeyboardInterrupt:
    print("\n⏹️ Stopping vehicle...")

finally:
    # Safe shutdown
    motor.emergency_stop()
    steering.center()
    pca_board.reset()
    print("✅ Vehicle safely stopped and reset")