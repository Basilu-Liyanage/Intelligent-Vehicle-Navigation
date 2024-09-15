import time
import RPi.GPIO as GPIO
import Lidar_Driver

# New test line

class CarController:
    def __init__(self, step_pin, dir_pin, servo_pin, step_delay=0.01):
        """
        Initialize the car controller.

        :param step_pin: GPIO pin connected to the STEP pin of the stepper driver.
        :param dir_pin: GPIO pin connected to the DIR pin of the stepper driver.
        :param servo_pin: GPIO pin connected to the servo motor.
        :param step_delay: Delay between steps for the stepper motor.
        """
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.servo_pin = servo_pin
        self.step_delay = step_delay

        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.servo_pin, GPIO.OUT)

        # Initialize Servo Motor
        self.servo = GPIO.PWM(self.servo_pin, 50)  # 50Hz frequency
        self.servo.start(0)  # Initialize with neutral position

    def set_speed(self, steps, clockwise=True):
        """
        Set the speed and direction of the stepper motor.

        :param steps: Number of steps to move.
        :param clockwise: Direction of movement.
        """
        GPIO.output(self.dir_pin, GPIO.HIGH if clockwise else GPIO.LOW)

        for _ in range(steps):
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(self.step_delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(self.step_delay)

    def stop(self):
        """
        Stop the car by disabling the stepper motor.
        """
        GPIO.output(self.step_pin, GPIO.LOW)

    def set_steering(self, angle):
        """
        Set the steering angle of the servo motor.

        :param angle: Desired steering angle (range -90 to 90).
        """
        duty_cycle = self.angle_to_duty_cycle(angle)
        self.servo.ChangeDutyCycle(duty_cycle)
        time.sleep(0.5)  # Allow time for the servo to reach the position
        self.servo.ChangeDutyCycle(0)  # Stop sending signal

    def angle_to_duty_cycle(self, angle):
        """
        Convert the angle to a duty cycle for the servo motor.

        :param angle: Angle in degrees (-90 to 90).
        :return: Corresponding duty cycle.
        """
        return 2.5 + (angle + 90) / 18.0

    def move_forward(self, steps):
        """
        Move the car forward, using LIDAR to avoid obstacles.
        :param steps: Number of steps to move.
        """
        Lidar_Driver.Avoid_obstacles_and_move(read_tfluna_data)

        self.set_speed(steps, clockwise=True)

    def move_backward(self, steps):
        """
        Move the car backward by a specified number of steps.

        :param steps: Number of steps to move.
        """
        self.set_speed(steps, clockwise=False)

    def turn_left(self):
        """
        Turn the car left by adjusting the servo motor.
        """
        self.set_steering(-45)  # Adjust this value based on your setup

    def turn_right(self):
        """
        Turn the car right by adjusting the servo motor.
        """
        self.set_steering(45)  # Adjust this value based on your setup

    def center_steering(self):
        """
        Center the steering by setting the servo to neutral.
        """
        self.set_steering(0)

    def cleanup(self):
        """
        Cleanup GPIO settings.
        """
        self.servo.stop()
        GPIO.cleanup()


# Example usage
if __name__ == "__main__":
    # GPIO pins setup
    STEP_PIN = 21
    DIR_PIN = 20
    SERVO_PIN = 12

    car = CarController(STEP_PIN, DIR_PIN, SERVO_PIN)

    try:
        car.move_forward(200)  # Move forward by 200 steps
        time.sleep(1)
        car.turn_left()         # Turn left
        time.sleep(1)
        car.center_steering()   # Center steering
        time.sleep(1)
        car.move_backward(200)  # Move backward by 200 steps
    finally:
        car.cleanup()           # Cleanup on exit
