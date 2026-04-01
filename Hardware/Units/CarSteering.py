class SteeringController:
    def __init__(self, pca, servo_channel=0, min_angle=60, max_angle=120):
        """
        pca: PCA9685 instance
        servo_channel: which channel your steering servo is on
        min_angle, max_angle: servo limits
        """
        self.pca = pca
        self.servo_channel = servo_channel
        self.min_angle = min_angle
        self.max_angle = max_angle

        # Start centered
        self.current_angle = (min_angle + max_angle) // 2
        self.set_angle(self.current_angle)

    def set_angle(self, angle):
        """Set steering to a specific angle (clamped to limits)"""
        angle = max(self.min_angle, min(self.max_angle, angle))
        self.current_angle = angle
        self.pca.channel_map[self.servo_channel].rotate(angle)
        print(f"Steering set to {angle}°")

    def step_left(self, step=5):
        """Turn left by step degrees"""
        self.set_angle(self.current_angle - step)

    def step_right(self, step=5):
        """Turn right by step degrees"""
        self.set_angle(self.current_angle + step)

    def center(self):
        """Center the steering"""
        center_angle = (self.min_angle + self.max_angle) // 2
        self.set_angle(center_angle)

    def get_current_angle(self):
        """Return current steering angle"""
        return self.current_angle