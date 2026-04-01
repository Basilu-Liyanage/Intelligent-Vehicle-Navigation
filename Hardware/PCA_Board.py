import smbus2
import time
from enum import Enum

# ================= CONFIG =================
BUS = 3
PCA_ADDR = 0x40
FREQ = 50  # Servo frequency

MIN_PULSE = 500
MAX_PULSE = 2500

# ---------------- SERVO SETTINGS ----------------
# Driver servo
DRIVER_DEFAULT_ANGLE = 35
DRIVER_ACTUATION_RANGE = 160
DRIVER_MAX_ANGLE = 60
DRIVER_MIN_ANGLE = 0
DRIVER_CHANNEL = 0

# Rear servo
REAR_DEFAULT_ANGLE = 35
REAR_ACTUATION_RANGE = 160
REAR_MAX_ANGLE = 60
REAR_MIN_ANGLE = 0
REAR_CHANNEL = 2

# Eye servo
EYE_DEFAULT_ANGLE = 125
EYE_ACTUATION_RANGE = 230
EYE_MAX_ANGLE = 230
EYE_MIN_ANGLE = 30
EYE_CHANNEL = 1

# ================= ENUMS =================
class ServoIds(Enum):
    Driver = DRIVER_CHANNEL
    Rear = REAR_CHANNEL
    Looker = EYE_CHANNEL

# ================= SERVO KIT =================
class ServoKit:
    def __init__(self, channels=16):
        self.channels = channels
        self.servo = {}
        for i in range(channels):
            self.servo[i] = None
        self._init_pca9685()

    def _init_pca9685(self):
        self.bus = smbus2.SMBus(BUS)
        # Reset
        self._write_reg(0x00, 0x00)
        time.sleep(0.01)
        prescale_val = int(25000000.0 / (4096 * FREQ) - 1)
        # Sleep
        self._write_reg(0x00, 0x10)
        self._write_reg(0xFE, prescale_val)
        # Wake
        self._write_reg(0x00, 0x00)
        time.sleep(0.01)
        # Restart
        self._write_reg(0x00, 0x80)

    def _write_reg(self, reg, value):
        self.bus.write_byte_data(PCA_ADDR, reg, value)

    def _angle_to_pulse(self, angle, actuation_range=180):
        pulse_us = MIN_PULSE + (angle / actuation_range) * (MAX_PULSE - MIN_PULSE)
        pulse_length = 1000000.0 / FREQ / 4096
        pulse_val = int(pulse_us / pulse_length)
        return pulse_val

    def _set_servo_pulse(self, channel, pulse):
        led0_on_l = 0x06 + 4 * channel
        self._write_reg(led0_on_l, 0)
        self._write_reg(led0_on_l + 1, 0)
        self._write_reg(led0_on_l + 2, pulse & 0xFF)
        self._write_reg(led0_on_l + 3, (pulse >> 8) & 0xFF)

    def set_angle(self, channel, angle, actuation_range=180):
        pulse = self._angle_to_pulse(angle, actuation_range)
        self._set_servo_pulse(channel, pulse)

# ================= PCA SERVO =================
class PcaServo:
    def __init__(self, kit: ServoKit, channel, act_range, min_angle, max_angle, mid_angle):
        self.kit = kit
        self.channel = channel
        self.actuation_range = act_range
        self.MIN_ANGLE = min_angle
        self.MAX_ANGLE = max_angle
        self.MID_ANGLE = mid_angle
        self.angle = mid_angle
        self.rotate(self.angle)

    def rotate(self, angle: int) -> int:
        if angle > self.MAX_ANGLE:
            angle = self.MAX_ANGLE
            print(f"[Channel {self.channel}] Max limit hit")
        elif angle < self.MIN_ANGLE:
            angle = self.MIN_ANGLE
            print(f"[Channel {self.channel}] Min limit hit")
        self.kit.set_angle(self.channel, angle, self.actuation_range)
        self.angle = angle
        time.sleep(0.1)
        return self.angle

    def reset(self):
        self.rotate(self.MID_ANGLE)

# ================= PCA9685 BOARD =================
class PCA9685:
    def __init__(self):
        self.kit = ServoKit(channels=16)

        self.driver_servo = PcaServo(
            self.kit, DRIVER_CHANNEL,
            DRIVER_ACTUATION_RANGE,
            DRIVER_MIN_ANGLE,
            DRIVER_MAX_ANGLE,
            DRIVER_DEFAULT_ANGLE
        )

        self.rear_servo = PcaServo(
            self.kit, REAR_CHANNEL,
            REAR_ACTUATION_RANGE,
            REAR_MIN_ANGLE,
            REAR_MAX_ANGLE,
            REAR_DEFAULT_ANGLE
        )

        self.eye_servo = PcaServo(
            self.kit, EYE_CHANNEL,
            EYE_ACTUATION_RANGE,
            EYE_MIN_ANGLE,
            EYE_MAX_ANGLE,
            EYE_DEFAULT_ANGLE
        )

        self.channel_map = {
            DRIVER_CHANNEL: self.driver_servo,
            REAR_CHANNEL: self.rear_servo,
            EYE_CHANNEL: self.eye_servo
        }

    def reset(self):
        for servo in self.channel_map.values():
            servo.reset()

# ================= MAIN TEST =================
if __name__ == "__main__":
    pca = PCA9685()
    print("Board ready. Bus 3. Servo limits enforced.")

    while True:
        try:
            ch_input = input("Enter channel (0-15) or 'q' to quit: ").strip()
            if ch_input.lower() == 'q':
                break
            ch = int(ch_input)
            if ch not in pca.channel_map:
                print("Channel not initialized. Choose 0 (Driver), 1 (Eye), 2 (Rear)")
                continue

            angle_input = int(input(f"Enter angle for channel {ch}: "))
            pca.channel_map[ch].rotate(angle_input)
            print(f"Channel {ch} now at {pca.channel_map[ch].angle}°")
        except ValueError:
            print("Enter a valid number")
        except KeyboardInterrupt:
            break

    pca.reset()
    print("All servos reset to mid angles")