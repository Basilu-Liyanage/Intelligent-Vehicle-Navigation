"""
PixhawkReader.py
Read sensor data from Pixhawk via MAVLink
"""

import time
import math

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    print("⚠️ pymavlink not installed, running in simulation mode")
    MAVLINK_AVAILABLE = False


class PixhawkReader:
    """
    Reads GPS, IMU, Compass, Barometer, and Attitude data
    from Pixhawk via MAVLink, or simulation mode if not connected
    """

    def __init__(self, port="/dev/ttyAMA0", baudrate=57600):
        self.port = port
        self.baudrate = baudrate
        self.connected = False
        self.simulation_mode = False
        self.connection = None
        self.last_update = time.time()

        # Sensor data placeholders
        self.latest_gps = None
        self.latest_imu = None
        self.latest_compass = None
        self.latest_barometer = None
        self.latest_attitude = None

        if MAVLINK_AVAILABLE:
            self._connect()
        else:
            self._init_simulation()

    def _connect(self):
        """Attempt to connect to Pixhawk"""
        try:
            self.connection = mavutil.mavlink_connection(
                self.port, baud=self.baudrate
            )
            print("Waiting for Pixhawk heartbeat...")
            self.connection.wait_heartbeat(timeout=5)
            print("✅ Pixhawk connected")
            self.connected = True
            self._request_streams()
        except Exception as e:
            print(f"⚠️ Connection failed: {e}")
            self._init_simulation()

    def _init_simulation(self):
        """Setup simulation mode"""
        self.simulation_mode = True
        self.connected = False
        # Example simulated data
        self.latest_gps = (37.7749, -122.4194, 10.0, 0.0, 8)
        self.latest_imu = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.latest_compass = 180.0
        self.latest_barometer = (10.0, 0.0)
        self.latest_attitude = (0.0, 0.0, 0.0)
        print("✅ Running in simulation mode")

    def _request_streams(self):
        """Request Pixhawk data streams (basic method)"""
        self.connection.mav.request_data_stream_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            5,  # 5 Hz
            1
        )

    def update(self):
        """Update sensor readings"""
        if not self.connected:
            self._update_simulation()
            return

        try:
            msg = self.connection.recv_match(blocking=False)
            while msg:
                self._process_msg(msg)
                msg = self.connection.recv_match(blocking=False)
            self.last_update = time.time()
        except Exception:
            # Reconnect if no data for a while
            if time.time() - self.last_update > 5:
                print("⚠️ No data, reconnecting...")
                self._connect()

    def _process_msg(self, msg):
        """Process a single MAVLink message"""
        t = msg.get_type()

        if t == "GPS_RAW_INT":
            self.latest_gps = (
                msg.lat / 1e7,
                msg.lon / 1e7,
                msg.alt / 1000.0,
                msg.vel / 100.0,
                msg.satellites_visible
            )
        elif t == "ATTITUDE":
            pitch = math.degrees(msg.pitch)
            roll = math.degrees(msg.roll)
            yaw = math.degrees(msg.yaw)
            self.latest_attitude = (pitch, roll, yaw)
            # Also update IMU placeholder
            self.latest_imu = (pitch, roll, yaw, 0.0, 0.0, 0.0)
        elif t == "VFR_HUD":
            self.latest_compass = msg.heading
        elif t == "SCALED_PRESSURE":
            altitude = (1 - (msg.press_abs / 1013.25) ** 0.190284) * 145366.45 * 0.3048
            self.latest_barometer = (altitude, 0.0)

    def _update_simulation(self):
        """Simulate some sensor changes"""
        t = time.time()
        angle = 0.1 * t
        lat_offset = 10 * math.cos(angle) / 111320.0
        lon_offset = 10 * math.sin(angle) / (111320.0 * math.cos(math.radians(37.7749)))
        self.latest_gps = (37.7749 + lat_offset, -122.4194 + lon_offset, 10.0, 1.0, 8)
        yaw = math.degrees(angle) % 360
        self.latest_imu = (0.0, 0.0, yaw, 0.1*math.cos(angle), 0.1*math.sin(angle), 9.81)
        self.latest_compass = yaw
        self.latest_barometer = (10 + math.sin(angle*2)*0.5, math.cos(angle*2)*0.1)
        self.latest_attitude = (0.0, 0.0, yaw)

    # ===== Public API =====
    def get_all_sensors(self):
        """Return all sensors as a dictionary"""
        self.update()
        return {
            "gps": self.latest_gps,
            "imu": self.latest_imu,
            "compass": self.latest_compass,
            "barometer": self.latest_barometer,
            "attitude": self.latest_attitude,
            "timestamp": time.time(),
            "connected": self.connected,
            "simulation": self.simulation_mode
        }

    def close(self):
        """Close connection"""
        if self.connection:
            try:
                self.connection.close()
                print("✅ Pixhawk connection closed")
            except:
                pass
        self.connected = False


# =========================
# TEST RUN (safe printing)
# =========================
if __name__ == "__main__":
    pixhawk = PixhawkReader(port="/dev/ttyACM0")
    try:
        for i in range(10):
            data = pixhawk.get_all_sensors()
            print(f"\nReading {i+1}:")
            
            gps = data['gps']
            print(f"GPS: {gps if gps else 'No data'}")
            
            compass = data['compass']
            print(f"Compass: {compass:.1f}°" if compass is not None else "Compass: No data")
            
            imu = data['imu']
            print(f"IMU: {imu if imu else 'No data'}")
            
            baro = data['barometer']
            print(f"Barometer: {baro if baro else 'No data'}")
            
            attitude = data['attitude']
            print(f"Attitude: {attitude if attitude else 'No data'}")
            
            print(f"Mode: {'SIMULATION' if data['simulation'] else 'REAL'}")
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        pixhawk.close()