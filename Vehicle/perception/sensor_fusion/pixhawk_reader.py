# Vehicle/perception/sensor_fusion/pixhawk_reader_working.py
"""Your working Pixhawk code integrated."""
import time

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    print("⚠️ pymavlink not installed. Install with: pip install pymavlink")
    MAVLINK_AVAILABLE = False

class PixhawkReader:
    """Your exact working Pixhawk code with compatibility methods."""
    
    def __init__(self, port: str = "/dev/ttyAMA0", baudrate: int = 57600):
        self.port = port
        self.baudrate = baudrate
        self.connection = None
        self.connected = False
        self.simulation_mode = False
        
        # Latest readings
        self.latest_gps = None
        self.latest_imu = None
        self.latest_compass = None
        self.latest_attitude = None
        
        print(f"🔧 Initializing Pixhawk on {port}...")
        
        if not MAVLINK_AVAILABLE:
            print("❌ pymavlink not available, using simulation")
            self._init_simulation()
            return
        
        self._connect()
    
    def connect(self):
        """Compatibility method - calls your connect logic."""
        if self.connected or self.simulation_mode:
            return True
        
        return self._connect()
    
    def _connect(self):
        """Your connection logic."""
        max_attempts = 3
        
        for attempt in range(max_attempts):
            try:
                print(f"Attempt {attempt + 1} to connect to Pixhawk...")
                
                self.connection = mavutil.mavlink_connection(
                    self.port,
                    baud=self.baudrate,
                    source_system=255,
                    source_component=0,
                    autoreconnect=True
                )
                
                print("Waiting for Pixhawk heartbeat...")
                self.connection.wait_heartbeat(timeout=5)
                
                print("✅ Pixhawk connected!")
                print(f"   System: {self.connection.target_system}")
                
                self._request_data_streams()
                self.connected = True
                return True
                
            except Exception as e:
                print(f"⚠️ Connection attempt {attempt + 1} failed: {e}")
                if attempt < max_attempts - 1:
                    time.sleep(1)
        
        print("❌ Failed to connect to Pixhawk, using simulation")
        self._init_simulation()
        return True  # Simulation mode is still "connected"
    
    def _init_simulation(self):
        """Your simulation initialization."""
        self.connected = False
        self.simulation_mode = True
        
        # Your simulated initial values
        self.latest_gps = (37.7749, -122.4194, 10.0, 0.0, 8)
        self.latest_imu = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.latest_compass = 180.0
        self.latest_attitude = (0.0, 0.0, 0.0)
        
        print("✅ Running in simulation mode")
        return True
    
    def _request_data_streams(self):
        """Your data stream request method."""
        if not self.connected:
            return
        
        try:
            print("📡 Configuring Pixhawk data streams...")
            time.sleep(0.5)
            
            try:
                # Request GPS_RAW_INT at 5Hz
                self.connection.mav.command_long_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
                    200000,
                    0, 0, 0, 0, 0
                )
                
                # Request ATTITUDE at 10Hz
                self.connection.mav.command_long_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                    100000,
                    0, 0, 0, 0, 0
                )
                
                print("✅ Data stream requests sent")
                
            except Exception as e:
                print(f"⚠️ SET_MESSAGE_INTERVAL failed: {e}")
                print("Trying fallback method...")
                self.connection.mav.request_data_stream_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    5,
                    1
                )
            
            time.sleep(1.0)
            
        except Exception as e:
            print(f"⚠️ Failed to request data streams: {e}")
    
    def update(self):
        """Your update method."""
        if not self.connected and not self.simulation_mode:
            return
        
        if self.simulation_mode:
            self._update_simulation()
            return
        
        try:
            message = self.connection.recv_match(blocking=False)
            
            while message is not None:
                self._process_message(message)
                message = self.connection.recv_match(blocking=False)
            
        except Exception as e:
            print(f"⚠️ Error updating Pixhawk data: {e}")
    
    def _process_message(self, msg):
        """Your message processing."""
        msg_type = msg.get_type()
        
        if msg_type == "GPS_RAW_INT":
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000.0
            speed = msg.vel / 100.0
            satellites = msg.satellites_visible
            
            self.latest_gps = (lat, lon, alt, speed, satellites)
            
        elif msg_type == "ATTITUDE":
            pitch = msg.pitch * 57.2958  # rad to deg
            roll = msg.roll * 57.2958
            yaw = msg.yaw * 57.2958
            
            self.latest_attitude = (pitch, roll, yaw)
            
            # Update IMU with attitude
            if self.latest_imu:
                _, _, _, xacc, yacc, zacc = self.latest_imu
                self.latest_imu = (pitch, roll, yaw, xacc, yacc, zacc)
        
        elif msg_type == "VFR_HUD":
            heading = msg.heading
            self.latest_compass = heading
    
    def _update_simulation(self):
        """Your simulation update."""
        if not self.simulation_mode:
            return
        
        import math
        current_time = time.time()
        
        # Simulate driving
        radius = 10.0
        angular_speed = 0.1
        angle = angular_speed * current_time
        
        base_lat = 37.7749
        base_lon = -122.4194
        
        lat_offset = radius * math.cos(angle) / 111320.0
        lon_offset = radius * math.sin(angle) / (111320.0 * math.cos(math.radians(base_lat)))
        
        self.latest_gps = (
            base_lat + lat_offset,
            base_lon + lon_offset,
            10.0,
            1.0,
            8
        )
        
        pitch = 0.0
        roll = 0.0
        yaw = math.degrees(angle) % 360
        
        accel_x = math.cos(angle) * 0.1
        accel_y = math.sin(angle) * 0.1
        accel_z = 9.81
        
        self.latest_imu = (pitch, roll, yaw, accel_x, accel_y, accel_z)
        self.latest_compass = yaw
        self.latest_attitude = (pitch, roll, yaw)
    
    # ===== COMPATIBILITY METHODS =====
    
    def get_state(self):
        """Get vehicle state in format needed by AI."""
        self.update()
        
        if self.latest_imu:
            pitch, roll, yaw_deg, acc_x, acc_y, acc_z = self.latest_imu
            yaw_rad = yaw_deg * 0.0174533  # deg to rad
        else:
            yaw_rad = 0.0
            acc_x = 0.0
        
        if self.latest_gps:
            _, _, _, speed, _ = self.latest_gps
        else:
            speed = 0.0
        
        return {
            'velocity_x': speed,  # Use GPS speed
            'velocity_y': 0.0,
            'velocity_z': 0.0,
            'yaw': yaw_rad,  # in radians
            'pitch': 0.0,
            'roll': 0.0,
            'latitude': self.latest_gps[0] if self.latest_gps else 0.0,
            'longitude': self.latest_gps[1] if self.latest_gps else 0.0,
            'altitude': self.latest_gps[2] if self.latest_gps else 0.0,
        }
    
    def get_all_sensors(self):
        """Get all sensor data."""
        return {
            'gps': self.latest_gps,
            'imu': self.latest_imu,
            'compass': self.latest_compass,
            'attitude': self.latest_attitude,
            'connected': self.connected,
            'simulation': self.simulation_mode
        }
    
    def close(self):
        """Close connection."""
        if self.connection:
            try:
                self.connection.close()
                print("✅ Pixhawk connection closed")
            except:
                pass
        
        self.connected = False