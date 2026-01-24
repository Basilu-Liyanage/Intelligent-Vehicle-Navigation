"""
Pixhawk_Sensors.py
Read sensor data from Pixhawk via MAVLink
"""

import time
import serial
from typing import Optional, Tuple, List
import struct

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    print("⚠️  pymavlink not installed. Install with: pip install pymavlink")
    MAVLINK_AVAILABLE = False

class PixhawkReader:
    """
    Read sensor data from Pixhawk
    Connects to Pixhawk via serial and reads IMU, GPS, Compass, etc.
    """
    
    def __init__(self, port: str = "/dev/ttyAMA0", baudrate: int = 57600):
        """
        Initialize connection to Pixhawk
        
        Args:
            port: Serial port (default: /dev/ttyAMA0 for RPi)
            baudrate: Baud rate (default: 57600 for Pixhawk telemetry)
        """
        self.port = port
        self.baudrate = baudrate
        self.connection = None
        self.connected = False
        
        # Latest sensor readings
        self.latest_gps = None
        self.latest_imu = None
        self.latest_compass = None
        self.latest_barometer = None
        self.latest_attitude = None
        
        # Timestamps
        self.last_update = time.time()
        
        print(f"🔧 Initializing Pixhawk reader on {port}...")
        
        if not MAVLINK_AVAILABLE:
            print("❌ pymavlink not available, running in simulation mode")
            self._init_simulation()
            return
        
        self._connect()
    
    def _connect(self):
        """Establish connection to Pixhawk"""
        max_attempts = 3
        
        for attempt in range(max_attempts):
            try:
                print(f"Attempt {attempt + 1} to connect to Pixhawk...")
                
                # Create MAVLink connection
                self.connection = mavutil.mavlink_connection(
                    self.port,
                    baud=self.baudrate,
                    source_system=255,  # Our system ID
                    source_component=0,
                    autoreconnect=True
                )
                
                # Wait for heartbeat
                print("Waiting for Pixhawk heartbeat...")
                self.connection.wait_heartbeat(timeout=5)
                
                print("✅ Pixhawk connected!")
                print(f"   System: {self.connection.target_system}")
                print(f"   Component: {self.connection.target_component}")
                
                # Request data streams
                self._request_data_streams()
                
                self.connected = True
                return
                
            except Exception as e:
                print(f"⚠️  Connection attempt {attempt + 1} failed: {e}")
                if attempt < max_attempts - 1:
                    time.sleep(1)
        
        print("❌ Failed to connect to Pixhawk, using simulation mode")
        self._init_simulation()
    
    def _init_simulation(self):
        """Initialize simulation mode for testing"""
        self.connected = False
        self.simulation_mode = True
        
        # Simulated initial values
        self.latest_gps = (37.7749, -122.4194, 10.0, 0.0, 8)  # SF coordinates
        self.latest_imu = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # pitch, roll, yaw, accel x,y,z
        self.latest_compass = 180.0  # South
        self.latest_barometer = (10.0, 0.0)  # altitude, vertical speed
        self.latest_attitude = (0.0, 0.0, 0.0)  # pitch, roll, yaw
        
        print("✅ Running in simulation mode")
    
    def _request_data_streams(self):
        """Request data streams from Pixhawk - IMPROVED VERSION"""
        if not self.connected:
            return
        
        try:
            print("📡 Configuring Pixhawk data streams...")
            
            # Wait a bit for connection to stabilize
            time.sleep(0.5)
            
            # Method 1: Use SET_MESSAGE_INTERVAL (better for PX4)
            try:
                # Request GPS_RAW_INT at 5Hz
                self.connection.mav.command_long_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,  # Confirmation
                    mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
                    200000,  # 200ms = 5Hz (in microseconds)
                    0, 0, 0, 0, 0
                )
                
                # Request ATTITUDE at 10Hz
                self.connection.mav.command_long_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                    100000,  # 100ms = 10Hz
                    0, 0, 0, 0, 0
                )
                
                # Request VFR_HUD at 4Hz (for heading and speed)
                self.connection.mav.command_long_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
                    250000,  # 250ms = 4Hz
                    0, 0, 0, 0, 0
                )
                
                print("✅ Data stream requests sent")
                
            except Exception as e:
                print(f"⚠️  SET_MESSAGE_INTERVAL failed: {e}")
                
                # Fallback: Old method
                print("Trying fallback method...")
                self.connection.mav.request_data_stream_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    5,  # 5 Hz
                    1   # Start
                )
            
            # Give Pixhawk time to start streaming
            time.sleep(1.0)
            
        except Exception as e:
            print(f"⚠️  Failed to request data streams: {e}")
    
    def update(self):
        """Update all sensor readings from Pixhawk"""
        if not self.connected:
            self._update_simulation()
            return
        
        try:
            # Process incoming messages
            message = self.connection.recv_match(blocking=False)
            
            while message is not None:
                self._process_message(message)
                message = self.connection.recv_match(blocking=False)
            
            self.last_update = time.time()
            
        except Exception as e:
            print(f"⚠️  Error updating Pixhawk data: {e}")
            if time.time() - self.last_update > 5.0:
                print("⚠️  No data from Pixhawk for 5 seconds, attempting reconnect...")
                self._connect()
    
    def _process_message(self, msg):
        """Process incoming MAVLink messages"""
        msg_type = msg.get_type()
        
        if msg_type == "GPS_RAW_INT":
            # GPS data
            lat = msg.lat / 1e7  # Convert from deg*1e7 to degrees
            lon = msg.lon / 1e7  # Convert from deg*1e7 to degrees
            alt = msg.alt / 1000.0  # Convert from mm to meters
            speed = msg.vel / 100.0  # Convert from cm/s to m/s
            satellites = msg.satellites_visible
            
            self.latest_gps = (lat, lon, alt, speed, satellites)
            
        elif msg_type == "RAW_IMU":
            # Raw IMU data (accelerometer, gyro, magnetometer)
            # Note: These are raw values, may need scaling
            xacc = msg.xacc / 1000.0  # mG to G? Depends on sensor
            yacc = msg.yacc / 1000.0
            zacc = msg.zacc / 1000.0
            
            # For now, store raw values
            self.latest_imu = (0, 0, 0, xacc, yacc, zacc)  # pitch, roll, yaw placeholder
            
        elif msg_type == "ATTITUDE":
            # Attitude (pitch, roll, yaw in radians)
            pitch = msg.pitch
            roll = msg.roll
            yaw = msg.yaw
            
            # Convert to degrees
            pitch_deg = pitch * 57.2958
            roll_deg = roll * 57.2958
            yaw_deg = yaw * 57.2958
            
            self.latest_attitude = (pitch_deg, roll_deg, yaw_deg)
            
            # Update IMU with attitude
            if self.latest_imu:
                _, _, _, xacc, yacc, zacc = self.latest_imu
                self.latest_imu = (pitch_deg, roll_deg, yaw_deg, xacc, yacc, zacc)
        
        elif msg_type == "VFR_HUD":
            # Airspeed, groundspeed, heading, etc.
            heading = msg.heading  # Degrees
            self.latest_compass = heading
            
        elif msg_type == "SCALED_PRESSURE":
            # Barometric pressure
            press_abs = msg.press_abs  # hPa
            # Convert pressure to altitude (simplified)
            # Actual conversion requires reference pressure
            altitude = (1 - (press_abs / 1013.25) ** 0.190284) * 145366.45 * 0.3048
            self.latest_barometer = (altitude, 0.0)  # altitude in meters
    
    def _update_simulation(self):
        """Update simulated sensor data for testing"""
        if not self.simulation_mode:
            return
        
        # Simulate some movement
        import math
        current_time = time.time()
        
        # Simulate driving in a circle
        radius = 10.0  # meters
        angular_speed = 0.1  # rad/s
        
        angle = angular_speed * current_time
        
        # Update GPS position (small movements)
        base_lat = 37.7749
        base_lon = -122.4194
        
        # Convert angle to small lat/lon changes
        lat_offset = radius * math.cos(angle) / 111320.0  # ~111km per degree latitude
        lon_offset = radius * math.sin(angle) / (111320.0 * math.cos(math.radians(base_lat)))
        
        self.latest_gps = (
            base_lat + lat_offset,
            base_lon + lon_offset,
            10.0,  # altitude
            1.0,   # speed (m/s)
            8      # satellites
        )
        
        # Update IMU (turning)
        pitch = 0.0
        roll = 0.0
        yaw = math.degrees(angle) % 360
        
        # Small accelerations
        accel_x = math.cos(angle) * 0.1
        accel_y = math.sin(angle) * 0.1
        accel_z = 9.81  # gravity
        
        self.latest_imu = (pitch, roll, yaw, accel_x, accel_y, accel_z)
        
        # Update compass
        self.latest_compass = yaw
        
        # Update barometer (small altitude changes)
        altitude = 10.0 + math.sin(angle * 2) * 0.5
        vertical_speed = math.cos(angle * 2) * 0.1
        self.latest_barometer = (altitude, vertical_speed)
        
        # Update attitude
        self.latest_attitude = (pitch, roll, yaw)
    
    # ===== PUBLIC API =====
    
    def get_gps(self) -> Optional[Tuple]:
        """
        Get GPS data
        
        Returns:
            Tuple: (latitude, longitude, altitude_m, speed_m_s, satellites)
            or None if no data
        """
        self.update()
        return self.latest_gps
    
    def get_imu(self) -> Optional[Tuple]:
        """
        Get IMU data
        
        Returns:
            Tuple: (pitch_deg, roll_deg, yaw_deg, accel_x, accel_y, accel_z)
            or None if no data
        """
        self.update()
        return self.latest_imu
    
    def get_compass(self) -> Optional[float]:
        """
        Get compass heading
        
        Returns:
            float: Heading in degrees (0-360, 0=North)
            or None if no data
        """
        self.update()
        return self.latest_compass
    
    def get_barometer(self) -> Optional[Tuple]:
        """
        Get barometer data
        
        Returns:
            Tuple: (altitude_m, vertical_speed_m_s)
            or None if no data
        """
        self.update()
        return self.latest_barometer
    
    def get_attitude(self) -> Optional[Tuple]:
        """
        Get attitude data
        
        Returns:
            Tuple: (pitch_deg, roll_deg, yaw_deg)
            or None if no data
        """
        self.update()
        return self.latest_attitude
    
    def get_all_sensors(self) -> dict:
        """
        Get all sensor data at once
        
        Returns:
            dict: Dictionary with all sensor readings
        """
        self.update()
        
        return {
            'gps': self.latest_gps,
            'imu': self.latest_imu,
            'compass': self.latest_compass,
            'barometer': self.latest_barometer,
            'attitude': self.latest_attitude,
            'timestamp': time.time(),
            'connected': self.connected,
            'simulation': getattr(self, 'simulation_mode', False)
        }
    
    def is_connected(self) -> bool:
        """Check if connected to Pixhawk"""
        return self.connected
    
    def close(self):
        """Close connection to Pixhawk"""
        if self.connection:
            try:
                self.connection.close()
                print("✅ Pixhawk connection closed")
            except:
                pass
        
        self.connected = False

# ===== SIMPLE TEST =====

if __name__ == "__main__":
    print("🧪 Testing Pixhawk Sensors...")
    
    # Try real connection first, fall back to simulation
    pixhawk = PixhawkReader(port="/dev/ttyAMA0")
    
    try:
        print("\n📊 Getting sensor data (Ctrl+C to stop):")
        print("-" * 60)
        
        for i in range(10):
            data = pixhawk.get_all_sensors()
            
            print(f"\nReading {i+1}:")
            
            if data['gps']:
                lat, lon, alt, speed, sats = data['gps']
                print(f"  GPS: {lat:.6f}, {lon:.6f} | Alt: {alt:.1f}m | Speed: {speed:.1f}m/s | Sats: {sats}")
            
            if data['compass']:
                print(f"  Compass: {data['compass']:.1f}°")
            
            if data['imu']:
                pitch, roll, yaw, acc_x, acc_y, acc_z = data['imu']
                print(f"  IMU: Pitch: {pitch:.1f}° | Roll: {roll:.1f}° | Yaw: {yaw:.1f}°")
                print(f"       Accel: X:{acc_x:.2f} Y:{acc_y:.2f} Z:{acc_z:.2f}")
            
            print(f"  Mode: {'SIMULATION' if data['simulation'] else 'REAL'}")
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n⏹️ Stopped by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        pixhawk.close()