"""
pixhawk_interface.py
Professional Pixhawk Interface for Intelligent Vehicle Navigation
"""
from pymavlink import mavutil
import threading
import time
import math
from dataclasses import dataclass
from typing import Optional, Tuple, List, Callable
from enum import Enum
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class VehicleState:
    """Complete vehicle state from Pixhawk"""
    timestamp: float
    latitude: float = 0.0          # degrees
    longitude: float = 0.0         # degrees
    altitude: float = 0.0          # meters
    ground_speed: float = 0.0      # m/s
    air_speed: float = 0.0         # m/s
    heading: float = 0.0           # degrees (0-360, N=0)
    roll: float = 0.0              # radians
    pitch: float = 0.0             # radians
    yaw: float = 0.0               # radians
    roll_rate: float = 0.0         # rad/s
    pitch_rate: float = 0.0        # rad/s
    yaw_rate: float = 0.0          # rad/s
    acceleration_x: float = 0.0    # m/s²
    acceleration_y: float = 0.0    # m/s²
    acceleration_z: float = 0.0    # m/s²
    gps_fix_type: int = 0          # 0-3 (0=no fix, 3=3D fix)
    gps_satellites: int = 0
    battery_voltage: float = 0.0   # volts
    battery_current: float = 0.0   # amps
    system_status: str = "UNKNOWN"
    flight_mode: str = "UNKNOWN"

class FlightMode(Enum):
    """Common Pixhawk flight modes (for vehicles)"""
    MANUAL = "MANUAL"
    STABILIZE = "STABILIZE"
    AUTO = "AUTO"
    GUIDED = "GUIDED"
    HOLD = "HOLD"
    RETURN_TO_LAUNCH = "RTL"
    BRAKE = "BRAKE"

class PixhawkInterface:
    """
    Professional Pixhawk interface for automotive applications.
    Thread-safe, with state caching, error recovery, and callback system.
    """
    
    def __init__(self, 
                 connection_string: str = '/dev/ttyACM0',
                 baudrate: int = 57600,
                 vehicle_id: int = 1,
                 update_rate_hz: float = 10.0):
        """
        Initialize Pixhawk interface.
        
        Args:
            connection_string: Serial port (e.g., '/dev/ttyACM0', '/dev/ttyAMA0')
            baudrate: Serial baud rate (57600, 115200, etc.)
            vehicle_id: MAVLink system ID
            update_rate_hz: Desired state update rate
        """
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.vehicle_id = vehicle_id
        self.update_rate = update_rate_hz
        
        # State management
        self.state = VehicleState(timestamp=time.time())
        self.state_lock = threading.Lock()
        self.connected = False
        self.running = False
        
        # MAVLink connection
        self.master: Optional[mavutil.mavlink_connection] = None
        
        # Threading
        self.read_thread: Optional[threading.Thread] = None
        self.heartbeat_thread: Optional[threading.Thread] = None
        
        # Callbacks
        self.state_callbacks: List[Callable[[VehicleState], None]] = []
        self.error_callbacks: List[Callable[[Exception], None]] = []
        
        # Statistics
        self.message_count = 0
        self.last_heartbeat_time = 0
        self.connection_attempts = 0
        
        logger.info(f"PixhawkInterface initialized for {connection_string}")
    
    def connect(self, timeout_seconds: int = 10) -> bool:
        """
        Connect to Pixhawk with retry logic.
        
        Args:
            timeout_seconds: Connection timeout
            
        Returns:
            True if connected successfully
        """
        logger.info(f"Connecting to Pixhawk at {self.connection_string}...")
        
        try:
            # Create MAVLink connection
            self.master = mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baudrate,
                source_system=255,  # Ground station ID
                source_component=0,
                autoreconnect=True
            )
            
            # Wait for heartbeat
            start_time = time.time()
            while time.time() - start_time < timeout_seconds:
                try:
                    heartbeat = self.master.wait_heartbeat(timeout=1)
                    if heartbeat:
                        self.connected = True
                        self.last_heartbeat_time = time.time()
                        
                        # Get system info
                        self.vehicle_id = heartbeat.get_srcSystem()
                        logger.info(f"✅ Connected to vehicle ID: {self.vehicle_id}")
                        
                        # Request data streams
                        self._setup_data_streams()
                        
                        # Start background threads
                        self._start_threads()
                        
                        return True
                        
                except Exception as e:
                    logger.debug(f"Heartbeat wait: {e}")
                    continue
                    
            logger.error("❌ Connection timeout - no heartbeat received")
            return False
            
        except Exception as e:
            logger.error(f"❌ Connection failed: {e}")
            self._notify_error(e)
            return False
    
    def _setup_data_streams(self):
        """Configure Pixhawk data streams at desired rates"""
        if not self.master:
            return
            
        # Request all necessary data streams
        streams = [
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5),    # GPS, position
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10),     # Attitude, rates
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 2),      # VFR HUD
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 1),      # AHRS
            (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 2), # IMU
        ]
        
        for stream_id, rate_hz in streams:
            self.master.mav.request_data_stream_send(
                self.vehicle_id,
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                stream_id,
                rate_hz,
                1  # Start streaming
            )
            time.sleep(0.01)
    
    def _start_threads(self):
        """Start background processing threads"""
        self.running = True
        
        # Start message reading thread
        self.read_thread = threading.Thread(
            target=self._message_read_loop,
            name="Pixhawk-Reader",
            daemon=True
        )
        self.read_thread.start()
        
        # Start heartbeat monitor thread
        self.heartbeat_thread = threading.Thread(
            target=self._heartbeat_monitor_loop,
            name="Pixhawk-Heartbeat",
            daemon=True
        )
        self.heartbeat_thread.start()
        
        logger.info("Background threads started")
    
    def _message_read_loop(self):
        """Main message processing loop (runs in background thread)"""
        logger.info("Message reader thread started")
        
        while self.running and self.connected:
            try:
                # Get next message with timeout
                msg = self.master.recv_match(blocking=True, timeout=0.1)
                if not msg:
                    continue
                    
                self.message_count += 1
                
                # Process based on message type
                self._process_message(msg)
                
                # Update state timestamp
                with self.state_lock:
                    self.state.timestamp = time.time()
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                logger.debug(f"Message read error: {e}")
                continue
    
    def _process_message(self, msg):
        """Process incoming MAVLink messages"""
        msg_type = msg.get_type()
        
        with self.state_lock:
            try:
                if msg_type == 'GLOBAL_POSITION_INT':
                    # GPS position and velocity
                    self.state.latitude = msg.lat / 1e7
                    self.state.longitude = msg.lon / 1e7
                    self.state.altitude = msg.alt / 1000.0  # mm to meters
                    self.state.ground_speed = math.sqrt(
                        msg.vx**2 + msg.vy**2
                    ) / 100.0  # cm/s to m/s
                    self.state.heading = msg.hdg / 100.0  # centidegrees to degrees
                    
                elif msg_type == 'ATTITUDE':
                    # Orientation
                    self.state.roll = msg.roll
                    self.state.pitch = msg.pitch
                    self.state.yaw = msg.yaw
                    self.state.roll_rate = msg.rollspeed
                    self.state.pitch_rate = msg.pitchspeed
                    self.state.yaw_rate = msg.yawspeed
                    
                elif msg_type == 'VFR_HUD':
                    # HUD data
                    self.state.air_speed = msg.airspeed
                    self.state.ground_speed = msg.groundspeed
                    self.state.heading = msg.heading
                    self.state.altitude = msg.alt
                    
                elif msg_type == 'GPS_RAW_INT':
                    # GPS status
                    self.state.gps_fix_type = msg.fix_type
                    self.state.gps_satellites = msg.satellites_visible
                    
                elif msg_type == 'SCALED_IMU2' or msg_type == 'SCALED_IMU':
                    # IMU data (acceleration)
                    self.state.acceleration_x = msg.xacc / 1000.0  # mG to m/s²
                    self.state.acceleration_y = msg.yacc / 1000.0
                    self.state.acceleration_z = msg.zacc / 1000.0
                    
                elif msg_type == 'SYS_STATUS':
                    # System status including battery
                    self.state.battery_voltage = msg.voltage_battery / 1000.0
                    self.state.battery_current = msg.current_battery / 100.0
                    
                elif msg_type == 'HEARTBEAT':
                    # System heartbeat
                    self.last_heartbeat_time = time.time()
                    self.state.system_status = mavutil.mavlink.enums['MAV_STATE'][
                        msg.system_status
                    ].name.replace('MAV_STATE_', '')
                    
                    # Get flight mode
                    if hasattr(msg, 'custom_mode'):
                        try:
                            mode_enum = self.master.flightmode
                            self.state.flight_mode = str(mode_enum)
                        except:
                            self.state.flight_mode = "UNKNOWN"
                
                # Notify state callbacks every N messages
                if self.message_count % 10 == 0 and self.state_callbacks:
                    for callback in self.state_callbacks:
                        try:
                            callback(self.state)
                        except Exception as e:
                            logger.error(f"Callback error: {e}")
                            
            except Exception as e:
                logger.debug(f"Message processing error: {e}")
    
    def _heartbeat_monitor_loop(self):
        """Monitor connection health (runs in background thread)"""
        logger.info("Heartbeat monitor thread started")
        
        while self.running:
            time.sleep(1)
            
            if not self.connected:
                continue
                
            # Check if heartbeat is stale
            time_since_heartbeat = time.time() - self.last_heartbeat_time
            
            if time_since_heartbeat > 5.0:  # 5 seconds without heartbeat
                logger.warning(f"Heartbeat stale: {time_since_heartbeat:.1f}s")
                self.connected = False
                
                # Try to reconnect
                self.reconnect()
    
    def reconnect(self, max_attempts: int = 3) -> bool:
        """Attempt to reconnect to Pixhawk"""
        logger.info("Attempting to reconnect...")
        
        for attempt in range(max_attempts):
            try:
                self.disconnect()
                time.sleep(1)
                
                if self.connect(timeout_seconds=5):
                    logger.info(f"✅ Reconnected on attempt {attempt + 1}")
                    return True
                    
            except Exception as e:
                logger.warning(f"Reconnect attempt {attempt + 1} failed: {e}")
                time.sleep(2)
        
        logger.error("❌ All reconnect attempts failed")
        return False
    
    def get_state(self) -> VehicleState:
        """
        Get current vehicle state (thread-safe).
        
        Returns:
            Current VehicleState object
        """
        with self.state_lock:
            # Return a copy to avoid race conditions
            return VehicleState(**self.state.__dict__)
    
    def get_position(self) -> Tuple[float, float, float]:
        """Get current position (lat, lon, alt)"""
        state = self.get_state()
        return (state.latitude, state.longitude, state.altitude)
    
    def get_speed(self) -> float:
        """Get current ground speed in m/s"""
        return self.get_state().ground_speed
    
    def get_heading(self) -> float:
        """Get current heading in degrees (0-360, North=0)"""
        return self.get_state().heading
    
    def get_attitude(self) -> Tuple[float, float, float]:
        """Get current attitude (roll, pitch, yaw) in radians"""
        state = self.get_state()
        return (state.roll, state.pitch, state.yaw)
    
    def set_flight_mode(self, mode: FlightMode) -> bool:
        """
        Change Pixhawk flight mode.
        
        Args:
            mode: Desired flight mode
            
        Returns:
            True if command sent successfully
        """
        if not self.connected or not self.master:
            logger.error("Not connected to Pixhawk")
            return False
            
        try:
            # Convert mode to MAVLink custom mode
            mode_mapping = {
                FlightMode.MANUAL: 0,
                FlightMode.STABILIZE: 1,
                FlightMode.AUTO: 3,
                FlightMode.GUIDED: 4,
                FlightMode.HOLD: 5,
                FlightMode.RETURN_TO_LAUNCH: 6,
                FlightMode.BRAKE: 7,
            }
            
            if mode not in mode_mapping:
                logger.error(f"Unsupported mode: {mode}")
                return False
            
            # Send command
            self.master.mav.set_mode_send(
                self.vehicle_id,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_mapping[mode]
            )
            
            logger.info(f"Flight mode set to: {mode.value}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to set flight mode: {e}")
            self._notify_error(e)
            return False
    
    def send_navigation_command(self, 
                                lat: float, 
                                lon: float, 
                                alt: float = 0.0) -> bool:
        """
        Send navigation command to go to specific coordinates.
        
        Args:
            lat: Target latitude (degrees)
            lon: Target longitude (degrees)
            alt: Target altitude (meters)
            
        Returns:
            True if command sent successfully
        """
        if not self.connected or not self.master:
            return False
            
        try:
            # Create mission item (simplified)
            self.master.mav.mission_item_int_send(
                self.vehicle_id,
                self.master.target_component,
                0,  # Sequence
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,  # Current
                1,  # Autocontinue
                0, 0, 0, 0,  # params
                int(lat * 1e7),  # x (latitude)
                int(lon * 1e7),  # y (longitude)
                alt              # z (altitude)
            )
            
            logger.info(f"Navigation command sent: ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
            return True
            
        except Exception as e:
            logger.error(f"Navigation command failed: {e}")
            return False
    
    def add_state_callback(self, callback: Callable[[VehicleState], None]):
        """Add callback for state updates"""
        self.state_callbacks.append(callback)
    
    def add_error_callback(self, callback: Callable[[Exception], None]):
        """Add callback for errors"""
        self.error_callbacks.append(callback)
    
    def _notify_error(self, error: Exception):
        """Notify all error callbacks"""
        for callback in self.error_callbacks:
            try:
                callback(error)
            except Exception as e:
                logger.error(f"Error in error callback: {e}")
    
    def get_statistics(self) -> dict:
        """Get connection statistics"""
        return {
            'connected': self.connected,
            'message_count': self.message_count,
            'connection_attempts': self.connection_attempts,
            'update_rate_hz': self.update_rate,
            'time_since_heartbeat': time.time() - self.last_heartbeat_time,
            'vehicle_id': self.vehicle_id,
        }
    
    def disconnect(self):
        """Clean shutdown"""
        logger.info("Disconnecting from Pixhawk...")
        
        self.running = False
        self.connected = False
        
        # Wait for threads to finish
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=2)
        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=2)
        
        # Close connection
        if self.master:
            try:
                self.master.close()
            except:
                pass
        
        logger.info("Disconnected from Pixhawk")
    
    def __enter__(self):
        """Context manager entry"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()


# ============================================================================
# Usage Example
# ============================================================================

def example_usage():
    """Example of how to use the PixhawkInterface class"""
    
    # Create interface
    pixhawk = PixhawkInterface(
        connection_string='/dev/ttyACM0',
        baudrate=57600,
        update_rate_hz=10.0
    )
    
    # Add state callback
    def on_state_update(state: VehicleState):
        print(f"\r📡 GPS: {state.latitude:.6f}, {state.longitude:.6f} "
              f"| Speed: {state.ground_speed:.1f} m/s "
              f"| Heading: {state.heading:.0f}°", end="")
    
    pixhawk.add_state_callback(on_state_update)
    
    # Add error callback
    def on_error(error: Exception):
        print(f"\n❌ Pixhawk error: {error}")
    
    pixhawk.add_error_callback(on_error)
    
    # Connect
    if not pixhawk.connect():
        print("Failed to connect to Pixhawk")
        return
    
    print("✅ Pixhawk connected. Press Ctrl+C to stop.")
    print("-" * 60)
    
    try:
        # Main loop
        while True:
            time.sleep(1)
            
            # Get current state
            state = pixhawk.get_state()
            
            # Example: Check if we have GPS fix
            if state.gps_fix_type >= 3:
                print(f"\n📍 GPS Fix: {state.gps_satellites} satellites")
            
            # Example: Change mode after 10 seconds
            if time.time() - pixhawk.last_heartbeat_time > 10:
                pixhawk.set_flight_mode(FlightMode.GUIDED)
                
                # Send navigation command
                current_lat, current_lon, _ = pixhawk.get_position()
                target_lat = current_lat + 0.0001  # ~10 meters north
                pixhawk.send_navigation_command(target_lat, current_lon)
                break
            
    except KeyboardInterrupt:
        print("\n\n🛑 Stopped by user")
    finally:
        pixhawk.disconnect()
        print("\n✅ Clean shutdown complete")


if __name__ == "__main__":
    example_usage()