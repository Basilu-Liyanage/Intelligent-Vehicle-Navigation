# Vehicle/perception/sensor_fusion/tf_luna_integrated.py
"""Integrated TF-Luna using your working code."""
import time
import serial

class TFLuna:
    """Your working TF-Luna implementation."""
    def __init__(self, port="/dev/serial0"):
        self.BAUD_RATE = 115200
        self.SERIAL_TIMEOUT = 1
        
        # Add rate limiting
        self.last_read_time = 0
        self.min_read_interval = 0.1  # Minimum 100ms between reads
        self.last_valid_distance = 0.0
        
        self.lidar_port = serial.Serial(
            port, self.BAUD_RATE, timeout=self.SERIAL_TIMEOUT
        )
        
        if not self.lidar_port.is_open:
            self.lidar_port.open()
            
        print(f"✓ TF-Luna connected on {port}")

    def connect(self):
        """Alias for compatibility with other code."""
        return True

    def get_distance_to_obstacle(self):
        """Reads distance from LiDAR with rate limiting"""
        current_time = time.time()
        
        # Rate limiting: Don't read too frequently
        if current_time - self.last_read_time < self.min_read_interval:
            return self.last_valid_distance if self.last_valid_distance > 0 else 0.3
        
        self.last_read_time = current_time
        
        start_time = time.time()
        timeout = 0.2  # Increased timeout
        
        while time.time() - start_time < timeout:
            count = self.lidar_port.in_waiting
            if count > 8:
                bytes_data = self.lidar_port.read(9)
                self.lidar_port.reset_input_buffer()
                
                if bytes_data[0] == 0x59 and bytes_data[1] == 0x59:
                    distance = bytes_data[2] + bytes_data[3] * 256
                    strength = bytes_data[4] + bytes_data[5] * 256
                    
                    if strength < 100:
                        self.last_valid_distance = 8.0
                        return 8.0
                    
                    distance_m = distance / 100.0
                    self.last_valid_distance = distance_m
                    return distance_m 
            
            time.sleep(0.1)
        
        # Timeout - return last valid reading
        return self.last_valid_distance if self.last_valid_distance > 0 else 0.0
    
    def read_distance(self):
        """Alias for compatibility - uses your working method."""
        return self.get_distance_to_obstacle()

    def close(self):
        """Closes the LIDAR sensor port."""
        if hasattr(self, 'lidar_port') and self.lidar_port.is_open:
            self.lidar_port.close()
            print("✓ TF-Luna disconnected")
if __name__ == "__main__":
    # Simple test routine
    lidar = TFLuna(port="/dev/serial0")
    
    try:
        while True:
            distance = lidar.get_distance_to_obstacle()
            print(f"Distance to obstacle: {distance:.2f} m")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        lidar.close()