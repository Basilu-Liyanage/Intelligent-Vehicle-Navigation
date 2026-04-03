import socket
import threading
import time
import sys
import os
import json
from datetime import datetime

sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

from AI.Vehicle_Brain_Module import Vehicle_Brain_Module

driver = Vehicle_Brain_Module()
SERVER_IP = "192.168.1.4"   # 🔁 CHANGE to your laptop IP
PORT = 5000
CLIENT_LOG_FILE = "client_log.txt"  # Path to client log file

class VehicleClient:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.driver = driver
        self.running = False
        self.drive_thread = None
        self.log_streaming = False
        self.last_file_position = 0
        self.log_monitor_thread = None
        self.socket = None
        self.connected = False

    def start_drive(self):
        if not self.running:
            print("🚀 START command received")
            self.running = True
            self.vehicle.running = True

            self.drive_thread = threading.Thread(target=self.driver.drive)
            self.drive_thread.daemon = True
            self.drive_thread.start()
            
            # Start monitoring client log file
            self.start_log_streaming()

    def stop_drive(self):
        print("🛑 STOP command received")
        self.running = False
        self.vehicle.running = False
        self.vehicle.stop()
        
        # Stop log streaming
        self.stop_log_streaming()

    def start_log_streaming(self):
        """Start monitoring and streaming client log file"""
        if not self.log_streaming:
            self.log_streaming = True
            self.last_file_position = 0
            self.log_monitor_thread = threading.Thread(target=self.monitor_and_stream_logs)
            self.log_monitor_thread.daemon = True
            self.log_monitor_thread.start()
            print("📁 Started client log streaming")

    def stop_log_streaming(self):
        """Stop log streaming"""
        self.log_streaming = False
        print("📁 Stopped client log streaming")

    def monitor_and_stream_logs(self):
        """Monitor client log file and send new entries to server"""
        # Create log file if it doesn't exist
        if not os.path.exists(CLIENT_LOG_FILE):
            with open(CLIENT_LOG_FILE, 'w') as f:
                f.write(f"[{datetime.now()}] Client log file created\n")

        while self.log_streaming and self.running and self.connected:
            try:
                if not self.socket:
                    time.sleep(1)
                    continue
                    
                with open(CLIENT_LOG_FILE, 'r') as f:
                    # Seek to last read position
                    f.seek(self.last_file_position)
                    
                    # Read new lines
                    new_lines = f.readlines()
                    
                    if new_lines:
                        self.last_file_position = f.tell()
                        
                        # Send new log entries to server
                        for line in new_lines:
                            if line.strip():  # Only send non-empty lines
                                log_data = {
                                    'type': 'client_log',
                                    'timestamp': datetime.now().isoformat(),
                                    'content': line.strip()
                                }
                                message = json.dumps(log_data)
                                try:
                                    self.socket.sendall(message.encode())
                                    print(f"📤 Sent log: {line.strip()[:50]}...")
                                except:
                                    print("❌ Failed to send log, connection lost")
                                    return
                    
                    time.sleep(1)  # Check for new logs every second
                    
            except Exception as e:
                print(f"❌ Error monitoring client log: {e}")
                time.sleep(2)

    def handle_server_communication(self):
        """Handle all server communication in one thread"""
        while self.connected:
            try:
                # Receive commands from server
                data = self.socket.recv(1024).decode().strip()
                
                if not data:
                    print("📭 Server disconnected")
                    break
                
                # Check if it's a command or log response
                try:
                    json_data = json.loads(data)
                    if json_data.get('type') == 'server_log_response':
                        print(f"📝 Server log updated: {json_data.get('message', '')}")
                    elif json_data.get('type') == 'log_ack':
                        print(f"✅ Server acknowledged log: {json_data.get('log_id', '')[:20]}...")
                except json.JSONDecodeError:
                    # Handle plain text commands
                    if data == "START":
                        self.start_drive()
                    elif data == "STOP":
                        self.stop_drive()
                    else:
                        print(f"📩 Unknown command: {data}")
                        
            except socket.timeout:
                continue
            except Exception as e:
                print(f"❌ Communication error: {e}")
                break
        
        self.connected = False

    def connect(self):
        while True:
            try:
                print("🔌 Connecting to server...")
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(5)  # Add timeout for recv
                self.socket.connect((SERVER_IP, PORT))
                self.connected = True
                print("✅ Connected to server")

                # Send client identification
                client_info = {
                    'type': 'client_info',
                    'client_id': 'vehicle_pi',
                    'timestamp': datetime.now().isoformat()
                }
                self.socket.sendall(json.dumps(client_info).encode())

                # Start communication handler
                comm_thread = threading.Thread(target=self.handle_server_communication)
                comm_thread.daemon = True
                comm_thread.start()
                
                # Keep the main connection alive
                comm_thread.join()
                
                print("🔌 Disconnected from server")

            except Exception as e:
                print(f"❌ Connection error: {e}")
                self.connected = False
                if self.socket:
                    try:
                        self.socket.close()
                    except:
                        pass
                    self.socket = None
                print("🔄 Reconnecting in 3s...")
                time.sleep(3)

if __name__ == "__main__":
    client = VehicleClient(driver)
    client.connect()