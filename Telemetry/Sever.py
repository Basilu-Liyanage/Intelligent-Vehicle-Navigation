#!/usr/bin/env python3
"""
CAR CONTROL SERVER - Run on LAPTOP
FIXED: Steering doesn't change speed
"""

import socket
import json
import time
import sys
import tty
import termios
import select

class CarServer:
    def __init__(self, pi_ip='192.168.1.100', port=9999):
        self.pi_ip = pi_ip
        self.port = port
        self.socket = None
        self.connected = False
        
        # Current state tracking
        self.current_speed = 0
        self.current_steering = 90
        
    def connect(self):
        """Connect to Raspberry Pi"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(2)
            self.socket.connect((self.pi_ip, self.port))
            self.connected = True
            print(f"✅ Connected to {self.pi_ip}:{self.port}")
            
            # Send initial state
            self.send_command('steer', 90)
            self.send_command('throttle', 0)
            
            return True
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def send_command(self, command_type, value):
        """Send command to car"""
        if not self.connected:
            return "Not connected"
            
        try:
            cmd = {'type': command_type, 'value': value}
            self.socket.send(json.dumps(cmd).encode())
            
            # Get response
            response = self.socket.recv(1024).decode()
            return response
            
        except Exception as e:
            self.connected = False
            return f"Error: {e}"
    
    def control_loop(self):
        """Control loop - SEPARATE steering and throttle"""
        print("\n" + "="*50)
        print("CAR CONTROL - WSAD (Fixed)")
        print("="*50)
        print("Controls:")
        print("  W - Increase speed (+2%)")
        print("  S - Decrease speed (-2%)")
        print("  A - Turn Left (2°)")
        print("  D - Turn Right (2°)")
        print("  SPACE - Stop (0% speed)")
        print("  C - Center steering (90°)")
        print("  E - Set 20% speed")
        print("  R - Reset to 0%")
        print("  X - Emergency stop")
        print("  Q - Quit")
        print("="*50)
        print("Important: Steering WON'T change speed!")
        print("="*50)
        
        # Setup keyboard
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            tty.setraw(sys.stdin.fileno())
            
            print(f"\n🚗 Ready! Current: Speed={self.current_speed:+3d}%, Steering={self.current_steering}°")
            print("Tap W/S for speed, A/D for steering (independent)")
            
            while True:
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    key = sys.stdin.read(1).lower()
                    
                    # SPEED CONTROL ONLY
                    if key == 'w':  # Increase speed
                        self.current_speed = min(100, self.current_speed + 2)
                        response = self.send_command('throttle', self.current_speed)
                        print(f"\n⬆️  Speed: {self.current_speed:+3d}%")
                        
                    elif key == 's':  # Decrease speed
                        self.current_speed = max(-100, self.current_speed - 2)
                        response = self.send_command('throttle', self.current_speed)
                        print(f"\n⬇️  Speed: {self.current_speed:+3d}%")
                        
                    # STEERING CONTROL ONLY (doesn't affect speed)
                    elif key == 'a':  # Turn left
                        self.current_steering = max(50, self.current_steering - 2)
                        response = self.send_command('steer', self.current_steering)
                        print(f"\n↖️  Steering: {self.current_steering:3d}°")
                        
                    elif key == 'd':  # Turn right
                        self.current_steering = min(130, self.current_steering + 2)
                        response = self.send_command('steer', self.current_steering)
                        print(f"\n↗️  Steering: {self.current_steering:3d}°")
                    
                    # OTHER CONTROLS
                    elif key == ' ':  # Stop
                        self.current_speed = 0
                        response = self.send_command('throttle', 0)
                        print(f"\n🛑 Stop! Speed: 0%")
                        
                    elif key == 'c':  # Center steering
                        self.current_steering = 90
                        response = self.send_command('steer', 90)
                        print(f"\n🎯 Steering centered: 90°")
                        
                    elif key == 'e':  # Set 20% speed
                        self.current_speed = 20
                        response = self.send_command('throttle', 20)
                        print(f"\n🚗 Speed set: 20%")
                        
                    elif key == 'r':  # Reset
                        self.current_speed = 0
                        self.current_steering = 90
                        self.send_command('throttle', 0)
                        self.send_command('steer', 90)
                        print(f"\n🔄 Reset: Speed=0%, Steering=90°")
                        
                    elif key == 'x':  # Emergency stop
                        self.current_speed = 0
                        self.current_steering = 90
                        response = self.send_command('emergency', 'stop')
                        print(f"\n🛑 EMERGENCY STOP - Speed=0%, Steering=90°")
                        
                    elif key == 'q':  # Quit
                        print("\n🛑 Quitting...")
                        break
                    
                    # Show response from Pi
                    try:
                        resp_data = json.loads(response)
                        if 'message' in resp_data:
                            print(f"   Pi: {resp_data['message']}")
                    except:
                        pass
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n🛑 Interrupted")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            self.close()
    
    def close(self):
        """Close connection"""
        if self.socket:
            self.socket.close()
            print("🔌 Connection closed")

if __name__ == "__main__":
    PI_IP = '192.168.1.7'  # ← CHANGE TO YOUR PI's IP!
    
    server = CarServer(pi_ip=PI_IP, port=9999)
    
    if server.connect():
        server.control_loop()
    else:
        print(f"\n❌ Couldn't connect to {PI_IP}:9999")
        print("Check Pi IP and run car_client.py on Pi")