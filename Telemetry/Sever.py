#!/usr/bin/env python3
"""
CAR CONTROL SERVER - WINDOWS VERSION
Steering and throttle are INDEPENDENT
No special Linux keyboard modules needed
"""

import socket
import json
import time
import sys
import msvcrt  # Windows keyboard library

class CarServer:
    def __init__(self, pi_ip='192.168.1.100', port=9999):
        self.pi_ip = pi_ip
        self.port = port
        self.socket = None
        self.connected = False
        
        # Current state
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
            
            # Initialize car
            self.send_command('steer', 90)
            self.send_command('throttle', 0)
            time.sleep(0.5)
            
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
    
    def windows_control(self):
        """Windows keyboard control (no Linux dependencies)"""
        print("\n" + "="*50)
        print("CAR CONTROL - WINDOWS VERSION")
        print("="*50)
        print("Controls:")
        print("  W - Increase speed (+2%)")
        print("  S - Decrease speed (-2%)")
        print("  A - Turn Left (2°)")
        print("  D - Turn Right (2°)")
        print("  SPACE - Stop (0% speed)")
        print("  C - Center steering (90°)")
        print("  E - Set 20% speed")
        print("  R - Reset to 0% speed")
        print("  X - Emergency stop")
        print("  Q - Quit")
        print("="*50)
        print("Steering and speed are INDEPENDENT")
        print("Press keys (no Enter needed)")
        print("="*50)
        
        print(f"\n🚗 Ready! Current: Speed={self.current_speed:+3d}%, Steering={self.current_steering}°")
        print("Tap keys to control...")
        
        last_key_time = 0
        key_repeat_delay = 0.1  # 100ms between repeated key presses
        
        try:
            while True:
                # Check if key is pressed (Windows specific)
                if msvcrt.kbhit():
                    key = msvcrt.getch().decode('utf-8').lower()
                    current_time = time.time()
                    
                    # Prevent too fast key repeats
                    if current_time - last_key_time < key_repeat_delay:
                        time.sleep(0.01)
                        continue
                    
                    last_key_time = current_time
                    
                    # SPEED CONTROL
                    if key == 'w':  # Increase speed
                        self.current_speed = min(100, self.current_speed + 2)
                        response = self.send_command('throttle', self.current_speed)
                        print(f"\n⬆️  Speed: {self.current_speed:+3d}%")
                        
                    elif key == 's':  # Decrease speed
                        self.current_speed = max(-100, self.current_speed - 2)
                        response = self.send_command('throttle', self.current_speed)
                        print(f"\n⬇️  Speed: {self.current_speed:+3d}%")
                    
                    # STEERING CONTROL (independent)
                    elif key == 'a':  # Turn left
                        self.current_steering = max(50, self.current_steering - 2)
                        response = self.send_command('steer', self.current_steering)
                        print(f"\n↖️  Steering: {self.current_steering:3d}°")
                        
                    elif key == 'd':  # Turn right
                        self.current_steering = min(130, self.current_steering + 2)
                        response = self.send_command('steer', self.current_steering)
                        print(f"\n↗️  Steering: {self.current_steering:3d}°")
                    
                    # OTHER CONTROLS
                    elif key == ' ':  # Space bar - Stop
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
                        time.sleep(0.1)
                        print(f"\n🔄 Reset: Speed=0%, Steering=90°")
                        
                    elif key == 'x':  # Emergency stop
                        self.current_speed = 0
                        self.current_steering = 90
                        response = self.send_command('emergency', 'stop')
                        print(f"\n🛑 EMERGENCY STOP - Speed=0%, Steering=90°")
                        
                    elif key == 'q':  # Quit
                        print("\n🛑 Quitting...")
                        break
                    
                    # Show response
                    try:
                        resp_data = json.loads(response)
                        if 'message' in resp_data:
                            print(f"   Pi: {resp_data['message']}")
                    except:
                        pass
                
                # Small sleep to prevent CPU overload
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n🛑 Interrupted")
        finally:
            self.close()
    
    def close(self):
        """Close connection"""
        if self.socket:
            self.socket.close()
            print("🔌 Connection closed")

def get_pi_ip():
    """Ask user for Pi IP"""
    print("\n" + "="*50)
    print("CAR CONTROL SERVER SETUP")
    print("="*50)
    
    # Try to auto-detect common Pi IPs
    common_ips = ['192.168.1.100', '192.168.1.101', '192.168.0.100', '192.168.0.101']
    
    print("Common Raspberry Pi IP addresses:")
    for i, ip in enumerate(common_ips, 1):
        print(f"  {i}. {ip}")
    print(f"  {len(common_ips)+1}. Enter custom IP")
    
    choice = input(f"\nSelect IP or enter custom: ").strip()
    
    if choice.isdigit() and 1 <= int(choice) <= len(common_ips):
        return common_ips[int(choice) - 1]
    elif choice.isdigit() and int(choice) == len(common_ips) + 1:
        custom_ip = input("Enter Pi's IP address: ").strip()
        return custom_ip
    else:
        # Assume it's a custom IP
        return choice if '.' in choice else '192.168.1.100'

if __name__ == "__main__":
    # Get Pi IP
    PI_IP = get_pi_ip()
    
    print(f"\n🔗 Connecting to {PI_IP}:9999")
    print("Make sure Raspberry Pi is running car_client.py")
    print("-" * 50)
    
    server = CarServer(pi_ip=PI_IP, port=9999)
    
    if server.connect():
        server.windows_control()
    else:
        print(f"\n❌ Couldn't connect to {PI_IP}:9999")
        print("\nTroubleshooting:")
        print("1. Check Pi is on same network")
        print("2. Run car_client.py on Pi")
        print("3. Check Windows Firewall allows connection")
        print("4. Verify IP address is correct")
        print("\nPress Enter to exit...")
        input()