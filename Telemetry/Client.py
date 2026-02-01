#!/usr/bin/env python3
"""
CAR CLIENT - Run on RASPBERRY PI
FIXED: Steering doesn't affect throttle
"""

import socket
import json
import sys
import time

sys.path.append('/home/pi/Desktop/Intelligent-Vehicle-Navigation')

# Hardware
try:
    from Hardware.PCA_Board import PCA9685Controller
    from Hardware.DC_Motor import DCMotor
    print("✅ Hardware loaded")
    HARDWARE = True
except:
    print("⚠️  Hardware not found - simulation mode")
    HARDWARE = False

class FixedCarClient:
    def __init__(self, host='0.0.0.0', port=9999):
        self.host = host
        self.port = port
        
        # INDEPENDENT controls
        self.steering = 90
        self.throttle = 0
        self.target_throttle = 0
        
        # Control settings
        self.MAX_THROTTLE = 100
        self.MIN_THROTTLE = -100
        self.MIN_STEERING = 50
        self.MAX_STEERING = 130
        
        # Hardware
        if HARDWARE:
            self.pca = PCA9685Controller()
            self.motor = DCMotor(4, 17, 27, 22, "FixedCar")
            self.pca.driver.set_angle(90)
            time.sleep(0.5)
            print("✅ Car initialized")
    
    def apply_controls(self):
        """Apply controls INDEPENDENTLY"""
        if not HARDWARE:
            return
            
        # 1. Apply steering IMMEDIATELY (no effect on throttle)
        self.pca.driver.set_angle(self.steering)
        
        # 2. Smooth throttle changes (independent of steering)
        if abs(self.target_throttle - self.throttle) < 0.5:
            self.throttle = self.target_throttle
        elif self.target_throttle > self.throttle:
            self.throttle += 0.5  # Slow smoothing
        else:
            self.throttle -= 0.5
        
        # Clamp throttle
        self.throttle = max(self.MIN_THROTTLE, min(self.MAX_THROTTLE, self.throttle))
        
        # 3. Apply motor control
        throttle_int = int(self.throttle)
        
        if abs(throttle_int) < 3:
            self.motor.stop()
        elif throttle_int > 0:
            self.motor.move_forward(throttle_int)
        else:
            self.motor.move_reverse(-throttle_int)
        
        # Update display
        self.update_display()
    
    def update_display(self):
        """Update status display"""
        # Get icons
        if self.steering < 85:
            steer_icon = "↖️"
            steer_color = "\033[93m"
        elif self.steering > 95:
            steer_icon = "↗️"
            steer_color = "\033[93m"
        else:
            steer_icon = "⬆️"
            steer_color = ""
        
        if self.throttle > 2:
            throttle_icon = "⬆️"
            throttle_color = "\033[92m"
        elif self.throttle < -2:
            throttle_icon = "⬇️"
            throttle_color = "\033[91m"
        else:
            throttle_icon = "⏹️"
            throttle_color = "\033[90m"
        
        # Clear and print - INDEPENDENT values
        print(f"\033[2K\033[1G", end="")
        print(f"🧭 {steer_color}{steer_icon} {self.steering:3d}°\033[0m ", end="")
        print(f"⚡ {throttle_color}{throttle_icon} {self.throttle:+6.1f}%\033[0m", end="")
    
    def handle_command(self, command):
        """Handle commands - STEERING AND THROTTLE ARE SEPARATE"""
        try:
            cmd = json.loads(command)
            cmd_type = cmd.get('type', '')
            value = cmd.get('value', '')
            
            response = {'status': 'ok'}
            
            # STEERING ONLY - doesn't touch throttle
            if cmd_type == 'steer':
                try:
                    if isinstance(value, str) and value == 'center':
                        self.steering = 90
                    else:
                        angle = int(value)
                        self.steering = max(self.MIN_STEERING, min(self.MAX_STEERING, angle))
                    
                    response['message'] = f"Steering: {self.steering}° (Throttle unchanged: {self.throttle:+.1f}%)"
                except:
                    response = {'status': 'error', 'message': 'Invalid steering value'}
            
            # THROTTLE ONLY - doesn't touch steering
            elif cmd_type == 'throttle':
                try:
                    if isinstance(value, str) and value == 'stop':
                        self.target_throttle = 0
                    else:
                        speed = int(value)
                        self.target_throttle = max(self.MIN_THROTTLE, min(self.MAX_THROTTLE, speed))
                    
                    response['message'] = f"Throttle: {self.target_throttle:+3d}% (Steering unchanged: {self.steering}°)"
                except:
                    response = {'status': 'error', 'message': 'Invalid throttle value'}
            
            # EMERGENCY STOP - stops both
            elif cmd_type == 'emergency':
                self.target_throttle = 0
                self.throttle = 0
                self.steering = 90
                response['message'] = "🛑 EMERGENCY - Speed=0%, Steering=90°"
            
            # Apply controls
            self.apply_controls()
            
            return json.dumps(response)
            
        except json.JSONDecodeError:
            return json.dumps({'status': 'error', 'message': 'Invalid JSON'})
    
    def start_server(self):
        """Start server"""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind((self.host, self.port))
            server_socket.listen(1)
            
            # Get IP
            import subprocess
            result = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
            ip = result.stdout.strip().split()[0] if result.stdout else self.host
            
            print(f"\n" + "="*50)
            print(f"FIXED CAR CLIENT - Raspberry Pi")
            print(f"IP: {ip}")
            print(f"Port: {self.port}")
            print("FIX: Steering and throttle are INDEPENDENT")
            print("="*50)
            print("Waiting for connection...")
            print("="*50 + "\n")
            
            # Initial display
            self.update_display()
            
            while True:
                client_socket, address = server_socket.accept()
                print(f"\n🔗 Connected to {address}")
                
                try:
                    while True:
                        data = client_socket.recv(1024).decode('utf-8')
                        if not data:
                            break
                        
                        response = self.handle_command(data)
                        client_socket.send(response.encode('utf-8'))
                        
                except ConnectionResetError:
                    print(f"\n🔌 {address} disconnected")
                finally:
                    client_socket.close()
                    # Stop car
                    if HARDWARE:
                        self.motor.stop()
                        self.pca.driver.set_angle(90)
                    print("\n✅ Car stopped. Waiting for connection...")
                    self.update_display()

if __name__ == "__main__":
    client = FixedCarClient(host='0.0.0.0', port=9999)
    
    try:
        client.start_server()
    except KeyboardInterrupt:
        print("\n🛑 Stopped")
        if HARDWARE:
            client.motor.stop()
            client.pca.driver.set_angle(90)
    except Exception as e:
        print(f"❌ Error: {e}")