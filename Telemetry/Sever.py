import socket
import threading
import json
import os
from datetime import datetime

HOST = "0.0.0.0"
PORT = 5000
SERVER_LOG_FILE = "server_log.txt"

class VehicleServer:
    def __init__(self):
        self.connections = []
        self.log_lock = threading.Lock()
        self.setup_server_log()
        
    def setup_server_log(self):
        """Create server log file if it doesn't exist"""
        if not os.path.exists(SERVER_LOG_FILE):
            with open(SERVER_LOG_FILE, 'w') as f:
                f.write(f"[{datetime.now()}] Server log file created\n")
    
    def update_server_log(self, log_entry, log_type="INFO"):
        """Update server log file with new entry"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        formatted_entry = f"[{timestamp}] [{log_type}] {log_entry}\n"
        
        with self.log_lock:
            try:
                with open(SERVER_LOG_FILE, 'a') as f:
                    f.write(formatted_entry)
                return True
            except Exception as e:
                print(f"❌ Error writing to server log: {e}")
                return False
    
    def handle_client_log(self, log_data, conn):
        """Process client log and update server log"""
        try:
            content = log_data.get('content', '')
            client_timestamp = log_data.get('timestamp', '')
            
            # Update server log with client data
            log_entry = f"CLIENT LOG [{client_timestamp}]: {content}"
            success = self.update_server_log(log_entry, "CLIENT")
            
            if success:
                # Send acknowledgment back to client
                response = {
                    'type': 'log_ack',
                    'log_id': datetime.now().isoformat(),
                    'message': 'Log received and saved'
                }
                conn.sendall(json.dumps(response).encode())
                print(f"📥 Received and logged client data: {content[:50]}...")
            else:
                print(f"❌ Failed to log client data")
                
        except Exception as e:
            print(f"❌ Error processing client log: {e}")
    
    def send_command_to_client(self, conn, command):
        """Send command to client"""
        try:
            conn.sendall(command.encode())
            self.update_server_log(f"Sent command: {command}", "COMMAND")
            print(f"📤 Sent: {command}")
            return True
        except Exception as e:
            print(f"❌ Error sending command: {e}")
            return False
    
    def send_log_to_client(self, conn, log_entry):
        """Send server log entry to client"""
        try:
            log_data = {
                'type': 'server_log_response',
                'timestamp': datetime.now().isoformat(),
                'message': log_entry
            }
            conn.sendall(json.dumps(log_data).encode())
        except Exception as e:
            print(f"❌ Error sending server log to client: {e}")
    
    def handle_client(self, conn, addr):
        """Handle individual client connection"""
        print(f"✅ Connected to {addr}")
        self.update_server_log(f"Client connected from {addr}", "CONNECTION")
        
        try:
            # Receive initial client info
            initial_data = conn.recv(1024).decode().strip()
            if initial_data:
                try:
                    client_info = json.loads(initial_data)
                    self.update_server_log(f"Client info: {client_info}", "INFO")
                except:
                    pass
            
            # Main communication loop
            while True:
                try:
                    data = conn.recv(4096).decode().strip()
                    
                    if not data:
                        break
                    
                    # Try to parse as JSON
                    try:
                        json_data = json.loads(data)
                        
                        if json_data.get('type') == 'client_log':
                            # Handle client log data
                            self.handle_client_log(json_data, conn)
                        else:
                            print(f"📩 Received unknown JSON: {json_data}")
                            
                    except json.JSONDecodeError:
                        # Handle non-JSON data (for backward compatibility)
                        print(f"📩 Received non-JSON data: {data}")
                        
                except Exception as e:
                    print(f"❌ Error receiving data: {e}")
                    break
                    
        except Exception as e:
            print(f"❌ Client handler error: {e}")
        finally:
            self.update_server_log(f"Client disconnected from {addr}", "DISCONNECTION")
            conn.close()
            print(f"🔌 Disconnected from {addr}")
    
    def command_input_handler(self):
        """Handle server console input for sending commands"""
        while True:
            cmd = input("\nEnter command (START/STOP): ").strip().upper()
            
            if cmd in ["START", "STOP"]:
                # Send command to all connected clients
                for conn in self.connections[:]:  # Iterate over copy
                    if self.send_command_to_client(conn, cmd):
                        # Log the command in server log
                        self.update_server_log(f"Command executed: {cmd}", "COMMAND")
                    else:
                        # Remove dead connection
                        if conn in self.connections:
                            self.connections.remove(conn)
            elif cmd == "VIEW_LOG":
                # Display recent server log entries
                self.display_recent_logs()
            elif cmd == "CLEAR_LOG":
                # Clear server log (with confirmation)
                confirm = input("Are you sure you want to clear the server log? (yes/no): ").strip().lower()
                if confirm == 'yes':
                    self.clear_server_log()
            else:
                print("❌ Invalid command. Use START, STOP, VIEW_LOG, or CLEAR_LOG")
    
    def display_recent_logs(self, lines=20):
        """Display recent server log entries"""
        try:
            with open(SERVER_LOG_FILE, 'r') as f:
                all_lines = f.readlines()
                recent_lines = all_lines[-lines:]
                
                print("\n" + "="*60)
                print(f"📋 RECENT SERVER LOGS (last {len(recent_lines)} entries)")
                print("="*60)
                for line in recent_lines:
                    print(line.strip())
                print("="*60 + "\n")
        except Exception as e:
            print(f"❌ Error reading server log: {e}")
    
    def clear_server_log(self):
        """Clear the server log file"""
        try:
            with open(SERVER_LOG_FILE, 'w') as f:
                f.write(f"[{datetime.now()}] Server log cleared by admin\n")
            print("✅ Server log cleared successfully")
            self.update_server_log("Server log cleared by admin", "ADMIN")
        except Exception as e:
            print(f"❌ Error clearing server log: {e}")
    
    def start(self):
        """Start the server"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen(5)
        
        print(f"🟢 Server started on {HOST}:{PORT}")
        print("Waiting for Pi connections...")
        print("\nAvailable commands:")
        print("  START  - Send START command to connected clients")
        print("  STOP   - Send STOP command to connected clients")
        print("  VIEW_LOG - Display recent server log entries")
        print("  CLEAR_LOG - Clear the server log file")
        
        self.update_server_log(f"Server started on {HOST}:{PORT}", "SYSTEM")
        
        # Start command input handler in separate thread
        command_thread = threading.Thread(target=self.command_input_handler)
        command_thread.daemon = True
        command_thread.start()
        
        # Main connection acceptance loop
        while True:
            try:
                conn, addr = server.accept()
                self.connections.append(conn)
                
                # Start client handler thread
                client_thread = threading.Thread(target=self.handle_client, args=(conn, addr))
                client_thread.daemon = True
                client_thread.start()
                
            except Exception as e:
                print(f"❌ Server error: {e}")
                self.update_server_log(f"Server error: {e}", "ERROR")

if __name__ == "__main__":
    server = VehicleServer()
    server.start()