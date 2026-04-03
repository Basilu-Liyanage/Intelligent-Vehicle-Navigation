import socket
import threading
import json
import os
import time
from datetime import datetime

HOST = "0.0.0.0"
PORT = 5000
SERVER_LOG_FILE = "Server_Log.json"

class VehicleServer:
    def __init__(self):
        self.connections = []
        self.clients = {}  # Store client info
        self.log_lock = threading.Lock()
        self.running = True
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
    
    def handle_client_log(self, log_data, conn, addr):
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
                try:
                    conn.sendall(json.dumps(response).encode())
                    print(f"📥 Received and logged client data: {content[:50]}...")
                except:
                    print(f"❌ Failed to send ACK to client")
            else:
                print(f"❌ Failed to log client data")
                
        except Exception as e:
            print(f"❌ Error processing client log: {e}")
    
    def send_command_to_client(self, conn, command, addr):
        """Send command to client"""
        try:
            conn.sendall(command.encode())
            self.update_server_log(f"Sent command: {command} to {addr}", "COMMAND")
            print(f"📤 Sent {command} to {addr}")
            return True
        except Exception as e:
            print(f"❌ Error sending command to {addr}: {e}")
            return False
    
    def handle_client(self, conn, addr):
        """Handle individual client connection"""
        print(f"✅ Client connected from {addr}")
        self.update_server_log(f"Client connected from {addr}", "CONNECTION")
        
        # Add to clients dict
        self.clients[addr] = {
            'connection': conn,
            'connected_time': datetime.now(),
            'last_activity': datetime.now()
        }
        
        try:
            # Receive initial client info
            conn.settimeout(1)  # Set timeout for recv
            try:
                initial_data = conn.recv(1024).decode().strip()
                if initial_data:
                    try:
                        client_info = json.loads(initial_data)
                        self.clients[addr]['info'] = client_info
                        self.update_server_log(f"Client info from {addr}: {client_info}", "INFO")
                        print(f"📋 Client {addr} identified as: {client_info.get('client_id', 'unknown')}")
                    except:
                        pass
            except socket.timeout:
                pass
            
            # Main communication loop
            while self.running and addr in self.clients:
                try:
                    conn.settimeout(5)
                    data = conn.recv(4096).decode().strip()
                    
                    if not data:
                        print(f"📭 Client {addr} disconnected (no data)")
                        break
                    
                    # Update last activity
                    self.clients[addr]['last_activity'] = datetime.now()
                    
                    # Try to parse as JSON
                    try:
                        json_data = json.loads(data)
                        
                        if json_data.get('type') == 'client_log':
                            # Handle client log data
                            self.handle_client_log(json_data, conn, addr)
                        else:
                            print(f"📩 Received unknown JSON from {addr}: {json_data}")
                            
                    except json.JSONDecodeError:
                        # Handle non-JSON data
                        print(f"📩 Received non-JSON data from {addr}: {data}")
                        
                except socket.timeout:
                    # Timeout is normal, just continue
                    continue
                except Exception as e:
                    print(f"❌ Error receiving data from {addr}: {e}")
                    break
                    
        except Exception as e:
            print(f"❌ Client handler error for {addr}: {e}")
        finally:
            self.disconnect_client(addr)
    
    def disconnect_client(self, addr):
        """Clean up client connection"""
        if addr in self.clients:
            print(f"🔌 Disconnected from {addr}")
            self.update_server_log(f"Client disconnected from {addr}", "DISCONNECTION")
            try:
                self.clients[addr]['connection'].close()
            except:
                pass
            del self.clients[addr]
    
    def command_input_handler(self):
        """Handle server console input for sending commands"""
        print("\n" + "="*60)
        print("🖥️  Server Console Ready")
        print("Available commands:")
        print("  START        - Send START command to all connected clients")
        print("  STOP         - Send STOP command to all connected clients")
        print("  STATUS       - Show connected clients")
        print("  VIEW_LOG     - Display recent server log entries")
        print("  CLEAR_LOG    - Clear the server log file")
        print("  EXIT         - Shutdown server")
        print("="*60 + "\n")
        
        while self.running:
            try:
                cmd = input("> ").strip().upper()
                
                if cmd in ["START", "STOP"]:
                    if not self.clients:
                        print("⚠️  No clients connected")
                        continue
                    
                    # Send command to all connected clients
                    success_count = 0
                    for addr, client_info in list(self.clients.items()):
                        if self.send_command_to_client(client_info['connection'], cmd, addr):
                            success_count += 1
                        else:
                            # Remove dead connection
                            self.disconnect_client(addr)
                    
                    print(f"📤 Command '{cmd}' sent to {success_count}/{len(self.clients)} clients")
                    self.update_server_log(f"Command '{cmd}' sent to {success_count} clients", "COMMAND")
                    
                elif cmd == "STATUS":
                    print(f"\n📊 Connected clients: {len(self.clients)}")
                    for addr, info in self.clients.items():
                        connected_time = datetime.now() - info['connected_time']
                        print(f"  • {addr} - Connected for {connected_time.seconds}s")
                        if 'info' in info:
                            print(f"    ID: {info['info'].get('client_id', 'unknown')}")
                    print()
                    
                elif cmd == "VIEW_LOG":
                    self.display_recent_logs()
                    
                elif cmd == "CLEAR_LOG":
                    confirm = input("Are you sure you want to clear the server log? (yes/no): ").strip().lower()
                    if confirm == 'yes':
                        self.clear_server_log()
                        
                elif cmd == "EXIT":
                    print("🛑 Shutting down server...")
                    self.running = False
                    break
                    
                else:
                    print("❌ Invalid command. Use START, STOP, STATUS, VIEW_LOG, CLEAR_LOG, or EXIT")
                    
            except KeyboardInterrupt:
                print("\n🛑 Server shutdown requested")
                self.running = False
                break
            except Exception as e:
                print(f"❌ Command handler error: {e}")
    
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
        
        print(f"\n🟢 Server started on {HOST}:{PORT}")
        print(f"📝 Log file: {SERVER_LOG_FILE}")
        print("Waiting for Pi connections...\n")
        
        self.update_server_log(f"Server started on {HOST}:{PORT}", "SYSTEM")
        
        # Start command input handler in separate thread
        command_thread = threading.Thread(target=self.command_input_handler)
        command_thread.daemon = True
        command_thread.start()
        
        # Main connection acceptance loop
        while self.running:
            try:
                server.settimeout(1)
                conn, addr = server.accept()
                
                # Start client handler thread
                client_thread = threading.Thread(target=self.handle_client, args=(conn, addr))
                client_thread.daemon = True
                client_thread.start()
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"❌ Server error: {e}")
                    self.update_server_log(f"Server error: {e}", "ERROR")
                time.sleep(1)
        
        # Cleanup
        print("Cleaning up connections...")
        for addr in list(self.clients.keys()):
            self.disconnect_client(addr)
        server.close()
        print("Server shutdown complete")

if __name__ == "__main__":
    server = VehicleServer()
    server.start()