#!/usr/bin/env python3
"""
CAR SERVER - Windows/PC
Displays live logs with error highlighting.
"""

import socket
import threading
import sys
import os
import time
from datetime import datetime
from collections import deque

# ANSI color codes
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    END = '\033[0m'
    CLEAR = '\033[2J\033[H'  # Clear screen and home cursor

LOG_FILE = "car_log.txt"
open(LOG_FILE, 'w').close()

# Store recent logs
log_buffer = deque(maxlen=20)
error_buffer = deque(maxlen=5)  # last 5 errors

def listen_for_logs(conn, stop_event):
    """Receive logs and update buffers."""
    while not stop_event.is_set():
        try:
            conn.settimeout(0.5)
            data = conn.recv(1024).decode().strip()
            if data:
                timestamp = datetime.now().strftime('%H:%M:%S')
                line = f"[{timestamp}] {data}"
                log_buffer.append(line)
                # If it's an error, also add to error buffer
                if "ERROR" in data.upper():
                    error_buffer.append(line)
                with open(LOG_FILE, 'a') as f:
                    f.write(line + "\n")
        except socket.timeout:
            continue
        except:
            break

def clear_screen():
    os.system('cls' if sys.platform == 'win32' else 'clear')

def print_header(connected=True, last_cmd="None"):
    print(f"{Colors.BOLD}{Colors.BLUE}{'='*60}{Colors.END}")
    print(f"{Colors.BOLD}{Colors.GREEN}🚗 AUTONOMOUS CAR CONTROLLER{Colors.END}")
    status = f"{Colors.GREEN}CONNECTED{Colors.END}" if connected else f"{Colors.RED}DISCONNECTED{Colors.END}"
    print(f"Status: {status}  |  Last Command: {last_cmd}")
    print(f"{Colors.BOLD}{Colors.BLUE}{'='*60}{Colors.END}")

def print_logs():
    """Display the last few log lines."""
    print(f"\n{Colors.BOLD}{Colors.YELLOW}📋 Recent Logs:{Colors.END}")
    if not log_buffer:
        print("   No logs yet.")
    else:
        for line in list(log_buffer)[-8:]:  # Show last 8
            # Color-code errors
            if "ERROR" in line.upper():
                print(f"   {Colors.RED}{line}{Colors.END}")
            elif "start" in line.lower() or "resume" in line.lower():
                print(f"   {Colors.GREEN}{line}{Colors.END}")
            elif "stop" in line.lower() or "pause" in line.lower():
                print(f"   {Colors.YELLOW}{line}{Colors.END}")
            else:
                print(f"   {Colors.END}{line}")

def print_errors():
    """Display recent errors."""
    print(f"\n{Colors.BOLD}{Colors.RED}⚠️  Recent Errors:{Colors.END}")
    if not error_buffer:
        print("   No errors.")
    else:
        for line in error_buffer:
            print(f"   {Colors.RED}{line}{Colors.END}")

def print_menu():
    print(f"\n{Colors.BOLD}{Colors.BLUE}Commands:{Colors.END}")
    print(f"  {Colors.GREEN}[1] START{Colors.END} - Begin driving")
    print(f"  {Colors.RED}[2] STOP{Colors.END}  - Pause driving")
    print(f"  {Colors.YELLOW}[3] EXIT{Colors.END}  - Shutdown")
    print(f"{Colors.BLUE}{'-'*60}{Colors.END}")

def start_server(host='0.0.0.0', port=65432):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)
    
    clear_screen()
    print_header(connected=False)
    print(f"📡 Server listening on port {port}")
    print("⏳ Waiting for Raspberry Pi to connect...\n")
    
    conn, addr = server.accept()
    clear_screen()
    last_cmd = "None"
    print_header(connected=True, last_cmd=last_cmd)
    print(f"✅ Connected to {addr}\n")
    
    stop_event = threading.Event()
    threading.Thread(target=listen_for_logs, args=(conn, stop_event), daemon=True).start()
    
    try:
        while True:
            print_logs()
            print_errors()
            print_menu()
            choice = input(f"{Colors.BOLD}Enter choice (1-3): {Colors.END}").strip()
            
            if choice == "1":
                conn.sendall("start".encode())
                last_cmd = "START"
                log_buffer.append(f"[{datetime.now().strftime('%H:%M:%S')}] >>> Sent START")
                clear_screen()
                print_header(connected=True, last_cmd=last_cmd)
                print(f"✅ Connected to {addr}\n")
            elif choice == "2":
                conn.sendall("stop".encode())
                last_cmd = "STOP"
                log_buffer.append(f"[{datetime.now().strftime('%H:%M:%S')}] >>> Sent STOP")
                clear_screen()
                print_header(connected=True, last_cmd=last_cmd)
                print(f"✅ Connected to {addr}\n")
            elif choice == "3":
                conn.sendall("exit".encode())
                print(f"\n{Colors.YELLOW}👋 Shutting down...{Colors.END}")
                break
            else:
                print(f"{Colors.RED}❌ Invalid choice. Press Enter to continue...{Colors.END}")
                input()
                clear_screen()
                print_header(connected=True, last_cmd=last_cmd)
                print(f"✅ Connected to {addr}\n")
    except KeyboardInterrupt:
        conn.sendall("exit".encode())
    finally:
        stop_event.set()
        conn.close()
        server.close()
        clear_screen()
        print(f"{Colors.GREEN}✅ Server shutdown complete{Colors.END}\n")

if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 65432
    start_server(port=port)