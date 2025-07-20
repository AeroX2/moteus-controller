#!/usr/bin/env python3
"""
CAN Monitor and Command Script with Rich TUI
Connects to COM3 at 115200 baud to monitor CAN FD messages and send commands
"""

import serial
import threading
import time
import struct
import sys
import select
from collections import deque
from datetime import datetime

from rich.console import Console
from rich.live import Live
from rich.panel import Panel
from rich.text import Text
from rich.layout import Layout
from rich import box

# Windows-compatible getch implementation
def getch():
    try:
        import msvcrt
        if msvcrt.kbhit():
            char = msvcrt.getch()
            if char == b'\r':  # Enter key
                return '\r'
            elif char == b'\x08':  # Backspace
                return '\b'
            elif char == b'\x03':  # Ctrl+C
                return '\x03'
            elif char == b'\t':  # Tab key
                return '\t'
            elif len(char) == 1 and 32 <= ord(char) <= 126:  # Printable ASCII
                return char.decode('utf-8')
    except ImportError:
        # Unix/Linux version
        import termios, tty
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            char = sys.stdin.read(1)
            if char == '\n':
                return '\r'
            elif char == '\x7f':  # Backspace
                return '\b'
            elif char == '\x03':  # Ctrl+C
                return '\x03'
            elif char == '\t':  # Tab key
                return '\t'
            return char
    return None

class CANMonitor:
    def __init__(self, port='COM3', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        self.devices = {}  # {id: {'angle': float, 'velocity': float, 'status': dict, 'last_seen': timestamp}}
        self.unique_ids = set()
        self.messages = deque(maxlen=100)  # Store last 100 messages
        self.console = Console()
        self.command_history = deque(maxlen=20)
        self.current_command = ""
        self.selected_device_index = 0
        self.pending_status_requests = set()  # Track devices we've sent status commands to
        self.message_count = 0  # Counter for real-time messages
        self.last_message_time = time.time()  # Track message timing
        
    def connect(self):
        """Connect to the serial port"""
        try:
            # Optimize for real-time performance
            self.ser = serial.Serial(
                self.port, 
                self.baudrate, 
                timeout=0.001,  # Much shorter timeout for faster response
                write_timeout=0.1,
                inter_byte_timeout=None,  # Disable inter-byte timeout
                exclusive=True  # Exclusive access for better performance
            )
            # Set buffer sizes for better performance
            self.ser.set_buffer_size(rx_size=8192, tx_size=8192)
            self.add_message(f"[green]Connected to {self.port} at {self.baudrate} baud[/green]")
            return True
        except serial.SerialException as e:
            self.add_message(f"[red]Failed to connect: {e}[/red]")
            return False
    
    def disconnect(self):
        """Disconnect from serial port"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.add_message("[yellow]Disconnected[/yellow]")
    
    def add_message(self, message):
        """Add a timestamped message to the display"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.messages.append(f"[dim]{timestamp}[/dim] {message}")
    
    def parse_message(self, line):
        """Parse CAN message line like: rcv 56004D 99C4734000000000 E B F r f-1"""
        parts = line.strip().split()
        if len(parts) >= 3 and parts[0] == 'rcv':
            try:
                # Extract ID and data
                can_id = parts[1]
                data_hex = parts[2]
                
                # Convert hex data to bytes
                data_bytes = bytes.fromhex(data_hex)
                
                # Check message type using data[0]
                if len(data_bytes) >= 1:
                    message_type = data_bytes[0]
                    
                    if message_type == 0x01:
                        # Status message
                        if len(data_bytes) >= 4:
                            status_flags = data_bytes[1]
                            driver_ready = data_bytes[2] == 0x01
                            driver_fault = data_bytes[3] == 0x01
                            
                            # Extract individual status flags
                            lock = (status_flags & 0x01) != 0
                            vcc_uvlo = (status_flags & 0x02) != 0
                            vds_p = (status_flags & 0x04) != 0
                            reset = (status_flags & 0x08) != 0
                            thsd = (status_flags & 0x10) != 0
                            
                            # Create or update device info
                            if can_id not in self.devices:
                                self.devices[can_id] = {
                                    'angle': None,
                                    'velocity': None,
                                    'status': None,
                                    'last_seen': time.time()
                                }
                            
                            # Update status
                            self.devices[can_id]['status'] = {
                                'lock': lock,
                                'vcc_uvlo': vcc_uvlo,
                                'vds_p': vds_p,
                                'reset': reset,
                                'thsd': thsd,
                                'ready': driver_ready,
                                'fault': driver_fault
                            }
                            self.devices[can_id]['last_seen'] = time.time()
                            self.unique_ids.add(can_id)
                            
                            # Clear pending status request if this was in response to our command
                            if can_id in self.pending_status_requests:
                                self.pending_status_requests.remove(can_id)
                                self.add_message(f"[green]Received status response from {can_id}[/green]")
                            
                            return can_id, None, None, 'status'
                            

                            
                    elif message_type == 0x03:
                        # Initialization status message
                        if len(data_bytes) >= 3:
                            error_flags = data_bytes[1]
                            motor_status = data_bytes[2]
                            
                            error_list = []
                            if error_flags & 0x01:
                                error_list.append("CORDIC")
                            if error_flags & 0x02:
                                error_list.append("DRIVER")
                            if error_flags & 0x04:
                                error_list.append("CURRENT")
                            if error_flags & 0x08:
                                error_list.append("MOTOR")
                            if error_flags & 0x10:
                                error_list.append("FOC")
                            
                            # Interpret motor status
                            motor_status_names = {
                                0x00: "UNINITIALIZED",
                                0x01: "INITIALIZING", 
                                0x02: "UNCALIBRATED",
                                0x03: "CALIBRATING",
                                0x04: "READY",
                                0x08: "ERROR",
                                0x0E: "CALIB_FAILED",
                                0x0F: "INIT_FAILED"
                            }
                            motor_status_name = motor_status_names.get(motor_status, f"UNKNOWN({motor_status:02X})")
                            
                            if error_flags == 0 and motor_status == 0x04:
                                self.add_message(f"[green]CAN {can_id}[/green]: All components initialized successfully, motor READY")
                            elif error_flags == 0:
                                self.add_message(f"[yellow]CAN {can_id}[/yellow]: Components OK, motor status: {motor_status_name}")
                            else:
                                self.add_message(f"[yellow]CAN {can_id}[/yellow]: Init errors: {', '.join(error_list)} (0x{error_flags:02X}), motor: {motor_status_name}")
                            
                            return can_id, None, None, 'init_status'
                            
                    elif message_type == 0x04:
                        # Debug message - now supports up to 63 bytes of debug data
                        if len(data_bytes) >= 2:
                            # Extract debug message from bytes 1 onwards (up to 63 bytes)
                            debug_data = data_bytes[1:]
                            debug_msg = debug_data.decode('ascii', errors='ignore').rstrip('\x00')
                            if debug_msg.strip():  # Only show non-empty messages
                                self.add_message(f"[blue]CAN {can_id}[/blue]: DEBUG: {debug_msg}")
                            return can_id, None, None, 'debug'
                    
                    elif message_type == 0x00:
                        # Motion message (angle/velocity)
                        if len(data_bytes) >= 9:
                            # Extract angle (bytes 1-4) and velocity (bytes 5-8) as little-endian floats
                            angle = struct.unpack('<f', data_bytes[1:5])[0]
                            velocity = struct.unpack('<f', data_bytes[5:9])[0]
                            
                            # Create or update device info
                            if can_id not in self.devices:
                                self.devices[can_id] = {
                                    'angle': None,
                                    'velocity': None,
                                    'status': None,
                                    'last_seen': time.time()
                                }
                            
                            # Update motion data
                            self.devices[can_id]['angle'] = angle
                            self.devices[can_id]['velocity'] = velocity
                            self.devices[can_id]['last_seen'] = time.time()
                            self.unique_ids.add(can_id)
                            
                            return can_id, angle, velocity, 'motion'
                    
            except (ValueError, struct.error) as e:
                self.add_message(f"[red]Error parsing message: {e}[/red]")
        
        return None, None, None, None
    
    def monitor_serial(self):
        """Monitor serial port for incoming data"""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore')
                    if line.strip():
                        can_id, angle, velocity, msg_type = self.parse_message(line)
                        if can_id:
                            # Update message counter and timing
                            self.message_count += 1
                            current_time = time.time()
                            time_since_last = current_time - self.last_message_time
                            self.last_message_time = current_time
                            
                            if msg_type == 'motion':
                                # Motion messages are handled silently for performance
                                # but we could add a counter for debugging
                                pass
                            elif msg_type == 'status':
                                status_info = self.devices[can_id]['status']
                                self.add_message(f"[cyan]CAN {can_id}[/cyan]: Status: Lock={status_info['lock']}, VCC={status_info['vcc_uvlo']}, VDS={status_info['vds_p']}, Reset={status_info['reset']}, THSD={status_info['thsd']}, Ready={status_info['ready']}, Fault={status_info['fault']}")
                            elif msg_type == 'init_status':
                                # This is already handled in parse_message
                                pass
                            elif msg_type == 'debug':
                                # Debug messages are handled silently for performance
                                pass
                            else:
                                # Show other serial output
                                self.add_message(f"[white]Serial:[/white] {line.strip()}")
                
                # Reduced sleep time for faster response
                time.sleep(0.001)  # 1ms instead of 10ms
                
            except Exception as e:
                self.add_message(f"[red]Error in monitor loop: {e}[/red]")
                break
    
    def send_command(self, target_id, command):
        """Send ASCII command to specified CAN ID"""
        if not self.ser or not self.ser.is_open:
            self.add_message("[red]Not connected to serial port[/red]")
            return False
        
        try:
            # Convert ASCII command to hex
            hex_data = command.encode('utf-8').hex().upper() + '00'
            
            # Format as CAN command: "can std id hex f"
            can_command = f"can std {target_id} {hex_data} f\n"
            
            self.ser.write(can_command.encode())
            self.add_message(f"[green]Sent to {target_id}:[/green] '{command}' [dim](hex: {hex_data})[/dim]")
            
            # Track status requests
            if command.strip().upper() == 'T':
                self.pending_status_requests.add(target_id)
                self.add_message(f"[yellow]Waiting for status response from {target_id}[/yellow]")
            
            return True
            
        except Exception as e:
            self.add_message(f"[red]Error sending command: {e}[/red]")
            return False
    
    def create_layout(self):
        """Create the Rich layout optimized for small terminals"""
        layout = Layout()
        
        # Split vertically: messages, devices, command
        layout.split(
            Layout(name="messages", minimum_size=10),
            Layout(name="devices", size=8),  
            Layout(name="command", size=4)
        )
        
        return layout
    
    def update_layout(self, layout):
        """Update the layout with current data"""
        # Messages panel
        messages_text = Text()
        
        # Add real-time statistics
        if self.message_count > 0:
            current_time = time.time()
            time_since_last = current_time - self.last_message_time
            messages_text.append_text(Text.from_markup(f"[dim]Messages: {self.message_count} | Last: {time_since_last:.3f}s ago[/dim]\n"))
        
        for msg in list(self.messages)[-20:]:  # Show last 20 messages for smaller terminals
            messages_text.append_text(Text.from_markup(msg + "\n"))
        
        layout["messages"].update(
            Panel(
                messages_text,
                title=Text.from_markup("[bold cyan]CAN Messages[/bold cyan]"),
                border_style="cyan",
                box=box.SIMPLE,
                padding=(0, 1)
            )
        )
        
        # Devices panel - horizontal layout for small terminals
        devices_text = Text()
        current_time = time.time()
        
        if not self.devices:
            devices_text.append("No devices discovered yet", style="dim")
        else:
            device_parts = []
            device_list = list(self.devices.keys())
            
            # Ensure selected index is valid
            if self.selected_device_index >= len(device_list):
                self.selected_device_index = 0
            
            for i, can_id in enumerate(device_list):
                info = self.devices[can_id]
                age = current_time - info['last_seen']
                status_color = "green" if age < 1.0 else "red"
                status = "●" if age < 1.0 else "○"
                
                # Build device string with both motion and status data
                motion_part = ""
                status_part = ""
                
                # Add motion data if available
                if 'angle' in info and info['angle'] is not None and 'velocity' in info and info['velocity'] is not None:
                    motion_part = f"[white]{info['angle']:+4.1f}°[/white] [white]{info['velocity']:+4.1f}°/s[/white]"
                
                # Add status data if available
                if 'status' in info and info['status'] is not None:
                    status_info = info['status']
                    ready_color = "green" if status_info['ready'] else "red"
                    fault_color = "red" if status_info['fault'] else "green"
                    lock_color = "green" if status_info['lock'] else "red"
                    status_part = f" [white]R:[{ready_color}]{'✓' if status_info['ready'] else '✗'}[/{ready_color}][/white] [white]F:[{fault_color}]{'✗' if status_info['fault'] else '✓'}[/{fault_color}][/white] [white]L:[{lock_color}]{'✓' if status_info['lock'] else '✗'}[/{lock_color}][/white]"
                
                # Combine motion and status parts
                if motion_part and status_part:
                    # Both motion and status available
                    if i == self.selected_device_index:
                        device_str = f"[{status_color}]{status}[/{status_color}] [bold yellow on blue]{can_id}[/bold yellow on blue]: {motion_part}{status_part}"
                    else:
                        device_str = f"[{status_color}]{status}[/{status_color}] [cyan]{can_id}[/cyan]: {motion_part}{status_part}"
                elif motion_part:
                    # Only motion data available
                    if i == self.selected_device_index:
                        device_str = f"[{status_color}]{status}[/{status_color}] [bold yellow on blue]{can_id}[/bold yellow on blue]: {motion_part}"
                    else:
                        device_str = f"[{status_color}]{status}[/{status_color}] [cyan]{can_id}[/cyan]: {motion_part}"
                elif status_part:
                    # Only status data available
                    if i == self.selected_device_index:
                        device_str = f"[{status_color}]{status}[/{status_color}] [bold yellow on blue]{can_id}[/bold yellow on blue]: {status_part}"
                    else:
                        device_str = f"[{status_color}]{status}[/{status_color}] [cyan]{can_id}[/cyan]: {status_part}"
                else:
                    # No data available yet
                    if i == self.selected_device_index:
                        device_str = f"[{status_color}]{status}[/{status_color}] [bold yellow on blue]{can_id}[/bold yellow on blue]: [white]No data[/white]"
                    else:
                        device_str = f"[{status_color}]{status}[/{status_color}] [cyan]{can_id}[/cyan]: [white]No data[/white]"
                
                device_parts.append(device_str)
            
            devices_markup = " | ".join(device_parts)
            devices_text = Text.from_markup(devices_markup)
        
        layout["devices"].update(
            Panel(
                devices_text,
                title=Text.from_markup("[bold magenta]Devices[/bold magenta]"),
                border_style="magenta",
                box=box.SIMPLE,
                padding=(0, 1)
            )
        )
        
        # Command panel
        selected_device = ""
        if self.devices:
            device_list = list(self.devices.keys())
            if self.selected_device_index < len(device_list):
                selected_device = device_list[self.selected_device_index]
        
        cmd_markup = f"[bold]Commands:[/bold] [yellow]<cmd>[/yellow] [dim]|[/dim] [yellow]tab[/yellow] [dim]|[/dim] [yellow]stats[/yellow] [dim]|[/dim] [yellow]quit[/yellow] [dim]| Selected:[/dim] [cyan]{selected_device}[/cyan]\n[bold green]>[/bold green] [white]{self.current_command}[/white][dim]_[/dim]"
        cmd_text = Text.from_markup(cmd_markup)
        
        layout["command"].update(
            Panel(
                cmd_text,
                border_style="yellow",
                box=box.SIMPLE,
                padding=(0, 1)
            )
        )
    
    def process_command(self, command_text):
        """Process user command"""
        if not command_text.strip():
            return
        
        self.command_history.append(command_text)
        cmd = command_text.strip()
        
        if cmd.lower() == 'quit' or cmd.lower() == 'exit':
            self.running = False
        elif cmd.lower() == 'stats':
            # Show real-time statistics
            current_time = time.time()
            time_since_last = current_time - self.last_message_time
            self.add_message(f"[cyan]Real-time Stats:[/cyan] Messages: {self.message_count} | Last message: {time_since_last:.3f}s ago | Devices: {len(self.devices)}")
        elif cmd.lower() == 'list':
            self.add_message("[yellow]Device list updated in sidebar[/yellow]")
        elif cmd.lower() == 'help':
            self.add_message("[yellow]Commands: <command> (send to selected device), tab (cycle devices), stats (show statistics), quit[/yellow]")
        else:
            # Send command to selected device
            if self.devices:
                device_list = list(self.devices.keys())
                if self.selected_device_index < len(device_list):
                    target_id = device_list[self.selected_device_index]
                    self.send_command(target_id, cmd)
                else:
                    self.add_message("[red]No device selected[/red]")
            else:
                self.add_message("[red]No devices available[/red]")
    
    def run_tui(self):
        """Run the terminal user interface"""
        layout = self.create_layout()
        
        with Live(layout, refresh_per_second=10, screen=True, console=self.console) as live:
            while self.running:
                try:
                    # Update display
                    self.update_layout(layout)
                    
                    # Handle keyboard input (non-blocking)
                    char = getch()
                    if char:
                        if char == '\r':  # Enter key
                            if self.current_command.strip():
                                self.process_command(self.current_command)
                                self.current_command = ""
                        elif char == '\b':  # Backspace
                            if self.current_command:
                                self.current_command = self.current_command[:-1]
                        elif char == '\t':  # Tab key - cycle through devices
                            if self.devices:
                                self.selected_device_index = (self.selected_device_index + 1) % len(self.devices)
                        elif char == '\x03':  # Ctrl+C
                            break
                        elif char and 32 <= ord(char) <= 126:  # Printable ASCII
                            self.current_command += char
                    
                    time.sleep(0.05)
                    
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    self.add_message(f"[red]TUI Error: {e}[/red]")

def main():
    monitor = CANMonitor()
    
    if not monitor.connect():
        return
    
    try:
        # Start monitoring in background thread
        monitor.running = True
        monitor_thread = threading.Thread(target=monitor.monitor_serial, daemon=True)
        monitor_thread.start()
        
        monitor.add_message("[green]CAN Monitor started. Type commands below.[/green]")
        monitor.add_message("[yellow]Commands: <command> (sent to selected device), tab (cycle devices), quit[/yellow]")
        
        # Run the TUI
        monitor.run_tui()
        
    except KeyboardInterrupt:
        monitor.add_message("[yellow]Exiting...[/yellow]")
    finally:
        monitor.disconnect()

if __name__ == "__main__":
    # Install required packages if not available
    try:
        import rich
    except ImportError:
        print("Please install required packages:")
        print("pip install rich pyserial")
        exit(1)
    
    main()