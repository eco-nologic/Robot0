#!/usr/bin/env python3
"""
GyroBot Control Panel - Python BLE GUI

Features:
- Auto-connect to GyroBot_BLE on startup
- Real-time BLE telemetry display (CSV format, 22 fields)
- Robot track visualization (matplotlib)
- Compass heading indicator
- Speed plot (left/right wheels)
- IMU plot (acceleration)
- Sequence control (S1, S2, S3, FLECHE, STOP)
- CSV telemetry logging (in-memory, explicit save)
"""

import asyncio
import csv
import os
import sys
import tkinter as tk
from tkinter import ttk, messagebox
import threading
from datetime import datetime
from collections import deque
import math
from concurrent.futures import TimeoutError

try:
    from bleak import BleakScanner, BleakClient
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
except ImportError as e:
    print(f"Missing dependency: {e}")
    print("Install with: pip install bleak matplotlib")
    sys.exit(1)

# === BLE Configuration (from BleManager.cpp) ===
ROBOT_NAME = "GyroBot_BLE"
SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
CHAR_TX_UUID = "87654321-4321-4321-4321-cba987654321"  # Notify  (telemetry in)
CHAR_RX_UUID = "11111111-2222-3333-4444-555555555555"  # Write   (commands out)

# === CSV Header and Field Names (from Telemetry.cpp) ===
CSV_HEADER = "timestamp,penX,penY,robotHeading,heading,L_steps,R_steps,accX,accY,accZ,gyroX,gyroY,gyroZ,magX,magY,magZ,batt,isMoving,targetL,actualV,targetW,actualW"
CSV_FIELDS = CSV_HEADER.split(',')

# Global state
class AppState:
    def __init__(self):
        self.connected = False
        self.client = None
        self.loop = None
        self.pen_x_history = deque(maxlen=500)
        self.pen_y_history = deque(maxlen=500)
        self.vl_history = deque(maxlen=200)
        self.vr_history = deque(maxlen=200)
        self.heading_history = deque(maxlen=200)
        self.accel_x_history = deque(maxlen=200)
        self.current_telemetry = {}
        self.telemetry_buffer = []
        self.logging = True  # Start logging on connect
        self.scanning = False

app_state = AppState()

class GyroBotGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("GyroBot Control Panel")
        self.geometry("1400x900")
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.script_dir = os.path.dirname(os.path.abspath(__file__))

        # Main frame
        main_frame = ttk.Frame(self)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Top control bar
        self.create_control_bar(main_frame)

        # Content frame (left + right)
        content_frame = ttk.Frame(main_frame)
        content_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        # Left panel
        left_panel = ttk.Frame(content_frame, width=250)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, padx=5)
        self.create_left_panel(left_panel)

        # Right panel (plotting)
        right_panel = ttk.Frame(content_frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)
        self.create_right_panel(right_panel)

        # Status bar
        self.create_status_bar(main_frame)

        # Start BLE event loop in background
        self.setup_ble()

        # Start periodic UI update (10 Hz)
        self.after(100, self.scheduled_update)

    def scheduled_update(self):
        """Periodic UI update (10 Hz = 100 ms) from main tkinter thread"""
        self.update_telemetry_display()
        self.refresh_plots()
        self.after(100, self.scheduled_update)

    def on_closing(self):
        """Auto-save CSV if buffer has data"""
        if app_state.telemetry_buffer:
            self.save_csv_to_disk()
        self.destroy()

    def create_control_bar(self, parent):
        bar = ttk.Frame(parent)
        bar.pack(fill=tk.X, pady=5)

        ttk.Label(bar, text="GyroBot", font=("Arial", 16, "bold")).pack(side=tk.LEFT)

        self.status_label = ttk.Label(bar, text="Scanning for GyroBot_BLE...", foreground="blue")
        self.status_label.pack(side=tk.LEFT, padx=20)

        self.disconnect_btn = ttk.Button(bar, text="Disconnect", command=self.on_disconnect, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, padx=5)

        ttk.Separator(bar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)

        self.save_csv_btn = ttk.Button(bar, text="Save CSV", command=self.on_save_csv)
        self.save_csv_btn.pack(side=tk.LEFT, padx=5)

        self.csv_status_label = ttk.Label(bar, text="", foreground="gray")
        self.csv_status_label.pack(side=tk.LEFT, padx=5)

    def create_left_panel(self, parent):
        # Sequence control
        seq_frame = ttk.LabelFrame(parent, text="Sequences", padding=10)
        seq_frame.pack(fill=tk.X, pady=5)

        ttk.Button(seq_frame, text="S1: Forward", width=18,
                   command=lambda: self.send_command("S1")).pack(fill=tk.X, pady=2)
        ttk.Button(seq_frame, text="S2: Circle", width=18,
                   command=lambda: self.send_command("S2")).pack(fill=tk.X, pady=2)
        ttk.Button(seq_frame, text="S3: Arrow", width=18,
                   command=lambda: self.send_command("S3")).pack(fill=tk.X, pady=2)
        ttk.Button(seq_frame, text="FLECHE: Arrow", width=18,
                   command=lambda: self.send_command("FLECHE")).pack(fill=tk.X, pady=2)

        # Emergency Stop
        stop_frame = ttk.LabelFrame(parent, text="Emergency Stop", padding=10)
        stop_frame.pack(fill=tk.X, pady=5)

        stop_btn = ttk.Button(stop_frame, text="STOP", command=lambda: self.send_command("STOP"))
        stop_btn.pack(fill=tk.X, pady=5)
        stop_btn.config(style='Stop.TButton')

        # CSV Logging
        csv_frame = ttk.LabelFrame(parent, text="CSV Logging", padding=10)
        csv_frame.pack(fill=tk.X, pady=5)

        ttk.Label(csv_frame, text="Buffer size:", font=("Arial", 9)).pack(anchor=tk.W)
        self.buffer_label = ttk.Label(csv_frame, text="0 rows", font=("Courier", 8))
        self.buffer_label.pack(anchor=tk.W)

        ttk.Label(csv_frame, text="Last file:", font=("Arial", 9)).pack(anchor=tk.W, pady=(5, 0))
        self.file_label = ttk.Label(csv_frame, text="(none)", font=("Courier", 7), foreground="gray")
        self.file_label.pack(anchor=tk.W)

        # Telemetry
        tel_frame = ttk.LabelFrame(parent, text="Telemetry", padding=10)
        tel_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.telemetry_text = tk.Text(tel_frame, height=15, width=30, font=("Courier", 8))
        self.telemetry_text.pack(fill=tk.BOTH, expand=True)

        scrollbar = ttk.Scrollbar(tel_frame, command=self.telemetry_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.telemetry_text['yscrollcommand'] = scrollbar.set

    def create_right_panel(self, parent):
        # Create figure with subplots
        self.fig = Figure(figsize=(9, 6), dpi=100)
        self.fig.tight_layout(pad=3)

        # Robot track plot
        self.ax_track = self.fig.add_subplot(2, 2, 1)
        self.ax_track.set_title("Robot Track")
        self.ax_track.set_xlabel("X (mm)")
        self.ax_track.set_ylabel("Y (mm)")
        self.ax_track.grid(True, alpha=0.3)
        self.track_line, = self.ax_track.plot([], [], 'b-', alpha=0.7)
        self.pen_dot, = self.ax_track.plot([], [], 'ro', markersize=8)

        # Compass plot
        self.ax_compass = self.fig.add_subplot(2, 2, 2, projection='polar')
        self.ax_compass.set_title("Heading")
        self.ax_compass.set_theta_zero_location('N')
        self.ax_compass.set_theta_direction(-1)
        self.ax_compass.set_ylim(0, 1)
        self.ax_compass.set_xticks([
            0, math.pi / 4, math.pi / 2, 3 * math.pi / 4,
            math.pi, 5 * math.pi / 4, 3 * math.pi / 2, 7 * math.pi / 4
        ])
        self.ax_compass.set_xticklabels(['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'])
        self.ax_compass.grid(True, alpha=0.3)
        self.compass_arrow = None

        # Speed plot
        self.ax_speed = self.fig.add_subplot(2, 2, 3)
        self.ax_speed.set_title("Wheel Speeds")
        self.ax_speed.set_ylabel("Speed (mm/s)")
        self.ax_speed.grid(True, alpha=0.3)
        self.vl_line, = self.ax_speed.plot([], label='Left', alpha=0.7)
        self.vr_line, = self.ax_speed.plot([], label='Right', alpha=0.7)
        self.ax_speed.legend(loc='upper right')

        # IMU plot (acceleration)
        self.ax_imu = self.fig.add_subplot(2, 2, 4)
        self.ax_imu.set_title("Acceleration (X-axis)")
        self.ax_imu.set_ylabel("m/s²")
        self.ax_imu.grid(True, alpha=0.3)
        self.accel_line, = self.ax_imu.plot([], label='AccelX', alpha=0.7)
        self.ax_imu.legend(loc='upper right')

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def create_status_bar(self, parent):
        bar = ttk.Frame(parent, relief=tk.SUNKEN, height=20)
        bar.pack(fill=tk.X, pady=5)
        self.status_bar_label = ttk.Label(bar, text="Ready", font=("Arial", 9))
        self.status_bar_label.pack(side=tk.LEFT, padx=5)

    def refresh_plots(self):
        """Refresh all matplotlib plots"""
        try:
            # Track plot
            if app_state.pen_x_history and app_state.pen_y_history:
                x = list(app_state.pen_x_history)
                y = list(app_state.pen_y_history)
                self.track_line.set_data(x, y)
                self.pen_dot.set_data(x[-1:], y[-1:])
                self.ax_track.relim()
                self.ax_track.autoscale_view()
            else:
                self.track_line.set_data([], [])
                self.pen_dot.set_data([], [])

            # Compass arrow
            if self.compass_arrow is not None:
                self.compass_arrow.remove()
                self.compass_arrow = None

            if "heading" in app_state.current_telemetry:
                try:
                    heading_deg = float(app_state.current_telemetry["heading"])
                    heading_rad = heading_deg * math.pi / 180.0
                    self.compass_arrow = self.ax_compass.annotate(
                        '', xy=(heading_rad, 1), xytext=(0, 0),
                        arrowprops=dict(facecolor='red', edgecolor='red', headwidth=10)
                    )
                except (ValueError, TypeError):
                    pass

            # Speed plot
            if app_state.vl_history and app_state.vr_history:
                vl = list(app_state.vl_history)
                vr = list(app_state.vr_history)
                self.vl_line.set_data(range(len(vl)), vl)
                self.vr_line.set_data(range(len(vr)), vr)
                self.ax_speed.relim()
                self.ax_speed.autoscale_view()
            else:
                self.vl_line.set_data([], [])
                self.vr_line.set_data([], [])

            # IMU plot (acceleration)
            if app_state.accel_x_history:
                ax = list(app_state.accel_x_history)
                self.accel_line.set_data(range(len(ax)), ax)
                self.ax_imu.relim()
                self.ax_imu.autoscale_view()
            else:
                self.accel_line.set_data([], [])

            self.canvas.draw()
        except Exception as e:
            print(f"Plot error: {e}")

    def setup_ble(self):
        """Setup BLE event loop in background thread"""
        self.loop = asyncio.new_event_loop()
        self.ble_thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.ble_thread.start()
        # Start auto-connect immediately
        asyncio.run_coroutine_threadsafe(self.ble_auto_connect(), self.loop)

    async def ble_auto_connect(self):
        """Auto-connect with infinite retry loop"""
        while True:
            try:
                await self.ble_connect()
            except Exception as e:
                print(f"Connection attempt failed: {e}")
            # Retry every 5 seconds
            await asyncio.sleep(5)

    async def ble_connect(self):
        """Connect to GyroBot_BLE via BLE"""
        app_state.scanning = True
        self.status_label.config(text="Scanning...", foreground="blue")

        try:
            print("Scanning for BLE devices...")
            device = await BleakScanner.find_device_by_name(ROBOT_NAME, timeout=5)

            if not device:
                print(f"Device '{ROBOT_NAME}' not found")
                return

            print(f"Found {device.name} ({device.address})")
            print(f"Connecting...")
            app_state.scanning = False
            self.status_label.config(text="Connecting...", foreground="orange")

            app_state.client = BleakClient(device.address, timeout=10.0)
            await app_state.client.connect()

            print(f"Connected! Starting notifications...")

            # Clear telemetry buffer on new connection
            app_state.telemetry_buffer = []
            app_state.logging = True

            await app_state.client.start_notify(CHAR_TX_UUID, self.on_telemetry)

            app_state.connected = True
            self.update_connection_ui()
            print(f"✓ Connected to {device.name}")

            # Keep connection alive, monitor actual BLE connection state
            while app_state.connected and app_state.client.is_connected:
                await asyncio.sleep(1)

            # Connection lost
            if app_state.connected:
                print("BLE connection lost")
                app_state.connected = False
                self.update_connection_ui()

        except asyncio.TimeoutError:
            print("Connection timeout")
        except Exception as e:
            print(f"Connection error: {e}")
        finally:
            # Cleanup
            if app_state.client:
                try:
                    if app_state.client.is_connected:
                        await app_state.client.disconnect()
                except Exception as e:
                    print(f"Disconnect error: {e}")
                app_state.client = None
            app_state.connected = False
            self.update_connection_ui()

    def on_disconnect(self):
        """Disconnect from robot"""
        asyncio.run_coroutine_threadsafe(self.ble_disconnect(), self.loop)

    async def ble_disconnect(self):
        """Disconnect from GyroBot"""
        if app_state.client and app_state.connected:
            try:
                print("Disconnecting...")
                await app_state.client.disconnect()
                print("✓ Disconnected")
            except Exception as e:
                print(f"Disconnect error: {e}")
        app_state.connected = False
        self.update_connection_ui()

    def on_telemetry(self, sender, data):
        """Handle incoming telemetry (callback - should be fast, no UI updates)"""
        try:
            if not data:
                return

            s = data.decode('utf-8').strip()
            if not s:
                return

            # Parse CSV (22 fields)
            parts = [p.strip() for p in s.split(',')]
            if len(parts) != len(CSV_FIELDS):
                print(f"Warning: telemetry CSV has {len(parts)} fields, expected {len(CSV_FIELDS)}")
                return

            try:
                telemetry = {}
                for i, field_name in enumerate(CSV_FIELDS):
                    val_str = parts[i]
                    try:
                        val = float(val_str)
                        # Convert specific fields to int or bool
                        if field_name in ('L_steps', 'R_steps'):
                            telemetry[field_name] = int(val) if math.isfinite(val) else 0
                        elif field_name == 'isMoving':
                            telemetry[field_name] = (int(val) != 0) if math.isfinite(val) else False
                        else:
                            telemetry[field_name] = val
                    except ValueError:
                        telemetry[field_name] = val_str

            except Exception as e:
                print(f"CSV parse error: {e}")
                return

            app_state.current_telemetry = telemetry

            # Update history deques
            try:
                penX = float(telemetry.get('penX', 0))
                penY = float(telemetry.get('penY', 0))
                app_state.pen_x_history.append(penX)
                app_state.pen_y_history.append(penY)

                vL = float(telemetry.get('L_steps', 0)) / 10.0  # Approximate speed from steps
                vR = float(telemetry.get('R_steps', 0)) / 10.0
                app_state.vl_history.append(vL)
                app_state.vr_history.append(vR)

                accX = float(telemetry.get('accX', 0))
                app_state.accel_x_history.append(accX)

                heading_deg = float(telemetry.get('heading', 0))
                app_state.heading_history.append(heading_deg)
            except (ValueError, TypeError):
                pass

            # Buffer telemetry if logging
            if app_state.logging:
                record = {'timestamp': datetime.now().isoformat()}
                record.update(telemetry)
                app_state.telemetry_buffer.append(record)

        except Exception as e:
            print(f"Telemetry error: {e}")

    def update_telemetry_display(self):
        """Update telemetry text box"""
        try:
            self.telemetry_text.config(state=tk.NORMAL)
            self.telemetry_text.delete('1.0', tk.END)

            if not app_state.current_telemetry:
                self.telemetry_text.insert(tk.END, "Waiting for data...\n")
            else:
                for key in CSV_FIELDS:
                    if key in app_state.current_telemetry:
                        value = app_state.current_telemetry[key]
                        if isinstance(value, float):
                            text = f"{key}: {value:.2f}\n"
                        elif isinstance(value, bool):
                            text = f"{key}: {'Yes' if value else 'No'}\n"
                        else:
                            text = f"{key}: {value}\n"
                        self.telemetry_text.insert(tk.END, text)

            self.telemetry_text.config(state=tk.DISABLED)

            # Update buffer count
            self.buffer_label.config(text=f"{len(app_state.telemetry_buffer)} rows")

        except Exception as e:
            print(f"Display update error: {e}")

    def update_connection_ui(self):
        """Update UI based on connection state"""
        if app_state.connected:
            self.status_label.config(text="Connected", foreground="green")
            self.disconnect_btn.config(state=tk.NORMAL)
            self.status_bar_label.config(text="Connected to GyroBot_BLE")
        else:
            self.status_label.config(text="Disconnected", foreground="red")
            self.disconnect_btn.config(state=tk.DISABLED)
            self.status_bar_label.config(text="Scanning for GyroBot_BLE...")

    async def _send_command_async(self, text: str):
        """Send plain-text command to robot"""
        if not app_state.connected:
            return
        try:
            await app_state.client.write_gatt_char(CHAR_RX_UUID, text.encode())
            print(f"Sent: {text}")
        except Exception as e:
            print(f"Send error: {e}")

    def send_command(self, text: str):
        """Send plain-text command to robot"""
        if not app_state.connected:
            messagebox.showwarning("Not Connected", "Robot not connected yet. Please wait for auto-connect.")
            return
        asyncio.run_coroutine_threadsafe(self._send_command_async(text), self.loop)

    def save_csv_to_disk(self):
        """Save buffered telemetry to CSV file"""
        if not app_state.telemetry_buffer:
            messagebox.showwarning("CSV", "No telemetry data to save.")
            return

        output_dir = os.path.join(self.script_dir, "Output")
        os.makedirs(output_dir, exist_ok=True)
        filename = f"gyrobot_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        filepath = os.path.join(output_dir, filename)

        try:
            with open(filepath, 'w', newline='') as f:
                # Use CSV_FIELDS as header
                writer = csv.DictWriter(f, fieldnames=['timestamp'] + CSV_FIELDS)
                writer.writeheader()
                writer.writerows(app_state.telemetry_buffer)

            self.file_label.config(text=filename, foreground="green")
            messagebox.showinfo("CSV Saved", f"Saved {len(app_state.telemetry_buffer)} rows to {filepath}")
            print(f"CSV saved: {filepath}")
        except Exception as e:
            messagebox.showerror("CSV Error", f"Could not save file: {e}")
            print(f"CSV save error: {e}")

        app_state.telemetry_buffer = []

    def on_save_csv(self):
        """Handle Save CSV button click"""
        self.save_csv_to_disk()

if __name__ == '__main__':
    root = GyroBotGUI()
    root.mainloop()
