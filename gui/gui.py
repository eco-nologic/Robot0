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
from dataclasses import dataclass
import math
from concurrent.futures import TimeoutError

# Add Documents directory to path for simulateur import
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'Documents'))
from simulateur import run_gcode_ik

try:
    from bleak import BleakScanner, BleakClient
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
except ImportError as e:
    print(f"Missing dependency: {e}")
    print("Install with: pip install bleak matplotlib")
    sys.exit(1)

# === PID Tuning Constants ===
@dataclass
class PIDValues:
    kp: float
    ki: float
    kd: float

DEFAULT_PID_FWD = PIDValues(kp=2.0, ki=0.1, kd=0.5)
DEFAULT_PID_REV = PIDValues(kp=5.0, ki=0.0, kd=0.0)

# === BLE Configuration (from BleManager.cpp) ===
ROBOT_NAME = "GyroBot_BLE"
SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
CHAR_TX_UUID = "87654321-4321-4321-4321-cba987654321"  # Notify  (telemetry in)
CHAR_RX_UUID = "11111111-2222-3333-4444-555555555555"  # Write   (commands out)

# === CSV Header and Field Names (from Telemetry.cpp) ===
CSV_HEADER = "timestamp,penX,penY,robotHeading,heading,L_steps,R_steps,accX,accY,accZ,gyroX,gyroY,gyroZ,magX,magY,magZ,batt,isMoving,targetL,actualV,targetW,actualW"
CSV_FIELDS = CSV_HEADER.split(',')

# === GCode Parsing ===
def parse_gcode_to_waypoints(gcode_content: str) -> list:
    """Parse G0/G1 X Y lines → list of (x, y) waypoints."""
    waypoints = []
    for line in gcode_content.splitlines():
        line = line.strip()
        if line.startswith(';') or not line:
            continue
        if not (line.startswith('G0') or line.startswith('G1')):
            continue
        x, y = None, None
        for token in line.split():
            if token.startswith('X'):
                try: x = float(token[1:])
                except: pass
            elif token.startswith('Y'):
                try: y = float(token[1:])
                except: pass
        if x is not None and y is not None:
            waypoints.append((x, y))
    return waypoints

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
        self.ik_sequences = []     # list of (tg, td, vg, vd, dg, dd)
        self.ik_sending = False

app_state = AppState()

class GyroBotGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("GyroBot Control Panel")
        self.geometry("1400x900")
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.script_dir = os.path.dirname(os.path.abspath(__file__))

        # PID tuning StringVars
        self.kp_fwd = tk.StringVar(value=str(DEFAULT_PID_FWD.kp))
        self.ki_fwd = tk.StringVar(value=str(DEFAULT_PID_FWD.ki))
        self.kd_fwd = tk.StringVar(value=str(DEFAULT_PID_FWD.kd))
        self.kp_rev = tk.StringVar(value=str(DEFAULT_PID_REV.kp))
        self.ki_rev = tk.StringVar(value=str(DEFAULT_PID_REV.ki))
        self.kd_rev = tk.StringVar(value=str(DEFAULT_PID_REV.kd))

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

    def on_clear_track(self):
        """Clear the track history plot"""
        app_state.pen_x_history.clear()
        app_state.pen_y_history.clear()
        self.track_line.set_data([], [])
        self.pen_dot.set_data([], [])
        self.canvas.draw()

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
        # Create scrollable frame for left panel (fixes small screen issue)
        canvas = tk.Canvas(parent, highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Bind mouse wheel to scroll
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        canvas.bind_all("<MouseWheel>", _on_mousewheel)

        # Now pack all controls into scrollable_frame instead of parent
        panel = scrollable_frame

        # Compass Calibration (AT TOP - always visible)
        compass_frame = ttk.LabelFrame(panel, text="Compass Offset", padding=10)
        compass_frame.pack(fill=tk.X, pady=5)

        ttk.Label(compass_frame, text="Heading Offset (degrees):", font=("Arial", 9)).pack(anchor=tk.W)
        offset_input_frame = ttk.Frame(compass_frame)
        offset_input_frame.pack(fill=tk.X, pady=3)
        self.heading_offset_var = tk.StringVar(value="-90")
        ttk.Entry(offset_input_frame, textvariable=self.heading_offset_var, width=10).pack(side=tk.LEFT, padx=2)
        ttk.Label(offset_input_frame, text="°", font=("Arial", 9)).pack(side=tk.LEFT)

        self.heading_offset_btn = ttk.Button(compass_frame, text="Set & Save to Robot",
                   command=self.on_set_heading_offset, state=tk.DISABLED)
        self.heading_offset_btn.pack(fill=tk.X, pady=2)

        # Sequence control
        seq_frame = ttk.LabelFrame(panel, text="Sequences", padding=10)
        seq_frame.pack(fill=tk.X, pady=5)

        ttk.Button(seq_frame, text="S1: Escalier (Lignes & 90°)", width=25,
                   command=lambda: self.send_command("S1")).pack(fill=tk.X, pady=2)
        ttk.Button(seq_frame, text="S2: Cercle (Rotation 390°)", width=25,
                   command=lambda: self.send_command("S2")).pack(fill=tk.X, pady=2)
        ttk.Button(seq_frame, text="S3: Boussole (Calibration & Nord)", width=25,
                   command=lambda: self.send_command("S3")).pack(fill=tk.X, pady=2)
        ttk.Button(seq_frame, text="Dessiner uniquement la Flèche", width=25,
                   command=lambda: self.send_command("FLECHE")).pack(fill=tk.X, pady=2)
        ttk.Button(seq_frame, text="📐 4️⃣ Corner 90 (Courbe fluide)", width=25,
                   command=lambda: self.send_command("CORNER90")).pack(fill=tk.X, pady=2)

        # Emergency Stop
        stop_frame = ttk.LabelFrame(panel, text="Emergency Stop", padding=10)
        stop_frame.pack(fill=tk.X, pady=5)

        stop_btn = ttk.Button(stop_frame, text="ARRÊT D'URGENCE (STOP)", command=lambda: self.send_command("STOP"))
        stop_btn.pack(fill=tk.X, pady=5)
        stop_btn.config(style='Stop.TButton')

        # Reset/Diagnostics
        reset_frame = ttk.LabelFrame(panel, text="Diagnostics", padding=10)
        reset_frame.pack(fill=tk.X, pady=5)

        ttk.Button(reset_frame, text="Reset Odometry", width=25,
                   command=lambda: self.send_command("RESET_ODO")).pack(fill=tk.X, pady=2)
        ttk.Button(reset_frame, text="Clear Track History", width=25,
                   command=self.on_clear_track).pack(fill=tk.X, pady=2)

        # CSV Logging
        csv_frame = ttk.LabelFrame(panel, text="CSV Logging", padding=10)
        csv_frame.pack(fill=tk.X, pady=5)

        ttk.Label(csv_frame, text="Buffer size:", font=("Arial", 9)).pack(anchor=tk.W)
        self.buffer_label = ttk.Label(csv_frame, text="0 rows", font=("Courier", 8))
        self.buffer_label.pack(anchor=tk.W)

        ttk.Label(csv_frame, text="Last file:", font=("Arial", 9)).pack(anchor=tk.W, pady=(5, 0))
        self.file_label = ttk.Label(csv_frame, text="(none)", font=("Courier", 7), foreground="gray")
        self.file_label.pack(anchor=tk.W)

        # Telemetry
        tel_frame = ttk.LabelFrame(panel, text="Telemetry", padding=10)
        tel_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.telemetry_text = tk.Text(tel_frame, height=15, width=30, font=("Courier", 8))
        self.telemetry_text.pack(fill=tk.BOTH, expand=True)

        scrollbar = ttk.Scrollbar(tel_frame, command=self.telemetry_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.telemetry_text['yscrollcommand'] = scrollbar.set

        # PID Tuning
        pid_frame = ttk.LabelFrame(panel, text="PID Tuning", padding=10)
        pid_frame.pack(fill=tk.X, pady=5)

        ttk.Label(pid_frame, text="Forward Movement", font=("Arial", 9, "bold")).pack(anchor=tk.W)
        fwd_frame = ttk.Frame(pid_frame)
        fwd_frame.pack(fill=tk.X, pady=2)
        ttk.Label(fwd_frame, text="Kp:").pack(side=tk.LEFT, padx=2)
        ttk.Entry(fwd_frame, textvariable=self.kp_fwd, width=6).pack(side=tk.LEFT, padx=1)
        ttk.Label(fwd_frame, text="Ki:").pack(side=tk.LEFT, padx=2)
        ttk.Entry(fwd_frame, textvariable=self.ki_fwd, width=6).pack(side=tk.LEFT, padx=1)
        ttk.Label(fwd_frame, text="Kd:").pack(side=tk.LEFT, padx=2)
        ttk.Entry(fwd_frame, textvariable=self.kd_fwd, width=6).pack(side=tk.LEFT, padx=1)

        ttk.Label(pid_frame, text="Reverse Movement", font=("Arial", 9, "bold")).pack(anchor=tk.W, pady=(5, 0))
        rev_frame = ttk.Frame(pid_frame)
        rev_frame.pack(fill=tk.X, pady=2)
        ttk.Label(rev_frame, text="Kp:").pack(side=tk.LEFT, padx=2)
        ttk.Entry(rev_frame, textvariable=self.kp_rev, width=6).pack(side=tk.LEFT, padx=1)
        ttk.Label(rev_frame, text="Ki:").pack(side=tk.LEFT, padx=2)
        ttk.Entry(rev_frame, textvariable=self.ki_rev, width=6).pack(side=tk.LEFT, padx=1)
        ttk.Label(rev_frame, text="Kd:").pack(side=tk.LEFT, padx=2)
        ttk.Entry(rev_frame, textvariable=self.kd_rev, width=6).pack(side=tk.LEFT, padx=1)

        ttk.Button(pid_frame, text="Apply PID", command=self.on_apply_pid).pack(fill=tk.X, pady=3)
        ttk.Button(pid_frame, text="Reset to Defaults", command=self.on_reset_pid).pack(fill=tk.X, pady=2)

        # IK Path (GCode)
        ik_frame = ttk.LabelFrame(panel, text="IK Path (GCode)", padding=10)
        ik_frame.pack(fill=tk.X, pady=5)

        ttk.Button(ik_frame, text="Ouvrir GCode...",
                   command=self.on_open_gcode).pack(fill=tk.X, pady=2)

        self.ik_info_label = ttk.Label(ik_frame, text="Aucun fichier chargé",
                                        font=("Courier", 8), foreground="gray")
        self.ik_info_label.pack(anchor=tk.W, pady=2)

        self.ik_progress = ttk.Progressbar(ik_frame, mode='determinate')
        self.ik_progress.pack(fill=tk.X, pady=2)

        self.ik_progress_label = ttk.Label(ik_frame, text="", font=("Courier", 8))
        self.ik_progress_label.pack(anchor=tk.W)

        ik_btn_frame = ttk.Frame(ik_frame)
        ik_btn_frame.pack(fill=tk.X, pady=3)

        self.ik_send_btn = ttk.Button(ik_btn_frame, text="Envoyer",
                                       command=self.on_send_ik_path, state=tk.DISABLED)
        self.ik_send_btn.pack(side=tk.LEFT, padx=2)

        self.ik_stop_btn = ttk.Button(ik_btn_frame, text="Arrêt",
                                       command=self.on_stop_ik_path, state=tk.DISABLED)
        self.ik_stop_btn.pack(side=tk.LEFT, padx=2)

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
            # Enable compass offset button when connected
            self.heading_offset_btn.config(state=tk.NORMAL)
            # Enable IK send button if sequences are loaded
            if app_state.ik_sequences:
                self.ik_send_btn.config(state=tk.NORMAL)
        else:
            self.status_label.config(text="Disconnected", foreground="red")
            self.disconnect_btn.config(state=tk.DISABLED)
            self.status_bar_label.config(text="Scanning for GyroBot_BLE...")
            self.heading_offset_btn.config(state=tk.DISABLED)
            self.ik_send_btn.config(state=tk.DISABLED)

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

    def on_apply_pid(self):
        """Send PID values to robot"""
        if not app_state.connected:
            messagebox.showwarning("Not Connected", "Robot not connected yet. Please wait for auto-connect.")
            return

        try:
            kp_fwd = float(self.kp_fwd.get())
            ki_fwd = float(self.ki_fwd.get())
            kd_fwd = float(self.kd_fwd.get())
            self.send_command(f"PID_FWD:{kp_fwd},{ki_fwd},{kd_fwd}")

            kp_rev = float(self.kp_rev.get())
            ki_rev = float(self.ki_rev.get())
            kd_rev = float(self.kd_rev.get())
            self.send_command(f"PID_REV:{kp_rev},{ki_rev},{kd_rev}")

            messagebox.showinfo("PID Updated", "Values sent to robot")
        except ValueError:
            messagebox.showerror("Input Error", "All PID values must be numbers")

    def _apply_pid_to_vars(self, fwd: PIDValues, rev: PIDValues):
        """Copy PIDValues objects into the StringVar fields"""
        self.kp_fwd.set(str(fwd.kp))
        self.ki_fwd.set(str(fwd.ki))
        self.kd_fwd.set(str(fwd.kd))
        self.kp_rev.set(str(rev.kp))
        self.ki_rev.set(str(rev.ki))
        self.kd_rev.set(str(rev.kd))

    def on_reset_pid(self):
        """Reset PID values to defaults"""
        self._apply_pid_to_vars(DEFAULT_PID_FWD, DEFAULT_PID_REV)
        self.send_command("PID_RESET")
        messagebox.showinfo("PID Reset", "Values reset to defaults")

    def on_set_heading_offset(self):
        """Set heading offset and save to robot EEPROM"""
        try:
            offset = float(self.heading_offset_var.get())
            cmd = f"SET_HEADING_OFFSET:{offset:.1f}"
            self.send_command(cmd)
            messagebox.showinfo(
                "Compass Calibrated",
                f"Heading offset set to {offset}°\nSaved to robot memory"
            )
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter a valid number for offset")

    def on_open_gcode(self):
        """Open and parse a GCode file"""
        from tkinter import filedialog
        gcode_dir = os.path.join(self.script_dir, "GCode")
        os.makedirs(gcode_dir, exist_ok=True)
        path = filedialog.askopenfilename(
            title="Ouvrir fichier GCode",
            initialdir=gcode_dir,
            filetypes=[("GCode", "*.gcode *.nc *.txt"), ("Tous", "*.*")]
        )
        if not path:
            return
        try:
            with open(path, 'r', encoding='utf-8') as f:
                content = f.read()
        except Exception as e:
            messagebox.showerror("Erreur", f"Impossible de lire le fichier : {e}")
            return

        waypoints = parse_gcode_to_waypoints(content)
        if not waypoints:
            messagebox.showwarning("GCode", "Aucun waypoint trouvé dans ce fichier.")
            return

        # Compute IK path from waypoints
        _, sequences = run_gcode_ik(waypoints)
        app_state.ik_sequences = sequences

        fname = os.path.basename(path)
        self.ik_info_label.config(
            text=f"{fname}\n{len(waypoints)} pts → {len(sequences)} segments IK",
            foreground="black"
        )
        self.ik_progress.config(maximum=max(1, len(sequences)), value=0)
        self.ik_progress_label.config(text=f"0 / {len(sequences)}")
        self.ik_send_btn.config(state=tk.NORMAL if app_state.connected else tk.DISABLED)

    def on_send_ik_path(self):
        """Send IK path to robot"""
        if not app_state.connected or not app_state.ik_sequences:
            return
        app_state.ik_sending = True
        self.ik_send_btn.config(state=tk.DISABLED)
        self.ik_stop_btn.config(state=tk.NORMAL)
        asyncio.run_coroutine_threadsafe(self._send_ik_path_async(), self.loop)

    def on_stop_ik_path(self):
        """Stop sending IK path"""
        app_state.ik_sending = False
        self.send_command("STOP")
        self.ik_send_btn.config(state=tk.NORMAL)
        self.ik_stop_btn.config(state=tk.DISABLED)

    async def _send_ik_path_async(self):
        """Send IK path segments sequentially"""
        sequences = app_state.ik_sequences
        total = len(sequences)
        for i, (tg, td, vg, vd, dg, dd) in enumerate(sequences):
            if not app_state.ik_sending or not app_state.connected:
                break
            cmd = f"IK_MOVE:{tg},{td},{vg},{vd},{dg:.2f},{dd:.2f}"
            await self._send_command_async(cmd)
            self.after(0, lambda i=i: (
                self.ik_progress.config(value=i + 1),
                self.ik_progress_label.config(text=f"{i+1} / {total}")
            ))
            await asyncio.sleep(1.1)
        app_state.ik_sending = False
        self.after(0, lambda: (
            self.ik_send_btn.config(state=tk.NORMAL),
            self.ik_stop_btn.config(state=tk.DISABLED),
            self.ik_progress_label.config(text=f"Terminé — {total} segments envoyés")
        ))

if __name__ == '__main__':
    root = GyroBotGUI()
    root.mainloop()
