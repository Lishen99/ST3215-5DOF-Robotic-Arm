import tkinter as tk
from tkinter import ttk
import serial.tools.list_ports
import sys
import os
import threading
import queue
import time
import serial
import json
import numpy as np
import kinematics

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

sys.path.append(os.path.join(os.path.dirname(__file__), "stservo-env"))

from STservo_sdk import *

# --- Constants ---
BAUDRATE = 1000000
MAX_MOTOR_ID = 252
MIN_POSITION, MAX_POSITION, START_POSITION = 0, 4095, 2047
DEFAULT_SPEED = 0
DEFAULT_ACC = 0
POLLING_DELAY_S = 0.1
SERVO_LIMITS_FILE = 'servo_limits.json'

class MotorControlGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("STServo Motor Controller")
        self.master.geometry("1200x800")

        # Core components
        self.portHandler = None
        self.packetHandler = None
        self.servo_limits = {}
        self.motor_ids = []
        
        # UI State
        self.sliders = {}
        self.info_labels = {}
        self.control_widgets = {}
        self.torque_states = {}
        self.last_ik_solution = None

        # Threading and Queues
        self.command_queue = queue.Queue()
        self.command_thread = None
        self.polling_thread = None
        self.polling_started = False
        self.stop_threads = False
        self.sweep_task = None
        self.stop_sweep_flag = threading.Event()

        self._load_servo_limits()
        self._create_ui()

        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _create_ui(self):
        main_frame = ttk.Frame(self.master)
        main_frame.pack(fill=tk.BOTH, expand=True)

        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        # --- Left Panel Widgets ---
        connection_frame = ttk.LabelFrame(left_frame, text="Connection")
        connection_frame.pack(padx=10, pady=10, fill="x")
        self._create_connection_widgets(connection_frame)

        self.controls_frame = ttk.LabelFrame(left_frame, text="Motor Controls")
        self.controls_frame.pack(padx=10, pady=10, fill="x")
        self.controls_frame.columnconfigure(1, weight=1)

        self.ik_frame = ttk.LabelFrame(left_frame, text="Inverse Kinematics (IK)")
        self.ik_frame.pack(padx=10, pady=10, fill="x")
        self._create_ik_widgets(self.ik_frame)

        self.info_frame = ttk.LabelFrame(left_frame, text="Motor Information")
        self.info_frame.pack(padx=10, pady=10, fill="both", expand=True)
        
        # --- Right Panel Widgets ---
        vis_frame = ttk.LabelFrame(right_frame, text="Arm Visualization")
        vis_frame.pack(fill=tk.BOTH, expand=True)
        self._create_plot_widgets(vis_frame)

        self.status_label = ttk.Label(self.master, text="Select a COM port and click 'Open Port & Scan'.")
        self.status_label.pack(side="bottom", fill="x", padx=10, pady=5)

    def _create_connection_widgets(self, parent):
        ttk.Label(parent, text="COM Port:").pack(side="left", padx=5, pady=5)
        self.com_port_var = tk.StringVar()
        self.com_port_menu = ttk.Combobox(parent, textvariable=self.com_port_var, state="readonly")
        self.com_port_menu.pack(side="left", padx=5, pady=5)
        self.refresh_com_ports()
        self.scan_button = ttk.Button(parent, text="Open Port & Scan", command=self.scan_for_motors)
        self.scan_button.pack(side="left", padx=5, pady=5)
        self.close_button = ttk.Button(parent, text="Close Port", command=self.close_port, state="disabled")
        self.close_button.pack(side="left", padx=5, pady=5)

    def _load_servo_limits(self):
        try:
            with open(SERVO_LIMITS_FILE, 'r') as f: self.servo_limits = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            print(f"Warning: Could not load {SERVO_LIMITS_FILE}. Error: {e}")
            self.servo_limits = {}

    def refresh_com_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.com_port_menu['values'] = ports
        if ports: self.com_port_var.set(ports[0])

    def scan_for_motors(self):
        self.stop_threads = False
        self.scanning_in_progress = True
        self.last_ik_solution = None
        self.scan_button.config(text="Stop Scan", command=self.stop_scan)
        selected_port = self.com_port_var.get()
        if not selected_port: self.on_scan_complete([]); return

        self.status_label.config(text=f"Opening port and scanning on {selected_port}...")
        for frame in [self.controls_frame, self.info_frame]:
            for widget in frame.winfo_children(): widget.destroy()
        self.motor_ids.clear(); self.sliders.clear(); self.info_labels.clear(); self.control_widgets.clear(); self.torque_states.clear()
        self.command_queue = queue.Queue(); self.polling_started = False
        self.next_control_row = 0; self.next_info_row = 0

        headers = ["ID", "Status", "Position", "Voltage (V)", "Temp (°C)"]
        for i, header in enumerate(headers): ttk.Label(self.info_frame, text=header, font=('TkDefaultFont', 9, 'bold')).grid(row=0, column=i, padx=5, pady=2, sticky='w')

        try:
            self.portHandler = PortHandler(selected_port)
            self.packetHandler = sts(self.portHandler)
            if not self.portHandler.openPort() or not self.portHandler.setBaudRate(BAUDRATE): raise serial.SerialException("Port setup failed")
        except serial.SerialException as e:
            self.status_label.config(text=f"Error: Could not open port {selected_port}. ({e})")
            self.scan_button.config(text="Open Port & Scan", command=self.scan_for_motors)
            if self.portHandler and self.portHandler.is_open: self.portHandler.closePort()
            return
        threading.Thread(target=self._scan_worker, daemon=True).start()

    def on_scan_complete(self, found_motors):
        self.scanning_in_progress = False
        self.scan_button.config(text="Open Port & Scan", command=self.scan_for_motors)
        if not found_motors:
            self.status_label.config(text=f"Scan finished. No motors found.")
            if self.portHandler and self.portHandler.is_open: self.portHandler.closePort()
            return

        self.status_label.config(text=f"Scan finished. Found {len(found_motors)} motor(s): {found_motors}")
        self.scan_button.config(state="disabled"); self.close_button.config(state="normal"); self.com_port_menu.config(state="disabled")
        self.ik_go_button.config(state="normal"); self.sweep_x_button.config(state="normal"); self.sweep_y_button.config(state="normal"); self.sweep_z_button.config(state="normal")
        self.motor_ids = sorted(found_motors)
        
        for motor_id in self.motor_ids:
            self.packetHandler.write1ByteTxRx(motor_id, STS_TORQUE_ENABLE, 1)
            self.torque_states[motor_id] = 1
            self.add_motor_ui(motor_id)

        if self.motor_ids and not self.polling_started:
            self.command_thread = threading.Thread(target=self._command_worker, daemon=True); self.command_thread.start()
            self.polling_thread = threading.Thread(target=self._polling_worker, daemon=True); self.polling_thread.start()
            self.polling_started = True
            
    def close_port(self):
        if self.sweep_task and self.sweep_task.is_alive(): self._toggle_sweep('x', force_stop=True)
        self.stop_threads = True
        if self.command_thread and self.command_thread.is_alive(): self.command_queue.put(None); self.command_thread.join(timeout=1.0)
        if self.polling_thread and self.polling_thread.is_alive(): self.polling_thread.join(timeout=1.0)

        if self.portHandler and self.portHandler.is_open:
            for motor_id in list(self.torque_states.keys()):
                if self.torque_states.get(motor_id, 0) == 1:
                    try: self.packetHandler.write1ByteTxRx(motor_id, STS_TORQUE_ENABLE, 0)
                    except Exception: pass
            self.portHandler.closePort()

        for frame in [self.controls_frame, self.info_frame]:
            for widget in frame.winfo_children(): widget.destroy()
        self.motor_ids.clear(); self.sliders.clear(); self.info_labels.clear(); self.control_widgets.clear(); self.torque_states.clear()
        self.polling_started = False; self.polling_thread = None; self.command_thread = None
        self.scan_button.config(state="normal"); self.close_button.config(state="disabled"); self.com_port_menu.config(state="normal")
        self.ik_go_button.config(state="disabled"); self.sweep_x_button.config(state="disabled"); self.sweep_y_button.config(state="disabled"); self.sweep_z_button.config(state="disabled")
        self.status_label.config(text="Port closed. Ready to open again.")

    def stop_scan(self): self.scanning_in_progress = False

    def _scan_worker(self):
        found_motors = []
        for motor_id in range(MAX_MOTOR_ID + 1):
            if not self.scanning_in_progress: break
            if self.packetHandler.ping(motor_id)[1] == COMM_SUCCESS: found_motors.append(motor_id)
            time.sleep(0.001)
        self.master.after(0, self.on_scan_complete, found_motors)

    def add_motor_ui(self, motor_id):
        info_row = self.next_info_row + 1
        labels = {'id': ttk.Label(self.info_frame, text=str(motor_id)), 'status': ttk.Label(self.info_frame, text="OK", foreground="green"), 'pos': ttk.Label(self.info_frame, text="---"), 'volt': ttk.Label(self.info_frame, text="---"), 'temp': ttk.Label(self.info_frame, text="---")}
        for i, label in enumerate(labels.values()): label.grid(row=info_row, column=i, padx=5, pady=2, sticky='w')
        self.info_labels[motor_id] = labels
        self.next_info_row += 1
        if motor_id == 3: self.info_labels[3]['status'].config(text="Ganged"); return
        row = self.next_control_row
        self.control_widgets[motor_id] = {}
        motor_limits = self.servo_limits.get(str(motor_id), {'min': MIN_POSITION, 'max': MAX_POSITION})
        min_pos, max_pos = motor_limits['min'], motor_limits['max']
        start_pos = int((min_pos + max_pos) / 2)
        label_text = f"Motor {motor_id}:" + (" (Shoulder)" if motor_id == 2 else "")
        ttk.Label(self.controls_frame, text=label_text, width=15).grid(row=row, column=0, padx=5, sticky='w')
        slider = ttk.Scale(self.controls_frame, from_=min_pos, to=max_pos, orient='horizontal', command=lambda val, mid=motor_id: self.on_slider_move(mid, val))
        slider.set(start_pos); slider.grid(row=row, column=1, padx=5, sticky='ew'); self.sliders[motor_id] = slider
        pos_label = ttk.Label(self.controls_frame, text=str(start_pos), width=5); pos_label.grid(row=row, column=2, padx=5); self.control_widgets[motor_id]['pos_label'] = pos_label
        center_button = ttk.Button(self.controls_frame, text="Center", command=lambda mid=motor_id: self.center_motor(mid)); center_button.grid(row=row, column=3, padx=5)
        torque_button = ttk.Button(self.controls_frame, text="Torque OFF", command=lambda mid=motor_id: self.toggle_torque(mid)); torque_button.grid(row=row, column=4, padx=5); self.control_widgets[motor_id]['torque_button'] = torque_button
        move_list = [(motor_id, start_pos)]
        if motor_id == 2 and 3 in self.motor_ids: move_list.append((3, int((self.servo_limits.get('3', {'min':0,'max':4095})['min'] + self.servo_limits.get('3', {'min':0,'max':4095})['max']) / 2)))
        self.command_queue.put(move_list)
        self.next_control_row += 1

    def _command_worker(self):
        while not self.stop_threads:
            try:
                move_list = self.command_queue.get(timeout=0.5)
                if move_list is None: break
                for motor_id, position in move_list: self.packetHandler.RegWritePosEx(motor_id, position, DEFAULT_SPEED, DEFAULT_ACC)
                self.packetHandler.RegAction()
                self.command_queue.task_done()
            except queue.Empty: continue
            except Exception as e: print(f"[ERROR] Command worker: {e}")

    def _polling_worker(self):
        while not self.stop_threads:
            all_motor_data = {}
            for motor_id in self.motor_ids[:]:
                try:
                    bulk_data, _, _ = self.packetHandler.readTxRx(motor_id, STS_PRESENT_POSITION_L, 10)
                    if bulk_data: all_motor_data[motor_id] = {'pos': self.packetHandler.sts_makeword(bulk_data[0], bulk_data[1]), 'volt': bulk_data[6] / 10.0, 'temp': bulk_data[7]}
                except Exception: pass
            if all_motor_data: self.master.after(0, self.update_info_table, all_motor_data); self.master.after(0, self._update_plot, all_motor_data)
            time.sleep(POLLING_DELAY_S)

    def update_info_table(self, all_motor_data):
        for motor_id, data in all_motor_data.items():
            if motor_id in self.info_labels:
                for key in ['pos', 'volt', 'temp']: self.info_labels[motor_id][key].config(text=f"{data.get(key, '---'):.1f}" if key =='volt' else data.get(key, '---'))
            if motor_id in self.control_widgets: self.control_widgets[motor_id]['pos_label'].config(text=str(data.get('pos', '---')))

    def center_motor(self, motor_id):
        if motor_id in self.sliders: self.on_slider_move(motor_id, (self.sliders[motor_id].cget('to') + self.sliders[motor_id].cget('from'))/2)

    def toggle_torque(self, motor_id):
        ids = [motor_id] + ([3] if motor_id == 2 and 3 in self.motor_ids else [])
        for mid in ids:
            if mid in self.torque_states:
                new_state = 1 - self.torque_states[mid]
                self.packetHandler.write1ByteTxRx(mid, STS_TORQUE_ENABLE, new_state); self.torque_states[mid] = new_state
                if mid in self.control_widgets: self.control_widgets[mid]['torque_button'].config(text=f"Torque {'OFF' if new_state else 'ON'}")

    def on_slider_move(self, motor_id, value):
        slider_pos = int(float(value))
        if self.torque_states.get(motor_id, 0) == 1:
            move_list = []
            if motor_id == 2:
                limits2 = self.servo_limits.get('2', {'min': MIN_POSITION, 'max': MAX_POSITION})
                pos_to_send = (limits2['min'] + limits2['max']) - slider_pos
                move_list.append((2, pos_to_send))
                if 3 in self.motor_ids:
                    limits3 = self.servo_limits.get('3', {'min': MIN_POSITION, 'max': MAX_POSITION})
                    range2 = (limits2['max'] - limits2['min']) or 1; range3 = (limits3['max'] - limits3['min']) or 1
                    percentage2 = (pos_to_send - limits2['min']) / range2
                    move_list.append((3, int(limits3['max'] - (percentage2 * range3))))
            else:
                move_list.append((motor_id, slider_pos))
            if move_list: self.command_queue.put(move_list)

    def on_closing(self): self.close_port(); self.master.destroy()

    def _create_plot_widgets(self, parent_frame):
        self.fig = Figure(figsize=(6, 6), dpi=100); self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent_frame); self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self._update_plot(None)

    def _pos_to_angle(self, motor_id, pos):
        motor_id_str = str(motor_id)
        if motor_id_str not in self.servo_limits: return 0
        limits = self.servo_limits[motor_id_str]; min_pos, max_pos = limits['min'], limits['max']
        center_pos = (min_pos + max_pos) / 2
        
        # Assume a 360-degree range for the base, 270 for pitch joints
        servo_angle_range_deg = 360.0 if motor_id == 1 else 270.0
        ticks_per_deg = ((max_pos - min_pos) / servo_angle_range_deg) or 1

        center_angle_deg = 90 if motor_id == 2 else 0
        return center_angle_deg + (pos - center_pos) / ticks_per_deg

    def _angle_to_pos(self, motor_id, angle_deg):
        motor_id_str = str(motor_id)
        if motor_id_str not in self.servo_limits: return None
        limits = self.servo_limits[motor_id_str]; min_pos, max_pos = limits['min'], limits['max']
        center_pos = (min_pos + max_pos) / 2

        # Assume a 360-degree range for the base, 270 for pitch joints
        servo_angle_range_deg = 360.0 if motor_id == 1 else 270.0
        ticks_per_deg = ((max_pos - min_pos) / servo_angle_range_deg) or 1

        center_angle_deg = 90 if motor_id == 2 else 0
        pos = center_pos + (angle_deg - center_angle_deg) * ticks_per_deg
        return int(np.clip(pos, min_pos, max_pos))

    def _update_plot(self, all_motor_data):
        motor_map = {1: 0, 2: 1, 4: 2, 5: 3}; q_rad = [0, np.deg2rad(90), 0, 0]
        if all_motor_data:
            if 2 in all_motor_data:
                pos2 = all_motor_data[2].get('pos', self.sliders[2].get())
                slider_pos = (self.servo_limits['2']['min'] + self.servo_limits['2']['max']) - pos2
                q_rad[1] = np.deg2rad(self._pos_to_angle(2, slider_pos))
            for motor_id, data in all_motor_data.items():
                if motor_id in motor_map and motor_id != 2: q_rad[motor_map[motor_id]] = np.deg2rad(self._pos_to_angle(motor_id, data.get('pos', 0)))
        joint_coords = kinematics.forward_kinematics(q_rad); self.ax.clear()
        xs, ys, zs = [p[0] for p in joint_coords], [p[1] for p in joint_coords], [p[2] for p in joint_coords]
        self.ax.plot(xs, ys, zs, 'o-', color='dodgerblue', lw=3, markersize=6); self.ax.scatter([0], [0], [0], c='black', s=50, marker='s')
        max_reach = sum(kinematics.LINK_LENGTHS); self.ax.set_xlim([-max_reach, max_reach]); self.ax.set_ylim([-max_reach, max_reach]); self.ax.set_zlim([0, max_reach])
        self.ax.set_xlabel('X (mm)'); self.ax.set_ylabel('Y (mm)'); self.ax.set_zlabel('Z (mm)'); self.ax.view_init(elev=30., azim=45)
        tip_pos = joint_coords[-1]; self.current_x_label.config(text=f"(Current: {tip_pos[0]:>6.1f})"); self.current_y_label.config(text=f"(Current: {tip_pos[1]:>6.1f})"); self.current_z_label.config(text=f"(Current: {tip_pos[2]:>6.1f})")
        self.canvas.draw()

    def _create_ik_widgets(self, ik_frame):
        ttk.Label(ik_frame, text="X:").grid(row=0, column=0, padx=5, pady=2, sticky='w'); self.ik_x_var = tk.StringVar(value="200"); ttk.Entry(ik_frame, textvariable=self.ik_x_var, width=6).grid(row=0, column=1, padx=0, pady=2)
        self.current_x_label = ttk.Label(ik_frame, text="(Current: ---)", width=18); self.current_x_label.grid(row=1, column=0, columnspan=2, padx=5, pady=2, sticky='w')
        ttk.Label(ik_frame, text="Y:").grid(row=0, column=2, padx=(10,0), pady=2, sticky='w'); self.ik_y_var = tk.StringVar(value="0"); ttk.Entry(ik_frame, textvariable=self.ik_y_var, width=6).grid(row=0, column=3, padx=0, pady=2)
        self.current_y_label = ttk.Label(ik_frame, text="(Current: ---)", width=18); self.current_y_label.grid(row=1, column=2, columnspan=2, padx=5, pady=2, sticky='w')
        ttk.Label(ik_frame, text="Z:").grid(row=0, column=4, padx=(10,0), pady=2, sticky='w'); self.ik_z_var = tk.StringVar(value="150"); ttk.Entry(ik_frame, textvariable=self.ik_z_var, width=6).grid(row=0, column=5, padx=0, pady=2)
        self.current_z_label = ttk.Label(ik_frame, text="(Current: ---)", width=18); self.current_z_label.grid(row=1, column=4, columnspan=2, padx=5, pady=2, sticky='w')
        self.ik_go_button = ttk.Button(ik_frame, text="Go to Position", command=self._go_to_ik_position); self.ik_go_button.config(state="disabled"); self.ik_go_button.grid(row=2, column=0, columnspan=6, pady=5, sticky="ew")
        self.sweep_x_button = ttk.Button(ik_frame, text="Sweep X", command=lambda: self._toggle_sweep('x')); self.sweep_x_button.grid(row=4, column=0, columnspan=2, sticky="ew")
        self.sweep_y_button = ttk.Button(ik_frame, text="Sweep Y", command=lambda: self._toggle_sweep('y')); self.sweep_y_button.grid(row=4, column=2, columnspan=2, sticky="ew")
        self.sweep_z_button = ttk.Button(ik_frame, text="Sweep Z", command=lambda: self._toggle_sweep('z')); self.sweep_z_button.grid(row=4, column=4, columnspan=2, sticky="ew")
        self.sweep_x_button.config(state="disabled"); self.sweep_y_button.config(state="disabled"); self.sweep_z_button.config(state="disabled")
        self.lock_wrist_var = tk.BooleanVar(value=False)
        self.wrist_angle_var = tk.StringVar(value="-45")
        self.wrist_angle_entry = ttk.Entry(ik_frame, textvariable=self.wrist_angle_var, width=6, state="disabled")
        self.lock_wrist_check = ttk.Checkbutton(ik_frame, text="Lock Wrist Pitch Angle (°):", variable=self.lock_wrist_var, command=self._on_toggle_wrist_lock); self.lock_wrist_check.grid(row=3, column=0, columnspan=4, sticky="w")
        self.wrist_angle_entry.grid(row=3, column=4, columnspan=2)

    def _on_toggle_wrist_lock(self):
        self.wrist_angle_entry.config(state="normal" if self.lock_wrist_var.get() else "disabled")

    def _execute_ik_solution(self, solution_rad):
        self.last_ik_solution = solution_rad; q_deg = np.rad2deg(solution_rad)
        base_deg, shoulder_deg, elbow_deg, wrist_deg = q_deg
        move_list = [(1, self._angle_to_pos(1, base_deg)), (4, self._angle_to_pos(4, elbow_deg)), (5, self._angle_to_pos(5, wrist_deg))]
        pos2_shoulder_target = self._angle_to_pos(2, shoulder_deg)
        limits2 = self.servo_limits['2']; pos2_to_send = (limits2['min'] + limits2['max']) - pos2_shoulder_target
        move_list.append((2, pos2_to_send))
        if 3 in self.motor_ids:
            limits3 = self.servo_limits.get('3', {'min': MIN_POSITION, 'max': MAX_POSITION})
            range2 = (limits2['max'] - limits2['min']) or 1; range3 = (limits3['max'] - limits3['min']) or 1
            percentage2 = (pos2_to_send - limits2['min']) / range2
            move_list.append((3, int(limits3['max'] - (percentage2 * range3))))
        self.command_queue.put(move_list)
        self.sliders[1].set(move_list[0][1]); self.sliders[4].set(move_list[1][1]); self.sliders[5].set(move_list[2][1]); self.sliders[2].set(pos2_shoulder_target)

    def _go_to_ik_position(self):
        try: x, y, z = float(self.ik_x_var.get()), float(self.ik_y_var.get()), float(self.ik_z_var.get())
        except ValueError: self.status_label.config(text="Error: Invalid IK input."); return
        self.status_label.config(text=f"Calculating IK for...")
        required_motors = {1, 2, 4, 5}
        if not required_motors.issubset(self.motor_ids): self.status_label.config(text=f"Error: IK requires motors {required_motors}."); return
        joint_limits_deg = {mid: (self._pos_to_angle(mid, self.servo_limits[str(mid)]['min']), self._pos_to_angle(mid, self.servo_limits[str(mid)]['max'])) for mid in required_motors if str(mid) in self.servo_limits}
        preferred_phi = None
        if self.lock_wrist_var.get():
            try: preferred_phi = float(self.wrist_angle_var.get())
            except ValueError: self.status_label.config(text="Error: Invalid wrist angle."); return
        elif self.last_ik_solution: preferred_phi = np.rad2deg(self.last_ik_solution[3])
        solution_rad = kinematics.inverse_kinematics([x, y, z], joint_limits_deg, preferred_phi_deg=preferred_phi, use_locked_angle=self.lock_wrist_var.get())
        if solution_rad is None: self.status_label.config(text=f"IK Error: No solution found."); return
        self._execute_ik_solution(solution_rad)
        self.status_label.config(text=f"IK solution sent to motors.")

    def _toggle_sweep(self, axis, force_stop=False):
        buttons = {'x': self.sweep_x_button, 'y': self.sweep_y_button, 'z': self.sweep_z_button}
        if self.sweep_task and self.sweep_task.is_alive() or force_stop:
            self.stop_sweep_flag.set()
            if force_stop and self.sweep_task: self.sweep_task.join()
            # UI reset is now handled by the worker thread upon exit
        else:
            self.stop_sweep_flag.clear()
            for b_axis, btn in buttons.items(): btn.config(state="disabled" if b_axis != axis else "normal")
            self.ik_go_button.config(state="disabled")
            buttons[axis].config(text="Stop Sweep")
            try: fixed_coords = {'x': float(self.ik_x_var.get()), 'y': float(self.ik_y_var.get()), 'z': float(self.ik_z_var.get())}
            except ValueError: self.status_label.config(text="Error: Invalid IK input for sweep."); self._reset_sweep_ui(axis); return
            self.sweep_task = threading.Thread(target=self._sweep_worker, args=(axis, fixed_coords), daemon=True)
            self.sweep_task.start()

    def _reset_sweep_ui(self, axis):
        buttons = {'x': self.sweep_x_button, 'y': self.sweep_y_button, 'z': self.sweep_z_button}
        for btn in buttons.values(): btn.config(state="normal")
        self.ik_go_button.config(state="normal")
        if axis in buttons: buttons[axis].config(text=f"Sweep {axis.upper()}")
        self.status_label.config(text="Sweep stopped.")
        self.sweep_task = None

    def _sweep_worker(self, axis, fixed_coords):
        self.status_label.config(text=f"Starting {axis.upper()} sweep...")
        axis_map = {'x': 0, 'y': 1, 'z': 2}; sweep_axis_idx = axis_map[axis]
        max_reach = sum(kinematics.LINK_LENGTHS); sweep_range = [-max_reach * 0.8, max_reach * 0.8]
        direction = 1
        current_pos = list(kinematics.forward_kinematics(self.last_ik_solution)[-1]) if self.last_ik_solution else [v for k, v in fixed_coords.items()]

        while not self.stop_sweep_flag.is_set():
            current_pos[sweep_axis_idx] += direction * 1.0
            if not (sweep_range[0] <= current_pos[sweep_axis_idx] <= sweep_range[1]):
                direction *= -1; current_pos[sweep_axis_idx] += direction * 2.0
            target_pos = list(current_pos)
            joint_limits_deg = {mid: (self._pos_to_angle(mid, self.servo_limits[str(mid)]['min']), self._pos_to_angle(mid, self.servo_limits[str(mid)]['max'])) for mid in {1,2,4,5} if str(mid) in self.servo_limits}
            preferred_phi = None
            if self.lock_wrist_var.get():
                try: preferred_phi = float(self.wrist_angle_var.get())
                except ValueError: continue # Skip if angle is invalid
            elif self.last_ik_solution: preferred_phi = np.rad2deg(self.last_ik_solution[3])
            solution_rad = kinematics.inverse_kinematics(target_pos, joint_limits_deg, preferred_phi_deg=preferred_phi, use_locked_angle=self.lock_wrist_var.get())
            if solution_rad:
                self._execute_ik_solution(solution_rad)
                self.command_queue.join()
                time.sleep(0.02)
        self.master.after(0, self._reset_sweep_ui, axis)

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControlGUI(root)
    root.mainloop()