import tkinter as tk
from tkinter import ttk
import serial.tools.list_ports
import numpy as np
import kinematics
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from arm_controller import ArmController

class MotorControlGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Arm Controller")
        self.master.geometry("1200x800")
        self.controller = ArmController(self)
        self.sliders = {}
        self.info_labels = {}
        self.control_widgets = {}
        self._create_ui()
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _create_ui(self):
        main_frame = ttk.Frame(self.master)
        main_frame.pack(fill=tk.BOTH, expand=True)
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)
        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        connection_frame = ttk.LabelFrame(left_frame, text="Connection")
        connection_frame.pack(padx=10, pady=10, fill="x")
        self._create_connection_widgets(connection_frame)

        self.controls_frame = ttk.LabelFrame(left_frame, text="Motor Controls")
        self.controls_frame.pack(padx=10, pady=10, fill="x")
        self.controls_frame.columnconfigure(1, weight=1)

        self.ik_frame = ttk.LabelFrame(left_frame, text="Inverse Kinematics")
        self.ik_frame.pack(padx=10, pady=10, fill="x")
        self._create_ik_widgets(self.ik_frame)

        self.info_frame = ttk.LabelFrame(left_frame, text="Motor Information")
        self.info_frame.pack(padx=10, pady=10, fill="both", expand=True)
        
        vis_frame = ttk.LabelFrame(right_frame, text="Arm Visualization")
        vis_frame.pack(fill=tk.BOTH, expand=True)
        self._create_plot_widgets(vis_frame)

        self.status_label = ttk.Label(self.master, text="Select a COM port and click 'Open & Scan'.")
        self.status_label.pack(side="bottom", fill="x", padx=10, pady=5)

    def _create_connection_widgets(self, parent):
        ttk.Label(parent, text="COM Port:").pack(side="left", padx=5, pady=5)
        self.com_port_var = tk.StringVar()
        self.com_port_menu = ttk.Combobox(parent, textvariable=self.com_port_var, state="readonly")
        self.com_port_menu.pack(side="left", padx=5, pady=5)
        self.refresh_com_ports()
        self.scan_button = ttk.Button(parent, text="Open & Scan", command=self.scan_for_motors)
        self.scan_button.pack(side="left", padx=5, pady=5)
        self.close_button = ttk.Button(parent, text="Close Port", command=self.close_port, state="disabled")
        self.close_button.pack(side="left", padx=5, pady=5)

    def _create_ik_widgets(self, ik_frame):
        ttk.Label(ik_frame, text="X:").grid(row=0, column=0, padx=5, pady=2, sticky='w'); self.ik_x_var = tk.StringVar(value="200"); ttk.Entry(ik_frame, textvariable=self.ik_x_var, width=6).grid(row=0, column=1, padx=0, pady=2)
        self.current_x_label = ttk.Label(ik_frame, text="(Current: ---)", width=18); self.current_x_label.grid(row=1, column=0, columnspan=2, padx=5, pady=2, sticky='w')
        ttk.Label(ik_frame, text="Y:").grid(row=0, column=2, padx=(10,0), pady=2, sticky='w'); self.ik_y_var = tk.StringVar(value="0"); ttk.Entry(ik_frame, textvariable=self.ik_y_var, width=6).grid(row=0, column=3, padx=0, pady=2)
        self.current_y_label = ttk.Label(ik_frame, text="(Current: ---)", width=18); self.current_y_label.grid(row=1, column=2, columnspan=2, padx=5, pady=2, sticky='w')
        ttk.Label(ik_frame, text="Z:").grid(row=0, column=4, padx=(10,0), pady=2, sticky='w'); self.ik_z_var = tk.StringVar(value="150"); ttk.Entry(ik_frame, textvariable=self.ik_z_var, width=6).grid(row=0, column=5, padx=0, pady=2)
        self.current_z_label = ttk.Label(ik_frame, text="(Current: ---)", width=18); self.current_z_label.grid(row=1, column=4, columnspan=2, padx=5, pady=2, sticky='w')
        self.ik_go_button = ttk.Button(ik_frame, text="Go to Position", command=self.go_to_ik_position); self.ik_go_button.config(state="disabled"); self.ik_go_button.grid(row=2, column=0, columnspan=6, pady=5, sticky="ew")
        self.sweep_x_button = ttk.Button(ik_frame, text="Sweep X", command=lambda: self.controller.toggle_sweep('x')); self.sweep_x_button.grid(row=4, column=0, columnspan=2, sticky="ew")
        self.sweep_y_button = ttk.Button(ik_frame, text="Sweep Y", command=lambda: self.controller.toggle_sweep('y')); self.sweep_y_button.grid(row=4, column=2, columnspan=2, sticky="ew")
        self.sweep_z_button = ttk.Button(ik_frame, text="Sweep Z", command=lambda: self.controller.toggle_sweep('z')); self.sweep_z_button.grid(row=4, column=4, columnspan=2, sticky="ew")
        self.sweep_x_button.config(state="disabled"); self.sweep_y_button.config(state="disabled"); self.sweep_z_button.config(state="disabled")
        self.lock_wrist_var = tk.BooleanVar(value=False); self.wrist_angle_var = tk.StringVar(value="-45")
        self.wrist_angle_entry = ttk.Entry(ik_frame, textvariable=self.wrist_angle_var, width=6, state="disabled")
        self.lock_wrist_check = ttk.Checkbutton(ik_frame, text="Lock Wrist Pitch Angle (°):", variable=self.lock_wrist_var, command=self.on_toggle_wrist_lock); self.lock_wrist_check.grid(row=3, column=0, columnspan=4, sticky="w")
        self.wrist_angle_entry.grid(row=3, column=4, columnspan=2)

    def _create_plot_widgets(self, parent):
        self.fig = Figure(figsize=(6, 6), dpi=100); self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent); self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.update_plot(None)

    def refresh_com_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.com_port_menu['values'] = ports
        if ports: self.com_port_var.set(ports[0])

    def scan_for_motors(self):
        port = self.com_port_var.get()
        if not port: self.update_status("Error: No COM port selected."); return
        self.update_status(f"Scanning on {port}..."); 
        self.scan_button.config(text="Stop Scan", command=self.controller.stop_scan)
        for frame in [self.controls_frame, self.info_frame]:
            for widget in frame.winfo_children(): widget.destroy()
        headers = ["ID", "Status", "Position", "Voltage (V)", "Temp (°C)"]
        for i, header in enumerate(headers): ttk.Label(self.info_frame, text=header, font=('TkDefaultFont', 9, 'bold')).grid(row=0, column=i, padx=5, pady=2, sticky='w')
        self.controller.connect(port)

    def on_scan_complete(self, found_motors):
        self.scan_button.config(state="normal")
        if not found_motors: self.update_status(f"Scan finished. No motors found."); return
        self.update_status(f"Scan finished. Found: {found_motors}")
        self.scan_button.config(state="disabled"); self.close_button.config(state="normal"); self.com_port_menu.config(state="disabled")
        self.ik_go_button.config(state="normal"); self.sweep_x_button.config(state="normal"); self.sweep_y_button.config(state="normal"); self.sweep_z_button.config(state="normal")
        self.controller.motor_ids = sorted(found_motors)
        for i, motor_id in enumerate(self.controller.motor_ids):
            self.controller.set_torque(motor_id, 1, force=True)
            self.add_motor_ui(motor_id, i)
        self.controller.start_threads()

    def add_motor_ui(self, motor_id, row_idx):
        info_row = row_idx + 1
        labels = {'id': ttk.Label(self.info_frame, text=str(motor_id)), 'status': ttk.Label(self.info_frame, text="OK", foreground="green"), 'pos': ttk.Label(self.info_frame, text="---"), 'volt': ttk.Label(self.info_frame, text="---"), 'temp': ttk.Label(self.info_frame, text="---")}
        for i, label in enumerate(labels.values()): label.grid(row=info_row, column=i, padx=5, pady=2, sticky='w')
        self.info_labels[motor_id] = labels
        if motor_id == 3: self.info_labels[3]['status'].config(text="Ganged"); return
        motor_limits = self.controller.servo_limits.get(str(motor_id), {'min': 0, 'max': 4095})
        min_pos, max_pos = motor_limits['min'], motor_limits['max']
        start_pos = int((min_pos + max_pos) / 2)
        label_text = f"Motor {motor_id}:" + (" (Shoulder)" if motor_id == 2 else "")
        ttk.Label(self.controls_frame, text=label_text, width=15).grid(row=row_idx, column=0, padx=5, sticky='w')
        slider = ttk.Scale(self.controls_frame, from_=min_pos, to=max_pos, orient='horizontal', command=lambda val, mid=motor_id: self.controller.set_joint_position(mid, int(float(val))))
        slider.set(start_pos); slider.grid(row=row_idx, column=1, padx=5, sticky='ew'); self.sliders[motor_id] = slider
        pos_label = ttk.Label(self.controls_frame, text=str(start_pos), width=5); pos_label.grid(row=row_idx, column=2, padx=5)
        self.control_widgets[motor_id] = {'pos_label': pos_label}
        center_button = ttk.Button(self.controls_frame, text="Center", command=lambda mid=motor_id: self.center_motor(mid)); center_button.grid(row=row_idx, column=3, padx=5)
        self.controller.set_joint_position(motor_id, start_pos)

    def close_port(self):
        self.controller.disconnect()
        for frame in [self.controls_frame, self.info_frame]:
            for widget in frame.winfo_children(): widget.destroy()
        self.scan_button.config(state="normal"); self.close_button.config(state="disabled"); self.com_port_menu.config(state="normal")
        self.ik_go_button.config(state="disabled"); self.sweep_x_button.config(state="disabled"); self.sweep_y_button.config(state="disabled"); self.sweep_z_button.config(state="disabled")
        self.update_status("Port closed. Ready to open again.")

    def go_to_ik_position(self):
        try: x, y, z = self.get_ik_inputs()
        except ValueError: self.update_status("Error: Invalid IK input."); return
        lock_wrist, wrist_angle = self.get_wrist_lock_state()
        if lock_wrist and wrist_angle is None: self.update_status("Error: Invalid wrist angle."); return
        self.controller.go_to_ik_position(x, y, z, lock_wrist, wrist_angle)

    def center_motor(self, motor_id):
        if motor_id in self.sliders:
            center_pos = (self.sliders[motor_id].cget('to') + self.sliders[motor_id].cget('from')) / 2
            self.sliders[motor_id].set(center_pos)
            self.controller.set_joint_position(motor_id, int(center_pos))

    def get_ik_inputs(self):
        return float(self.ik_x_var.get()), float(self.ik_y_var.get()), float(self.ik_z_var.get())

    def get_wrist_lock_state(self):
        if not self.lock_wrist_var.get(): return False, None
        try: return True, float(self.wrist_angle_var.get())
        except ValueError: return True, None

    def update_status(self, msg):
        self.status_label.config(text=msg)

    def update_ui(self, all_motor_data):
        self.update_info_table(all_motor_data)
        self.update_plot(all_motor_data)

    def update_info_table(self, all_motor_data):
        for motor_id, data in all_motor_data.items():
            if motor_id in self.info_labels:
                for key in ['pos', 'volt', 'temp']: self.info_labels[motor_id][key].config(text=f"{data.get(key, '---'):.1f}" if key =='volt' else data.get(key, '---'))
            if motor_id in self.control_widgets: self.control_widgets[motor_id]['pos_label'].config(text=str(data.get('pos', '---')))

    def update_plot(self, all_motor_data):
        q_rad = self.controller.get_kinematic_state(all_motor_data)
        joint_coords = kinematics.forward_kinematics(q_rad)
        self.ax.clear()
        xs, ys, zs = [p[0] for p in joint_coords], [p[1] for p in joint_coords], [p[2] for p in joint_coords]
        self.ax.plot(xs, ys, zs, 'o-', color='dodgerblue', lw=3, markersize=6)
        self.ax.scatter([0], [0], [0], c='black', s=50, marker='s')
        max_reach = sum(kinematics.LINK_LENGTHS); self.ax.set_xlim([-max_reach, max_reach]); self.ax.set_ylim([-max_reach, max_reach]); self.ax.set_zlim([0, max_reach])
        self.ax.set_xlabel('X (mm)'); self.ax.set_ylabel('Y (mm)'); self.ax.set_zlabel('Z (mm)'); self.ax.view_init(elev=30., azim=45)
        tip_pos = joint_coords[-1]; self.current_x_label.config(text=f"(Current: {tip_pos[0]:>6.1f})"); self.current_y_label.config(text=f"(Current: {tip_pos[1]:>6.1f})"); self.current_z_label.config(text=f"(Current: {tip_pos[2]:>6.1f})")
        self.canvas.draw()

    def on_toggle_wrist_lock(self):
        self.wrist_angle_entry.config(state="normal" if self.lock_wrist_var.get() else "disabled")

    def update_sliders(self, move_list, shoulder_target):
        pos_map = {m[0]: m[1] for m in move_list}
        if 1 in self.sliders: self.sliders[1].set(pos_map.get(1,0))
        if 4 in self.sliders: self.sliders[4].set(pos_map.get(4,0))
        if 5 in self.sliders: self.sliders[5].set(pos_map.get(5,0))
        if 2 in self.sliders: self.sliders[2].set(shoulder_target)

    def set_sweep_ui_state(self, axis, is_sweeping):
        buttons = {'x': self.sweep_x_button, 'y': self.sweep_y_button, 'z': self.sweep_z_button}
        for b_axis, btn in buttons.items(): btn.config(state="disabled" if is_sweeping and b_axis != axis else "normal")
        self.ik_go_button.config(state="disabled" if is_sweeping else "normal")
        if is_sweeping:
            buttons[axis].config(text="Stop Sweep")
        else:
            if axis in buttons: buttons[axis].config(text=f"Sweep {axis.upper()}")
            self.update_status("Sweep stopped.")

    def on_closing(self): 
        self.controller.disconnect()
        self.master.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControlGUI(root)
    root.mainloop()
