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
        self.master.title("Jacobian Arm Controller")
        self.master.geometry("1200x800")
        self.controller = ArmController(self)
        self.info_labels = {}
        self.control_widgets = {}
        self.is_closing = False
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

        self.controls_frame = ttk.LabelFrame(left_frame, text="Manual Joint Control")
        self.controls_frame.pack(padx=10, pady=10, fill="x")
        self.controls_frame.columnconfigure(1, weight=1)

        self.ik_frame = ttk.LabelFrame(left_frame, text="Cartesian Control")
        self.ik_frame.pack(padx=10, pady=10, fill="x")
        self._create_ik_widgets(self.ik_frame)

        self.sweep_frame = ttk.LabelFrame(left_frame, text="Sweep Controls")
        self.sweep_frame.pack(padx=10, pady=10, fill="x")
        self._create_sweep_widgets(self.sweep_frame)

        self.position_frame = ttk.LabelFrame(left_frame, text="Current Position")
        self.position_frame.pack(padx=10, pady=10, fill="x")
        self._create_position_display(self.position_frame)

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
        ttk.Label(ik_frame, text="X:").grid(row=0, column=0, padx=5, pady=2, sticky='w'); self.ik_x_var = tk.StringVar(value="200"); ttk.Entry(ik_frame, textvariable=self.ik_x_var, width=8).grid(row=0, column=1, padx=0, pady=2)
        ttk.Label(ik_frame, text="Y:").grid(row=0, column=2, padx=(10,0), pady=2, sticky='w'); self.ik_y_var = tk.StringVar(value="0"); ttk.Entry(ik_frame, textvariable=self.ik_y_var, width=8).grid(row=0, column=3, padx=0, pady=2)
        ttk.Label(ik_frame, text="Z:").grid(row=0, column=4, padx=(10,0), pady=2, sticky='w'); self.ik_z_var = tk.StringVar(value="150"); ttk.Entry(ik_frame, textvariable=self.ik_z_var, width=8).grid(row=0, column=5, padx=0, pady=2)
        self.go_button = ttk.Button(ik_frame, text="Move to Target", command=self.move_to_target); self.go_button.config(state="disabled"); self.go_button.grid(row=1, column=0, columnspan=6, pady=5, sticky="ew")

    def _create_sweep_widgets(self, parent):
        self.sweep_x_button = ttk.Button(parent, text="Sweep X", command=lambda: self.controller.start_sweep('x'))
        self.sweep_x_button.pack(side="left", expand=True, fill='x', padx=2)
        self.sweep_y_button = ttk.Button(parent, text="Sweep Y", command=lambda: self.controller.start_sweep('y'))
        self.sweep_y_button.pack(side="left", expand=True, fill='x', padx=2)
        self.sweep_z_button = ttk.Button(parent, text="Sweep Z", command=lambda: self.controller.start_sweep('z'))
        self.sweep_z_button.pack(side="left", expand=True, fill='x', padx=2)
        self.sweep_x_button.config(state="disabled")
        self.sweep_y_button.config(state="disabled")
        self.sweep_z_button.config(state="disabled")

    def _create_position_display(self, parent):
        self.x_pos_label = ttk.Label(parent, text="X: --- mm")
        self.x_pos_label.pack(anchor='w', padx=5)
        self.y_pos_label = ttk.Label(parent, text="Y: --- mm")
        self.y_pos_label.pack(anchor='w', padx=5)
        self.z_pos_label = ttk.Label(parent, text="Z: --- mm")
        self.z_pos_label.pack(anchor='w', padx=5)

    def _create_plot_widgets(self, parent):
        self.fig = Figure(figsize=(6, 6), dpi=100); self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent); self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.update_plot()

    def refresh_com_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.com_port_menu['values'] = ports
        if "COM3" in ports:
            self.com_port_var.set("COM3")
        elif ports:
            self.com_port_var.set(ports[0])

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
        self.scan_button.config(text="Open & Scan", command=self.scan_for_motors)
        if not found_motors: self.update_status(f"Scan finished. No motors found."); return
        self.update_status(f"Scan finished. Found: {found_motors}")
        self.scan_button.config(state="disabled"); self.close_button.config(state="normal"); self.com_port_menu.config(state="disabled")
        self.go_button.config(state="normal")
        self.sweep_x_button.config(state="normal")
        self.sweep_y_button.config(state="normal")
        self.sweep_z_button.config(state="normal")
        
        self.controller.motor_ids = sorted(found_motors)
        self.controller.initialize_motors()
        
        for i, motor_id in enumerate(self.controller.motor_ids):
            self.add_motor_ui(motor_id, i)

    def add_motor_ui(self, motor_id, row_idx):
        info_row = row_idx + 1
        labels = {'id': ttk.Label(self.info_frame, text=str(motor_id)), 'status': ttk.Label(self.info_frame, text="OK", foreground="green"), 'pos': ttk.Label(self.info_frame, text="---"), 'volt': ttk.Label(self.info_frame, text="---"), 'temp': ttk.Label(self.info_frame, text="---")}
        for i, (key, label) in enumerate(labels.items()):
            col_map = {'id': 0, 'status': 1, 'pos': 2, 'volt': 3, 'temp': 4}
            label.grid(row=info_row, column=col_map[key], padx=5, pady=2, sticky='w')
        self.info_labels[motor_id] = labels

        if motor_id == 3: self.info_labels[3]['status'].config(text="Ganged"); return
        
        label_text = f"Motor {motor_id}:"
        ttk.Label(self.controls_frame, text=label_text).grid(row=row_idx, column=0, padx=5, pady=5, sticky='w')
        
        entry_var = tk.StringVar()
        entry = ttk.Entry(self.controls_frame, textvariable=entry_var, width=10)
        entry.grid(row=row_idx, column=1, padx=5, pady=5, sticky='ew')
        
        go_button = ttk.Button(self.controls_frame, text="Go", command=lambda mid=motor_id, var=entry_var: self.go_to_pos(mid, var))
        go_button.grid(row=row_idx, column=2, padx=5, pady=5)

        torque_button = ttk.Button(self.controls_frame, text="Torque OFF", command=lambda mid=motor_id: self.toggle_torque(mid))
        torque_button.grid(row=row_idx, column=3, padx=5, pady=5)
        
        self.control_widgets[motor_id] = {'entry': entry, 'go_button': go_button, 'torque_button': torque_button}

    def go_to_pos(self, motor_id, entry_var):
        try:
            target_pos = int(entry_var.get())
            self.controller.set_joint_position(motor_id, target_pos)
        except ValueError:
            self.update_status(f"Error: Invalid position for Motor {motor_id}")

    def move_to_target(self):
        if self.controller.is_moving():
            self.controller.stop_all_motion()
            return
        try:
            target_pos = np.array([float(self.ik_x_var.get()), float(self.ik_y_var.get()), float(self.ik_z_var.get())])
            self.controller.move_to_target(target_pos)
        except ValueError:
            self.update_status("Error: Invalid IK input.")

    def close_port(self):
        self.controller.disconnect()
        for frame in [self.controls_frame, self.info_frame]:
            for widget in frame.winfo_children(): widget.destroy()
        self.scan_button.config(state="normal"); self.close_button.config(state="disabled"); self.com_port_menu.config(state="normal")
        self.go_button.config(state="disabled")
        self.sweep_x_button.config(state="disabled")
        self.sweep_y_button.config(state="disabled")
        self.sweep_z_button.config(state="disabled")
        self.update_status("Port closed. Ready to open again.")

    def toggle_torque(self, motor_id):
        button = self.control_widgets[motor_id]['torque_button']
        current_text = button.cget('text')
        new_state = 0 if current_text == "Torque OFF" else 1
        button.config(text=f"Torque {'ON' if new_state == 0 else 'OFF'}")
        self.controller.set_torque(motor_id, new_state)

    def update_status(self, msg):
        self.status_label.config(text=msg)

    def update_ui(self, all_motor_data):
        if self.is_closing: return
        try:
            if not self.master.winfo_exists(): return
        except: return

        for motor_id, data in all_motor_data.items():
            if motor_id in self.info_labels:
                try:
                    self.info_labels[motor_id]['pos'].config(text=str(data.get('pos', '---')))
                    volt_val = data.get('volt', '---')
                    temp_val = data.get('temp', '---')
                    self.info_labels[motor_id]['volt'].config(text=f"{volt_val:.1f}" if isinstance(volt_val, (int, float)) else str(volt_val))
                    self.info_labels[motor_id]['temp'].config(text=f"{temp_val:.1f}" if isinstance(temp_val, (int, float)) else str(temp_val))
                except tk.TclError: pass
        self.update_plot()

    def update_plot(self):
        if self.is_closing: return
        try:
            if not self.master.winfo_exists(): return
        except: return
        
        try:
            q_rad = self.controller.current_q_rad
            joint_coords = kinematics.forward_kinematics(q_rad)
            
            current_pos = joint_coords[-1]
            self.x_pos_label.config(text=f"X: {current_pos[0]:.2f} mm")
            self.y_pos_label.config(text=f"Y: {current_pos[1]:.2f} mm")
            self.z_pos_label.config(text=f"Z: {current_pos[2]:.2f} mm")

            self.ax.clear()
            # The first point (p0) is the floor origin, the second (p1) is the shoulder.
            # We draw the static base link from floor to shoulder.
            self.ax.plot([joint_coords[0][0], joint_coords[1][0]], 
                        [joint_coords[0][1], joint_coords[1][1]], 
                        [joint_coords[0][2], joint_coords[1][2]], 
                        'o-', color='grey', lw=3, markersize=6)
            # Then, draw the moving part of the arm from the shoulder onwards.
            self.ax.plot([p[0] for p in joint_coords[1:]], 
                        [p[1] for p in joint_coords[1:]], 
                        [p[2] for p in joint_coords[1:]], 
                        'o-', color='dodgerblue', lw=3, markersize=6)

            self.ax.scatter([0], [0], [0], c='red', s=100, marker='X', label='Origin')
            max_reach = sum(kinematics.LINK_LENGTHS); self.ax.set_xlim([-max_reach, max_reach]); self.ax.set_ylim([-max_reach, max_reach]); self.ax.set_zlim([0, max_reach])
            self.ax.set_xlabel('X (mm)'); self.ax.set_ylabel('Y (mm)'); self.ax.set_zlabel('Z (mm)'); self.ax.view_init(elev=30., azim=45)
            self.canvas.draw()
        except tk.TclError: pass

    def set_motion_ui_state(self, is_moving):
        button_text = "Stop Motion" if is_moving else "Move to Target"
        sweep_state = "disabled" if is_moving else "normal"
        
        self.go_button.config(text=button_text)
        self.sweep_x_button.config(state=sweep_state)
        self.sweep_y_button.config(state=sweep_state)
        self.sweep_z_button.config(state=sweep_state)

        for motor_id, widgets in self.control_widgets.items():
            state = "disabled" if is_moving else "normal"
            widgets['entry'].config(state=state)
            widgets['go_button'].config(state=state)

    def on_closing(self): 
        self.is_closing = True
        self.controller.disconnect()
        self.master.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControlGUI(root)
    root.mainloop()