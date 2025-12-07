import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import threading
import time
import queue
import math
import numpy as np
import json
import os

# Matplotlib for 3D visualization
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

# Link lengths (mm) - must match firmware
L1 = 133.39  # Shoulder to Elbow
L2 = 124.97  # Elbow to Wrist
L3 = 73.39   # Wrist to Tip

# Config file path (same directory as script)
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_config.json")

# Default PID values
DEFAULT_PID = {
    "1": {"kp": 8.0, "ki": 0.0, "kd": 0.8},   # Base
    "2": {"kp": 12.0, "ki": 0.0, "kd": 1.0},  # Shoulder
    "3": {"kp": 12.0, "ki": 0.0, "kd": 1.0},  # Elbow
    "4": {"kp": 10.0, "ki": 0.0, "kd": 0.6},  # Wrist Pitch
    "5": {"kp": 8.0, "ki": 0.0, "kd": 0.4}    # Wrist Roll
}

class ArmGUI:
    def __init__(self, root, port='COM6', baud=921600):
        self.root = root
        self.root.title("5DOF Arm Controller")
        self.root.state('zoomed')  # Maximize window by default
        
        self.ser = None
        self.port = port
        self.baud = baud
        self.connected = False
        self.msg_queue = queue.Queue()
        
        # Current joint angles (radians) for visualization
        self.current_joints = [0.0, 0.0, 0.0, 0.0, 0.0]  # q1, q2, q3, q4, roll
        self.current_pos = [0.0, 0.0, 331.75]  # x, y, z
        
        # Position programming variables
        self.program_positions = []  # List of {x, y, z, roll, wait}
        self.program_running = False
        self.program_loop = False
        self.program_current_idx = 0
        self.program_waiting = False
        self.program_wait_start = 0
        
        # Load saved configuration
        self.config = self.load_config()
        
        self.create_widgets()
        self.connect_serial()
        
        # Start UI update loop
        self.root.after(10, self.process_queue)
        
        # Start visualization update loop (slower rate)
        self.root.after(100, self.update_visualization)
    
    def load_config(self):
        """Load configuration from JSON file"""
        try:
            if os.path.exists(CONFIG_FILE):
                with open(CONFIG_FILE, 'r') as f:
                    config = json.load(f)
                    print(f"Loaded config from {CONFIG_FILE}")
                    return config
        except Exception as e:
            print(f"Error loading config: {e}")
        
        # Return default config
        return {"pid": DEFAULT_PID.copy()}
    
    def save_config(self):
        """Save configuration to JSON file"""
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump(self.config, f, indent=2)
            print(f"Config saved to {CONFIG_FILE}")
        except Exception as e:
            print(f"Error saving config: {e}")
    
    def send_all_pid_to_esp32(self):
        """Send all saved PID values to ESP32"""
        if not self.connected:
            return
        
        print("Sending saved PID values to ESP32...")
        for joint_id, gains in self.config.get("pid", DEFAULT_PID).items():
            kp = gains.get("kp", 10.0)
            ki = gains.get("ki", 0.0)
            kd = gains.get("kd", 0.5)
            cmd = f"P {joint_id} {kp} {ki} {kd}\n"
            self.ser.write(cmd.encode())
            print(f"  Sent: P {joint_id} {kp} {ki} {kd}")
            time.sleep(0.05)  # Small delay between commands
        print("All PID values sent!")

    def forward_kinematics(self, q):
        """Calculate FK - returns joint positions for visualization"""
        q1, q2, q3, q4 = q[0], q[1], q[2], q[3]
        
        # Apply 90° offset to shoulder (same as firmware)
        q2_kin = q2 + math.pi / 2
        
        c1, s1 = math.cos(q1), math.sin(q1)
        c2, s2 = math.cos(q2_kin), math.sin(q2_kin)
        c23, s23 = math.cos(q2_kin + q3), math.sin(q2_kin + q3)
        c234, s234 = math.cos(q2_kin + q3 + q4), math.sin(q2_kin + q3 + q4)
        
        # Joint positions
        positions = []
        
        # p0 - Base origin
        positions.append([0, 0, 0])
        
        # p1 - Same as base (shoulder rotation point)
        positions.append([0, 0, 0])
        
        # p2 - After L1 (elbow)
        r2 = L1 * c2
        z2 = L1 * s2
        positions.append([r2 * c1, r2 * s1, z2])
        
        # p3 - After L2 (wrist)
        r3 = L1 * c2 + L2 * c23
        z3 = L1 * s2 + L2 * s23
        positions.append([r3 * c1, r3 * s1, z3])
        
        # p4 - End effector (after L3)
        r4 = L1 * c2 + L2 * c23 + L3 * c234
        z4 = L1 * s2 + L2 * s23 + L3 * s234
        positions.append([r4 * c1, r4 * s1, z4])
        
        return positions

    def create_widgets(self):
        # Main horizontal paned window
        main_pane = ttk.PanedWindow(self.root, orient='horizontal')
        main_pane.pack(fill='both', expand=True)
        
        # Left panel - Controls (with scrollbar)
        left_container = ttk.Frame(main_pane)
        main_pane.add(left_container, weight=1)
        
        # Add canvas and scrollbar for left panel
        left_canvas = tk.Canvas(left_container)
        left_scrollbar = ttk.Scrollbar(left_container, orient="vertical", command=left_canvas.yview)
        left_frame = ttk.Frame(left_canvas)
        
        left_frame.bind("<Configure>", lambda e: left_canvas.configure(scrollregion=left_canvas.bbox("all")))
        left_canvas.create_window((0, 0), window=left_frame, anchor="nw")
        left_canvas.configure(yscrollcommand=left_scrollbar.set)
        
        left_scrollbar.pack(side="right", fill="y")
        left_canvas.pack(side="left", fill="both", expand=True)
        
        # Enable mousewheel scrolling
        def on_mousewheel(event):
            left_canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        left_canvas.bind_all("<MouseWheel>", on_mousewheel)
        
        # Right panel - 3D Visualization
        right_frame = ttk.Frame(main_pane, width=500)
        main_pane.add(right_frame, weight=1)
        
        # ========== LEFT PANEL - CONTROLS ==========
        
        # Connection Status
        self.status_frame = ttk.Frame(left_frame, padding=5)
        self.status_frame.pack(fill='x')
        self.lbl_status = ttk.Label(self.status_frame, text="Disconnected", foreground="red")
        self.lbl_status.pack(side='left')
        
        # Telemetry Display - Cartesian Position
        self.telemetry_frame = ttk.LabelFrame(left_frame, text="Cartesian Position (mm, rad)", padding=5)
        self.telemetry_frame.pack(fill='x', padx=5, pady=2)
        
        self.vars_pos = [tk.StringVar(value="0.00") for _ in range(4)]
        labels = ["X", "Y", "Z", "Roll"]
        for i, lbl in enumerate(labels):
            f = ttk.Frame(self.telemetry_frame)
            f.pack(side='left', expand=True)
            ttk.Label(f, text=lbl).pack()
            ttk.Label(f, textvariable=self.vars_pos[i], font=('Consolas', 12)).pack()

        # Raw Motor Positions Display
        self.raw_frame = ttk.LabelFrame(left_frame, text="Raw Motor Positions (0-4095)", padding=5)
        self.raw_frame.pack(fill='x', padx=5, pady=2)
        
        self.vars_raw = [tk.StringVar(value="2048") for _ in range(6)]
        motor_labels = ["M1", "M2", "M3", "M4", "M5", "M6"]
        for i, lbl in enumerate(motor_labels):
            f = ttk.Frame(self.raw_frame)
            f.pack(side='left', expand=True)
            ttk.Label(f, text=lbl, font=('Consolas', 8)).pack()
            ttk.Label(f, textvariable=self.vars_raw[i], font=('Consolas', 10, 'bold')).pack()

        # Control Panel
        self.control_frame = ttk.LabelFrame(left_frame, text="Cartesian Control", padding=5)
        self.control_frame.pack(fill='x', padx=5, pady=2)
        
        # Target Inputs
        input_frame = ttk.Frame(self.control_frame)
        input_frame.pack(fill='x', pady=2)
        
        self.entries_target = []
        target_labels = ["X", "Y", "Z", "Roll"]
        defaults = ["200", "0", "200", "0"]
        for i, lbl in enumerate(target_labels):
            f = ttk.Frame(input_frame)
            f.pack(side='left', padx=3)
            ttk.Label(f, text=lbl).pack()
            e = ttk.Entry(f, width=7)
            e.insert(0, defaults[i])
            e.pack()
            self.entries_target.append(e)
        
        btn_frame = ttk.Frame(self.control_frame)
        btn_frame.pack(fill='x', pady=5)
        ttk.Button(btn_frame, text="MOVE", command=self.send_move_cmd).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="STOP", command=self.stop_movement).pack(side='left', padx=5)
        
        # Move status label for feedback (reachability, errors, etc.)
        self.lbl_move_status = ttk.Label(btn_frame, text="", foreground="gray")
        self.lbl_move_status.pack(side='left', padx=10)
        
        # Speed multiplier and Wrist control
        speed_wrist_frame = ttk.Frame(self.control_frame)
        speed_wrist_frame.pack(fill='x', pady=2)
        
        ttk.Label(speed_wrist_frame, text="Speed:").pack(side='left')
        self.speed_var = tk.StringVar(value="2.0")
        self.speed_slider = ttk.Scale(speed_wrist_frame, from_=0.2, to=3.0, orient='horizontal',
                                       command=self.on_speed_change)
        self.speed_slider.set(2.0)
        self.speed_slider.pack(side='left', padx=3)
        ttk.Label(speed_wrist_frame, textvariable=self.speed_var, width=4).pack(side='left')
        
        ttk.Separator(speed_wrist_frame, orient='vertical').pack(side='left', padx=10, fill='y')
        
        self.wrist_locked = tk.BooleanVar(value=True)
        self.chk_wrist = ttk.Checkbutton(speed_wrist_frame, text="Wrist Locked", 
                                          variable=self.wrist_locked, command=self.on_wrist_toggle)
        self.chk_wrist.pack(side='left', padx=5)
        
        ttk.Label(speed_wrist_frame, text="Angle:").pack(side='left')
        self.ent_wrist_angle = ttk.Entry(speed_wrist_frame, width=5)
        self.ent_wrist_angle.insert(0, "0")
        self.ent_wrist_angle.pack(side='left', padx=2)
        ttk.Button(speed_wrist_frame, text="Set", command=self.send_wrist_angle).pack(side='left')

        # Joint Control - Text entries instead of sliders
        joint_frame = ttk.LabelFrame(left_frame, text="Joint Control (Degrees) - Enter angle or leave blank", padding=5)
        joint_frame.pack(fill='x', padx=5, pady=2)
        
        self.entries_joint = []  # Text entries for target angles
        self.vars_joint = [tk.StringVar(value="") for _ in range(5)]  # Display current angle
        joint_names = ["Base", "Shoulder", "Elbow", "WristP", "Roll"]
        
        joint_row = ttk.Frame(joint_frame)
        joint_row.pack(fill='x', pady=2)
        
        for i in range(5):
            f = ttk.Frame(joint_row)
            f.pack(side='left', padx=5, expand=True)
            ttk.Label(f, text=joint_names[i], font=('Arial', 9, 'bold')).pack()
            
            # Current angle display
            curr_f = ttk.Frame(f)
            curr_f.pack()
            ttk.Label(curr_f, text="Now:", font=('Arial', 8)).pack(side='left')
            ttk.Label(curr_f, textvariable=self.vars_joint[i], width=6, font=('Consolas', 9)).pack(side='left')
            
            # Target angle entry
            tgt_f = ttk.Frame(f)
            tgt_f.pack()
            ttk.Label(tgt_f, text="Go:", font=('Arial', 8)).pack(side='left')
            e = ttk.Entry(tgt_f, width=6)
            e.pack(side='left')
            self.entries_joint.append(e)
        
        # Joint control buttons
        joint_btn_frame = ttk.Frame(joint_frame)
        joint_btn_frame.pack(fill='x', pady=2)
        ttk.Button(joint_btn_frame, text="Move Joints", command=self.send_joint_targets).pack(side='left', padx=5)
        ttk.Button(joint_btn_frame, text="Clear All", command=self.clear_joint_entries).pack(side='left', padx=5)

        # PID Tuning
        pid_frame = ttk.LabelFrame(left_frame, text="PID Tuning", padding=5)
        pid_frame.pack(fill='x', padx=5, pady=2)
        
        # Get default values from saved config
        default_joint = "1"
        saved_pid = self.config.get("pid", DEFAULT_PID).get(default_joint, DEFAULT_PID["1"])
        
        f = ttk.Frame(pid_frame)
        f.pack(fill='x')
        ttk.Label(f, text="Joint:").pack(side='left')
        self.ent_pid_id = ttk.Entry(f, width=3)
        self.ent_pid_id.insert(0, "1")
        self.ent_pid_id.pack(side='left', padx=2)
        self.ent_pid_id.bind('<FocusOut>', self.on_pid_joint_change)
        self.ent_pid_id.bind('<Return>', self.on_pid_joint_change)
        
        ttk.Label(f, text="Kp:").pack(side='left')
        self.ent_kp = ttk.Entry(f, width=6)
        self.ent_kp.insert(0, str(saved_pid.get("kp", 10.0)))
        self.ent_kp.pack(side='left', padx=2)
        
        ttk.Label(f, text="Ki:").pack(side='left')
        self.ent_ki = ttk.Entry(f, width=6)
        self.ent_ki.insert(0, str(saved_pid.get("ki", 0.0)))
        self.ent_ki.pack(side='left', padx=2)
        
        ttk.Label(f, text="Kd:").pack(side='left')
        self.ent_kd = ttk.Entry(f, width=6)
        self.ent_kd.insert(0, str(saved_pid.get("kd", 0.5)))
        self.ent_kd.pack(side='left', padx=2)
        
        ttk.Button(f, text="Set & Save", command=self.send_and_save_pid).pack(side='left', padx=5)
        
        # Second row for Load All / Save All buttons
        f2 = ttk.Frame(pid_frame)
        f2.pack(fill='x', pady=3)
        ttk.Button(f2, text="Send All to ESP32", command=self.send_all_pid_to_esp32).pack(side='left', padx=3)
        ttk.Button(f2, text="Load from File", command=self.load_pid_from_file).pack(side='left', padx=3)
        self.lbl_pid_status = ttk.Label(f2, text="", foreground="gray")
        self.lbl_pid_status.pack(side='left', padx=5)

        # Advanced Features
        adv_frame = ttk.LabelFrame(left_frame, text="Sweep & Control", padding=5)
        adv_frame.pack(fill='x', padx=5, pady=2)
        
        sweep_row = ttk.Frame(adv_frame)
        sweep_row.pack(fill='x')
        ttk.Button(sweep_row, text="SWEEP X", command=lambda: self.start_sweep('x')).pack(side='left', padx=3)
        ttk.Button(sweep_row, text="SWEEP Y", command=lambda: self.start_sweep('y')).pack(side='left', padx=3)
        ttk.Button(sweep_row, text="SWEEP Z", command=lambda: self.start_sweep('z')).pack(side='left', padx=3)
        
        ctrl_row = ttk.Frame(adv_frame)
        ctrl_row.pack(fill='x', pady=3)
        self.btn_torque = ttk.Button(ctrl_row, text="TORQUE OFF", command=self.toggle_torque)
        self.btn_torque.pack(side='left', padx=3)
        self.torque_enabled = True
        
        # ========== POSITION PROGRAMMING ==========
        prog_frame = ttk.LabelFrame(left_frame, text="Position Programming", padding=5)
        prog_frame.pack(fill='x', padx=5, pady=2)
        
        # Position list with scrollbar
        list_frame = ttk.Frame(prog_frame)
        list_frame.pack(fill='x', pady=2)
        
        self.pos_listbox = tk.Listbox(list_frame, height=6, width=50, font=('Courier', 9))
        scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=self.pos_listbox.yview)
        self.pos_listbox.configure(yscrollcommand=scrollbar.set)
        self.pos_listbox.pack(side='left', fill='x', expand=True)
        scrollbar.pack(side='right', fill='y')
        
        # Add position controls
        add_frame = ttk.Frame(prog_frame)
        add_frame.pack(fill='x', pady=2)
        
        ttk.Button(add_frame, text="Add Current Pos", command=self.prog_add_current).pack(side='left', padx=2)
        ttk.Button(add_frame, text="Add Target", command=self.prog_add_target).pack(side='left', padx=2)
        ttk.Label(add_frame, text="Wait(s):").pack(side='left', padx=(10,2))
        self.ent_wait_time = ttk.Entry(add_frame, width=5)
        self.ent_wait_time.insert(0, "0")
        self.ent_wait_time.pack(side='left')
        
        # Edit controls
        edit_frame = ttk.Frame(prog_frame)
        edit_frame.pack(fill='x', pady=2)
        
        ttk.Button(edit_frame, text="Move Up", command=self.prog_move_up).pack(side='left', padx=2)
        ttk.Button(edit_frame, text="Move Down", command=self.prog_move_down).pack(side='left', padx=2)
        ttk.Button(edit_frame, text="Edit", command=self.prog_edit_selected).pack(side='left', padx=2)
        ttk.Button(edit_frame, text="Delete", command=self.prog_delete).pack(side='left', padx=2)
        ttk.Button(edit_frame, text="Clear All", command=self.prog_clear).pack(side='left', padx=2)
        ttk.Button(edit_frame, text="Go To", command=self.prog_goto_selected).pack(side='left', padx=2)
        
        # Motion functions frame
        func_frame = ttk.Frame(prog_frame)
        func_frame.pack(fill='x', pady=2)
        
        ttk.Label(func_frame, text="Add Pattern:").pack(side='left', padx=2)
        self.pattern_var = tk.StringVar(value="Circle XY")
        pattern_combo = ttk.Combobox(func_frame, textvariable=self.pattern_var, width=12, state='readonly')
        pattern_combo['values'] = ('Circle XY', 'Circle XZ', 'Circle YZ', 'Square XY', 'Triangle XY', 'Line X', 'Line Y', 'Line Z', 'Wave XZ')
        pattern_combo.pack(side='left', padx=2)
        
        ttk.Label(func_frame, text="Size:").pack(side='left', padx=2)
        self.ent_pattern_size = ttk.Entry(func_frame, width=5)
        self.ent_pattern_size.insert(0, "50")
        self.ent_pattern_size.pack(side='left')
        
        ttk.Label(func_frame, text="Pts:").pack(side='left', padx=2)
        self.ent_pattern_points = ttk.Entry(func_frame, width=4)
        self.ent_pattern_points.insert(0, "12")
        self.ent_pattern_points.pack(side='left')
        
        ttk.Button(func_frame, text="Add", command=self.prog_add_pattern).pack(side='left', padx=5)
        
        # Presets frame
        preset_frame = ttk.Frame(prog_frame)
        preset_frame.pack(fill='x', pady=2)
        
        ttk.Label(preset_frame, text="Presets:").pack(side='left', padx=2)
        self.preset_var = tk.StringVar(value="Pick & Place")
        preset_combo = ttk.Combobox(preset_frame, textvariable=self.preset_var, width=15, state='readonly')
        preset_combo['values'] = ('Pick & Place', 'Home Position', 'Scan Area', 'Wave Hello', 'Demo Routine', 'Touch Corners')
        preset_combo.pack(side='left', padx=2)
        ttk.Button(preset_frame, text="Load Preset", command=self.prog_load_preset).pack(side='left', padx=5)
        
        # Run controls
        run_frame = ttk.Frame(prog_frame)
        run_frame.pack(fill='x', pady=2)
        
        self.btn_prog_run = ttk.Button(run_frame, text="▶ RUN", command=self.prog_run)
        self.btn_prog_run.pack(side='left', padx=2)
        ttk.Button(run_frame, text="■ STOP", command=self.prog_stop).pack(side='left', padx=2)
        
        self.prog_loop_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(run_frame, text="Loop", variable=self.prog_loop_var).pack(side='left', padx=10)
        
        self.lbl_prog_status = ttk.Label(run_frame, text="Stopped", foreground="gray")
        self.lbl_prog_status.pack(side='left', padx=10)
        
        # Save/Load controls
        file_frame = ttk.Frame(prog_frame)
        file_frame.pack(fill='x', pady=2)
        
        ttk.Button(file_frame, text="Save Program", command=self.prog_save).pack(side='left', padx=2)
        ttk.Button(file_frame, text="Load Program", command=self.prog_load).pack(side='left', padx=2)
        
        # ========== RIGHT PANEL - 3D VISUALIZATION ==========
        
        viz_frame = ttk.LabelFrame(right_frame, text="3D Arm Visualization", padding=5)
        viz_frame.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Create matplotlib figure
        self.fig = Figure(figsize=(6, 6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Embed in Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=viz_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
        
        # Initialize the plot
        self.init_3d_plot()

    def init_3d_plot(self):
        """Initialize the 3D plot with arm structure"""
        self.ax.clear()
        
        # Set axis limits (mm)
        limit = 400
        self.ax.set_xlim([-limit, limit])
        self.ax.set_ylim([-limit, limit])
        self.ax.set_zlim([0, limit])
        
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        self.ax.set_title('Arm Posture')
        
        # Draw arm at initial position
        self.update_arm_plot()

    def update_arm_plot(self):
        """Update the 3D arm visualization"""
        self.ax.clear()
        
        # Set axis limits
        limit = 400
        self.ax.set_xlim([-limit, limit])
        self.ax.set_ylim([-limit, limit])
        self.ax.set_zlim([0, limit])
        
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        
        # Calculate joint positions from current joints
        positions = self.forward_kinematics(self.current_joints)
        
        # Extract coordinates
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        zs = [p[2] for p in positions]
        
        # Draw arm links
        self.ax.plot(xs, ys, zs, 'b-', linewidth=3, marker='o', markersize=8, 
                     markerfacecolor='red', label='Arm')
        
        # Draw base platform (circle)
        theta = np.linspace(0, 2*np.pi, 30)
        base_r = 40
        self.ax.plot(base_r * np.cos(theta), base_r * np.sin(theta), 
                     np.zeros_like(theta), 'g-', linewidth=2)
        
        # Draw end effector point
        self.ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], c='green', s=100, marker='^', label='End Effector')
        
        # Show current position text
        pos_text = f"Pos: ({xs[-1]:.1f}, {ys[-1]:.1f}, {zs[-1]:.1f})"
        self.ax.set_title(pos_text)
        
        # Draw target if we have one
        try:
            tx = float(self.entries_target[0].get())
            ty = float(self.entries_target[1].get())
            tz = float(self.entries_target[2].get())
            self.ax.scatter([tx], [ty], [tz], c='orange', s=150, marker='*', label='Target')
        except:
            pass
        
        self.ax.legend(loc='upper left', fontsize=8)
        self.canvas.draw_idle()

    def update_visualization(self):
        """Periodic update of 3D visualization"""
        self.update_arm_plot()
        self.root.after(100, self.update_visualization)  # 10 Hz update

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.connected = True
            self.lbl_status.config(text=f"Connected to {self.port} @ {self.baud}", foreground="green")
            threading.Thread(target=self.read_serial, daemon=True).start()
            
            # Auto-send saved settings to ESP32 after connection
            self.root.after(500, self.send_all_pid_to_esp32)
            self.root.after(600, self.sync_settings_to_esp32)
        except Exception as e:
            self.lbl_status.config(text=f"Error: {e}", foreground="red")
    
    def sync_settings_to_esp32(self):
        """Send current speed and wrist settings to ESP32"""
        if not self.connected: return
        # Send speed multiplier
        speed = self.speed_slider.get()
        self.ser.write(f"SP {speed:.2f}\n".encode())
        # Send wrist lock state
        lock_val = 1 if self.wrist_locked.get() else 0
        self.ser.write(f"WL {lock_val}\n".encode())
        print("Synced speed and wrist settings")

    def read_serial(self):
        while self.connected:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        self.msg_queue.put(line)
            except:
                break
            time.sleep(0.005)

    def process_queue(self):
        while not self.msg_queue.empty():
            msg = self.msg_queue.get()
            if msg.startswith("POS"):
                try:
                    if "|" in msg:
                        parts = msg.split("|")
                        if len(parts) < 2:
                            # Corrupted message, skip it
                            continue
                        pos_part = parts[0]
                        raw_part = parts[1]
                        # FREQ part is optional (parts[2] if exists)
                        
                        # Parse position: "POS x y z roll"
                        pos_parts = pos_part.strip().split()
                        if len(pos_parts) >= 5:
                            for i in range(4):
                                self.vars_pos[i].set(pos_parts[i+1])
                            # Update current position for visualization
                            self.current_pos = [float(pos_parts[1]), float(pos_parts[2]), float(pos_parts[3])]
                        
                        # Parse raw: "RAW m1 m2 m3 m4 m5 m6"
                        raw_parts = raw_part.strip().split()
                        if len(raw_parts) >= 7:
                            for i in range(6):
                                self.vars_raw[i].set(raw_parts[i+1])
                            
                            # Convert raw to joint angles for visualization
                            # Physical arm layout (must match firmware):
                            # - Motor 2 > center = +X direction -> NEGATE
                            # - Motor 4 < center = +X direction -> NOT negated (negative raw = +X)
                            # - Motor 5 < center = +X direction -> NOT negated (negative raw = +X)
                            centers = [2207, 2617, 2771, 2563, 2160, 2047]
                            raw_vals = [int(raw_parts[i+1]) for i in range(6)]
                            
                            # Joint 0 (Base) = Motor 1 - not negated
                            self.current_joints[0] = (raw_vals[0] - centers[0]) * 2 * math.pi / 4096
                            # Joint 1 (Shoulder) = Motor 2 - NEGATED
                            self.current_joints[1] = -(raw_vals[1] - centers[1]) * 2 * math.pi / 4096
                            # Joint 2 (Elbow) = Motor 4 (index 3) - NOT negated
                            self.current_joints[2] = (raw_vals[3] - centers[3]) * 2 * math.pi / 4096
                            # Joint 3 (Wrist Pitch) = Motor 5 (index 4) - NOT negated
                            self.current_joints[3] = (raw_vals[4] - centers[4]) * 2 * math.pi / 4096
                            # Roll = Motor 6 (index 5) - not negated
                            self.current_joints[4] = (raw_vals[5] - centers[5]) * 2 * math.pi / 4096
                            
                            # Update joint angle displays
                            for i in range(5):
                                deg = math.degrees(self.current_joints[i])
                                self.vars_joint[i].set(f"{deg:.1f}")
                                
                except Exception as e:
                    print(f"Parse error: {e}")
            elif msg.startswith("JOINTS"):
                print(f"[Joints]: {msg}")
            elif msg.startswith("UNREACHABLE"):
                # Show unreachable error prominently
                reason = msg.replace("UNREACHABLE:", "").strip()
                self.lbl_move_status.config(text=f"Can't reach: {reason}", foreground="red")
                print(f"[ESP32]: {msg}")
            elif msg.startswith("OK") or msg.startswith("ERR") or msg.startswith("DONE"):
                print(f"[ESP32]: {msg}")
                # Clear error on successful move start
                if msg.startswith("OK Moving"):
                    self.lbl_move_status.config(text="Moving...", foreground="green")
                elif msg.startswith("DONE"):
                    self.lbl_move_status.config(text="Done", foreground="green")
            elif msg.startswith("STATS"):
                pass  # Suppress stats spam
            elif "PATH_DONE" in msg:
                print(f"[ESP32]: {msg}")
                # Signal path completion for loop handling
                if self.program_running:
                    self.path_done_received = True
            elif msg.startswith("PATH"):
                print(f"[ESP32]: {msg}")
            else:
                print(f"[ESP32]: {msg}")
        
        self.root.after(20, self.process_queue)

    def send_move_cmd(self):
        if not self.connected: return
        try:
            vals = [e.get() for e in self.entries_target]
            # M x y z roll time - use fixed 2s duration, speed is controlled via multiplier
            cmd = f"M {vals[0]} {vals[1]} {vals[2]} {vals[3]} 2.0\n"
            self.ser.write(cmd.encode())
            print(f"Sent: {cmd.strip()}")
        except Exception as e:
            print(f"Invalid Input: {e}")
    
    def send_wrist_angle(self):
        if not self.connected: return
        try:
            angle = self.ent_wrist_angle.get()
            cmd = f"W {angle}\n"
            self.ser.write(cmd.encode())
            print(f"Sent: {cmd.strip()}")
        except Exception as e:
            print(f"Invalid Input: {e}")
    
    def on_speed_change(self, val):
        """Update speed multiplier"""
        speed = float(val)
        self.speed_var.set(f"{speed:.1f}")
        if self.connected:
            cmd = f"SP {speed:.2f}\n"
            self.ser.write(cmd.encode())
            print(f"Sent: {cmd.strip()}")
    
    def on_wrist_toggle(self):
        """Toggle wrist locked/free mode"""
        if self.connected:
            lock_val = 1 if self.wrist_locked.get() else 0
            cmd = f"WL {lock_val}\n"
            self.ser.write(cmd.encode())
            print(f"Sent: {cmd.strip()}")
    
    def send_joint_angles(self):
        """Send joint angles from entry boxes. Empty entries use current position."""
        if not self.connected:
            return
        try:
            # Get target angles - use current position for empty entries
            target_rads = []
            for i, entry in enumerate(self.entries_joint):
                val = entry.get().strip()
                if val == "":
                    # Use current position from telemetry (vars_joint stores deg as "XX.X")
                    current_deg = float(self.vars_joint[i].get())
                    target_rads.append(current_deg * (math.pi / 180.0))
                else:
                    target_rads.append(float(val) * (math.pi / 180.0))
            
            cmd = f"D {target_rads[0]:.4f} {target_rads[1]:.4f} {target_rads[2]:.4f} {target_rads[3]:.4f} {target_rads[4]:.4f}\n"
            self.ser.write(cmd.encode())
            print(f"Sent: {cmd.strip()}")
        except Exception as e:
            print(f"Error sending joint angles: {e}")
    
    # Alias for button
    def send_joint_targets(self):
        """Alias for send_joint_angles() - called by Move Joints button"""
        self.send_joint_angles()
    
    def clear_joint_entries(self):
        """Clear all joint entry boxes"""
        for entry in self.entries_joint:
            entry.delete(0, tk.END)

    def send_pid_cmd(self):
        """Send PID command to ESP32 (without saving)"""
        if not self.connected: return
        try:
            mid = self.ent_pid_id.get()
            kp = self.ent_kp.get()
            ki = self.ent_ki.get()
            kd = self.ent_kd.get()
            cmd = f"P {mid} {kp} {ki} {kd}\n"
            self.ser.write(cmd.encode())
            print(f"Sent: {cmd.strip()}")
        except:
            pass
    
    def send_and_save_pid(self):
        """Send PID command to ESP32 AND save to local config file"""
        if not self.connected: 
            self.lbl_pid_status.config(text="Not connected!", foreground="red")
            return
        try:
            mid = self.ent_pid_id.get()
            kp = float(self.ent_kp.get())
            ki = float(self.ent_ki.get())
            kd = float(self.ent_kd.get())
            
            # Send to ESP32
            cmd = f"P {mid} {kp} {ki} {kd}\n"
            self.ser.write(cmd.encode())
            print(f"Sent: {cmd.strip()}")
            
            # Save to config
            if "pid" not in self.config:
                self.config["pid"] = DEFAULT_PID.copy()
            self.config["pid"][mid] = {"kp": kp, "ki": ki, "kd": kd}
            self.save_config()
            
            self.lbl_pid_status.config(text=f"Joint {mid} saved!", foreground="green")
        except Exception as e:
            self.lbl_pid_status.config(text=f"Error: {e}", foreground="red")
    
    def on_pid_joint_change(self, event=None):
        """When joint ID changes, load saved values for that joint"""
        try:
            joint_id = self.ent_pid_id.get()
            saved_pid = self.config.get("pid", DEFAULT_PID).get(joint_id, DEFAULT_PID.get(joint_id, {"kp": 10.0, "ki": 0.0, "kd": 0.5}))
            
            # Update entry fields
            self.ent_kp.delete(0, tk.END)
            self.ent_kp.insert(0, str(saved_pid.get("kp", 10.0)))
            self.ent_ki.delete(0, tk.END)
            self.ent_ki.insert(0, str(saved_pid.get("ki", 0.0)))
            self.ent_kd.delete(0, tk.END)
            self.ent_kd.insert(0, str(saved_pid.get("kd", 0.5)))
            
            self.lbl_pid_status.config(text=f"Loaded J{joint_id}", foreground="gray")
        except:
            pass
    
    def load_pid_from_file(self):
        """Reload PID values from config file"""
        self.config = self.load_config()
        self.on_pid_joint_change()  # Refresh displayed values
        self.lbl_pid_status.config(text="Config reloaded", foreground="blue")

    def start_sweep(self, axis):
        """Start velocity-based sweep on specified axis"""
        if not self.connected: return
        
        # Map axis name to number: X=0, Y=1, Z=2
        axis_map = {'x': 0, 'y': 1, 'z': 2}
        axis_num = axis_map.get(axis.lower(), 0)
        
        # Send SW command: SW axis velocity
        cmd = f"SW {axis_num} 60.0\n"  # 60 mm/s sweep speed
        self.ser.write(cmd.encode())
        print(f"Started Sweep {axis.upper()} at 60 mm/s")

    def stop_movement(self):
        """Send stop command"""
        if not self.connected: return
        self.ser.write(b"S\n")
        print("STOP sent")

    def toggle_torque(self):
        if not self.connected: return
        self.torque_enabled = not self.torque_enabled
        val = 1 if self.torque_enabled else 0
        self.ser.write(f"T {val}\n".encode())
        self.btn_torque.config(text="TORQUE OFF" if self.torque_enabled else "TORQUE ON")

    # ========== POSITION PROGRAMMING METHODS ==========
    
    def prog_update_listbox(self):
        """Update the position listbox display"""
        self.pos_listbox.delete(0, tk.END)
        for i, pos in enumerate(self.program_positions):
            wait_str = f" [wait {pos['wait']}s]" if pos['wait'] > 0 else ""
            line = f"{i+1}: X={pos['x']:7.1f} Y={pos['y']:7.1f} Z={pos['z']:7.1f} R={pos['roll']:5.2f}{wait_str}"
            self.pos_listbox.insert(tk.END, line)
    
    def prog_add_current(self):
        """Add current arm position to program"""
        try:
            wait = float(self.ent_wait_time.get())
        except:
            wait = 0.0
        
        pos = {
            'x': self.current_pos[0],
            'y': self.current_pos[1],
            'z': self.current_pos[2],
            'roll': self.current_joints[4],
            'wait': wait
        }
        self.program_positions.append(pos)
        self.prog_update_listbox()
        print(f"Added position: {pos}")
    
    def prog_add_target(self):
        """Add target entry values to program"""
        try:
            x = float(self.entries_target[0].get())
            y = float(self.entries_target[1].get())
            z = float(self.entries_target[2].get())
            roll = float(self.entries_target[3].get())
            wait = float(self.ent_wait_time.get())
        except Exception as e:
            messagebox.showerror("Error", f"Invalid input: {e}")
            return
        
        pos = {'x': x, 'y': y, 'z': z, 'roll': roll, 'wait': wait}
        self.program_positions.append(pos)
        self.prog_update_listbox()
        print(f"Added target: {pos}")
    
    def prog_move_up(self):
        """Move selected position up in list"""
        sel = self.pos_listbox.curselection()
        if not sel or sel[0] == 0:
            return
        idx = sel[0]
        self.program_positions[idx], self.program_positions[idx-1] = \
            self.program_positions[idx-1], self.program_positions[idx]
        self.prog_update_listbox()
        self.pos_listbox.selection_set(idx-1)
    
    def prog_move_down(self):
        """Move selected position down in list"""
        sel = self.pos_listbox.curselection()
        if not sel or sel[0] >= len(self.program_positions) - 1:
            return
        idx = sel[0]
        self.program_positions[idx], self.program_positions[idx+1] = \
            self.program_positions[idx+1], self.program_positions[idx]
        self.prog_update_listbox()
        self.pos_listbox.selection_set(idx+1)
    
    def prog_delete(self):
        """Delete selected position"""
        sel = self.pos_listbox.curselection()
        if not sel:
            return
        idx = sel[0]
        del self.program_positions[idx]
        self.prog_update_listbox()
    
    def prog_clear(self):
        """Clear all positions"""
        if self.program_positions:
            if messagebox.askyesno("Confirm", "Clear all positions?"):
                self.program_positions = []
                self.prog_update_listbox()
    
    def prog_goto_selected(self):
        """Go to selected position"""
        sel = self.pos_listbox.curselection()
        if not sel:
            return
        if not self.connected:
            return
        
        pos = self.program_positions[sel[0]]
        cmd = f"M {pos['x']} {pos['y']} {pos['z']} {pos['roll']} 2.0\n"
        self.ser.write(cmd.encode())
        print(f"Going to position {sel[0]+1}: {pos}")
    
    def prog_run(self):
        """Start running the program"""
        if not self.program_positions:
            messagebox.showwarning("Warning", "No positions in program")
            return
        if not self.connected:
            messagebox.showwarning("Warning", "Not connected")
            return
        
        self.program_running = True
        self.program_loop = self.prog_loop_var.get()
        self.program_current_idx = 0
        self.program_waiting = False
        self.path_done_received = False  # Initialize flag for loop detection
        self.lbl_prog_status.config(text="Running...", foreground="green")
        self.btn_prog_run.config(state='disabled')
        
        # Check if we can use smooth path mode (ESP32 handles blending)
        # Use smooth mode if we have multiple waypoints
        if len(self.program_positions) > 1:
            self.prog_run_smooth_path()
        else:
            # Single waypoint - use old method
            self.prog_execute_next()
    
    def prog_run_smooth_path(self):
        """Send all waypoints to ESP32 for smooth path following"""
        # Clear any existing path
        self.ser.write(b"PC\n")
        time.sleep(0.05)
        
        # Add all waypoints
        for pos in self.program_positions:
            cmd = f"PA {pos['x']} {pos['y']} {pos['z']} {pos['roll']} {pos['wait']}\n"
            self.ser.write(cmd.encode())
            time.sleep(0.02)  # Small delay between commands
        
        # Start path execution
        self.ser.write(b"PR\n")
        print(f"Started smooth path with {len(self.program_positions)} waypoints")
        
        # Monitor path completion
        self.root.after(100, self.prog_check_path_done)
    
    def prog_check_path_done(self):
        """Check if ESP32 path execution is complete"""
        if not self.program_running:
            return
        
        # Check if PATH_DONE was received (set by process_queue)
        if hasattr(self, 'path_done_received') and self.path_done_received:
            self.path_done_received = False  # Reset flag
            
            if self.program_loop and self.prog_loop_var.get():
                # Restart the path
                print("Looping path...")
                self.lbl_prog_status.config(text="Looping...", foreground="green")
                self.root.after(200, self._send_path_restart)  # Small delay before restart
                return
            else:
                self.prog_stop()
                self.lbl_prog_status.config(text="Completed", foreground="blue")
                return
        
        # Keep checking
        self.root.after(100, self.prog_check_path_done)
    
    def _send_path_restart(self):
        """Send PR command to restart the path and continue monitoring"""
        if self.connected and self.program_running:
            self.ser.write(b"PR\n")
            self.root.after(100, self.prog_check_path_done)
    
    def prog_stop(self):
        """Stop the program"""
        self.program_running = False
        self.program_waiting = False
        self.lbl_prog_status.config(text="Stopped", foreground="gray")
        self.btn_prog_run.config(state='normal')
        
        # Also stop the arm
        if self.connected:
            self.ser.write(b"S\n")
    
    def prog_execute_next(self):
        """Execute next position in program"""
        if not self.program_running:
            return
        
        # Check if we're done
        if self.program_current_idx >= len(self.program_positions):
            if self.program_loop:
                self.program_current_idx = 0
            else:
                self.prog_stop()
                self.lbl_prog_status.config(text="Completed", foreground="blue")
                return
        
        pos = self.program_positions[self.program_current_idx]
        
        # Highlight current position
        self.pos_listbox.selection_clear(0, tk.END)
        self.pos_listbox.selection_set(self.program_current_idx)
        self.pos_listbox.see(self.program_current_idx)
        
        self.lbl_prog_status.config(text=f"Step {self.program_current_idx + 1}/{len(self.program_positions)}", 
                                     foreground="green")
        
        # Send move command
        cmd = f"M {pos['x']} {pos['y']} {pos['z']} {pos['roll']} 2.0\n"
        self.ser.write(cmd.encode())
        print(f"Program step {self.program_current_idx + 1}: Moving to {pos}")
        
        # Schedule check for completion
        self.root.after(100, self.prog_check_done)
    
    def prog_check_done(self):
        """Check if current move is done"""
        if not self.program_running:
            return
        
        pos = self.program_positions[self.program_current_idx]
        
        # Check if we're close to target
        dx = self.current_pos[0] - pos['x']
        dy = self.current_pos[1] - pos['y']
        dz = self.current_pos[2] - pos['z']
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        if dist < 5.0:  # Within 5mm of target
            # Handle wait time
            if pos['wait'] > 0 and not self.program_waiting:
                self.program_waiting = True
                self.program_wait_start = time.time()
                self.lbl_prog_status.config(
                    text=f"Step {self.program_current_idx + 1} - Waiting {pos['wait']}s", 
                    foreground="orange")
            
            if self.program_waiting:
                elapsed = time.time() - self.program_wait_start
                if elapsed >= pos['wait']:
                    self.program_waiting = False
                    self.program_current_idx += 1
                    self.root.after(100, self.prog_execute_next)
                else:
                    # Keep waiting
                    self.root.after(100, self.prog_check_done)
            else:
                # No wait, move to next
                self.program_current_idx += 1
                self.root.after(100, self.prog_execute_next)
        else:
            # Still moving, check again
            self.root.after(100, self.prog_check_done)
    
    def prog_save(self):
        """Save program to file"""
        if not self.program_positions:
            messagebox.showwarning("Warning", "No positions to save")
            return
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            title="Save Program"
        )
        if filename:
            try:
                with open(filename, 'w') as f:
                    json.dump(self.program_positions, f, indent=2)
                print(f"Program saved to {filename}")
                messagebox.showinfo("Saved", f"Program saved to {filename}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save: {e}")
    
    def prog_load(self):
        """Load program from file"""
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            title="Load Program"
        )
        if filename:
            try:
                with open(filename, 'r') as f:
                    self.program_positions = json.load(f)
                self.prog_update_listbox()
                print(f"Program loaded from {filename}")
                messagebox.showinfo("Loaded", f"Loaded {len(self.program_positions)} positions")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load: {e}")
    
    def prog_edit_selected(self):
        """Edit selected position in a dialog"""
        sel = self.pos_listbox.curselection()
        if not sel:
            messagebox.showwarning("Warning", "Select a position to edit")
            return
        
        idx = sel[0]
        pos = self.program_positions[idx]
        
        # Create edit dialog
        dialog = tk.Toplevel(self.root)
        dialog.title(f"Edit Position {idx + 1}")
        dialog.geometry("300x200")
        dialog.transient(self.root)
        dialog.grab_set()
        
        # Position fields
        fields = [('X', pos['x']), ('Y', pos['y']), ('Z', pos['z']), 
                  ('Roll', pos['roll']), ('Wait (s)', pos['wait'])]
        entries = {}
        
        for i, (label, value) in enumerate(fields):
            ttk.Label(dialog, text=label + ":").grid(row=i, column=0, padx=10, pady=5, sticky='e')
            ent = ttk.Entry(dialog, width=15)
            ent.insert(0, str(value))
            ent.grid(row=i, column=1, padx=10, pady=5)
            entries[label] = ent
        
        def save_edit():
            try:
                self.program_positions[idx] = {
                    'x': float(entries['X'].get()),
                    'y': float(entries['Y'].get()),
                    'z': float(entries['Z'].get()),
                    'roll': float(entries['Roll'].get()),
                    'wait': float(entries['Wait (s)'].get())
                }
                self.prog_update_listbox()
                self.pos_listbox.selection_set(idx)
                dialog.destroy()
            except ValueError as e:
                messagebox.showerror("Error", f"Invalid value: {e}")
        
        def use_current():
            entries['X'].delete(0, tk.END)
            entries['X'].insert(0, f"{self.current_pos[0]:.1f}")
            entries['Y'].delete(0, tk.END)
            entries['Y'].insert(0, f"{self.current_pos[1]:.1f}")
            entries['Z'].delete(0, tk.END)
            entries['Z'].insert(0, f"{self.current_pos[2]:.1f}")
            entries['Roll'].delete(0, tk.END)
            entries['Roll'].insert(0, f"{self.current_joints[4]:.2f}")
        
        btn_frame = ttk.Frame(dialog)
        btn_frame.grid(row=5, column=0, columnspan=2, pady=15)
        ttk.Button(btn_frame, text="Use Current Pos", command=use_current).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="Save", command=save_edit).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="Cancel", command=dialog.destroy).pack(side='left', padx=5)
    
    def prog_add_pattern(self):
        """Add a motion pattern to the program"""
        pattern = self.pattern_var.get()
        try:
            size = float(self.ent_pattern_size.get())
            num_points = int(self.ent_pattern_points.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid size or points value")
            return
        
        if num_points < 3:
            num_points = 3
        
        # ESP32 can only hold 32 waypoints max
        MAX_WAYPOINTS = 32
        if num_points > MAX_WAYPOINTS:
            print(f"Note: Limiting points from {num_points} to {MAX_WAYPOINTS} (ESP32 max)")
            num_points = MAX_WAYPOINTS
        
        # Get center from current position or use defaults
        cx, cy, cz = self.current_pos[0], self.current_pos[1], self.current_pos[2]
        roll = self.current_joints[4] if len(self.current_joints) > 4 else 0
        
        new_positions = []
        
        if pattern == "Circle XY":
            for i in range(num_points):
                angle = 2 * math.pi * i / num_points
                new_positions.append({
                    'x': cx + size * math.cos(angle),
                    'y': cy + size * math.sin(angle),
                    'z': cz,
                    'roll': roll,
                    'wait': 0
                })
        
        elif pattern == "Circle XZ":
            for i in range(num_points):
                angle = 2 * math.pi * i / num_points
                new_positions.append({
                    'x': cx + size * math.cos(angle),
                    'y': cy,
                    'z': cz + size * math.sin(angle),
                    'roll': roll,
                    'wait': 0
                })
        
        elif pattern == "Circle YZ":
            for i in range(num_points):
                angle = 2 * math.pi * i / num_points
                new_positions.append({
                    'x': cx,
                    'y': cy + size * math.cos(angle),
                    'z': cz + size * math.sin(angle),
                    'roll': roll,
                    'wait': 0
                })
        
        elif pattern == "Square XY":
            corners = [(1, 0), (1, 1), (0, 1), (0, 0)]
            for dx, dy in corners:
                new_positions.append({
                    'x': cx + (dx - 0.5) * size,
                    'y': cy + (dy - 0.5) * size,
                    'z': cz,
                    'roll': roll,
                    'wait': 0
                })
        
        elif pattern == "Triangle XY":
            for i in range(3):
                angle = 2 * math.pi * i / 3 - math.pi / 2  # Start from top
                new_positions.append({
                    'x': cx + size * math.cos(angle),
                    'y': cy + size * math.sin(angle),
                    'z': cz,
                    'roll': roll,
                    'wait': 0
                })
        
        elif pattern == "Line X":
            for i in range(num_points):
                t = i / (num_points - 1) if num_points > 1 else 0.5
                new_positions.append({
                    'x': cx - size/2 + size * t,
                    'y': cy,
                    'z': cz,
                    'roll': roll,
                    'wait': 0
                })
        
        elif pattern == "Line Y":
            for i in range(num_points):
                t = i / (num_points - 1) if num_points > 1 else 0.5
                new_positions.append({
                    'x': cx,
                    'y': cy - size/2 + size * t,
                    'z': cz,
                    'roll': roll,
                    'wait': 0
                })
        
        elif pattern == "Line Z":
            for i in range(num_points):
                t = i / (num_points - 1) if num_points > 1 else 0.5
                new_positions.append({
                    'x': cx,
                    'y': cy,
                    'z': cz - size/2 + size * t,
                    'roll': roll,
                    'wait': 0
                })
        
        elif pattern == "Wave XZ":
            for i in range(num_points):
                t = i / (num_points - 1) if num_points > 1 else 0.5
                new_positions.append({
                    'x': cx - size + 2 * size * t,
                    'y': cy,
                    'z': cz + size/2 * math.sin(4 * math.pi * t),
                    'roll': roll,
                    'wait': 0
                })
        
        # Add to program
        self.program_positions.extend(new_positions)
        self.prog_update_listbox()
        print(f"Added {pattern} pattern with {len(new_positions)} points")
    
    def prog_load_preset(self):
        """Load a preset motion program"""
        preset = self.preset_var.get()
        
        # Ask to clear existing
        if self.program_positions:
            if not messagebox.askyesno("Confirm", "Clear existing program and load preset?"):
                return
        
        self.program_positions = []
        
        if preset == "Pick & Place":
            # Pick and place demo - pick from one side, place on other
            self.program_positions = [
                {'x': 200, 'y': 0, 'z': 150, 'roll': 0, 'wait': 0},      # Above pick
                {'x': 200, 'y': 0, 'z': 80, 'roll': 0, 'wait': 0.5},     # Pick position
                {'x': 200, 'y': 0, 'z': 150, 'roll': 0, 'wait': 0},      # Lift
                {'x': 0, 'y': 0, 'z': 200, 'roll': 0, 'wait': 0},        # Transit high
                {'x': -200, 'y': 0, 'z': 150, 'roll': 0, 'wait': 0},     # Above place
                {'x': -200, 'y': 0, 'z': 80, 'roll': 0, 'wait': 0.5},    # Place position
                {'x': -200, 'y': 0, 'z': 150, 'roll': 0, 'wait': 0},     # Lift
                {'x': 0, 'y': 0, 'z': 200, 'roll': 0, 'wait': 0},        # Transit back
            ]
        
        elif preset == "Home Position":
            self.program_positions = [
                {'x': 200, 'y': 0, 'z': 200, 'roll': 0, 'wait': 0},
            ]
        
        elif preset == "Scan Area":
            # Zigzag scan pattern
            z_height = 150
            for row in range(4):
                y = -60 + row * 40
                if row % 2 == 0:
                    x_vals = [100, 200]
                else:
                    x_vals = [200, 100]
                for x in x_vals:
                    self.program_positions.append({
                        'x': x, 'y': y, 'z': z_height, 'roll': 0, 'wait': 0.3
                    })
        
        elif preset == "Wave Hello":
            # Wave motion - move wrist back and forth
            base_x, base_z = 180, 200
            self.program_positions = [
                {'x': base_x, 'y': 0, 'z': base_z, 'roll': 0, 'wait': 0},
                {'x': base_x, 'y': 50, 'z': base_z + 30, 'roll': 0.5, 'wait': 0.2},
                {'x': base_x, 'y': -50, 'z': base_z + 30, 'roll': -0.5, 'wait': 0.2},
                {'x': base_x, 'y': 50, 'z': base_z + 30, 'roll': 0.5, 'wait': 0.2},
                {'x': base_x, 'y': -50, 'z': base_z + 30, 'roll': -0.5, 'wait': 0.2},
                {'x': base_x, 'y': 0, 'z': base_z, 'roll': 0, 'wait': 0},
            ]
        
        elif preset == "Demo Routine":
            # Comprehensive demo showing arm capabilities
            self.program_positions = [
                {'x': 200, 'y': 0, 'z': 200, 'roll': 0, 'wait': 0.5},    # Start
                {'x': 250, 'y': 0, 'z': 100, 'roll': 0, 'wait': 0.3},    # Forward low
                {'x': 150, 'y': 100, 'z': 150, 'roll': 0.5, 'wait': 0.3}, # Side
                {'x': 150, 'y': -100, 'z': 150, 'roll': -0.5, 'wait': 0.3},# Other side
                {'x': 0, 'y': 0, 'z': 300, 'roll': 0, 'wait': 0.5},      # High center
                {'x': -150, 'y': 0, 'z': 200, 'roll': 0, 'wait': 0.5},   # Back side
                {'x': 200, 'y': 0, 'z': 200, 'roll': 0, 'wait': 0},      # Return home
            ]
        
        elif preset == "Touch Corners":
            # Touch four corners of workspace
            z = 100
            self.program_positions = [
                {'x': 200, 'y': 100, 'z': z, 'roll': 0, 'wait': 0.3},
                {'x': 200, 'y': -100, 'z': z, 'roll': 0, 'wait': 0.3},
                {'x': -200, 'y': -100, 'z': z, 'roll': 0, 'wait': 0.3},
                {'x': -200, 'y': 100, 'z': z, 'roll': 0, 'wait': 0.3},
                {'x': 200, 'y': 0, 'z': 200, 'roll': 0, 'wait': 0},  # Return home
            ]
        
        self.prog_update_listbox()
        print(f"Loaded preset: {preset} with {len(self.program_positions)} positions")

if __name__ == "__main__":
    root = tk.Tk()
    app = ArmGUI(root)
    root.mainloop()
