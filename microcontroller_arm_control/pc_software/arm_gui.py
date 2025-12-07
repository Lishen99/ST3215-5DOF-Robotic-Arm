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
from scipy.interpolate import CubicSpline

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

# Default MPC weights (not PID - firmware uses MPC)
DEFAULT_MPC_WEIGHTS = {
    "position": 10.0,
    "velocity": 1.0,
    "terminal": 100.0
}

class ArmGUI:
    def __init__(self, root, port='COM8', baud=921600):
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
        
        # Path visualization
        self.path_preview = None  # Stores generated path for visualization
        self.path_type = 'linear'  # 'linear', 'smooth', 'parametric'
        self.show_path = tk.BooleanVar(value=True)
        
        # Continuous loop management
        self.loop_waypoint_index = 0  # Current index in path being sent
        self.loop_active = False  # True when continuously feeding buffer
        self.filtered_path_points = []  # Cached filtered points for looping
        self.current_buffer_count = 0  # Track firmware buffer level from responses
        self.current_buffer_count = 0  # Track firmware buffer level from responses
        
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
        return {"mpc": DEFAULT_MPC_WEIGHTS.copy()}
    
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
        # PID functions removed - firmware uses MPC, not PID
        pass

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

        # MPC Tuning (future enhancement)
        # Note: Firmware uses MPC with QP solver, not traditional PID

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
        
        # Path type selector
        path_type_frame = ttk.Frame(prog_frame)
        path_type_frame.pack(fill='x', pady=2)
        
        ttk.Label(path_type_frame, text="Path Type:").pack(side='left', padx=2)
        self.path_type_var = tk.StringVar(value="linear")
        ttk.Radiobutton(path_type_frame, text="Linear", variable=self.path_type_var, 
                       value="linear", command=self.on_path_type_change).pack(side='left', padx=5)
        ttk.Radiobutton(path_type_frame, text="Smooth", variable=self.path_type_var, 
                       value="smooth", command=self.on_path_type_change).pack(side='left', padx=5)
        
        ttk.Button(path_type_frame, text="Generate Path", command=self.generate_path_preview).pack(side='left', padx=10)
        ttk.Checkbutton(path_type_frame, text="Show Path", variable=self.show_path,
                       command=self.update_visualization).pack(side='left', padx=5)
        
        # Function input
        func_frame = ttk.Frame(prog_frame)
        func_frame.pack(fill='x', pady=2)
        
        ttk.Label(func_frame, text="Function (x(t), y(t), z(t)):").pack(side='left', padx=2)
        self.func_entry = ttk.Entry(func_frame, width=50)
        self.func_entry.pack(side='left', padx=2, fill='x', expand=True)
        self.func_entry.insert(0, "200*cos(t), 200*sin(t), 150 + 50*sin(3*t)")
        ttk.Button(func_frame, text="Generate Path", command=self.generate_function_path).pack(side='left', padx=2)
        ttk.Button(func_frame, text="Execute Function", command=self.execute_function).pack(side='left', padx=2)
        
        # Run controls
        run_frame = ttk.Frame(prog_frame)
        run_frame.pack(fill='x', pady=2)
        
        self.btn_prog_run = ttk.Button(run_frame, text="▶ RUN", command=self.prog_run)
        self.btn_prog_run.pack(side='left', padx=2)
        ttk.Button(run_frame, text="■ STOP", command=self.prog_stop).pack(side='left', padx=2)
        
        self.prog_loop_var = tk.BooleanVar(value=False)
        loop_check = ttk.Checkbutton(run_frame, text="Loop", variable=self.prog_loop_var)
        loop_check.pack(side='left', padx=10)
        # Auto-regenerate path when loop is toggled
        loop_check.configure(command=self.on_loop_toggle)
        
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
        
        # Draw programmed waypoints
        if self.program_positions:
            wp_xs = [p['x'] for p in self.program_positions]
            wp_ys = [p['y'] for p in self.program_positions]
            wp_zs = [p['z'] for p in self.program_positions]
            
            # Waypoint markers
            self.ax.scatter(wp_xs, wp_ys, wp_zs, c='orange', s=80, marker='s', 
                          alpha=0.7, label=f'Waypoints ({len(self.program_positions)})')
            
            # Number labels for waypoints
            for i, (x, y, z) in enumerate(zip(wp_xs, wp_ys, wp_zs)):
                self.ax.text(x, y, z, f' {i+1}', fontsize=8, color='orange')
        
        # Draw path preview if enabled
        if self.show_path.get() and self.path_preview is not None and len(self.path_preview) > 1:
            path_xs = [p[0] for p in self.path_preview]
            path_ys = [p[1] for p in self.path_preview]
            path_zs = [p[2] for p in self.path_preview]
            
            # Draw entire path as single line (much faster than loop)
            self.ax.plot(path_xs, path_ys, path_zs, 
                       color='cyan', linewidth=2, alpha=0.7, label='Generated Path')
            
            # Start and end markers
            self.ax.scatter([path_xs[0]], [path_ys[0]], [path_zs[0]], 
                          c='lime', s=150, marker='o', label='Path Start')
            self.ax.scatter([path_xs[-1]], [path_ys[-1]], [path_zs[-1]], 
                          c='red', s=150, marker='X', label='Path End')
        
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
    
    def generate_path_preview(self):
        """Generate path preview based on waypoints and path type"""
        if len(self.program_positions) < 2:
            messagebox.showwarning("Path Preview", "Need at least 2 waypoints to generate path")
            return
        
        path_type = self.path_type_var.get()
        
        # Add first waypoint to end if looping
        waypoints = self.program_positions.copy()
        is_loop = self.prog_loop_var.get()
        if is_loop and len(waypoints) >= 2:
            waypoints.append(waypoints[0].copy())  # Close the loop
        
        if path_type == 'linear':
            # Linear segments between waypoints
            self.path_preview = self.generate_linear_path(waypoints)
        elif path_type == 'smooth':
            # Smooth cubic spline through waypoints
            self.path_preview = self.generate_smooth_path(waypoints)
        
        # Update visualization (will happen on next cycle)
        # Don't call update_visualization() here to avoid blocking
        
        # Show statistics
        path_length = self.calculate_path_length(self.path_preview)
        loop_text = " (Loop)" if is_loop else ""
        print(f"Path generated: {path_type.capitalize()}{loop_text}, "
              f"Waypoints: {len(self.program_positions)}, "
              f"Points: {len(self.path_preview)}, "
              f"Length: {path_length:.1f}mm")
    
    def generate_linear_path(self, waypoints, points_per_segment=10):
        """Generate linear path through waypoints"""
        path = []
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            
            # Interpolate between waypoints
            for t in np.linspace(0, 1, points_per_segment, endpoint=(i == len(waypoints) - 2)):
                x = start['x'] + t * (end['x'] - start['x'])
                y = start['y'] + t * (end['y'] - start['y'])
                z = start['z'] + t * (end['z'] - start['z'])
                path.append([x, y, z])
        
        return path
    
    def generate_smooth_path(self, waypoints, num_points=100):
        """Generate smooth cubic spline through waypoints"""
        try:
            # Extract coordinates
            xs = [w['x'] for w in waypoints]
            ys = [w['y'] for w in waypoints]
            zs = [w['z'] for w in waypoints]
            
            # Parameter for spline (use cumulative chord length)
            t = [0]
            for i in range(1, len(waypoints)):
                dx = xs[i] - xs[i-1]
                dy = ys[i] - ys[i-1]
                dz = zs[i] - zs[i-1]
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                t.append(t[-1] + dist)
            
            # Normalize parameter
            t = np.array(t)
            t = t / t[-1] if t[-1] > 0 else t
            
            # Create cubic splines
            cs_x = CubicSpline(t, xs, bc_type='natural')
            cs_y = CubicSpline(t, ys, bc_type='natural')
            cs_z = CubicSpline(t, zs, bc_type='natural')
            
            # Generate smooth path
            t_smooth = np.linspace(0, 1, num_points)
            path = [[cs_x(ti), cs_y(ti), cs_z(ti)] for ti in t_smooth]
            
            return path
        except Exception as e:
            messagebox.showerror("Spline Error", f"Failed to generate smooth path: {e}")
            return self.generate_linear_path(waypoints)
    
    def calculate_path_length(self, path):
        """Calculate total path length in mm"""
        if not path or len(path) < 2:
            return 0.0
        
        length = 0.0
        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            dz = path[i+1][2] - path[i][2]
            length += math.sqrt(dx*dx + dy*dy + dz*dz)
        
        return length
    
    def on_path_type_change(self):
        """Called when path type radio button changes"""
        self.path_type = self.path_type_var.get()
        # Auto-regenerate if we have waypoints
        if len(self.program_positions) >= 2:
            self.generate_path_preview()
    
    def on_loop_toggle(self):
        """Called when loop checkbox is toggled"""
        # Auto-regenerate path to show/hide loop connection
        if len(self.program_positions) >= 2 and self.path_preview is not None:
            self.generate_path_preview()
    
    def generate_function_path(self):
        """Generate path preview from parametric function"""
        func_str = self.func_entry.get().strip()
        if not func_str:
            messagebox.showerror("Error", "Please enter a function")
            return
        
        try:
            # Parse the function string - expect format like "x_expr, y_expr, z_expr"
            parts = [p.strip() for p in func_str.split(',')]
            if len(parts) != 3:
                messagebox.showerror("Error", "Function must have 3 components: x(t), y(t), z(t) separated by commas")
                return
            
            # Import math functions for eval
            import numpy as np
            from math import sin, cos, tan, sqrt, pi, exp, log
            
            # Generate path by evaluating function at different t values
            t_values = np.linspace(0, 2*pi, 100)  # t from 0 to 2π with 100 points
            path_points = []
            
            for t in t_values:
                try:
                    x = eval(parts[0], {"__builtins__": {}}, 
                            {"t": t, "sin": sin, "cos": cos, "tan": tan, "sqrt": sqrt, 
                             "pi": pi, "exp": exp, "log": log, "abs": abs})
                    y = eval(parts[1], {"__builtins__": {}}, 
                            {"t": t, "sin": sin, "cos": cos, "tan": tan, "sqrt": sqrt, 
                             "pi": pi, "exp": exp, "log": log, "abs": abs})
                    z = eval(parts[2], {"__builtins__": {}}, 
                            {"t": t, "sin": sin, "cos": cos, "tan": tan, "sqrt": sqrt, 
                             "pi": pi, "exp": exp, "log": log, "abs": abs})
                    path_points.append([float(x), float(y), float(z)])
                except Exception as e:
                    messagebox.showerror("Error", f"Error evaluating function at t={t:.3f}: {e}")
                    return
            
            # Convert to program positions format (for compatibility with existing code)
            self.program_positions = []
            for point in path_points:
                self.program_positions.append({
                    'x': point[0],
                    'y': point[1],
                    'z': point[2],
                    'roll': 0.0
                })
            
            # Set as path preview
            self.path_preview = path_points
            self.path_type_var.set('smooth')  # Functions are smooth
            
            # Update visualization
            self.update_arm_plot()
            
            print(f"Generated function path: {func_str}")
            print(f"Generated {len(path_points)} points")
            messagebox.showinfo("Success", f"Generated {len(path_points)} waypoints from function")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to parse function: {e}")
    
    def execute_function(self):
        """Execute parametric function trajectory"""
        if not self.ser or not self.ser.is_open:
            messagebox.showerror("Error", "Not connected to Teensy")
            return
        
        # Generate path first if not already done
        if not self.path_preview or len(self.program_positions) == 0:
            self.generate_function_path()
            if not self.path_preview:
                return  # Generation failed
        
        # Execute using the normal program run (which supports looping)
        print(f"Executing parametric function with {len(self.program_positions)} waypoints")
        self.prog_run()

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.connected = True
            self.lbl_status.config(text=f"Connected to {self.port} @ {self.baud}", foreground="green")
            threading.Thread(target=self.read_serial, daemon=True).start()
            
            # Auto-send speed setting and MPC parameters to Teensy after connection
            self.root.after(500, self.sync_settings_to_esp32)
            self.root.after(800, self.load_and_apply_mpc_params)
        except Exception as e:
            self.lbl_status.config(text=f"Error: {e}", foreground="red")
    
    def load_and_apply_mpc_params(self):
        """Load MPC parameters from mpc_params.json and apply to firmware"""
        if not self.connected:
            return
        
        try:
            if os.path.exists('mpc_params.json'):
                with open('mpc_params.json', 'r') as f:
                    params = json.load(f)
                
                # Send each parameter to firmware
                for name, value in params.items():
                    cmd = f"MPCSET {name} {value:.2f}\n"
                    self.ser.write(cmd.encode())
                    time.sleep(0.02)
                
                print(f"✓ Applied MPC parameters from mpc_params.json")
                print(f"  Qpos={params.get('Qpos', 'N/A')}, Qfpos={params.get('Qfpos', 'N/A')}, R={params.get('R', 'N/A')}")
            else:
                print("No mpc_params.json found - using firmware defaults")
        except Exception as e:
            print(f"Error loading MPC parameters: {e}")
    
    def sync_settings_to_esp32(self):
        """Send current speed setting to Teensy"""
        if not self.connected: return
        # Send speed multiplier (firmware uses SPD command)
        speed = self.speed_slider.get()
        self.ser.write(f"SPD {speed:.2f}\n".encode())
        print(f"Synced speed: {speed:.2f}")

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
            # Parse buffer count from ANY message containing "buffer:"
            if "buffer:" in msg:
                try:
                    # Extract buffer count from either format:
                    # "OK Path added (buffer: 123/200)" or "PATH Blend (buffer: 123)"
                    buffer_part = msg.split("buffer:")[1].split(")")[0].strip()
                    
                    # Handle both "123/200" and "123" formats
                    if "/" in buffer_part:
                        count_str = buffer_part.split("/")[0].strip()
                    else:
                        count_str = buffer_part.strip()
                    
                    new_count = int(count_str)
                    if new_count != self.current_buffer_count:
                        print(f"DEBUG PARSE: '{msg}' -> buffer_part='{buffer_part}' -> count={new_count}")
                        self.current_buffer_count = new_count
                except Exception as e:
                    print(f"DEBUG PARSE ERROR: '{msg}' -> {e}")
            
            if msg.startswith("PATH"):
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
    
    def on_speed_change(self, val):
        """Update speed multiplier"""
        speed = float(val)
        self.speed_var.set(f"{speed:.1f}")
        if self.connected:
            cmd = f"SPD {speed:.2f}\n"
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

    # PID tuning functions removed - firmware uses MPC

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
        """Send path to firmware for smooth execution"""
        # If we have a generated smooth path preview, use those points
        if self.path_preview is not None and self.path_type_var.get() == 'smooth':
            print(f"Using pre-generated smooth path with {len(self.path_preview)} points")
            
            # Set path type in firmware (1 = smooth)
            self.ser.write(b"PTYPE 1\n")
            time.sleep(0.05)
            
            # Clear existing path
            self.ser.write(b"PC\n")
            time.sleep(0.05)
            
            # Filter path points - only send if distance from last point > threshold
            # Also insert intermediate waypoints when J1 rotation exceeds 150° to avoid boundary crossing
            filtered_points = []
            min_distance = 20.0  # mm - minimum distance between sent waypoints
            max_j1_rotation = 150.0 * (math.pi / 180.0)  # Maximum J1 rotation per segment (radians)
            
            filtered_points.append(self.path_preview[0])  # Always include first point
            
            for i in range(1, len(self.path_preview)):
                point = self.path_preview[i]
                last_point = filtered_points[-1]
                
                dx = point[0] - last_point[0]
                dy = point[1] - last_point[1]
                dz = point[2] - last_point[2]
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                # Calculate J1 angle change (base rotation around Z)
                last_j1 = math.atan2(last_point[1], last_point[0])
                current_j1 = math.atan2(point[1], point[0])
                j1_delta = current_j1 - last_j1
                
                # Normalize to [-π, π]
                while j1_delta > math.pi:
                    j1_delta -= 2 * math.pi
                while j1_delta < -math.pi:
                    j1_delta += 2 * math.pi
                
                # Check if J1 rotation is too large (would cross boundary)
                if abs(j1_delta) > max_j1_rotation:
                    # Insert intermediate waypoints to break up the rotation
                    num_segments = int(math.ceil(abs(j1_delta) / max_j1_rotation))
                    print(f"  Large J1 rotation detected ({j1_delta * 57.3:.1f}°), inserting {num_segments-1} intermediate waypoints")
                    
                    for seg in range(1, num_segments + 1):
                        t = seg / num_segments
                        interp_point = [
                            last_point[0] + t * dx,
                            last_point[1] + t * dy,
                            last_point[2] + t * dz
                        ]
                        filtered_points.append(interp_point)
                elif dist >= min_distance:
                    filtered_points.append(point)
            
            # Check if we should close the loop
            is_looping = self.prog_loop_var.get()
            
            # In loop mode, check if last point is close to first - if so, skip it to avoid micro-adjustments
            if is_looping and len(filtered_points) > 2:
                first = filtered_points[0]
                last = self.path_preview[-1]
                dx = last[0] - first[0]
                dy = last[1] - first[1]
                dz = last[2] - first[2]
                loop_dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                if loop_dist < min_distance * 2:  # If already close, skip adding last point
                    print(f"Loop already closed (distance: {loop_dist:.1f}mm), skipping duplicate endpoint")
                else:
                    # Add last point if not too close
                    if filtered_points[-1] != self.path_preview[-1]:
                        filtered_points.append(self.path_preview[-1])
            else:
                # Not looping - always include last point
                if len(filtered_points) == 0 or filtered_points[-1] != self.path_preview[-1]:
                    filtered_points.append(self.path_preview[-1])
            
            print(f"Filtered to {len(filtered_points)} waypoints (min spacing: {min_distance}mm)")
            
            # Send all filtered waypoints at once (firmware has 200-waypoint buffer)
            print(f"Sending {len(filtered_points)} waypoints to firmware...")
            for i, point in enumerate(filtered_points):
                # Get roll and wait time from corresponding waypoint (interpolate if needed)
                t = i / max(len(filtered_points) - 1, 1)
                wp_idx = int(t * (len(self.program_positions) - 1))
                wp_idx = min(wp_idx, len(self.program_positions) - 1)
                roll = self.program_positions[wp_idx]['roll']
                wait = self.program_positions[wp_idx]['wait']
                
                # Note: For smooth paths, wait times are typically 0 (continuous motion)
                # If wait > 0 is needed, user should use linear mode
                cmd = f"PA {point[0]:.2f} {point[1]:.2f} {point[2]:.2f} {roll:.3f} {wait:.3f}\n"
                self.ser.write(cmd.encode())
                time.sleep(0.008)  # Small delay for serial transmission
                
                # Print progress every 10 waypoints
                if (i + 1) % 10 == 0 or i == len(filtered_points) - 1:
                    print(f"  Sent {i + 1}/{len(filtered_points)} waypoints")
            
            # Store for potential re-queueing only if loop mode
            if self.prog_loop_var.get():
                self.filtered_path_points = filtered_points
                self.loop_waypoint_index = len(filtered_points)  # Start from end, will wrap to 0
                self.loop_active = True
                print(f"Sent all {len(filtered_points)} waypoints, will continuously top up buffer")
            else:
                self.loop_active = False
        else:
            # Use original waypoint mode (linear)
            print(f"Using waypoint mode with {len(self.program_positions)} waypoints")
            
            # Set path type in firmware (0 = linear)
            self.ser.write(b"PTYPE 0\n")
            time.sleep(0.05)
            
            # Clear any existing path
            self.ser.write(b"PC\n")
            time.sleep(0.05)
            
            # Add all waypoints
            for pos in self.program_positions:
                cmd = f"PA {pos['x']} {pos['y']} {pos['z']} {pos['roll']} {pos['wait']}\n"
                self.ser.write(cmd.encode())
                time.sleep(0.02)
            
            # Store for potential loop support
            if self.prog_loop_var.get():
                self.loop_active = True
                self.linear_loop_positions = self.program_positions.copy()
                print(f"Linear mode loop enabled with {len(self.linear_loop_positions)} waypoints")
            else:
                self.loop_active = False
        
        # Start path execution
        self.ser.write(b"PR\n")
        path_type = "smooth" if self.path_preview is not None and self.path_type_var.get() == 'smooth' else "linear"
        loop_text = " (looping)" if self.prog_loop_var.get() else ""
        print(f"Started {path_type} path execution{loop_text}")
        
        # Monitor path completion
        self.root.after(100, self.prog_check_path_done)
    
    def prog_check_path_done(self):
        """Check if path execution is complete and top up buffer when needed for loops"""
        if not self.program_running:
            return
        
        # Debug: Print state every check
        print(f"DEBUG: Check - loop_active={self.loop_active}, loop_var={self.prog_loop_var.get()}, buffer={self.current_buffer_count}")
        
        # In loop mode, continuously top up buffer - ignore PATH_DONE
        if self.loop_active and self.prog_loop_var.get():
            # Only add waypoints if buffer is below 50% (100 waypoints)
            buffer_threshold = 100
            
            if self.current_buffer_count < buffer_threshold:
                # Add enough waypoints to bring buffer to ~120-150 range
                waypoints_to_add = min(50, 150 - self.current_buffer_count)
                
                if waypoints_to_add > 0:
                    # Handle smooth mode
                    if self.path_preview is not None and self.path_type_var.get() == 'smooth':
                        for _ in range(waypoints_to_add):
                            # Loop back to start when reaching end of path
                            if self.loop_waypoint_index >= len(self.filtered_path_points):
                                self.loop_waypoint_index = 0
                            
                            point = self.filtered_path_points[self.loop_waypoint_index]
                            t = self.loop_waypoint_index / max(len(self.filtered_path_points) - 1, 1)
                            wp_idx = int(t * (len(self.program_positions) - 1))
                            wp_idx = min(wp_idx, len(self.program_positions) - 1)
                            roll = self.program_positions[wp_idx]['roll']
                            wait = self.program_positions[wp_idx]['wait']
                            
                            cmd = f"PA {point[0]:.2f} {point[1]:.2f} {point[2]:.2f} {roll:.3f} {wait:.3f}\n"
                            self.ser.write(cmd.encode())
                            time.sleep(0.005)
                            
                            self.loop_waypoint_index += 1
                        
                        print(f"Buffer low ({self.current_buffer_count}), added {waypoints_to_add} smooth waypoints")
                    
                    # Handle linear mode
                    elif hasattr(self, 'linear_loop_positions'):
                        for i in range(waypoints_to_add):
                            # Cycle through original positions
                            if not hasattr(self, 'linear_loop_index'):
                                self.linear_loop_index = 0
                            
                            pos = self.linear_loop_positions[self.linear_loop_index]
                            cmd = f"PA {pos['x']} {pos['y']} {pos['z']} {pos['roll']} {pos['wait']}\n"
                            self.ser.write(cmd.encode())
                            time.sleep(0.02)
                            
                            self.linear_loop_index = (self.linear_loop_index + 1) % len(self.linear_loop_positions)
                        
                        print(f"Buffer low ({self.current_buffer_count}), added {waypoints_to_add} linear waypoints")
                else:
                    print(f"DEBUG: No top-up - waypoints_to_add={waypoints_to_add}")
            
            # Check more frequently - every 100ms instead of 200ms
            self.root.after(100, self.prog_check_path_done)
            return
        
        # Not in loop mode - check for PATH_DONE to stop
        if hasattr(self, 'path_done_received') and self.path_done_received:
            self.path_done_received = False  # Reset flag
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
        self.loop_active = False
        self.loop_waypoint_index = 0
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
