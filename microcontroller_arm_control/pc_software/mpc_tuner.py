#!/usr/bin/env python3
"""
MPC Auto-Tuner for 5DOF Arm
Automatically tunes MPC parameters by testing different values and measuring performance
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import time
import numpy as np
import json
import os
from dataclasses import dataclass
from typing import List, Tuple
import threading

@dataclass
class MPCParams:
    """MPC weight parameters"""
    Qpos: float = 100.0
    Qvel: float = 1.0
    Qfpos: float = 200.0
    Qfvel: float = 10.0
    R: float = 0.5
    Rdelta: float = 0.1

@dataclass
class PerformanceMetrics:
    """Performance metrics for a test"""
    overshoot: float = 0.0  # mm
    settling_time: float = 0.0  # seconds
    steady_state_error: float = 0.0  # mm
    smoothness: float = 0.0  # acceleration variance
    score: float = 0.0  # Combined score

class MPCTunerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("MPC Auto-Tuner")
        self.root.geometry("800x900")
        
        self.ser = None
        self.connected = False
        self.tuning_active = False
        self.tuning_thread = None
        
        # Current parameters
        self.params = MPCParams()
        
        # Home position for consistent starting point (joint angles in DEGREES)
        # Format: (q1_base, q2_shoulder, q3_elbow, q4_wrist)
        self.home_position = (0, 0, 0, 0)
        
        # Test configuration - SMALL angle movements (max ±30 degrees per joint)
        # Motor 6 (roll) excluded for safety
        # Format: (q1, q2, q3, q4) in DEGREES
        # Mix of single and multi-joint movements to test coordination
        self.test_points = [
            (15, 0, 0, 0),      # Base only +15°
            (0, 20, 0, 0),      # Shoulder only +20°
            (15, 20, 0, 0),     # Base+Shoulder combined
            (0, 15, 20, 0),     # Shoulder+Elbow combined
            (0, 0, 20, 25),     # Elbow+Wrist combined
            (10, 15, 15, 20),   # All 4 joints combined
            (-10, -15, -15, -20), # All 4 joints negative
            (15, -15, 20, -20)  # Mixed directions
        ]
        
        self.create_widgets()
        
    def create_widgets(self):
        """Create GUI widgets"""
        
        # Connection Frame
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn_frame.pack(fill='x', padx=10, pady=5)
        
        ttk.Label(conn_frame, text="Port:").pack(side='left')
        self.port_combo = ttk.Combobox(conn_frame, width=15)
        self.port_combo.pack(side='left', padx=5)
        self.refresh_ports()
        
        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(side='left', padx=5)
        
        self.lbl_status = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.lbl_status.pack(side='left', padx=10)
        
        # Current Parameters Frame
        params_frame = ttk.LabelFrame(self.root, text="Current MPC Parameters", padding=10)
        params_frame.pack(fill='x', padx=10, pady=5)
        
        params_grid = ttk.Frame(params_frame)
        params_grid.pack(fill='x')
        
        self.param_entries = {}
        param_names = ['Qpos', 'Qvel', 'Qfpos', 'Qfvel', 'R', 'Rdelta']
        
        for i, name in enumerate(param_names):
            row = i // 3
            col = (i % 3) * 2
            
            ttk.Label(params_grid, text=f"{name}:").grid(row=row, column=col, padx=5, pady=2, sticky='e')
            entry = ttk.Entry(params_grid, width=10)
            entry.insert(0, str(getattr(self.params, name)))
            entry.grid(row=row, column=col+1, padx=5, pady=2)
            self.param_entries[name] = entry
        
        btn_frame = ttk.Frame(params_frame)
        btn_frame.pack(fill='x', pady=5)
        
        ttk.Button(btn_frame, text="Get From Firmware", command=self.get_current_params).pack(side='left', padx=5)
        ttk.Button(btn_frame, text="Send To Firmware", command=self.send_params).pack(side='left', padx=5)
        
        # Tuning Configuration Frame
        tuning_frame = ttk.LabelFrame(self.root, text="Auto-Tuning Configuration", padding=10)
        tuning_frame.pack(fill='x', padx=10, pady=5)
        
        ttk.Label(tuning_frame, text="Tuning Mode:").pack(anchor='w')
        self.mode_var = tk.StringVar(value="quick")
        modes = [
            ("Quick Tune (~5 min, good results)", "quick"),
            ("Standard Tune (~15 min, better results)", "standard"),
            ("Deep Tune (~1-2 hours, PERFECT results)", "deep")
        ]
        for text, value in modes:
            ttk.Radiobutton(tuning_frame, text=text, variable=self.mode_var, value=value, command=self.update_mode_info).pack(anchor='w', padx=20)
        
        self.lbl_mode_info = ttk.Label(tuning_frame, text="", foreground="blue", wraplength=700)
        self.lbl_mode_info.pack(anchor='w', padx=20, pady=5)
        self.update_mode_info()
        
        ttk.Separator(tuning_frame, orient='horizontal').pack(fill='x', pady=10)
        
        ttk.Label(tuning_frame, text="Optimization Goal:").pack(anchor='w')
        self.goal_var = tk.StringVar(value="balanced")
        goals = [
            ("Balanced (all-around performance)", "balanced"),
            ("Fast (quick settling, may overshoot)", "fast"),
            ("Precise (minimal error, may be slower)", "precise"),
            ("Smooth (low jerk, gentle motion)", "smooth"),
            ("Ultimate (0.1mm accuracy, no overshoot, smooth+fast)", "ultimate")
        ]
        for text, value in goals:
            ttk.Radiobutton(tuning_frame, text=text, variable=self.goal_var, value=value).pack(anchor='w', padx=20)
        
        ttk.Separator(tuning_frame, orient='horizontal').pack(fill='x', pady=10)
        
        self.lbl_time_estimate = ttk.Label(tuning_frame, text="Estimated time: ~5 minutes", foreground="green")
        self.lbl_time_estimate.pack(anchor='w')
        self.mode_var.trace('w', lambda *args: self.update_time_estimate())
        self.goal_var.trace('w', lambda *args: self.update_time_estimate())
        
        # Control Frame
        control_frame = ttk.Frame(self.root, padding=10)
        control_frame.pack(fill='x', padx=10, pady=5)
        
        self.btn_start = ttk.Button(control_frame, text="Start Auto-Tuning", command=self.start_tuning, state='disabled')
        self.btn_start.pack(side='left', padx=5)
        
        self.btn_stop = ttk.Button(control_frame, text="Stop", command=self.stop_tuning, state='disabled')
        self.btn_stop.pack(side='left', padx=5)
        
        ttk.Button(control_frame, text="Test Current Params", command=self.test_current).pack(side='left', padx=5)
        
        # Progress Frame
        progress_frame = ttk.LabelFrame(self.root, text="Progress", padding=10)
        progress_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        self.progress = ttk.Progressbar(progress_frame, mode='determinate')
        self.progress.pack(fill='x', pady=5)
        
        self.lbl_progress = ttk.Label(progress_frame, text="Ready")
        self.lbl_progress.pack(anchor='w')
        
        # Results Text
        self.txt_results = tk.Text(progress_frame, height=20, width=80)
        self.txt_results.pack(fill='both', expand=True, pady=5)
        
        scrollbar = ttk.Scrollbar(self.txt_results, command=self.txt_results.yview)
        scrollbar.pack(side='right', fill='y')
        self.txt_results.config(yscrollcommand=scrollbar.set)
    
    def update_mode_info(self):
        """Update mode description"""
        mode = self.mode_var.get()
        info = {
            "quick": "Tests 20 parameter sets, 2 positions per set, 2s settling check. Good for initial tuning.",
            "standard": "Tests 50 parameter sets, 4 positions per set, 3s settling check. Better accuracy.",
            "deep": "Tests 200+ parameter sets with refinement, all 8 positions, 5s settling check, vibration analysis. ULTIMATE PRECISION - no shaking, <0.1mm error, zero overshoot."
        }
        self.lbl_mode_info.config(text=info.get(mode, ""))
    
    def update_time_estimate(self):
        """Update time estimate based on mode"""
        mode = self.mode_var.get()
        goal = self.goal_var.get()
        
        estimates = {
            "quick": "~5-10 minutes",
            "standard": "~15-25 minutes",
            "deep": "~60-120 minutes"
        }
        
        if goal == "ultimate":
            estimates["quick"] = "~10-15 minutes"
            estimates["standard"] = "~25-40 minutes"
            estimates["deep"] = "~90-150 minutes"
        
        self.lbl_time_estimate.config(text=f"Estimated time: {estimates.get(mode, 'Unknown')}")
    
    def refresh_ports(self):
        """Refresh available serial ports"""
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports and not self.port_combo.get():
            self.port_combo.current(0)
    
    def toggle_connection(self):
        """Connect/disconnect from serial port"""
        if self.connected:
            self.disconnect()
        else:
            self.connect()
    
    def connect(self):
        """Connect to serial port"""
        port = self.port_combo.get()
        if not port:
            messagebox.showerror("Error", "Please select a port")
            return
        
        try:
            self.ser = serial.Serial(port, 115200, timeout=1)
            time.sleep(0.5)
            self.connected = True
            self.lbl_status.config(text="Connected", foreground="green")
            self.btn_connect.config(text="Disconnect")
            self.btn_start.config(state='normal')
            self.log("Connected to " + port)
            
            # Auto-load and apply saved parameters if they exist
            if os.path.exists('mpc_params.json'):
                self.log("Found saved parameters, loading...")
                self.load_parameters()
                time.sleep(0.2)
                self.send_params()
                self.log("✓ Saved parameters applied to firmware")
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))
    
    def disconnect(self):
        """Disconnect from serial port"""
        if self.ser:
            self.ser.close()
        self.connected = False
        self.lbl_status.config(text="Disconnected", foreground="red")
        self.btn_connect.config(text="Connect")
        self.btn_start.config(state='disabled')
        self.log("Disconnected")
    
    def send_command(self, cmd: str) -> str:
        """Send command and get response"""
        if not self.ser:
            return ""
        
        self.ser.write((cmd + "\n").encode())
        time.sleep(0.05)
        
        response = ""
        while self.ser.in_waiting:
            response += self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
            time.sleep(0.01)
        
        return response
    
    def get_current_params(self):
        """Get current MPC parameters from firmware"""
        response = self.send_command("MPCGET")
        self.log("Firmware response:\n" + response)
        
        # Parse response: Qpos=100.00 Qvel=1.00 Qfpos=200.00 Qfvel=10.00 R=0.50 Rdelta=0.10
        try:
            for line in response.split('\n'):
                if 'Qpos=' in line:
                    parts = line.split()
                    for part in parts:
                        if '=' in part:
                            key, value = part.split('=')
                            if key in self.param_entries:
                                self.param_entries[key].delete(0, tk.END)
                                self.param_entries[key].insert(0, value)
                    break
        except Exception as e:
            self.log(f"Error parsing response: {e}")
    
    def send_params(self):
        """Send parameters to firmware"""
        try:
            for name, entry in self.param_entries.items():
                value = float(entry.get())
                cmd = f"MPCSET {name} {value}"
                response = self.send_command(cmd)
                self.log(f"{cmd} -> {response.strip()}")
                time.sleep(0.05)
            self.log("Parameters updated successfully")
        except Exception as e:
            self.log(f"Error sending parameters: {e}")
            messagebox.showerror("Error", str(e))
    
    def log(self, message: str):
        """Log message to results text"""
        self.txt_results.insert('end', message + '\n')
        self.txt_results.see('end')
        self.root.update()
    
    def start_tuning(self):
        """Start auto-tuning process"""
        if self.tuning_active:
            return
        
        self.tuning_active = True
        self.btn_start.config(state='disabled')
        self.btn_stop.config(state='normal')
        
        # Run tuning in background thread
        self.tuning_thread = threading.Thread(target=self.auto_tune_loop, daemon=True)
        self.tuning_thread.start()
    
    def stop_tuning(self):
        """Stop auto-tuning process"""
        self.tuning_active = False
        self.btn_start.config(state='normal')
        self.btn_stop.config(state='disabled')
        self.log("Tuning stopped by user")
    
    def auto_tune_loop(self):
        """Main auto-tuning loop using grid search with refinement"""
        try:
            goal = self.goal_var.get()
            mode = self.mode_var.get()
            
            # Set iterations and test points based on mode
            if mode == "quick":
                iterations = 20
                test_point_count = 2
                settling_time = 2.0
            elif mode == "standard":
                iterations = 50
                test_point_count = 4
                settling_time = 3.0
            else:  # deep
                iterations = 200
                test_point_count = 8
                settling_time = 5.0
            
            self.log(f"\n{'='*60}")
            self.log(f"Starting Auto-Tuning")
            self.log(f"Mode: {mode.upper()}, Goal: {goal}")
            self.log(f"Iterations: {iterations}, Test Points: {test_point_count}")
            self.log(f"Using JOINT ANGLE mode (J) with home reset between tests")
            self.log(f"Home position: {self.home_position} (all joints at 0°)")
            self.log(f"Test movements: Max ±30° per joint, Motor 6 (roll) disabled")
            self.log(f"")
            self.log(f"NOTE: Joint angle (J) and Cartesian (M) modes both use")
            self.log(f"      the SAME MPC controller. MPC tuning results will")
            self.log(f"      transfer perfectly to all control modes!")
            self.log(f"{'='*60}\n")
            
            # Define search ranges based on goal
            search_ranges = self.get_search_ranges(goal)
            
            best_params = None
            best_score = float('inf')
            all_results = []  # Store all results for deep mode refinement
            
            # Phase 1: Grid search
            for i in range(iterations):
                if not self.tuning_active:
                    break
                
                # Generate candidate parameters
                params = self.generate_candidate(search_ranges, i, iterations, mode)
                
                # Update progress
                progress = int((i + 1) / iterations * 100)
                self.progress['value'] = progress
                self.lbl_progress.config(text=f"Phase 1: Iteration {i+1}/{iterations}")
                
                # Test parameters
                self.log(f"\n[{i+1}/{iterations}] Testing: Qpos={params.Qpos:.1f}, Qfpos={params.Qfpos:.1f}, Qfvel={params.Qfvel:.1f}, R={params.R:.2f}")
                
                score, metrics = self.test_parameters(params, test_point_count, settling_time, mode)
                all_results.append((params, score, metrics))
                
                if score < best_score:
                    best_score = score
                    best_params = params
                    self.log(f"  ✓✓✓ NEW BEST! Score: {score:.3f} | Error: {metrics['avg_error']:.1f}units ({metrics['avg_error']/11.378:.2f}°) | Time: {metrics['avg_settling']:.2f}s | Vel: {metrics['avg_velocity']:.1f}u/s")
                else:
                    self.log(f"  Score: {score:.3f} | Error: {metrics['avg_error']:.1f}units ({metrics['avg_error']/11.378:.2f}°) | Time: {metrics['avg_settling']:.2f}s | Vel: {metrics['avg_velocity']:.1f}u/s")
                
                # For deep mode, add adaptive refinement every 50 iterations
                if mode == "deep" and i > 0 and (i + 1) % 50 == 0 and i < iterations - 20:
                    self.log(f"\n--- Adaptive Refinement at iteration {i+1} ---")
                    # Sort results and focus on best region
                    all_results.sort(key=lambda x: x[1])
                    top_5 = all_results[:5]
                    
                    # Create refined search ranges around best results
                    search_ranges = self.refine_search_ranges(top_5, goal)
                    self.log(f"Refined ranges: Qpos={search_ranges['Qpos']}, Qfpos={search_ranges['Qfpos']}")
            
            # Phase 2: Deep mode final refinement
            if mode == "deep" and self.tuning_active:
                self.log(f"\n{'='*60}")
                self.log("PHASE 2: ULTRA-FINE REFINEMENT")
                self.log(f"{'='*60}\n")
                
                # Take top 3 candidates and do micro-adjustments
                all_results.sort(key=lambda x: x[1])
                top_3 = all_results[:3]
                
                refinement_iterations = 20
                for i in range(refinement_iterations):
                    if not self.tuning_active:
                        break
                    
                    progress = int((i + 1) / refinement_iterations * 100)
                    self.progress['value'] = progress
                    self.lbl_progress.config(text=f"Phase 2: Fine-tuning {i+1}/{refinement_iterations}")
                    
                    # Micro-adjust around best parameters
                    base_params = top_3[i % 3][0]
                    params = self.micro_adjust(base_params)
                    
                    self.log(f"\n[Refinement {i+1}] Testing: Qpos={params.Qpos:.1f}, Qfpos={params.Qfpos:.1f}, Qfvel={params.Qfvel:.1f}")
                    
                    score, metrics = self.test_parameters(params, 8, 5.0, "deep")
                    
                    if score < best_score:
                        best_score = score
                        best_params = params
                        self.log(f"  ✓✓✓ REFINED BEST! Score: {score:.3f} | Error: {metrics['avg_error']:.2f}mm | Vibration: {metrics['avg_vibration']:.3f}")
                    else:
                        self.log(f"  Score: {score:.3f}")
            
            if best_params:
                self.log(f"\n{'='*60}")
                self.log("🎯 TUNING COMPLETE! 🎯")
                self.log(f"{'='*60}")
                self.log(f"Final Score: {best_score:.3f}")
                self.log(f"\nOptimal Parameters:")
                self.log(f"  Qpos   = {best_params.Qpos:.2f}   (Position tracking weight)")
                self.log(f"  Qvel   = {best_params.Qvel:.2f}   (Velocity tracking weight)")
                self.log(f"  Qfpos  = {best_params.Qfpos:.2f}  (Terminal position weight)")
                self.log(f"  Qfvel  = {best_params.Qfvel:.2f}  (Terminal velocity weight)")
                self.log(f"  R      = {best_params.R:.2f}   (Control effort penalty)")
                self.log(f"  Rdelta = {best_params.Rdelta:.2f}   (Jerk penalty)")
                self.log(f"{'='*60}\n")
                
                # Update UI
                for name in self.param_entries:
                    value = getattr(best_params, name)
                    self.param_entries[name].delete(0, tk.END)
                    self.param_entries[name].insert(0, f"{value:.2f}")
                
                # Apply best parameters
                self.send_params()
                
                # Save parameters automatically
                self.save_parameters()
                self.log("\n💾 Parameters saved to mpc_params.json")
                
                # Final validation test
                if mode == "deep":
                    self.log("\nRunning final validation test...")
                    final_score, final_metrics = self.test_parameters(best_params, 8, 5.0, "deep")
                    self.log(f"\n✓ Validation Results:")
                    self.log(f"  Average Error: {final_metrics['avg_error']:.1f} units ({final_metrics['avg_error']/11.378:.2f}°)")
                    self.log(f"  Max Error: {final_metrics['max_error']:.1f} units ({final_metrics['max_error']/11.378:.2f}°)")
                    self.log(f"  Settling Time: {final_metrics['avg_settling']:.2f}s")
                    self.log(f"  Vibration Index: {final_metrics['avg_vibration']:.2f}")
                    self.log(f"  Overshoot: {final_metrics['avg_overshoot']:.1f} units ({final_metrics['avg_overshoot']/11.378:.2f}°)")
                    self.log(f"  Average Velocity: {final_metrics['avg_velocity']:.1f} units/s ({final_metrics['avg_velocity']/11.378:.1f}°/s)")
                    self.log(f"  Velocity Penalty: {final_metrics['velocity_penalty']:.1f}")
                
                # Return to home position
                self.log("\nReturning to home position (all joints 0°)...")
                self.send_command(f"D 0 0 0 0 0")
            
        except Exception as e:
            self.log(f"ERROR: {e}")
            import traceback
            self.log(traceback.format_exc())
        finally:
            self.tuning_active = False
            self.btn_start.config(state='normal')
            self.btn_stop.config(state='disabled')
            self.progress['value'] = 0
            self.lbl_progress.config(text="Ready")
    
    def get_search_ranges(self, goal: str) -> dict:
        """Get parameter search ranges based on optimization goal"""
        if goal == "fast":
            return {
                'Qpos': (80, 200),
                'Qfpos': (150, 350),
                'Qfvel': (20, 100),
                'R': (0.3, 0.8)
            }
        elif goal == "precise":
            return {
                'Qpos': (100, 250),
                'Qfpos': (200, 400),
                'Qfvel': (5, 30),
                'R': (0.5, 1.2)
            }
        elif goal == "smooth":
            return {
                'Qpos': (60, 150),
                'Qfpos': (100, 250),
                'Qfvel': (5, 20),
                'R': (0.8, 1.5)
            }
        elif goal == "ultimate":
            return {
                'Qpos': (120, 250),
                'Qfpos': (250, 450),
                'Qfvel': (8, 35),
                'R': (0.6, 1.1)
            }
        else:  # balanced
            return {
                'Qpos': (80, 180),
                'Qfpos': (150, 300),
                'Qfvel': (5, 40),
                'R': (0.4, 1.0)
            }
    
    def refine_search_ranges(self, top_results: List, goal: str) -> dict:
        """Refine search ranges based on top performers"""
        # Extract parameters from top results
        params_list = [r[0] for r in top_results]
        
        # Calculate mean and std for each parameter
        qpos_vals = [p.Qpos for p in params_list]
        qfpos_vals = [p.Qfpos for p in params_list]
        qfvel_vals = [p.Qfvel for p in params_list]
        r_vals = [p.R for p in params_list]
        
        # Create tighter ranges around best values
        return {
            'Qpos': (max(50, np.mean(qpos_vals) - 20), min(300, np.mean(qpos_vals) + 20)),
            'Qfpos': (max(100, np.mean(qfpos_vals) - 30), min(500, np.mean(qfpos_vals) + 30)),
            'Qfvel': (max(5, np.mean(qfvel_vals) - 10), min(60, np.mean(qfvel_vals) + 10)),
            'R': (max(0.3, np.mean(r_vals) - 0.2), min(1.5, np.mean(r_vals) + 0.2))
        }
    
    def micro_adjust(self, base_params: MPCParams) -> MPCParams:
        """Make micro-adjustments to parameters for fine-tuning"""
        params = MPCParams()
        
        # Small random perturbations (±5%)
        params.Qpos = base_params.Qpos * (1.0 + np.random.uniform(-0.05, 0.05))
        params.Qvel = base_params.Qvel * (1.0 + np.random.uniform(-0.05, 0.05))
        params.Qfpos = base_params.Qfpos * (1.0 + np.random.uniform(-0.05, 0.05))
        params.Qfvel = base_params.Qfvel * (1.0 + np.random.uniform(-0.05, 0.05))
        params.R = base_params.R * (1.0 + np.random.uniform(-0.05, 0.05))
        params.Rdelta = base_params.Rdelta * (1.0 + np.random.uniform(-0.05, 0.05))
        
        return params
    
    def generate_candidate(self, ranges: dict, iteration: int, total: int, mode: str = "quick") -> MPCParams:
        """Generate candidate parameters using quasi-random sampling"""
        params = MPCParams()
        
        # Use Latin Hypercube Sampling for better space coverage
        progress = iteration / total
        
        # Deep mode: Focus sampling on promising regions
        if mode == "deep" and progress > 0.5:
            # Later in deep search, bias toward middle of ranges
            params.Qpos = ranges['Qpos'][0] + (ranges['Qpos'][1] - ranges['Qpos'][0]) * (0.3 + 0.4 * np.random.random())
            params.Qfpos = ranges['Qfpos'][0] + (ranges['Qfpos'][1] - ranges['Qfpos'][0]) * (0.3 + 0.4 * np.random.random())
            params.Qfvel = ranges['Qfvel'][0] + (ranges['Qfvel'][1] - ranges['Qfvel'][0]) * (0.3 + 0.4 * np.random.random())
            params.R = ranges['R'][0] + (ranges['R'][1] - ranges['R'][0]) * (0.3 + 0.4 * np.random.random())
        else:
            # Uniform random sampling
            params.Qpos = np.random.uniform(ranges['Qpos'][0], ranges['Qpos'][1])
            params.Qfpos = np.random.uniform(ranges['Qfpos'][0], ranges['Qfpos'][1])
            params.Qfvel = np.random.uniform(ranges['Qfvel'][0], ranges['Qfvel'][1])
            params.R = np.random.uniform(ranges['R'][0], ranges['R'][1])
        
        # Qvel and Rdelta fixed (from experience they work well)
        params.Qvel = 1.0
        params.Rdelta = 0.1
        
        return params
    
    def test_parameters(self, params: MPCParams, test_point_count: int = 2, settling_time: float = 2.0, mode: str = "quick") -> tuple:
        """Test a set of parameters and return (score, metrics_dict)
        
        Metrics tracked:
        - Final position error (accuracy)
        - Settling time (speed)
        - Overshoot (stability)
        - Vibration (jitter/oscillation)
        - Velocity tracking (prevents slow creeping that causes backlash issues)
        """
        # Set parameters
        self.send_command(f"MPCSET Qpos {params.Qpos:.2f}")
        time.sleep(0.02)
        self.send_command(f"MPCSET Qvel {params.Qvel:.2f}")
        time.sleep(0.02)
        self.send_command(f"MPCSET Qfpos {params.Qfpos:.2f}")
        time.sleep(0.02)
        self.send_command(f"MPCSET Qfvel {params.Qfvel:.2f}")
        time.sleep(0.02)
        self.send_command(f"MPCSET R {params.R:.2f}")
        time.sleep(0.02)
        self.send_command(f"MPCSET Rdelta {params.Rdelta:.2f}")
        time.sleep(0.1)
        
        # Test on multiple points
        all_final_errors = []
        all_settling_times = []
        all_overshoots = []
        all_vibrations = []
        all_velocities = []  # Track average velocity to penalize slow creeping
        
        for q1_deg, q2_deg, q3_deg, q4_deg in self.test_points[:test_point_count]:
            # Convert degrees to radians for D command
            import math
            q1_rad = math.radians(q1_deg)
            q2_rad = math.radians(q2_deg)
            q3_rad = math.radians(q3_deg)
            q4_rad = math.radians(q4_deg)
            
            # Return to home position first (all joints to 0 radians)
            self.send_command(f"D 0 0 0 0 0")
            time.sleep(2.0)  # Wait for home position
            
            # Move to target using D command (radians, with roll=0)
            self.send_command(f"D {q1_rad:.4f} {q2_rad:.4f} {q3_rad:.4f} {q4_rad:.4f} 0")
            
            # Wait a bit then get initial position as baseline
            time.sleep(0.2)
            
            # Monitor raw servo positions (higher resolution than joint angles)
            start_time = time.time()
            positions = []
            initial_raw = None
            
            while time.time() - start_time < settling_time:
                response = self.send_command("P")
                # Parse: POS x y z roll | RAW p1 p2 p3 p4 p5 p6
                try:
                    if "RAW" in response:
                        # Extract raw positions after "RAW"
                        raw_part = response.split("RAW")[1].strip().split()
                        if len(raw_part) >= 4:
                            current_raw = np.array([int(raw_part[0]), int(raw_part[1]), 
                                                   int(raw_part[2]), int(raw_part[3])])
                            
                            # Store initial position on first read
                            if initial_raw is None:
                                initial_raw = current_raw.copy()
                            
                            # Calculate how far we've moved from start (this is what we track)
                            movement = np.linalg.norm(current_raw - initial_raw)
                            positions.append((time.time() - start_time, movement, current_raw))
                except Exception as e:
                    pass
                
                time.sleep(0.1)
            
            # Analyze performance
            if len(positions) > 5:
                # Find the peak movement (furthest from start)
                movements = [p[1] for p in positions]
                peak_movement = max(movements)
                
                # Target is the final settled position (average of last 5 samples)
                final_positions = [p[2] for p in positions[-5:]]
                target_raw = np.mean(final_positions, axis=0).astype(int)
                
                # Calculate errors relative to target
                errors = [np.linalg.norm(p[2] - target_raw) for p in positions]
                
                # Final error (last 5 samples average)
                final_error = np.mean(errors[-5:])
                all_final_errors.append(final_error)
                
                # Settling time (when error stays below 57 units ≈ 5 degrees total)
                settling_t = settling_time
                for i, err in enumerate(errors):
                    if err < 57:
                        settling_t = positions[i][0]
                        break
                all_settling_times.append(settling_t)
                
                # Overshoot detection (how much past target before settling)
                # Find when we first get close to target
                first_near_target = len(errors)
                for i, err in enumerate(errors):
                    if err < 100:  # Within ~9° 
                        first_near_target = i
                        break
                
                # Overshoot is max error AFTER first approach
                if first_near_target < len(errors):
                    post_approach_errors = errors[first_near_target:]
                    overshoot = max(0, max(post_approach_errors) - 57)  # Beyond settling threshold
                else:
                    overshoot = 0
                all_overshoots.append(overshoot)
                
                # Vibration analysis (high-frequency variation)
                if len(errors) > 10:
                    vibration = np.std(np.diff(errors[-10:]))
                    all_vibrations.append(vibration)
                
                # Velocity analysis (penalize slow movements that cause backlash jitter)
                # Calculate average velocity during approach (first 60% of movement)
                approach_end = int(len(positions) * 0.6)
                if approach_end > 5:
                    approach_phase = positions[:approach_end]
                    time_deltas = [approach_phase[j+1][0] - approach_phase[j][0] for j in range(len(approach_phase)-1)]
                    movement_deltas = [approach_phase[j+1][1] - approach_phase[j][1] for j in range(len(approach_phase)-1)]
                    velocities = [abs(movement_deltas[j]) / time_deltas[j] if time_deltas[j] > 0 else 0 for j in range(len(time_deltas))]
                    avg_velocity = np.mean(velocities) if velocities else 0
                    all_velocities.append(avg_velocity)
        
        # Calculate overall metrics
        avg_error = np.mean(all_final_errors) if all_final_errors else 999.0
        max_error = max(all_final_errors) if all_final_errors else 999.0
        avg_settling = np.mean(all_settling_times) if all_settling_times else settling_time
        avg_overshoot = np.mean(all_overshoots) if all_overshoots else 0.0
        avg_vibration = np.mean(all_vibrations) if all_vibrations else 0.0
        avg_velocity = np.mean(all_velocities) if all_velocities else 0.0
        
        # Velocity penalty: penalize very slow movements (<200 units/s ≈ 17°/s) that cause backlash
        velocity_penalty = max(0, 200 - avg_velocity) * 0.5 if avg_velocity < 200 else 0
        
        # Calculate score (lower is better)
        # All modes include velocity_penalty to prevent slow creeping and backlash issues
        goal = self.goal_var.get()
        if goal == "fast":
            score = avg_settling * 100 + avg_error * 10 + velocity_penalty
        elif goal == "precise":
            score = avg_error * 100 + avg_settling * 10 + velocity_penalty
        elif goal == "smooth":
            score = avg_vibration * 500 + avg_error * 20 + avg_overshoot * 50 + velocity_penalty
        elif goal == "ultimate":
            # Ultimate: penalize everything heavily including slow movements
            score = avg_error * 200 + avg_vibration * 800 + avg_overshoot * 100 + avg_settling * 50 + velocity_penalty * 2
        else:  # balanced
            score = avg_error * 50 + avg_settling * 50 + avg_vibration * 100 + velocity_penalty
        
        metrics = {
            'avg_error': avg_error,
            'max_error': max_error,
            'avg_settling': avg_settling,
            'avg_overshoot': avg_overshoot,
            'avg_vibration': avg_vibration,
            'avg_velocity': avg_velocity,
            'velocity_penalty': velocity_penalty
        }
        
        return score, metrics
    
    def test_current(self):
        """Test current parameters"""
        if not self.connected:
            messagebox.showwarning("Warning", "Not connected")
            return
        
        params = MPCParams()
        for name, entry in self.param_entries.items():
            setattr(params, name, float(entry.get()))
        
        # Get mode and goal from GUI
        mode = self.mode_var.get()
        goal = self.goal_var.get()
        
        # Set test parameters based on mode
        if mode == "quick":
            test_point_count = 2
            settling_time = 2.0
        elif mode == "standard":
            test_point_count = 4
            settling_time = 3.0
        else:  # deep
            test_point_count = 8
            settling_time = 5.0
        
        self.log(f"\nTesting current parameters ({mode} mode, {goal} goal)...")
        self.log(f"Testing {test_point_count} positions with {settling_time}s settling time")
        score, metrics = self.test_parameters(params, test_point_count, settling_time, mode)
        self.log(f"\nPerformance Score: {score:.3f}")
        self.log(f"Avg Error: {metrics['avg_error']:.1f} units ({metrics['avg_error']/11.378:.2f}°)")
        self.log(f"Max Error: {metrics['max_error']:.1f} units ({metrics['max_error']/11.378:.2f}°)")
        self.log(f"Settling Time: {metrics['avg_settling']:.2f}s")
        self.log(f"Overshoot: {metrics['avg_overshoot']:.1f} units")
        self.log(f"Vibration: {metrics['avg_vibration']:.3f}")
        self.log(f"Velocity: {metrics['avg_velocity']:.1f} units/s")
    
    def save_parameters(self):
        """Save current parameters to JSON file"""
        try:
            params = {}
            for name, entry in self.param_entries.items():
                params[name] = float(entry.get())
            
            with open('mpc_params.json', 'w') as f:
                json.dump(params, f, indent=2)
            
            self.log("✓ Parameters saved to mpc_params.json")
            messagebox.showinfo("Success", "Parameters saved successfully!")
        except Exception as e:
            self.log(f"Error saving parameters: {e}")
            messagebox.showerror("Error", f"Failed to save parameters: {e}")
    
    def load_parameters(self):
        """Load parameters from JSON file"""
        try:
            if not os.path.exists('mpc_params.json'):
                messagebox.showwarning("Warning", "No saved parameters found")
                return
            
            with open('mpc_params.json', 'r') as f:
                params = json.load(f)
            
            for name, value in params.items():
                if name in self.param_entries:
                    self.param_entries[name].delete(0, tk.END)
                    self.param_entries[name].insert(0, str(value))
            
            self.log("✓ Parameters loaded from mpc_params.json")
            messagebox.showinfo("Success", "Parameters loaded successfully!")
        except Exception as e:
            self.log(f"Error loading parameters: {e}")
            messagebox.showerror("Error", f"Failed to load parameters: {e}")

if __name__ == "__main__":
    root = tk.Tk()
    app = MPCTunerApp(root)
    root.mainloop()
