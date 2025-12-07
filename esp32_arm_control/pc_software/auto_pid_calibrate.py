#!/usr/bin/env python3
"""
Auto PID Calibration Script for 5DOF Robotic Arm
Tests various PID combinations and measures settling time/overshoot.
Motors 2 and 3 share the same joint (shoulder) so they get the same PID values.
"""

import serial
import serial.tools.list_ports
import time
import math
import json
import sys
from dataclasses import dataclass
from typing import List, Tuple, Optional
import threading
from queue import Queue


# Motor centers (from your config)
MOTOR_CENTERS = [2207, 2617, 2771, 2563, 2160, 2047]

# Test motion amplitude in radians (~29 degrees) - larger for more stress
TEST_AMPLITUDE = 0.5

# Joint names
JOINT_NAMES = ["Base", "Shoulder", "Elbow", "Wrist Pitch", "Roll"]


class PIDCalibrator:
    def __init__(self, port: str, baudrate: int = 921600):
        print(f"Connecting to {port}...")
        # Simple connection like the GUI does - ESP32 uses 921600 baud
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        time.sleep(0.5)  # Brief pause
        
        self.running = True
        self.current_joints = [0.0] * 5
        self.raw_positions = MOTOR_CENTERS.copy()  # Start at centers
        self.connected = False
        
        # Start reader thread
        self.reader_thread = threading.Thread(target=self._read_serial, daemon=True)
        self.reader_thread.start()
        
        # Wait for telemetry
        print("Waiting for telemetry...")
        for i in range(30):  # 3 second timeout
            time.sleep(0.1)
            # Check if raw positions differ from centers (meaning we got data)
            if self.raw_positions != MOTOR_CENTERS:
                self.connected = True
                print(f"Connected! Raw positions: {self.raw_positions}")
                break
            # Print dots for progress
            print(".", end="", flush=True)
        
        print()
        if not self.connected:
            print("WARNING: No telemetry received. Continuing anyway...")
            print("Make sure the GUI is closed!")
    
    def _read_serial(self):
        """Background thread to read serial data"""
        buffer = ""
        msg_count = 0
        while self.running:
            try:
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            msg_count += 1
                            if msg_count <= 5:
                                print(f"  [RX] {line[:60]}")
                            self._process_message(line)
                else:
                    time.sleep(0.005)
            except Exception as e:
                time.sleep(0.01)
    
    def _process_message(self, msg: str):
        """Process incoming serial message"""
        # Format: "POS x y z roll | RAW r1 r2 r3 r4 r5 r6"
        if "RAW" in msg:
            try:
                # Find RAW part
                idx = msg.find("RAW")
                raw_part = msg[idx+3:].strip()
                values = raw_part.split()
                if len(values) >= 6:
                    self.raw_positions = [int(v) for v in values[:6]]
                    # Convert to radians
                    self.current_joints[0] = (self.raw_positions[0] - MOTOR_CENTERS[0]) * 2 * math.pi / 4096
                    self.current_joints[1] = -(self.raw_positions[1] - MOTOR_CENTERS[1]) * 2 * math.pi / 4096  # NEGATED
                    self.current_joints[2] = (self.raw_positions[3] - MOTOR_CENTERS[3]) * 2 * math.pi / 4096
                    self.current_joints[3] = (self.raw_positions[4] - MOTOR_CENTERS[4]) * 2 * math.pi / 4096
                    self.current_joints[4] = (self.raw_positions[5] - MOTOR_CENTERS[5]) * 2 * math.pi / 4096
            except:
                pass
    
    def send(self, cmd: str):
        """Send command to ESP32"""
        self.ser.write(f"{cmd}\n".encode())
        print(f"  >> {cmd}")
        time.sleep(0.03)
    
    def stop(self):
        """Stop all motion"""
        self.send("S")
        time.sleep(0.1)
    
    def set_pid(self, joint_id: int, kp: float, ki: float, kd: float):
        """Set PID for a joint. Motor 2&3 are linked."""
        self.send(f"P {joint_id} {kp:.2f} {ki:.2f} {kd:.2f}")
        if joint_id == 2:
            self.send(f"P 3 {kp:.2f} {ki:.2f} {kd:.2f}")
    
    def move_to_center(self):
        """Move all joints to center"""
        print("  Moving to center...")
        self.send("D 0.0 0.0 0.0 0.0 0.0")
        # Wait for convergence
        for _ in range(30):
            time.sleep(0.1)
            max_err = max(abs(j) for j in self.current_joints)
            if max_err < 0.05:
                break
        time.sleep(0.3)
    
    def test_step(self, joint_idx: int, amplitude: float, max_duration: float = 4.0) -> Tuple[float, float, float, int]:
        """
        Test step response for one joint using RAW servo positions.
        Waits for actual settling before measuring oscillations.
        Returns: (settling_time, overshoot%, steady_state_error_steps, oscillation_count)
        """
        # Map joint index to raw_positions index
        # Joint 0=Base→raw[0], 1=Shoulder→raw[1], 2=Elbow→raw[3], 3=WristP→raw[4], 4=Roll→raw[5]
        raw_idx_map = [0, 1, 3, 4, 5]
        raw_idx = raw_idx_map[joint_idx]
        
        # Get current RAW position and calculate target
        initial_raw = self.raw_positions[raw_idx]
        # Convert amplitude (radians) to steps: 4096 steps = 2*pi radians
        amplitude_steps = int(amplitude * 4096 / (2 * math.pi))
        # Account for motor 2 being negated
        if joint_idx == 1:
            amplitude_steps = -amplitude_steps
        target_raw = initial_raw + amplitude_steps
        
        # Build command using radians for the ESP32
        start_joints = self.current_joints.copy()
        cmd_joints = start_joints.copy()
        cmd_joints[joint_idx] = start_joints[joint_idx] + amplitude
        cmd = f"D {cmd_joints[0]:.4f} {cmd_joints[1]:.4f} {cmd_joints[2]:.4f} {cmd_joints[3]:.4f} {cmd_joints[4]:.4f}"
        
        # Record trajectory using RAW positions
        raw_trajectory = []
        start_time = time.time()
        self.send(cmd)
        
        # ALWAYS record for a minimum time to capture oscillations
        min_record_time = 2.5  # Minimum recording time
        settle_threshold = 2  # steps - stricter threshold
        settled_count = 0
        settled_required = 50  # Need 50 consecutive samples (0.5s) of being still
        
        while time.time() - start_time < max_duration:
            raw_pos = self.raw_positions[raw_idx]
            elapsed = time.time() - start_time
            raw_trajectory.append((elapsed, raw_pos))
            
            # Only check for early exit AFTER minimum recording time
            if elapsed > min_record_time and len(raw_trajectory) >= 10:
                recent_range = max(r for t,r in raw_trajectory[-15:]) - min(r for t,r in raw_trajectory[-15:])
                if recent_range <= settle_threshold:
                    settled_count += 1
                    if settled_count >= settled_required:
                        # Record a bit more after settling
                        extra_time = time.time()
                        while time.time() - extra_time < 0.8:
                            raw_pos = self.raw_positions[raw_idx]
                            raw_trajectory.append((time.time() - start_time, raw_pos))
                            time.sleep(0.01)
                        break
                else:
                    settled_count = 0
            
            time.sleep(0.01)
        
        # Analyze using RAW positions
        if len(raw_trajectory) < 20:
            return (999.0, 999.0, 999.0, 0)
        
        # Final position (average of last samples)
        final_raw = sum(r for t, r in raw_trajectory[-30:]) / min(30, len(raw_trajectory))
        ss_error_steps = abs(target_raw - final_raw)
        
        # Convert to radians for error reporting
        ss_error_rad = ss_error_steps * 2 * math.pi / 4096
        
        # Overshoot - find peak in direction of movement
        raw_values = [r for t, r in raw_trajectory]
        if amplitude_steps > 0:
            peak = max(raw_values)
            overshoot_steps = max(0, peak - target_raw)
        else:
            peak = min(raw_values)
            overshoot_steps = max(0, target_raw - peak)
        
        overshoot_pct = (overshoot_steps / abs(amplitude_steps)) * 100 if amplitude_steps != 0 else 0
        
        # Settling time (when position stays within 5% of target)
        tolerance_steps = abs(amplitude_steps) * 0.05
        settling = raw_trajectory[-1][0]  # Default to full duration
        for i in range(len(raw_trajectory) - 1, -1, -1):
            if abs(raw_trajectory[i][1] - target_raw) > tolerance_steps:
                if i < len(raw_trajectory) - 1:
                    settling = raw_trajectory[i + 1][0]
                break
        
        # IMPROVED Oscillation detection using smoothed velocity zero-crossings
        # First, find when we've reached the target area
        approach_done_idx = 0
        for i, (t, r) in enumerate(raw_trajectory):
            if abs(r - target_raw) < abs(amplitude_steps) * 0.4:
                approach_done_idx = i
                break
        
        # Calculate velocities after approach
        velocities = []
        analysis_data = raw_trajectory[approach_done_idx:]
        if len(analysis_data) > 5:
            for i in range(2, len(analysis_data) - 2):
                # Use centered difference for smoother velocity estimate
                dt = analysis_data[i+2][0] - analysis_data[i-2][0]
                if dt > 0:
                    dp = analysis_data[i+2][1] - analysis_data[i-2][1]
                    velocities.append((analysis_data[i][0], dp / dt))
        
        # Count velocity sign changes (zero crossings)
        zero_crossings = 0
        if len(velocities) > 10:
            # Skip first few to avoid initial transient
            start_v_idx = len(velocities) // 8
            prev_sign = None
            for t, v in velocities[start_v_idx:]:
                if abs(v) > 3:  # Velocity threshold (steps/sec)
                    curr_sign = 1 if v > 0 else -1
                    if prev_sign is not None and curr_sign != prev_sign:
                        zero_crossings += 1
                    prev_sign = curr_sign
        
        # Also count peaks/valleys as backup method
        peaks = 0
        if len(analysis_data) > 10:
            min_peak_height = 3  # Minimum 3 steps for a peak
            for i in range(5, len(analysis_data) - 5):
                p = analysis_data[i][1]
                # Check if local maximum
                window_before = [analysis_data[j][1] for j in range(i-5, i)]
                window_after = [analysis_data[j][1] for j in range(i+1, i+6)]
                if p > max(window_before) + min_peak_height and p > max(window_after) + min_peak_height:
                    peaks += 1
                # Check if local minimum
                if p < min(window_before) - min_peak_height and p < min(window_after) - min_peak_height:
                    peaks += 1
        
        # Use whichever method detected more oscillations
        oscillations = max(zero_crossings // 2, peaks // 2)
        
        # Debug output
        raw_range = max(raw_values) - min(raw_values)
        duration = raw_trajectory[-1][0]
        print(f"    [Raw] dur={duration:.1f}s range={raw_range} steps, target={target_raw}, zcross={zero_crossings}, peaks={peaks}, osc={oscillations}")
        
        return (settling, overshoot_pct, ss_error_rad, oscillations)
    
    def score(self, settling: float, overshoot: float, error: float, oscillations: int) -> float:
        """
        Calculate score (lower = better)
        Balanced priorities - overshoot matters too!
        """
        # Overshoot penalty - scales up for high overshoot
        # 0-15%: acceptable, 15-30%: moderate penalty, 30%+: heavy penalty
        if overshoot <= 15:
            os_penalty = overshoot * 0.3
        elif overshoot <= 30:
            os_penalty = 4.5 + (overshoot - 15) * 0.6
        else:
            os_penalty = 13.5 + (overshoot - 30) * 1.0  # Heavy penalty above 30%
        
        # Oscillation penalty  
        osc_penalty = 8.0 * oscillations + (5.0 if oscillations >= 2 else 0)
        
        # Error penalty (in degrees)
        err_penalty = 5.0 * error * 57.3
        
        # Settling time penalty
        settle_penalty = 1.5 * settling
        
        return settle_penalty + os_penalty + err_penalty + osc_penalty
    
    def test_full_cycle(self, joint_idx: int) -> Tuple[float, float, float, int]:
        """
        Test a full movement cycle: out -> center -> other way -> center
        Returns average metrics across all 4 movements
        """
        # Test positive step (away from center)
        s1, o1, e1, osc1 = self.test_step(joint_idx, TEST_AMPLITUDE)
        
        # Test return to center - WHERE GRAVITY FIGHTS PID!
        s2, o2, e2, osc2 = self.test_step(joint_idx, -TEST_AMPLITUDE)
        
        # Test negative step (other direction from center)
        s3, o3, e3, osc3 = self.test_step(joint_idx, -TEST_AMPLITUDE)
        
        # Test return to center from negative
        s4, o4, e4, osc4 = self.test_step(joint_idx, TEST_AMPLITUDE)
        
        # Average metrics, use MAX oscillations (worst case)
        settling = (s1 + s2 + s3 + s4) / 4
        overshoot = (o1 + o2 + o3 + o4) / 4
        error = (e1 + e2 + e3 + e4) / 4
        oscillations = max(osc1, osc2, osc3, osc4)
        
        return settling, overshoot, error, oscillations
    
    def find_critical_gain(self, joint_idx: int, joint_id: int) -> Tuple[float, float]:
        """
        Coarse-to-fine grid search over Kp, Ki, Kd combinations.
        Phase 1: Coarse grid to find promising regions
        Phase 2: Fine grid around top candidates
        
        Returns (best_kp, best_ki, best_kd)
        """
        print("\n  === PHASE 1: COARSE GRID SEARCH ===")
        print("    Finding promising regions with wide steps...")
        
        # Coarse grid - adjusted for our custom PID controller
        # Higher Kp = faster response (servos work better at speed)
        # Lower Ki than firmware (different scaling in our controller)
        if joint_idx in [0, 1, 2]:  # Base, Shoulder, Elbow - heavy joints
            kp_coarse = [8.0, 12.0, 16.0, 20.0, 25.0]  # Higher Kp range, skip sluggish 4
            ki_coarse = [0.0, 0.5, 1.0, 2.0, 4.0, 6.0]  # Wider Ki range
            kd_coarse = [0.0, 0.5, 1.0]
        else:  # Wrist, Roll - lighter joints  
            kp_coarse = [6.0, 10.0, 14.0, 18.0]
            ki_coarse = [0.0, 0.5, 1.0, 2.0, 3.0]
            kd_coarse = [0.0, 0.3, 0.6]
        
        all_results = []
        total = len(kp_coarse) * len(ki_coarse) * len(kd_coarse)
        tested = 0
        
        print(f"    Testing {total} coarse combinations...\n")
        
        for kp in kp_coarse:
            for ki in ki_coarse:
                for kd in kd_coarse:
                    tested += 1
                    
                    self.set_pid(joint_id, kp, ki, kd)
                    time.sleep(0.08)
                    self.move_to_center()
                    
                    # Quick 2-movement test
                    s1, o1, e1, osc1 = self.test_step(joint_idx, TEST_AMPLITUDE)
                    s2, o2, e2, osc2 = self.test_step(joint_idx, -TEST_AMPLITUDE)
                    
                    settling = (s1 + s2) / 2
                    overshoot = (o1 + o2) / 2
                    error = (e1 + e2) / 2
                    oscillations = max(osc1, osc2)
                    
                    sc = self.score(settling, overshoot, error, oscillations)
                    all_results.append((kp, ki, kd, sc, settling, overshoot, error, oscillations))
                    
                    status = "OSC!" if oscillations >= 2 else ("GOOD" if oscillations == 0 else "ok")
                    print(f"    [{tested:2d}/{total}] P={kp:5.1f} I={ki:5.1f} D={kd:4.2f}: "
                          f"score={sc:6.1f} settle={settling:.2f}s os={overshoot:3.0f}% err={math.degrees(error):4.1f}° osc={oscillations} [{status}]")
        
        # Sort and get top 3 candidates
        all_results.sort(key=lambda x: x[3])
        top_candidates = all_results[:3]
        
        print(f"\n    === TOP 3 FROM COARSE SEARCH ===")
        for i, (kp, ki, kd, sc, st, os, er, osc) in enumerate(top_candidates):
            print(f"    #{i+1}: P={kp:.1f} I={ki:.1f} D={kd:.2f} -> score={sc:.1f}")
        
        # Phase 2: Fine search around top candidates
        print(f"\n  === PHASE 2: FINE GRID SEARCH ===")
        print("    Searching around top candidates with small steps...")
        
        fine_results = []
        
        for rank, (kp_center, ki_center, kd_center, _, _, _, _, _) in enumerate(top_candidates):
            print(f"\n    --- Refining around P={kp_center:.1f} I={ki_center:.1f} D={kd_center:.2f} ---")
            
            # Fine grid around this candidate (+/- with small steps)
            kp_fine = [kp_center + delta for delta in [-2.0, -1.0, -0.5, 0, 0.5, 1.0, 2.0]]
            ki_fine = [ki_center + delta for delta in [-2.0, -1.0, -0.5, 0, 0.5, 1.0, 2.0]]
            kd_fine = [kd_center + delta for delta in [-0.3, -0.15, 0, 0.15, 0.3]]
            
            # Filter out negatives
            kp_fine = [x for x in kp_fine if x >= 1.0]
            ki_fine = [x for x in ki_fine if x >= 0.0]
            kd_fine = [x for x in kd_fine if x >= 0.0]
            
            # Remove duplicates and sort
            kp_fine = sorted(set(kp_fine))
            ki_fine = sorted(set(ki_fine))
            kd_fine = sorted(set(kd_fine))
            
            fine_total = len(kp_fine) * len(ki_fine) * len(kd_fine)
            fine_tested = 0
            
            for kp in kp_fine:
                for ki in ki_fine:
                    for kd in kd_fine:
                        fine_tested += 1
                        
                        self.set_pid(joint_id, kp, ki, kd)
                        time.sleep(0.08)
                        self.move_to_center()
                        
                        s1, o1, e1, osc1 = self.test_step(joint_idx, TEST_AMPLITUDE)
                        s2, o2, e2, osc2 = self.test_step(joint_idx, -TEST_AMPLITUDE)
                        
                        settling = (s1 + s2) / 2
                        overshoot = (o1 + o2) / 2
                        error = (e1 + e2) / 2
                        oscillations = max(osc1, osc2)
                        
                        sc = self.score(settling, overshoot, error, oscillations)
                        fine_results.append((kp, ki, kd, sc, settling, overshoot, error, oscillations))
                        
                        status = "OSC!" if oscillations >= 2 else ("GOOD" if oscillations == 0 else "ok")
                        # Only print every few or if it's good
                        if fine_tested % 10 == 0 or oscillations == 0 or sc < 20:
                            print(f"      [{fine_tested:3d}/{fine_total}] P={kp:5.2f} I={ki:5.2f} D={kd:5.2f}: "
                                  f"score={sc:6.1f} osc={oscillations} [{status}]")
        
        # Combine and find best
        fine_results.sort(key=lambda x: x[3])
        
        print(f"\n    === TOP 5 OVERALL (FINE SEARCH) ===")
        for i, (kp, ki, kd, sc, st, os, er, osc) in enumerate(fine_results[:5]):
            print(f"    #{i+1}: P={kp:.2f} I={ki:.2f} D={kd:.2f} -> score={sc:.1f} (settle={st:.2f}s, os={os:.0f}%, err={math.degrees(er):.1f}°, osc={osc})")
        
        best = fine_results[0]
        best_gains = (best[0], best[1], best[2])
        
        print(f"\n    >>> Best: Kp={best_gains[0]:.2f}, Ki={best_gains[1]:.2f}, Kd={best_gains[2]:.2f}")
        
        return best_gains

    def tune_from_critical(self, joint_idx: int, joint_id: int, initial_gains: tuple) -> Tuple[float, float, float]:
        """
        Final validation with full 4-movement cycle and ultra-fine tuning.
        """
        Kp_init, Ki_init, Kd_init = initial_gains
        
        print(f"\n  === PHASE 3: FINAL VALIDATION & ULTRA-FINE TUNING ===")
        print(f"    Starting from P={Kp_init:.2f}, I={Ki_init:.2f}, D={Kd_init:.2f}")
        
        # Full 4-movement test on best candidate
        self.set_pid(joint_id, Kp_init, Ki_init, Kd_init)
        self.move_to_center()
        s, o, e, osc = self.test_full_cycle(joint_idx)
        best_score = self.score(s, o, e, osc)
        best_gains = (Kp_init, Ki_init, Kd_init)
        print(f"    Full test: score={best_score:.2f} (settle={s:.2f}s, os={o:.0f}%, err={math.degrees(e):.1f}°, osc={osc})")
        
        # Ultra-fine tuning - tiny adjustments
        print("\n    Ultra-fine tuning (±0.25 adjustments)...")
        
        improved = True
        iteration = 0
        while improved and iteration < 3:
            improved = False
            iteration += 1
            
            Kp_best, Ki_best, Kd_best = best_gains
            
            # Try tiny adjustments to each parameter
            for param_name, adjustments in [
                ("Kp", [-0.5, -0.25, 0.25, 0.5]),
                ("Ki", [-0.5, -0.25, 0.25, 0.5]),
                ("Kd", [-0.15, -0.08, 0.08, 0.15])
            ]:
                for adj in adjustments:
                    if param_name == "Kp":
                        kp, ki, kd = max(1.0, Kp_best + adj), Ki_best, Kd_best
                    elif param_name == "Ki":
                        kp, ki, kd = Kp_best, max(0.0, Ki_best + adj), Kd_best
                    else:
                        kp, ki, kd = Kp_best, Ki_best, max(0.0, Kd_best + adj)
                    
                    self.set_pid(joint_id, kp, ki, kd)
                    self.move_to_center()
                    s, o, e, osc = self.test_full_cycle(joint_idx)
                    sc = self.score(s, o, e, osc)
                    
                    if sc < best_score - 0.5:  # Must be notably better
                        print(f"      {param_name}={kp if param_name=='Kp' else ki if param_name=='Ki' else kd:.2f}: "
                              f"score={sc:.2f} (was {best_score:.2f}) <<<IMPROVED")
                        best_score = sc
                        best_gains = (kp, ki, kd)
                        improved = True
        
        print(f"\n    Final: P={best_gains[0]:.2f} I={best_gains[1]:.2f} D={best_gains[2]:.2f} (score={best_score:.2f})")
        return best_gains
    
    def calibrate_joint(self, joint_idx: int):
        """
        Calibrate one joint using brute-force grid search + local refinement.
        1. Grid search over Kp, Ki, Kd combinations
        2. Fine-tune around the best found
        """
        joint_id = joint_idx + 1
        name = JOINT_NAMES[joint_idx]
        
        print(f"\n{'='*50}")
        print(f"Calibrating Joint {joint_id}: {name}")
        print(f"{'='*50}")
        
        # Grid search to find good starting point
        initial_gains = self.find_critical_gain(joint_idx, joint_id)
        
        # Fine-tune around best values
        best_gains = self.tune_from_critical(joint_idx, joint_id, initial_gains)
        
        # Apply best gains
        self.set_pid(joint_id, *best_gains)
        self.move_to_center()
        
        print(f"\n  RESULT for {name}: Kp={best_gains[0]:.2f}, Ki={best_gains[1]:.2f}, Kd={best_gains[2]:.2f}")
        return best_gains
    
    def calibrate(self, joints: List[int] = None):
        """Calibrate specified joints using brute-force grid search over Kp, Ki, Kd"""
        if joints is None:
            joints = [0, 1, 2, 3, 4]
        
        print("\n" + "="*60)
        print("AUTO PID CALIBRATION (Grid Search)")
        print(f"Joints: {[JOINT_NAMES[j] for j in joints]}")
        print("Based on firmware experience: testing high Ki values!")
        print("="*60)
        
        # Initial setup - set safe defaults and go to center
        print("\nInitial setup...")
        for j in range(1, 6):
            self.set_pid(j, 8.0, 0.0, 0.5)
        self.stop()
        self.move_to_center()
        time.sleep(1)
        
        results = {}
        for joint_idx in joints:
            gains = self.calibrate_joint(joint_idx)
            results[joint_idx] = gains
        
        # Final summary
        print("\n" + "="*60)
        print("CALIBRATION COMPLETE")
        print("="*60)
        for j, (kp, ki, kd) in results.items():
            print(f"Joint {j+1} ({JOINT_NAMES[j]:12}): Kp={kp:5.1f} Ki={ki:4.2f} Kd={kd:4.2f}")
        
        if 1 in joints:
            print("\nNote: Motor 2 and 3 share the same PID (shoulder joint)")
        
        # Save to config
        self.save_config(results)
        
        self.move_to_center()
        self.stop()
        
        return results
    
    def save_config(self, results: dict):
        """Save to arm_config.json"""
        config_path = "arm_config.json"
        try:
            with open(config_path, 'r') as f:
                config = json.load(f)
        except:
            config = {}
        
        if "pid" not in config:
            config["pid"] = {}
        
        for j, (kp, ki, kd) in results.items():
            config["pid"][str(j + 1)] = {"kp": kp, "ki": ki, "kd": kd}
            if j == 1:  # Shoulder - also set motor 3
                config["pid"]["3"] = {"kp": kp, "ki": ki, "kd": kd}
        
        with open(config_path, 'w') as f:
            json.dump(config, f, indent=2)
        print(f"\nSaved to {config_path}")
    
    def close(self):
        self.running = False
        self.stop()
        time.sleep(0.1)
        self.ser.close()


def find_port():
    """Auto-detect serial port"""
    for p in serial.tools.list_ports.comports():
        if any(x in p.description.lower() for x in ["cp210", "ch340", "usb", "serial"]):
            return p.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Auto PID Calibration using Ziegler-Nichols method")
    parser.add_argument("--port", "-p", help="Serial port")
    parser.add_argument("--joint", "-j", type=int, help="Single joint (1-5)")
    parser.add_argument("--joints", type=str, help="Multiple joints, e.g. '1,2,4'")
    args = parser.parse_args()
    
    port = args.port or find_port()
    if not port:
        print("ERROR: No serial port found")
        sys.exit(1)
    
    print(f"Port: {port}")
    
    # Parse joints
    joints = None
    if args.joint:
        joints = [args.joint - 1]
    elif args.joints:
        joints = [int(j.strip()) - 1 for j in args.joints.split(",")]
    
    cal = None
    try:
        cal = PIDCalibrator(port)
        
        print("\nStarting in 3 seconds...")
        print("Make sure arm has room to move!")
        time.sleep(3)
        
        cal.calibrate(joints=joints)
        
    except KeyboardInterrupt:
        print("\n\nAborted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if cal:
            cal.close()


if __name__ == "__main__":
    main()
