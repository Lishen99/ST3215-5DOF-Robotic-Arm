#!/usr/bin/env python3
"""
Auto PID Calibration V2 for 5DOF Robotic Arm
Improved version that:
1. Calibrates from joint 5 → 1 (wrist to base, outer joints first)
2. Tests each joint with outer joints in multiple postures/configurations
3. Uses more realistic test conditions with varied arm poses

This approach finds PIDs that work across the arm's operating envelope,
not just at the center position.
"""

import serial
import serial.tools.list_ports
import time
import math
import json
import sys
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
import threading


# Motor centers (from your config)
MOTOR_CENTERS = [2207, 2617, 2771, 2563, 2160, 2047]

# Test motion amplitude in radians (~23 degrees for safety)
TEST_AMPLITUDE = 0.4

# Joint names (index 0-4)
JOINT_NAMES = ["Base", "Shoulder", "Elbow", "Wrist Pitch", "Roll"]

# Joint calibration order: 4, 3, 2, 1 (wrist pitch to base)
# Skip Roll (joint 5/index 4) - no gripper attached yet
# This ensures outer joints are stable before calibrating inner joints
CALIBRATION_ORDER = [3, 2, 1, 0]  # Wrist Pitch, Elbow, Shoulder, Base

# Posture configurations for testing
# Each posture is [q0, q1, q2, q3, q4] in radians
# We test each joint with the other joints in various postures to stress different loads
POSTURES = {
    # Joint 4 (Roll) - Skip for now, no gripper attached
    4: [
        {"name": "center", "joints": [0.0, 0.0, 0.0, 0.0, 0.0]},
    ],
    # Joint 3 (Wrist Pitch) - Test with different elbow/shoulder configurations
    3: [
        {"name": "center", "joints": [0.0, 0.0, 0.0, 0.0, 0.0]},
        {"name": "elbow_bent", "joints": [0.0, 0.0, 0.4, 0.0, 0.0]},  # Elbow bent changes load
        {"name": "arm_raised", "joints": [0.0, -0.3, 0.0, 0.0, 0.0]},  # Shoulder raised
    ],
    # Joint 2 (Elbow) - Test with different shoulder/wrist positions
    2: [
        {"name": "center", "joints": [0.0, 0.0, 0.0, 0.0, 0.0]},
        {"name": "shoulder_up", "joints": [0.0, -0.3, 0.0, 0.0, 0.0]},  # Shoulder raised = more load
        {"name": "wrist_bent", "joints": [0.0, 0.0, 0.0, 0.4, 0.0]},  # Wrist adds end mass
    ],
    # Joint 1 (Shoulder) - Test with different arm configurations
    1: [
        {"name": "center", "joints": [0.0, 0.0, 0.0, 0.0, 0.0]},
        {"name": "elbow_bent", "joints": [0.0, 0.0, 0.5, 0.0, 0.0]},  # Changes moment of inertia
        {"name": "elbow_straight", "joints": [0.0, 0.0, -0.3, 0.0, 0.0]},  # Arm extended
    ],
    # Joint 0 (Base) - Test with different arm extensions
    0: [
        {"name": "center", "joints": [0.0, 0.0, 0.0, 0.0, 0.0]},
        {"name": "arm_extended", "joints": [0.0, -0.3, -0.3, 0.0, 0.0]},  # Arm reaching out
        {"name": "arm_retracted", "joints": [0.0, 0.3, 0.5, 0.0, 0.0]},  # Arm folded up
    ],
}


class PIDCalibratorV2:
    def __init__(self, port: str, baudrate: int = 921600):
        print(f"Connecting to {port}...")
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        time.sleep(0.5)
        
        self.running = True
        self.current_joints = [0.0] * 5
        self.raw_positions = MOTOR_CENTERS.copy()
        self.connected = False
        
        # Current best PIDs (start with reasonable defaults with Ki for precision)
        # Higher Ki values needed because error is in radians (small numbers)
        self.best_pids = {
            0: (25.0, 5.0, 0.8),   # Base
            1: (18.0, 5.0, 0.5),   # Shoulder
            2: (14.0, 4.0, 0.4),   # Elbow
            3: (16.0, 4.0, 0.5),   # Wrist
            4: (14.0, 4.0, 0.4),   # Roll
        }
        
        # Reader thread
        self.reader_thread = threading.Thread(target=self._read_serial, daemon=True)
        self.reader_thread.start()
        
        # Wait for telemetry
        print("Waiting for telemetry...")
        for i in range(30):
            time.sleep(0.1)
            if self.raw_positions != MOTOR_CENTERS:
                self.connected = True
                print(f"Connected! Raw positions: {self.raw_positions}")
                break
            print(".", end="", flush=True)
        
        print()
        if not self.connected:
            print("WARNING: No telemetry received. Make sure GUI is closed!")
    
    def _read_serial(self):
        """Background thread to read serial data"""
        buffer = ""
        while self.running:
            try:
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self._process_message(line)
                else:
                    time.sleep(0.005)
            except:
                time.sleep(0.01)
    
    def _process_message(self, msg: str):
        """Process incoming serial message"""
        if "RAW" in msg:
            try:
                idx = msg.find("RAW")
                raw_part = msg[idx+3:].strip()
                values = raw_part.split()
                if len(values) >= 6:
                    self.raw_positions = [int(v) for v in values[:6]]
                    # Convert to radians
                    self.current_joints[0] = (self.raw_positions[0] - MOTOR_CENTERS[0]) * 2 * math.pi / 4096
                    self.current_joints[1] = -(self.raw_positions[1] - MOTOR_CENTERS[1]) * 2 * math.pi / 4096
                    self.current_joints[2] = (self.raw_positions[3] - MOTOR_CENTERS[3]) * 2 * math.pi / 4096
                    self.current_joints[3] = (self.raw_positions[4] - MOTOR_CENTERS[4]) * 2 * math.pi / 4096
                    self.current_joints[4] = (self.raw_positions[5] - MOTOR_CENTERS[5]) * 2 * math.pi / 4096
            except:
                pass
    
    def send(self, cmd: str):
        """Send command to ESP32"""
        self.ser.write(f"{cmd}\n".encode())
        print(f"    >> {cmd}")
        time.sleep(0.03)
    
    def stop(self):
        """Stop all motion"""
        self.send("S")
        time.sleep(0.1)
    
    def set_pid(self, joint_id: int, kp: float, ki: float, kd: float):
        """Set PID for a joint (1-indexed). Motor 2&3 are linked."""
        self.send(f"P {joint_id} {kp:.2f} {ki:.2f} {kd:.2f}")
        if joint_id == 2:
            self.send(f"P 3 {kp:.2f} {ki:.2f} {kd:.2f}")
    
    def set_all_pids(self):
        """Apply all current best PIDs"""
        for j in range(5):
            kp, ki, kd = self.best_pids[j]
            self.set_pid(j + 1, kp, ki, kd)
    
    def sync_speed_settings(self):
        """Sync speed and wrist settings like GUI does"""
        self.send("SP 2.00")  # Speed multiplier
        self.send("WL 1")     # Wrist locked
        time.sleep(0.1)
    
    def move_to_pose(self, joints: List[float], wait_time: float = 2.5):
        """Move to a specific joint configuration and wait for convergence"""
        cmd = f"D {joints[0]:.4f} {joints[1]:.4f} {joints[2]:.4f} {joints[3]:.4f} {joints[4]:.4f}"
        self.send(cmd)
        
        # Wait for convergence with longer timeout
        start = time.time()
        converged_count = 0
        while time.time() - start < wait_time:
            max_err = max(abs(self.current_joints[i] - joints[i]) for i in range(5))
            if max_err < 0.03:  # ~1.7 degrees
                converged_count += 1
                if converged_count >= 10:  # Stay converged for 100ms
                    time.sleep(0.3)  # Extra settling time
                    break
            else:
                converged_count = 0
            time.sleep(0.01)
    
    def move_to_center(self):
        """Move all joints to center"""
        self.move_to_pose([0.0, 0.0, 0.0, 0.0, 0.0])
    
    def test_step_response(self, joint_idx: int, amplitude: float, 
                           base_pose: List[float], max_duration: float = 4.0) -> Dict:
        """
        Test step response for one joint from a specific base pose.
        Returns dict with settling_time, overshoot, error, oscillations
        """
        # Map joint index to raw_positions index
        raw_idx_map = [0, 1, 3, 4, 5]
        raw_idx = raw_idx_map[joint_idx]
        
        # Get current RAW position and calculate target
        initial_raw = self.raw_positions[raw_idx]
        amplitude_steps = int(amplitude * 4096 / (2 * math.pi))
        if joint_idx == 1:  # Shoulder is negated
            amplitude_steps = -amplitude_steps
        target_raw = initial_raw + amplitude_steps
        
        # Create target pose
        target_pose = base_pose.copy()
        target_pose[joint_idx] = base_pose[joint_idx] + amplitude
        
        # Record trajectory
        raw_trajectory = []
        start_time = time.time()
        
        # Send move command
        cmd = f"D {target_pose[0]:.4f} {target_pose[1]:.4f} {target_pose[2]:.4f} {target_pose[3]:.4f} {target_pose[4]:.4f}"
        self.send(cmd)
        
        # Record for minimum time before checking settling
        min_record_time = 2.5
        settle_threshold = 3  # steps
        settled_count = 0
        required_settled = 50  # 0.5 seconds of stability at 10ms sampling
        
        while time.time() - start_time < max_duration:
            raw_pos = self.raw_positions[raw_idx]
            elapsed = time.time() - start_time
            raw_trajectory.append((elapsed, raw_pos))
            
            if elapsed > min_record_time and len(raw_trajectory) >= 20:
                recent = raw_trajectory[-20:]
                recent_range = max(r for t,r in recent) - min(r for t,r in recent)
                if recent_range <= settle_threshold:
                    settled_count += 1
                    if settled_count >= required_settled:
                        break
                else:
                    settled_count = 0
            
            time.sleep(0.01)
        
        # Analyze
        if len(raw_trajectory) < 20:
            return {"settling": 999, "overshoot": 999, "error": 999, "oscillations": 99}
        
        # Final position
        final_raw = sum(r for t, r in raw_trajectory[-20:]) / 20
        ss_error_steps = abs(target_raw - final_raw)
        ss_error_rad = ss_error_steps * 2 * math.pi / 4096
        
        # Overshoot
        raw_values = [r for t, r in raw_trajectory]
        if amplitude_steps > 0:
            peak = max(raw_values)
            overshoot_steps = max(0, peak - target_raw)
        else:
            peak = min(raw_values)
            overshoot_steps = max(0, target_raw - peak)
        overshoot_pct = (overshoot_steps / abs(amplitude_steps)) * 100 if amplitude_steps else 0
        
        # Settling time
        tolerance_steps = abs(amplitude_steps) * 0.05
        settling = raw_trajectory[-1][0]
        for i in range(len(raw_trajectory) - 1, -1, -1):
            if abs(raw_trajectory[i][1] - target_raw) > tolerance_steps:
                if i < len(raw_trajectory) - 1:
                    settling = raw_trajectory[i + 1][0]
                break
        
        # Oscillation detection
        approach_done_idx = 0
        for i, (t, r) in enumerate(raw_trajectory):
            if abs(r - target_raw) < abs(amplitude_steps) * 0.4:
                approach_done_idx = i
                break
        
        peaks = 0
        analysis_data = raw_trajectory[approach_done_idx:]
        if len(analysis_data) > 10:
            min_peak_height = 4
            for i in range(5, len(analysis_data) - 5):
                p = analysis_data[i][1]
                window_before = [analysis_data[j][1] for j in range(i-5, i)]
                window_after = [analysis_data[j][1] for j in range(i+1, i+6)]
                if p > max(window_before) + min_peak_height and p > max(window_after) + min_peak_height:
                    peaks += 1
                if p < min(window_before) - min_peak_height and p < min(window_after) - min_peak_height:
                    peaks += 1
        
        oscillations = peaks // 2
        
        return {
            "settling": settling,
            "overshoot": overshoot_pct,
            "error": ss_error_rad,
            "oscillations": oscillations
        }
    
    def test_joint_in_posture(self, joint_idx: int, posture: Dict) -> Dict:
        """Test a joint's response in a specific arm posture"""
        pose = posture["joints"].copy()
        
        # Move to posture first and wait for full settling
        print(f"      Moving to base pose: {pose}")
        self.move_to_pose(pose, wait_time=2.5)
        time.sleep(0.5)
        print(f"      Current raw: {self.raw_positions}")
        
        # Test positive step
        print(f"      Testing +{TEST_AMPLITUDE:.2f} rad step on joint {joint_idx}")
        r1 = self.test_step_response(joint_idx, TEST_AMPLITUDE, pose)
        print(f"      Result: settle={r1['settling']:.2f}s os={r1['overshoot']:.0f}% err={math.degrees(r1['error']):.1f}° osc={r1['oscillations']}")
        
        # Return to posture and wait for settling
        self.move_to_pose(pose, wait_time=2.0)
        time.sleep(0.3)
        
        # Test negative step
        print(f"      Testing -{TEST_AMPLITUDE:.2f} rad step on joint {joint_idx}")
        r2 = self.test_step_response(joint_idx, -TEST_AMPLITUDE, pose)
        print(f"      Result: settle={r2['settling']:.2f}s os={r2['overshoot']:.0f}% err={math.degrees(r2['error']):.1f}° osc={r2['oscillations']}")
        
        # Average results
        return {
            "settling": (r1["settling"] + r2["settling"]) / 2,
            "overshoot": (r1["overshoot"] + r2["overshoot"]) / 2,
            "error": (r1["error"] + r2["error"]) / 2,
            "oscillations": max(r1["oscillations"], r2["oscillations"])
        }
    
    def test_joint_all_postures(self, joint_idx: int) -> Dict:
        """Test a joint across all its test postures, return worst-case metrics"""
        postures = POSTURES.get(joint_idx, [{"name": "center", "joints": [0,0,0,0,0]}])
        
        all_results = []
        for posture in postures:
            print(f"      Testing in posture: {posture['name']}")
            result = self.test_joint_in_posture(joint_idx, posture)
            all_results.append(result)
            print(f"        settle={result['settling']:.2f}s os={result['overshoot']:.0f}% "
                  f"err={math.degrees(result['error']):.1f}° osc={result['oscillations']}")
        
        # Return WORST case (max) for robustness
        return {
            "settling": max(r["settling"] for r in all_results),
            "overshoot": max(r["overshoot"] for r in all_results),
            "error": max(r["error"] for r in all_results),
            "oscillations": max(r["oscillations"] for r in all_results)
        }
    
    def score(self, metrics: Dict) -> float:
        """Calculate score (lower = better)"""
        s = metrics["settling"]
        o = metrics["overshoot"]
        e = metrics["error"]
        osc = metrics["oscillations"]
        
        # Oscillation is the WORST - penalize heavily
        osc_penalty = 15.0 * osc + (10.0 if osc >= 2 else 0)
        
        # Overshoot penalty
        if o <= 10:
            os_penalty = o * 0.2
        elif o <= 25:
            os_penalty = 2 + (o - 10) * 0.4
        else:
            os_penalty = 8 + (o - 25) * 0.8
        
        # Error penalty (degrees) - INCREASED weight for precision!
        # 1 degree error should add ~10 to score
        err_penalty = 10.0 * e * 57.3
        
        # Settling penalty
        settle_penalty = 1.0 * s
        
        return settle_penalty + os_penalty + err_penalty + osc_penalty
    
    def calibrate_joint(self, joint_idx: int) -> Tuple[float, float, float]:
        """Calibrate one joint using grid search across multiple postures"""
        joint_id = joint_idx + 1
        name = JOINT_NAMES[joint_idx]
        
        print(f"\n{'='*60}")
        print(f"CALIBRATING JOINT {joint_id}: {name}")
        print(f"{'='*60}")
        
        # Apply current best PIDs for all OTHER joints
        self.set_all_pids()
        
        # Define search space based on joint
        # NOTE: Ki needs to be HIGH because error is in radians (small numbers)
        # and PID output * 100 = motor speed. With 0.05 rad error:
        # Ki=5 → integral after 1s = 0.05, i_out = 5*0.05 = 0.25 → speed = 25 (still low!)
        # So we need Ki in range 5-20 to get meaningful integral action
        if joint_idx == 0:  # Base - needs high Kp, high Ki for precision
            kp_range = [20.0, 25.0, 30.0, 35.0]
            ki_range = [3.0, 5.0, 8.0, 12.0]  # Much higher!
            kd_range = [0.5, 0.8, 1.0, 1.2]
        elif joint_idx == 1:  # Shoulder - fighting gravity, needs strong Ki
            kp_range = [12.0, 16.0, 20.0, 24.0]
            ki_range = [3.0, 5.0, 8.0, 12.0]
            kd_range = [0.3, 0.5, 0.7]
        elif joint_idx == 2:  # Elbow
            kp_range = [8.0, 12.0, 16.0, 20.0]
            ki_range = [2.0, 4.0, 6.0, 10.0]
            kd_range = [0.2, 0.4, 0.6]
        else:  # Wrist, Roll
            kp_range = [10.0, 15.0, 20.0, 25.0]
            ki_range = [2.0, 4.0, 6.0, 10.0]
            kd_range = [0.3, 0.5, 0.7]
        
        # PHASE 1: Coarse search (quick test per combo)
        print(f"\n  PHASE 1: Coarse Search")
        print(f"  Testing {len(kp_range) * len(ki_range) * len(kd_range)} combinations...")
        
        coarse_results = []
        total = len(kp_range) * len(ki_range) * len(kd_range)
        tested = 0
        
        for kp in kp_range:
            for ki in ki_range:
                for kd in kd_range:
                    tested += 1
                    
                    self.set_pid(joint_id, kp, ki, kd)
                    time.sleep(0.05)
                    
                    # Quick test: center posture only
                    self.move_to_center()
                    metrics = self.test_joint_in_posture(joint_idx, 
                                                          {"name": "center", "joints": [0,0,0,0,0]})
                    sc = self.score(metrics)
                    coarse_results.append((kp, ki, kd, sc, metrics))
                    
                    status = "OSC!" if metrics["oscillations"] >= 2 else "ok"
                    print(f"    [{tested:2d}/{total}] P={kp:5.1f} I={ki:4.1f} D={kd:4.2f} "
                          f"-> score={sc:5.1f} osc={metrics['oscillations']} [{status}]")
        
        # Get top 5 candidates
        coarse_results.sort(key=lambda x: x[3])
        top_candidates = coarse_results[:5]
        
        print(f"\n  TOP 5 from coarse search:")
        for i, (kp, ki, kd, sc, m) in enumerate(top_candidates):
            print(f"    #{i+1}: P={kp:.1f} I={ki:.1f} D={kd:.2f} score={sc:.1f}")
        
        # PHASE 2: Full posture test on top candidates
        print(f"\n  PHASE 2: Full Posture Testing on Top Candidates")
        
        full_results = []
        for kp, ki, kd, _, _ in top_candidates:
            print(f"\n    Testing P={kp:.1f} I={ki:.1f} D={kd:.2f} across all postures...")
            
            self.set_pid(joint_id, kp, ki, kd)
            time.sleep(0.05)
            
            metrics = self.test_joint_all_postures(joint_idx)
            sc = self.score(metrics)
            full_results.append((kp, ki, kd, sc, metrics))
            
            print(f"    -> FULL SCORE: {sc:.1f} (settle={metrics['settling']:.2f}s "
                  f"os={metrics['overshoot']:.0f}% err={math.degrees(metrics['error']):.1f}° "
                  f"osc={metrics['oscillations']})")
        
        # PHASE 3: Fine-tune around best
        full_results.sort(key=lambda x: x[3])
        best_kp, best_ki, best_kd, best_score, _ = full_results[0]
        
        print(f"\n  PHASE 3: Fine-tuning around P={best_kp:.1f} I={best_ki:.1f} D={best_kd:.2f}")
        
        # Try small adjustments
        fine_adjustments = [
            (0, 0, 0),
            (-1.0, 0, 0), (1.0, 0, 0),
            (0, -0.3, 0), (0, 0.3, 0),
            (0, 0, -0.1), (0, 0, 0.1),
            (-1.0, 0.3, 0), (1.0, -0.3, 0),
        ]
        
        for dkp, dki, dkd in fine_adjustments:
            kp = max(1.0, best_kp + dkp)
            ki = max(0.0, best_ki + dki)
            kd = max(0.0, best_kd + dkd)
            
            self.set_pid(joint_id, kp, ki, kd)
            time.sleep(0.05)
            
            metrics = self.test_joint_all_postures(joint_idx)
            sc = self.score(metrics)
            
            print(f"    P={kp:.1f} I={ki:.1f} D={kd:.2f} -> score={sc:.1f}")
            
            if sc < best_score - 0.5:
                print(f"      *** IMPROVED from {best_score:.1f} to {sc:.1f}")
                best_kp, best_ki, best_kd = kp, ki, kd
                best_score = sc
        
        # Save best for this joint
        self.best_pids[joint_idx] = (best_kp, best_ki, best_kd)
        
        print(f"\n  RESULT for {name}: P={best_kp:.2f} I={best_ki:.2f} D={best_kd:.2f}")
        return (best_kp, best_ki, best_kd)
    
    def calibrate(self, joints: List[int] = None):
        """Calibrate joints in order from wrist to base"""
        if joints is None:
            joints = CALIBRATION_ORDER
        
        print("\n" + "="*60)
        print("AUTO PID CALIBRATION V2")
        print("Calibration order: Wrist → Base (outer joints first)")
        print("Testing each joint in multiple arm postures for robustness")
        print("="*60)
        
        # Initial setup - sync settings like GUI does
        print("\nInitial setup - syncing speed settings and PIDs...")
        self.sync_speed_settings()
        self.set_all_pids()
        self.stop()
        self.move_to_center()
        time.sleep(1)
        
        results = {}
        for joint_idx in joints:
            gains = self.calibrate_joint(joint_idx)
            results[joint_idx] = gains
            
            # Apply immediately so subsequent joints use updated PIDs
            self.set_pid(joint_idx + 1, *gains)
        
        # Final summary
        print("\n" + "="*60)
        print("CALIBRATION COMPLETE")
        print("="*60)
        print("\nFinal PID values:")
        for j in range(5):
            kp, ki, kd = self.best_pids[j]
            print(f"  Joint {j+1} ({JOINT_NAMES[j]:12}): Kp={kp:5.2f} Ki={ki:5.2f} Kd={kd:5.2f}")
        
        print("\nNote: Motor 2 and 3 share the same PID (shoulder joint)")
        
        # Save to config
        self.save_config()
        
        self.move_to_center()
        self.stop()
        
        return results
    
    def save_config(self):
        """Save to arm_config.json"""
        config_path = "arm_config.json"
        try:
            with open(config_path, 'r') as f:
                config = json.load(f)
        except:
            config = {}
        
        if "pid" not in config:
            config["pid"] = {}
        
        for j in range(5):
            kp, ki, kd = self.best_pids[j]
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
    parser = argparse.ArgumentParser(description="Auto PID Calibration V2 - Multi-posture testing")
    parser.add_argument("--port", "-p", help="Serial port (default: auto-detect)")
    parser.add_argument("--joint", "-j", type=int, help="Single joint to calibrate (1-5)")
    args = parser.parse_args()
    
    port = args.port or find_port()
    if not port:
        print("ERROR: No serial port found")
        sys.exit(1)
    
    print(f"Using port: {port}")
    
    # Parse joints
    joints = None
    if args.joint:
        joints = [args.joint - 1]
    
    cal = None
    try:
        cal = PIDCalibratorV2(port)
        
        print("\nStarting calibration in 3 seconds...")
        print("Make sure the arm has room to move safely!")
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
