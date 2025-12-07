import time
import json
import numpy as np
import threading
from arm_controller import ArmController
from STservo_sdk import *

class PIDTuner:
    def __init__(self, port):
        class MockGUI:
            def __init__(self):
                self.master = self
                self.found_motors = None
            def update_status(self, msg): print(f"[STATUS] {msg}")
            def on_scan_complete(self, motors): 
                self.found_motors = motors
            def update_ui(self, data): pass
            def set_motion_ui_state(self, state): pass
            def after(self, delay, func, *args): func(*args)
            @property
            def is_closing(self): return False
            
        self.gui = MockGUI()
        self.controller = ArmController(self.gui)
        self.controller.connect(port)
        
        # Wait for scan to complete
        print("Scanning for motors...")
        timeout = 60.0
        start_wait = time.time()
        while self.gui.found_motors is None and (time.time() - start_wait < timeout):
            time.sleep(1.0)
            if self.controller.scanning_in_progress:
                print(f"  Scanning... ({int(time.time() - start_wait)}s)")
            
        if not self.gui.found_motors:
            print("No motors found!")
            exit(1)
            
        self.controller.motor_ids = sorted(self.gui.found_motors)
        self.controller.initialize_motors()
        print(f"Connected. Found motors: {self.controller.motor_ids}")

    def run_step_response(self, motor_id, start_pos, step_size, duration=1.0):
        """
        Performs a step response test.
        Moves motor to start_pos, waits, then steps to start_pos + step_size.
        Returns time and position arrays.
        """
        # Safety Check
        sid = str(motor_id)
        if sid in self.controller.servo_limits:
            limits = self.controller.servo_limits[sid]
            target = start_pos + step_size
            if target < limits['min'] or target > limits['max']:
                print(f"SAFETY ABORT: Target {target} exceeds limits [{limits['min']}, {limits['max']}]")
                return np.array([]), np.array([])
        
        # Move to start
        print(f"Moving Motor {motor_id} to start position {start_pos}...")
        self.controller.set_joint_position(motor_id, start_pos)
        time.sleep(2.0) # Wait to settle
        
        # Start recording
        times = []
        positions = []
        target = start_pos + step_size
        
        print(f"Executing step to {target}...")
        start_time = time.time()
        
        # Send step command (using the controller's low-level write for direct control)
        # We bypass the smooth profile to test the raw PID response
        # But wait, the controller runs a loop. We need to inject the command.
        # Actually, we should update the controller's target.
        
        # For tuning, we want to see how the *internal* PID (if using firmware) or *software* PID responds.
        # Since we are tuning the SOFTWARE PID (JointController), we should use that.
        # But `set_joint_position` uses a smooth profile (square root braking).
        # To tune PID, we usually want to see the response to a raw error step.
        
        # Let's temporarily disable the profiler in JointController for this test
        # or just set a very high accel limit.
        
        if motor_id not in self.controller.motor_ids:
             print(f"Motor {motor_id} not found in scanned list.")
             return np.array([]), np.array([])
             
        jc = None
        for controller in self.controller.joint_controllers.values():
            if controller.motor_id == motor_id:
                jc = controller
                break
        
        if jc is None:
             print(f"Motor {motor_id} has no JointController.")
             return np.array([]), np.array([])

        original_accel = jc.max_accel
        jc.max_accel = 99999.0 # Disable ramping
        
        self.controller.set_joint_position(motor_id, target)
        
        while time.time() - start_time < duration:
            t = time.time() - start_time
            # Get latest pos from controller's polling thread
            if motor_id in self.controller.raw_motor_data:
                pos = self.controller.raw_motor_data[motor_id]['pos']
                times.append(t)
                positions.append(pos)
            time.sleep(0.01)
            
        # Restore accel
        jc.max_accel = original_accel
        
        return np.array(times), np.array(positions)

    def analyze_response(self, times, positions, target):
        """
        Calculates Rise Time, Overshoot, and Steady State Error.
        """
        if len(positions) == 0: return 0, 0, 0
        
        final_pos = np.mean(positions[-10:]) # Average of last 10 points
        ss_error = abs(target - final_pos)
        
        peak_pos = np.max(positions) if target > positions[0] else np.min(positions)
        overshoot = abs(peak_pos - target) / abs(target - positions[0]) * 100.0
        
        # Rise time (10% to 90%)
        start_val = positions[0]
        target_delta = target - start_val
        t_10 = None
        t_90 = None
        
        for i, p in enumerate(positions):
            progress = (p - start_val) / target_delta
            if t_10 is None and progress >= 0.1: t_10 = times[i]
            if t_90 is None and progress >= 0.9: t_90 = times[i]
            
        rise_time = (t_90 - t_10) if (t_10 and t_90) else 0.0
        
        return rise_time, overshoot, ss_error

    def tune_joint(self, motor_id):
        print(f"\n=== Tuning Motor {motor_id} ===")
        
        # Load current limits to get safe range
        sid = str(motor_id)
        if sid not in self.controller.servo_limits:
            print("No limits found for motor.")
            return
            
        limits = self.controller.servo_limits[sid]
        center = (limits['min'] + limits['max']) // 2
        step = 200 # Steps (approx 17 degrees)
        
        # Find JointController
        jc = None
        for controller in self.controller.joint_controllers.values():
            if controller.motor_id == motor_id:
                jc = controller
                break
        
        if jc is None:
            print(f"Skipping Motor {motor_id}: No JointController found.")
            return

        # --- Phase 1: Coarse Kp Search ---
        print("--- Phase 1: Coarse Kp Search ---")
        best_kp = 0
        best_score = float('inf')
        test_kps = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0, 12.0, 15.0]
        
        for kp in test_kps:
            print(f"Testing Kp={kp}...")
            jc.kp = kp; jc.ki = 0.0; jc.kd = 0.0
            
            times, positions = self.run_step_response(motor_id, center - step//2, step)
            rise, overshoot, ss_err = self.analyze_response(times, positions, center + step//2)
            
            print(f"  Rise: {rise:.3f}s, Overshoot: {overshoot:.1f}%, SS Err: {ss_err:.1f}")
            
            if overshoot > 20.0:
                print("  -> Too much overshoot.")
                break
            
            score = ss_err + (overshoot * 2.0)
            if score < best_score:
                best_score = score
                best_kp = kp

        print(f"Selected Coarse Kp: {best_kp}")
        
        # --- Phase 2: Fine Tuning Loop ---
        print("--- Phase 2: Fine Tuning (Target SS Err < 10) ---")
        
        # Initial guesses based on Coarse Kp
        current_kp = best_kp * 0.8
        current_kd = current_kp * 0.1
        current_ki = 0.05
        
        for iteration in range(10): # Max 10 iterations
            jc.kp = current_kp; jc.ki = current_ki; jc.kd = current_kd
            print(f"Iter {iteration+1}: Kp={current_kp:.2f}, Ki={current_ki:.3f}, Kd={current_kd:.3f}")
            
            times, positions = self.run_step_response(motor_id, center - step//2, step)
            rise, overshoot, ss_err = self.analyze_response(times, positions, center + step//2)
            print(f"  Result -> SS Err: {ss_err:.1f}, Overshoot: {overshoot:.1f}%")
            
            if ss_err < 10.0 and overshoot < 15.0:
                print("  -> Target Achieved!")
                break
                
            # Adjust Gains
            if ss_err > 10.0:
                current_ki += 0.05 # Boost Integral
                print("  -> Increasing Ki to reduce error")
                
            if overshoot > 10.0:
                current_kd += 0.1 # Boost Derivative
                print("  -> Increasing Kd to dampen overshoot")
            elif overshoot < 2.0 and ss_err > 10.0:
                current_kp += 0.5 # Boost Proportional if sluggish
                print("  -> Increasing Kp for speed")
                
        # Save final
        final_kp, final_ki, final_kd = current_kp, current_ki, current_kd
        print(f"Final Gains -> Kp: {final_kp:.2f}, Ki: {final_ki:.2f}, Kd: {final_kd:.2f}")
        
        self.controller.servo_limits[sid]['kp'] = final_kp
        self.controller.servo_limits[sid]['ki'] = final_ki
        self.controller.servo_limits[sid]['kd'] = final_kd
        
    def save_results(self):
        with open('jacob_ik/servo_limits.json', 'w') as f:
            json.dump(self.controller.servo_limits, f, indent=4)
        print("Saved new gains to servo_limits.json")

    def close(self):
        self.controller.disconnect()

if __name__ == "__main__":
    tuner = PIDTuner("COM3") # Adjust port as needed
    try:
        # Tune each joint
        # Motor map: 0->1, 1->2, 2->4, 3->5
        # Tune Base (1)
        tuner.tune_joint(1)
        # Tune Shoulder (2) - Note: 3 is coupled, so tuning 2 drives both
        tuner.tune_joint(2)
        # Tune Elbow (4)
        tuner.tune_joint(4)
        # Tune Wrist (5)
        tuner.tune_joint(5)
        
        tuner.save_results()
        
    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        tuner.close()
