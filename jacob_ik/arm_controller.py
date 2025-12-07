import threading
import time
import json
import numpy as np
import kinematics
import trajectory
import joint_control
from STservo_sdk import *

class PID:
    """A simple PID controller."""
    def __init__(self, Kp, Ki, Kd, setpoint, output_limits):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.min_output, self.max_output = output_limits
        self.last_error, self.integral = 0, 0
        self.last_time = time.time()

    def compute(self, current_value):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0: return 0
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.last_error = error
        self.last_time = current_time
        return np.clip(output, self.min_output, self.max_output)

class ArmController:
    def __init__(self, gui_app):
        self.gui = gui_app
        self.portHandler = None
        self.packetHandler = None
        self.groupSyncWrite = None
        self.groupSyncRead = None
        self.servo_limits = {}
        self.motor_ids = []
        self.torque_states = {}
        self.current_q_rad = np.array([0.0, np.pi/2, 0.0, 0.0])
        self.raw_motor_data = {}
        self.polling_thread = None
        self.motion_thread = None
        self.joint_follower_threads = {}
        self.stop_all_threads = threading.Event()
        self.stop_motion_flag = threading.Event()
        self.stop_joint_motion_flags = {}
        self.scanning_in_progress = False
        
        self.joint_controllers = {}
        self.trajectory_gen = trajectory.TrajectoryGenerator()
        self.lock = threading.Lock() # Lock for serial port access
        
        self._load_servo_limits()

    def _load_servo_limits(self):
        try:
            with open('jacob_ik/servo_limits.json', 'r') as f: self.servo_limits = json.load(f)
        except Exception as e:
            self.gui.update_status(f"Warning: Could not load servo_limits.json. Error: {e}")

    def _init_joint_controllers(self):
        self.joint_controllers = {}
        # Motor map: 0->1 (Base), 1->2 (Shoulder), 2->4 (Elbow), 3->5 (Wrist)
        motor_map = {0: 1, 1: 2, 2: 4, 3: 5}
        
        for i in range(4):
            mid = motor_map[i]
            sid = str(mid)
            if sid in self.servo_limits:
                params = self.servo_limits[sid]
                self.joint_controllers[i] = joint_control.JointController(
                    motor_id=mid,
                    kp=params.get('kp', 4.0),
                    ki=params.get('ki', 0.1),
                    kd=params.get('kd', 0.5),
                    max_vel=params.get('max_vel', 2.0),
                    max_accel=params.get('max_accel', 5.0)
                )
            else:
                # Defaults
                self.joint_controllers[i] = joint_control.JointController(motor_id=mid)

    def connect(self, port):
        self.scanning_in_progress = True
        self.stop_all_threads.clear()
        try:
            self.portHandler = PortHandler(port)
            self.packetHandler = sts(self.portHandler)
            if not self.portHandler.openPort() or not self.portHandler.setBaudRate(1000000):
                raise Exception("Port setup failed")
            
            # Initialize Sync Read/Write
            # STS_GOAL_SPEED_L = 46, 2 bytes
            self.groupSyncWrite = GroupSyncWrite(self.packetHandler, STS_GOAL_SPEED_L, 2)
            # STS_PRESENT_POSITION_L = 56, 2 bytes
            self.groupSyncRead = GroupSyncRead(self.packetHandler, STS_PRESENT_POSITION_L, 2)
            
        except Exception as e:
            self.gui.on_scan_complete([]); self.gui.update_status(f"Error: {e}"); return
        threading.Thread(target=self._scan_worker, daemon=True).start()

    def stop_scan(self):
        self.scanning_in_progress = False

    def _scan_worker(self):
        print("Scan worker started")
        found_motors = []
        for motor_id in range(253):
            if not self.scanning_in_progress: break
            with self.lock:
                res = self.packetHandler.ping(motor_id)
                # print(f"Ping {motor_id}: {res}")
                if res[1] == COMM_SUCCESS: 
                    print(f"Found motor {motor_id}")
                    found_motors.append(motor_id)
            time.sleep(0.001)
        self.scanning_in_progress = False
        print(f"Scan complete. Found: {found_motors}")
        self.gui.master.after(0, self.gui.on_scan_complete, found_motors)

    def initialize_motors(self):
        with self.lock:
            for motor_id in self.motor_ids:
                self.packetHandler.WheelMode(motor_id)
                self.torque_states[motor_id] = 1
                self.stop_joint_motion_flags[motor_id] = threading.Event()
        
        self._init_joint_controllers()
        self.polling_thread = threading.Thread(target=self._polling_worker, daemon=True); self.polling_thread.start()

    def disconnect(self):
        self.stop_all_motion()
        self.stop_all_threads.set()
        
        # Join threads to ensure they stop using the port
        if self.polling_thread and self.polling_thread.is_alive(): 
            self.polling_thread.join(1.0)
        if self.motion_thread and self.motion_thread.is_alive():
            self.motion_thread.join(1.0)
            
        with self.lock:
            if self.portHandler and self.portHandler.is_open:
                for motor_id in self.motor_ids: 
                    # Force torque off
                    try: self.packetHandler.write1ByteTxRx(motor_id, STS_TORQUE_ENABLE, 0)
                    except: pass
                try:
                    self.portHandler.closePort()
                except Exception as e:
                    print(f"Error closing port: {e}")

    def _send_velocities_to_motors(self, q_dot_rad_s):
        RAD_S_TO_SERVO_UNITS = 650; BASE_MOTOR_DEADZONE = 0.01
        motor_map = {0: 1, 1: 2, 2: 4, 3: 5}
        
        self.groupSyncWrite.clearParam()
        
        for i, q_dot_val in enumerate(q_dot_rad_s):
            motor_id = motor_map[i]
            speed = int(np.clip(q_dot_val, -6.0, 6.0) * RAD_S_TO_SERVO_UNITS)
            
            # Deadzone check
            if motor_id == 1 and abs(q_dot_val) < BASE_MOTOR_DEADZONE: 
                speed = 0
                
            if motor_id == 2:
                speed = -speed # Inverted for shoulder
                self._add_sync_write_param(2, speed)
                if 3 in self.motor_ids: self._add_sync_write_param(3, -speed)
            else: 
                self._add_sync_write_param(motor_id, speed)
                
        try:
            self.groupSyncWrite.txPacket()
            # print("SyncWrite Sent") 
        except Exception as e:
            if not self.stop_all_threads.is_set():
                print(f"SyncWrite Error: {e}")

    def _add_sync_write_param(self, motor_id, speed):
        if speed < 0:
            speed_val = abs(speed)
            speed_val |= (1 << 15)
        else:
            speed_val = speed
        
        # Split into 2 bytes
        lo_byte = speed_val & 0xFF
        hi_byte = (speed_val >> 8) & 0xFF
        self.groupSyncWrite.addParam(motor_id, [lo_byte, hi_byte])

    def _polling_worker(self):
        print("Polling worker started")
        while not self.stop_all_threads.is_set():
            motor_data = {}
            
            # Ensure all motors are in the SyncRead param list
            with self.lock:
                for motor_id in self.motor_ids:
                    self.groupSyncRead.addParam(motor_id)
                
                # Perform Sync Read
                try:
                    result = self.groupSyncRead.txRxPacket()
                    if result != COMM_SUCCESS:
                        print(f"SyncRead Failed: {result}")
                    
                    if result == COMM_SUCCESS:
                        for motor_id in self.motor_ids:
                            if self.groupSyncRead.isAvailable(motor_id, STS_PRESENT_POSITION_L, 2):
                                pos = self.groupSyncRead.getData(motor_id, STS_PRESENT_POSITION_L, 2)
                                motor_data[motor_id] = {'pos': pos, 'volt': 0, 'temp': 0} 
                            else:
                                print(f"ID {motor_id} data not available")
                except Exception as e: 
                    print(f"SyncRead Exception: {e}")
                    import traceback; traceback.print_exc()
            
            if motor_data:
                self.raw_motor_data = motor_data
                self.update_current_state(motor_data)
                if not self.gui.is_closing: self.gui.master.after(0, self.gui.update_ui, motor_data)
            
            # SyncRead is fast, we can poll faster or sleep less
            time.sleep(0.01)

    def update_current_state(self, motor_data):
        q_new = list(self.current_q_rad)
        motor_map = {1: 0, 2: 1, 4: 2, 5: 3}
        for mid, data in motor_data.items():
            if mid in motor_map: q_new[motor_map[mid]] = np.deg2rad(self._pos_to_angle(mid, data.get('pos', 0)))
        self.current_q_rad = np.array(q_new)

    def is_moving(self):
        return self.motion_thread and self.motion_thread.is_alive()

    def stop_all_motion(self):
        self.stop_motion_flag.set()
        for flag in self.stop_joint_motion_flags.values(): flag.set()
        with self.lock:
            if self.portHandler: self._send_velocities_to_motors(np.zeros(4))

    def move_to_target(self, target_pos):
        if self.is_moving(): self.stop_all_motion()
        self.stop_motion_flag.clear()
        self.motion_thread = threading.Thread(target=self._path_follower_worker, args=(target_pos,), daemon=True)
        self.motion_thread.start()

    def _clamp_q_dot(self, q_dot):
        q_dot_clamped = np.copy(q_dot)
        current_q_deg = np.rad2deg(self.current_q_rad)
        
        # Strict hard limits
        # We must read the limits from self.servo_limits
        # Motor map: 0->1, 1->2, 2->4, 3->5
        motor_map = {0: 1, 1: 2, 2: 4, 3: 5}
        
        for i in range(4):
            mid = motor_map[i]
            sid = str(mid)
            if sid not in self.servo_limits: continue
            
            limits = self.servo_limits[sid]
            # Get min/max in degrees
            min_angle = self._pos_to_angle(mid, limits['min'])
            max_angle = self._pos_to_angle(mid, limits['max'])
            
            # Ensure min < max
            if min_angle > max_angle: min_angle, max_angle = max_angle, min_angle
            
            # Buffer in degrees
            buffer = 2.0 
            
            # Check limits
            # If we are near max and velocity is positive (trying to go higher), clamp to 0
            if (current_q_deg[i] >= max_angle - buffer) and (q_dot_clamped[i] > 0):
                q_dot_clamped[i] = 0
            # If we are near min and velocity is negative (trying to go lower), clamp to 0
            elif (current_q_deg[i] <= min_angle + buffer) and (q_dot_clamped[i] < 0):
                q_dot_clamped[i] = 0
                
        return np.clip(q_dot_clamped, -2.0, 2.0)

    def _path_follower_worker(self, target_pos):
        self.gui.master.after(0, self.gui.set_motion_ui_state, True)
        
        # Reset joint controllers
        for jc in self.joint_controllers.values(): jc.reset()
        
        # Plan trajectory
        start_pos = kinematics.forward_kinematics(self.current_q_rad)[-1]
        self.trajectory_gen.plan_linear_path(start_pos, target_pos, max_vel=100.0, max_accel=150.0)
        
        # DISABLE Null Space to prevent oscillations/wrapping issues
        K_null = 0.0 
        q_rest = np.array([0, np.pi/2, 0, 0])
        dt = 0.02
        
        while not self.stop_motion_flag.is_set():
            current_time = time.time()
            
            # 1. Get desired state from trajectory
            traj_pos, traj_vel, finished = self.trajectory_gen.get_target(current_time)
            
            if finished:
                current_pos = kinematics.forward_kinematics(self.current_q_rad)[-1]
                if np.linalg.norm(target_pos - current_pos) < 5.0:
                    break
            
            # 2. Calculate Jacobian and Adaptive Damping
            J = kinematics.calculate_jacobian(self.current_q_rad)
            w = kinematics.calculate_manipulability(J)
            
            # Emergency Stop for Singularity
            if w < 0.005:
                self.gui.master.after(0, self.gui.update_status, "Stopping: Near Singularity")
                break

            lambda_val = kinematics.calculate_adaptive_damping(J, lambda_max=0.05, epsilon=0.1)
            J_pinv_dls = kinematics.get_jacobian_pinv_damped(J, damping_factor=lambda_val)
            
            # 3. Closed-loop correction
            current_pos = kinematics.forward_kinematics(self.current_q_rad)[-1]
            error = traj_pos - current_pos
            
            # Cartesian Deadband to prevent jitter
            if np.linalg.norm(error) < 1.0:
                v_cmd = traj_vel
            else:
                v_cmd = traj_vel + 5.0 * error
            
            # 4. Inverse Kinematics
            q_dot_primary = J_pinv_dls @ v_cmd
            
            # 5. Null-space control (DISABLED)
            q_dot = q_dot_primary 
            
            # 6. Apply Joint Limits and Acceleration Limiting
            q_dot_limited = []
            for i in range(4):
                if i in self.joint_controllers:
                    q_dot_limited.append(self.joint_controllers[i].update(q_dot[i], dt))
                else:
                    q_dot_limited.append(q_dot[i])
            
            # Clamp for hard limits
            q_dot_final = self._clamp_q_dot(np.array(q_dot_limited))
            
            with self.lock:
                self._send_velocities_to_motors(q_dot_final)
            time.sleep(dt)
        
        with self.lock:
            self._send_velocities_to_motors(np.zeros(4))
        if not self.stop_motion_flag.is_set():
            self.gui.master.after(0, self.gui.update_status, "Motion complete.")
        self.gui.master.after(0, self.gui.set_motion_ui_state, False)

    def start_sweep(self, axis):
        if self.is_moving(): self.stop_all_motion()
        self.stop_motion_flag.clear()
        self.motion_thread = threading.Thread(target=self._sweep_worker, args=(axis,), daemon=True)
        self.motion_thread.start()

    def _sweep_worker(self, axis):
        self.gui.master.after(0, self.gui.set_motion_ui_state, True)
        axis_map = {'x': 0, 'y': 1, 'z': 2}
        idx = axis_map[axis]
        start_pos = kinematics.forward_kinematics(self.current_q_rad)[-1]
        
        sweep_dist = 175
        pos1 = np.copy(start_pos); pos1[idx] = -sweep_dist
        pos2 = np.copy(start_pos); pos2[idx] = sweep_dist

        # Check reachability
        if kinematics.inverse_kinematics(pos1, self.current_q_rad) is None: self.gui.master.after(0, self.gui.update_status, f"Warning: Negative sweep for {axis.upper()} may be unreachable.")
        if kinematics.inverse_kinematics(pos2, self.current_q_rad) is None: self.gui.master.after(0, self.gui.update_status, f"Warning: Positive sweep for {axis.upper()} may be unreachable.")

        # Reset joint controllers
        for jc in self.joint_controllers.values(): jc.reset()

        targets = [pos2, pos1]
        current_target_idx = 0
        
        # Initial plan - Reduce speed for better tracking
        self.trajectory_gen.plan_linear_path(start_pos, targets[0], max_vel=100.0, max_accel=50.0)
        
        # Re-enable Null Space to maintain "Elbow Up" posture
        K_null = 0.5
        # Preferred pose: Base=0, Shoulder=45deg, Elbow=-90deg (Up), Wrist=-45deg
        q_rest = np.array([0, np.pi/4, -np.pi/2, -np.pi/4])
        dt = 0.02

        # Open log file
        import csv
        log_file = open('sweep_log.csv', 'w', newline='')
        writer = csv.writer(log_file)
        writer.writerow(['timestamp', 'target_x', 'target_y', 'target_z', 'actual_x', 'actual_y', 'actual_z', 'error'])
        start_time_log = time.time()
        
        next_wake_time = time.time() + dt

        while not self.stop_motion_flag.is_set():
            current_time = time.time()
            traj_pos, traj_vel, finished = self.trajectory_gen.get_target(current_time)
            
            # Log data
            current_pos_log = kinematics.forward_kinematics(self.current_q_rad)[-1]
            error_log = np.linalg.norm(traj_pos - current_pos_log)
            writer.writerow([current_time - start_time_log, traj_pos[0], traj_pos[1], traj_pos[2], current_pos_log[0], current_pos_log[1], current_pos_log[2], error_log])
            
            if finished:
                # Wait for arrival
                dist_to_target = np.linalg.norm(traj_pos - current_pos_log)
                if dist_to_target > 10.0:
                    # Continue servoing to the final point
                    pass
                else:
                    # Switch target
                    current_target_idx = (current_target_idx + 1) % 2
                    current_pos = kinematics.forward_kinematics(self.current_q_rad)[-1]
                    self.trajectory_gen.plan_linear_path(current_pos, targets[current_target_idx], max_vel=100.0, max_accel=50.0)
                    status_msg = f"Sweeping to {axis.upper()}{'+' if current_target_idx == 0 else '-'}"
                    self.gui.master.after(0, self.gui.update_status, status_msg)
                    # Reset timer for new segment to avoid catch-up
                    next_wake_time = time.time() + dt
                    continue

            J = kinematics.calculate_jacobian(self.current_q_rad)
            w = kinematics.calculate_manipulability(J)
            
            if w < 0.005:
                self.gui.master.after(0, self.gui.update_status, "Stopping: Near Singularity")
                break

            lambda_val = kinematics.calculate_adaptive_damping(J, lambda_max=0.01, epsilon=0.05)
            J_pinv_dls = kinematics.get_jacobian_pinv_damped(J, damping_factor=lambda_val)
            
            current_pos = kinematics.forward_kinematics(self.current_q_rad)[-1]
            error = traj_pos - current_pos
            
            # Cartesian Deadband
            if np.linalg.norm(error) < 1.0:
                v_cmd = traj_vel
            else:
                v_cmd = traj_vel + 5.0 * error
            
            q_dot_primary = J_pinv_dls @ v_cmd
            
            # Null space control for Posture Optimization
            J_pinv_std = np.linalg.pinv(J)
            null_space_projector = np.eye(4) - (J_pinv_std @ J)
            
            # Calculate attraction to rest pose
            q_diff = q_rest - self.current_q_rad
            # Normalize angles
            for i in range(4):
                q_diff[i] = (q_diff[i] + np.pi) % (2 * np.pi) - np.pi
            
            q_dot_secondary = K_null * q_diff
            
            q_dot = q_dot_primary + (null_space_projector @ q_dot_secondary)
            
            q_dot_limited = []
            for i in range(4):
                if i in self.joint_controllers:
                    q_dot_limited.append(self.joint_controllers[i].update(q_dot[i], dt))
                else:
                    q_dot_limited.append(q_dot[i])
            
            q_dot_final = self._clamp_q_dot(np.array(q_dot_limited))
            
            with self.lock:
                self._send_velocities_to_motors(q_dot_final)
            
            # Drift-correcting sleep
            now = time.time()
            sleep_time = next_wake_time - now
            if sleep_time > 0:
                time.sleep(sleep_time)
            next_wake_time += dt

        log_file.close()
        with self.lock:
            self._send_velocities_to_motors(np.zeros(4))
        self.gui.master.after(0, self.gui.set_motion_ui_state, False)

    def _send_velocities_to_motors(self, q_dot_rad_s):
        RAD_S_TO_SERVO_UNITS = 650; BASE_MOTOR_DEADZONE = 0.01
        motor_map = {0: 1, 1: 2, 2: 4, 3: 5}
        for i, q_dot_val in enumerate(q_dot_rad_s):
            motor_id = motor_map[i]
            speed = int(np.clip(q_dot_val, -6.0, 6.0) * RAD_S_TO_SERVO_UNITS)
            
            # Deadzone check
            if motor_id == 1 and abs(q_dot_val) < BASE_MOTOR_DEADZONE: 
                speed = 0
                
            if motor_id == 2:
                speed = -speed # Inverted for shoulder
                self.packetHandler.WriteSpec(2, speed, 0)
                if 3 in self.motor_ids: self.packetHandler.WriteSpec(3, -speed, 0) # Coupled servo 3
            else: self.packetHandler.WriteSpec(motor_id, speed, 0)

    def set_joint_position(self, motor_id, target_pos):
        if self.is_moving(): self.stop_all_motion()
        self.stop_motion_flag.clear(); self.stop_joint_motion_flags[motor_id].clear()
        thread = threading.Thread(target=self._joint_follower_worker, args=(motor_id, target_pos), daemon=True)
        self.joint_follower_threads[motor_id] = thread; thread.start()

    def _joint_follower_worker(self, motor_id, target_pos):
        sid = str(motor_id)
        params = self.servo_limits.get(sid, {})
        # Use servo limits for profile
        max_vel_rad = params.get('max_vel', 6.0)
        max_accel_rad = params.get('max_accel', 5.0)
        
        # Convert to servo units (approx)
        RAD_S_TO_SERVO_UNITS = 650
        # Steps per radian: 4096 / (2*pi) approx 651
        STEPS_PER_RAD = 651.0
        
        max_vel_steps = max_vel_rad * RAD_S_TO_SERVO_UNITS
        max_accel_steps = max_accel_rad * RAD_S_TO_SERVO_UNITS
        
        while not self.stop_joint_motion_flags.get(motor_id, threading.Event()).is_set():
            motor_info = self.raw_motor_data.get(motor_id)
            if not motor_info: 
                time.sleep(0.01); continue
                
            current_pos = motor_info.get('pos')
            error = target_pos - current_pos
            
            if abs(error) < 10: # Deadband
                break
            
            # Square root braking profile: v = sqrt(2 * a * d)
            # v_req = sign(error) * min(max_vel, sqrt(2 * max_accel * abs(error)))
            
            dist = abs(error)
            # Calculate max velocity allowed by braking distance
            # v^2 = u^2 + 2as -> v = sqrt(2as) assuming u=0 at target
            braking_vel = np.sqrt(2 * max_accel_steps * dist)
            
            req_vel = min(max_vel_steps, braking_vel)
            
            if error < 0: req_vel = -req_vel
            
            velocity_command = int(req_vel)
            
            with self.lock:
                if motor_id == 2:
                    self.packetHandler.WriteSpec(2, velocity_command, 0)
                    if 3 in self.motor_ids: self.packetHandler.WriteSpec(3, -velocity_command, 0)
                else: self.packetHandler.WriteSpec(motor_id, velocity_command, 0)
            time.sleep(0.02)
        
        with self.lock:
            # Stop
            if motor_id == 2:
                self.packetHandler.WriteSpec(2, 0, 0)
                if 3 in self.motor_ids: self.packetHandler.WriteSpec(3, 0, 0)
            else: self.packetHandler.WriteSpec(motor_id, 0, 0)

    def set_torque(self, motor_id, state, force=False):
        ids = [motor_id] + ([3] if motor_id == 2 and 3 in self.motor_ids else [])
        for mid in ids:
            if mid in self.torque_states or force:
                try: 
                    with self.lock:
                        self.packetHandler.write1ByteTxRx(mid, STS_TORQUE_ENABLE, state)
                    self.torque_states[mid] = state
                except: pass

    def _pos_to_angle(self, motor_id, pos):
        sid = str(motor_id)
        if sid not in self.servo_limits: return 0
        limits = self.servo_limits[sid]; min_p, max_p = limits['min'], limits['max']; center_p = (min_p + max_p) / 2
        center_angle = 90.0 if motor_id == 2 else 0.0
        range_deg = 360.0 if motor_id == 1 else 270.0
        ticks_per_deg = ((max_p - min_p) / range_deg) or 1
        if motor_id == 2:
            logical_pos = (min_p + max_p) - pos
            return center_angle + (logical_pos - center_p) / ticks_per_deg
        else: return center_angle + (pos - center_p) / ticks_per_deg
