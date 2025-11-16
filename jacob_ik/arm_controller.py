import threading
import time
import json
import numpy as np
import kinematics
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
        self._load_servo_limits()

    def _load_servo_limits(self):
        try:
            with open('jacob_ik/servo_limits.json', 'r') as f: self.servo_limits = json.load(f)
        except Exception as e:
            self.gui.update_status(f"Warning: Could not load servo_limits.json. Error: {e}")

    def connect(self, port):
        self.scanning_in_progress = True
        self.stop_all_threads.clear()
        try:
            self.portHandler = PortHandler(port)
            self.packetHandler = sts(self.portHandler)
            if not self.portHandler.openPort() or not self.portHandler.setBaudRate(1000000):
                raise Exception("Port setup failed")
        except Exception as e:
            self.gui.on_scan_complete([]); self.gui.update_status(f"Error: {e}"); return
        threading.Thread(target=self._scan_worker, daemon=True).start()

    def stop_scan(self):
        self.scanning_in_progress = False

    def _scan_worker(self):
        found_motors = []
        for motor_id in range(253):
            if not self.scanning_in_progress: break
            if self.packetHandler.ping(motor_id)[1] == COMM_SUCCESS: found_motors.append(motor_id)
            time.sleep(0.001)
        self.scanning_in_progress = False
        self.gui.master.after(0, self.gui.on_scan_complete, found_motors)

    def initialize_motors(self):
        for motor_id in self.motor_ids:
            self.packetHandler.WheelMode(motor_id)
            self.torque_states[motor_id] = 1
            self.stop_joint_motion_flags[motor_id] = threading.Event()
        self.polling_thread = threading.Thread(target=self._polling_worker, daemon=True); self.polling_thread.start()

    def disconnect(self):
        self.stop_all_motion()
        self.stop_all_threads.set()
        if self.polling_thread and self.polling_thread.is_alive(): self.polling_thread.join(0.1)
        if self.portHandler and self.portHandler.is_open:
            for motor_id in self.motor_ids: self.set_torque(motor_id, 0, force=True)
            self.portHandler.closePort()

    def _polling_worker(self):
        while not self.stop_all_threads.is_set():
            motor_data = {}
            for motor_id in self.motor_ids:
                try:
                    data, _, _ = self.packetHandler.readTxRx(motor_id, STS_PRESENT_POSITION_L, 10)
                    if data:
                        motor_data[motor_id] = {'pos': self.packetHandler.sts_makeword(data[0], data[1]), 'volt': data[6]/10.0, 'temp': data[7]}
                except Exception: pass
            if motor_data:
                self.raw_motor_data = motor_data
                self.update_current_state(motor_data)
                if not self.gui.is_closing: self.gui.master.after(0, self.gui.update_ui, motor_data)
            time.sleep(0.05)

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
        if self.portHandler: self._send_velocities_to_motors(np.zeros(4))

    def move_to_target(self, target_pos):
        if self.is_moving(): self.stop_all_motion()
        self.stop_motion_flag.clear()
        self.motion_thread = threading.Thread(target=self._path_follower_worker, args=(target_pos,), daemon=True)
        self.motion_thread.start()

    def _path_follower_worker(self, target_pos):
        self.gui.master.after(0, self.gui.set_motion_ui_state, True)
        
        K_p = 5.0; K_null = 0.2; dt = 0.02; tolerance = 5.0
        q_rest = np.array([0, np.pi/2, 0, 0])

        while not self.stop_motion_flag.is_set():
            current_pos = kinematics.forward_kinematics(self.current_q_rad)[-1]
            error_to_final = target_pos - current_pos
            if np.linalg.norm(error_to_final) < tolerance: break

            v_desired = K_p * error_to_final
            J = kinematics.calculate_jacobian(self.current_q_rad)
            
            J_pinv_dls = kinematics.get_jacobian_pinv_damped(J)
            q_dot_primary = J_pinv_dls @ v_desired

            J_pinv_std = np.linalg.pinv(J)
            null_space_projector = np.eye(4) - (J_pinv_std @ J)
            
            # Unwrap the angle error for the shortest path
            unwrapped_q_rest = np.unwrap(np.vstack([self.current_q_rad, q_rest]), axis=0)[1]
            q_dot_secondary = K_null * (unwrapped_q_rest - self.current_q_rad)
            
            q_dot = q_dot_primary + (null_space_projector @ q_dot_secondary)

            self._send_velocities_to_motors(self._clamp_q_dot(q_dot))
            time.sleep(dt)
        
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

        if kinematics.inverse_kinematics(pos1, self.current_q_rad) is None: self.gui.master.after(0, self.gui.update_status, f"Warning: Negative sweep for {axis.upper()} may be unreachable.")
        if kinematics.inverse_kinematics(pos2, self.current_q_rad) is None: self.gui.master.after(0, self.gui.update_status, f"Warning: Positive sweep for {axis.upper()} may be unreachable.")

        target_pos = pos2
        K_p = 5.0; K_null = 0.2; dt = 0.02; tolerance = 15.0
        q_rest = np.array([0, np.pi/2, 0, 0])

        while not self.stop_motion_flag.is_set():
            current_pos = kinematics.forward_kinematics(self.current_q_rad)[-1]
            if np.linalg.norm(target_pos - current_pos) < tolerance:
                target_pos = pos1 if np.array_equal(target_pos, pos2) else pos2
                status_msg = f"Sweeping to {axis.upper()}-" if np.array_equal(target_pos, pos1) else f"Sweeping to {axis.upper()}+"
                self.gui.master.after(0, self.gui.update_status, status_msg)

            error = target_pos - current_pos
            v_desired = K_p * error

            J = kinematics.calculate_jacobian(self.current_q_rad)
            J_pinv_dls = kinematics.get_jacobian_pinv_damped(J)
            q_dot_primary = J_pinv_dls @ v_desired

            J_pinv_std = np.linalg.pinv(J)
            null_space_projector = np.eye(4) - (J_pinv_std @ J)
            
            # Unwrap the angle error for the shortest path
            unwrapped_q_rest = np.unwrap(np.vstack([self.current_q_rad, q_rest]), axis=0)[1]
            q_dot_secondary = K_null * (unwrapped_q_rest - self.current_q_rad)
            
            q_dot = q_dot_primary + (null_space_projector @ q_dot_secondary)

            self._send_velocities_to_motors(self._clamp_q_dot(q_dot))
            time.sleep(dt)

        self._send_velocities_to_motors(np.zeros(4))
        self.gui.master.after(0, self.gui.set_motion_ui_state, False)

    def _clamp_q_dot(self, q_dot):
        q_dot_clamped = np.copy(q_dot)
        current_q_deg = np.rad2deg(self.current_q_rad)
        joint_limits_deg = {}
        for mid_str, limits in self.servo_limits.items():
            mid = int(mid_str)
            min_angle, max_angle = sorted((self._pos_to_angle(mid, limits['min']), self._pos_to_angle(mid, limits['max'])))
            joint_limits_deg[mid] = (min_angle, max_angle)
        motor_map = {0: 1, 1: 2, 2: 4, 3: 5}
        for i in range(4):
            motor_id = motor_map.get(i)
            if motor_id not in joint_limits_deg: continue
            min_lim, max_lim = joint_limits_deg[motor_id]
            buffer = 1.0
            if (current_q_deg[i] >= max_lim - buffer) and (q_dot_clamped[i] > 0): q_dot_clamped[i] = 0
            elif (current_q_deg[i] <= min_lim + buffer) and (q_dot_clamped[i] < 0): q_dot_clamped[i] = 0
        return np.clip(q_dot_clamped, -2.0, 2.0)

    def _send_velocities_to_motors(self, q_dot_rad_s):
        RAD_S_TO_SERVO_UNITS = 160; BASE_MOTOR_DEADZONE = 0.01
        motor_map = {0: 1, 1: 2, 2: 4, 3: 5}
        for i, q_dot_val in enumerate(q_dot_rad_s):
            motor_id = motor_map[i]
            speed = int(np.clip(q_dot_val, -2.5, 2.5) * RAD_S_TO_SERVO_UNITS)
            if motor_id == 1 and abs(q_dot_val) < BASE_MOTOR_DEADZONE: speed = 0
            if motor_id == 2:
                speed = -speed
                self.packetHandler.WriteSpec(2, speed, 0)
                if 3 in self.motor_ids: self.packetHandler.WriteSpec(3, -speed, 0)
            else: self.packetHandler.WriteSpec(motor_id, speed, 0)

    def set_joint_position(self, motor_id, target_pos):
        if self.is_moving(): self.stop_all_motion()
        self.stop_motion_flag.clear(); self.stop_joint_motion_flags[motor_id].clear()
        thread = threading.Thread(target=self._joint_follower_worker, args=(motor_id, target_pos), daemon=True)
        self.joint_follower_threads[motor_id] = thread; thread.start()

    def _joint_follower_worker(self, motor_id, target_pos):
        pid = PID(Kp=4.0, Ki=0.2, Kd=1.5, setpoint=target_pos, output_limits=(-1023, 1023))
        while not self.stop_joint_motion_flags.get(motor_id, threading.Event()).is_set():
            motor_info = self.raw_motor_data.get(motor_id)
            if not motor_info or abs(target_pos - motor_info.get('pos')) < 5: break
            velocity_command = int(pid.compute(motor_info.get('pos')))
            if motor_id == 2:
                self.packetHandler.WriteSpec(2, velocity_command, 0)
                if 3 in self.motor_ids: self.packetHandler.WriteSpec(3, -velocity_command, 0)
            else: self.packetHandler.WriteSpec(motor_id, velocity_command, 0)
            time.sleep(0.02)
        self._send_velocities_to_motors(np.zeros(4))

    def set_torque(self, motor_id, state, force=False):
        ids = [motor_id] + ([3] if motor_id == 2 and 3 in self.motor_ids else [])
        for mid in ids:
            if mid in self.torque_states or force:
                try: self.packetHandler.write1ByteTxRx(mid, STS_TORQUE_ENABLE, state); self.torque_states[mid] = state
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
