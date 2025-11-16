import threading
import queue
import time
import serial
import json
import numpy as np
import kinematics
from STservo_sdk import *

class ArmController:
    def __init__(self, gui_app):
        self.gui = gui_app
        self.portHandler = None
        self.packetHandler = None
        self.servo_limits = {}
        self.motor_ids = []
        self.torque_states = {}
        self.last_ik_solution = None

        self.command_queue = queue.Queue()
        self.command_thread = None
        self.polling_thread = None
        self.polling_started = False
        self.stop_threads = threading.Event()
        self.scanning_in_progress = False
        self.stop_sweep_flag = threading.Event()
        self.sweep_task = None

        self._load_servo_limits()

    def _load_servo_limits(self):
        try:
            with open('servo_limits.json', 'r') as f: self.servo_limits = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            self.gui.update_status(f"Warning: Could not load servo_limits.json. Error: {e}")
            self.servo_limits = {}

    def connect(self, port):
        self.stop_threads.clear()
        self.scanning_in_progress = True
        try:
            self.portHandler = PortHandler(port)
            self.packetHandler = sts(self.portHandler)
            if not self.portHandler.openPort() or not self.portHandler.setBaudRate(1000000):
                raise serial.SerialException("Port setup failed")
        except serial.SerialException as e:
            self.gui.on_scan_complete([]); self.gui.update_status(f"Error: {e}"); return
        threading.Thread(target=self._scan_worker, daemon=True).start()

    def _scan_worker(self):
        found_motors = []
        for motor_id in range(253):
            if not self.scanning_in_progress: break
            if self.packetHandler.ping(motor_id)[1] == COMM_SUCCESS: found_motors.append(motor_id)
            time.sleep(0.001)
        self.scanning_in_progress = False
        self.gui.master.after(0, self.gui.on_scan_complete, found_motors)

    def start_threads(self):
        if not self.polling_started:
            self.command_thread = threading.Thread(target=self._command_worker, daemon=True); self.command_thread.start()
            self.polling_thread = threading.Thread(target=self._polling_worker, daemon=True); self.polling_thread.start()
            self.polling_started = True

    def disconnect(self):
        if self.sweep_task and self.sweep_task.is_alive(): self.toggle_sweep('x', force_stop=True)
        self.stop_threads.set()
        if self.command_thread and self.command_thread.is_alive(): self.command_queue.put(None); self.command_thread.join(timeout=1.0)
        if self.polling_thread and self.polling_thread.is_alive(): self.polling_thread.join(timeout=1.0)
        
        self.scanning_in_progress = False # Ensure scanning stops if closing during scan

        if self.portHandler and self.portHandler.is_open:
            for motor_id in self.motor_ids: self.set_torque(motor_id, 0, force=True)
            self.portHandler.closePort()
        self.polling_started = False

    def _command_worker(self):
        while not self.stop_threads.is_set():
            try:
                move_list = self.command_queue.get(timeout=0.5)
                if move_list is None: break
                for motor_id, position in move_list: self.packetHandler.RegWritePosEx(motor_id, position, 0, 0)
                self.packetHandler.RegAction()
                self.command_queue.task_done()
            except queue.Empty: continue
            except Exception as e: print(f"[ERROR] Command worker: {e}")

    def _polling_worker(self):
        while not self.stop_threads.is_set():
            motor_data = {}
            for motor_id in self.motor_ids:
                try:
                    data, _, _ = self.packetHandler.readTxRx(motor_id, STS_PRESENT_POSITION_L, 10)
                    if data: motor_data[motor_id] = {'pos': self.packetHandler.sts_makeword(data[0], data[1]), 'volt': data[6]/10.0, 'temp': data[7]}
                except Exception: pass
            if motor_data: self.gui.master.after(0, self.gui.update_ui, motor_data)
            time.sleep(0.1)

    def set_joint_position(self, motor_id, slider_pos):
        if self.torque_states.get(motor_id, 0) == 1:
            move_list = []
            if motor_id == 2:
                limits2 = self.servo_limits.get('2', {'min': 0, 'max': 4095})
                pos_to_send = (limits2['min'] + limits2['max']) - slider_pos
                move_list.append((2, pos_to_send))
                if 3 in self.motor_ids:
                    limits3 = self.servo_limits.get('3', {'min': 0, 'max': 4095})
                    range2 = (limits2['max'] - limits2['min']) or 1; range3 = (limits3['max'] - limits3['min']) or 1
                    percentage2 = (pos_to_send - limits2['min']) / range2
                    move_list.append((3, int(limits3['max'] - (percentage2 * range3))))
            else:
                move_list.append((motor_id, slider_pos))
            if move_list: self.command_queue.put(move_list)

    def go_to_ik_position(self, x, y, z, lock_wrist, wrist_angle):
        required_motors = {1, 2, 4, 5}
        if not required_motors.issubset(self.motor_ids): self.gui.update_status(f"Error: IK requires motors {required_motors}."); return
        joint_limits_deg = {mid: (self._pos_to_angle(mid, self.servo_limits[str(mid)]['min']), self._pos_to_angle(mid, self.servo_limits[str(mid)]['max'])) for mid in required_motors if str(mid) in self.servo_limits}
        preferred_phi = wrist_angle if lock_wrist else (np.rad2deg(self.last_ik_solution[3]) if self.last_ik_solution else None)
        solution_rad = kinematics.inverse_kinematics([x, y, z], joint_limits_deg, preferred_phi_deg=preferred_phi, use_locked_angle=lock_wrist)
        if solution_rad is None: self.gui.update_status(f"IK Error: No solution found."); return
        self._execute_ik_solution(solution_rad)
        self.gui.update_status(f"IK solution sent to motors.")

    def _execute_ik_solution(self, solution_rad):
        self.last_ik_solution = solution_rad; q_deg = np.rad2deg(solution_rad)
        base_deg, shoulder_deg, elbow_deg, wrist_deg = q_deg
        move_list = [(1, self._angle_to_pos(1, base_deg)), (4, self._angle_to_pos(4, elbow_deg)), (5, self._angle_to_pos(5, wrist_deg))]
        pos2_shoulder_target = self._angle_to_pos(2, shoulder_deg)
        limits2 = self.servo_limits['2']; pos2_to_send = (limits2['min'] + limits2['max']) - pos2_shoulder_target
        move_list.append((2, pos2_to_send))
        if 3 in self.motor_ids:
            limits3 = self.servo_limits.get('3', {'min': 0, 'max': 4095})
            range2 = (limits2['max'] - limits2['min']) or 1; range3 = (limits3['max'] - limits3['min']) or 1
            percentage2 = (pos2_to_send - limits2['min']) / range2
            move_list.append((3, int(limits3['max'] - (percentage2 * range3))))
        self.command_queue.put(move_list)
        self.gui.update_sliders(move_list, pos2_shoulder_target)

    def toggle_sweep(self, axis, force_stop=False):
        if self.sweep_task and self.sweep_task.is_alive() or force_stop:
            self.stop_sweep_flag.set()
            if force_stop and self.sweep_task: self.sweep_task.join()
        else:
            self.stop_sweep_flag.clear()
            self.gui.set_sweep_ui_state(axis, is_sweeping=True)
            try: fixed_coords = self.gui.get_ik_inputs()
            except ValueError: self.gui.update_status("Error: Invalid IK input for sweep."); self.gui.set_sweep_ui_state(axis, is_sweeping=False); return
            self.sweep_task = threading.Thread(target=self._sweep_worker, args=(axis, fixed_coords), daemon=True)
            self.sweep_task.start()

    def _sweep_worker(self, axis, fixed_coords):
        self.gui.update_status(f"Starting {axis.upper()} sweep...")
        axis_map = {'x': 0, 'y': 1, 'z': 2}; sweep_axis_idx = axis_map[axis]
        max_reach = sum(kinematics.LINK_LENGTHS); sweep_range = [-max_reach * 0.9, max_reach * 0.9]
        direction = 1
        current_pos = kinematics.forward_kinematics(self.last_ik_solution)[-1].tolist() if self.last_ik_solution else [v for k, v in fixed_coords.items()]

        while not self.stop_sweep_flag.is_set():
            current_pos[sweep_axis_idx] += direction * 1.0
            if not (sweep_range[0] <= current_pos[sweep_axis_idx] <= sweep_range[1]):
                direction *= -1; current_pos[sweep_axis_idx] += direction * 2.0
            target_pos = list(current_pos)
            joint_limits_deg = {mid: (self._pos_to_angle(mid, self.servo_limits[str(mid)]['min']), self._pos_to_angle(mid, self.servo_limits[str(mid)]['max'])) for mid in {1,2,4,5} if str(mid) in self.servo_limits}
            lock_wrist, wrist_angle = self.gui.get_wrist_lock_state()
            preferred_phi = wrist_angle if lock_wrist else (np.rad2deg(self.last_ik_solution[3]) if self.last_ik_solution else None)
            solution_rad = kinematics.inverse_kinematics(target_pos, joint_limits_deg, preferred_phi_deg=preferred_phi, use_locked_angle=lock_wrist)
            if solution_rad:
                self._execute_ik_solution(solution_rad)
                self.command_queue.join()
                time.sleep(0.02)
        self.gui.master.after(0, self.gui.set_sweep_ui_state, axis, False)

    def set_torque(self, motor_id, state, force=False):
        if motor_id in self.torque_states or force:
            self.packetHandler.write1ByteTxRx(motor_id, STS_TORQUE_ENABLE, state)
            self.torque_states[motor_id] = state

    def get_kinematic_state(self, all_motor_data):
        motor_map = {1: 0, 2: 1, 4: 2, 5: 3}
        q_rad = [0, np.deg2rad(90), 0, 0]
        if all_motor_data:
            if 2 in all_motor_data and '2' in self.servo_limits:
                pos2 = all_motor_data[2].get('pos', 2048)
                slider_pos = (self.servo_limits['2']['min'] + self.servo_limits['2']['max']) - pos2
                q_rad[1] = np.deg2rad(self._pos_to_angle(2, slider_pos))
            for motor_id, data in all_motor_data.items():
                if motor_id in motor_map and motor_id != 2:
                    q_rad[motor_map[motor_id]] = np.deg2rad(self._pos_to_angle(motor_id, data.get('pos', 0)))
        return q_rad

    def _pos_to_angle(self, motor_id, pos):
        sid = str(motor_id); limits = self.servo_limits.get(sid, {'min':0,'max':4095})
        min_p, max_p = limits['min'], limits['max']
        center_p = (min_p + max_p) / 2
        range_deg = 360.0 if motor_id == 1 else 270.0
        ticks_per_deg = ((max_p - min_p) / range_deg) or 1
        center_angle = 90 if motor_id == 2 else 0
        return center_angle + (pos - center_p) / ticks_per_deg

    def _angle_to_pos(self, motor_id, angle_deg):
        sid = str(motor_id); limits = self.servo_limits.get(sid, {'min':0,'max':4095})
        min_p, max_p = limits['min'], limits['max']
        center_p = (min_p + max_p) / 2
        range_deg = 360.0 if motor_id == 1 else 270.0
        ticks_per_deg = ((max_p - min_p) / range_deg) or 1
        center_angle = 90 if motor_id == 2 else 0
        pos = center_p + (angle_deg - center_angle) * ticks_per_deg
        return int(np.clip(pos, min_p, max_p))

    def stop_scan(self):
        self.scanning_in_progress = False