import numpy as np
import time
from jacob_ik import kinematics, trajectory, joint_control

# Mock the ArmController's path follower logic
class MockController:
    def __init__(self):
        self.current_q_rad = np.array([0.0, np.pi/2, 0.0, 0.0]) # Start vertical
        self.path = []
        self.trajectory_gen = trajectory.TrajectoryGenerator()
        self.joint_controllers = {}
        for i in range(4):
            self.joint_controllers[i] = joint_control.JointController(i, max_accel=5.0)
        
    def move_to_target(self, target_pos):
        self.path = [kinematics.forward_kinematics(self.current_q_rad)[-1]]
        
        start_pos = kinematics.forward_kinematics(self.current_q_rad)[-1]
        self.trajectory_gen.plan_linear_path(start_pos, target_pos, max_vel=100.0, max_accel=150.0)
        
        K_null = 0.5
        q_rest = np.array([0, np.pi/2, 0, 0])
        dt = 0.02
        
        max_steps = 2000
        for _ in range(max_steps):
            current_time = time.time()
            if _ == 0: self.trajectory_gen.start_time = current_time # Hack to sync start time
            
            # 1. Get desired state
            # We simulate time passing by incrementing manually since this is a fast loop
            sim_time = self.trajectory_gen.start_time + (_ * dt)
            traj_pos, traj_vel, finished = self.trajectory_gen.get_target(sim_time)
            
            if finished:
                print("Trajectory finished.")
                break
            
            # 2. Control
            J = kinematics.calculate_jacobian(self.current_q_rad)
            lambda_val = kinematics.calculate_adaptive_damping(J)
            J_pinv_dls = kinematics.get_jacobian_pinv_damped(J, damping_factor=lambda_val)
            
            current_pos = kinematics.forward_kinematics(self.current_q_rad)[-1]
            self.path.append(current_pos)
            
            error = traj_pos - current_pos
            v_cmd = traj_vel + 5.0 * error
            
            q_dot_primary = J_pinv_dls @ v_cmd
            
            J_pinv_std = np.linalg.pinv(J)
            null_space_projector = np.eye(4) - (J_pinv_std @ J)
            unwrapped_q_rest = np.unwrap(np.vstack([self.current_q_rad, q_rest]), axis=0)[1]
            q_dot_secondary = K_null * (unwrapped_q_rest - self.current_q_rad)
            
            q_dot = q_dot_primary + (null_space_projector @ q_dot_secondary)
            
            # 3. Limit
            q_dot_limited = []
            for i in range(4):
                q_dot_limited.append(self.joint_controllers[i].update(q_dot[i], dt))
            
            # 4. Update
            self.current_q_rad += np.array(q_dot_limited) * dt

def run_simulation():
    controller = MockController()
    
    # Target: Move from (0, 0, 331) to (200, 0, 100)
    target_pos = np.array([200.0, 0.0, 100.0])
    
    print(f"Start Pos: {kinematics.forward_kinematics(controller.current_q_rad)[-1]}")
    print(f"Target Pos: {target_pos}")
    
    controller.move_to_target(target_pos)
    
    path = np.array(controller.path)
    final_pos = path[-1]
    print(f"Final Pos: {final_pos}")
    print(f"Error: {np.linalg.norm(target_pos - final_pos)}")
    
    # Check linearity
    # Vector from start to end
    start_pos = path[0]
    vec_ideal = target_pos - start_pos
    vec_ideal_norm = vec_ideal / np.linalg.norm(vec_ideal)
    
    max_deviation = 0.0
    for p in path:
        vec_p = p - start_pos
        proj = np.dot(vec_p, vec_ideal_norm) * vec_ideal_norm
        deviation = np.linalg.norm(vec_p - proj)
        if deviation > max_deviation: max_deviation = deviation
        
    print(f"Max Deviation from Line: {max_deviation:.4f} mm")

if __name__ == "__main__":
    run_simulation()
