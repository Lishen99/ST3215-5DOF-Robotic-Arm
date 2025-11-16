import numpy as np
import json

# Link lengths in mm
L1 = 133.39 # Shoulder to Elbow
L2 = 124.97 # Elbow to Wrist
L3 = 73.39  # Wrist to Tip
LINK_LENGTHS = [L1, L2, L3]

# --- Advanced Kinematic Functions ---

def get_jacobian_pinv_damped(J, damping_factor=0.1):
    """
    Calculates the damped least-squares pseudoinverse (Levenberg-Marquardt).
    """
    J_T = J.T
    I = np.identity(J.shape[0])
    k_sq = damping_factor**2
    inv_term = np.linalg.inv(J @ J_T + k_sq * I)
    return J_T @ inv_term

def calculate_manipulability(J):
    """
    Calculates the Yoshikawa manipulability index.
    w = sqrt(det(J @ J.T))
    A measure of how well the arm can move in all Cartesian directions.
    Returns 0 if the matrix is singular.
    """
    try:
        det_val = np.linalg.det(J @ J.T)
        # Return 0 for negative determinants which can happen with numerical instability
        return np.sqrt(max(0, det_val))
    except np.linalg.LinAlgError:
        return 0

def calculate_manipulability_gradient(q_rad):
    """
    Calculates the gradient of the manipulability index with respect to the joint angles.
    This is used to drive the arm towards more dextrous configurations.
    Uses a numerical approximation (finite differences).
    """
    grad = np.zeros_like(q_rad)
    epsilon = 1e-5  # Small perturbation for finite differences

    # Calculate manipulability at the current configuration
    J_current = calculate_jacobian(q_rad)
    w_current = calculate_manipulability(J_current)

    for i in range(len(q_rad)):
        # Perturb one joint angle
        q_perturbed = np.copy(q_rad)
        q_perturbed[i] += epsilon

        # Calculate manipulability at the perturbed configuration
        J_perturbed = calculate_jacobian(q_perturbed)
        w_perturbed = calculate_manipulability(J_perturbed)

        # Approximate the partial derivative
        grad[i] = (w_perturbed - w_current) / epsilon
    
    return grad

# --- Standard Kinematic Functions ---

def forward_kinematics(q_rad):
    q1, q2, q3, q4 = q_rad
    p0 = np.array([0., 0., 0.]); p1 = p0
    x2 = L1 * np.cos(q2) * np.cos(q1); y2 = L1 * np.cos(q2) * np.sin(q1); z2 = L1 * np.sin(q2)
    p2 = np.array([x2, y2, z2])
    x3 = (L1*np.cos(q2) + L2*np.cos(q2+q3))*np.cos(q1); y3 = (L1*np.cos(q2) + L2*np.cos(q2+q3))*np.sin(q1); z3 = L1*np.sin(q2) + L2*np.sin(q2+q3)
    p3 = np.array([x3, y3, z3])
    x4 = (L1*np.cos(q2) + L2*np.cos(q2+q3) + L3*np.cos(q2+q3+q4))*np.cos(q1); y4 = (L1*np.cos(q2) + L2*np.cos(q2+q3) + L3*np.cos(q2+q3+q4))*np.sin(q1); z4 = L1*np.sin(q2) + L2*np.sin(q2+q3) + L3*np.sin(q2+q3+q4)
    p4 = np.array([x4, y4, z4])
    return [p0, p1, p2, p3, p4]

def calculate_jacobian(q_rad):
    q1, q2, q3, q4 = q_rad
    s1, c1 = np.sin(q1), np.cos(q1); s2, c2 = np.sin(q2), np.cos(q2); s23, c23 = np.sin(q2+q3), np.cos(q2+q3); s234, c234 = np.sin(q2+q3+q4), np.cos(q2+q3+q4)
    R = L1*c2 + L2*c23 + L3*c234
    dx_dq1 = -R*s1; dy_dq1 = R*c1; dz_dq1 = 0
    dx_dq2 = (-L1*s2 - L2*s23 - L3*s234)*c1; dy_dq2 = (-L1*s2 - L2*s23 - L3*s234)*s1; dz_dq2 = L1*c2 + L2*c23 + L3*c234
    dx_dq3 = (-L2*s23 - L3*s234)*c1; dy_dq3 = (-L2*s23 - L3*s234)*s1; dz_dq3 = L2*c23 + L3*c234
    dx_dq4 = (-L3*s234)*c1; dy_dq4 = (-L3*s234)*s1; dz_dq4 = L3*c234
    return np.array([[dx_dq1,dx_dq2,dx_dq3,dx_dq4], [dy_dq1,dy_dq2,dy_dq3,dy_dq4], [dz_dq1,dz_dq2,dz_dq3,dz_dq4]])

def inverse_kinematics(target_pos, current_q_rad, wrist_angle_rad=0):
    try:
        with open('jacob_ik/servo_limits.json', 'r') as f: servo_limits_raw = json.load(f)
        joint_limits_rad = _get_joint_limits_rad(servo_limits_raw)
    except (IOError, json.JSONDecodeError): joint_limits_rad = {i: (-np.pi*2, np.pi*2) for i in range(1, 6)}
    solutions = _calculate_ik_solutions(target_pos, current_q_rad, wrist_angle_rad)
    if not solutions: return None
    motor_map = {0: 1, 1: 2, 2: 4, 3: 5}
    sol_up, sol_down = solutions['up'], solutions['down']
    is_up_valid, is_down_valid = _is_solution_valid(sol_up, joint_limits_rad, motor_map), _is_solution_valid(sol_down, joint_limits_rad, motor_map)
    r_wrist, _ = _get_wrist_coords(target_pos, wrist_angle_rad)
    is_near_elbow_singularity = r_wrist > 0.95 * (L1 + L2)
    if is_near_elbow_singularity:
        if is_up_valid and is_down_valid:
            dist_up = np.sum((np.array(sol_up) - current_q_rad)**2); dist_down = np.sum((np.array(sol_down) - current_q_rad)**2)
            return sol_up if dist_up < dist_down else sol_down
        elif is_up_valid: return sol_up
        elif is_down_valid: return sol_down
    else:
        if is_up_valid: return sol_up
        if is_down_valid: return sol_down
    return None

def _get_wrist_coords(target_pos, wrist_angle_rad=0):
    x, y, z = target_pos; phi = wrist_angle_rad; r_tip = np.sqrt(x**2 + y**2)
    r_wrist = r_tip - L3 * np.cos(phi); z_wrist = z - L3 * np.sin(phi)
    return r_wrist, z_wrist

def _calculate_ik_solutions(target_pos, current_q_rad, wrist_angle_rad=0):
    x, y, z = target_pos
    q1 = np.arctan2(y, x) if not (np.isclose(x, 0) and np.isclose(y, 0)) else current_q_rad[0]
    r_wrist, z_wrist = _get_wrist_coords(target_pos, wrist_angle_rad)
    D_sq = r_wrist**2 + z_wrist**2
    if not ((L1 - L2)**2 <= D_sq <= (L1 + L2)**2): return {}
    cos_q3_arg = np.clip((D_sq - L1**2 - L2**2) / (2 * L1 * L2), -1.0, 1.0); q3 = np.arccos(cos_q3_arg)
    alpha = np.arctan2(z_wrist, r_wrist); D = np.sqrt(max(0, D_sq))
    beta_cos_arg = np.clip((D_sq + L1**2 - L2**2) / (2 * D * L1), -1.0, 1.0); beta = np.arccos(beta_cos_arg)
    q2_up = alpha + beta; q3_up = -q3; q4_up = wrist_angle_rad - q2_up - q3_up
    sol_up = [q1, q2_up, q3_up, q4_up]
    q2_down = alpha - beta; q3_down = q3; q4_down = wrist_angle_rad - q2_down - q3_down
    sol_down = [q1, q2_down, q3_down, q4_down]
    return {'up': sol_up, 'down': sol_down}

def _is_solution_valid(q_sol, joint_limits_rad, motor_map):
    if not q_sol: return False
    for i, q_val in enumerate(q_sol):
        motor_id = motor_map.get(i)
        if motor_id and motor_id in joint_limits_rad:
            min_lim, max_lim = joint_limits_rad[motor_id]
            if not (min_lim - 0.01 <= q_val <= max_lim + 0.01): return False
    return True

def _get_joint_limits_rad(servo_limits_raw):
    joint_limits_rad = {}
    for mid_str, limits in servo_limits_raw.items():
        mid = int(mid_str)
        min_angle = _pos_to_angle(mid, limits['min'], servo_limits_raw); max_angle = _pos_to_angle(mid, limits['max'], servo_limits_raw)
        joint_limits_rad[mid] = tuple(sorted((np.deg2rad(min_angle), np.deg2rad(max_angle))))
    return joint_limits_rad

def _pos_to_angle(motor_id, pos, servo_limits_raw):
    sid = str(motor_id)
    if sid not in servo_limits_raw: return 0
    limits = servo_limits_raw[sid]; min_p, max_p = limits['min'], limits['max']; center_p = (min_p + max_p) / 2
    center_angle = 90.0 if motor_id == 2 else 0.0
    range_deg = 360.0 if motor_id == 1 else 270.0
    ticks_per_deg = ((max_p - min_p) / range_deg) or 1
    if motor_id == 2:
        logical_pos = (min_p + max_p) - pos
        return center_angle + (logical_pos - center_p) / ticks_per_deg
    else:
        return center_angle + (pos - center_p) / ticks_per_deg
