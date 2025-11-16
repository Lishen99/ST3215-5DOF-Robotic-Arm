import numpy as np
from math import atan2, acos, asin, sqrt, sin, cos, pi, degrees, radians

L1, L2, L3 = 133.39, 124.97, 73.39
LINK_LENGTHS = [L1, L2, L3]

def forward_kinematics(q_rad, L=LINK_LENGTHS):
    t1, t2, t3, t4 = q_rad; L1, L2, L3 = L
    r = [0, L1*cos(t2), L1*cos(t2)+L2*cos(t2+t3), L1*cos(t2)+L2*cos(t2+t3)+L3*cos(t2+t3+t4)]
    z = [0, L1*sin(t2), L1*sin(t2)+L2*sin(t2+t3), L1*sin(t2)+L2*sin(t2+t3)+L3*sin(t2+t3+t4)]
    points = [np.array([r[i]*cos(t1), r[i]*sin(t1), z[i]]) for i in range(4)]
    return [points[0], points[0], points[1], points[2], points[3]]

def _find_best_wrist_angle(target_pos, L, joint_limits_deg, preferred_phi_deg=None):
    L1, L2, L3 = L; x, y, z = target_pos
    r_e = sqrt(x**2 + y**2); d_e = sqrt(r_e**2 + z**2)
    R_outer, R_inner = L1 + L2, abs(L1 - L2)
    if not (R_inner - L3 <= d_e <= R_outer + L3): return None

    alpha = atan2(z, r_e)
    cos_gamma_outer = np.clip((d_e**2 + L3**2 - R_outer**2) / (2 * d_e * L3 if d_e * L3 != 0 else 1), -1.0, 1.0)
    kinematic_min_rad, kinematic_max_rad = alpha - acos(cos_gamma_outer), alpha + acos(cos_gamma_outer)
    wrist_limit_min_rad, wrist_limit_max_rad = radians(joint_limits_deg[4][0]), radians(joint_limits_deg[4][1])
    valid_min_rad, valid_max_rad = max(kinematic_min_rad, wrist_limit_min_rad), min(kinematic_max_rad, wrist_limit_max_rad)
    if valid_min_rad > valid_max_rad: return None

    if preferred_phi_deg is not None:
        preferred_phi_rad = radians(preferred_phi_deg)
        if valid_min_rad <= preferred_phi_rad <= valid_max_rad: return preferred_phi_deg

    for angle in [-45, 0, -90, 45]:
        if valid_min_rad <= radians(angle) <= valid_max_rad: return angle

    return degrees((valid_min_rad + valid_max_rad) / 2)

def _solve_ik_for_angle(target_pos, gripper_angle_deg, L, joint_limits_deg):
    x, y, z = target_pos; L1, L2, L3 = L; phi = radians(gripper_angle_deg)
    theta1 = atan2(y, x); r_e = sqrt(x**2 + y**2)
    r_w, z_w = r_e - L3*cos(phi), z - L3*sin(phi)
    d_sq = r_w**2 + z_w**2
    if not (abs(L1 - L2)**2 - 1e-6 <= d_sq <= (L1 + L2)**2 + 1e-6): return None

    cos_theta3 = np.clip((d_sq - L1**2 - L2**2) / (2 * L1 * L2 if L1 * L2 != 0 else 1), -1.0, 1.0)
    theta3 = -acos(cos_theta3)
    d = sqrt(d_sq); 
    if d == 0: return None
    cos_beta = np.clip((d_sq + L1**2 - L2**2) / (2 * L1 * d), -1.0, 1.0)
    theta2 = atan2(z_w, r_w) + acos(cos_beta)
    theta4 = phi - theta2 - theta3
    solution_rad = [theta1, theta2, theta3, theta4]
    solution_deg = [degrees(a) for a in solution_rad]

    j_limits = joint_limits_deg
    if not (j_limits[1][0] <= solution_deg[0] <= j_limits[1][1]): return None
    if not (j_limits[2][0] <= solution_deg[1] <= j_limits[2][1]): return None
    if not (j_limits[3][0] <= solution_deg[2] <= j_limits[3][1]): return None
    if not (j_limits[4][0] <= solution_deg[3] <= j_limits[4][1]): return None
    return solution_rad

def inverse_kinematics(target_pos, joint_limits_deg, preferred_phi_deg=None, use_locked_angle=False, L=LINK_LENGTHS):
    mapped_limits = {1: joint_limits_deg.get(1, (-180, 180)), 2: joint_limits_deg.get(2, (-90, 180)), 3: joint_limits_deg.get(4, (-180, 180)), 4: joint_limits_deg.get(5, (-180, 180))}
    
    if use_locked_angle and preferred_phi_deg is not None:
        # Locked mode: only try the specified angle
        return _solve_ik_for_angle(target_pos, preferred_phi_deg, L, mapped_limits)
    else:
        # Automatic mode: find the best possible angle, prioritizing the preferred one for consistency
        best_angle_deg = _find_best_wrist_angle(target_pos, L, mapped_limits, preferred_phi_deg)
        if best_angle_deg is None: return None
        return _solve_ik_for_angle(target_pos, best_angle_deg, L, mapped_limits)
