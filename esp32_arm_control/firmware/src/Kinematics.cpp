#include "Kinematics.h"

Kinematics::Kinematics() {}

float Kinematics::wrap_angle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

Matrix<3> Kinematics::forward_kinematics(const Matrix<4>& q) {
    float q1 = q(0); // Base rotation
    // Add 90° offset: when q2=0 (motor at center), arm points UP (Z direction)
    // In the kinematic model, vertical = q2 = PI/2
    float q2 = q(1) + M_PI_2; // Shoulder (offset so 0 = vertical)
    float q3 = q(2); // Elbow
    float q4 = q(3); // Wrist Pitch

    float c1 = cos(q1), s1 = sin(q1);
    float c2 = cos(q2), s2 = sin(q2);
    float c23 = cos(q2 + q3), s23 = sin(q2 + q3);
    float c234 = cos(q2 + q3 + q4), s234 = sin(q2 + q3 + q4);

    // Radial distance from base axis (horizontal projection)
    float r = L1 * c2 + L2 * c23 + L3 * c234;

    Matrix<3> pos;
    pos(0) = r * c1;  // x (forward when q1=0)
    pos(1) = r * s1;  // y (left when q1=0)
    pos(2) = L1 * s2 + L2 * s23 + L3 * s234;  // z (up)

    return pos;
}

void Kinematics::get_joint_positions(const Matrix<4>& q, Matrix<3>* positions) {
    float q1 = q(0);
    float q2 = q(1) + M_PI_2; // Apply same offset as FK
    float q3 = q(2), q4 = q(3);
    float c1 = cos(q1), s1 = sin(q1);
    float c2 = cos(q2), s2 = sin(q2);
    float c23 = cos(q2 + q3), s23 = sin(q2 + q3);
    float c234 = cos(q2 + q3 + q4), s234 = sin(q2 + q3 + q4);

    // p0 - Base origin
    positions[0](0) = 0; positions[0](1) = 0; positions[0](2) = 0;
    
    // p1 - Same as p0 for this arm
    positions[1](0) = 0; positions[1](1) = 0; positions[1](2) = 0;
    
    // p2 - After L1 (shoulder)
    float r2 = L1 * c2;
    positions[2](0) = r2 * c1;
    positions[2](1) = r2 * s1;
    positions[2](2) = L1 * s2;
    
    // p3 - After L2 (elbow)
    float r3 = L1 * c2 + L2 * c23;
    positions[3](0) = r3 * c1;
    positions[3](1) = r3 * s1;
    positions[3](2) = L1 * s2 + L2 * s23;
    
    // p4 - End effector (after L3)
    float r4 = L1 * c2 + L2 * c23 + L3 * c234;
    positions[4](0) = r4 * c1;
    positions[4](1) = r4 * s1;
    positions[4](2) = L1 * s2 + L2 * s23 + L3 * s234;
}

Matrix<3, 4> Kinematics::calculate_jacobian(const Matrix<4>& q) {
    float q1 = q(0);
    float q2 = q(1) + M_PI_2; // Apply same offset as FK
    float q3 = q(2), q4 = q(3);
    
    float s1 = sin(q1), c1 = cos(q1);
    float s2 = sin(q2), c2 = cos(q2);
    float s23 = sin(q2 + q3), c23 = cos(q2 + q3);
    float s234 = sin(q2 + q3 + q4), c234 = cos(q2 + q3 + q4);

    // Radial distance
    float R = L1 * c2 + L2 * c23 + L3 * c234;
    
    // Partial derivatives of R
    float dR_dq2 = -L1 * s2 - L2 * s23 - L3 * s234;
    float dR_dq3 = -L2 * s23 - L3 * s234;
    float dR_dq4 = -L3 * s234;
    
    // Partial derivatives of z
    float dz_dq2 = L1 * c2 + L2 * c23 + L3 * c234;
    float dz_dq3 = L2 * c23 + L3 * c234;
    float dz_dq4 = L3 * c234;

    Matrix<3, 4> J;
    
    // Row 0: dx/dq
    J(0, 0) = -R * s1;        // dx/dq1
    J(0, 1) = dR_dq2 * c1;    // dx/dq2
    J(0, 2) = dR_dq3 * c1;    // dx/dq3
    J(0, 3) = dR_dq4 * c1;    // dx/dq4

    // Row 1: dy/dq
    J(1, 0) = R * c1;         // dy/dq1
    J(1, 1) = dR_dq2 * s1;    // dy/dq2
    J(1, 2) = dR_dq3 * s1;    // dy/dq3
    J(1, 3) = dR_dq4 * s1;    // dy/dq4

    // Row 2: dz/dq
    J(2, 0) = 0;              // dz/dq1
    J(2, 1) = dz_dq2;         // dz/dq2
    J(2, 2) = dz_dq3;         // dz/dq3
    J(2, 3) = dz_dq4;         // dz/dq4

    return J;
}

float Kinematics::calculate_manipulability(const Matrix<3, 4>& J) {
    // w = sqrt(det(J * J^T))
    Matrix<4, 3> JT = ~J;
    Matrix<3, 3> JJT = J * JT;
    
    // Calculate determinant of 3x3 matrix manually
    float det = JJT(0,0) * (JJT(1,1)*JJT(2,2) - JJT(1,2)*JJT(2,1))
              - JJT(0,1) * (JJT(1,0)*JJT(2,2) - JJT(1,2)*JJT(2,0))
              + JJT(0,2) * (JJT(1,0)*JJT(2,1) - JJT(1,1)*JJT(2,0));
    
    if (det < 0) det = 0; // Numerical stability
    return sqrt(det);
}

float Kinematics::calculate_adaptive_damping(const Matrix<3, 4>& J, float lambda_max, float epsilon) {
    float w = calculate_manipulability(J);
    if (w < epsilon) {
        float ratio = 1.0f - w / epsilon;
        return lambda_max * ratio * ratio;
    }
    return 0.0f;
}

Matrix<4, 3> Kinematics::get_jacobian_pinv_dls(const Matrix<3, 4>& J, float damping) {
    // DLS: J_pinv = J^T * (J * J^T + lambda^2 * I)^-1
    Matrix<4, 3> JT = ~J;
    Matrix<3, 3> JJT = J * JT;
    
    // Add damping to diagonal
    float lambda_sq = damping * damping;
    JJT(0, 0) += lambda_sq;
    JJT(1, 1) += lambda_sq;
    JJT(2, 2) += lambda_sq;
    
    // Invert the 3x3 matrix
    Matrix<3, 3> JJT_inv = Inverse(JJT);
    
    return JT * JJT_inv;
}

Matrix<4> Kinematics::clamp_joints(const Matrix<4>& q) {
    Matrix<4> clamped;
    for (int i = 0; i < 4; i++) {
        clamped(i) = q(i);
        if (clamped(i) < JOINT_LIMITS[i].min_rad) 
            clamped(i) = JOINT_LIMITS[i].min_rad;
        if (clamped(i) > JOINT_LIMITS[i].max_rad) 
            clamped(i) = JOINT_LIMITS[i].max_rad;
    }
    return clamped;
}

Matrix<4> Kinematics::clamp_velocities(const Matrix<4>& q, const Matrix<4>& q_dot, float buffer_rad) {
    Matrix<4> clamped_vel = q_dot;
    
    for (int i = 0; i < 4; i++) {
        // If near max limit and trying to go higher, clamp to 0
        if (q(i) >= JOINT_LIMITS[i].max_rad - buffer_rad && q_dot(i) > 0) {
            clamped_vel(i) = 0;
        }
        // If near min limit and trying to go lower, clamp to 0
        else if (q(i) <= JOINT_LIMITS[i].min_rad + buffer_rad && q_dot(i) < 0) {
            clamped_vel(i) = 0;
        }
    }
    
    return clamped_vel;
}

bool Kinematics::inverse_kinematics_analytical(const Matrix<3>& target_pos, const Matrix<4>& current_q,
                                                float wrist_angle_rad, Matrix<4>& result) {
    float x = target_pos(0);
    float y = target_pos(1);
    float z = target_pos(2);
    
    // Determine if we should use Y≈0 special handling (arm flips over top)
    bool y_near_zero = fabs(y) < 1.0f;
    bool negative_x = (x < 0) && y_near_zero;
    
    // q1: Base angle
    float q1;
    if (y_near_zero) {
        q1 = 0.0f;  // Keep base at 0 for Y≈0 - arm flips over top
    } else if (fabs(x) < 0.001f && fabs(y) < 0.001f) {
        q1 = current_q(0);
    } else {
        q1 = atan2(y, x);
    }
    
    // For Y≈0 with base at 0:
    // - Positive X: r is positive, arm reaches forward
    // - Negative X: r is negative, arm reaches backward (over the top)
    // The "r" in our 2D IK plane can be negative!
    float r_tip = (q1 == 0.0f) ? x : sqrt(x*x + y*y);  // r can be negative for x<0!
    
    // Gripper world angle: 
    // For +X reach: wrist_angle=0 means gripper horizontal pointing +X
    // For -X reach: wrist_angle=0 means gripper horizontal pointing -X
    // In the r-z plane, phi is measured from the +r axis
    // For negative r, we're working in a "flipped" coordinate frame
    float phi_rz = wrist_angle_rad;  // Angle from horizontal in the reaching direction
    
    // Wrist position: back off from tip along gripper direction
    // The gripper points "outward" in the +r direction (or -r for negative reach)
    float r_wrist = r_tip - L3 * cos(phi_rz);  // Can be negative
    float z_wrist = z - L3 * sin(phi_rz);
    
    // Distance from shoulder to wrist (always positive)
    float D_sq = r_wrist * r_wrist + z_wrist * z_wrist;
    float D = sqrt(D_sq);
    
    // Check reachability
    float L1_plus_L2 = L1 + L2;
    float L1_minus_L2 = fabs(L1 - L2);
    
    if (D > L1_plus_L2 || D < L1_minus_L2) {
        Serial.printf("IK: Unreachable D=%.1f range=[%.1f,%.1f]\n", D, L1_minus_L2, L1_plus_L2);
        return false;
    }
    
    // Elbow angle using law of cosines
    float cos_q3_arg = (D_sq - L1*L1 - L2*L2) / (2.0f * L1 * L2);
    cos_q3_arg = fmax(-1.0f, fmin(1.0f, cos_q3_arg));
    float q3_mag = acos(cos_q3_arg);
    
    // Angle to wrist position in the r-z plane
    // atan2 handles negative r correctly!
    float alpha = atan2(z_wrist, r_wrist);
    
    // Angle offset due to elbow
    float beta_cos_arg = (D_sq + L1*L1 - L2*L2) / (2.0f * D * L1);
    beta_cos_arg = fmax(-1.0f, fmin(1.0f, beta_cos_arg));
    float beta = acos(beta_cos_arg);
    
    // Solution A: Elbow UP (q3 negative) - arm bends "inward"
    // Solution B: Elbow DOWN (q3 positive) - arm bends "outward"
    // q2_raw is the angle of the first link from the +r axis (horizontal forward)
    float q2_raw_A = alpha + beta;
    float q3_A = -q3_mag;  // Elbow UP
    
    float q2_raw_B = alpha - beta;
    float q3_B = q3_mag;   // Elbow DOWN
    
    // phi_world is the sum q2_raw + q3 + q4 (total angle of gripper from +r axis)
    // For the gripper to point "outward" (in +r direction if r>0, -r direction if r<0)
    // phi_world should equal wrist_angle_rad (for +r) or PI + wrist_angle_rad (for -r)
    float phi_world = negative_x ? (M_PI + wrist_angle_rad) : wrist_angle_rad;
    
    float q4_A = wrap_angle(phi_world - q2_raw_A - q3_A);
    float q4_B = wrap_angle(phi_world - q2_raw_B - q3_B);
    
    // Convert q2_raw to motor space (subtract PI/2 because motor 0 = vertical)
    float q2_A = wrap_angle(q2_raw_A - M_PI_2);
    float q2_B = wrap_angle(q2_raw_B - M_PI_2);
    
    Matrix<4> sol_A = {q1, q2_A, q3_A, q4_A};
    Matrix<4> sol_B = {q1, q2_B, q3_B, q4_B};
    
    // Check joint limits
    bool A_valid = true, B_valid = true;
    for (int i = 0; i < 4; i++) {
        if (sol_A(i) < JOINT_LIMITS[i].min_rad - 0.05f || 
            sol_A(i) > JOINT_LIMITS[i].max_rad + 0.05f) {
            A_valid = false;
        }
        if (sol_B(i) < JOINT_LIMITS[i].min_rad - 0.05f || 
            sol_B(i) > JOINT_LIMITS[i].max_rad + 0.05f) {
            B_valid = false;
        }
    }
    
    // Prefer elbow UP (solution A with q3 negative)
    if (A_valid) {
        result = sol_A;
        return true;
    } else if (B_valid) {
        result = sol_B;
        return true;
    }
    
    return false;
}

bool Kinematics::is_position_reachable(float x, float y, float z, char* reason, size_t reason_len) {
    // Arm reach parameters
    const float total_reach = L1 + L2 + L3;  // Maximum straight-arm reach: ~331.75mm
    const float min_z = -200.0f;  // Allow negative Z - arm can reach below base level
    const float max_z = total_reach + 10.0f;  // Maximum Z (arm pointing straight up)
    
    // Calculate horizontal distance from base axis
    float r_horizontal = sqrtf(x*x + y*y);
    
    // Calculate 3D distance from base
    float dist_3d = sqrtf(x*x + y*y + z*z);
    
    // Check if position is beyond maximum reach
    if (dist_3d > total_reach) {
        snprintf(reason, reason_len, "Too far (%.0fmm > %.0fmm max)", dist_3d, total_reach);
        return false;
    }
    
    // Check Z bounds - only enforce max, not min (arm can go below base)
    if (z > max_z) {
        snprintf(reason, reason_len, "Z too high (%.0fmm)", z);
        return false;
    }
    
    // For low Z, limit horizontal reach based on how low
    // At Z=25, max reach is about 280mm. At Z=100, can reach further.
    float max_r_at_z = 280.0f + (z - 25.0f) * 0.5f;  // Increases with Z
    if (z < 100.0f && r_horizontal > max_r_at_z) {
        snprintf(reason, reason_len, "Too far at Z=%.0f (max R=%.0f)", z, max_r_at_z);
        return false;
    }
    
    // Check minimum distance - can't reach directly under/over the base easily
    // The arm needs some horizontal reach except when going straight up
    float min_dist = 30.0f;  // Minimum distance from base axis
    if (z < 250.0f && r_horizontal < min_dist && z > 50.0f) {
        // This is the "dead zone" directly above the base at medium heights
        snprintf(reason, reason_len, "Too close to base axis");
        return false;
    }
    
    // For positions behind the base (negative X with Y≈0), check the flip is possible
    // The arm CAN reach negative X by flipping over the top
    // Just need to make sure the 2D reach in the r-z plane is valid
    float r_2d = sqrtf(r_horizontal*r_horizontal + z*z);  // Distance in vertical plane
    
    // Triangle inequality for the arm links
    float max_reach_2d = L1 + L2 + L3;
    float min_reach_2d = 20.0f;  // Roughly minimum when fully folded
    
    if (r_2d > max_reach_2d) {
        snprintf(reason, reason_len, "Out of reach (%.0fmm)", r_2d);
        return false;
    }
    
    if (r_2d < min_reach_2d && z < 100.0f) {
        snprintf(reason, reason_len, "Too close to base");
        return false;
    }
    
    // Position seems reachable
    return true;
}