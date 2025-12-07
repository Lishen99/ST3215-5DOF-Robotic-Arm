#ifndef ANALYTICAL_IK_H
#define ANALYTICAL_IK_H

#include <BasicLinearAlgebra.h>
#include "ArmModel.h"
#include <math.h>

using namespace BLA;

/**
 * ANALYTICAL INVERSE KINEMATICS
 * 
 * Closed-form IK solution for the 4-DOF arm.
 * Can explicitly select elbow-up or elbow-down configuration.
 * 
 * This is used by MPC to compute reference joint trajectories
 * from Cartesian targets.
 */

struct IKResult {
    Matrix<N_JOINTS> q;     // Joint angles
    bool valid;             // True if solution exists
    ElbowConfig config;     // Which configuration was used
    float error;            // Position error (mm)
};

/**
 * Normalize angle to [-π, π]
 */
inline float normalizeAngle(float a) {
    while (a > M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

/**
 * Choose the base angle (theta1) that requires minimum rotation from current
 * This prevents wrap-around (e.g., going from +170° to -170° the long way)
 * 
 * For J1 which is NOT continuous (can't cross 0/4095 boundary), we need to:
 * 1. Prefer the solution closest to current angle
 * 2. Ensure the path stays within [-π, +π] (approx -180° to +180°)
 */
inline float chooseBaseAngle(float target_theta1, float current_theta1) {
    // Normalize both angles to [-π, π]
    target_theta1 = normalizeAngle(target_theta1);
    current_theta1 = normalizeAngle(current_theta1);
    
    // The equivalent angle is target_theta1 ± 2π, but we want to stay in [-π, π]
    // For non-continuous joint, just use the direct path
    float diff = target_theta1 - current_theta1;
    
    // If the direct path crosses the ±π boundary, we have a problem
    // For a truly non-continuous joint at 0/4095, we should avoid crossing ±π
    // But the joint limits are ±3.2 rad (~±183°) so we have some margin
    
    // Simply return the normalized target - the path planner should handle intermediate waypoints
    return target_theta1;
}

/**
 * Solve IK analytically with explicit configuration selection
 * 
 * MATCHES PYTHON EXACTLY:
 *   theta1 = atan2(y, x)
 *   r_w, z_w = r_e - L3*cos(phi), z - L3*sin(phi)
 *   cos_theta3 = (d^2 - L1^2 - L2^2) / (2*L1*L2)
 *   theta3 = -acos(cos_theta3)  (elbow up)
 *   theta2 = atan2(z_w, r_w) + acos(cos_beta)
 *   theta4 = phi - theta2 - theta3
 * 
 * @param target_pos Target end-effector position [x, y, z] in mm
 * @param wrist_angle Desired wrist pitch angle from horizontal (rad) - this is phi
 * @param config Elbow configuration (UP, DOWN, or AUTO)
 * @param current_q Current joint angles (used for AUTO mode)
 * @return IKResult with joint angles and validity
 */
inline IKResult solveIK(const Matrix<3>& target_pos, 
                        float wrist_angle,
                        ElbowConfig config = ElbowConfig::UP,
                        const Matrix<N_JOINTS>& current_q = {0, 0, 0, 0}) {
    IKResult result;
    result.valid = false;
    result.error = 999.0f;
    
    float x = target_pos(0);
    float y = target_pos(1);
    float z = target_pos(2);
    
    // Step 1: Base angle (theta1) from x, y - matches Python
    float theta1 = atan2f(y, x);
    
    // Clamp base angle to limits
    if (theta1 < JOINT_LIMITS[0].min_rad) theta1 = JOINT_LIMITS[0].min_rad;
    if (theta1 > JOINT_LIMITS[0].max_rad) theta1 = JOINT_LIMITS[0].max_rad;
    
    // Step 2: Reduce to 2D problem in the vertical plane
    float r_e = sqrtf(x*x + y*y);  // Horizontal distance to end-effector
    
    // Wrist position (before L3) - matches Python
    float phi = wrist_angle;  // Desired end-effector angle from horizontal
    float r_w = r_e - L3 * cosf(phi);
    float z_w = z - L3 * sinf(phi);
    
    // Distance from shoulder to wrist
    float d_sq = r_w*r_w + z_w*z_w;
    float d = sqrtf(d_sq);
    
    // Check reachability - matches Python
    float R_outer = L1 + L2;
    float R_inner = fabsf(L1 - L2);
    if (d_sq > R_outer * R_outer + 1e-6f || d_sq < R_inner * R_inner - 1e-6f) {
        result.error = fabsf(d - (L1 + L2));
        return result;
    }
    if (d < 1e-6f) {
        result.error = 999.0f;
        return result;
    }
    
    // Step 3: Elbow angle (theta3) using law of cosines - matches Python
    float cos_theta3 = (d_sq - L1*L1 - L2*L2) / (2.0f * L1 * L2);
    cos_theta3 = fmaxf(-1.0f, fminf(1.0f, cos_theta3));
    
    // Choose elbow configuration
    if (config == ElbowConfig::AUTO) {
        config = getElbowConfig(current_q);
    }
    
    float theta3;
    if (config == ElbowConfig::UP) {
        theta3 = -acosf(cos_theta3);  // Elbow up: negative - matches Python
    } else {
        theta3 = acosf(cos_theta3);   // Elbow down: positive
    }
    
    // Step 4: Shoulder angle (theta2) - matches Python
    float cos_beta = (d_sq + L1*L1 - L2*L2) / (2.0f * L1 * d);
    cos_beta = fmaxf(-1.0f, fminf(1.0f, cos_beta));
    float beta = acosf(cos_beta);
    
    float theta2;
    if (config == ElbowConfig::UP) {
        theta2 = atan2f(z_w, r_w) + beta;  // Matches Python for elbow-up
    } else {
        theta2 = atan2f(z_w, r_w) - beta;
    }
    
    // Step 5: Wrist pitch angle (theta4) - matches Python
    // NOTE: theta4 = phi - (q2_kinematic) - theta3
    //       where q2_kinematic = theta2 (the value computed above)
    float theta4 = phi - theta2 - theta3;
    
    // Normalize theta4 to [-π, π]
    while (theta4 > M_PI) theta4 -= 2.0f * M_PI;
    while (theta4 < -M_PI) theta4 += 2.0f * M_PI;
    
    // Convert from kinematic frame to joint frame
    // FK uses: q2_kinematic = q2_joint + 90°
    // So: q2_joint = theta2 - 90°
    float q2_joint = theta2 - M_PI_2;
    
    // Check all joint limits (using joint frame values)
    if (q2_joint < JOINT_LIMITS[1].min_rad || q2_joint > JOINT_LIMITS[1].max_rad) {
        result.error = 100.0f;
        return result;
    }
    if (theta3 < JOINT_LIMITS[2].min_rad || theta3 > JOINT_LIMITS[2].max_rad) {
        result.error = 100.0f;
        return result;
    }
    if (theta4 < JOINT_LIMITS[3].min_rad || theta4 > JOINT_LIMITS[3].max_rad) {
        result.error = 100.0f;
        return result;
    }
    
    // Build result (in joint frame)
    result.q(0) = theta1;
    result.q(1) = q2_joint;  // Joint frame, NOT kinematic frame
    result.q(2) = theta3;
    result.q(3) = theta4;
    result.config = config;
    result.valid = true;
    
    // Verify with FK
    Matrix<3> fk = forwardKinematics(result.q);
    float dx = fk(0) - x;
    float dy = fk(1) - y;
    float dz = fk(2) - z;
    result.error = sqrtf(dx*dx + dy*dy + dz*dz);
    
    return result;
}

/**
 * Solve IK trying both configurations and multiple wrist angles
 * Now considers current joint position to avoid large J1 jumps
 */
inline IKResult solveIKBest(const Matrix<3>& target_pos, float wrist_angle, 
                            const Matrix<N_JOINTS>& current_q = {0, 0, 0, 0}) {
    // Try elbow-up with requested wrist angle first (preferred)
    IKResult up = solveIK(target_pos, wrist_angle, ElbowConfig::UP);
    if (up.valid && up.error < 5.0f) {
        // Check if J1 move is reasonable (not a huge jump)
        float j1_diff = fabsf(normalizeAngle(up.q(0) - current_q(0)));
        if (j1_diff < 2.5f) {  // Less than ~143 degrees is OK
            return up;
        }
    }
    
    // Try elbow-down with requested wrist angle
    IKResult down = solveIK(target_pos, wrist_angle, ElbowConfig::DOWN);
    if (down.valid && down.error < 5.0f) {
        float j1_diff = fabsf(normalizeAngle(down.q(0) - current_q(0)));
        if (j1_diff < 2.5f) {
            return down;
        }
    }
    
    // Try different wrist angles if default failed
    float wrist_angles[] = {0.0f, -0.5f, 0.5f, -1.0f, 1.0f, -M_PI_4, M_PI_4};
    IKResult best_result;
    best_result.valid = false;
    best_result.error = 999.0f;
    float best_j1_diff = 999.0f;
    
    for (float wa : wrist_angles) {
        IKResult r = solveIK(target_pos, wa, ElbowConfig::UP);
        if (r.valid && r.error < 5.0f) {
            float j1_diff = fabsf(normalizeAngle(r.q(0) - current_q(0)));
            if (j1_diff < best_j1_diff) {
                best_result = r;
                best_j1_diff = j1_diff;
            }
        }
        r = solveIK(target_pos, wa, ElbowConfig::DOWN);
        if (r.valid && r.error < 5.0f) {
            float j1_diff = fabsf(normalizeAngle(r.q(0) - current_q(0)));
            if (j1_diff < best_j1_diff) {
                best_result = r;
                best_j1_diff = j1_diff;
            }
        }
    }
    
    // If we found a good solution, use it
    if (best_result.valid && best_result.error < 5.0f) {
        return best_result;
    }
    
    // Return the better one from original attempts
    return (up.error < down.error) ? up : down;
}

/**
 * Compute target configuration for flipping from +X to -X
 * This ensures elbow-up is maintained during the flip
 */
inline IKResult computeFlipTarget(const Matrix<3>& target_pos, 
                                   float wrist_angle,
                                   const Matrix<N_JOINTS>& current_q) {
    // For flip to -X, we want elbow-up configuration
    IKResult result = solveIK(target_pos, wrist_angle, ElbowConfig::UP, current_q);
    
    if (!result.valid) {
        // If elbow-up doesn't work, try with a more vertical wrist
        result = solveIK(target_pos, 0.0f, ElbowConfig::UP, current_q);
    }
    
    return result;
}

#endif // ANALYTICAL_IK_H
