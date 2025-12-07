/**
 * JacobianController.cpp
 * 
 * Implementation of Jacobian-based Cartesian velocity control.
 * Ported from the working old firmware with enhancements.
 */

#include "JacobianController.h"
#include <Arduino.h>

// ============================================================================
// Constructor
// ============================================================================

JacobianController::JacobianController() 
    : m_wrist_target(0.0f)
    , m_last_manipulability(0.0f)
    , m_last_damping(0.0f)
    , m_near_singularity(false)
{
    // Initialize previous velocities to zero
    for (int i = 0; i < N_JOINTS; i++) {
        m_prev_q_dot(i) = 0.0f;
    }
}

void JacobianController::init(const JacobianConfig& config) {
    m_config = config;
    reset();
}

void JacobianController::reset() {
    for (int i = 0; i < N_JOINTS; i++) {
        m_prev_q_dot(i) = 0.0f;
    }
    m_near_singularity = false;
}

// ============================================================================
// Jacobian Computation
// ============================================================================

Matrix<3, 4> JacobianController::computeJacobian(const Matrix<N_JOINTS>& q) {
    // Apply the 90° offset to q2 (shoulder)
    // When q2=0 (motor at center), arm points UP in our convention
    float q1 = q(0);
    float q2 = q(1) + M_PI_2;  // Offset so motor center = vertical
    float q3 = q(2);
    float q4 = q(3);
    
    float s1 = sinf(q1), c1 = cosf(q1);
    float s2 = sinf(q2), c2 = cosf(q2);
    float s23 = sinf(q2 + q3), c23 = cosf(q2 + q3);
    float s234 = sinf(q2 + q3 + q4), c234 = cosf(q2 + q3 + q4);
    
    // Radial distance from base axis
    float R = L1 * c2 + L2 * c23 + L3 * c234;
    
    // Partial derivatives of R w.r.t. joint angles
    float dR_dq2 = -L1 * s2 - L2 * s23 - L3 * s234;
    float dR_dq3 = -L2 * s23 - L3 * s234;
    float dR_dq4 = -L3 * s234;
    
    // Partial derivatives of Z w.r.t. joint angles
    float dz_dq2 = L1 * c2 + L2 * c23 + L3 * c234;
    float dz_dq3 = L2 * c23 + L3 * c234;
    float dz_dq4 = L3 * c234;
    
    Matrix<3, 4> J;
    
    // Row 0: dx/dq
    J(0, 0) = -R * s1;         // dx/dq1
    J(0, 1) = dR_dq2 * c1;     // dx/dq2
    J(0, 2) = dR_dq3 * c1;     // dx/dq3
    J(0, 3) = dR_dq4 * c1;     // dx/dq4
    
    // Row 1: dy/dq
    J(1, 0) = R * c1;          // dy/dq1
    J(1, 1) = dR_dq2 * s1;     // dy/dq2
    J(1, 2) = dR_dq3 * s1;     // dy/dq3
    J(1, 3) = dR_dq4 * s1;     // dy/dq4
    
    // Row 2: dz/dq
    J(2, 0) = 0.0f;            // dz/dq1 (base doesn't affect Z)
    J(2, 1) = dz_dq2;          // dz/dq2
    J(2, 2) = dz_dq3;          // dz/dq3
    J(2, 3) = dz_dq4;          // dz/dq4
    
    return J;
}

float JacobianController::computeManipulability(const Matrix<3, 4>& J) {
    // w = sqrt(det(J * J^T))
    Matrix<4, 3> JT = ~J;  // Transpose
    Matrix<3, 3> JJT = J * JT;
    
    // Calculate determinant of 3x3 matrix
    float det = JJT(0,0) * (JJT(1,1)*JJT(2,2) - JJT(1,2)*JJT(2,1))
              - JJT(0,1) * (JJT(1,0)*JJT(2,2) - JJT(1,2)*JJT(2,0))
              + JJT(0,2) * (JJT(1,0)*JJT(2,1) - JJT(1,1)*JJT(2,0));
    
    if (det < 0.0f) det = 0.0f;  // Numerical stability
    
    m_last_manipulability = sqrtf(det);
    return m_last_manipulability;
}

float JacobianController::computeAdaptiveDamping(const Matrix<3, 4>& J) {
    float w = computeManipulability(J);
    float epsilon = m_config.manipulability_threshold;
    
    if (w < epsilon) {
        // Near singularity - increase damping
        float ratio = 1.0f - w / epsilon;
        m_last_damping = m_config.lambda_base + m_config.lambda_max * ratio * ratio;
        m_near_singularity = true;
    } else {
        m_last_damping = m_config.lambda_base;
        m_near_singularity = false;
    }
    
    return m_last_damping;
}

Matrix<4, 3> JacobianController::computeJacobianPinvDLS(const Matrix<3, 4>& J, float damping) {
    // DLS pseudoinverse: J_pinv = J^T * (J*J^T + λ²*I)^-1
    Matrix<4, 3> JT = ~J;  // Transpose
    Matrix<3, 3> JJT = J * JT;
    
    // Add damping to diagonal
    float lambda_sq = damping * damping;
    JJT(0, 0) += lambda_sq;
    JJT(1, 1) += lambda_sq;
    JJT(2, 2) += lambda_sq;
    
    // Invert the 3x3 matrix using BLA's Inverse function
    Matrix<3, 3> JJT_inv = Inverse(JJT);
    
    return JT * JJT_inv;
}

// ============================================================================
// Main Control Function
// ============================================================================

Matrix<N_JOINTS> JacobianController::computeVelocity(
    const Matrix<N_JOINTS>& current_q,
    const Matrix<3>& current_pos,
    const Matrix<3>& target_pos,
    float dt,
    int raw_base_pos)
{
    // Position error
    Matrix<3> pos_error;
    pos_error(0) = target_pos(0) - current_pos(0);
    pos_error(1) = target_pos(1) - current_pos(1);
    pos_error(2) = target_pos(2) - current_pos(2);
    
    float error_norm = sqrtf(pos_error(0)*pos_error(0) + 
                             pos_error(1)*pos_error(1) + 
                             pos_error(2)*pos_error(2));
    
    // Compute Jacobian and adaptive damping
    Matrix<3, 4> J = computeJacobian(current_q);
    float damping = computeAdaptiveDamping(J);
    Matrix<4, 3> J_pinv = computeJacobianPinvDLS(J, damping);
    
    // Compute approach scale (decelerate as we get close)
    float approach_scale = 1.0f;
    if (error_norm < m_config.stop_threshold) {
        approach_scale = 0.0f;  // Stop when very close
    } else if (error_norm < m_config.decel_start) {
        // Linear ramp from min_approach_speed to 1.0
        float t = (error_norm - m_config.stop_threshold) / 
                  (m_config.decel_start - m_config.stop_threshold);
        approach_scale = m_config.min_approach_speed + 
                         t * (1.0f - m_config.min_approach_speed);
    }
    
    // Cartesian velocity from position error
    Matrix<3> x_dot;
    for (int i = 0; i < 3; i++) {
        x_dot(i) = pos_error(i) * m_config.cart_gain * approach_scale;
    }
    
    // Limit Cartesian velocity
    float x_dot_norm = sqrtf(x_dot(0)*x_dot(0) + x_dot(1)*x_dot(1) + x_dot(2)*x_dot(2));
    if (x_dot_norm > m_config.max_cart_vel) {
        float scale = m_config.max_cart_vel / x_dot_norm;
        for (int i = 0; i < 3; i++) {
            x_dot(i) *= scale;
        }
    }
    
    // Joint velocities from Jacobian pseudoinverse
    Matrix<N_JOINTS> q_dot = J_pinv * x_dot;
    
    // Apply posture control (elbow-up, wrist horizontal, etc.)
    applyPostureControl(q_dot, current_q, target_pos, error_norm);
    
    // Clamp base velocity at mechanical limits
    q_dot(0) = clampBaseVelocity(current_q(0), q_dot(0), raw_base_pos);
    
    // Clamp joint velocities
    for (int i = 0; i < N_JOINTS; i++) {
        float limit = (i == 0) ? m_config.max_base_vel : m_config.max_joint_vel;
        if (q_dot(i) > limit) q_dot(i) = limit;
        if (q_dot(i) < -limit) q_dot(i) = -limit;
    }
    
    // Apply acceleration limiting and smoothing
    applySmoothing(q_dot);
    
    return q_dot;
}

// ============================================================================
// Posture Control
// ============================================================================

void JacobianController::applyPostureControl(
    Matrix<N_JOINTS>& q_dot,
    const Matrix<N_JOINTS>& current_q,
    const Matrix<3>& target_pos,
    float error_norm)
{
    // Scale posture corrections based on distance to target
    // Strong when far, weak when close (to avoid fighting position control)
    float posture_scale = fminf(1.0f, error_norm / 50.0f);
    if (posture_scale < 0.1f) posture_scale = 0.1f;
    
    // =========================================================================
    // ELBOW CONTROL - Keep elbow UP (q3 negative)
    // =========================================================================
    float elbow_angle = current_q(2);
    float elbow_correction = 0.0f;
    
    if (elbow_angle > m_config.elbow_danger_threshold) {
        // DANGER: Elbow nearly straight or flipped!
        float urgency = 1.0f + fmaxf(0.0f, elbow_angle) * 2.0f;
        elbow_correction = -m_config.elbow_correction_gain * urgency;
        
        // Reduce other velocities to prioritize elbow fix
        for (int i = 0; i < N_JOINTS; i++) {
            if (i != 2) q_dot(i) *= 0.5f;
        }
    } else if (elbow_angle > m_config.elbow_warning_threshold) {
        // WARNING: Getting close to straight
        float urgency = (elbow_angle - m_config.elbow_warning_threshold) / 
                        (m_config.elbow_danger_threshold - m_config.elbow_warning_threshold);
        elbow_correction = -0.3f * urgency * posture_scale;
    }
    // No correction if elbow is safely bent
    
    q_dot(2) += elbow_correction;
    
    // =========================================================================
    // WRIST HORIZONTAL CONTROL - Keep gripper level
    // =========================================================================
    // phi_current = q2_raw + q3 + q4 (gripper world angle)
    float q2_raw = current_q(1) + M_PI_2;
    float phi_current = q2_raw + current_q(2) + current_q(3);
    float phi_target = m_wrist_target;  // Usually 0 for horizontal
    
    // For moves to -X with Y≈0, gripper should point backward
    bool y_near_zero = fabsf(target_pos(1)) < 30.0f;
    if (target_pos(0) < -20.0f && y_near_zero) {
        phi_target = M_PI;  // Point backward
    }
    
    // Wrap angle error
    float phi_error = phi_target - phi_current;
    while (phi_error > M_PI) phi_error -= 2.0f * M_PI;
    while (phi_error < -M_PI) phi_error += 2.0f * M_PI;
    
    // Gentle wrist correction
    q_dot(3) += phi_error * m_config.wrist_horizontal_gain * posture_scale;
    
    // =========================================================================
    // BASE CENTERING for Y≈0 moves
    // =========================================================================
    // When Y is near zero, keep base at 0 to allow smooth crossing
    if (y_near_zero) {
        float base_error = 0.0f - current_q(0);
        q_dot(0) += base_error * m_config.base_centering_gain * posture_scale;
    }
    
    // =========================================================================
    // FLIP GUIDANCE for Y≈0 crossing
    // =========================================================================
    // When moving from +X to -X (or vice versa) with Y≈0, guide through top
    if (y_near_zero) {
        float current_x = forwardKinematics(current_q)(0);
        
        bool going_to_negative_x = target_pos(0) < -20.0f;
        bool currently_positive_x = current_x > 20.0f;
        
        if (going_to_negative_x && currently_positive_x) {
            // Need to flip over the top
            float flip_urgency = fminf(1.0f, fabsf(current_x) / 80.0f);
            
            // Strong elbow-up bias for proper flip
            float elbow_target_flip = -1.2f;
            float elbow_error = elbow_target_flip - current_q(2);
            q_dot(2) += elbow_error * 1.0f * flip_urgency;
        }
        
        bool going_to_positive_x = target_pos(0) > 20.0f;
        bool currently_negative_x = current_x < -20.0f;
        
        if (going_to_positive_x && currently_negative_x) {
            // Flip from -X to +X
            float flip_urgency = fminf(1.0f, fabsf(current_x) / 80.0f);
            float elbow_target_flip = -1.2f;
            float elbow_error = elbow_target_flip - current_q(2);
            q_dot(2) += elbow_error * 1.0f * flip_urgency;
        }
    }
}

// ============================================================================
// Smoothing
// ============================================================================

void JacobianController::applySmoothing(Matrix<N_JOINTS>& q_dot) {
    // Acceleration limiting
    for (int i = 0; i < N_JOINTS; i++) {
        float delta = q_dot(i) - m_prev_q_dot(i);
        if (delta > m_config.max_acceleration) delta = m_config.max_acceleration;
        if (delta < -m_config.max_acceleration) delta = -m_config.max_acceleration;
        q_dot(i) = m_prev_q_dot(i) + delta;
    }
    
    // Low-pass smoothing
    float alpha = m_config.smoothing_alpha;
    for (int i = 0; i < N_JOINTS; i++) {
        q_dot(i) = alpha * q_dot(i) + (1.0f - alpha) * m_prev_q_dot(i);
    }
    
    // Store for next iteration
    m_prev_q_dot = q_dot;
}

// ============================================================================
// Target Detection
// ============================================================================

bool JacobianController::isTargetReached(const Matrix<3>& current_pos, const Matrix<3>& target_pos) {
    float dx = target_pos(0) - current_pos(0);
    float dy = target_pos(1) - current_pos(1);
    float dz = target_pos(2) - current_pos(2);
    float dist = sqrtf(dx*dx + dy*dy + dz*dz);
    
    return dist < m_config.stop_threshold;
}

// ============================================================================
// Base Velocity Clamping
// ============================================================================

float JacobianController::clampBaseVelocity(float q1_rad, float q1_dot, int raw_pos) {
    const float MARGIN_RAD = 0.1f;  // ~6 degrees safety margin
    
    // Check if moving toward a limit
    if (q1_dot > 0 && (q1_rad > BASE_MAX_RAD - MARGIN_RAD || raw_pos > BASE_MAX_RAW - 50)) {
        return 0.0f;  // Stop at max limit
    }
    if (q1_dot < 0 && (q1_rad < BASE_MIN_RAD + MARGIN_RAD || raw_pos < BASE_MIN_RAW + 50)) {
        return 0.0f;  // Stop at min limit
    }
    
    return q1_dot;
}
