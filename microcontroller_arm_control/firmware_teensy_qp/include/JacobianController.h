/**
 * JacobianController.h
 * 
 * Jacobian-based Cartesian velocity control with posture maintenance.
 * Ported from the working old firmware with enhancements.
 * 
 * Key features:
 * - Damped Least Squares (DLS) Jacobian pseudoinverse
 * - Adaptive damping near singularities
 * - Null-space posture control (elbow-up, wrist horizontal, base centering)
 * - Acceleration limiting for smooth motion
 * - Low-pass velocity smoothing
 * 
 * This replaces the analytical IK → PD approach which caused jerky motion.
 */

#ifndef JACOBIAN_CONTROLLER_H
#define JACOBIAN_CONTROLLER_H

#include <BasicLinearAlgebra.h>
#include "ArmModel.h"

using namespace BLA;

// ============================================================================
// Jacobian Controller Configuration
// ============================================================================

struct JacobianConfig {
    // Damping parameters
    float lambda_base = 0.03f;       // Base damping (always applied)
    float lambda_max = 0.15f;        // Maximum damping near singularity
    float manipulability_threshold = 300.0f;  // Below this, increase damping
    
    // Cartesian control gains
    float cart_gain = 2.5f;          // Position error → velocity gain
    float max_cart_vel = 150.0f;     // Maximum Cartesian velocity (mm/s)
    
    // Posture control gains (scaled by distance to target)
    float elbow_correction_gain = 0.8f;   // Emergency elbow-up correction
    float wrist_horizontal_gain = 0.2f;   // Keep gripper level
    float base_centering_gain = 0.15f;    // Keep base at 0 for Y≈0 moves
    
    // Elbow angle thresholds (radians, negative = elbow up)
    float elbow_danger_threshold = -0.15f;   // Emergency zone
    float elbow_warning_threshold = -0.4f;   // Soft correction zone
    float elbow_target = -0.5f;              // Preferred elbow angle
    
    // Motion smoothing
    float max_acceleration = 2.0f;   // Max joint acceleration per loop (rad/s per step)
    float smoothing_alpha = 0.4f;    // Low-pass filter coefficient (0-1, higher = faster response)
    
    // Joint velocity limits
    float max_joint_vel = 1.5f;      // Maximum joint velocity (rad/s)
    float max_base_vel = 0.8f;       // Maximum base velocity (rad/s)
    
    // Approach behavior
    float stop_threshold = 2.0f;     // Stop when within this distance (mm)
    float decel_start = 50.0f;       // Start decelerating at this distance (mm)
    float min_approach_speed = 0.2f; // Minimum speed while approaching (fraction)
};

// ============================================================================
// Jacobian Controller Class
// ============================================================================

class JacobianController {
public:
    JacobianController();
    
    /**
     * Initialize with custom configuration
     */
    void init(const JacobianConfig& config);
    
    /**
     * Compute the Jacobian matrix for current joint configuration
     * J is 3x4 (Cartesian position derivatives w.r.t. joint angles)
     */
    Matrix<3, 4> computeJacobian(const Matrix<N_JOINTS>& q);
    
    /**
     * Compute manipulability index w = sqrt(det(J*J^T))
     * Higher values = better conditioning, lower = near singularity
     */
    float computeManipulability(const Matrix<3, 4>& J);
    
    /**
     * Compute adaptive damping based on manipulability
     */
    float computeAdaptiveDamping(const Matrix<3, 4>& J);
    
    /**
     * Compute Damped Least Squares pseudoinverse
     * J_pinv = J^T * (J*J^T + λ²*I)^-1
     */
    Matrix<4, 3> computeJacobianPinvDLS(const Matrix<3, 4>& J, float damping);
    
    /**
     * Main control function: compute joint velocities to track Cartesian target
     * Returns optimal joint velocity (rad/s) for each joint
     * 
     * @param current_q Current joint angles (4 joints)
     * @param current_pos Current Cartesian position from FK
     * @param target_pos Target Cartesian position
     * @param dt Time step
     * @param raw_base_pos Raw encoder value for base (for limit detection)
     * @return Joint velocities (rad/s)
     */
    Matrix<N_JOINTS> computeVelocity(
        const Matrix<N_JOINTS>& current_q,
        const Matrix<3>& current_pos,
        const Matrix<3>& target_pos,
        float dt,
        int raw_base_pos
    );
    
    /**
     * Apply posture corrections in null-space
     * Modifies q_dot in-place
     */
    void applyPostureControl(
        Matrix<N_JOINTS>& q_dot,
        const Matrix<N_JOINTS>& current_q,
        const Matrix<3>& target_pos,
        float error_norm
    );
    
    /**
     * Apply acceleration limiting and smoothing
     * Modifies q_dot in-place
     */
    void applySmoothing(Matrix<N_JOINTS>& q_dot);
    
    /**
     * Check if target is reached
     */
    bool isTargetReached(const Matrix<3>& current_pos, const Matrix<3>& target_pos);
    
    /**
     * Reset controller state (call when switching modes)
     */
    void reset();
    
    /**
     * Set wrist target angle (for horizontal gripper)
     */
    void setWristAngle(float angle) { m_wrist_target = angle; }
    
    /**
     * Get current configuration
     */
    const JacobianConfig& getConfig() const { return m_config; }
    
    /**
     * Update configuration
     */
    void setConfig(const JacobianConfig& config) { m_config = config; }
    
    // Debug info
    float getLastManipulability() const { return m_last_manipulability; }
    float getLastDamping() const { return m_last_damping; }
    bool isNearSingularity() const { return m_near_singularity; }

private:
    JacobianConfig m_config;
    
    // Previous velocities for smoothing
    Matrix<N_JOINTS> m_prev_q_dot;
    
    // Wrist target angle
    float m_wrist_target;
    
    // Debug state
    float m_last_manipulability;
    float m_last_damping;
    bool m_near_singularity;
    
    // Clamp base velocity based on limits
    float clampBaseVelocity(float q1_rad, float q1_dot, int raw_pos);
};

// ============================================================================
// Base Joint Limits (from old firmware)
// The base joint is NOT continuous - wires prevent full rotation
// ============================================================================

// Raw encoder limits
constexpr int BASE_MIN_RAW = 100;   // ~-156 degrees
constexpr int BASE_MAX_RAW = 3995;  // ~+156 degrees

// Corresponding radian limits
constexpr float BASE_MIN_RAD = -2.72f;  // About -156 degrees
constexpr float BASE_MAX_RAD = 2.72f;   // About +156 degrees

#endif // JACOBIAN_CONTROLLER_H
