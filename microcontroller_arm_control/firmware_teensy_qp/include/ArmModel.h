#ifndef ARM_MODEL_H
#define ARM_MODEL_H

#include <BasicLinearAlgebra.h>
#include <math.h>

using namespace BLA;

/**
 * ARM MODEL - Abstract arm kinematics for research extensibility
 * 
 * This defines the physical properties and kinematic functions
 * that can be easily swapped for different arm configurations.
 */

// ============================================================================
// Physical Constants
// ============================================================================

// Link Lengths (mm) - Your 5DOF arm
constexpr float L1 = 133.39f;  // Shoulder to Elbow
constexpr float L2 = 124.97f;  // Elbow to Wrist 
constexpr float L3 = 73.39f;   // Wrist to End-effector

// Total arm reach
constexpr float ARM_REACH_MAX = L1 + L2 + L3;  // ~331.75mm
constexpr float ARM_REACH_MIN = 50.0f;          // Minimum safe radius

// Number of controllable joints (excluding wrist roll for IK)
constexpr int N_JOINTS = 4;  // Base, Shoulder, Elbow, WristPitch
constexpr int N_MOTORS = 6;  // Including coupled shoulder and wrist roll

// ============================================================================
// Joint Limits
// ============================================================================

struct JointLimit {
    float min_rad;
    float max_rad;
    float max_vel;      // rad/s
    float max_accel;    // rad/s²
};

// Tested physical limits - expanded for full workspace
constexpr JointLimit JOINT_LIMITS[N_JOINTS] = {
    {-3.05f, 3.05f, 6.0f, 20.0f},   // J1 (Base): ±175° - CANNOT cross 0↔4096 due to wiring, leave margin
    {-1.92f, 1.92f, 4.0f, 15.0f},   // J2 (Shoulder): ±110°
    {-1.90f, 1.90f, 5.0f, 18.0f},   // J3 (Elbow): ±109°
    {-2.00f, 2.00f, 6.0f, 20.0f}    // J4 (Wrist Pitch): ±115°
};

// Motor mappings (servo ID to joint)
constexpr uint8_t MOTOR_IDS[N_MOTORS] = {1, 2, 3, 4, 5, 6};

// Motor centers (from calibration)
constexpr int MOTOR_CENTERS[N_MOTORS] = {2207, 2617, 2771, 2563, 2160, 2047};

// ============================================================================
// Workspace Constraints
// ============================================================================

struct WorkspaceConstraint {
    float z_min;        // Minimum Z (table level)
    float z_max;        // Maximum Z
    float radius_min;   // Minimum XY radius (not enforced, just info)
    float radius_max;   // Maximum XY radius
};

constexpr WorkspaceConstraint WORKSPACE = {
    .z_min = -150.0f,     // Allow below-table (expanded from -100)
    .z_max = 400.0f,      // Full extension up
    .radius_min = 0.0f,   // Allow center (arm pointing up)
    .radius_max = 335.0f  // Full extension out (L1+L2+L3 = 331.75)
};

// ============================================================================
// Elbow Configuration
// ============================================================================

enum class ElbowConfig {
    UP,     // Elbow above the line from shoulder to wrist (preferred)
    DOWN,   // Elbow below
    AUTO    // Choose based on current configuration
};

// ============================================================================
// Arm State
// ============================================================================

struct ArmState {
    Matrix<N_JOINTS> q;         // Joint positions (rad)
    Matrix<N_JOINTS> q_dot;     // Joint velocities (rad/s)
    Matrix<3> pos;              // End-effector position (mm)
    Matrix<3> vel;              // End-effector velocity (mm/s)
    float roll;                 // Wrist roll (rad)
    float roll_dot;             // Wrist roll velocity (rad/s)
    bool valid;                 // True if state is within limits
};

// ============================================================================
// Kinematic Functions
// ============================================================================

/**
 * Forward Kinematics
 * @param q Joint angles [q1, q2, q3, q4]
 * @return End-effector position [x, y, z]
 * 
 * Matches Python implementation exactly:
 *   q2_kin = q2 + 90° (when arm is horizontal at home, q2=0, q2_kin=90°)
 *   r = L1*cos(q2_kin) + L2*cos(q2_kin+q3) + L3*cos(q2_kin+q3+q4)
 *   z = L1*sin(q2_kin) + L2*sin(q2_kin+q3) + L3*sin(q2_kin+q3+q4)
 *   x = r * cos(t1), y = r * sin(t1)
 */
inline Matrix<3> forwardKinematics(const Matrix<N_JOINTS>& q) {
    float q1 = q(0);
    float q2 = q(1) + M_PI_2;  // Add 90° offset - matches Python/GUI convention
    float q3 = q(2);
    float q4 = q(3);
    
    float c1 = cosf(q1), s1 = sinf(q1);
    float c2 = cosf(q2), s2 = sinf(q2);
    float c23 = cosf(q2 + q3), s23 = sinf(q2 + q3);
    float c234 = cosf(q2 + q3 + q4), s234 = sinf(q2 + q3 + q4);
    
    float r = L1 * c2 + L2 * c23 + L3 * c234;
    float z = L1 * s2 + L2 * s23 + L3 * s234;
    
    return {r * c1, r * s1, z};
}

/**
 * Calculate Jacobian (3x4)
 * Maps joint velocities to end-effector velocity
 * Must use same convention as FK - with 90° offset on q2
 */
inline Matrix<3, N_JOINTS> calculateJacobian(const Matrix<N_JOINTS>& q) {
    float q1 = q(0);
    float q2 = q(1) + M_PI_2;  // Same 90° offset as FK
    float q3 = q(2);
    float q4 = q(3);
    
    float c1 = cosf(q1), s1 = sinf(q1);
    float c2 = cosf(q2), s2 = sinf(q2);
    float c23 = cosf(q2 + q3), s23 = sinf(q2 + q3);
    float c234 = cosf(q2 + q3 + q4), s234 = sinf(q2 + q3 + q4);
    
    float r = L1 * c2 + L2 * c23 + L3 * c234;
    
    Matrix<3, N_JOINTS> J;
    
    // dx/dq
    J(0, 0) = -r * s1;                                      // dx/dq1
    J(0, 1) = (-L1 * s2 - L2 * s23 - L3 * s234) * c1;      // dx/dq2
    J(0, 2) = (-L2 * s23 - L3 * s234) * c1;                // dx/dq3
    J(0, 3) = (-L3 * s234) * c1;                           // dx/dq4
    
    // dy/dq
    J(1, 0) = r * c1;                                       // dy/dq1
    J(1, 1) = (-L1 * s2 - L2 * s23 - L3 * s234) * s1;      // dy/dq2
    J(1, 2) = (-L2 * s23 - L3 * s234) * s1;                // dy/dq3
    J(1, 3) = (-L3 * s234) * s1;                           // dy/dq4
    
    // dz/dq
    J(2, 0) = 0;                                            // dz/dq1
    J(2, 1) = L1 * c2 + L2 * c23 + L3 * c234;              // dz/dq2
    J(2, 2) = L2 * c23 + L3 * c234;                        // dz/dq3
    J(2, 3) = L3 * c234;                                   // dz/dq4
    
    return J;
}

/**
 * Check if joint configuration is within limits
 */
inline bool isWithinLimits(const Matrix<N_JOINTS>& q) {
    for (int i = 0; i < N_JOINTS; i++) {
        if (q(i) < JOINT_LIMITS[i].min_rad || q(i) > JOINT_LIMITS[i].max_rad) {
            return false;
        }
    }
    return true;
}

/**
 * Clamp joint angles to limits
 */
inline Matrix<N_JOINTS> clampToLimits(const Matrix<N_JOINTS>& q) {
    Matrix<N_JOINTS> clamped = q;
    for (int i = 0; i < N_JOINTS; i++) {
        if (clamped(i) < JOINT_LIMITS[i].min_rad) clamped(i) = JOINT_LIMITS[i].min_rad;
        if (clamped(i) > JOINT_LIMITS[i].max_rad) clamped(i) = JOINT_LIMITS[i].max_rad;
    }
    return clamped;
}

/**
 * Check if position is in workspace
 * Uses total distance from origin, not just XY radius
 */
inline bool isInWorkspace(float x, float y, float z) {
    // Check Z limits
    if (z < WORKSPACE.z_min || z > WORKSPACE.z_max) {
        return false;
    }
    
    // Check total reach (spherical approximation)
    float total_dist = sqrtf(x*x + y*y + z*z);
    if (total_dist > ARM_REACH_MAX + 10.0f) {  // Small margin
        return false;
    }
    
    // Check XY radius if far from vertical
    float r = sqrtf(x*x + y*y);
    if (r > WORKSPACE.radius_max) {
        return false;
    }
    
    return true;
}

/**
 * Determine elbow configuration from current state
 * Returns UP if J3 is negative (elbow bent upward in the inverted-V shape)
 */
inline ElbowConfig getElbowConfig(const Matrix<N_JOINTS>& q) {
    return (q(2) < 0) ? ElbowConfig::UP : ElbowConfig::DOWN;
}

/**
 * Convert raw servo position to radians
 */
inline float rawToRad(int raw, int center) {
    return (raw - center) * (2.0f * M_PI / 4096.0f);
}

/**
 * Convert radians to raw servo position
 */
inline int radToRaw(float rad, int center) {
    return center + (int)(rad * 4096.0f / (2.0f * M_PI));
}

#endif // ARM_MODEL_H
