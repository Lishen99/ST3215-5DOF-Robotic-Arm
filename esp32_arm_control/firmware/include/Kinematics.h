#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <BasicLinearAlgebra.h>
#include <math.h>

using namespace BLA;

// 4-DOF Arm Kinematics (Base, Shoulder, Elbow, Wrist Pitch)
// Motor 6 is Wrist Roll - controlled independently

// Link Lengths (mm) - Matching Python kinematics.py
const float L1 = 133.39f; // Shoulder to Elbow
const float L2 = 124.97f; // Elbow to Wrist 
const float L3 = 73.39f;  // Wrist to Tip

// Servo Limits from servo_limits.json
// Motor 1 (Base): min=0, max=4095, center=2207 (calibrated)
// Motor 2 (Shoulder): min=1365, max=3869, center=2617
// Motor 3 (Shoulder coupled): min=1521, max=4021, center=2771
// Motor 4 (Elbow): min=1324, max=3803, center=2563
// Motor 5 (Wrist Pitch): min=854, max=3466, center=2160
// Motor 6 (Wrist Roll): min=0, max=4095, center=2047

// Centers calculated as (min + max) / 2, Motor 1 manually calibrated
const int MOTOR_CENTERS[6] = {2207, 2617, 2771, 2563, 2160, 2047};

// Joint limits in radians (calculated from servo limits)
// Ticks to radians: (pos - center) * 2*PI / 4096
// For 270 degree servos: (pos - center) * 270 * PI / 180 / (max - min)
struct JointLimits {
    float min_rad;
    float max_rad;
};

// Limits for 4 IK joints (Base, Shoulder, Elbow, WristPitch)
// Calculated from servo_limits.json: (pos - center) * 2*PI / 4096
// These are the TESTED PHYSICAL LIMITS - do not exceed!
// Motor 1 (Base): Safe range 100-3995 raw to avoid wrap-around
// Motor 2 (Shoulder): min=1365, max=3869, center=2617 → ±1252 steps → ±1.92 rad
// Motor 4 (Elbow): min=1324, max=3803, center=2563 → ±1240 steps → ±1.90 rad
// Motor 5 (WristP): min=854, max=3466, center=2160 → ±1306 steps → ±2.00 rad
const JointLimits JOINT_LIMITS[4] = {
    {-3.20f, 2.70f},          // Base: ~-183° to +155° (cannot wrap!)
    {-1.92f, 1.92f},          // Shoulder: ±110° (from servo_limits.json)
    {-1.90f, 1.90f},          // Elbow: ±109° (from servo_limits.json)
    {-2.00f, 2.00f}           // Wrist Pitch: ±115° (from servo_limits.json)
};

class Kinematics {
public:
    Kinematics();

    // Forward Kinematics: q[4] -> [x, y, z] position
    Matrix<3> forward_kinematics(const Matrix<4>& q);
    
    // Get all joint positions for visualization (p0, p1, p2, p3, p4)
    void get_joint_positions(const Matrix<4>& q, Matrix<3>* positions);
    
    // Calculate 3x4 Jacobian for position control
    Matrix<3, 4> calculate_jacobian(const Matrix<4>& q);
    
    // Calculate manipulability index: sqrt(det(J*J^T))
    float calculate_manipulability(const Matrix<3, 4>& J);
    
    // Damped Least Squares pseudoinverse
    Matrix<4, 3> get_jacobian_pinv_dls(const Matrix<3, 4>& J, float damping);
    
    // Adaptive damping based on manipulability
    float calculate_adaptive_damping(const Matrix<3, 4>& J, float lambda_max = 0.01f, float epsilon = 0.05f);
    
    // Clamp joints to limits
    Matrix<4> clamp_joints(const Matrix<4>& q);
    
    // Clamp joint velocities based on proximity to limits
    Matrix<4> clamp_velocities(const Matrix<4>& q, const Matrix<4>& q_dot, float buffer_rad = 0.05f);
    
    // Analytical IK with elbow up/down solutions
    bool inverse_kinematics_analytical(const Matrix<3>& target_pos, const Matrix<4>& current_q,
                                        float wrist_angle_rad, Matrix<4>& result);
    
    // Check if a target position is reachable
    // Returns true if position is within workspace, false if unreachable
    // reason will contain a description of why position is unreachable
    bool is_position_reachable(float x, float y, float z, char* reason, size_t reason_len);

private:
    float wrap_angle(float angle);
};

#endif
