/**
 * Hybrid Jacobian+MPC 5DOF Arm Controller - TEENSY 4.1 VERSION
 * 
 * Ported from ESP32 firmware using TeensyThreads for multi-threading.
 * 
 * REVISED ARCHITECTURE (v3):
 * - Jacobian-based Cartesian velocity control (smooth motion)
 * - Posture control in null-space (elbow-up, wrist horizontal)
 * - MPC constraint projection (safe velocities at limits)
 * - High-speed loop (500Hz+ like old firmware)
 * 
 * This combines the best of both approaches:
 * - Old firmware: Jacobian + posture control for smooth sweeps
 * - MPC: Constraint projection for safety
 * 
 * Compatible with existing arm_gui.py
 */

#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <ArduinoJson.h>
#include <vector>
#include <TeensyThreads.h>

#include "ArmModel.h"
#include "TinyMPC.h"
#include "AnalyticalIK.h"
#include "JacobianController.h"
#include "ServoDriver.h"

using namespace BLA;

// ============================================================================
// Pin Definitions - TEENSY 4.1
// ============================================================================
// Using Serial1: TX = Pin 1, RX = Pin 0
// Connect: Waveshare board TX -> Teensy Pin 0 (RX1)
//          Waveshare board RX -> Teensy Pin 1 (TX1)
//          GND -> GND

// ============================================================================
// Objects
// ============================================================================
ServoDriver servoDriver(Serial1);
TinyMPC mpc;
JacobianController jacobian;

// ============================================================================
// Velocity Scaling for Servo Speed
// ============================================================================
// ST3215: 4096 steps/revolution, velocity in steps/s
// 1 rad/s = 4096 / (2*PI) = 652 steps/s
const float RAD_TO_STEPS = 4096.0f / (2.0f * M_PI);

// Minimum motor speed - motors jitter below this
const int MIN_MOTOR_SPEED = 50;  // Normal operation minimum
const int FINAL_APPROACH_SPEED = 30;  // Allow slower during final approach (< 5mm from target)

// ============================================================================
// State Variables - Protected by mutex
// ============================================================================
Threads::Mutex stateMutex;

// Current arm state
Matrix<N_JOINTS> current_q = {0, 0, 0, 0};
Matrix<N_JOINTS> current_q_dot = {0, 0, 0, 0};
Matrix<N_JOINTS> prev_q = {0, 0, 0, 0};
float current_roll = 0;
float prev_roll = 0;

// Target state
Matrix<3> cartesian_target = {200, 0, 200};
Matrix<N_JOINTS> target_q = {0, 0, 0, 0};
float target_roll = 0;
float target_wrist_angle = 0.0f;

// Mode flags (volatile for thread safety)
volatile bool torque_enabled = true;
volatile bool cartesian_mode = false;
volatile bool direct_mode = false;
volatile bool sweep_mode = false;
volatile bool moving = false;

// Speed and wrist settings
float speed_multiplier = 2.0f;
bool wrist_locked = true;

// Raw positions for telemetry
int rawPositions[N_MOTORS] = {2207, 2617, 2771, 2563, 2160, 2047};
int lastPositions[N_MOTORS] = {2207, 2617, 2771, 2563, 2160, 2047};

// Read failure tracking
int readFailCount = 0;
const int MAX_READ_FAILS = 10;

// Motor IDs
std::vector<uint8_t> motor_ids = {1, 2, 3, 4, 5, 6};

// ============================================================================
// Sweep Mode Variables
// ============================================================================
int sweep_axis = 0;
float sweep_velocity = 50.0f;
int sweep_direction = 1;
Matrix<3> sweep_start_pos = {0, 0, 0};
bool sweep_initialized = false;

// ============================================================================
// Path Mode Variables
// ============================================================================
struct Waypoint {
    Matrix<3> pos;
    float roll;
    float wait_time;
};

const int MAX_WAYPOINTS = 200;
Waypoint waypoints[MAX_WAYPOINTS];
int waypoint_head = 0;  // Index of first waypoint
int waypoint_tail = 0;  // Index where next waypoint will be added
int waypoint_count = 0; // Number of waypoints in buffer
int current_waypoint_offset = 0;  // Offset from head to current waypoint
bool path_mode = false;
int path_type = 0;  // 0=linear, 1=smooth
float path_blend_radius = 40.0f;  // Increased for smoother motion with filtered waypoints
bool path_waiting = false;
unsigned long path_wait_start = 0;

// ============================================================================
// Backlash Compensation (from old firmware)
// ============================================================================
int prev_direction[6] = {0, 0, 0, 0, 0, 0};
float backlash_offset[6] = {0, 0, 0, 0, 0, 0};

void applyBacklashCompensation(std::vector<int>& speeds) {
    const int BACKLASH_STEPS = 7;  // ~0.6 degrees
    const int SPEED_THRESHOLD = 10;
    
    for (size_t i = 0; i < speeds.size() && i < 6; i++) {
        int new_direction = 0;
        if (speeds[i] > SPEED_THRESHOLD) new_direction = 1;
        else if (speeds[i] < -SPEED_THRESHOLD) new_direction = -1;
        
        if (new_direction != 0 && prev_direction[i] != 0 && new_direction != prev_direction[i]) {
            backlash_offset[i] = BACKLASH_STEPS * new_direction;
        }
        
        if (backlash_offset[i] != 0.0f && new_direction != 0) {
            float comp_speed = backlash_offset[i] * 50.0f;
            speeds[i] += (int)comp_speed;
            
            float decay = fabs(speeds[i]) * 0.001f;
            if (decay < 0.5f) decay = 0.5f;
            if (backlash_offset[i] > 0) {
                backlash_offset[i] -= decay;
                if (backlash_offset[i] < 0) backlash_offset[i] = 0;
            } else {
                backlash_offset[i] += decay;
                if (backlash_offset[i] > 0) backlash_offset[i] = 0;
            }
        }
        
        if (new_direction != 0) {
            prev_direction[i] = new_direction;
        }
    }
}

// ============================================================================
// Statistics
// ============================================================================
unsigned long last_control_time = 0;
float mpc_solve_time = 0;

// ============================================================================
// Helper Functions
// ============================================================================

bool readAllPositions(std::vector<int>& positions) {
    bool success = servoDriver.syncReadPosition(motor_ids, positions);
    
    if (success) {
        bool all_valid = true;
        for (size_t i = 0; i < positions.size(); i++) {
            if (positions[i] < 0 || positions[i] > 4095) {
                positions[i] = lastPositions[i];
                all_valid = false;
            } else {
                lastPositions[i] = positions[i];
            }
        }
        readFailCount = 0;
        return all_valid;
    } else {
        readFailCount++;
        for (size_t i = 0; i < positions.size(); i++) {
            positions[i] = lastPositions[i];
        }
        return false;
    }
}

void updateArmState(const std::vector<int>& positions, float dt) {
    for (int i = 0; i < N_MOTORS; i++) {
        rawPositions[i] = positions[i];
    }
    
    prev_q = current_q;
    prev_roll = current_roll;
    
    current_q(0) = rawToRad(positions[0], MOTOR_CENTERS[0]);  // Base
    // Shoulder (motor 2) - NEGATE to match convention
    current_q(1) = -rawToRad(positions[1], MOTOR_CENTERS[1]);
    current_q(2) = rawToRad(positions[3], MOTOR_CENTERS[3]);  // Elbow (motor 4)
    current_q(3) = rawToRad(positions[4], MOTOR_CENTERS[4]);  // Wrist pitch (motor 5)
    current_roll = rawToRad(positions[5], MOTOR_CENTERS[5]);  // Wrist roll (motor 6)
    
    // Estimate velocities (low-pass filtered)
    if (dt > 0.001f) {
        float alpha = 0.3f;
        for (int i = 0; i < N_JOINTS; i++) {
            float raw_vel = (current_q(i) - prev_q(i)) / dt;
            current_q_dot(i) = alpha * raw_vel + (1.0f - alpha) * current_q_dot(i);
        }
    }
}

void stopAllMotors() {
    std::vector<int> speeds(N_MOTORS, 0);
    servoDriver.syncWriteVelocity(motor_ids, speeds);
}

/**
 * Send velocity commands to motors
 * Takes joint velocities (rad/s) and converts to motor speeds
 * distance_to_target: distance to target in mm (0 = unknown, use normal boost)
 */
void sendVelocityCommands(const Matrix<N_JOINTS>& q_dot, float roll_vel, bool boost_low_speeds = true, float distance_to_target = 0.0f) {
    std::vector<int> speeds(N_MOTORS, 0);
    
    // J1 (Base) -> Motor 1
    speeds[0] = (int)(q_dot(0) * RAD_TO_STEPS);
    
    // J2 (Shoulder) -> Motor 2 & 3 (coupled, opposite)
    // NEGATE because joint angle is negated from motor position
    int speed_shoulder = (int)(-q_dot(1) * RAD_TO_STEPS);
    speeds[1] = speed_shoulder;
    speeds[2] = -speed_shoulder;  // Motor 3 opposite direction
    
    // J3 (Elbow) -> Motor 4
    speeds[3] = (int)(q_dot(2) * RAD_TO_STEPS);
    
    // J4 (Wrist Pitch) -> Motor 5
    speeds[4] = (int)(q_dot(3) * RAD_TO_STEPS);
    
    // Roll -> Motor 6
    speeds[5] = (int)(roll_vel * RAD_TO_STEPS);
    
    // Apply speed limits
    int max_speed = (int)(1500 * speed_multiplier);
    for (int& s : speeds) {
        if (s > max_speed) s = max_speed;
        if (s < -max_speed) s = -max_speed;
    }
    
    // Adaptive minimum speed boost based on distance to target
    // Allow slower speeds during final approach to avoid overshoot
    if (boost_low_speeds) {
        int min_speed = MIN_MOTOR_SPEED;
        
        // If very close to target (< 5mm), allow slower speeds to reduce jitter
        if (distance_to_target > 0 && distance_to_target < 5.0f) {
            min_speed = FINAL_APPROACH_SPEED;
        }
        
        for (int& s : speeds) {
            int abs_speed = abs(s);
            if (abs_speed > 0 && abs_speed < min_speed) {
                s = (s > 0) ? min_speed : -min_speed;
            }
        }
    }
    
    // Apply backlash compensation
    applyBacklashCompensation(speeds);
    
    servoDriver.syncWriteVelocity(motor_ids, speeds);
}

// ============================================================================
// Control Task - Runs in separate thread via TeensyThreads
// ============================================================================

void controlTask() {
    unsigned long last_loop = millis();
    unsigned long last_telemetry = 0;
    unsigned long loop_count = 0;
    unsigned long freq_measure_start = millis();
    float measured_freq = 0;
    
    // Timing diagnostics
    unsigned long total_read_time = 0;
    unsigned long total_compute_time = 0;
    unsigned long total_write_time = 0;
    unsigned long timing_samples = 0;
    
    // Target 500Hz+ control rate (like old firmware)
    const float TARGET_DT = 0.002f;  // 2ms = 500Hz
    
    while (true) {
        unsigned long loop_start = micros();
        unsigned long now = millis();
        float dt = (now - last_loop) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.05f) dt = 0.05f;  // Clamp max dt
        last_loop = now;
        
        // Measure actual loop frequency every second
        loop_count++;
        if (now - freq_measure_start >= 1000) {
            measured_freq = loop_count / ((now - freq_measure_start) / 1000.0f);
            loop_count = 0;
            freq_measure_start = now;
        }
        
        // Read positions
        unsigned long read_start = micros();
        std::vector<int> positions(N_MOTORS, -1);
        bool read_ok = readAllPositions(positions);
        unsigned long read_time = micros() - read_start;
        
        unsigned long compute_start = micros();
        
        if (read_ok || readFailCount < MAX_READ_FAILS) {
            updateArmState(positions, dt);
            
            if (torque_enabled) {
                Matrix<N_JOINTS> q_dot = {0, 0, 0, 0};
                float roll_vel = 0;
                bool should_boost = true;  // Boost low motor speeds
                float distance_to_target = 0.0f;  // Distance to target for adaptive boost
                
                // =========================================================
                // CARTESIAN MODE - Jacobian-based velocity control
                // =========================================================
                if (cartesian_mode && !sweep_mode) {
                    Matrix<3> current_pos = forwardKinematics(current_q);
                    
                    // =========================================================
                    // J1 is a LIMITED joint (cannot cross 0/4096 boundary)
                    // Range: [-175°, +175°] - treat like other joints
                    // Path planner should generate intermediate waypoints for large rotations
                    // =========================================================
                    
                    float current_base = current_q(0);
                    float target_base = atan2f(cartesian_target(1), cartesian_target(0));
                    
                    // Clamp target to J1 limits
                    if (target_base < JOINT_LIMITS[0].min_rad) target_base = JOINT_LIMITS[0].min_rad;
                    if (target_base > JOINT_LIMITS[0].max_rad) target_base = JOINT_LIMITS[0].max_rad;
                    
                    float base_error = target_base - current_base;
                    
                    // Check if motion would require crossing the boundary (error > 180°)
                    // This indicates path planning failure - should not happen with proper waypoints
                    if (fabsf(base_error) > M_PI) {
                        Serial.printf("WARN: J1 motion requires wrap (error=%.2f°), rejecting. Use intermediate waypoints!\n", base_error * 57.3f);
                        // Don't move - wait for better waypoints
                        cartesian_mode = false;
                        moving = false;
                        continue;
                    }
                    
                    // Normal Jacobian control
                    q_dot = jacobian.computeVelocity(
                        current_q, current_pos, cartesian_target, 
                        dt, rawPositions[0]
                    );
                    
                    // Calculate distance to target for adaptive boost control
                    float distance_to_target = 0;
                    for (int i = 0; i < 3; i++) {
                        float d = cartesian_target(i) - current_pos(i);
                        distance_to_target += d * d;
                    }
                    distance_to_target = sqrtf(distance_to_target);
                    
                    // Scale by speed multiplier
                    for (int i = 0; i < N_JOINTS; i++) {
                        q_dot(i) *= speed_multiplier;
                    }
                    
                    // Use QP-MPC with elbow-up posture constraint
                    MPCState mpc_state;
                    mpc_state.q = current_q;
                    mpc_state.q_dot = q_dot;
                    
                    MPCReference mpc_ref;
                    mpc_ref.q_ref = current_q + q_dot * dt;  // Desired next position
                    mpc_ref.q_dot_ref = q_dot;  // Desired velocity
                    mpc_ref.use_terminal_constraint = false;  // No terminal constraint
                    
                    MPCSolution sol = mpc.solveQP(mpc_state, mpc_ref, MPC_MODE_PROJECTION);
                    q_dot = sol.q_dot_next;
                    
                    // Roll velocity
                    roll_vel = (target_roll - current_roll) * 2.0f * speed_multiplier;
                    
                    // Check if target reached
                    if (jacobian.isTargetReached(current_pos, cartesian_target)) {
                        // Path mode - advance to next waypoint (using circular buffer)
                        if (path_mode && waypoint_count > 0) {
                            int current_idx = waypoint_head;  // Always working on waypoint at head
                            Waypoint& wp = waypoints[current_idx];
                            
                            // Check if we need to wait
                            if (wp.wait_time > 0 && !path_waiting) {
                                path_waiting = true;
                                path_wait_start = now;
                                stopAllMotors();
                                Serial.printf("PATH Waypoint, waiting %.1fs (buffer: %d)\n", 
                                    wp.wait_time, waypoint_count);
                            } else if (path_waiting) {
                                // Check if wait is done
                                if (now - path_wait_start >= (unsigned long)(wp.wait_time * 1000)) {
                                    path_waiting = false;
                                    // Remove completed waypoint from queue
                                    waypoint_head = (waypoint_head + 1) % MAX_WAYPOINTS;
                                    waypoint_count--;
                                    
                                    if (waypoint_count > 0) {
                                        int next_idx = waypoint_head;
                                        Waypoint& next = waypoints[next_idx];
                                        cartesian_target = next.pos;
                                        target_roll = next.roll;
                                        Serial.printf("PATH Moving to next waypoint (buffer: %d)\n", waypoint_count);
                                    } else {
                                        path_mode = false;
                                        cartesian_mode = false;
                                        moving = false;
                                        stopAllMotors();
                                        Serial.println("PATH_DONE Path completed");
                                    }
                                }
                                continue;  // Skip sending velocities while waiting
                            } else {
                                // No wait - remove and advance immediately
                                waypoint_head = (waypoint_head + 1) % MAX_WAYPOINTS;
                                waypoint_count--;
                                
                                if (waypoint_count > 0) {
                                    int next_idx = waypoint_head;
                                    Waypoint& next = waypoints[next_idx];
                                    cartesian_target = next.pos;
                                    target_roll = next.roll;
                                    Serial.printf("PATH Moving to next waypoint (buffer: %d)\n", waypoint_count);
                                } else {
                                    path_mode = false;
                                    cartesian_mode = false;
                                    moving = false;
                                    stopAllMotors();
                                    Serial.println("PATH_DONE Path completed");
                                }
                            }
                        } else {
                            // Simple move complete
                            cartesian_mode = false;
                            moving = false;
                            stopAllMotors();
                            Serial.println("DONE Cartesian target reached");
                        }
                        continue;
                    }
                    
                    // Path blending - advance early for smooth motion
                    if (path_mode && waypoint_count > 0) {
                        float err = 0;
                        for (int i = 0; i < 3; i++) {
                            float d = current_pos(i) - cartesian_target(i);
                            err += d * d;
                        }
                        err = sqrtf(err);
                        
                        // Blend to next waypoint if we have more than one
                        int current_idx = waypoint_head;  // Always working on head waypoint
                        if (waypoint_count > 1 && waypoints[current_idx].wait_time == 0) {
                            if (err < path_blend_radius) {
                                // Remove completed waypoint from queue
                                waypoint_head = (waypoint_head + 1) % MAX_WAYPOINTS;
                                waypoint_count--;
                                
                                // Target next waypoint (now at head)
                                int next_idx = waypoint_head;
                                Waypoint& next = waypoints[next_idx];
                                cartesian_target = next.pos;
                                target_roll = next.roll;
                                Serial.printf("PATH Blend to next waypoint (buffer: %d)\n", waypoint_count);
                            }
                        }
                        // On last waypoint, use blend radius for completion
                        else if (waypoint_count == 1 && err < path_blend_radius) {
                            // Close enough to last waypoint - finish immediately
                            waypoint_head = 0;
                            waypoint_tail = 0;
                            waypoint_count = 0;
                            path_mode = false;
                            cartesian_mode = false;
                            moving = false;
                            stopAllMotors();
                            Serial.println("PATH_DONE Path completed");
                        }
                    }
                }
                
                // =========================================================
                // SWEEP MODE - Direct Cartesian velocity control
                // =========================================================
                else if (sweep_mode) {
                    Matrix<3> current_pos = forwardKinematics(current_q);
                    float elbow = current_q(2);
                    
                    // Initialize sweep start position
                    if (!sweep_initialized) {
                        sweep_start_pos = current_pos;
                        sweep_initialized = true;
                        jacobian.reset();
                        
                        // Choose smart initial direction based on position
                        if (sweep_axis == 0) {  // X axis
                            sweep_direction = (current_pos(0) > 0) ? -1 : 1;
                        } else if (sweep_axis == 1) {  // Y axis
                            sweep_direction = (current_pos(1) > 0) ? -1 : 1;
                        } else {  // Z axis - start going DOWN to avoid singularity
                            sweep_direction = -1;  // Always start down for safety
                        }
                        
                        Serial.printf("SWEEP Start: (%.1f, %.1f, %.1f) axis=%d dir=%d q3=%.2f\n",
                            sweep_start_pos(0), sweep_start_pos(1), sweep_start_pos(2),
                            sweep_axis, sweep_direction, elbow);
                    }
                    
                    // Emergency elbow handling
                    if (elbow > 0.05f) {
                        Matrix<N_JOINTS> fix_q_dot = {0, 0, 0, 0};
                        fix_q_dot(2) = -1.5f * speed_multiplier;
                        fix_q_dot(1) = 0.3f * speed_multiplier;
                        fix_q_dot(3) = 0.5f * speed_multiplier;
                        
                        sendVelocityCommands(fix_q_dot, 0, false);
                        continue;
                    }
                    
                    // Warning zone
                    if (elbow > -0.15f) {
                        Matrix<N_JOINTS> safe_q_dot = {0, 0, 0, 0};
                        safe_q_dot(2) = -1.0f * speed_multiplier;
                        safe_q_dot(1) = 0.2f * speed_multiplier;
                        
                        if (sweep_axis == 2 && sweep_direction > 0) {
                            sweep_direction = -1;
                        }
                        
                        sendVelocityCommands(safe_q_dot, 0, false);
                        continue;
                    }
                    
                    // Get Jacobian
                    Matrix<3, N_JOINTS> J = calculateJacobian(current_q);
                    
                    // Check reversal conditions
                    static unsigned long last_reverse = 0;
                    float pos = current_pos(sweep_axis);
                    
                    if (now - last_reverse > 600) {
                        bool should_reverse = false;
                        
                        if (sweep_axis == 0) {
                            if ((pos > 180.0f && sweep_direction > 0) ||
                                (pos < -180.0f && sweep_direction < 0)) {
                                should_reverse = true;
                            }
                        } else if (sweep_axis == 1) {
                            if ((pos > 150.0f && sweep_direction > 0) ||
                                (pos < -150.0f && sweep_direction < 0)) {
                                should_reverse = true;
                            }
                        } else {
                            if (sweep_direction > 0 && elbow > -0.3f) {
                                should_reverse = true;
                            }
                            if ((pos > 260.0f && sweep_direction > 0) ||
                                (pos < 20.0f && sweep_direction < 0)) {
                                should_reverse = true;
                            }
                        }
                        
                        if (should_reverse) {
                            sweep_direction = -sweep_direction;
                            last_reverse = now;
                            Serial.printf("SWEEP: Reversed at pos=%.1f\n", pos);
                        }
                    }
                    
                    // Compute desired Cartesian velocity
                    Matrix<3> x_dot = {0, 0, 0};
                    float base_vel = sweep_velocity * sweep_direction * speed_multiplier;
                    
                    if (sweep_axis == 2 && sweep_direction > 0 && elbow > -0.35f) {
                        sweep_direction = -1;
                        base_vel = -fabsf(base_vel);
                    }
                    
                    x_dot(sweep_axis) = base_vel;
                    
                    // Drift correction
                    float correction_gain = (sweep_axis == 1) ? 3.0f : 5.0f;
                    float max_correction = (sweep_axis == 1) ? 30.0f : 50.0f;
                    
                    for (int axis = 0; axis < 3; axis++) {
                        if (axis != sweep_axis) {
                            float error = sweep_start_pos(axis) - current_pos(axis);
                            float correction = error * correction_gain;
                            if (correction > max_correction) correction = max_correction;
                            if (correction < -max_correction) correction = -max_correction;
                            x_dot(axis) = correction;
                        }
                    }
                    
                    // Compute DLS pseudoinverse
                    float damping = 0.05f;
                    if (elbow > -0.4f) {
                        float ratio = (elbow + 0.4f) / 0.25f;
                        if (ratio > 1.0f) ratio = 1.0f;
                        damping = 0.05f + ratio * 0.3f;
                    }
                    
                    Matrix<4, 3> J_pinv = jacobian.computeJacobianPinvDLS(J, damping);
                    q_dot = J_pinv * x_dot;
                    
                    // Clamp elbow velocity
                    if (elbow > -0.6f && q_dot(2) > 0) {
                        float scale = (-elbow - 0.15f) / 0.45f;
                        if (scale < 0) scale = 0;
                        q_dot(2) *= scale;
                    }
                    
                    if (elbow > -0.3f) {
                        q_dot(2) = fminf(q_dot(2), -0.5f * speed_multiplier);
                    }
                    
                    // Posture control
                    float target_elbow = -0.6f;
                    float elbow_error = target_elbow - elbow;
                    float elbow_correction = elbow_error * 0.5f;
                    if (elbow > -0.4f) {
                        float urgency = (elbow + 0.4f) / 0.25f;
                        elbow_correction -= urgency * 0.3f;
                    }
                    q_dot(2) += elbow_correction;
                    
                    // Gripper horizontal
                    float q2_raw = current_q(1) + M_PI_2;
                    float phi_current = q2_raw + current_q(2) + current_q(3);
                    float phi_error = 0.0f - phi_current;
                    q_dot(3) += phi_error * 0.2f;
                    
                    // Clamp base velocity
                    float max_base = 0.4f;
                    if (q_dot(0) > max_base) q_dot(0) = max_base;
                    if (q_dot(0) < -max_base) q_dot(0) = -max_base;
                    
                    // Scale to max joint speed
                    float max_qd = 0;
                    for (int i = 0; i < N_JOINTS; i++) {
                        if (fabsf(q_dot(i)) > max_qd) max_qd = fabsf(q_dot(i));
                    }
                    float max_joint_speed = 1.2f * speed_multiplier;
                    if (max_qd > max_joint_speed) {
                        float scale = max_joint_speed / max_qd;
                        for (int i = 0; i < N_JOINTS; i++) q_dot(i) *= scale;
                    }
                    
                    // Use QP-MPC for sweep mode
                    MPCState mpc_state;
                    mpc_state.q = current_q;
                    mpc_state.q_dot = q_dot;
                    
                    MPCReference mpc_ref;
                    mpc_ref.q_ref = current_q + q_dot * dt;
                    mpc_ref.q_dot_ref = q_dot;
                    mpc_ref.use_terminal_constraint = false;
                    
                    MPCSolution sol = mpc.solveQP(mpc_state, mpc_ref, MPC_MODE_PROJECTION);
                    q_dot = sol.q_dot_next;
                    
                    roll_vel = 0;
                }
                
                // =========================================================
                // DIRECT MODE - Joint control with MPC velocity tracking
                // =========================================================
                else if (direct_mode) {
                    // Calculate error and desired velocities
                    float max_err = 0;
                    Matrix<N_JOINTS> desired_vel;
                    for (int i = 0; i < N_JOINTS; i++) {
                        float err = target_q(i) - current_q(i);
                        if (i == 0) {
                            while (err > M_PI) err -= 2.0f * M_PI;
                            while (err < -M_PI) err += 2.0f * M_PI;
                        }
                        if (fabsf(err) > max_err) max_err = fabsf(err);
                        desired_vel(i) = err * 3.0f * speed_multiplier;  // P-gain for velocity
                    }
                    
                    // Check if target reached
                    if (max_err < 0.02f) {  // ~1 degree threshold
                        direct_mode = false;
                        moving = false;
                        stopAllMotors();
                        Serial.println("DONE Direct target reached");
                        continue;
                    }
                    
                    // Use MPC to track the desired velocity smoothly (with damping)
                    // IMPORTANT: Pass desired_vel as current velocity for MPC to regulate
                    MPCState mpc_state;
                    mpc_state.q = current_q;
                    mpc_state.q_dot = desired_vel;  // Use desired vel, not zero!
                    
                    MPCReference mpc_ref;
                    mpc_ref.q_ref = current_q + desired_vel * dt;  // Next position if following desired velocity
                    mpc_ref.q_dot_ref = desired_vel;  // Track the desired velocity
                    mpc_ref.use_terminal_constraint = false;  // Let MPC regulate smoothly
                    
                    MPCSolution sol = mpc.solveQP(mpc_state, mpc_ref, MPC_MODE_PROJECTION);
                    q_dot = sol.q_dot_next;
                    
                    roll_vel = (target_roll - current_roll) * 2.0f * speed_multiplier;
                    should_boost = false;
                }
                
                // =========================================================
                // IDLE - Hold position
                // =========================================================
                else {
                    q_dot = {0, 0, 0, 0};
                    roll_vel = 0;
                    should_boost = false;
                }
                
                // Disable boost if very slow (settling) or very close to target
                float max_vel = 0;
                for (int i = 0; i < N_JOINTS; i++) {
                    if (fabsf(q_dot(i)) > max_vel) max_vel = fabsf(q_dot(i));
                }
                
                // Disable boost in these cases:
                // 1. Very low velocity (< 0.05 rad/s) - likely settling
                // 2. Very close to target (< 3mm) in cartesian mode - final precision approach
                if (max_vel < 0.05f) {
                    should_boost = false;
                }
                if (cartesian_mode && distance_to_target > 0 && distance_to_target < 3.0f) {
                    should_boost = false;
                }
                
                unsigned long compute_time = micros() - compute_start;
                
                // Send velocity commands with distance info for adaptive boost
                unsigned long write_start = micros();
                if (cartesian_mode) {
                    sendVelocityCommands(q_dot, roll_vel, should_boost, distance_to_target);
                } else {
                    sendVelocityCommands(q_dot, roll_vel, should_boost);
                }
                unsigned long write_time = micros() - write_start;
                
                // Accumulate timing stats
                total_read_time += read_time;
                total_compute_time += compute_time;
                total_write_time += write_time;
                timing_samples++;
                
            } else {
                // Torque disabled
            }
        }
        
        // Telemetry at 10Hz
        if (now - last_telemetry >= 100) {
            last_telemetry = now;
            
            Matrix<3> pos = forwardKinematics(current_q);
            
            // Get MPC stats
            float mpc_time = mpc.getLastSolveTime();
            int qp_iters = mpc.getLastQPIterations();
            
            Serial.printf("POS %.2f %.2f %.2f %.3f | RAW %d %d %d %d %d %d | %.0fHz | MPC:%.2fms(%d)\n",
                pos(0), pos(1), pos(2), current_roll,
                rawPositions[0], rawPositions[1], rawPositions[2],
                rawPositions[3], rawPositions[4], rawPositions[5],
                measured_freq, mpc_time, qp_iters);
            
            // Timing breakdown removed - no longer printed to reduce serial spam
            // Reset counters for next measurement window
            total_read_time = 0;
            total_compute_time = 0;
            total_write_time = 0;
            timing_samples = 0;
        }
        
        // No delay - run as fast as servo communication allows (~800Hz)
        // threads.delay() in TeensyThreads blocks for 12ms, not 1ms!
        threads.yield();  // Just yield to allow other threads to run
    }
}

// ============================================================================
// Serial Command Processing
// ============================================================================

void processCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;
    
    // Debug: print all received commands
    Serial.printf("CMD: [%s] len=%d\n", cmd.c_str(), cmd.length());
    
    // ========== Cartesian Move: M x y z roll time ==========
    if (cmd.startsWith("M ") || cmd.startsWith("M\t")) {
        float x, y, z, roll, time;
        int n = sscanf(cmd.c_str(), "M %f %f %f %f %f", &x, &y, &z, &roll, &time);
        
        if (n >= 3) {
            if (n < 4) roll = 0;
            if (n < 5) time = 1.0f;
            
            if (!isInWorkspace(x, y, z)) {
                Serial.println("ERR Position out of workspace");
                return;
            }
            
            cartesian_target = {x, y, z};
            target_roll = roll;
            target_wrist_angle = wrist_locked ? 0.0f : target_wrist_angle;
            
            cartesian_mode = true;
            direct_mode = false;
            sweep_mode = false;
            moving = true;
            
            Serial.printf("OK Moving to (%.1f, %.1f, %.1f) roll=%.2f\n", x, y, z, roll);
        } else {
            Serial.println("ERR Invalid M command format");
        }
    }
    
    // ========== Direct Joint: D q1 q2 q3 q4 roll ==========
    else if (cmd.startsWith("D ") || cmd.startsWith("D\t")) {
        float q1, q2, q3, q4, roll;
        int n = sscanf(cmd.c_str(), "D %f %f %f %f %f", &q1, &q2, &q3, &q4, &roll);
        
        if (n >= 4) {
            if (n < 5) roll = current_roll;
            
            target_q = {q1, q2, q3, q4};
            target_roll = roll;
            target_q = clampToLimits(target_q);
            
            direct_mode = true;
            moving = true;  // Enable motion
            cartesian_mode = false;
            sweep_mode = false;
            Serial.printf("OK Direct mode activated - Target: [%.3f %.3f %.3f %.3f] moving=%d\n", 
                         target_q(0), target_q(1), target_q(2), target_q(3), moving);
        } else {
            Serial.println("ERR Invalid D command format");
        }
    }
    
    // ========== Stop: S ==========
    else if (cmd == "S" || cmd.startsWith("S ")) {
        cartesian_mode = false;
        direct_mode = false;
        sweep_mode = false;
        sweep_initialized = false;
        path_mode = false;
        path_waiting = false;
        moving = false;
        jacobian.reset();
        stopAllMotors();
        Serial.println("OK Stopped");
    }
    
    // ========== Torque: T 0/1 ==========
    else if (cmd.startsWith("T")) {
        int enable = -1;
        if (sscanf(cmd.c_str(), "T %d", &enable) == 1 || 
            sscanf(cmd.c_str(), "T%d", &enable) == 1) {
            
            if (enable == 1) {
                stopAllMotors();
                delay(50);
                
                for (uint8_t id : motor_ids) {
                    servoDriver.unlockEEPROM(id);
                    servoDriver.setWheelMode(id);
                    servoDriver.setTorqueEnable(id, true);
                }
                
                torque_enabled = true;
                Serial.println("OK Torque enabled");
            } else if (enable == 0) {
                torque_enabled = false;
                stopAllMotors();
                
                for (uint8_t id : motor_ids) {
                    servoDriver.setTorqueEnable(id, false);
                }
                
                Serial.println("OK Torque disabled");
            }
        } else if (cmd == "T") {
            torque_enabled = !torque_enabled;
            if (torque_enabled) {
                stopAllMotors();
                delay(50);
                for (uint8_t id : motor_ids) {
                    servoDriver.unlockEEPROM(id);
                    servoDriver.setWheelMode(id);
                    servoDriver.setTorqueEnable(id, true);
                }
                Serial.println("OK Torque enabled");
            } else {
                stopAllMotors();
                for (uint8_t id : motor_ids) {
                    servoDriver.setTorqueEnable(id, false);
                }
                Serial.println("OK Torque disabled");
            }
        }
    }
    
    // ========== Speed: SPD multiplier ==========
    else if (cmd.startsWith("SPD ")) {
        float spd;
        if (sscanf(cmd.c_str(), "SPD %f", &spd) == 1) {
            speed_multiplier = fmaxf(0.1f, fminf(5.0f, spd));
            Serial.printf("OK Speed multiplier = %.2f\n", speed_multiplier);
        }
    }
    
    // ========== Path Type: PTYPE 0=linear, 1=smooth ==========
    else if (cmd.startsWith("PTYPE ")) {
        int type;
        if (sscanf(cmd.c_str(), "PTYPE %d", &type) == 1) {
            path_type = (type == 1) ? 1 : 0;
            Serial.printf("OK Path type = %s\n", path_type == 1 ? "smooth" : "linear");
        }
    }
    
    // ========== Sweep: SW axis velocity ==========
    else if (cmd.startsWith("SW ")) {
        int axis;
        float vel;
        if (sscanf(cmd.c_str(), "SW %d %f", &axis, &vel) == 2) {
            if (axis >= 0 && axis <= 2) {
                sweep_axis = axis;
                sweep_velocity = fabsf(vel);
                sweep_mode = true;
                sweep_initialized = false;
                cartesian_mode = false;
                direct_mode = false;
                Serial.printf("OK Sweep axis=%d vel=%.1f\n", axis, sweep_velocity);
            }
        } else if (cmd == "SW" || cmd.startsWith("SW ")) {
            sweep_mode = false;
            sweep_initialized = false;
            stopAllMotors();
            Serial.println("OK Sweep stopped");
        }
    }
    
    // ========== Path Control: PA, PR, PC ==========
    else if (cmd.startsWith("PA ")) {
        // Path Add: PA x y z roll wait_ms
        float x, y, z, roll;
        int wait_ms = 0;
        int n = sscanf(cmd.c_str(), "PA %f %f %f %f %d", &x, &y, &z, &roll, &wait_ms);
        
        if (n >= 4) {
            // Add to circular buffer (will never be full - just overwrite oldest if needed)
            if (waypoint_count >= MAX_WAYPOINTS) {
                // Buffer full - this shouldn't happen anymore with continuous removal
                Serial.println("WARN Path buffer full, oldest waypoint dropped");
                waypoint_head = (waypoint_head + 1) % MAX_WAYPOINTS;
                waypoint_count--;
            }
            
            waypoints[waypoint_tail].pos(0) = x;
            waypoints[waypoint_tail].pos(1) = y;
            waypoints[waypoint_tail].pos(2) = z;
            waypoints[waypoint_tail].roll = roll;
            waypoints[waypoint_tail].wait_time = wait_ms / 1000.0f;
            
            waypoint_tail = (waypoint_tail + 1) % MAX_WAYPOINTS;
            waypoint_count++;
            
            Serial.printf("OK Path added (buffer: %d/%d) (%.1f,%.1f,%.1f)\n", waypoint_count, MAX_WAYPOINTS, x, y, z);
        } else {
            Serial.println("ERR Invalid PA format");
        }
    }
    else if (cmd == "PR" || cmd.startsWith("PR ")) {
        // Path Run
        if (waypoint_count > 0) {
            path_mode = true;
            cartesian_mode = true;
            direct_mode = false;
            sweep_mode = false;
            moving = true;
            
            // Set first waypoint (at head) as target
            int first_idx = waypoint_head;
            cartesian_target = waypoints[first_idx].pos;
            target_roll = waypoints[first_idx].roll;
            
            Serial.printf("OK Path running with %d waypoints in buffer\n", waypoint_count);
        } else {
            Serial.println("ERR No waypoints");
        }
    }
    else if (cmd == "PC" || cmd.startsWith("PC ")) {
        // Path Clear
        waypoint_count = 0;
        waypoint_head = 0;
        waypoint_tail = 0;
        path_mode = false;
        path_waiting = false;
        Serial.println("OK Path cleared");
    }
    
    // ========== MPC Tuning: MPCSET param value ==========
    else if (cmd.startsWith("MPCSET ")) {
        String param;
        float value;
        int space_idx = cmd.indexOf(' ', 7);
        if (space_idx > 0) {
            param = cmd.substring(7, space_idx);
            value = cmd.substring(space_idx + 1).toFloat();
            
            MPCWeights w = mpc.getWeights();
            bool updated = false;
            
            if (param == "Qpos") { for (int i = 0; i < N_JOINTS; i++) w.Q_pos(i) = value; updated = true; }
            else if (param == "Qvel") { for (int i = 0; i < N_JOINTS; i++) w.Q_vel(i) = value; updated = true; }
            else if (param == "Qfpos") { for (int i = 0; i < N_JOINTS; i++) w.Qf_pos(i) = value; updated = true; }
            else if (param == "Qfvel") { for (int i = 0; i < N_JOINTS; i++) w.Qf_vel(i) = value; updated = true; }
            else if (param == "R") { for (int i = 0; i < N_JOINTS; i++) w.R(i) = value; updated = true; }
            else if (param == "Rdelta") { for (int i = 0; i < N_JOINTS; i++) w.R_delta(i) = value; updated = true; }
            
            if (updated) {
                mpc.setWeights(w);
                Serial.printf("OK MPC %s = %.2f\n", param.c_str(), value);
            } else {
                Serial.printf("ERR Unknown MPC param: %s\n", param.c_str());
            }
        }
    }
    
    // ========== Get MPC Parameters: MPCGET ==========
    else if (cmd == "MPCGET") {
        MPCWeights w = mpc.getWeights();
        Serial.println("MPC_WEIGHTS:");
        Serial.printf("  Qpos=%.2f Qvel=%.2f Qfpos=%.2f Qfvel=%.2f R=%.2f Rdelta=%.2f\n",
            w.Q_pos(0), w.Q_vel(0), w.Qf_pos(0), w.Qf_vel(0), w.R(0), w.R_delta(0));
    }
    
    // ========== Get Position: P ==========
    else if (cmd == "P" || cmd.startsWith("P ")) {
        Matrix<3> pos = forwardKinematics(current_q);
        Serial.printf("POS %.2f %.2f %.2f %.3f | Q %.3f %.3f %.3f %.3f\n",
            pos(0), pos(1), pos(2), current_roll,
            current_q(0), current_q(1), current_q(2), current_q(3));
    }
    
    // ========== Status ==========
    else if (cmd == "?" || cmd.startsWith("H")) {
        Serial.println("Hybrid Jacobian+MPC Arm Controller - TEENSY 4.1");
        Serial.println("Commands:");
        Serial.println("  M x y z roll time  - Cartesian move");
        Serial.println("  D q1 q2 q3 q4 roll - Direct joint control");
        Serial.println("  PA x y z roll wait - Add waypoint to path");
        Serial.println("  PR                 - Run path");
        Serial.println("  PC                 - Clear path");
        Serial.println("  S                  - Stop");
        Serial.println("  T / T0 / T1        - Toggle/set torque");
        Serial.println("  SPD value          - Set speed multiplier");
        Serial.println("  SW axis vel        - Start sweep (0=X,1=Y,2=Z)");
        Serial.println("  MPCSET param val   - Set MPC weight (Qpos,Qvel,Qfpos,Qfvel,R,Rdelta)");
        Serial.println("  MPCGET             - Get MPC weights");
        Serial.println("  P                  - Get position");
    }
}

// ============================================================================
// Setup
// ============================================================================

void setup() {
    // USB Serial for commands/debugging
    Serial.begin(921600);
    while (!Serial && millis() < 3000);  // Wait for USB serial
    
    Serial.println("\n====================================");
    Serial.println("5DOF Arm Controller - Teensy 4.1");
    Serial.println("Hybrid Jacobian+MPC with TeensyThreads");
    Serial.println("====================================\n");
    
    // Initialize servo communication
    Serial.print("Initializing servos: ");
    servoDriver.begin(1000000);
    Serial.println("OK");
    
    // Ping all servos
    Serial.print("Checking servos: ");
    int found = 0;
    for (uint8_t id : motor_ids) {
        if (servoDriver.ping(id)) {
            Serial.printf("%d ", id);
            found++;
        }
    }
    Serial.printf("(%d/6 found)\n", found);
    
    // Configure servos for velocity mode
    Serial.print("Configuring servos: ");
    for (uint8_t id : motor_ids) {
        servoDriver.unlockEEPROM(id);
        servoDriver.setWheelMode(id);
        servoDriver.setTorqueEnable(id, true);
    }
    Serial.println("OK");
    
    // Initialize MPC
    Serial.print("Initializing MPC: ");
    MPCWeights weights = getDefaultWeights();
    MPCConstraints constraints = getDefaultConstraints();
    mpc.init(weights, constraints);
    mpc.setElbowUpEnforcement(true, 0.1f);
    Serial.println("OK");
    
    // Initialize Jacobian controller
    Serial.print("Initializing Jacobian: ");
    JacobianConfig jconfig;
    jconfig.cart_gain = 2.5f;
    jconfig.max_cart_vel = 150.0f;
    jconfig.max_joint_vel = 1.5f;
    jconfig.max_base_vel = 0.8f;
    jconfig.smoothing_alpha = 0.4f;
    jconfig.max_acceleration = 2.0f;
    jacobian.init(jconfig);
    Serial.println("OK");
    
    // Read initial positions
    std::vector<int> positions(N_MOTORS, 2048);
    readAllPositions(positions);
    updateArmState(positions, 0.02f);
    
    Matrix<3> init_pos = forwardKinematics(current_q);
    cartesian_target = init_pos;
    target_q = current_q;
    target_roll = current_roll;
    
    Serial.printf("Initial position: (%.1f, %.1f, %.1f)\n",
        init_pos(0), init_pos(1), init_pos(2));
    Serial.printf("Initial joints: (%.2f, %.2f, %.2f, %.2f)\n",
        current_q(0), current_q(1), current_q(2), current_q(3));
    
    // Create control thread using TeensyThreads
    // Stack size 8192, returns thread ID
    Serial.print("Starting control thread: ");
    int thread_id = threads.addThread(controlTask, 0, 8192);
    if (thread_id >= 0) {
        Serial.printf("OK (id=%d)\n", thread_id);
    } else {
        Serial.println("FAILED!");
    }
    
    Serial.println("\nReady! Type ? for help.");
}

// ============================================================================
// Main Loop (Serial handling)
// ============================================================================

void loop() {
    // Process serial commands
    static String cmdBuffer = "";
    
    while (Serial.available()) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (cmdBuffer.length() > 0) {
                processCommand(cmdBuffer);
                cmdBuffer = "";
            }
        } else {
            cmdBuffer += c;
            
            // Prevent buffer overflow
            if (cmdBuffer.length() > 200) {
                cmdBuffer = "";
            }
        }
    }
    
    threads.yield();  // Yield to control thread without blocking
}
