/**
 * Hybrid Jacobian+MPC 5DOF Arm Controller
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

#include "ArmModel.h"
#include "TinyMPC.h"
#include "AnalyticalIK.h"
#include "JacobianController.h"
#include "ServoDriver.h"

using namespace BLA;

// ============================================================================
// Pin Definitions
// ============================================================================
#define SERVO_RX_PIN 16
#define SERVO_TX_PIN 17
#define SERVO_DIR_PIN 4

// ============================================================================
// Objects
// ============================================================================
ServoDriver servoDriver(Serial2, SERVO_DIR_PIN, SERVO_RX_PIN, SERVO_TX_PIN);
TinyMPC mpc;
JacobianController jacobian;

// ============================================================================
// Velocity Scaling for Servo Speed
// ============================================================================
// ST3215: 4096 steps/revolution, velocity in steps/s
// 1 rad/s = 4096 / (2*PI) = 652 steps/s
const float RAD_TO_STEPS = 4096.0f / (2.0f * M_PI);

// Minimum motor speed - motors jitter below this
const int MIN_MOTOR_SPEED = 50;

// ============================================================================
// State Variables
// ============================================================================
SemaphoreHandle_t stateMutex;

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

// Mode flags
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

const int MAX_WAYPOINTS = 32;
Waypoint waypoints[MAX_WAYPOINTS];
int waypoint_count = 0;
int current_waypoint = 0;
bool path_mode = false;
float path_blend_radius = 15.0f;
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
 */
void sendVelocityCommands(const Matrix<N_JOINTS>& q_dot, float roll_vel, bool boost_low_speeds = true) {
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
    
    // Boost low speeds to minimum usable level (motors jitter below ~50 steps/s)
    if (boost_low_speeds) {
        for (int& s : speeds) {
            int abs_speed = abs(s);
            if (abs_speed > 0 && abs_speed < MIN_MOTOR_SPEED) {
                s = (s > 0) ? MIN_MOTOR_SPEED : -MIN_MOTOR_SPEED;
            }
        }
    }
    
    // Apply backlash compensation
    applyBacklashCompensation(speeds);
    
    servoDriver.syncWriteVelocity(motor_ids, speeds);
}

// ============================================================================
// Control Task (runs on Core 1) - HIGH SPEED LOOP
// ============================================================================

void controlTask(void* parameter) {
    unsigned long last_loop = millis();
    unsigned long last_telemetry = 0;
    
    // Target 500Hz+ control rate (like old firmware)
    const float TARGET_DT = 0.002f;  // 2ms = 500Hz
    
    while (true) {
        unsigned long now = millis();
        float dt = (now - last_loop) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.05f) dt = 0.05f;  // Clamp max dt
        last_loop = now;
        
        // Read positions
        std::vector<int> positions(N_MOTORS, -1);
        bool read_ok = readAllPositions(positions);
        
        if (read_ok || readFailCount < MAX_READ_FAILS) {
            updateArmState(positions, dt);
            
            if (torque_enabled) {
                Matrix<N_JOINTS> q_dot = {0, 0, 0, 0};
                float roll_vel = 0;
                bool should_boost = true;  // Boost low motor speeds
                
                // =========================================================
                // CARTESIAN MODE - Jacobian-based velocity control
                // =========================================================
                if (cartesian_mode && !sweep_mode) {
                    Matrix<3> current_pos = forwardKinematics(current_q);
                    
                    // =========================================================
                    // J1 DISCONTINUITY HANDLING - Check BEFORE Jacobian
                    // If target requires crossing 0↔4095, go the long way
                    // =========================================================
                    
                    // Static state for wrap mode persistence
                    static bool wrap_active = false;
                    static int wrap_direction = 0;  // +1 or -1
                    static int wrap_start_raw = 0;
                    
                    float current_base = current_q(0);
                    float target_base = atan2f(cartesian_target(1), cartesian_target(0));
                    float base_error = target_base - current_base;
                    
                    // Normalize to [-π, π]
                    while (base_error > M_PI) base_error -= 2.0f * M_PI;
                    while (base_error < -M_PI) base_error += 2.0f * M_PI;
                    
                    // Check if we should START wrapping
                    if (!wrap_active) {
                        bool near_max = rawPositions[0] > 3700;
                        bool near_min = rawPositions[0] < 500;
                        
                        if (near_max && base_error > 0.1f) {
                            // At max limit, need to go negative (long way)
                            wrap_active = true;
                            wrap_direction = -1;
                            wrap_start_raw = rawPositions[0];
                            Serial.printf("WRAP START: dir=%d from raw=%d\n", wrap_direction, wrap_start_raw);
                        } else if (near_min && base_error < -0.1f) {
                            // At min limit, need to go positive (long way)
                            wrap_active = true;
                            wrap_direction = 1;
                            wrap_start_raw = rawPositions[0];
                            Serial.printf("WRAP START: dir=%d from raw=%d\n", wrap_direction, wrap_start_raw);
                        }
                    }
                    
                    // Check if we should END wrapping
                    if (wrap_active) {
                        // We're done when we've moved past the middle (2047) 
                        // and are now on the "other side"
                        bool passed_middle = false;
                        if (wrap_direction > 0) {
                            // Going positive (from ~500 toward 2047 and beyond)
                            passed_middle = rawPositions[0] > 1500;
                        } else {
                            // Going negative (from ~3700 toward 2047 and below)
                            passed_middle = rawPositions[0] < 2500;
                        }
                        
                        // Also check if error is now small enough for normal control
                        bool error_small = fabsf(base_error) < 0.5f;
                        
                        if (passed_middle && error_small) {
                            wrap_active = false;
                            Serial.printf("WRAP END: raw=%d error=%.2f\n", rawPositions[0], base_error);
                        }
                    }
                    
                    if (wrap_active) {
                        // WRAP MODE: Move J1 in locked direction, hold other joints
                        float wrap_vel = wrap_direction * 1.5f * speed_multiplier;
                        
                        int motor_speed = (int)(wrap_vel * RAD_TO_STEPS);
                        if (abs(motor_speed) < 150) motor_speed = wrap_direction * 150;
                        
                        // Send ONLY J1 velocity, others at 0
                        std::vector<int> wrap_speeds = {motor_speed, 0, 0, 0, 0, 0};
                        servoDriver.syncWriteVelocity(motor_ids, wrap_speeds);
                        
                        static unsigned long last_wrap_msg = 0;
                        if (now - last_wrap_msg > 200) {
                            Serial.printf("J1 WRAP: raw=%d dir=%d motor_spd=%d\n",
                                rawPositions[0], wrap_direction, motor_speed);
                            last_wrap_msg = now;
                        }
                        
                        // Telemetry during wrap
                        if (now - last_telemetry >= 100) {
                            last_telemetry = now;
                            Matrix<3> pos = forwardKinematics(current_q);
                            Serial.printf("POS %.2f %.2f %.2f %.3f | RAW %d %d %d %d %d %d\n",
                                pos(0), pos(1), pos(2), current_roll,
                                rawPositions[0], rawPositions[1], rawPositions[2],
                                rawPositions[3], rawPositions[4], rawPositions[5]);
                        }
                        
                        vTaskDelay(1);
                        continue;
                    } else {
                        // Normal Jacobian control
                        q_dot = jacobian.computeVelocity(
                            current_q, current_pos, cartesian_target, 
                            dt, rawPositions[0]
                        );
                        
                        // Scale by speed multiplier
                        for (int i = 0; i < N_JOINTS; i++) {
                            q_dot(i) *= speed_multiplier;
                        }
                        
                        // Project velocities through MPC constraints
                        q_dot = mpc.projectVelocity(q_dot, current_q, dt);
                        
                        // Roll velocity
                        roll_vel = (target_roll - current_roll) * 2.0f * speed_multiplier;
                    }
                    
                    // Check if target reached
                    if (jacobian.isTargetReached(current_pos, cartesian_target)) {
                        // Path mode - advance to next waypoint
                        if (path_mode && current_waypoint < waypoint_count) {
                            Waypoint& wp = waypoints[current_waypoint];
                            
                            // Check if we need to wait
                            if (wp.wait_time > 0 && !path_waiting) {
                                path_waiting = true;
                                path_wait_start = now;
                                stopAllMotors();
                                Serial.printf("PATH Waypoint %d, waiting %.1fs\n", 
                                    current_waypoint + 1, wp.wait_time);
                            } else if (path_waiting) {
                                // Check if wait is done
                                if (now - path_wait_start >= (unsigned long)(wp.wait_time * 1000)) {
                                    path_waiting = false;
                                    current_waypoint++;
                                    
                                    if (current_waypoint < waypoint_count) {
                                        Waypoint& next = waypoints[current_waypoint];
                                        cartesian_target = next.pos;
                                        target_roll = next.roll;
                                        Serial.printf("PATH Moving to waypoint %d\n", current_waypoint + 1);
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
                                // No wait - advance immediately
                                current_waypoint++;
                                if (current_waypoint < waypoint_count) {
                                    Waypoint& next = waypoints[current_waypoint];
                                    cartesian_target = next.pos;
                                    target_roll = next.roll;
                                    Serial.printf("PATH Moving to waypoint %d\n", current_waypoint + 1);
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
                    if (path_mode && current_waypoint < waypoint_count - 1) {
                        float err = 0;
                        for (int i = 0; i < 3; i++) {
                            float d = current_pos(i) - cartesian_target(i);
                            err += d * d;
                        }
                        err = sqrtf(err);
                        
                        if (err < path_blend_radius && waypoints[current_waypoint].wait_time == 0) {
                            current_waypoint++;
                            Waypoint& next = waypoints[current_waypoint];
                            cartesian_target = next.pos;
                            target_roll = next.roll;
                            Serial.printf("PATH Blend to waypoint %d\n", current_waypoint + 1);
                        }
                    }
                }
                
                // =========================================================
                // SWEEP MODE - Direct Cartesian velocity control
                // This bypasses the Jacobian controller's error-based approach
                // and directly applies constant velocity along the sweep axis.
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
                    
                    // =====================================================
                    // EMERGENCY: If elbow is DOWN (q3 > 0), fix it first!
                    // This is CRITICAL - bypass Jacobian and use direct control
                    // =====================================================
                    if (elbow > 0.05f) {
                        // Elbow has flipped to DOWN configuration - must fix!
                        Matrix<N_JOINTS> fix_q_dot = {0, 0, 0, 0};
                        fix_q_dot(2) = -1.5f * speed_multiplier;  // Push elbow UP (negative)
                        fix_q_dot(1) = 0.3f * speed_multiplier;   // Shoulder adjustment
                        fix_q_dot(3) = 0.5f * speed_multiplier;   // Wrist compensation
                        
                        static unsigned long last_fix_msg = 0;
                        if (now - last_fix_msg > 500) {
                            Serial.printf("SWEEP: Fixing elbow-down (q3=%.2f)...\n", elbow);
                            last_fix_msg = now;
                        }
                        
                        sendVelocityCommands(fix_q_dot, 0, false);
                        continue;  // Skip normal sweep until elbow is fixed
                    }
                    
                    // =====================================================
                    // WARNING: If elbow is getting close to straight, take action
                    // =====================================================
                    if (elbow > -0.15f) {
                        // Elbow is dangerously close to singularity!
                        // Apply strong correction and slow down
                        Matrix<N_JOINTS> safe_q_dot = {0, 0, 0, 0};
                        safe_q_dot(2) = -1.0f * speed_multiplier;  // Push elbow UP
                        safe_q_dot(1) = 0.2f * speed_multiplier;   // Help shoulder
                        
                        // If doing Z sweep going up, also reverse direction
                        if (sweep_axis == 2 && sweep_direction > 0) {
                            sweep_direction = -1;
                            Serial.printf("SWEEP: Elbow singularity (q3=%.2f), reversing Z\n", elbow);
                        }
                        
                        static unsigned long last_warn = 0;
                        if (now - last_warn > 300) {
                            Serial.printf("SWEEP: Elbow near singularity (q3=%.2f)\n", elbow);
                            last_warn = now;
                        }
                        
                        sendVelocityCommands(safe_q_dot, 0, false);
                        continue;  // Skip normal sweep until safer
                    }
                    
                    // Get Jacobian for this configuration
                    Matrix<3, N_JOINTS> J = calculateJacobian(current_q);
                    
                    // Compute manipulability for adaptive behavior
                    Matrix<3, 3> JJT = J * ~J;
                    float det = JJT(0,0) * (JJT(1,1)*JJT(2,2) - JJT(1,2)*JJT(2,1))
                              - JJT(0,1) * (JJT(1,0)*JJT(2,2) - JJT(1,2)*JJT(2,0))
                              + JJT(0,2) * (JJT(1,0)*JJT(2,1) - JJT(1,1)*JJT(2,0));
                    float manipulability = sqrtf(fabsf(det));
                    
                    // Check reversal conditions
                    static unsigned long last_reverse = 0;
                    float pos = current_pos(sweep_axis);
                    
                    if (now - last_reverse > 600) {  // 600ms debounce
                        bool should_reverse = false;
                        
                        if (sweep_axis == 0) {  // X axis
                            if ((pos > 180.0f && sweep_direction > 0) ||
                                (pos < -180.0f && sweep_direction < 0)) {
                                should_reverse = true;
                            }
                        } else if (sweep_axis == 1) {  // Y axis
                            if ((pos > 150.0f && sweep_direction > 0) ||
                                (pos < -150.0f && sweep_direction < 0)) {
                                should_reverse = true;
                            }
                        } else {  // Z axis
                            // Reverse BEFORE hitting elbow singularity when going up
                            if (sweep_direction > 0 && elbow > -0.3f) {
                                should_reverse = true;
                                Serial.printf("SWEEP: Approaching singularity (q3=%.2f)\n", elbow);
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
                    
                    // =====================================================
                    // Z SWEEP SPECIAL HANDLING
                    // Going UP requires straightening elbow - we must limit this!
                    // =====================================================
                    if (sweep_axis == 2) {
                        if (sweep_direction > 0) {
                            // Going UP - check elbow status
                            if (elbow > -0.35f) {
                                // Elbow too straight - can't go up anymore!
                                // Force reverse
                                sweep_direction = -1;
                                base_vel = -fabsf(base_vel);
                                Serial.printf("SWEEP Z: Elbow limit (q3=%.2f), forcing DOWN\n", elbow);
                            } else if (elbow > -0.5f) {
                                // Getting close - slow down significantly
                                float scale = (-elbow - 0.35f) / 0.15f;  // 1.0 at -0.5, 0 at -0.35
                                if (scale < 0.1f) scale = 0.1f;
                                base_vel *= scale;
                            }
                        }
                        // Going DOWN is safe - elbow will naturally bend more
                    }
                    
                    // Primary: move along sweep axis
                    x_dot(sweep_axis) = base_vel;
                    
                    // Secondary: correct drift on other axes (P control to start position)
                    // Use LOWER gain for Y sweep to prevent Z jump
                    float correction_gain = (sweep_axis == 1) ? 3.0f : 5.0f;
                    float max_correction = (sweep_axis == 1) ? 30.0f : 50.0f;
                    
                    for (int axis = 0; axis < 3; axis++) {
                        if (axis != sweep_axis) {
                            float error = sweep_start_pos(axis) - current_pos(axis);
                            float correction = error * correction_gain;
                            // Clamp correction velocity
                            if (correction > max_correction) correction = max_correction;
                            if (correction < -max_correction) correction = -max_correction;
                            x_dot(axis) = correction;
                        }
                    }
                    
                    // Compute adaptive damping
                    float damping = 0.05f;
                    if (elbow > -0.4f) {
                        // Near singularity - increase damping
                        float ratio = (elbow + 0.4f) / 0.25f;
                        if (ratio > 1.0f) ratio = 1.0f;
                        damping = 0.05f + ratio * 0.3f;
                    }
                    
                    // Compute DLS pseudoinverse
                    Matrix<4, 3> J_pinv = jacobian.computeJacobianPinvDLS(J, damping);
                    
                    // Convert Cartesian velocity to joint velocity
                    q_dot = J_pinv * x_dot;
                    
                    // =====================================================
                    // CRITICAL: HARD CLAMP on elbow velocity
                    // The Jacobian may compute positive q_dot(2) which flips elbow
                    // We MUST prevent this - the arm should NEVER flip during sweep
                    // =====================================================
                    
                    // If elbow is anywhere near straight, NEVER allow positive q_dot(2)
                    if (elbow > -0.6f && q_dot(2) > 0) {
                        // Scale down or zero out positive elbow velocity
                        float scale = (-elbow - 0.15f) / 0.45f;  // 1.0 at q3=-0.6, 0 at q3=-0.15
                        if (scale < 0) scale = 0;
                        q_dot(2) *= scale;  // This will zero it when close to straight
                    }
                    
                    // If elbow is REALLY close to straight, force it negative
                    if (elbow > -0.3f) {
                        // Override with strong negative velocity
                        q_dot(2) = fminf(q_dot(2), -0.5f * speed_multiplier);
                    }
                    
                    // =====================================================
                    // POSTURE CONTROL - Keep elbow UP and gripper horizontal
                    // =====================================================
                    
                    // Elbow-up bias - ALWAYS push elbow toward safe zone
                    float target_elbow = -0.6f;  // Target well-bent elbow
                    float elbow_error = target_elbow - elbow;  // Negative if too straight
                    float elbow_correction = elbow_error * 0.5f;
                    
                    // Stronger correction when approaching danger zone
                    if (elbow > -0.4f) {
                        float urgency = (elbow + 0.4f) / 0.25f;
                        elbow_correction -= urgency * 0.3f;  // Extra push toward negative
                    }
                    q_dot(2) += elbow_correction;
                    
                    // Gripper horizontal (null-space)
                    float q2_raw = current_q(1) + M_PI_2;
                    float phi_current = q2_raw + current_q(2) + current_q(3);
                    float phi_error = 0.0f - phi_current;  // Target horizontal
                    q_dot(3) += phi_error * 0.2f;  // Reduced from 0.3
                    
                    // Clamp base velocity (prevent wild swings)
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
                    
                    // Project through MPC constraints
                    q_dot = mpc.projectVelocity(q_dot, current_q, dt);
                    
                    roll_vel = 0;
                }
                
                // =========================================================
                // DIRECT MODE - Joint control with MPC
                // =========================================================
                else if (direct_mode) {
                    // Simple proportional control toward target joints
                    for (int i = 0; i < N_JOINTS; i++) {
                        float err = target_q(i) - current_q(i);
                        // Wrap J1 error
                        if (i == 0) {
                            while (err > M_PI) err -= 2.0f * M_PI;
                            while (err < -M_PI) err += 2.0f * M_PI;
                        }
                        q_dot(i) = err * 5.0f * speed_multiplier;
                    }
                    
                    // Project through constraints
                    q_dot = mpc.projectVelocity(q_dot, current_q, dt);
                    
                    roll_vel = (target_roll - current_roll) * 5.0f * speed_multiplier;
                    
                    // Don't boost for slider control (more precise)
                    should_boost = false;
                }
                
                // =========================================================
                // IDLE - Hold position (zero velocity)
                // =========================================================
                else {
                    q_dot = {0, 0, 0, 0};
                    roll_vel = 0;
                    should_boost = false;
                }
                
                // Disable boost if all velocities are very low (arm settling)
                // This prevents jitter at the target position
                float max_vel = 0;
                for (int i = 0; i < N_JOINTS; i++) {
                    if (fabsf(q_dot(i)) > max_vel) max_vel = fabsf(q_dot(i));
                }
                if (max_vel < 0.05f) {  // Less than ~3 deg/s
                    should_boost = false;
                }
                
                // Send velocity commands
                sendVelocityCommands(q_dot, roll_vel, should_boost);
                
            } else {
                // Torque disabled - do nothing
            }
        }
        
        // Telemetry at 10Hz
        if (now - last_telemetry >= 100) {
            last_telemetry = now;
            
            Matrix<3> pos = forwardKinematics(current_q);
            
            Serial.printf("POS %.2f %.2f %.2f %.3f | RAW %d %d %d %d %d %d\n",
                pos(0), pos(1), pos(2), current_roll,
                rawPositions[0], rawPositions[1], rawPositions[2],
                rawPositions[3], rawPositions[4], rawPositions[5]);
        }
        
        // Fast loop - minimal delay
        vTaskDelay(1);  // ~1ms = ~1000Hz max
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
            
            // Check workspace
            if (!isInWorkspace(x, y, z)) {
                Serial.println("ERR Position out of workspace");
                return;
            }
            
            // Set target
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
            
            // Clamp to limits
            target_q = clampToLimits(target_q);
            
            direct_mode = true;
            cartesian_mode = false;
            sweep_mode = false;
            
            // No response for slider control (too frequent)
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
    
    // ========== Torque: T 0/1 (like old firmware) ==========
    else if (cmd.startsWith("T")) {
        int enable = -1;
        // Try parsing "T 0", "T 1", "T0", "T1"
        if (sscanf(cmd.c_str(), "T %d", &enable) == 1 || 
            sscanf(cmd.c_str(), "T%d", &enable) == 1) {
            
            if (enable == 1) {
                // First stop all motors
                stopAllMotors();
                delay(20);
                // Re-enable torque on each motor with unlock sequence
                for (int i = 0; i < N_MOTORS; i++) {
                    servoDriver.unlockEEPROM(motor_ids[i]);
                    delay(5);
                    servoDriver.setTorqueEnable(motor_ids[i], true);
                    delay(10);
                }
                torque_enabled = true;
                Serial.println("OK Torque ON");
            } else if (enable == 0) {
                // First stop all motors
                stopAllMotors();
                delay(20);
                torque_enabled = false;  // Stop control loop from sending commands
                delay(50);  // Give control loop time to stop
                // Now disable torque with unlock sequence
                for (int i = 0; i < N_MOTORS; i++) {
                    servoDriver.unlockEEPROM(motor_ids[i]);
                    delay(5);
                    servoDriver.setTorqueEnable(motor_ids[i], false);
                    delay(10);
                }
                Serial.println("OK Torque OFF");
            } else {
                Serial.println("ERR Invalid T value (use 0 or 1)");
            }
        } else if (cmd == "T") {
            // Toggle torque
            bool new_state = !torque_enabled;
            stopAllMotors();
            delay(20);
            if (!new_state) {
                torque_enabled = false;
                delay(50);
            }
            for (int i = 0; i < N_MOTORS; i++) {
                servoDriver.unlockEEPROM(motor_ids[i]);
                delay(5);
                servoDriver.setTorqueEnable(motor_ids[i], new_state);
                delay(10);
            }
            if (new_state) {
                torque_enabled = true;
            }
            Serial.printf("OK Torque %s\n", torque_enabled ? "ON" : "OFF");
        } else {
            Serial.printf("ERR Unknown T command: [%s]\n", cmd.c_str());
        }
    }
    
    // ========== PID Tuning: P joint kp ki kd ==========
    // For GUI compatibility - just acknowledge (Jacobian doesn't use PID)
    else if (cmd.startsWith("P ")) {
        int joint;
        float kp, ki, kd;
        if (sscanf(cmd.c_str(), "P %d %f %f %f", &joint, &kp, &ki, &kd) == 4) {
            if (joint >= 1 && joint <= 5) {
                // Acknowledge but Jacobian controller doesn't use per-joint PID
                // Speed multiplier controls overall speed
                Serial.printf("OK PID %d set to Kp=%.2f Ki=%.2f Kd=%.2f (Jacobian mode)\n", joint, kp, ki, kd);
            }
        }
    }
    
    // ========== Get PID: GP joint ==========
    else if (cmd.startsWith("GP")) {
        int joint;
        if (sscanf(cmd.c_str(), "GP %d", &joint) == 1) {
            if (joint >= 1 && joint <= 5) {
                // Return default PID values for compatibility
                Serial.printf("PID %d 25.00 0.00 0.50\n", joint);
            }
        }
    }
    
    // ========== Speed Multiplier: SP value ==========
    else if (cmd.startsWith("SP ")) {
        float sp;
        if (sscanf(cmd.c_str(), "SP %f", &sp) == 1) {
            speed_multiplier = fmaxf(0.5f, fminf(3.0f, sp));
            Serial.printf("OK Speed multiplier set to %.2f\n", speed_multiplier);
        }
    }
    
    // ========== Wrist Lock: WL ==========
    else if (cmd.startsWith("WL")) {
        wrist_locked = !wrist_locked;
        if (wrist_locked) {
            Serial.println("OK Wrist LOCKED");
        } else {
            Serial.println("OK Wrist FREE");
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
                sweep_direction = (vel >= 0) ? 1 : -1;
                sweep_initialized = false;  // Reset sweep state
                jacobian.reset();  // Reset Jacobian smoothing
                
                sweep_mode = true;
                cartesian_mode = false;
                direct_mode = false;
                
                const char* axis_names[] = {"X", "Y", "Z"};
                Serial.printf("OK Sweep %s at %.1f mm/s\n", axis_names[axis], sweep_velocity);
            }
        }
    }
    
    // ========== Read Position: R ==========
    else if (cmd.startsWith("R")) {
        Matrix<3> pos = forwardKinematics(current_q);
        // Use same format as periodic telemetry
        Serial.printf("POS %.2f %.2f %.2f %.3f | RAW %d %d %d %d %d %d\n",
            pos(0), pos(1), pos(2), current_roll,
            rawPositions[0], rawPositions[1], rawPositions[2],
            rawPositions[3], rawPositions[4], rawPositions[5]);
    }
    
    // ========== Path Commands ==========
    else if (cmd.startsWith("PC")) {
        // Path Clear
        waypoint_count = 0;
        current_waypoint = 0;
        path_mode = false;
        Serial.println("OK Path cleared");
    }
    else if (cmd.startsWith("PA ")) {
        // Path Add: PA x y z roll wait
        float x, y, z, roll = 0, wait = 0;
        int n = sscanf(cmd.c_str(), "PA %f %f %f %f %f", &x, &y, &z, &roll, &wait);
        
        if (n >= 3 && waypoint_count < MAX_WAYPOINTS) {
            waypoints[waypoint_count].pos = {x, y, z};
            waypoints[waypoint_count].roll = roll;
            waypoints[waypoint_count].wait_time = wait;
            waypoint_count++;
            
            Serial.printf("OK Added waypoint %d\n", waypoint_count);
        }
    }
    else if (cmd.startsWith("PR")) {
        // Path Run
        if (waypoint_count > 0) {
            current_waypoint = 0;
            path_mode = true;
            
            // Set first waypoint as target
            cartesian_target = waypoints[0].pos;
            target_roll = waypoints[0].roll;
            cartesian_mode = true;
            
            Serial.printf("OK Running path with %d waypoints\n", waypoint_count);
        }
    }
    else if (cmd.startsWith("PB ")) {
        // Path Blend radius
        float r;
        if (sscanf(cmd.c_str(), "PB %f", &r) == 1) {
            path_blend_radius = fmaxf(5.0f, fminf(50.0f, r));
            Serial.printf("OK Blend radius = %.1f mm\n", path_blend_radius);
        }
    }
    
    // ========== MPC Settings ==========
    else if (cmd.startsWith("MPC")) {
        // Show MPC/Jacobian status
        Serial.printf("MPC solve=%.2fms iter=%d\n", mpc.getLastSolveTime(), mpc.getTotalIterations());
        Serial.printf("Jacobian: manip=%.0f damp=%.3f singularity=%s\n",
            jacobian.getLastManipulability(), jacobian.getLastDamping(),
            jacobian.isNearSingularity() ? "YES" : "no");
    }
    else if (cmd.startsWith("ELBOW")) {
        // Toggle elbow-up enforcement
        static bool elbow_up = true;
        elbow_up = !elbow_up;
        mpc.setElbowUpEnforcement(elbow_up, 0.1f);
        
        if (elbow_up) {
            Serial.println("OK Elbow-UP enforced");
        } else {
            Serial.println("OK Elbow constraint disabled");
        }
    }
    
    // ========== Help ==========
    else if (cmd.startsWith("?") || cmd.startsWith("H")) {
        Serial.println("Hybrid Jacobian+MPC Arm Controller Commands:");
        Serial.println("  M x y z roll time  - Cartesian move");
        Serial.println("  D q1 q2 q3 q4 roll - Direct joint control");
        Serial.println("  S                  - Stop");
        Serial.println("  T / T0 / T1        - Toggle/set torque");
        Serial.println("  P j kp ki kd       - Set PID gains");
        Serial.println("  GP j               - Get PID gains");
        Serial.println("  SP value           - Speed multiplier");
        Serial.println("  WL                 - Toggle wrist lock");
        Serial.println("  SW axis vel        - Sweep mode");
        Serial.println("  R                  - Read position");
        Serial.println("  PC/PA/PR/PB        - Path commands");
        Serial.println("  MPC                - Show status");
        Serial.println("  ELBOW              - Toggle elbow-up");
    }
}

// ============================================================================
// Setup
// ============================================================================

void setup() {
    Serial.begin(921600);
    delay(100);
    
    Serial.println("\n=== Hybrid Jacobian+MPC 5DOF Arm Controller v3 ===");
    Serial.println("Jacobian velocity control + MPC constraint projection");
    Serial.println("High-speed loop (500Hz+) for smooth motion");
    
    // Initialize state mutex
    stateMutex = xSemaphoreCreateMutex();
    
    // Initialize servo driver
    servoDriver.begin(1000000);
    delay(100);
    
    // Set all servos to wheel mode
    Serial.print("Setting wheel mode: ");
    for (int i = 0; i < N_MOTORS; i++) {
        servoDriver.setWheelMode(motor_ids[i]);
        Serial.print(motor_ids[i]);
        Serial.print(" ");
        delay(20);
    }
    Serial.println("OK");
    
    // Enable torque
    Serial.print("Enabling torque: ");
    for (int i = 0; i < N_MOTORS; i++) {
        servoDriver.setTorqueEnable(motor_ids[i], true);
        delay(10);
    }
    Serial.println("OK");
    
    // Initialize MPC (constraint projector)
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
    
    // Create control task on Core 1 with high priority
    xTaskCreatePinnedToCore(
        controlTask,
        "ControlLoop",
        8192,
        NULL,
        3,  // Higher priority for fast loop
        NULL,
        1   // Core 1
    );
    
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
    
    delay(1);  // Yield to other tasks
}
