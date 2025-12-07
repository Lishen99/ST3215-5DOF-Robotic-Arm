#include <Arduino.h>
#include "Kinematics.h"
#include "ServoDriver.h"
#include "PID.h"
#include "Trajectory.h"

// Pin Definitions for ESP32-WROOM-32E
#define SERVO_RX_PIN 16 // RX2
#define SERVO_TX_PIN 17 // TX2
#define SERVO_DIR_PIN 4 // GPIO for RS-485 direction control

// Objects
ServoDriver servoDriver(Serial2, SERVO_DIR_PIN, SERVO_RX_PIN, SERVO_TX_PIN);
Kinematics kinematics;
Trajectory trajectory;

// PIDs - 4 joints for IK (Base, Shoulder, Elbow, WristPitch) + WristRoll separate
// Tuned for smooth motion with less overshoot
PID pids[5] = {
    PID(8.0, 0.0, 0.8, 1500),   // Base (motor 1) - lower P, higher D
    PID(12.0, 0.0, 1.0, 1500),  // Shoulder (motor 2) - lower P, higher D
    PID(12.0, 0.0, 1.0, 1500),  // Elbow (motor 4)
    PID(10.0, 0.0, 0.6, 1500),  // Wrist Pitch (motor 5)
    PID(8.0, 0.0, 0.4, 1500)    // Wrist Roll (motor 6)
};

// State - using volatile for cross-task access
volatile bool moving = false;
volatile bool joint_mode = false; // False = Cartesian (IK), True = Joint Space
volatile bool torque_enabled = true;
volatile bool position_valid = false;

// Protected shared data with mutex
SemaphoreHandle_t stateMutex;
SemaphoreHandle_t pidMutex; // Separate mutex for PID access
Matrix<4> current_q = {0, 0, 0, 0};     // 4-DOF for IK
Matrix<3> current_pos = {0, 0, 0};      // XYZ end-effector
float current_roll = 0;                  // Wrist roll stored separately

// Motor IDs - 6 motors: 1=Base, 2&3=Shoulder(coupled), 4=Elbow, 5=WristPitch, 6=WristRoll
std::vector<uint8_t> motor_ids = {1, 2, 3, 4, 5, 6}; 

// Motor Centers calculated from servo_limits.json: (min + max) / 2
// Motor 1: Manually calibrated to 2207
// Motor 2: (1365+3869)/2=2617, Motor 3: (1521+4021)/2=2771
// Motor 4: (1324+3803)/2=2563, Motor 5: (854+3466)/2=2160
// Motor 6: (0+4095)/2=2047
int centers[6] = {2207, 2617, 2771, 2563, 2160, 2047}; 

// Last known good positions for fallback
int lastPositions[6] = {2207, 2617, 2771, 2563, 2160, 2047};

// Raw positions for telemetry
int rawPositions[6] = {2207, 2617, 2771, 2563, 2160, 2047};

// Consecutive read failure counter
int readFailCount = 0;
const int MAX_READ_FAILS = 10;

// Target wrist angle for IK (pitch angle of end effector from horizontal)
float target_wrist_angle = 0.0f; // radians

// Move command parameters (set by loop(), read by controlTask())
float move_target_x = 0, move_target_y = 0, move_target_z = 0;
float move_target_roll = 0, move_duration_global = 1.0f;
Matrix<4> move_start_q = {0, 0, 0, 0};
Matrix<4> move_end_q = {0, 0, 0, 0};
float move_start_roll = 0, move_end_roll = 0;

// Direct joint target mode (for slider control)
volatile bool direct_joint_mode = false;
Matrix<4> direct_joint_target = {0, 0, 0, 0};
float direct_roll_target = 0;

// Sweep mode for continuous trajectory
volatile bool sweep_mode = false;
int sweep_axis = 0;  // 0=X, 1=Y, 2=Z
float sweep_velocity = 50.0f;  // mm/s
int sweep_direction = 1;  // 1 or -1
Matrix<3> sweep_start_pos = {200, 0, 200};  // Store initial position when sweep starts
bool sweep_initialized = false;

// Speed multiplier (adjustable from GUI, 0.5 to 2.0)
float speed_multiplier = 2.0f;  // Default 2x speed

// Wrist mode: true = locked at target_wrist_angle, false = free (IK controls it)
bool wrist_locked = true;

// Cartesian target for Jacobian-based control
Matrix<3> cartesian_target = {200, 0, 200};
volatile bool cartesian_mode = false;

// Center-crossing state: when moving from +X to -X (or vice versa), go to neutral first
volatile bool center_crossing_mode = false;

// FLIP MODE: Joint-space interpolation for large base rotations
// When target needs >90° base rotation, we flip arm configuration instead
volatile bool flip_mode = false;                  // Currently in flip interpolation?
Matrix<4> flip_start_q = {0, 0, 0, 0};           // Joint state when flip started
Matrix<4> flip_target_q = {0, 0, 0, 0};          // Target joint state after flip
float flip_start_roll = 0.0f;
float flip_target_roll = 0.0f;
volatile unsigned long flip_start_time = 0;
volatile float flip_duration = 1.0f;                       // Seconds for flip interpolation
Matrix<3> flip_final_cartesian = {0, 0, 0};      // Final Cartesian target after flip
float flip_final_roll = 0.0f;
Matrix<3> final_target_after_neutral = {0, 0, 0};
float final_roll_after_neutral = 0.0f;

// ========== SMOOTH PATH FOLLOWING ==========
// Waypoint queue for continuous motion without stops
struct Waypoint {
    float x, y, z, roll;
    float wait_time;  // 0 = blend through, >0 = stop and wait
};
const int MAX_WAYPOINTS = 32;
Waypoint path_waypoints[MAX_WAYPOINTS];
int path_count = 0;           // Total waypoints in path
int path_current_idx = 0;     // Current waypoint index
volatile bool path_mode = false;
float path_blend_radius = 15.0f;  // Start blending to next waypoint when within this distance (mm)
unsigned long path_wait_start = 0;
bool path_waiting = false;

// Backlash compensation: ~0.6 degrees = 0.0105 radians per joint
const float BACKLASH_RAD = 0.0105f;  // 0.6 degrees in radians
// Track previous direction for each joint (1 = positive, -1 = negative, 0 = stopped)
int prev_direction[6] = {0, 0, 0, 0, 0, 0};
// Accumulated backlash compensation for each motor
float backlash_offset[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// Helper to map raw (0-4095) to radians based on center
float rawToRad(int raw, int center_raw) {
    return (raw - center_raw) * (2.0f * PI / 4096.0f);
}

// Helper to map radians to raw
int radToRaw(float rad, int center_raw) {
    return center_raw + (int)(rad * 4096.0f / (2.0f * PI));
}

// CRITICAL: Motor 1 (base) cannot wrap around 0↔4095
// Safe range is 100 to 3995 (with small margin from 0 and 4095)
// Center is 2207, so safe range in radians is about ±2.75 rad (±158°)
const int BASE_MIN_RAW = 100;
const int BASE_MAX_RAW = 3995;
const float BASE_MIN_RAD = (BASE_MIN_RAW - 2207) * (2.0f * PI / 4096.0f);  // ~-3.23 rad
const float BASE_MAX_RAD = (BASE_MAX_RAW - 2207) * (2.0f * PI / 4096.0f);  // ~+2.75 rad

// Clamp base angle to valid range (prevents wrapping)
float clampBaseAngle(float q1_rad) {
    if (q1_rad > BASE_MAX_RAD) return BASE_MAX_RAD;
    if (q1_rad < BASE_MIN_RAD) return BASE_MIN_RAD;
    return q1_rad;
}

// Clamp base velocity to prevent crossing the no-go zone
// Returns clamped velocity (0 if would cross limit)
float clampBaseVelocity(float q1_current_rad, float q1_dot, int raw_pos) {
    // If moving toward a limit, check if we're close
    const float MARGIN_RAD = 0.1f;  // ~6 degrees safety margin
    
    if (q1_dot > 0 && (q1_current_rad > BASE_MAX_RAD - MARGIN_RAD || raw_pos > BASE_MAX_RAW - 50)) {
        // Moving positive but near max limit - stop
        return 0.0f;
    }
    if (q1_dot < 0 && (q1_current_rad < BASE_MIN_RAD + MARGIN_RAD || raw_pos < BASE_MIN_RAW + 50)) {
        // Moving negative but near min limit - stop
        return 0.0f;
    }
    return q1_dot;
}

// Clamp base motor speed based on current position
int clampBaseSpeed(float q1_current_rad, int speed, int raw_pos) {
    const float MARGIN_RAD = 0.1f;
    
    if (speed > 0 && (q1_current_rad > BASE_MAX_RAD - MARGIN_RAD || raw_pos > BASE_MAX_RAW - 50)) {
        return 0;
    }
    if (speed < 0 && (q1_current_rad < BASE_MIN_RAD + MARGIN_RAD || raw_pos < BASE_MIN_RAW + 50)) {
        return 0;
    }
    return speed;
}

// Clamp joint angles to within limits
Matrix<4> clampJointsToLimits(const Matrix<4>& q) {
    Matrix<4> result = q;
    for (int i = 0; i < 4; i++) {
        if (result(i) < JOINT_LIMITS[i].min_rad) result(i) = JOINT_LIMITS[i].min_rad;
        if (result(i) > JOINT_LIMITS[i].max_rad) result(i) = JOINT_LIMITS[i].max_rad;
    }
    return result;
}

// Check if target joint configuration is valid (within limits)
bool isJointConfigValid(const Matrix<4>& q) {
    for (int i = 0; i < 4; i++) {
        if (q(i) < JOINT_LIMITS[i].min_rad || q(i) > JOINT_LIMITS[i].max_rad) {
            return false;
        }
    }
    return true;
}

// Calculate "flipped" joint configuration (for crossing +X to -X)
// 
// From the logs, when arm reaches -X with elbow-up:
//   J2 ≈ +1.7 (shoulder tilted BACK - positive!)
//   J3 ≈ -1.75 (elbow bent)
//   J4 ≈ +2.0 (wrist compensating)
//
// When arm reaches +X with elbow-up:
//   J2 ≈ -0.1 (shoulder slightly forward - negative)
//   J3 ≈ -1.6 (elbow bent)
//   J4 ≈ -0.5 (wrist compensating)
//
// So the flip is essentially: swap J2 sign (and magnitude), adjust J4
Matrix<4> computeFlippedConfig(const Matrix<4>& current_q, const Matrix<3>& target_xyz) {
    Matrix<4> flipped = current_q;
    
    // Determine direction of flip based on target X
    bool going_to_negative_x = target_xyz(0) < 0;
    
    if (going_to_negative_x) {
        // Going to -X: need J2 positive (shoulder tilted back)
        // This puts the elbow high and behind, forearm reaches down toward -X
        flipped(1) = 1.5f;  // Shoulder back - POSITIVE
        
        // J3 stays negative (elbow bent)
        flipped(2) = fmin(current_q(2), -1.5f);  // Keep good bend
        
        // J4 needs to keep gripper roughly level pointing toward -X
        // When J2=1.5, J3=-1.5, we need J4 to compensate
        // For horizontal gripper: J2 + J3 + J4 ≈ 0 (but accounting for offsets)
        flipped(3) = 1.8f;  // Wrist pitched up significantly
        
    } else {
        // Going to +X: need J2 slightly negative (shoulder forward)
        flipped(1) = -0.1f;
        
        // J3 negative (elbow bent)
        flipped(2) = fmin(current_q(2), -1.2f);
        
        // J4 for forward-pointing gripper
        float j2_angle = flipped(1) + M_PI_2;
        float target_phi = 0.0f;  // Point forward
        flipped(3) = target_phi - j2_angle - flipped(2);
    }
    
    // Clamp to joint limits
    flipped = clampJointsToLimits(flipped);
    
    return flipped;
}

// Apply backlash compensation to motor speeds
// When direction reverses, add extra steps to overcome backlash
void applyBacklashCompensation(std::vector<int>& speeds) {
    const int BACKLASH_STEPS = 7;  // ~0.6 degrees = 6.8 steps, round to 7
    const int SPEED_THRESHOLD = 10;  // Minimum speed to consider "moving"
    
    for (size_t i = 0; i < speeds.size() && i < 6; i++) {
        int new_direction = 0;
        if (speeds[i] > SPEED_THRESHOLD) new_direction = 1;
        else if (speeds[i] < -SPEED_THRESHOLD) new_direction = -1;
        
        // Check for direction reversal
        if (new_direction != 0 && prev_direction[i] != 0 && new_direction != prev_direction[i]) {
            // Direction reversed - add compensation in the new direction
            backlash_offset[i] = BACKLASH_STEPS * new_direction;
        }
        
        // Apply any accumulated compensation
        if (backlash_offset[i] != 0.0f && new_direction != 0) {
            // Add compensation to speed (will be applied over several cycles)
            float comp_speed = backlash_offset[i] * 50.0f;  // Fast compensation
            speeds[i] += (int)comp_speed;
            
            // Decay the compensation
            float decay = fabs(speeds[i]) * 0.001f;  // Decay based on speed
            if (decay < 0.5f) decay = 0.5f;
            if (backlash_offset[i] > 0) {
                backlash_offset[i] -= decay;
                if (backlash_offset[i] < 0) backlash_offset[i] = 0;
            } else {
                backlash_offset[i] += decay;
                if (backlash_offset[i] > 0) backlash_offset[i] = 0;
            }
        }
        
        // Update direction tracking
        if (new_direction != 0) {
            prev_direction[i] = new_direction;
        }
    }
}

// MINIMUM MOTOR SPEED - motors are jerky below this threshold
// Used to boost low speeds to a usable level
const int MIN_MOTOR_SPEED = 50;  // steps/s minimum

// Read positions with fallback to individual reads
bool readAllPositions(std::vector<int>& positions) {
    // First try sync read (faster)
    if (servoDriver.syncReadPosition(motor_ids, positions)) {
        // Check how many were successful
        int valid_count = 0;
        for (size_t i = 0; i < positions.size(); i++) {
            if (positions[i] != -1) {
                valid_count++;
                lastPositions[i] = positions[i]; // Update last known good
            }
        }
        
        if (valid_count == (int)positions.size()) {
            readFailCount = 0;
            return true; // All successful
        }
        
        // Some failed - try individual reads for failed ones
        for (size_t i = 0; i < positions.size(); i++) {
            if (positions[i] == -1) {
                int pos = servoDriver.readPositionSingle(motor_ids[i]);
                if (pos != -1) {
                    positions[i] = pos;
                    lastPositions[i] = pos;
                    valid_count++;
                } else {
                    // Use last known good position
                    positions[i] = lastPositions[i];
                }
            }
        }
        
        if (valid_count > 0) {
            readFailCount = 0;
            return true;
        }
    }
    
    // Complete failure - try individual reads for all
    bool any_success = false;
    for (size_t i = 0; i < motor_ids.size(); i++) {
        int pos = servoDriver.readPositionSingle(motor_ids[i]);
        if (pos != -1) {
            positions[i] = pos;
            lastPositions[i] = pos;
            any_success = true;
        } else {
            positions[i] = lastPositions[i]; // Fallback to last known
        }
    }
    
    if (!any_success) {
        readFailCount++;
    } else {
        readFailCount = 0;
    }
    
    return any_success;
}

void controlTask(void *parameter) {
    unsigned long last_loop_time = millis();
    unsigned long start_move_time = 0;
    unsigned long last_telemetry = 0;
    unsigned long last_stats = 0;
    
    // Local copies to reduce mutex contention
    Matrix<4> local_q = {0, 0, 0, 0};
    float local_roll = 0;
    
    // For straight-line Cartesian interpolation
    Matrix<3> start_pos = {0, 0, 0};
    float start_roll_val = 0;
    
    while (true) {
        unsigned long now = millis();
        float dt = (now - last_loop_time) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.1f) dt = 0.1f; // Cap to prevent huge jumps
        last_loop_time = now;

        // 1. Read Positions
        std::vector<int> positions(6, -1);
        bool read_ok = readAllPositions(positions);
        
        if (read_ok || readFailCount < MAX_READ_FAILS) {
            // Store raw positions for telemetry
            for (int i = 0; i < 6; i++) {
                rawPositions[i] = positions[i];
            }
            
            // Map 6 Motors to 4 IK Joints
            // Physical arm layout (empirically determined):
            // - Motor 2 > center = +X direction → NEGATE (FK needs negative q2 for +X)
            // - Motor 4 < center = +X direction → NO NEGATE (negative raw gives +X in FK)
            // - Motor 5 < center = +X direction → NO NEGATE (same as motor 4)
            
            // Joint 0 (Base) = Motor 1
            local_q(0) = rawToRad(positions[0], centers[0]);
            
            // Joint 1 (Shoulder) = Motor 2 - NEGATED
            local_q(1) = -rawToRad(positions[1], centers[1]);
            
            // Joint 2 (Elbow) = Motor 4 - NOT negated
            local_q(2) = rawToRad(positions[3], centers[3]);
            
            // Joint 3 (Wrist Pitch) = Motor 5 - NOT negated
            local_q(3) = rawToRad(positions[4], centers[4]);
            
            // Roll = Motor 6 (separate, not part of position IK)
            local_roll = rawToRad(positions[5], centers[5]);
            
            // Calculate FK
            Matrix<3> pos = kinematics.forward_kinematics(local_q);
            
            // Update shared state with mutex
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                current_q = local_q;
                current_pos = pos;
                current_roll = local_roll;
                position_valid = true;
                xSemaphoreGive(stateMutex);
            }
            
            // Telemetry - send position at 10Hz (every 100ms) to avoid serial overflow
            // Disable during path mode to prevent corruption with path messages
            if (now - last_telemetry >= 100 && !path_mode) {
                Serial.printf("POS %.2f %.2f %.2f %.3f | RAW %d %d %d %d %d %d\n", 
                    pos(0), pos(1), pos(2), local_roll,
                    rawPositions[0], rawPositions[1], rawPositions[2],
                    rawPositions[3], rawPositions[4], rawPositions[5]);
                last_telemetry = now;
            }
        } else {
            // Too many consecutive failures
            if (now - last_telemetry >= 1000) {
                Serial.println("ERR: Servo communication lost!");
                last_telemetry = now;
            }
            position_valid = false;
        }
        
        // Print stats periodically
        if (now - last_stats >= 5000) {
            Serial.printf("STATS OK:%lu FAIL:%lu\n", 
                servoDriver.getSuccessCount(), servoDriver.getFailCount());
            last_stats = now;
        }

        // 2. Control Modes (ONLY IF TORQUE ENABLED AND POSITION VALID)
        if (torque_enabled && position_valid) {
            
            // =====================================================
            // FLIP MODE: Smooth joint-space interpolation for large base rotations
            // =====================================================
            // When transitioning +X to -X (or vice versa), we flip the arm
            // configuration instead of rotating the base. This is phase 1 of
            // a two-phase move: (1) flip via joint interpolation, (2) Jacobian refine.
            if (flip_mode && !moving && !sweep_mode) {
                // Initialize start time on first iteration (avoids cross-core timing issues)
                if (flip_start_time == 0) {
                    flip_start_time = now;
                    Serial.printf("FLIP starting: target_q=(%.2f,%.2f,%.2f,%.2f)\n",
                        flip_target_q(0), flip_target_q(1), flip_target_q(2), flip_target_q(3));
                }
                
                unsigned long elapsed = now - flip_start_time;
                float t = (float)elapsed / (flip_duration * 1000.0f);  // 0 to 1
                
                if (t >= 1.0f) {
                    // Flip phase complete - transition to Cartesian mode for final positioning
                    t = 1.0f;
                    flip_mode = false;
                    cartesian_mode = true;
                    
                    // Set Cartesian target to the final position
                    cartesian_target(0) = flip_final_cartesian(0);
                    cartesian_target(1) = flip_final_cartesian(1);
                    cartesian_target(2) = flip_final_cartesian(2);
                    move_target_roll = flip_final_roll;
                    
                    Serial.printf("FLIP complete, Jacobian taking over to (%.1f, %.1f, %.1f)\n",
                        flip_final_cartesian(0), flip_final_cartesian(1), flip_final_cartesian(2));
                    continue;  // Skip the rest of this iteration
                }
                
                // Smooth interpolation using cosine easing (smooth start and end)
                float smooth_t = 0.5f * (1.0f - cosf(t * M_PI));
                
                // Interpolate joint angles
                Matrix<4> target_q;
                for (int i = 0; i < 4; i++) {
                    target_q(i) = flip_start_q(i) + smooth_t * (flip_target_q(i) - flip_start_q(i));
                }
                float target_roll = flip_start_roll + smooth_t * (flip_target_roll - flip_start_roll);
                
                // Clamp to limits
                target_q = kinematics.clamp_joints(target_q);
                
                // PID control to track interpolated position
                std::vector<int> speeds;
                
                // Motor 1 (Base) - not negated
                float out0 = pids[0].compute(target_q(0), local_q(0), dt);
                int base_speed = (int)(out0 * 100.0f);
                base_speed = clampBaseSpeed(local_q(0), base_speed, rawPositions[0]);
                speeds.push_back(base_speed);
                
                // Motor 2 (Shoulder) - NEGATED
                float out1 = pids[1].compute(target_q(1), local_q(1), dt);
                int speed1 = (int)(-out1 * 100.0f);
                speeds.push_back(speed1);
                speeds.push_back(-speed1);  // Motor 3
                
                // Motor 4 (Elbow) - NOT negated
                float out2 = pids[2].compute(target_q(2), local_q(2), dt);
                speeds.push_back((int)(out2 * 100.0f));
                
                // Motor 5 (Wrist) - NOT negated
                float out3 = pids[3].compute(target_q(3), local_q(3), dt);
                speeds.push_back((int)(out3 * 100.0f));
                
                // Motor 6 (Roll)
                float out_roll = pids[4].compute(target_roll, local_roll, dt);
                speeds.push_back((int)(out_roll * 100.0f));
                
                // Clamp speeds (gentler during flip for smooth motion)
                for (int& s : speeds) {
                    if (s > 1000) s = 1000;
                    if (s < -1000) s = -1000;
                }
                
                servoDriver.syncWriteVelocity(motor_ids, speeds);
                
                // Debug output (sparse)
                static unsigned long last_flip_debug = 0;
                if (now - last_flip_debug > 200) {
                    Matrix<3> current_fk = kinematics.forward_kinematics(local_q);
                    Serial.printf("FLIP t=%.2f pos=(%.1f,%.1f,%.1f) q=(%.2f,%.2f,%.2f,%.2f)\n",
                        t, current_fk(0), current_fk(1), current_fk(2),
                        local_q(0), local_q(1), local_q(2), local_q(3));
                    last_flip_debug = now;
                }
            }
            // Direct joint control mode (for sliders)
            else if (direct_joint_mode && !moving && !sweep_mode && !cartesian_mode) {
                Matrix<4> target_q = direct_joint_target;
                float target_roll = direct_roll_target;
                
                // Clamp base angle to prevent wrap-around
                target_q(0) = clampBaseAngle(target_q(0));
                
                // Clamp to limits
                target_q = kinematics.clamp_joints(target_q);
                
                // PID Control with velocity smoothing
                // Note: Only Motor 2 is negated (motor > center = +X but FK needs negative q2)
                std::vector<int> speeds;
                
                // Motor 1 (Base) - not negated
                float out0 = pids[0].compute(target_q(0), local_q(0), dt);
                int base_speed = (int)(out0 * 100.0f);
                // Clamp base speed to prevent crossing limits
                base_speed = clampBaseSpeed(local_q(0), base_speed, rawPositions[0]);
                speeds.push_back(base_speed);
                
                // Motor 2 (Shoulder) - NEGATED
                float out1 = pids[1].compute(target_q(1), local_q(1), dt);
                int speed1 = (int)(-out1 * 100.0f);
                speeds.push_back(speed1);
                speeds.push_back(-speed1); // Motor 3 mechanically coupled/inverted
                
                // Motor 4 (Elbow) - NOT negated
                float out2 = pids[2].compute(target_q(2), local_q(2), dt);
                speeds.push_back((int)(out2 * 100.0f));
                
                // Motor 5 (Wrist Pitch) - NOT negated
                float out3 = pids[3].compute(target_q(3), local_q(3), dt);
                speeds.push_back((int)(out3 * 100.0f));
                
                // Motor 6 (Roll) - not negated
                float out_roll = pids[4].compute(target_roll, local_roll, dt);
                speeds.push_back((int)(out_roll * 100.0f));
                
                // Velocity smoothing (low-pass filter)
                static std::vector<int> prev_direct_speeds = {0, 0, 0, 0, 0, 0};
                float smooth_alpha = 0.4f;  // 0 = full smoothing, 1 = no smoothing
                for (size_t i = 0; i < speeds.size(); i++) {
                    speeds[i] = (int)(smooth_alpha * speeds[i] + (1.0f - smooth_alpha) * prev_direct_speeds[i]);
                    prev_direct_speeds[i] = speeds[i];
                }
                
                for(int& s : speeds) {
                    if (s > 1200) s = 1200;
                    if (s < -1200) s = -1200;
                }
                
                // Apply backlash compensation
                applyBacklashCompensation(speeds);
                
                servoDriver.syncWriteVelocity(motor_ids, speeds);
            }
            // Sweep mode - TRUE LINEAR Cartesian sweep with elbow-up preference
            else if (sweep_mode && !moving) {
                Matrix<3> current_fk = kinematics.forward_kinematics(local_q);
                
                // Initialize sweep start position on first iteration
                if (!sweep_initialized) {
                    sweep_start_pos = current_fk;
                    sweep_initialized = true;
                    
                    // Choose SMART initial direction based on current position
                    // This prevents immediately hitting singularities at workspace edges
                    if (sweep_axis == 0) {  // X axis
                        // If X > 0, go negative first (toward center)
                        sweep_direction = (current_fk(0) > 0) ? -1 : 1;
                    } else if (sweep_axis == 1) {  // Y axis
                        // If Y > 0, go negative first
                        sweep_direction = (current_fk(1) > 0) ? -1 : 1;
                    } else if (sweep_axis == 2) {  // Z axis
                        // Z=165 is midpoint of 50-280 range
                        // Above midpoint: go DOWN first (away from singularity at top)
                        // Below midpoint: go UP first
                        sweep_direction = (current_fk(2) > 165.0f) ? -1 : 1;
                    }
                    
                    Serial.printf("SWEEP: Started at (%.1f, %.1f, %.1f) on axis %d, dir=%d, q3=%.2f\n", 
                                  sweep_start_pos(0), sweep_start_pos(1), sweep_start_pos(2), 
                                  sweep_axis, sweep_direction, local_q(2));
                }
                
                // EMERGENCY: If elbow is DOWN (q3 > 0), we MUST fix it before sweeping
                // Use direct joint control to flip elbow up - bypass Jacobian entirely
                float current_elbow = local_q(2);
                if (current_elbow > 0.05f) {
                    // Elbow is in DOWN configuration - can't sweep like this!
                    // Apply direct velocities to flip elbow up
                    // NOTE: Negative velocity makes q3 go negative (elbow up)
                    // Motor 4 velocity sign: the motor command is NOT negated, so
                    // negative q_dot → negative motor speed → q3 decreases (toward negative)
                    Matrix<4> fix_q_dot = {0, 0, 0, 0};
                    fix_q_dot(2) = -1.5f * speed_multiplier;  // Negative = elbow UP (q3 decreasing toward negative)
                    fix_q_dot(1) = 0.3f * speed_multiplier;   // Shoulder adjustment
                    fix_q_dot(3) = 0.5f * speed_multiplier;   // Wrist compensation
                    
                    // Convert to motor speeds
                    std::vector<int> speeds;
                    const float RAD_TO_STEPS = 4096.0f / (2.0f * M_PI);
                    speeds.push_back(0);  // Base stays still
                    int speed1 = (int)(-fix_q_dot(1) * RAD_TO_STEPS);
                    speeds.push_back(speed1);
                    speeds.push_back(-speed1);
                    speeds.push_back((int)(fix_q_dot(2) * RAD_TO_STEPS));
                    speeds.push_back((int)(fix_q_dot(3) * RAD_TO_STEPS));
                    speeds.push_back(0);
                    
                    static unsigned long last_fix_msg = 0;
                    if (now - last_fix_msg > 500) {
                        Serial.printf("SWEEP: Fixing elbow-down (q3=%.2f), please wait...\n", current_elbow);
                        last_fix_msg = now;
                    }
                    
                    servoDriver.syncWriteVelocity(motor_ids, speeds);
                    continue;  // Skip normal sweep processing until elbow is fixed
                }
                
                // SAFETY: Check if base is near mechanical limits during sweep
                float base_angle = local_q(0);
                bool abort_sweep = false;
                
                // Only check mechanical limits, not wrap protection
                // The Jacobian naturally handles direction changes near ±180°
                if (rawPositions[0] < BASE_MIN_RAW + 30 || rawPositions[0] > BASE_MAX_RAW - 30) {
                    // Very close to mechanical limits - abort
                    abort_sweep = true;
                    Serial.printf("SWEEP: ABORTED - Base at mechanical limit (raw=%d, limits=%d-%d)\n",
                                  rawPositions[0], BASE_MIN_RAW, BASE_MAX_RAW);
                }
                
                if (abort_sweep) {
                    sweep_mode = false;
                    sweep_initialized = false;
                    std::vector<int> stops(6, 0);
                    servoDriver.syncWriteVelocity(motor_ids, stops);
                    continue;
                }
                
                // Get Jacobian and manipulability
                Matrix<3, 4> J = kinematics.calculate_jacobian(local_q);
                float manipulability = kinematics.calculate_manipulability(J);
                
                // Check workspace limits for reversal
                float pos_on_axis = current_fk(sweep_axis);
                float max_reach = L1 + L2 + L3 - 50.0f;
                
                // ELBOW SINGULARITY CHECK - only reverse if VERY close to straight
                float elbow_angle = local_q(2);
                bool elbow_singularity_critical = (elbow_angle > -0.1f);  // Really dangerous
                
                // Reversal logic - simple position-based limits, no manipulability check
                // The key is to reverse BEFORE hitting singularity, not after
                static unsigned long last_reverse_time = 0;
                static float last_pos_on_axis = 0;
                
                // Track if we're making progress (position changing in sweep direction)
                float pos_delta = pos_on_axis - last_pos_on_axis;
                bool making_progress = (sweep_direction > 0 && pos_delta > 0.5f) || 
                                       (sweep_direction < 0 && pos_delta < -0.5f);
                last_pos_on_axis = pos_on_axis;
                
                if (now - last_reverse_time > 800) {  // Longer debounce - 800ms
                    
                    // Simple position-based limits - reverse when reaching edges
                    if (sweep_axis == 0) {  // X axis
                        float x_limit = 180.0f;  // Fixed limit in mm, not percentage
                        if (pos_on_axis > x_limit && sweep_direction > 0) {
                            sweep_direction = -1;
                            last_reverse_time = now;
                            Serial.printf("SWEEP: Hit +X limit (%.1f > %.1f), reversing\n", pos_on_axis, x_limit);
                        }
                        if (pos_on_axis < -x_limit && sweep_direction < 0) {
                            sweep_direction = 1;
                            last_reverse_time = now;
                            Serial.printf("SWEEP: Hit -X limit (%.1f < %.1f), reversing\n", pos_on_axis, -x_limit);
                        }
                    } else if (sweep_axis == 1) {  // Y axis
                        float y_limit = 150.0f;
                        if (pos_on_axis > y_limit && sweep_direction > 0) {
                            sweep_direction = -1;
                            last_reverse_time = now;
                            Serial.printf("SWEEP: Y+ limit (%.1f), reversing\n", pos_on_axis);
                        }
                        if (pos_on_axis < -y_limit && sweep_direction < 0) {
                            sweep_direction = 1;
                            last_reverse_time = now;
                            Serial.printf("SWEEP: Y- limit (%.1f), reversing\n", pos_on_axis);
                        }
                    } else {  // Z axis
                        // Z limits - simple fixed limits as requested
                        float z_min = 0.0f;    // User wants Z=0 as bottom
                        float z_max = 280.0f;  // Top limit
                        
                        // ONLY check elbow singularity when going UP
                        // Going DOWN doesn't approach singularity (arm bends more)
                        float elbow_for_limit = local_q(2);
                        
                        // If going UP and elbow is getting too straight, reverse BEFORE flip!
                        if (sweep_direction > 0 && elbow_for_limit > -0.25f && elbow_for_limit < 0.1f) {
                            sweep_direction = -1;
                            last_reverse_time = now;
                            Serial.printf("SWEEP: Elbow near singularity (q3=%.2f), reversing DOWN\n", elbow_for_limit);
                        }
                        
                        if (pos_on_axis > z_max && sweep_direction > 0) {
                            sweep_direction = -1;
                            last_reverse_time = now;
                            Serial.printf("SWEEP: Hit +Z limit (%.1f), reversing\n", pos_on_axis);
                        }
                        if (pos_on_axis < z_min && sweep_direction < 0) {
                            sweep_direction = 1;
                            last_reverse_time = now;
                            Serial.printf("SWEEP: Hit -Z limit (%.1f), reversing\n", pos_on_axis);
                        }
                    }
                }
                
                // Calculate desired Cartesian velocity
                // PRIMARY: Move along sweep axis
                Matrix<3> x_dot = {0, 0, 0};
                float base_velocity = sweep_velocity * sweep_direction * speed_multiplier;
                
                // For Y sweep: reduce velocity near singularity to avoid wiggle
                if (sweep_axis == 1 && manipulability < 500000.0f) {
                    float scale = manipulability / 500000.0f;
                    if (scale < 0.3f) scale = 0.3f;  // Minimum 30% speed
                    base_velocity *= scale;
                }
                
                // For Z sweep: only slow down when going UP and elbow is straightening
                // Going DOWN is fine - arm naturally bends more
                float sweep_q3 = local_q(2);
                if (sweep_axis == 2 && sweep_direction > 0) {
                    // Going UP - check if elbow is getting straight
                    if (sweep_q3 > -0.4f) {
                        float scale = (-sweep_q3) / 0.4f;  // 1.0 at q3=-0.4, 0 at q3=0
                        if (scale < 0.3f) scale = 0.3f;    // Minimum 30% speed
                        base_velocity *= scale;
                    }
                }
                
                // For X sweep at low Z: slow down to maintain stability
                if (sweep_axis == 0) {
                    float z_current = current_fk(2);
                    if (z_current < 170.0f) {
                        // Scale down velocity at low Z
                        float scale = 0.5f + 0.5f * (z_current - 100.0f) / 70.0f;  // 0.5 at z=100, 1.0 at z=170
                        if (scale < 0.5f) scale = 0.5f;
                        base_velocity *= scale;
                    }
                }
                
                x_dot(sweep_axis) = base_velocity;
                
                // SECONDARY: Correct drift on OTHER axes (keep them at start values)
                // Use stronger correction when arm is more extended (lower Z, straighter elbow)
                float base_correction_gain = 5.0f;
                
                // For X sweep: increase Z correction gain when Z is low (arm more extended)
                if (sweep_axis == 0) {
                    float z_target = sweep_start_pos(2);
                    if (z_target < 180.0f) {
                        // Low Z = more extended arm = harder to maintain Z
                        // Increase gain significantly
                        base_correction_gain = 8.0f + (180.0f - z_target) / 20.0f;  // Up to 12 at Z=100
                    }
                }
                // Z sweep needs strong X/Y correction
                if (sweep_axis == 2) {
                    base_correction_gain = 8.0f;
                }
                
                for (int axis = 0; axis < 3; axis++) {
                    if (axis != sweep_axis) {
                        float error = sweep_start_pos(axis) - current_fk(axis);
                        x_dot(axis) = error * base_correction_gain;
                        // Higher limit for correction velocity
                        float max_correction = 80.0f;
                        if (x_dot(axis) > max_correction) x_dot(axis) = max_correction;
                        if (x_dot(axis) < -max_correction) x_dot(axis) = -max_correction;
                    }
                }
                
                // Adaptive damping - MORE aggressive near singularities
                // At low Z, the elbow is straighter so we need more damping
                float damping = 0.05f;
                float sweep_z = current_fk(2);
                
                // Increase base damping when Z is low (arm more extended)
                if (sweep_z < 180.0f) {
                    damping = 0.08f + (180.0f - sweep_z) / 1000.0f;  // Up to ~0.16 at Z=100
                }
                
                // Also increase damping based on manipulability
                if (manipulability < 200.0f) {
                    float ratio = 1.0f - manipulability / 200.0f;
                    damping = fmax(damping, 0.05f + ratio * ratio * 0.5f);  // Up to 0.55 damping
                }
                
                Matrix<4, 3> J_pinv = kinematics.get_jacobian_pinv_dls(J, damping);
                Matrix<4> q_dot = J_pinv * x_dot;
                
                // CRITICAL: Constrain base joint to maintain Y position during X/Z sweeps
                // Calculate target base angle from desired Y and current X
                if (sweep_axis == 0 || sweep_axis == 2) {
                    // For X/Z sweep, base angle should track atan2(Y_target, X_current)
                    // This keeps Y at sweep_start_pos(1) as X changes
                    float target_Y = sweep_start_pos(1);
                    float current_X = current_fk(0);
                    float r_horizontal = sqrt(current_X * current_X + target_Y * target_Y);
                    
                    // SPECIAL CASE: When Y is near 0, keep base at 0 regardless of X sign
                    // This allows smooth crossing through the singularity
                    float target_base_angle = 0.0f;
                    if (fabs(target_Y) < 20.0f) {
                        // Y near 0 - keep base at 0
                        target_base_angle = 0.0f;
                    } else if (r_horizontal > 10.0f) {
                        target_base_angle = atan2(target_Y, current_X);
                    }
                    
                    float base_angle = local_q(0);
                    float base_error = target_base_angle - base_angle;
                    
                    // Wrap angle error to [-pi, pi]
                    while (base_error > M_PI) base_error -= 2.0f * M_PI;
                    while (base_error < -M_PI) base_error += 2.0f * M_PI;
                    
                    // P control to track target base angle
                    float base_correction = base_error * 3.0f;
                    
                    // Blend DLS output with correction (attenuate DLS, add correction)
                    float dls_base_contrib = q_dot(0) * 0.2f;  // Allow 20% of DLS
                    q_dot(0) = dls_base_contrib + base_correction;
                    
                    // Hard limit on base velocity
                    float max_base_vel = 0.5f;  // rad/s
                    if (q_dot(0) > max_base_vel) q_dot(0) = max_base_vel;
                    if (q_dot(0) < -max_base_vel) q_dot(0) = -max_base_vel;
                }
                // For Y sweep, limit base velocity to prevent wild swings
                else if (sweep_axis == 1) {
                    float max_base_vel = 0.5f;
                    if (q_dot(0) > max_base_vel) q_dot(0) = max_base_vel;
                    if (q_dot(0) < -max_base_vel) q_dot(0) = -max_base_vel;
                }
                
                // Z SWEEP: Same base constraint as X sweep, keep it simple
                if (sweep_axis == 2) {
                    // Base should stay nearly fixed during Z sweep
                    float max_base_vel_z = 0.3f;
                    if (q_dot(0) > max_base_vel_z) q_dot(0) = max_base_vel_z;
                    if (q_dot(0) < -max_base_vel_z) q_dot(0) = -max_base_vel_z;
                }
                
                // Null-space control: prefer horizontal gripper (phi = q2_raw + q3 + q4 near 0)
                // phi is the world-frame angle of the gripper: 0 = horizontal, + = up, - = down
                float q2_raw = local_q(1) + M_PI_2;
                float q3 = local_q(2);
                float q4 = local_q(3);
                float phi_current = q2_raw + q3 + q4;  // Current gripper world angle
                float phi_target = 0.0f;  // Horizontal gripper
                float phi_error = phi_target - phi_current;
                
                // Correct q4 to achieve horizontal gripper
                // Use WEAKER correction for Z sweep to avoid weird wrist angles
                float null_gain = (sweep_axis == 2) ? 0.2f : 0.5f;
                q_dot(3) += phi_error * null_gain;
                
                // =====================================================
                // ELBOW SINGULARITY AVOIDANCE - CRITICAL FOR SWEEPS
                // =====================================================
                // The elbow lock singularity occurs when q3 ≈ 0 (arm straight)
                // We MUST keep elbow bent (q3 negative for elbow-up)
                // 
                // Elbow angle meanings:
                //   q3 < 0 : Elbow UP (bent inward) - PREFERRED
                //   q3 = 0 : Arm STRAIGHT - SINGULARITY, AVOID!
                //   q3 > 0 : Elbow DOWN (bent outward) - BAD
                
                float sweep_elbow_angle = local_q(2);
                
                // Target elbow angle: keep arm well-bent
                float target_elbow = -0.5f;  // About -30 degrees, nicely bent
                
                // Soft posture control - gently bias elbow toward target
                // This is NOT emergency correction, just preference
                float elbow_correction = 0.0f;
                bool in_singularity = false;
                
                // CORRECTED: Negative elbow velocity → q3 decreases (elbow UP)
                //            Positive elbow velocity → q3 increases (elbow DOWN)
                
                if (sweep_elbow_angle > 0.0f) {
                    // DANGER: Elbow has flipped DOWN - emergency!
                    in_singularity = true;
                    elbow_correction = -1.5f;  // NEGATIVE = push elbow UP (q3 decreasing)
                    for (int i = 0; i < 4; i++) q_dot(i) *= 0.3f;
                    
                    static unsigned long last_warn = 0;
                    if (now - last_warn > 1000) {
                        Serial.printf("SWEEP: Elbow DOWN! q3=%.2f, fixing\n", sweep_elbow_angle);
                        last_warn = now;
                    }
                } else if (sweep_elbow_angle > -0.15f) {
                    // WARNING: Getting close to straight
                    in_singularity = true;
                    float urgency = (sweep_elbow_angle + 0.15f) / 0.15f;  // 0 at -0.15, 1 at 0
                    elbow_correction = -0.8f * urgency;  // NEGATIVE = push UP
                    for (int i = 0; i < 4; i++) q_dot(i) *= (0.7f - 0.3f * urgency);
                } else if (sweep_elbow_angle > -0.3f) {
                    // CAUTION: Approaching warning zone
                    float urgency = (sweep_elbow_angle + 0.3f) / 0.15f;  // 0 at -0.3, 1 at -0.15
                    elbow_correction = -0.3f * urgency;  // NEGATIVE = gentle push UP
                } else {
                    // SAFE: Gentle posture control toward target (q3 = -0.5)
                    // We want q3 to move toward -0.5
                    // If q3 = -0.8 (too bent), we want q3 to increase toward -0.5 → need POSITIVE velocity
                    // If q3 = -0.35 (too straight), we want q3 to decrease toward -0.5 → need NEGATIVE velocity
                    float elbow_error = target_elbow - sweep_elbow_angle;  // positive if too straight (need to bend more)
                    elbow_correction = elbow_error * 0.3f;  // If too straight, error>0, correction>0... wait
                    // Actually: if q3=-0.35, target=-0.5, error=-0.5-(-0.35)=-0.15, we need q3 to go MORE negative
                    // More negative q3 = negative velocity needed
                    // So elbow_correction = -0.15 * 0.3 = -0.045 → negative → q3 decreases → correct!
                }
                
                q_dot(2) += elbow_correction;
                
                // CRITICAL: Prevent base from crossing limits (cannot wrap 0↔4095)
                q_dot(0) = clampBaseVelocity(local_q(0), q_dot(0), rawPositions[0]);
                
                // Clamp joint velocities
                q_dot = kinematics.clamp_velocities(local_q, q_dot);
                
                // Scale velocities to max joint speed
                // BUT: in singularity, prioritize elbow correction
                float max_qd = 0;
                for (int i = 0; i < 4; i++) {
                    if (fabs(q_dot(i)) > max_qd) max_qd = fabs(q_dot(i));
                }
                float max_joint_speed = 1.5f * speed_multiplier;
                if (in_singularity) {
                    max_joint_speed = 2.5f * speed_multiplier;  // Allow faster elbow recovery
                }
                if (max_qd > max_joint_speed) {
                    float scale = max_joint_speed / max_qd;
                    for (int i = 0; i < 4; i++) q_dot(i) *= scale;
                }
                
                // ACCELERATION LIMITING - prevents jerky motion
                // Limit how fast velocity can change per loop iteration
                static Matrix<4> prev_sweep_q_dot = {0, 0, 0, 0};
                float max_accel = 3.0f * speed_multiplier;  // rad/s per loop (~50Hz = 0.02s, so ~150 rad/s^2)
                for (int i = 0; i < 4; i++) {
                    float delta = q_dot(i) - prev_sweep_q_dot(i);
                    if (delta > max_accel) delta = max_accel;
                    if (delta < -max_accel) delta = -max_accel;
                    q_dot(i) = prev_sweep_q_dot(i) + delta;
                }
                
                // Additional low-pass smoothing on top of accel limiting
                float smooth_alpha = 0.4f;  // 40% new, 60% old
                for (int i = 0; i < 4; i++) {
                    q_dot(i) = smooth_alpha * q_dot(i) + (1.0f - smooth_alpha) * prev_sweep_q_dot(i);
                }
                prev_sweep_q_dot = q_dot;
                
                // Convert to motor speeds (rad/s -> steps/s)
                std::vector<int> speeds;
                const float RAD_TO_STEPS = 4096.0f / (2.0f * M_PI);
                
                speeds.push_back((int)(q_dot(0) * RAD_TO_STEPS));           // Motor 1 (Base)
                int speed1 = (int)(-q_dot(1) * RAD_TO_STEPS);               // Motor 2 (Shoulder) NEGATED
                speeds.push_back(speed1);
                speeds.push_back(-speed1);                                   // Motor 3
                speeds.push_back((int)(q_dot(2) * RAD_TO_STEPS));           // Motor 4 (Elbow)
                speeds.push_back((int)(q_dot(3) * RAD_TO_STEPS));           // Motor 5 (Wrist)
                speeds.push_back(0);                                         // Roll doesn't change
                
                int max_speed = (int)(1500 * speed_multiplier);
                for(int& s : speeds) {
                    if (s > max_speed) s = max_speed;
                    if (s < -max_speed) s = -max_speed;
                }
                
                // Apply minimum speed threshold to avoid motor jitter at low speeds
                // For sweep mode, use joint velocity as error proxy (if we want to move, we should move)
                // Boost low speeds to minimum - in sweep mode we always want motion
                for (int& s : speeds) {
                    int abs_speed = abs(s);
                    if (abs_speed > 0 && abs_speed < MIN_MOTOR_SPEED) {
                        s = (s > 0) ? MIN_MOTOR_SPEED : -MIN_MOTOR_SPEED;  // Boost to minimum
                    }
                }
                
                // Apply backlash compensation
                applyBacklashCompensation(speeds);
                
                servoDriver.syncWriteVelocity(motor_ids, speeds);
            }
            // Cartesian tracking mode - Jacobian-based position control
            else if (cartesian_mode && !moving && !sweep_mode) {
                Matrix<3> current_fk = kinematics.forward_kinematics(local_q);
                
                // Position error
                Matrix<3> pos_error;
                pos_error(0) = cartesian_target(0) - current_fk(0);
                pos_error(1) = cartesian_target(1) - current_fk(1);
                pos_error(2) = cartesian_target(2) - current_fk(2);
                
                float error_norm = sqrt(pos_error(0)*pos_error(0) + pos_error(1)*pos_error(1) + pos_error(2)*pos_error(2));
                
                // Debug output - less frequent during path mode to avoid serial conflicts
                static unsigned long last_debug = 0;
                unsigned long debug_interval = path_mode ? 1000 : 500;  // 1s during path, 500ms otherwise
                if (now - last_debug >= debug_interval) {
                    // Check if any joints are near limits
                    const char* limit_info = "";
                    for (int i = 0; i < 4; i++) {
                        if (local_q(i) >= JOINT_LIMITS[i].max_rad - 0.05f) {
                            Serial.printf("LIMIT J%d at MAX (%.2f >= %.2f)\n", i+1, local_q(i), JOINT_LIMITS[i].max_rad);
                        } else if (local_q(i) <= JOINT_LIMITS[i].min_rad + 0.05f) {
                            Serial.printf("LIMIT J%d at MIN (%.2f <= %.2f)\n", i+1, local_q(i), JOINT_LIMITS[i].min_rad);
                        }
                    }
                    Serial.printf("DBG FK=(%.1f,%.1f,%.1f) TGT=(%.1f,%.1f,%.1f) ERR=%.1f Q=(%.2f,%.2f,%.2f,%.2f)\n",
                        current_fk(0), current_fk(1), current_fk(2),
                        cartesian_target(0), cartesian_target(1), cartesian_target(2),
                        error_norm,
                        local_q(0), local_q(1), local_q(2), local_q(3));
                    last_debug = now;
                }
                
                // If close enough, check if we need to continue to final target
                // Use looser threshold for center_crossing intermediate point
                float done_threshold = center_crossing_mode ? 20.0f : 3.0f;
                if (error_norm < done_threshold) {
                    if (center_crossing_mode) {
                        // We reached the high point, now go to final target
                        cartesian_target(0) = final_target_after_neutral(0);
                        cartesian_target(1) = final_target_after_neutral(1);
                        cartesian_target(2) = final_target_after_neutral(2);
                        move_target_roll = final_roll_after_neutral;
                        center_crossing_mode = false;
                        Serial.printf("OK High point reached, descending to (%.1f, %.1f, %.1f)\n",
                                      final_target_after_neutral(0), final_target_after_neutral(1), final_target_after_neutral(2));
                    } else if (path_mode && path_current_idx < path_count) {
                        // Path mode: check if current waypoint has wait time
                        Waypoint& wp = path_waypoints[path_current_idx];
                        if (wp.wait_time > 0 && !path_waiting) {
                            // Start waiting
                            path_waiting = true;
                            path_wait_start = now;
                            std::vector<int> stops(6, 0);
                            servoDriver.syncWriteVelocity(motor_ids, stops);
                            Serial.printf("PATH Waypoint %d reached, waiting %.1fs\n", path_current_idx + 1, wp.wait_time);
                        } else if (path_waiting) {
                            if (now - path_wait_start >= (unsigned long)(wp.wait_time * 1000)) {
                                path_waiting = false;
                                path_current_idx++;
                                if (path_current_idx < path_count) {
                                    Waypoint& next = path_waypoints[path_current_idx];
                                    cartesian_target(0) = next.x;
                                    cartesian_target(1) = next.y;
                                    cartesian_target(2) = next.z;
                                    move_target_roll = next.roll;
                                    Serial.printf("PATH Moving to waypoint %d (%.1f, %.1f, %.1f)\n", 
                                        path_current_idx + 1, next.x, next.y, next.z);
                                } else {
                                    path_mode = false;
                                    cartesian_mode = false;
                                    std::vector<int> stops(6, 0);
                                    servoDriver.syncWriteVelocity(motor_ids, stops);
                                    Serial.println("PATH_DONE Path completed");
                                }
                            }
                        } else {
                            // No wait time - advance immediately
                            path_current_idx++;
                            if (path_current_idx < path_count) {
                                Waypoint& next = path_waypoints[path_current_idx];
                                cartesian_target(0) = next.x;
                                cartesian_target(1) = next.y;
                                cartesian_target(2) = next.z;
                                move_target_roll = next.roll;
                                Serial.printf("PATH Moving to waypoint %d (%.1f, %.1f, %.1f)\n", 
                                    path_current_idx + 1, next.x, next.y, next.z);
                            } else {
                                path_mode = false;
                                cartesian_mode = false;
                                std::vector<int> stops(6, 0);
                                servoDriver.syncWriteVelocity(motor_ids, stops);
                                Serial.println("PATH_DONE Path completed");
                            }
                        }
                    } else {
                        // Actually done
                        cartesian_mode = false;
                        path_mode = false;
                        std::vector<int> stops(6, 0);
                        servoDriver.syncWriteVelocity(motor_ids, stops);
                        Serial.println("DONE Cartesian target reached");
                    }
                }
                // PATH MODE BLENDING: When close to waypoint with no wait, advance early without stopping
                // Rate limit: only advance one waypoint per 100ms to prevent skipping
                else if (path_mode && path_current_idx < path_count - 1 && error_norm < path_blend_radius) {
                    static unsigned long last_waypoint_advance = 0;
                    if (now - last_waypoint_advance >= 100 && path_waypoints[path_current_idx].wait_time == 0) {
                        // Close enough to current waypoint - advance to next
                        // This allows smooth motion through waypoints without fully stopping
                        path_current_idx++;
                        last_waypoint_advance = now;
                        Waypoint& next_wp = path_waypoints[path_current_idx];
                        
                        // Set target directly to next waypoint
                        cartesian_target(0) = next_wp.x;
                        cartesian_target(1) = next_wp.y;
                        cartesian_target(2) = next_wp.z;
                        move_target_roll = next_wp.roll;
                        
                        Serial.printf("PATH wp%d (%.1f,%.1f,%.1f)\n", 
                            path_current_idx + 1, next_wp.x, next_wp.y, next_wp.z);
                    }
                } else {
                    // =====================================================
                    // JACOBIAN CONTROL WITH POSTURE GUIDANCE
                    // =====================================================
                    // Use Jacobian for smooth Cartesian motion, with null-space
                    // control to maintain proper arm configuration.
                    
                    Matrix<3, 4> J = kinematics.calculate_jacobian(local_q);
                    
                    // Adaptive damping near singularities
                    float damping = kinematics.calculate_adaptive_damping(J, 0.05f, 0.08f);
                    damping = fmax(damping, 0.01f);  // Minimum damping
                    
                    // Get damped pseudoinverse
                    Matrix<4, 3> J_pinv = kinematics.get_jacobian_pinv_dls(J, damping);
                    
                    // Cartesian velocity proportional to error with LINEAR deceleration
                    // This maintains minimum usable motor speeds until very close
                    float cart_gain = 2.5f;  // base gain
                    Matrix<3> x_dot;
                    
                    // LINEAR VELOCITY CURVE with minimum floor
                    // Motors struggle below ~100 steps/s, so maintain usable speeds
                    // At error_norm=50mm: full speed
                    // At error_norm=5mm: minimum speed (20% of full)
                    // At error_norm<2mm: stop
                    float approach_scale = 1.0f;
                    const float MIN_APPROACH_SPEED = 0.2f;  // 20% minimum while moving
                    const float STOP_THRESHOLD = 2.0f;       // Stop when within 2mm
                    const float DECEL_START = 50.0f;         // Start decelerating at 50mm
                    
                    if (error_norm < STOP_THRESHOLD) {
                        approach_scale = 0.0f;  // Close enough - stop
                    } else if (error_norm < DECEL_START) {
                        // Linear ramp from MIN_APPROACH_SPEED at STOP_THRESHOLD to 1.0 at DECEL_START
                        float t = (error_norm - STOP_THRESHOLD) / (DECEL_START - STOP_THRESHOLD);
                        approach_scale = MIN_APPROACH_SPEED + t * (1.0f - MIN_APPROACH_SPEED);
                    }
                    
                    for (int i = 0; i < 3; i++) {
                        x_dot(i) = pos_error(i) * cart_gain * approach_scale;
                    }
                    
                    // Limit Cartesian velocity
                    float x_dot_norm = sqrt(x_dot(0)*x_dot(0) + x_dot(1)*x_dot(1) + x_dot(2)*x_dot(2));
                    float max_cart_vel = 150.0f * speed_multiplier;
                    if (x_dot_norm > max_cart_vel) {
                        for (int i = 0; i < 3; i++) x_dot(i) *= max_cart_vel / x_dot_norm;
                    }
                    
                    // Joint velocities from Jacobian
                    Matrix<4> q_dot = J_pinv * x_dot;
                    
                    // =====================================================
                    // POSTURE CONTROL - SCALED BY DISTANCE TO TARGET
                    // =====================================================
                    // Posture corrections should be strong when far from target
                    // but fade out when close, to avoid fighting position control.
                    
                    // Scale factor: 1.0 when far (>50mm), fades to 0.1 when close (<10mm)
                    float posture_scale = fmin(1.0f, error_norm / 50.0f);
                    if (posture_scale < 0.1f) posture_scale = 0.1f;
                    
                    // =====================================================
                    // ALTERNATIVE CONFIGURATION FOR LOW Z TARGETS
                    // =====================================================
                    // When trying to reach low Z and J3 hits its minimum limit,
                    // guide toward alternative configuration:
                    // - J2 more positive (shoulder tilted back)
                    // - J3 less negative (elbow more extended)
                    // - J4 more negative (wrist bent down to reach low Z)
                    bool j3_at_min_limit = local_q(2) <= JOINT_LIMITS[2].min_rad + 0.1f;
                    bool target_low_z = cartesian_target(2) < 50.0f;
                    bool still_above_target = current_fk(2) > cartesian_target(2) + 5.0f;
                    
                    if (j3_at_min_limit && target_low_z && still_above_target) {
                        // J3 is stuck at limit but we need to go lower
                        // Use alternative configuration: tilt shoulder back, extend elbow, bend wrist down
                        
                        // How much more do we need to drop?
                        float z_gap = current_fk(2) - cartesian_target(2);
                        float urgency = fmin(1.0f, z_gap / 30.0f);
                        
                        // Tilt shoulder back (positive J2) to raise elbow point
                        q_dot(1) += 0.3f * urgency;
                        
                        // Let elbow extend slightly (less negative, toward zero)
                        q_dot(2) += 0.2f * urgency;
                        
                        // Bend wrist DOWN aggressively to reach low Z
                        q_dot(3) -= 0.5f * urgency;
                        
                        static unsigned long last_alt_warn = 0;
                        if (now - last_alt_warn > 1000) {
                            Serial.printf("ALT CONFIG: J3 at limit, using shoulder/wrist to reach Z=%.0f\n", 
                                cartesian_target(2));
                            last_alt_warn = now;
                        }
                    }
                    
                    // --- ELBOW CONTROL ---
                    float cart_elbow_angle = local_q(2);
                    float elbow_correction = 0.0f;
                    
                    // Only apply strong correction if elbow is dangerously straight
                    if (cart_elbow_angle > -0.15f) {
                        // DANGER: Elbow is nearly straight or flipped!
                        float urgency = 1.0f + fmax(0.0f, cart_elbow_angle) * 2.0f;
                        elbow_correction = -0.8f * urgency;  // Strong push to elbow UP
                        
                        // Reduce other velocities to prioritize elbow fix
                        for (int i = 0; i < 4; i++) {
                            if (i != 2) q_dot(i) *= 0.5f;
                        }
                        
                        static unsigned long last_elbow_warn = 0;
                        if (now - last_elbow_warn > 500) {
                            Serial.printf("CART: Elbow danger! q3=%.2f\n", cart_elbow_angle);
                            last_elbow_warn = now;
                        }
                    }
                    // WARNING: Getting close to straight - gentle correction
                    else if (cart_elbow_angle > -0.4f) {
                        float urgency = (cart_elbow_angle + 0.4f) / 0.25f;  // 0 at -0.4, 1 at -0.15
                        elbow_correction = -0.3f * urgency * posture_scale;
                    }
                    // Otherwise: no elbow correction - let Jacobian work
                    
                    q_dot(2) += elbow_correction;
                    
                    // --- WRIST HORIZONTAL CONTROL (very gentle) ---
                    float q2_raw = local_q(1) + M_PI_2;
                    float phi_current = q2_raw + local_q(2) + local_q(3);
                    float phi_target = 0.0f;  // Horizontal
                    
                    // For moves to -X with Y≈0, gripper should point backward
                    bool y_near_zero = fabs(cartesian_target(1)) < 30.0f;
                    if (cartesian_target(0) < -20.0f && y_near_zero) {
                        phi_target = M_PI;
                    }
                    
                    float phi_error = phi_target - phi_current;
                    while (phi_error > M_PI) phi_error -= 2.0f * M_PI;
                    while (phi_error < -M_PI) phi_error += 2.0f * M_PI;
                    
                    // Very gentle wrist correction, scaled down near target
                    q_dot(3) += phi_error * 0.2f * posture_scale;
                    
                    // --- BASE CONTROL for Y≈0 moves (very gentle) ---
                    if (y_near_zero) {
                        float base_error = 0.0f - local_q(0);
                        q_dot(0) += base_error * 0.15f * posture_scale;
                    }
                    
                    // =====================================================
                    // SAFETY FALLBACK: Handle "flip over top" for Y≈0 moves
                    // =====================================================
                    // This is a fallback in case flip_mode wasn't triggered
                    // (e.g., path mode waypoints). Guide elbow-up when crossing.
                    // Note: For single M commands, flip_mode handles this better.
                    
                    if (y_near_zero && fabs(current_fk(0)) > 20.0f) {
                        // Check if we're crossing from +X to -X
                        bool going_to_negative_x = cartesian_target(0) < -20.0f;
                        bool currently_positive_x = current_fk(0) > 20.0f;
                        
                        if (going_to_negative_x && currently_positive_x) {
                            // Need to flip over the top
                            // Scale urgency based on how far we still need to go
                            float flip_urgency = fmin(1.0f, fabs(current_fk(0)) / 80.0f);
                            
                            // Strong elbow-up bias - this is the key to proper flip
                            // The more negative q3, the more "elbow up"
                            float elbow_error = -1.2f - local_q(2);  // Target very negative elbow
                            q_dot(2) += elbow_error * 1.0f * flip_urgency;  // Strong pull to elbow up
                            
                            // Gentle shoulder guidance - don't fight the Jacobian too much
                            if (current_fk(2) < 300.0f) {  // Still need to go higher
                                q_dot(1) += 0.15f * flip_urgency;  // Very gentle lean back
                            }
                        }
                        
                        bool going_to_positive_x = cartesian_target(0) > 20.0f;
                        bool currently_negative_x = current_fk(0) < -20.0f;
                        
                        if (going_to_positive_x && currently_negative_x) {
                            // Need to flip over the top (opposite direction)
                            float flip_urgency = fmin(1.0f, fabs(current_fk(0)) / 80.0f);
                            
                            // Strong elbow-up bias
                            float elbow_error = -1.2f - local_q(2);
                            q_dot(2) += elbow_error * 1.0f * flip_urgency;
                            
                            if (current_fk(2) < 300.0f) {
                                q_dot(1) -= 0.15f * flip_urgency;  // Very gentle lean forward
                            }
                        }
                    }
                    
                    // CRITICAL: Hard limit on base velocity
                    float max_base_vel_cart = 0.8f;
                    if (q_dot(0) > max_base_vel_cart) q_dot(0) = max_base_vel_cart;
                    if (q_dot(0) < -max_base_vel_cart) q_dot(0) = -max_base_vel_cart;
                    
                    // CRITICAL: Prevent base from crossing limits (cannot wrap 0↔4095)
                    q_dot(0) = clampBaseVelocity(local_q(0), q_dot(0), rawPositions[0]);
                    
                    // Clamp joint velocities
                    q_dot = kinematics.clamp_velocities(local_q, q_dot);
                    
                    // Additional velocity limiting
                    float max_joint_vel = 1.2f;
                    for (int i = 0; i < 4; i++) {
                        if (q_dot(i) > max_joint_vel) q_dot(i) = max_joint_vel;
                        if (q_dot(i) < -max_joint_vel) q_dot(i) = -max_joint_vel;
                    }
                    
                    // ACCELERATION LIMITING - prevents jerky motion
                    static Matrix<4> prev_q_dot = {0, 0, 0, 0};
                    float max_accel = 2.0f;  // rad/s per loop
                    for (int i = 0; i < 4; i++) {
                        float delta = q_dot(i) - prev_q_dot(i);
                        if (delta > max_accel) delta = max_accel;
                        if (delta < -max_accel) delta = -max_accel;
                        q_dot(i) = prev_q_dot(i) + delta;
                    }
                    
                    // Low-pass smoothing on top of accel limiting
                    float alpha = 0.4f;
                    for (int i = 0; i < 4; i++) {
                        q_dot(i) = alpha * q_dot(i) + (1.0f - alpha) * prev_q_dot(i);
                    }
                    prev_q_dot = q_dot;
                    
                    // Convert to motor speeds
                    std::vector<int> speeds;
                    const float RAD_TO_STEPS = 4096.0f / (2.0f * M_PI);
                    
                    speeds.push_back((int)(q_dot(0) * RAD_TO_STEPS));  // Motor 1
                    int speed1 = (int)(-q_dot(1) * RAD_TO_STEPS);      // Motor 2 NEGATED
                    speeds.push_back(speed1);
                    speeds.push_back(-speed1);                          // Motor 3
                    speeds.push_back((int)(q_dot(2) * RAD_TO_STEPS));  // Motor 4
                    speeds.push_back((int)(q_dot(3) * RAD_TO_STEPS));  // Motor 5
                    
                    // Roll control
                    float roll_error = move_target_roll - local_roll;
                    speeds.push_back((int)(roll_error * RAD_TO_STEPS * 2.0f));
                    
                    // Clamp speeds
                    for(int& s : speeds) {
                        if (s > 2000) s = 2000;
                        if (s < -2000) s = -2000;
                    }
                    
                    // Boost low speeds to minimum usable level (motors jitter below ~50 steps/s)
                    // Only boost if we're still trying to move (approach_scale > 0)
                    if (approach_scale > 0.0f) {
                        for (int& s : speeds) {
                            int abs_speed = abs(s);
                            if (abs_speed > 0 && abs_speed < MIN_MOTOR_SPEED) {
                                s = (s > 0) ? MIN_MOTOR_SPEED : -MIN_MOTOR_SPEED;
                            }
                        }
                    }
                    
                    applyBacklashCompensation(speeds);
                    servoDriver.syncWriteVelocity(motor_ids, speeds);
                }
            }
            // Trajectory move mode
            else if (moving) {
            if (start_move_time == 0) {
                start_move_time = now;
                // Capture start pose for linear interpolation
                start_pos = kinematics.forward_kinematics(local_q);
                start_roll_val = local_roll;
            }
            
            float t = (now - start_move_time) / 1000.0f;
            float total_time = move_duration_global;
            
            if (t >= total_time) {
                // Movement complete
                moving = false;
                start_move_time = 0;
                
                // Stop all motors
                std::vector<int> stops(6, 0);
                servoDriver.syncWriteVelocity(motor_ids, stops);
                
                // Reset PID integrators
                for (int i = 0; i < 5; i++) {
                    pids[i].reset();
                }
                
                Serial.println("DONE Move Complete");
            } else {
                // Smooth interpolation factor (s-curve)
                float s = t / total_time;
                s = s * s * (3.0f - 2.0f * s); // Smoothstep
                
                Matrix<4> target_q_ik;
                float target_roll_interp;
                
                if (joint_mode) {
                    // Direct joint interpolation
                    for (int i = 0; i < 4; i++) {
                        target_q_ik(i) = move_start_q(i) + s * (move_end_q(i) - move_start_q(i));
                    }
                    target_roll_interp = move_start_roll + s * (move_end_roll - move_start_roll);
                } else {
                    // Cartesian linear interpolation for straight-line motion
                    Matrix<3> interp_pos;
                    interp_pos(0) = start_pos(0) + s * (move_target_x - start_pos(0));
                    interp_pos(1) = start_pos(1) + s * (move_target_y - start_pos(1));
                    interp_pos(2) = start_pos(2) + s * (move_target_z - start_pos(2));
                    
                    target_roll_interp = start_roll_val + s * (move_target_roll - start_roll_val);
                    
                    // Use analytical IK to find joint angles
                    bool ik_ok = kinematics.inverse_kinematics_analytical(
                        interp_pos, local_q, target_wrist_angle, target_q_ik);
                    
                    if (!ik_ok) {
                        // Fallback: use Jacobian-based IK
                        Matrix<3> pos_error;
                        pos_error(0) = interp_pos(0) - kinematics.forward_kinematics(local_q)(0);
                        pos_error(1) = interp_pos(1) - kinematics.forward_kinematics(local_q)(1);
                        pos_error(2) = interp_pos(2) - kinematics.forward_kinematics(local_q)(2);
                        
                        // Jacobian pseudoinverse with DLS
                        Matrix<3, 4> J = kinematics.calculate_jacobian(local_q);
                        float damping = kinematics.calculate_adaptive_damping(J);
                        Matrix<4, 3> J_pinv = kinematics.get_jacobian_pinv_dls(J, damping);
                        
                        // Joint velocity = J_pinv * position_error * gain
                        Matrix<4> dq = J_pinv * pos_error;
                        
                        // Scale and clamp
                        float max_dq = 0.2f; // Max step per iteration
                        for (int i = 0; i < 4; i++) {
                            if (dq(i) > max_dq) dq(i) = max_dq;
                            if (dq(i) < -max_dq) dq(i) = -max_dq;
                        }
                        
                        target_q_ik = kinematics.clamp_joints(local_q + dq);
                    }
                }
                
                // Safety: Clamp Target Angles (including base wrap protection)
                target_q_ik(0) = clampBaseAngle(target_q_ik(0));
                target_q_ik = kinematics.clamp_joints(target_q_ik);
                
                // PID Control to generate velocity commands
                // Note: Only Motor 2 is negated
                std::vector<int> speeds;
                
                // Motor 1 (Base) - not negated, with limit protection
                float out0 = pids[0].compute(target_q_ik(0), local_q(0), dt);
                int base_speed = (int)(out0 * 100.0f);
                base_speed = clampBaseSpeed(local_q(0), base_speed, rawPositions[0]);
                speeds.push_back(base_speed);
                
                // Motor 2 (Shoulder) - NEGATED
                float out1 = pids[1].compute(target_q_ik(1), local_q(1), dt);
                int speed1 = (int)(-out1 * 100.0f);
                speeds.push_back(speed1);      // Motor 2
                speeds.push_back(-speed1);     // Motor 3 (mechanically coupled/inverted)
                
                // Motor 4 (Elbow) - NOT negated
                float out2 = pids[2].compute(target_q_ik(2), local_q(2), dt);
                speeds.push_back((int)(out2 * 100.0f));
                
                // Motor 5 (Wrist Pitch) - NOT negated
                float out3 = pids[3].compute(target_q_ik(3), local_q(3), dt);
                speeds.push_back((int)(out3 * 100.0f));
                
                // Motor 6 (Wrist Roll) - not negated
                float out_roll = pids[4].compute(target_roll_interp, local_roll, dt);
                speeds.push_back((int)(out_roll * 100.0f));
                
                // Clamp All Speeds
                for(int& s : speeds) {
                    if (s > 1000) s = 1000;
                    if (s < -1000) s = -1000;
                }
                
                // Boost low speeds to minimum usable level (motors jitter below ~50 steps/s)
                // In trajectory mode, always boost - the trajectory handles when to stop
                for (int& s : speeds) {
                    int abs_speed = abs(s);
                    if (abs_speed > 0 && abs_speed < MIN_MOTOR_SPEED) {
                        s = (s > 0) ? MIN_MOTOR_SPEED : -MIN_MOTOR_SPEED;
                    }
                }
                
                // Apply backlash compensation
                applyBacklashCompensation(speeds);
                
                servoDriver.syncWriteVelocity(motor_ids, speeds);
            }
            } // End of moving block
        } else if (!torque_enabled && moving) {
            // Stop movement if torque was disabled
            moving = false;
            start_move_time = 0;
        }
        
        // Control loop runs at ~1000Hz (1ms period)
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(921600); // PC communication
    delay(100);
    Serial.println("5DOF Arm Controller Starting...");
    
    // Create mutex for shared state
    stateMutex = xSemaphoreCreateMutex();
    pidMutex = xSemaphoreCreateMutex();
    
    // Initialize servo communication
    servoDriver.begin(1000000); // 1Mbps for STS servos
    delay(100);
    
    // Initialize Motors
    Serial.println("Initializing servos...");
    for (uint8_t id : motor_ids) {
        // Disable torque first to allow mode change
        servoDriver.setTorqueEnable(id, false);
        delay(20);
        
        // Set to Wheel Mode (Velocity Control)
        servoDriver.setWheelMode(id);
        delay(20);
        
        // Enable Torque
        servoDriver.setTorqueEnable(id, true);
        delay(20);
        
        Serial.printf("  Motor %d initialized\n", id);
    }
    
    // Print centers
    Serial.println("Motor centers (from servo_limits.json):");
    for (int i = 0; i < 6; i++) {
        Serial.printf("  Motor %d: center=%d\n", i+1, centers[i]);
    }
    
    // Initial position read
    std::vector<int> positions(6, -1);
    if (readAllPositions(positions)) {
        Serial.println("Initial position read OK");
        for (size_t i = 0; i < positions.size(); i++) {
            Serial.printf("  Motor %d: %d (center=%d, rad=%.3f)\n", 
                motor_ids[i], positions[i], centers[i], rawToRad(positions[i], centers[i]));
        }
    } else {
        Serial.println("WARNING: Initial position read failed!");
    }

    // Create Control Task on Core 1
    xTaskCreatePinnedToCore(
        controlTask,       // Function
        "ControlLoop",     // Name
        8192,              // Stack size (increased for safety)
        NULL,              // Parameters
        2,                 // Priority (higher than default)
        NULL,              // Task handle
        1                  // Core 1 (leave Core 0 for WiFi if needed)
    );
    
    Serial.println("Ready. Commands:");
    Serial.println("  M x y z roll time    - Cartesian move (mm, rad)");
    Serial.println("  J q1 q2 q3 q4 roll t - Joint move (rad, sec)");
    Serial.println("  D q1 q2 q3 q4 roll   - Direct joint target (sliders)");
    Serial.println("  SW axis velocity     - Sweep (0=X,1=Y,2=Z at mm/s)");
    Serial.println("  T 0/1                - Torque off/on");
    Serial.println("  S                    - Stop movement");
}

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd.length() == 0) return;
        
        if (cmd.startsWith("M")) { // Cartesian Move: M x y z roll time
            float x, y, z, roll, t;
            if (sscanf(cmd.c_str(), "M %f %f %f %f %f", &x, &y, &z, &roll, &t) == 5) {
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    // Check if we need to do a flip-over move
                    Matrix<3> current_fk = kinematics.forward_kinematics(current_q);
                    
                    // Calculate base angles for current and target positions
                    float current_base_angle = atan2(current_fk(1), current_fk(0));
                    float target_base_angle = atan2(y, x);
                    
                    // Calculate the angular difference (accounting for wraparound)
                    float angle_diff = target_base_angle - current_base_angle;
                    while (angle_diff > M_PI) angle_diff -= 2.0f * M_PI;
                    while (angle_diff < -M_PI) angle_diff += 2.0f * M_PI;
                    
                    // Check if this requires a flip (>90° base rotation with Y near zero on both ends)
                    bool y_near_zero = fabs(y) < 60.0f && fabs(current_fk(1)) < 60.0f;
                    bool large_rotation = fabs(angle_diff) > M_PI_2 * 0.9f;  // >81° rotation
                    
                    Serial.printf("DBG: FK=(%.1f,%.1f,%.1f) TGT=(%.1f,%.1f,%.1f)\n", 
                        current_fk(0), current_fk(1), current_fk(2), x, y, z);
                    Serial.printf("DBG: base_angles cur=%.2f tgt=%.2f diff=%.2f\n", 
                        current_base_angle, target_base_angle, angle_diff);
                    Serial.printf("DBG: y_near_zero=%d large_rotation=%d\n", y_near_zero, large_rotation);
                    
                    if (y_near_zero && large_rotation) {
                        // FLIP MODE: Instead of rotating base, flip the arm configuration
                        // This gives a smooth, natural motion for +X to -X transitions
                        
                        // Compute the flipped joint configuration
                        Matrix<4> flipped = computeFlippedConfig(current_q, {x, y, z});
                        
                        Serial.printf("DBG: current_q=(%.2f,%.2f,%.2f,%.2f)\n",
                            current_q(0), current_q(1), current_q(2), current_q(3));
                        Serial.printf("DBG: flipped=(%.2f,%.2f,%.2f,%.2f) valid=%d\n",
                            flipped(0), flipped(1), flipped(2), flipped(3), isJointConfigValid(flipped));
                        
                        // Check if flipped config is valid
                        if (isJointConfigValid(flipped)) {
                            // Set up flip interpolation
                            flip_start_q = current_q;
                            flip_target_q = flipped;
                            flip_start_roll = current_roll;
                            flip_target_roll = roll;
                            flip_start_time = 0;  // Sentinel - control task will set actual time
                            
                            // Duration based on how much joints need to move
                            float max_joint_change = 0.0f;
                            for (int i = 0; i < 4; i++) {
                                float change = fabs(flipped(i) - current_q(i));
                                if (change > max_joint_change) max_joint_change = change;
                            }
                            flip_duration = fmax(0.8f, fmin(2.0f, max_joint_change / 1.5f));
                            
                            // Store final Cartesian target - Jacobian takes over after flip
                            flip_final_cartesian(0) = x;
                            flip_final_cartesian(1) = y;
                            flip_final_cartesian(2) = z;
                            flip_final_roll = roll;
                            
                            flip_mode = true;
                            cartesian_mode = false;  // Will be enabled after flip
                            center_crossing_mode = false;
                            moving = false;
                            sweep_mode = false;
                            direct_joint_mode = false;
                            
                            Serial.printf("OK FLIP MODE: Flipping arm (%.1fs) then Jacobian to (%.1f, %.1f, %.1f)\n", 
                                flip_duration, x, y, z);
                            Serial.printf("   Flip q: (%.2f,%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f,%.2f)\n",
                                current_q(0), current_q(1), current_q(2), current_q(3),
                                flipped(0), flipped(1), flipped(2), flipped(3));
                        } else {
                            // Flipped config hits limits - fall back to regular Jacobian
                            Serial.println("WARN: Flip config hits limits, using direct Jacobian");
                            cartesian_target(0) = x;
                            cartesian_target(1) = y;
                            cartesian_target(2) = z;
                            move_target_roll = roll;
                            center_crossing_mode = false;
                            flip_mode = false;
                            cartesian_mode = true;
                            moving = false;
                            sweep_mode = false;
                            direct_joint_mode = false;
                        }
                    } else {
                        // Normal direct move - let Jacobian handle it
                        cartesian_target(0) = x;
                        cartesian_target(1) = y;
                        cartesian_target(2) = z;
                        move_target_roll = roll;
                        center_crossing_mode = false;
                        flip_mode = false;
                        cartesian_mode = true;
                        moving = false;
                        sweep_mode = false;
                        direct_joint_mode = false;
                        Serial.printf("OK Moving to (%.1f, %.1f, %.1f) roll=%.2f\n", x, y, z, roll);
                    }
                    
                    xSemaphoreGive(stateMutex);
                }
            } else {
                Serial.println("ERR: Invalid M command. Use: M x y z roll time");
            }
        } else if (cmd.startsWith("J")) { // Joint Move: J q1 q2 q3 q4 roll time
            float q1, q2, q3, q4, roll, t;
            if (sscanf(cmd.c_str(), "J %f %f %f %f %f %f", &q1, &q2, &q3, &q4, &roll, &t) == 6) {
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    move_start_q = current_q;
                    move_end_q = {q1, q2, q3, q4};
                    move_start_roll = current_roll;
                    move_end_roll = roll;
                    move_duration_global = t;
                    moving = true;
                    joint_mode = true; // Disable IK
                    direct_joint_mode = false;
                    xSemaphoreGive(stateMutex);
                    Serial.printf("OK Joint move to (%.2f, %.2f, %.2f, %.2f, %.2f) in %.2fs\n", 
                        q1, q2, q3, q4, roll, t);
                }
            } else {
                Serial.println("ERR: Invalid J command. Use: J q1 q2 q3 q4 roll time");
            }
        } else if (cmd.startsWith("D")) { // Direct joint target: D q1 q2 q3 q4 roll
            float q1, q2, q3, q4, roll;
            if (sscanf(cmd.c_str(), "D %f %f %f %f %f", &q1, &q2, &q3, &q4, &roll) == 5) {
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    direct_joint_target = {q1, q2, q3, q4};
                    direct_roll_target = roll;
                    direct_joint_mode = true;
                    moving = false;
                    sweep_mode = false;
                    cartesian_mode = false;
                    xSemaphoreGive(stateMutex);
                }
            }
        } else if (cmd.startsWith("PC")) { // Path Clear: PC - clear waypoint queue
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                path_count = 0;
                path_current_idx = 0;
                path_mode = false;
                path_waiting = false;
                xSemaphoreGive(stateMutex);
                Serial.println("OK Path cleared");
            }
        } else if (cmd.startsWith("PA")) { // Path Add: PA x y z roll wait - add waypoint
            float x, y, z, roll, wait;
            if (sscanf(cmd.c_str(), "PA %f %f %f %f %f", &x, &y, &z, &roll, &wait) == 5) {
                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (path_count < MAX_WAYPOINTS) {
                        path_waypoints[path_count].x = x;
                        path_waypoints[path_count].y = y;
                        path_waypoints[path_count].z = z;
                        path_waypoints[path_count].roll = roll;
                        path_waypoints[path_count].wait_time = wait;
                        path_count++;
                        xSemaphoreGive(stateMutex);
                        Serial.printf("OK Added waypoint %d: (%.1f, %.1f, %.1f) roll=%.2f wait=%.1f\n", 
                            path_count, x, y, z, roll, wait);
                    } else {
                        xSemaphoreGive(stateMutex);
                        Serial.println("ERR: Path full (max 32 waypoints)");
                    }
                }
            } else {
                Serial.println("ERR: Invalid PA command. Use: PA x y z roll wait");
            }
        } else if (cmd.startsWith("PR")) { // Path Run: PR - start executing path
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (path_count > 0) {
                    path_current_idx = 0;
                    path_mode = true;
                    path_waiting = false;
                    flip_mode = false;  // Cancel any flip in progress
                    center_crossing_mode = false;
                    
                    // Set first waypoint as target
                    cartesian_target(0) = path_waypoints[0].x;
                    cartesian_target(1) = path_waypoints[0].y;
                    cartesian_target(2) = path_waypoints[0].z;
                    move_target_roll = path_waypoints[0].roll;
                    
                    cartesian_mode = true;
                    moving = false;
                    sweep_mode = false;
                    direct_joint_mode = false;
                    xSemaphoreGive(stateMutex);
                    Serial.printf("OK Running path with %d waypoints\n", path_count);
                } else {
                    xSemaphoreGive(stateMutex);
                    Serial.println("ERR: No waypoints in path");
                }
            }
        } else if (cmd.startsWith("PB")) { // Path Blend radius: PB radius - set blend distance
            float radius;
            if (sscanf(cmd.c_str(), "PB %f", &radius) == 1) {
                if (radius >= 5.0f && radius <= 50.0f) {
                    path_blend_radius = radius;
                    Serial.printf("OK Blend radius set to %.1f mm\n", radius);
                } else {
                    Serial.println("ERR: Blend radius must be 5-50 mm");
                }
            } else {
                Serial.printf("Blend radius: %.1f mm\n", path_blend_radius);
            }
        } else if (cmd.startsWith("SW")) { // Sweep: SW axis velocity (axis: 0=X, 1=Y, 2=Z)
            int axis;
            float vel;
            if (sscanf(cmd.c_str(), "SW %d %f", &axis, &vel) == 2) {
                if (axis >= 0 && axis <= 2) {
                    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        sweep_axis = axis;
                        sweep_velocity = vel;
                        sweep_direction = 1;
                        sweep_initialized = false;  // Reset so it captures start position
                        sweep_mode = true;
                        moving = false;
                        direct_joint_mode = false;
                        cartesian_mode = false;
                        xSemaphoreGive(stateMutex);
                        Serial.printf("OK Sweep axis %d at %.1f mm/s\n", axis, vel);
                    }
                } else {
                    Serial.println("ERR: Axis must be 0(X), 1(Y), or 2(Z)");
                }
            } else {
                Serial.println("ERR: Invalid SW command. Use: SW axis velocity");
            }
        } else if (cmd.startsWith("WL")) { // Wrist Lock toggle
            int lock;
            if (sscanf(cmd.c_str(), "WL %d", &lock) == 1) {
                wrist_locked = (lock != 0);
                Serial.printf("OK Wrist %s\n", wrist_locked ? "LOCKED" : "FREE");
            } else {
                Serial.printf("Wrist mode: %s\n", wrist_locked ? "LOCKED" : "FREE");
            }
        } else if (cmd.startsWith("W")) { // Set wrist angle
            float angle;
            if (sscanf(cmd.c_str(), "W %f", &angle) == 1) {
                target_wrist_angle = angle;
                Serial.printf("OK Wrist angle set to %.3f rad\n", angle);
            } else {
                Serial.println("ERR: Invalid W command. Use: W angle_rad");
            }
        } else if (cmd.startsWith("P")) { // PID Tuning
            int id;
            float kp, ki, kd;
            if (sscanf(cmd.c_str(), "P %d %f %f %f", &id, &kp, &ki, &kd) == 4) {
                if (id >= 1 && id <= 5) {
                    if (xSemaphoreTake(pidMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        pids[id-1].setGains(kp, ki, kd);
                        xSemaphoreGive(pidMutex);
                        Serial.printf("OK PID %d set to Kp=%.2f Ki=%.2f Kd=%.2f\n", id, kp, ki, kd);
                    } else {
                        Serial.printf("ERR: Failed to acquire PID mutex for joint %d\n", id);
                    }
                } else {
                    Serial.println("ERR: Joint ID must be 1-5");
                }
            } else {
                Serial.println("ERR: Invalid P command. Use: P jointId kp ki kd");
            }
        } else if (cmd.startsWith("GP")) { // Get PID values
            int id = 0;
            if (sscanf(cmd.c_str(), "GP %d", &id) == 1 && id >= 1 && id <= 5) {
                float kp, ki, kd;
                pids[id-1].getGains(kp, ki, kd);
                Serial.printf("PID %d: Kp=%.2f Ki=%.2f Kd=%.2f\n", id, kp, ki, kd);
            } else if (cmd == "GP" || cmd == "GP\n" || cmd == "GP\r\n") {
                // Print all PID values
                Serial.println("Current PID values:");
                for (int i = 0; i < 5; i++) {
                    float kp, ki, kd;
                    pids[i].getGains(kp, ki, kd);
                    Serial.printf("  PID %d: Kp=%.2f Ki=%.2f Kd=%.2f\n", i+1, kp, ki, kd);
                }
            } else {
                Serial.println("ERR: Invalid GP command. Use: GP [jointId]");
            }
        } else if (cmd.startsWith("T")) { // Torque Toggle
            int enable;
            if (sscanf(cmd.c_str(), "T %d", &enable) == 1) {
                torque_enabled = (enable != 0);
                for (uint8_t id : motor_ids) {
                    servoDriver.setTorqueEnable(id, torque_enabled);
                    delay(10);
                }
                if (torque_enabled) {
                    Serial.println("OK Torque ON");
                } else {
                    Serial.println("OK Torque OFF");
                    // Reset PIDs when torque is disabled
                    for (int i = 0; i < 5; i++) {
                        pids[i].reset();
                    }
                }
            } else {
                Serial.println("ERR: Invalid T command. Use: T 0 or T 1");
            }
        } else if (cmd.startsWith("C")) { // Set Center
            int id, center;
            if (sscanf(cmd.c_str(), "C %d %d", &id, &center) == 2) {
                if (id >= 1 && id <= 6 && center >= 0 && center <= 4095) {
                    centers[id-1] = center;
                    Serial.printf("OK Center %d set to %d\n", id, center);
                } else {
                    Serial.println("ERR: Motor ID must be 1-6, center 0-4095");
                }
            } else {
                Serial.println("ERR: Invalid C command. Use: C motorId centerValue");
            }
        } else if (cmd.startsWith("SP")) { // Speed multiplier (check BEFORE "S")
            float mult;
            if (sscanf(cmd.c_str(), "SP %f", &mult) == 1) {
                if (mult >= 0.1f && mult <= 3.0f) {
                    speed_multiplier = mult;
                    Serial.printf("OK Speed multiplier set to %.2f\n", mult);
                } else {
                    Serial.println("ERR: Speed must be 0.1 to 3.0");
                }
            } else {
                Serial.printf("Speed multiplier: %.2f\n", speed_multiplier);
            }
        } else if (cmd == "S" || cmd == "S\n" || cmd == "S\r\n" || cmd.startsWith("S ")) { // Stop movement (exact match)
            moving = false;
            sweep_mode = false;
            sweep_initialized = false;
            cartesian_mode = false;
            direct_joint_mode = false;
            path_mode = false;
            path_waiting = false;
            flip_mode = false;  // Cancel flip mode too
            center_crossing_mode = false;
            std::vector<int> stops(6, 0);
            servoDriver.syncWriteVelocity(motor_ids, stops);
            for (int i = 0; i < 5; i++) {
                pids[i].reset();
            }
            Serial.println("OK Stopped");
        } else if (cmd.startsWith("R")) { // Read current position
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                Serial.printf("JOINTS %.4f %.4f %.4f %.4f %.4f\n", 
                    current_q(0), current_q(1), current_q(2), current_q(3), current_roll);
                Serial.printf("CARTESIAN %.2f %.2f %.2f\n", 
                    current_pos(0), current_pos(1), current_pos(2));
                Serial.printf("RAW %d %d %d %d %d %d\n",
                    rawPositions[0], rawPositions[1], rawPositions[2],
                    rawPositions[3], rawPositions[4], rawPositions[5]);
                xSemaphoreGive(stateMutex);
            }
        } else if (cmd.startsWith("?") || cmd.startsWith("H")) { // Help
            Serial.println("Commands:");
            Serial.println("  M x y z roll time    - Cartesian move (mm, rad, sec)");
            Serial.println("  J q1 q2 q3 q4 roll t - Joint move (rad, sec)");
            Serial.println("  D q1 q2 q3 q4 roll   - Direct joint (rad, for sliders)");
            Serial.println("  SW axis velocity     - Sweep axis (0=X,1=Y,2=Z) at mm/s");
            Serial.println("  SP multiplier        - Set speed multiplier (0.1-3.0)");
            Serial.println("  PC                   - Path Clear (reset waypoints)");
            Serial.println("  PA x y z roll wait   - Path Add waypoint (wait=0 for smooth)");
            Serial.println("  PR                   - Path Run (execute path)");
            Serial.println("  PB radius            - Path Blend radius (5-50mm)");
            Serial.println("  W angle              - Set wrist IK angle (rad)");
            Serial.println("  WL 0/1               - Wrist FREE/LOCKED mode");
            Serial.println("  T 0/1                - Torque off/on");
            Serial.println("  P joint kp ki kd     - Set PID gains (joint 1-5)");
            Serial.println("  GP [joint]           - Get PID gains");
            Serial.println("  C motor center       - Set motor center (motor 1-6)");
            Serial.println("  R                    - Read current position");
            Serial.println("  S                    - Stop movement");
            Serial.println("  ?                    - This help");
        } else {
            Serial.println("ERR: Unknown command. Use ? for help.");
        }
    }
    
    delay(1); // Small delay for serial processing
}
