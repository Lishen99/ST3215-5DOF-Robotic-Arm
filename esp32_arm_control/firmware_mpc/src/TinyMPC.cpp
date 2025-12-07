#include "TinyMPC.h"
#include <Arduino.h>

/**
 * TinyMPC Implementation
 * 
 * REVISED: Now primarily a constraint projector for the Jacobian controller.
 * The main function is projectVelocity() which takes desired velocities
 * and ensures they satisfy joint limits with smooth deceleration.
 * 
 * The old solve() function is kept for backward compatibility with direct mode.
 */

TinyMPC::TinyMPC() 
    : _last_solve_time(0), _total_iterations(0) {
    // Initialize with defaults
    _weights = getDefaultWeights();
    _constraints = getDefaultConstraints();
    
    // Initialize warm start
    for (int k = 0; k < MPC_HORIZON; k++) {
        for (int i = 0; i < N_JOINTS; i++) {
            _u_warm(i, k) = 0.0f;
        }
    }
    
    buildStateSpaceMatrices();
}

void TinyMPC::init(const MPCWeights& weights, const MPCConstraints& constraints) {
    _weights = weights;
    _constraints = constraints;
    buildStateSpaceMatrices();
}

void TinyMPC::setWeights(const MPCWeights& weights) {
    _weights = weights;
}

void TinyMPC::setConstraints(const MPCConstraints& constraints) {
    _constraints = constraints;
}

void TinyMPC::setElbowUpEnforcement(bool enable, float margin) {
    _constraints.enforce_elbow_up = enable;
    _constraints.elbow_margin = margin;
}

void TinyMPC::buildStateSpaceMatrices() {
    float dt = MPC_DT;
    float dt2_half = 0.5f * dt * dt;
    
    // Initialize to zero
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            m_A(i, j) = 0.0f;
        }
        for (int j = 0; j < CONTROL_DIM; j++) {
            m_B(i, j) = 0.0f;
        }
    }
    
    // Fill A matrix (double integrator)
    for (int i = 0; i < N_JOINTS; i++) {
        m_A(i, i) = 1.0f;
        m_A(i, N_JOINTS + i) = dt;
        m_A(N_JOINTS + i, N_JOINTS + i) = 1.0f;
    }
    
    // Fill B matrix
    for (int i = 0; i < N_JOINTS; i++) {
        m_B(i, i) = dt2_half;
        m_B(N_JOINTS + i, i) = dt;
    }
}

// ============================================================================
// NEW: Velocity Projection (main interface for hybrid architecture)
// ============================================================================

float TinyMPC::computeDecelerationFactor(float pos, float vel, float limit_min, float limit_max, float dt) {
    // Compute a smooth deceleration factor based on distance to limits
    // Returns 1.0 when far from limits, smoothly goes to 0.0 at limits
    
    const float MARGIN = 0.1f;  // Start decelerating 0.1 rad before limit
    const float HARD_MARGIN = 0.02f;  // Hard stop at 0.02 rad
    
    float factor = 1.0f;
    
    // Check upper limit
    if (vel > 0.0f) {
        float dist_to_max = limit_max - pos;
        if (dist_to_max < HARD_MARGIN) {
            factor = 0.0f;  // Hard stop
        } else if (dist_to_max < MARGIN) {
            // Smooth deceleration
            float t = (dist_to_max - HARD_MARGIN) / (MARGIN - HARD_MARGIN);
            factor = fminf(factor, t * t);  // Quadratic slowdown
        }
    }
    
    // Check lower limit  
    if (vel < 0.0f) {
        float dist_to_min = pos - limit_min;
        if (dist_to_min < HARD_MARGIN) {
            factor = 0.0f;  // Hard stop
        } else if (dist_to_min < MARGIN) {
            float t = (dist_to_min - HARD_MARGIN) / (MARGIN - HARD_MARGIN);
            factor = fminf(factor, t * t);
        }
    }
    
    return factor;
}

Matrix<N_JOINTS> TinyMPC::projectVelocity(
    const Matrix<N_JOINTS>& q_dot_desired,
    const Matrix<N_JOINTS>& q,
    float dt)
{
    unsigned long start_time = micros();
    
    Matrix<N_JOINTS> q_dot_proj = q_dot_desired;
    
    // For each joint, apply constraint projection
    for (int i = 0; i < N_JOINTS; i++) {
        // 1. Velocity magnitude limits
        if (q_dot_proj(i) > _constraints.q_dot_max(i)) {
            q_dot_proj(i) = _constraints.q_dot_max(i);
        }
        if (q_dot_proj(i) < _constraints.q_dot_min(i)) {
            q_dot_proj(i) = _constraints.q_dot_min(i);
        }
        
        // 2. Position limit projection with smooth deceleration
        float decel = computeDecelerationFactor(
            q(i), q_dot_proj(i), 
            _constraints.q_min(i), _constraints.q_max(i), 
            dt
        );
        q_dot_proj(i) *= decel;
        
        // 3. If moving toward limit and close, apply hard stop
        float q_next = q(i) + q_dot_proj(i) * dt;
        if (q_next < _constraints.q_min(i) && q_dot_proj(i) < 0) {
            q_dot_proj(i) = 0.0f;
        }
        if (q_next > _constraints.q_max(i) && q_dot_proj(i) > 0) {
            q_dot_proj(i) = 0.0f;
        }
    }
    
    // 4. Elbow-up constraint (J3 = index 2)
    if (_constraints.enforce_elbow_up) {
        float q3 = q(2);
        float q3_dot = q_dot_proj(2);
        float q3_next = q3 + q3_dot * dt;
        
        // q3 should stay negative (elbow up)
        // If approaching q3 = -margin (close to straight), slow down
        float elbow_limit = -_constraints.elbow_margin;
        
        if (q3_dot > 0 && q3 > elbow_limit - 0.2f) {
            // Moving toward singularity
            float dist_to_danger = elbow_limit - q3;  // Negative when past limit
            if (dist_to_danger <= 0) {
                // Already past limit - push back
                q_dot_proj(2) = fminf(q_dot_proj(2), -0.5f);  // Force elbow down
            } else if (dist_to_danger < 0.2f) {
                // Approaching - scale down positive velocity
                float scale = dist_to_danger / 0.2f;
                if (q_dot_proj(2) > 0) {
                    q_dot_proj(2) *= scale * scale;
                }
            }
        }
        
        // Hard block if elbow would go positive (flipped)
        if (q3_next > 0.0f && q3_dot > 0) {
            q_dot_proj(2) = 0.0f;
        }
    }
    
    _last_solve_time = (micros() - start_time) / 1000.0f;
    _total_iterations++;
    
    return q_dot_proj;
}

MPCSolution TinyMPC::solve(const MPCState& state, const MPCReference& ref) {
    unsigned long start_time = micros();
    
    MPCSolution sol;
    sol.feasible = false;
    sol.cost = 999999.0f;
    sol.iterations = 0;
    
    // Current state
    Matrix<N_JOINTS> q = state.q;
    Matrix<N_JOINTS> q_dot = state.q_dot;
    
    // Reference
    Matrix<N_JOINTS> q_ref = ref.q_ref;
    Matrix<N_JOINTS> q_dot_ref = ref.q_dot_ref;
    
    // Simple approach: compute optimal acceleration for one step
    // that moves toward reference while respecting constraints
    
    // Error from reference - HANDLE ANGLE WRAP FOR J1!
    Matrix<N_JOINTS> q_err;
    for (int i = 0; i < N_JOINTS; i++) {
        float err = q_ref(i) - q(i);
        
        // For J1 (base), handle angle wrapping
        // BUT: J1 is NOT continuous (wires), so we should NOT wrap
        // Instead, we should prevent large movements
        if (i == 0) {
            // Clamp J1 error to prevent huge movements
            // Max movement per control step should be small
            if (err > M_PI) err -= 2.0f * M_PI;
            if (err < -M_PI) err += 2.0f * M_PI;
            
            // Additional safety: if error is still large, clamp it
            // This prevents the arm from spinning continuously
            float max_err = 0.5f;  // ~28 degrees max error to track
            if (err > max_err) err = max_err;
            if (err < -max_err) err = -max_err;
        }
        q_err(i) = err;
    }
    Matrix<N_JOINTS> q_dot_err = q_dot_ref - q_dot;
    
    // Desired acceleration: PD-like control on position and velocity
    // u = Kp * (q_ref - q) + Kd * (q_dot_ref - q_dot)
    // Balanced gains for fast but stable motion
    
    Matrix<N_JOINTS> u_desired;
    for (int i = 0; i < N_JOINTS; i++) {
        // Moderate gains - fast but stable
        float kp = _weights.Q_pos(i) * 0.8f;   // Position gain
        float kd = _weights.Q_vel(i) * 1.5f;   // Damping to prevent oscillation
        
        u_desired(i) = kp * q_err(i) + kd * q_dot_err(i);
    }
    
    // Project to constraint set
    Matrix<N_JOINTS> u = projectToConstraints(u_desired, q, q_dot);
    
    // Apply acceleration to predict next state
    float dt = MPC_DT;
    Matrix<N_JOINTS> q_next = q + q_dot * dt + u * (0.5f * dt * dt);
    Matrix<N_JOINTS> q_dot_next = q_dot + u * dt;
    
    // Clamp predictions to limits
    for (int i = 0; i < N_JOINTS; i++) {
        if (q_next(i) < _constraints.q_min(i)) {
            q_next(i) = _constraints.q_min(i);
            if (q_dot_next(i) < 0) q_dot_next(i) = 0;
        }
        if (q_next(i) > _constraints.q_max(i)) {
            q_next(i) = _constraints.q_max(i);
            if (q_dot_next(i) > 0) q_dot_next(i) = 0;
        }
        
        if (q_dot_next(i) < _constraints.q_dot_min(i)) {
            q_dot_next(i) = _constraints.q_dot_min(i);
        }
        if (q_dot_next(i) > _constraints.q_dot_max(i)) {
            q_dot_next(i) = _constraints.q_dot_max(i);
        }
    }
    
    // Compute cost
    float cost = 0.0f;
    for (int i = 0; i < N_JOINTS; i++) {
        cost += _weights.Q_pos(i) * q_err(i) * q_err(i);
        cost += _weights.Q_vel(i) * q_dot_err(i) * q_dot_err(i);
        cost += _weights.R(i) * u(i) * u(i);
    }
    
    // Fill solution
    sol.u = u;
    sol.q_next = q_next;
    sol.q_dot_next = q_dot_next;
    sol.cost = cost;
    sol.iterations = 1;
    sol.feasible = true;
    
    // Update warm start for next solve
    for (int k = 0; k < MPC_HORIZON - 1; k++) {
        for (int i = 0; i < N_JOINTS; i++) {
            _u_warm(i, k) = _u_warm(i, k + 1);
        }
    }
    for (int i = 0; i < N_JOINTS; i++) {
        _u_warm(i, MPC_HORIZON - 1) = u(i);
    }
    
    _last_solve_time = (micros() - start_time) / 1000.0f;  // ms
    _total_iterations++;
    
    return sol;
}

Matrix<N_JOINTS> TinyMPC::projectToConstraints(const Matrix<N_JOINTS>& u,
                                                 const Matrix<N_JOINTS>& q,
                                                 const Matrix<N_JOINTS>& q_dot) {
    Matrix<N_JOINTS> u_proj = u;
    float dt = MPC_DT;
    
    for (int i = 0; i < N_JOINTS; i++) {
        // Acceleration limits
        if (u_proj(i) < _constraints.q_ddot_min(i)) {
            u_proj(i) = _constraints.q_ddot_min(i);
        }
        if (u_proj(i) > _constraints.q_ddot_max(i)) {
            u_proj(i) = _constraints.q_ddot_max(i);
        }
        
        // Velocity limits: check if acceleration would violate velocity limit
        float v_next = q_dot(i) + u_proj(i) * dt;
        if (v_next < _constraints.q_dot_min(i)) {
            // Need to limit acceleration to not exceed velocity limit
            u_proj(i) = (_constraints.q_dot_min(i) - q_dot(i)) / dt;
        }
        if (v_next > _constraints.q_dot_max(i)) {
            u_proj(i) = (_constraints.q_dot_max(i) - q_dot(i)) / dt;
        }
        
        // Position limits: use barrier-like approach
        float q_next = q(i) + q_dot(i) * dt + 0.5f * u_proj(i) * dt * dt;
        
        // If we would hit the min limit, decelerate
        if (q_next < _constraints.q_min(i) + 0.02f) {
            if (q_dot(i) < 0 && u_proj(i) < 0) {
                // Moving toward limit and accelerating toward it - brake!
                u_proj(i) = fmaxf(u_proj(i), -q_dot(i) / dt);
            }
        }
        
        // If we would hit the max limit, decelerate
        if (q_next > _constraints.q_max(i) - 0.02f) {
            if (q_dot(i) > 0 && u_proj(i) > 0) {
                // Moving toward limit and accelerating toward it - brake!
                u_proj(i) = fminf(u_proj(i), -q_dot(i) / dt);
            }
        }
    }
    
    // Elbow-up constraint: keep J3 (index 2) negative
    if (_constraints.enforce_elbow_up) {
        float q3_next = q(2) + q_dot(2) * dt + 0.5f * u_proj(2) * dt * dt;
        
        // If elbow would go above the threshold (toward elbow-down)
        if (q3_next > -_constraints.elbow_margin) {
            // Only allow negative acceleration (pushing elbow back down)
            if (u_proj(2) > 0) {
                u_proj(2) = 0;
            }
            // If already moving up, apply braking
            if (q_dot(2) > 0) {
                u_proj(2) = fminf(u_proj(2), -q_dot(2) / dt);
            }
        }
    }
    
    return u_proj;
}
