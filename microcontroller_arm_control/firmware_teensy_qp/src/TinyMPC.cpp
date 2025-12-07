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
    : _last_solve_time(0), _total_iterations(0), _last_qp_iters(0), _mode(MPC_MODE_AUTO) {
    // Initialize with defaults
    _weights = getDefaultWeights();
    _constraints = getDefaultConstraints();
    
    // Initialize warm start
    for (int k = 0; k < MPC_HORIZON; k++) {
        for (int i = 0; i < N_JOINTS; i++) {
            _u_warm[k](i) = 0.0f;
        }
    }
    
    // Initialize previous control
    for (int i = 0; i < N_JOINTS; i++) {
        _u_prev(i) = 0.0f;
    }
}

void TinyMPC::init(const MPCWeights& weights, const MPCConstraints& constraints) {
    _weights = weights;
    _constraints = constraints;
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

// ============================================================================
// Velocity Projection (main interface for hybrid architecture)
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
        _u_warm[k] = _u_warm[k + 1];
    }
    _u_warm[MPC_HORIZON - 1] = u;
    
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

// ============================================================================
// QP-MPC Implementation (Condensed Formulation)
// ============================================================================

void TinyMPC::predictTrajectory(
    const Matrix<N_JOINTS>& q0,
    const Matrix<N_JOINTS>& q_dot0,
    const Matrix<N_JOINTS> U[MPC_HORIZON],
    Matrix<N_JOINTS> Q[MPC_HORIZON+1],
    Matrix<N_JOINTS> Q_dot[MPC_HORIZON+1])
{
    float dt = MPC_DT;
    float dt2_half = 0.5f * dt * dt;
    
    // Initial state
    Q[0] = q0;
    Q_dot[0] = q_dot0;
    
    // Forward simulate double integrator dynamics
    for (int k = 0; k < MPC_HORIZON; k++) {
        Q[k+1] = Q[k] + Q_dot[k] * dt + U[k] * dt2_half;
        Q_dot[k+1] = Q_dot[k] + U[k] * dt;
    }
}

float TinyMPC::computeQPCost(
    const Matrix<N_JOINTS> Q[MPC_HORIZON+1],
    const Matrix<N_JOINTS> Q_dot[MPC_HORIZON+1],
    const Matrix<N_JOINTS> U[MPC_HORIZON],
    const MPCReference& ref)
{
    float cost = 0.0f;
    
    // Running cost
    for (int k = 0; k < MPC_HORIZON; k++) {
        // Position tracking
        for (int i = 0; i < N_JOINTS; i++) {
            float q_err = Q[k](i) - ref.q_ref(i);
            cost += _weights.Q_pos(i) * q_err * q_err;
        }
        
        // Velocity tracking
        for (int i = 0; i < N_JOINTS; i++) {
            float qd_err = Q_dot[k](i) - ref.q_dot_ref(i);
            cost += _weights.Q_vel(i) * qd_err * qd_err;
        }
        
        // Control cost
        for (int i = 0; i < N_JOINTS; i++) {
            cost += _weights.R(i) * U[k](i) * U[k](i);
        }
    }
    
    // Terminal cost - high weight for posture control
    if (ref.use_terminal_constraint) {
        for (int i = 0; i < N_JOINTS; i++) {
            float q_term_err = Q[MPC_HORIZON](i) - ref.q_terminal(i);
            // Use 10x weight for terminal state
            cost += 10.0f * _weights.Q_pos(i) * q_term_err * q_term_err;
        }
    } else {
        // Regular terminal cost
        for (int i = 0; i < N_JOINTS; i++) {
            float q_err = Q[MPC_HORIZON](i) - ref.q_ref(i);
            cost += _weights.Q_pos(i) * q_err * q_err;
        }
    }
    
    return cost;
}

void TinyMPC::computeQPGradient(
    const MPCState& state,
    const Matrix<N_JOINTS> U[MPC_HORIZON],
    const MPCReference& ref,
    Matrix<N_JOINTS> grad_U[MPC_HORIZON])
{
    float dt = MPC_DT;
    float dt2_half = 0.5f * dt * dt;
    
    // Predict trajectory first
    Matrix<N_JOINTS> Q[MPC_HORIZON+1], Q_dot[MPC_HORIZON+1];
    predictTrajectory(state.q, state.q_dot, U, Q, Q_dot);
    
    // Adjoint variables (costate)
    Matrix<N_JOINTS> lambda_q[MPC_HORIZON+1];
    Matrix<N_JOINTS> lambda_qd[MPC_HORIZON+1];
    
    // Terminal condition for adjoint - derivative of terminal cost
    if (ref.use_terminal_constraint) {
        for (int i = 0; i < N_JOINTS; i++) {
            float q_term_err = Q[MPC_HORIZON](i) - ref.q_terminal(i);
            lambda_q[MPC_HORIZON](i) = 2.0f * 10.0f * _weights.Q_pos(i) * q_term_err;
        }
    } else {
        for (int i = 0; i < N_JOINTS; i++) {
            float q_err = Q[MPC_HORIZON](i) - ref.q_ref(i);
            lambda_q[MPC_HORIZON](i) = 2.0f * _weights.Q_pos(i) * q_err;
        }
    }
    
    // Terminal velocity adjoint is zero (no terminal velocity cost)
    for (int i = 0; i < N_JOINTS; i++) {
        lambda_qd[MPC_HORIZON](i) = 0.0f;
    }
    
    // Backward pass - propagate adjoints
    for (int k = MPC_HORIZON - 1; k >= 0; k--) {
        // Stage cost gradients
        Matrix<N_JOINTS> dL_dq, dL_dqd;
        for (int i = 0; i < N_JOINTS; i++) {
            float q_err = Q[k](i) - ref.q_ref(i);
            dL_dq(i) = 2.0f * _weights.Q_pos(i) * q_err;
            
            float qd_err = Q_dot[k](i) - ref.q_dot_ref(i);
            dL_dqd(i) = 2.0f * _weights.Q_vel(i) * qd_err;
        }
        
        // Adjoint update (dynamics transpose)
        // q[k+1] = q[k] + q_dot[k]*dt + u[k]*dt²/2
        // q_dot[k+1] = q_dot[k] + u[k]*dt
        // 
        // lambda_q[k] = dL/dq[k] + lambda_q[k+1] + lambda_qd[k+1]*dt
        // lambda_qd[k] = dL/dqd[k] + lambda_q[k+1]*dt + lambda_qd[k+1]
        
        for (int i = 0; i < N_JOINTS; i++) {
            lambda_q[k](i) = dL_dq(i) + lambda_q[k+1](i) + lambda_qd[k+1](i) * dt;
            lambda_qd[k](i) = dL_dqd(i) + lambda_q[k+1](i) * dt + lambda_qd[k+1](i);
        }
    }
    
    // Gradient of cost w.r.t. controls
    for (int k = 0; k < MPC_HORIZON; k++) {
        for (int i = 0; i < N_JOINTS; i++) {
            // dL/du[k] = R*u[k] + lambda_q[k+1]*dt²/2 + lambda_qd[k+1]*dt
            grad_U[k](i) = 2.0f * _weights.R(i) * U[k](i) 
                         + lambda_q[k+1](i) * dt2_half 
                         + lambda_qd[k+1](i) * dt;
        }
    }
}

void TinyMPC::projectControls(
    Matrix<N_JOINTS> U[MPC_HORIZON],
    const Matrix<N_JOINTS>& q0,
    const Matrix<N_JOINTS>& q_dot0)
{
    float dt = MPC_DT;
    
    // Simple box projection for now - could be improved with trajectory analysis
    for (int k = 0; k < MPC_HORIZON; k++) {
        for (int i = 0; i < N_JOINTS; i++) {
            // Acceleration limits
            if (U[k](i) < _constraints.q_ddot_min(i)) {
                U[k](i) = _constraints.q_ddot_min(i);
            }
            if (U[k](i) > _constraints.q_ddot_max(i)) {
                U[k](i) = _constraints.q_ddot_max(i);
            }
        }
    }
    
    // Forward simulate to check velocity and position limits
    Matrix<N_JOINTS> q = q0;
    Matrix<N_JOINTS> q_dot = q_dot0;
    
    for (int k = 0; k < MPC_HORIZON; k++) {
        // Update state with current control
        Matrix<N_JOINTS> q_next = q + q_dot * dt + U[k] * (0.5f * dt * dt);
        Matrix<N_JOINTS> q_dot_next = q_dot + U[k] * dt;
        
        // Project velocity
        for (int i = 0; i < N_JOINTS; i++) {
            if (q_dot_next(i) < _constraints.q_dot_min(i)) {
                q_dot_next(i) = _constraints.q_dot_min(i);
                // Back-calculate control to match
                U[k](i) = (q_dot_next(i) - q_dot(i)) / dt;
            }
            if (q_dot_next(i) > _constraints.q_dot_max(i)) {
                q_dot_next(i) = _constraints.q_dot_max(i);
                U[k](i) = (q_dot_next(i) - q_dot(i)) / dt;
            }
        }
        
        // Project position
        for (int i = 0; i < N_JOINTS; i++) {
            if (q_next(i) < _constraints.q_min(i)) {
                q_next(i) = _constraints.q_min(i);
                // Stop motion toward limit
                if (q_dot_next(i) < 0) q_dot_next(i) = 0;
                // Back-calculate control
                U[k](i) = (q_dot_next(i) - q_dot(i)) / dt;
            }
            if (q_next(i) > _constraints.q_max(i)) {
                q_next(i) = _constraints.q_max(i);
                if (q_dot_next(i) > 0) q_dot_next(i) = 0;
                U[k](i) = (q_dot_next(i) - q_dot(i)) / dt;
            }
        }
        
        q = q_next;
        q_dot = q_dot_next;
    }
}

MPCSolution TinyMPC::solveCondensedQP(const MPCState& state, const MPCReference& ref) {
    unsigned long start_time = micros();
    
    MPCSolution sol;
    sol.feasible = false;
    sol.iterations = 0;
    
    // Initial control sequence from warm start
    Matrix<N_JOINTS> U[MPC_HORIZON];
    for (int k = 0; k < MPC_HORIZON; k++) {
        U[k] = _u_warm[k];
    }
    
    // Set up QP solver
    const int n_vars = N_JOINTS * MPC_HORIZON;
    
    // Set box constraints
    Matrix<n_vars> lb, ub;
    for (int k = 0; k < MPC_HORIZON; k++) {
        for (int i = 0; i < N_JOINTS; i++) {
            int idx = k * N_JOINTS + i;
            lb(idx) = _constraints.q_ddot_min(i);
            ub(idx) = _constraints.q_ddot_max(i);
        }
    }
    
    // Initial guess from warm start
    Matrix<n_vars> u_vec;
    for (int k = 0; k < MPC_HORIZON; k++) {
        for (int i = 0; i < N_JOINTS; i++) {
            u_vec(k * N_JOINTS + i) = U[k](i);
        }
    }
    
    // Define cost and gradient computation
    auto costFunc = [&](const Matrix<n_vars>& u_flat) -> float {
        // Unpack control sequence
        Matrix<N_JOINTS> U_temp[MPC_HORIZON];
        for (int k = 0; k < MPC_HORIZON; k++) {
            for (int i = 0; i < N_JOINTS; i++) {
                U_temp[k](i) = u_flat(k * N_JOINTS + i);
            }
        }
        
        // Predict trajectory
        Matrix<N_JOINTS> Q[MPC_HORIZON+1], Q_dot[MPC_HORIZON+1];
        predictTrajectory(state.q, state.q_dot, U_temp, Q, Q_dot);
        
        // Compute cost
        return computeQPCost(Q, Q_dot, U_temp, ref);
    };
    
    auto gradFunc = [&](const Matrix<n_vars>& u_flat, Matrix<n_vars>& grad) {
        // Unpack control sequence
        Matrix<N_JOINTS> U_temp[MPC_HORIZON];
        for (int k = 0; k < MPC_HORIZON; k++) {
            for (int i = 0; i < N_JOINTS; i++) {
                U_temp[k](i) = u_flat(k * N_JOINTS + i);
            }
        }
        
        // Compute gradient
        Matrix<N_JOINTS> grad_U[MPC_HORIZON];
        computeQPGradient(state, U_temp, ref, grad_U);
        
        // Pack gradient
        for (int k = 0; k < MPC_HORIZON; k++) {
            for (int i = 0; i < N_JOINTS; i++) {
                grad(k * N_JOINTS + i) = grad_U[k](i);
            }
        }
    };
    
    // Solve QP using TinyQP static solve method
    TinyQP<n_vars> qp;
    Matrix<n_vars> u_opt = u_vec;
    sol.iterations = qp.solveWithCallbacks(u_opt, lb, ub, costFunc, gradFunc, 20);
    
    // Unpack solution
    for (int k = 0; k < MPC_HORIZON; k++) {
        for (int i = 0; i < N_JOINTS; i++) {
            U[k](i) = u_opt(k * N_JOINTS + i);
        }
    }
    
    // Project to constraints
    projectControls(U, state.q, state.q_dot);
    
    // Predict final trajectory
    Matrix<N_JOINTS> Q[MPC_HORIZON+1], Q_dot[MPC_HORIZON+1];
    predictTrajectory(state.q, state.q_dot, U, Q, Q_dot);
    
    // Fill solution with first control
    sol.u = U[0];
    sol.q_next = Q[1];
    sol.q_dot_next = Q_dot[1];
    sol.cost = computeQPCost(Q, Q_dot, U, ref);
    sol.feasible = true;
    
    // Update warm start
    for (int k = 0; k < MPC_HORIZON - 1; k++) {
        _u_warm[k] = U[k + 1];
    }
    _u_warm[MPC_HORIZON - 1] = U[MPC_HORIZON - 1];
    
    _last_solve_time = (micros() - start_time) / 1000.0f;
    _last_qp_iters = sol.iterations;
    
    return sol;
}

MPCSolution TinyMPC::solveQP(const MPCState& state, const MPCReference& ref, MPCMode mode) {
    // Decide whether to use QP or simple projection based on mode
    MPCMode effective_mode = (mode == MPC_MODE_AUTO) ? _mode : mode;
    
    if (effective_mode == MPC_MODE_PROJECTION) {
        // Use existing projection-based solver
        return solve(state, ref);
    } else if (effective_mode == MPC_MODE_QP) {
        // Use full QP solver
        return solveCondensedQP(state, ref);
    } else {
        // AUTO mode - use QP if terminal constraint is active
        if (ref.use_terminal_constraint) {
            return solveCondensedQP(state, ref);
        } else {
            return solve(state, ref);
        }
    }
}
