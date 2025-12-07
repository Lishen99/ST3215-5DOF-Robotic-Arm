#ifndef TINY_MPC_H
#define TINY_MPC_H

#include <BasicLinearAlgebra.h>
#include "ArmModel.h"

using namespace BLA;

/**
 * TinyMPC - Constraint Projection for Velocity Commands
 * 
 * REVISED ARCHITECTURE:
 * The Jacobian controller computes desired joint velocities.
 * TinyMPC projects these velocities to satisfy constraints:
 *   - Joint position limits (with margin)
 *   - Joint velocity limits
 *   - Elbow-up constraint (keep q3 negative)
 * 
 * This is NOT full MPC anymore - it's a fast constraint projector
 * that works with the Jacobian controller for smooth motion.
 * 
 * Benefits:
 *   - Simple, fast, and predictable
 *   - Smooth deceleration at limits
 *   - Respects elbow configuration
 *   - No iterative QP solving needed
 */

// ============================================================================
// Configuration
// ============================================================================

// Prediction horizon for constraint checking
constexpr int MPC_HORIZON = 5;      // Reduced - just for look-ahead
constexpr float MPC_DT = 0.002f;    // 500Hz control rate (fast!)

// State dimension
constexpr int STATE_DIM = 2 * N_JOINTS;  // 8
constexpr int CONTROL_DIM = N_JOINTS;    // 4

// Tolerance
constexpr float QP_TOL = 1e-4f;

// ============================================================================
// Cost Weights
// ============================================================================

struct MPCWeights {
    // Position tracking weights
    Matrix<N_JOINTS> Q_pos;     // Position error weight
    Matrix<N_JOINTS> Q_vel;     // Velocity error weight (usually penalize high velocity)
    Matrix<N_JOINTS> Qf_pos;    // Terminal position weight
    Matrix<N_JOINTS> Qf_vel;    // Terminal velocity weight
    
    // Control effort weights
    Matrix<N_JOINTS> R;         // Acceleration penalty
    Matrix<N_JOINTS> R_delta;   // Jerk penalty (change in acceleration)
};

// Default weights tuned for smooth arm motion
inline MPCWeights getDefaultWeights() {
    MPCWeights w;
    
    // Position tracking: high weight to track reference
    w.Q_pos = {100.0f, 100.0f, 100.0f, 50.0f};
    
    // Velocity: small weight to allow movement
    w.Q_vel = {1.0f, 1.0f, 1.0f, 1.0f};
    
    // Terminal cost: stop at target
    w.Qf_pos = {200.0f, 200.0f, 200.0f, 100.0f};
    w.Qf_vel = {50.0f, 50.0f, 50.0f, 50.0f};
    
    // Control effort: penalize large accelerations
    w.R = {0.1f, 0.1f, 0.1f, 0.1f};
    
    // Jerk penalty: smooth acceleration changes
    w.R_delta = {0.01f, 0.01f, 0.01f, 0.01f};
    
    return w;
}

// ============================================================================
// Constraints
// ============================================================================

struct MPCConstraints {
    // Joint position limits
    Matrix<N_JOINTS> q_min;
    Matrix<N_JOINTS> q_max;
    
    // Joint velocity limits
    Matrix<N_JOINTS> q_dot_min;
    Matrix<N_JOINTS> q_dot_max;
    
    // Acceleration limits (control input)
    Matrix<N_JOINTS> q_ddot_min;
    Matrix<N_JOINTS> q_ddot_max;
    
    // Elbow configuration constraint
    bool enforce_elbow_up;
    float elbow_margin;  // How far from q3=0 to stay
};

inline MPCConstraints getDefaultConstraints() {
    MPCConstraints c;
    
    for (int i = 0; i < N_JOINTS; i++) {
        c.q_min(i) = JOINT_LIMITS[i].min_rad + 0.05f;  // 3° margin
        c.q_max(i) = JOINT_LIMITS[i].max_rad - 0.05f;
        c.q_dot_min(i) = -JOINT_LIMITS[i].max_vel;
        c.q_dot_max(i) = JOINT_LIMITS[i].max_vel;
        c.q_ddot_min(i) = -JOINT_LIMITS[i].max_accel;
        c.q_ddot_max(i) = JOINT_LIMITS[i].max_accel;
    }
    
    c.enforce_elbow_up = true;
    c.elbow_margin = 0.1f;  // Keep J3 < -0.1 rad for elbow-up
    
    return c;
}

// ============================================================================
// MPC State and Reference
// ============================================================================

struct MPCState {
    Matrix<N_JOINTS> q;
    Matrix<N_JOINTS> q_dot;
};

struct MPCReference {
    Matrix<N_JOINTS> q_ref;      // Target joint positions
    Matrix<N_JOINTS> q_dot_ref;  // Target velocities (usually zero)
};

// ============================================================================
// MPC Solution
// ============================================================================

struct MPCSolution {
    Matrix<N_JOINTS> u;              // Optimal control (acceleration)
    Matrix<N_JOINTS> q_next;         // Predicted next position
    Matrix<N_JOINTS> q_dot_next;     // Predicted next velocity
    float cost;                       // Achieved cost
    int iterations;                   // QP iterations used
    bool feasible;                    // True if solution found
};

// ============================================================================
// TinyMPC Solver Class
// ============================================================================

class TinyMPC {
public:
    TinyMPC();
    
    /**
     * Initialize MPC with weights and constraints
     */
    void init(const MPCWeights& weights, const MPCConstraints& constraints);
    
    /**
     * NEW: Project velocities to satisfy constraints
     * This is the main interface for the hybrid Jacobian+MPC architecture.
     * Takes desired velocities from Jacobian controller and ensures they
     * respect all joint limits with smooth deceleration at boundaries.
     * 
     * @param q_dot_desired Desired joint velocities from Jacobian controller
     * @param q Current joint positions
     * @param dt Time step
     * @return Constrained velocities that are safe to execute
     */
    Matrix<N_JOINTS> projectVelocity(
        const Matrix<N_JOINTS>& q_dot_desired,
        const Matrix<N_JOINTS>& q,
        float dt
    );
    
    /**
     * Legacy: Solve MPC for one step (still available for direct mode)
     * @param state Current arm state (q, q_dot)
     * @param ref Reference to track
     * @return Optimal control and predictions
     */
    MPCSolution solve(const MPCState& state, const MPCReference& ref);
    
    /**
     * Update weights online
     */
    void setWeights(const MPCWeights& weights);
    
    /**
     * Update constraints online
     */
    void setConstraints(const MPCConstraints& constraints);
    
    /**
     * Enable/disable elbow-up enforcement
     */
    void setElbowUpEnforcement(bool enable, float margin = 0.1f);
    
    /**
     * Get solve statistics
     */
    float getLastSolveTime() const { return _last_solve_time; }
    int getTotalIterations() const { return _total_iterations; }
    
    /**
     * Get current constraints for inspection
     */
    const MPCConstraints& getConstraints() const { return _constraints; }
    
private:
    MPCWeights _weights;
    MPCConstraints _constraints;
    
    // Pre-computed matrices for efficiency
    Matrix<STATE_DIM, STATE_DIM> m_A;       // State transition
    Matrix<STATE_DIM, CONTROL_DIM> m_B;     // Control input matrix
    
    // QP matrices (condensed form)
    Matrix<N_JOINTS * MPC_HORIZON, N_JOINTS * MPC_HORIZON> _H;  // Hessian
    Matrix<N_JOINTS * MPC_HORIZON> _g;                          // Gradient
    
    // Warm start from previous solution
    Matrix<N_JOINTS, MPC_HORIZON> _u_warm;
    
    // Statistics
    float _last_solve_time;
    int _total_iterations;
    
    // Internal methods
    void buildStateSpaceMatrices();
    void buildQPMatrices(const MPCState& state, const MPCReference& ref);
    Matrix<N_JOINTS> solveQPStep(const Matrix<N_JOINTS>& gradient, 
                                  const Matrix<N_JOINTS>& q_current,
                                  const Matrix<N_JOINTS>& q_dot_current);
    Matrix<N_JOINTS> projectToConstraints(const Matrix<N_JOINTS>& u,
                                           const Matrix<N_JOINTS>& q,
                                           const Matrix<N_JOINTS>& q_dot);
    
    /**
     * Compute deceleration factor based on distance to limit
     * Returns 1.0 when far from limit, 0.0 at limit
     */
    float computeDecelerationFactor(float pos, float vel, float limit_min, float limit_max, float dt);
};

#endif // TINY_MPC_H
