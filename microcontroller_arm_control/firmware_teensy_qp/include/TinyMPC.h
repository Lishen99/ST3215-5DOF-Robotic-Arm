#ifndef TINY_MPC_H
#define TINY_MPC_H

#include <BasicLinearAlgebra.h>
#include "ArmModel.h"
#include "TinyQP.h"

using namespace BLA;

/**
 * TinyMPC - Model Predictive Control with QP Solver
 * 
 * TWO MODES:
 * 1. Velocity Projection Mode (FAST ~10us):
 *    - Simple constraint projection for Jacobian velocities
 *    - Used during normal motion when far from constraints
 * 
 * 2. QP-MPC Mode (SLOWER ~100-300us):
 *    - Full QP optimization over short horizon
 *    - Terminal state constraints for desired final posture
 *    - Activated near constraints or when posture control needed
 * 
 * Benefits:
 *   - Fast most of the time (velocity projection)
 *   - Precise when needed (QP optimization)
 *   - Terminal posture control (specify desired final joints)
 *   - Smooth constraint handling
 */

// ============================================================================
// Configuration
// ============================================================================

// Prediction horizon for MPC
constexpr int MPC_HORIZON = 3;        // Short horizon for speed (3 steps = ~4ms lookahead)
constexpr float MPC_DT = 0.0015f;     // 1.5ms per step (~667Hz)

// State dimension
constexpr int STATE_DIM = 2 * N_JOINTS;  // 8 (position + velocity)
constexpr int CONTROL_DIM = N_JOINTS;    // 4 (accelerations)

// Tolerance
constexpr float QP_TOL = 1e-4f;

// Control mode selection
enum MPCMode {
    MPC_MODE_PROJECTION = 0,  // Fast projection only
    MPC_MODE_QP = 1,          // Full QP optimization
    MPC_MODE_AUTO = 2         // Automatic switching based on constraints
};

// ============================================================================
// Cost Weights
// ============================================================================

struct MPCWeights {
    // Position tracking weights
    Matrix<N_JOINTS> Q_pos;     // Position error weight
    Matrix<N_JOINTS> Q_vel;     // Velocity error weight
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
    
    // Terminal cost: stop at target (reduced Qf_vel to reduce jitter)
    w.Qf_pos = {200.0f, 200.0f, 200.0f, 100.0f};
    w.Qf_vel = {10.0f, 10.0f, 10.0f, 10.0f};  // Reduced from 50 to reduce stopping jitter
    
    // Control effort: penalize large accelerations
    w.R = {0.5f, 0.5f, 0.5f, 0.5f};  // Increased for smoother motion
    
    // Jerk penalty: smooth acceleration changes
    w.R_delta = {0.1f, 0.1f, 0.1f, 0.1f};  // Increased for smoother motion
    
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
    
    c.enforce_elbow_up = false;  // Disabled: 0 degrees is valid center
    c.elbow_margin = 0.1f;       // Unused when disabled
    
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
    Matrix<N_JOINTS> q_ref;        // Target joint positions
    Matrix<N_JOINTS> q_dot_ref;    // Target velocities (usually zero)
    Matrix<N_JOINTS> q_terminal;   // DESIRED TERMINAL POSTURE (for posture control)
    bool use_terminal_constraint;  // Whether to enforce terminal posture
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
    MPCMode mode_used;                // Which mode was used
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
     * FAST MODE: Project velocities to satisfy constraints (~10us)
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
     * QP MODE: Solve full MPC with terminal posture control (~100-300us)
     * Optimizes trajectory over short horizon with terminal state constraints.
     * Use this when precise posture control is needed or near constraints.
     * 
     * @param state Current arm state (q, q_dot)
     * @param ref Reference with terminal posture goal
     * @param mode Control mode (PROJECTION/QP/AUTO)
     * @return Optimal control and predictions
     */
    MPCSolution solveQP(const MPCState& state, const MPCReference& ref, MPCMode mode = MPC_MODE_AUTO);
    
    /**
     * Legacy: Simple MPC solve (still available for backward compatibility)
     */
    MPCSolution solve(const MPCState& state, const MPCReference& ref);
    
    /**
     * Set control mode
     */
    void setMode(MPCMode mode) { _mode = mode; }
    MPCMode getMode() const { return _mode; }
    
    /**
     * Update weights online
     */
    void setWeights(const MPCWeights& weights);
    
    /**
     * Get current weights for inspection/tuning
     */
    const MPCWeights& getWeights() const { return _weights; }
    
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
    int getLastQPIterations() const { return _last_qp_iters; }
    
    /**
     * Get current constraints for inspection
     */
    const MPCConstraints& getConstraints() const { return _constraints; }
    
private:
    MPCWeights _weights;
    MPCConstraints _constraints;
    MPCMode _mode;
    
    // Warm start from previous solution
    Matrix<N_JOINTS> _u_warm[MPC_HORIZON];
    Matrix<N_JOINTS> _u_prev;  // Previous control for jerk penalty
    
    // Statistics
    float _last_solve_time;
    int _total_iterations;
    int _last_qp_iters;
    
    // Internal QP solver methods
    
    /**
     * Build and solve condensed QP for MPC
     */
    MPCSolution solveCondensedQP(const MPCState& state, const MPCReference& ref);
    
    /**
     * Predict state sequence given control sequence
     */
    void predictTrajectory(
        const Matrix<N_JOINTS>& q0,
        const Matrix<N_JOINTS>& q_dot0,
        const Matrix<N_JOINTS> u[MPC_HORIZON],
        Matrix<N_JOINTS> q[MPC_HORIZON + 1],
        Matrix<N_JOINTS> q_dot[MPC_HORIZON + 1]
    );
    
    /**
     * Compute QP cost for control sequence
     */
    float computeQPCost(
        const Matrix<N_JOINTS> q[MPC_HORIZON + 1],
        const Matrix<N_JOINTS> q_dot[MPC_HORIZON + 1],
        const Matrix<N_JOINTS> u[MPC_HORIZON],
        const MPCReference& ref
    );
    
    /**
     * Compute gradient of cost w.r.t. control sequence using adjoint method
     */
    void computeQPGradient(
        const MPCState& state,
        const Matrix<N_JOINTS> u[MPC_HORIZON],
        const MPCReference& ref,
        Matrix<N_JOINTS> grad[MPC_HORIZON]
    );
    
    /**
     * Project control sequence onto constraints
     */
    void projectControls(
        Matrix<N_JOINTS> u[MPC_HORIZON],
        const Matrix<N_JOINTS>& q0,
        const Matrix<N_JOINTS>& q_dot0
    );
    
    /**
     * Simple constraint projection (fast path)
     */
    Matrix<N_JOINTS> projectToConstraints(const Matrix<N_JOINTS>& u,
                                           const Matrix<N_JOINTS>& q,
                                           const Matrix<N_JOINTS>& q_dot);
    
    /**
     * Compute deceleration factor based on distance to limit
     */
    float computeDecelerationFactor(float pos, float vel, float limit_min, float limit_max, float dt);
};

#endif // TINY_MPC_H
