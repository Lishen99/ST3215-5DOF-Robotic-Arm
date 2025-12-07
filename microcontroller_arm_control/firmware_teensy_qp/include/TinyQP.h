#ifndef TINY_QP_H
#define TINY_QP_H

#include <BasicLinearAlgebra.h>
#include "ArmModel.h"

using namespace BLA;

/**
 * TinyQP - Lightweight Quadratic Programming Solver
 * 
 * Solves QP problems of the form:
 *   minimize:   0.5 * x' * H * x + g' * x
 *   subject to: lb <= x <= ub
 * 
 * Uses Fast Projected Gradient Descent with warm-starting.
 * Optimized for real-time MPC on embedded systems.
 */

constexpr int QP_MAX_ITER = 20;        // Max iterations for QP
constexpr float QP_TOLERANCE = 1e-4f;   // Convergence tolerance

template<int N>
class TinyQP {
public:
    /**
     * Solve QP problem with box constraints
     * 
     * @param H Hessian matrix (N x N, symmetric positive definite)
     * @param g Gradient vector (N)
     * @param lb Lower bounds (N)
     * @param ub Upper bounds (N)
     * @param x_init Initial guess (warm start)
     * @param x_sol Solution output
     * @return Number of iterations taken (0 if already converged)
     */
    static int solve(
        const Matrix<N, N>& H,
        const Matrix<N>& g,
        const Matrix<N>& lb,
        const Matrix<N>& ub,
        const Matrix<N>& x_init,
        Matrix<N>& x_sol,
        int max_iter = QP_MAX_ITER
    ) {
        // Start from warm-start guess
        x_sol = x_init;
        
        // Compute initial gradient: grad = H * x + g
        Matrix<N> grad = H * x_sol + g;
        
        // Armijo backtracking line search parameters
        const float ALPHA = 0.3f;   // Step size reduction factor
        const float BETA = 0.8f;    // Armijo constant
        float step_size = 1.0f;     // Initial step size
        
        for (int iter = 0; iter < max_iter; iter++) {
            // Compute gradient: grad = H * x + g
            grad = H * x_sol + g;
            
            // Check convergence: ||grad||^2 for active set
            float grad_norm_sq = 0.0f;
            for (int i = 0; i < N; i++) {
                // Only count gradient if not at a constraint
                if ((x_sol(i) > lb(i) + 1e-6f && x_sol(i) < ub(i) - 1e-6f) ||
                    (x_sol(i) <= lb(i) + 1e-6f && grad(i) < 0) ||
                    (x_sol(i) >= ub(i) - 1e-6f && grad(i) > 0)) {
                    grad_norm_sq += grad(i) * grad(i);
                }
            }
            
            if (grad_norm_sq < QP_TOLERANCE * QP_TOLERANCE) {
                return iter;  // Converged
            }
            
            // Projected gradient descent with Armijo line search
            Matrix<N> x_new;
            float f_old = computeCost(H, g, x_sol);
            
            // Try step with backtracking
            for (int ls = 0; ls < 10; ls++) {
                // Take gradient step with projection
                for (int i = 0; i < N; i++) {
                    float x_trial = x_sol(i) - step_size * grad(i);
                    // Project onto box constraints
                    x_new(i) = fmaxf(lb(i), fminf(ub(i), x_trial));
                }
                
                // Check Armijo condition
                float f_new = computeCost(H, g, x_new);
                Matrix<N> dx = x_new - x_sol;
                float directional_deriv = 0.0f;
                for (int i = 0; i < N; i++) {
                    directional_deriv += grad(i) * dx(i);
                }
                
                if (f_new <= f_old + BETA * directional_deriv) {
                    // Accept step
                    x_sol = x_new;
                    step_size = fminf(2.0f, step_size / ALPHA);  // Increase for next iter
                    break;
                } else {
                    // Reduce step size
                    step_size *= ALPHA;
                }
            }
        }
        
        return max_iter;  // Hit max iterations
    }
    
    /**
     * Solve QP with callback functions for cost and gradient
     * Returns number of iterations taken
     */
    template<typename CostFunc, typename GradFunc>
    int solveWithCallbacks(
        Matrix<N>& x,
        const Matrix<N>& lb,
        const Matrix<N>& ub,
        CostFunc costFunc,
        GradFunc gradFunc,
        int max_iters = 20)
    {
        const float alpha_init = 1.0f;
        const float beta = 0.5f;
        const float c1 = 1e-4f;
        const float tol = 1e-6f;
        
        int iter = 0;
        Matrix<N> grad, x_new;
        
        for (iter = 0; iter < max_iters; iter++) {
            // Compute gradient
            gradFunc(x, grad);
            
            // Check convergence
            float grad_norm = 0.0f;
            for (int i = 0; i < N; i++) {
                grad_norm += grad(i) * grad(i);
            }
            if (grad_norm < tol * tol) {
                break;
            }
            
            // Line search with Armijo condition
            float alpha = alpha_init;
            float f_x = costFunc(x);
            
            for (int ls_iter = 0; ls_iter < 10; ls_iter++) {
                // Take step and project
                for (int i = 0; i < N; i++) {
                    x_new(i) = x(i) - alpha * grad(i);
                    if (x_new(i) < lb(i)) x_new(i) = lb(i);
                    if (x_new(i) > ub(i)) x_new(i) = ub(i);
                }
                
                float f_new = costFunc(x_new);
                
                // Check Armijo condition
                float expected_decrease = 0.0f;
                for (int i = 0; i < N; i++) {
                    expected_decrease += grad(i) * (x_new(i) - x(i));
                }
                
                if (f_new <= f_x + c1 * expected_decrease) {
                    x = x_new;
                    break;
                }
                
                alpha *= beta;
            }
        }
        
        return iter;
    }

private:
    /**
     * Compute QP cost: 0.5 * x' * H * x + g' * x
     */
    static float computeCost(const Matrix<N, N>& H, const Matrix<N>& g, const Matrix<N>& x) {
        float cost = 0.0f;
        
        // x' * H * x term
        Matrix<N> Hx = H * x;
        for (int i = 0; i < N; i++) {
            cost += 0.5f * x(i) * Hx(i);
        }
        
        // g' * x term
        for (int i = 0; i < N; i++) {
            cost += g(i) * x(i);
        }
        
        return cost;
    }
};

#endif // TINY_QP_H
