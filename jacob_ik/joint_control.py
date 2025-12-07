import time
import numpy as np

class JointController:
    def __init__(self, motor_id, kp=1.0, ki=0.0, kd=0.0, max_vel=2.0, max_accel=5.0):
        self.motor_id = motor_id
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_vel = max_vel
        self.max_accel = max_accel
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.current_vel = 0.0 # Track current velocity for ramping
        self.last_time = time.time()

    def update(self, target_vel, dt):
        """
        Applies acceleration limiting to the target velocity.
        This is used when we are sending velocity commands directly from the IK solver.
        """
        # Clamp target velocity
        target_vel = np.clip(target_vel, -self.max_vel, self.max_vel)
        
        # Calculate max change in velocity allowed
        max_delta_v = self.max_accel * dt
        
        # Ramp velocity
        delta_v = target_vel - self.current_vel
        delta_v = np.clip(delta_v, -max_delta_v, max_delta_v)
        
        self.current_vel += delta_v
        return self.current_vel

    def compute_pid(self, target_pos, current_pos, dt):
        """
        Computes velocity command based on position error (PID).
        Useful for joint-space moves or correcting drift.
        """
        if dt <= 0: return 0.0
        
        error = target_pos - current_pos
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        output_vel = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        self.prev_error = error
        
        # Pass through acceleration limiter
        return self.update(output_vel, dt)

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.current_vel = 0.0
        self.last_time = time.time()
