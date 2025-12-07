import numpy as np
import time

class TrajectoryGenerator:
    def __init__(self):
        self.start_pos = None
        self.end_pos = None
        self.total_distance = 0.0
        self.start_time = 0.0
        self.duration = 0.0
        self.max_vel = 0.0
        self.max_accel = 0.0
        
        # Trapezoidal profile parameters
        self.t_accel = 0.0
        self.t_flat = 0.0
        self.d_accel = 0.0
        
    def plan_linear_path(self, start_pos, end_pos, max_vel=50.0, max_accel=100.0):
        """
        Plans a linear path in Cartesian space with a trapezoidal velocity profile.
        Units: mm, mm/s, mm/s^2
        """
        self.start_pos = np.array(start_pos)
        self.end_pos = np.array(end_pos)
        self.total_distance = np.linalg.norm(self.end_pos - self.start_pos)
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.start_time = time.time()
        
        if self.total_distance < 1e-3:
            self.duration = 0
            return

        # Calculate profile
        # Time to reach max velocity
        self.t_accel = self.max_vel / self.max_accel
        self.d_accel = 0.5 * self.max_accel * self.t_accel**2
        
        if 2 * self.d_accel > self.total_distance:
            # Triangle profile (cannot reach max vel)
            self.d_accel = self.total_distance / 2
            self.t_accel = np.sqrt(2 * self.d_accel / self.max_accel)
            self.t_flat = 0
            self.max_vel = self.max_accel * self.t_accel # Adjusted max vel
        else:
            # Trapezoidal profile
            d_flat = self.total_distance - 2 * self.d_accel
            self.t_flat = d_flat / self.max_vel
            
        self.duration = 2 * self.t_accel + self.t_flat

    def get_target(self, current_time):
        """
        Returns the target position and velocity at the given time.
        """
        t = current_time - self.start_time
        
        if t >= self.duration:
            return self.end_pos, np.zeros(3), True # Finished
            
        # Calculate distance along path (s) and velocity (v)
        s = 0.0
        v = 0.0
        
        if t < self.t_accel:
            # Acceleration phase
            s = 0.5 * self.max_accel * t**2
            v = self.max_accel * t
        elif t < self.t_accel + self.t_flat:
            # Constant velocity phase
            dt_flat = t - self.t_accel
            s = self.d_accel + self.max_vel * dt_flat
            v = self.max_vel
        else:
            # Deceleration phase
            dt_decel = t - (self.t_accel + self.t_flat)
            t_rem = self.t_accel - dt_decel
            s = self.total_distance - 0.5 * self.max_accel * t_rem**2
            v = self.max_accel * t_rem
            
        # Interpolate position
        direction = (self.end_pos - self.start_pos) / self.total_distance
        target_pos = self.start_pos + direction * s
        target_vel = direction * v
        
        return target_pos, target_vel, False
