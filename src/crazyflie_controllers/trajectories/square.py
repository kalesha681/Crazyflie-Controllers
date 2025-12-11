import numpy as np
from typing import Tuple
from crazyflie_controllers.trajectories.base_trajectory import BaseTrajectory

class SquareTrajectory(BaseTrajectory):
    def __init__(self, side_length=1.0, height=1.0, speed=0.5):
        self.side_length = side_length
        self.height = height
        # Derived: Period = perimeter / speed
        # Perimeter = 4 * side_length
        self.period = (4 * side_length) / speed 
        self.time_per_side = self.period / 4.0

    def _quintic_scaling(self, t_norm):
        # s = 10t^3 - 15t^4 + 6t^5
        t2 = t_norm * t_norm
        t3 = t2 * t_norm
        s = 10 * t3 - 15 * t2 * t2 + 6 * t2 * t3
        ds = 30 * t2 - 60 * t3 + 30 * t2 * t2
        dds = 60 * t_norm - 180 * t2 + 120 * t3
        return s, ds, dds

    def _get_target(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        if t < 2.0: 
            # Smooth Takeoff: Cubic Ramp
            T = 2.0
            t_norm = t / T
            s = 3 * t_norm**2 - 2 * t_norm**3
            v_spec = (6 * t_norm - 6 * t_norm**2) / T
            a_spec = (6 - 12 * t_norm) / (T**2)
            
            z_ramp = s * self.height
            vz_ramp = v_spec * self.height
            az_ramp = a_spec * self.height
            
            return np.array([0.0, 0.0, z_ramp]), np.array([0.0, 0.0, vz_ramp]), np.array([0.0, 0.0, az_ramp]), 0.0
        
        t_flight = t - 2.0
        stage = (t_flight % self.period) / self.time_per_side
        
        pos = np.array([0.0, 0.0, self.height])
        vel = np.array([0.0, 0.0, 0.0])
        acc = np.array([0.0, 0.0, 0.0])
        
        # Vertices for Square (CCW) starting at 0,0
        p0 = np.array([0.0, 0.0])
        p1 = np.array([self.side_length, 0.0])
        p2 = np.array([self.side_length, self.side_length])
        p3 = np.array([0.0, self.side_length])
        
        if stage < 1: # Leg 1: p0 -> p1
            start, end = p0, p1
            progress = stage
        elif stage < 2: # Leg 2: p1 -> p2
            start, end = p1, p2
            progress = stage - 1.0
        elif stage < 3: # Leg 3: p2 -> p3
            start, end = p2, p3
            progress = stage - 2.0
        else: # Leg 4: p3 -> p0
            start, end = p3, p0
            progress = stage - 3.0
            
        # Apply quintic scaling
        s, ds, dds = self._quintic_scaling(progress)
        
        # Position interpolation
        diff = end - start
        pos[:2] = start + s * diff
        
        # Velocity
        vel[:2] = diff * ds / self.time_per_side
        
        # Acceleration
        acc[:2] = diff * dds / (self.time_per_side**2)
            
        return pos, vel, acc, 0.0
