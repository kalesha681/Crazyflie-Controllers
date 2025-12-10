import numpy as np
from src.trajectories.base_trajectory import BaseTrajectory

class TriangleTrajectory(BaseTrajectory):
    def __init__(self, height=1.0, period=6.0):
        self.height = height
        self.period = period
        self.time_per_leg = period / 3.0

    @property
    def characteristic_scale(self):
        # Approximate scale as height or max width (~1.0 for vertices 0,0 -> 1,0.5)
        return 1.0

    def _quintic_scaling(self, t_norm):
        # s = 10t^3 - 15t^4 + 6t^5
        t2 = t_norm * t_norm
        t3 = t2 * t_norm
        s = 10 * t3 - 15 * t2 * t2 + 6 * t2 * t3
        ds = 30 * t2 - 60 * t3 + 30 * t2 * t2
        dds = 60 * t_norm - 180 * t2 + 120 * t3
        return s, ds, dds

    def get_target(self, t):
        if t < 2.0: 
            # Smooth Takeoff
            T = 2.0
            t_norm = t / T
            s, ds, dds = self._quintic_scaling(t_norm)
            
            z_ramp = s * self.height
            vz_ramp = ds * self.height / T
            az_ramp = dds * self.height / (T**2)
            
            return np.array([0.0, 0.0, z_ramp]), np.array([0.0, 0.0, vz_ramp]), np.array([0.0, 0.0, az_ramp])
        
        t_flight = t - 2.0
        stage = (t_flight % self.period) / self.time_per_leg
        
        pos = np.array([0.0, 0.0, self.height])
        vel = np.array([0.0, 0.0, 0.0])
        acc = np.array([0.0, 0.0, 0.0])
        
        # Vertices: (0,0) -> (1, 0.5) -> (1, -0.5) -> (0,0)
        p0 = np.array([0.0, 0.0])
        p1 = np.array([1.0, 0.5])
        p2 = np.array([1.0, -0.5])
        
        # Determine current leg
        if stage < 1: 
            # Leg 1: p0 -> p1
            start, end = p0, p1
            progress = stage
        elif stage < 2: 
            # Leg 2: p1 -> p2
            start, end = p1, p2
            progress = stage - 1.0
        else:           
            # Leg 3: p2 -> p0
            start, end = p2, p0
            progress = stage - 2.0
            
        # Apply quintic scaling
        s, ds, dds = self._quintic_scaling(progress)
        
        # Position
        pos[:2] = start + s * (end - start)
        
        # Velocity (chain rule: d/dt = d/dtau * dtau/dt, where tau is progress)
        # dtau/dt = 1 / time_per_leg
        vel[:2] = (end - start) * ds / self.time_per_leg
        
        # Acceleration
        acc[:2] = (end - start) * dds / (self.time_per_leg**2)
            
        return pos, vel, acc
