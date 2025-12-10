
import numpy as np
from src.trajectories.base_trajectory import BaseTrajectory

class EightTrajectory(BaseTrajectory):
    def __init__(self, scale=1.0, z_height=1.0, period=12.0):
        # A simple figure-8 (Lemniscate of Gerono or similar Lissajous)
        # x = A * sin(omega * t)
        # y = A * sin(2 * omega * t) / 2
        
        self.scale = scale
        self.z = z_height
        self.period = period
        self.omega = 2 * np.pi / period

    def get_target(self, t):
        # Position
        # x(t) = A * sin(w*t)
        # y(t) = (A/2) * sin(2*w*t)
        
        x = self.scale * np.sin(self.omega * t)
        y = (self.scale / 2.0) * np.sin(2 * self.omega * t)
        z = self.z
        
        pos = np.array([x, y, z])
        
        # Velocity (derivative of position)
        # vx(t) = A * w * cos(w*t)
        # vy(t) = A * w * cos(2*w*t)  <-- (A/2) * 2w * cos(...)
        
        vx = self.scale * self.omega * np.cos(self.omega * t)
        vy = self.scale * self.omega * np.cos(2 * self.omega * t)
        vz = 0.0
        
        vel = np.array([vx, vy, vz])
        
        # Acceleration (derivative of velocity)
        # ax(t) = -A * w^2 * sin(w*t)
        # ay(t) = -A * w * 2w * sin(2*w*t) = -2 * A * w^2 * sin(2*w*t)
        
        ax = -self.scale * (self.omega**2) * np.sin(self.omega * t)
        ay = -2.0 * (self.scale/2.0) * (4.0 * self.omega**2) * np.sin(2 * self.omega * t) # Wait, check deriv
        # y = (A/2) * sin(2wt)
        # vy = (A/2) * 2w * cos(2wt) = A*w*cos(2wt) (Matches code line 32)
        # ay = -A*w * 2w * sin(2wt) = -2*A*w^2 * sin(2wt)
        
        ay = -2.0 * self.scale * (self.omega**2) * np.sin(2 * self.omega * t)
        az = 0.0
        
        acc = np.array([ax, ay, az])
        
        return pos, vel, acc
