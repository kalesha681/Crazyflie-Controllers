import numpy as np
from typing import Tuple
from crazyflie_controllers.trajectories.base_trajectory import BaseTrajectory


class EightTrajectory(BaseTrajectory):
    def __init__(self, scale=1.0, z_height=1.0, period=12.0):
        # A simple figure-8 (Lemniscate of Gerono or similar Lissajous)
        # x = A * sin(omega * t)
        # y = A * sin(2 * omega * t) / 2

        self.scale = scale
        self.z = z_height
        self.period = period
        self.omega = 2 * np.pi / period

    def _get_target(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        # Position
        x = self.scale * np.sin(self.omega * t)
        y = (self.scale / 2.0) * np.sin(2 * self.omega * t)
        z = self.z

        pos = np.array([x, y, z])

        # Velocity
        vx = self.scale * self.omega * np.cos(self.omega * t)
        vy = self.scale * self.omega * np.cos(2 * self.omega * t)
        vz = 0.0

        vel = np.array([vx, vy, vz])

        # Acceleration
        ax = -self.scale * (self.omega**2) * np.sin(self.omega * t)
        ay = -2.0 * self.scale * (self.omega**2) * np.sin(2 * self.omega * t)
        az = 0.0

        acc = np.array([ax, ay, az])

        # Determine Yaw to face direction of travel (optional, but good for cameras)
        # For now, keep yaw=0 for fair comparison with legacy code, unless requested.
        # But BaseTrajectory expects a float yaw return.

        return pos, vel, acc, 0.0
