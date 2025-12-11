import numpy as np
from typing import Tuple
from crazyflie_controllers.trajectories.base_trajectory import BaseTrajectory


class CircleTrajectory(BaseTrajectory):
    def __init__(self, radius=0.5, omega=1.0, height=1.0):
        self.radius = radius
        self.omega = omega
        self.height = height

    def _get_target(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        if t < 2.0:
            # Smooth Takeoff: Cubic Ramp (Smoothstep)
            T = 2.0
            t_norm = t / T
            # s(x) = 3x^2 - 2x^3
            s = 3 * t_norm**2 - 2 * t_norm**3
            # v_factor = s'(t) = (6x - 6x^2) / T
            v_spec = (6 * t_norm - 6 * t_norm**2) / T

            # Position Interpolation
            z_ramp = s * self.height
            x_ramp = s * self.radius

            # Velocity Interpolation
            vz_ramp = v_spec * self.height
            vx_ramp = v_spec * self.radius

            return (
                np.array([x_ramp, 0.0, z_ramp]),
                np.array([vx_ramp, 0.0, vz_ramp]),
                np.zeros(3),
                0.0,
            )

        t_flight = t - 2.0

        # Position
        x = self.radius * np.cos(self.omega * t_flight)
        y = self.radius * np.sin(self.omega * t_flight)
        z = self.height

        # Velocity
        vx = -self.radius * self.omega * np.sin(self.omega * t_flight)
        vy = self.radius * self.omega * np.cos(self.omega * t_flight)
        vz = 0.0

        # Acceleration
        ax = -self.radius * (self.omega**2) * np.cos(self.omega * t_flight)
        ay = -self.radius * (self.omega**2) * np.sin(self.omega * t_flight)
        az = 0.0

        return np.array([x, y, z]), np.array([vx, vy, vz]), np.array([ax, ay, az]), 0.0
