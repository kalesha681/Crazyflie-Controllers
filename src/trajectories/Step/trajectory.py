import numpy as np
from src.trajectories.base_trajectory import BaseTrajectory

class StepTrajectory(BaseTrajectory):
    def __init__(self, step_time=1.0, initial_pos=None, final_pos=None):
        self.step_time = step_time
        self.initial_pos = np.array([0.0, 0.0, 1.0]) if initial_pos is None else np.array(initial_pos)
        self.final_pos = np.array([1.0, 0.0, 1.0]) if final_pos is None else np.array(final_pos)
        
        # Step magnitude for normalization
        self.step_vector = self.final_pos - self.initial_pos
        self.magnitude = np.linalg.norm(self.step_vector)

    @property
    def characteristic_scale(self):
        return self.magnitude

    def get_target(self, t):
        # Velocity and Accel are zero (Step input is theoretically infinite vel/acc at t0, but zero elsewhere)
        # We provide the Setpoint.
        
        vel = np.zeros(3)
        acc = np.zeros(3)
        
        if t < self.step_time:
            return self.initial_pos, vel, acc
        else:
            return self.final_pos, vel, acc
