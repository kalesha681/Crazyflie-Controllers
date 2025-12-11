import numpy as np
from typing import Tuple
from crazyflie_controllers.trajectories.base_trajectory import BaseTrajectory

class StepTrajectory(BaseTrajectory):
    def __init__(self, step_time=1.0, initial_pos=None, final_pos=None):
        self.step_time = step_time
        self.initial_pos = np.array([0.0, 0.0, 1.0]) if initial_pos is None else np.array(initial_pos)
        self.final_pos = np.array([1.0, 0.0, 1.0]) if final_pos is None else np.array(final_pos)
        
    def _get_target(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        vel = np.zeros(3)
        acc = np.zeros(3)
        
        if t < self.step_time:
            return self.initial_pos, vel, acc, 0.0
        else:
            return self.final_pos, vel, acc, 0.0
