import numpy as np
from abc import ABC, abstractmethod
from typing import Tuple

class BaseTrajectory(ABC):
    """
    Abstract Base Class for all trajectories.
    Trajectories must be pure functions of time and independent of controllers.
    """

    @abstractmethod
    def get_target(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        """
        Returns the target state at time t.

        Args:
            t (float): Current simulation time.

        Returns:
            Tuple containing:
            - pos (np.ndarray): [x, y, z] position.
            - vel (np.ndarray): [vx, vy, vz] velocity.
            - acc (np.ndarray): [ax, ay, az] acceleration.
            - yaw (float): Desired yaw angle (radians).
        """
        pass
