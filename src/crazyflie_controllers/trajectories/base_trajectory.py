import numpy as np
from abc import ABC, abstractmethod
from typing import Tuple

class BaseTrajectory(ABC):
    """
    Abstract Base Class for all trajectories.
    Trajectories must be pure functions of time and independent of controllers.
    """

    def get_target(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        """
        Returns the target state at time t.
        This Public method runs strict output validation on the trajectory.

        Args:
            t (float): Current simulation time.

        Returns:
            Tuple containing:
            - pos (np.ndarray): [x, y, z] position.
            - vel (np.ndarray): [vx, vy, vz] velocity.
            - acc (np.ndarray): [ax, ay, az] acceleration.
            - yaw (float): Desired yaw angle (radians).
            
        Raises:
            ValueError: If outputs have incorrect shapes or types.
        """
        # Delegate
        pos, vel, acc, yaw = self._get_target(t)
        
        # Runtime Output Validation
        for name, val in [("pos", pos), ("vel", vel), ("acc", acc)]:
            if not isinstance(val, np.ndarray):
                raise TypeError(f"Trajectory output {name} must be a numpy array.")
            if val.shape != (3,):
                raise ValueError(f"Trajectory output {name} must be shape (3,), got {val.shape}")
            if not np.all(np.isfinite(val)):
                raise ValueError(f"Trajectory output {name} contains NaN or Inf.")
                
        if not isinstance(yaw, float):
             # Try formatting if it's a numpy scalar
             try:
                 yaw = float(yaw)
             except:
                 raise TypeError(f"Trajectory output yaw must be a float, got {type(yaw)}")

        return pos, vel, acc, yaw

    @abstractmethod
    def _get_target(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        """
        Internal implementation of the trajectory.
        """
        pass
