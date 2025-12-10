from abc import ABC, abstractmethod
import numpy as np

class BaseTrajectory(ABC):
    """
    Abstract base class for trajectory generation.
    """
    
    @abstractmethod
    def get_target(self, t):
        """
        Get the target state at time t.
        
        Args:
            t (float): Current time in seconds.
            
        Returns:
            np.ndarray: Target position (x, y, z).
            np.ndarray: Target velocity (vx, vy, vz).
            np.ndarray: Target acceleration (ax, ay, az) (optional/zeros if unused).
        """
        pass

    @property
    def characteristic_scale(self):
        """Returns the characteristic scale of the trajectory (e.g. radius, side length) in meters."""
        return 1.0
