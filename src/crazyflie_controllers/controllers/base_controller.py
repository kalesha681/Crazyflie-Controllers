import numpy as np
from abc import ABC, abstractmethod

class BaseController(ABC):
    """
    Abstract Base Class for all controllers.
    Enforces a strict interface for resetting and computing control actions.
    """

    @abstractmethod
    def reset(self):
        """
        Resets the internal state of the controller (e.g., integral errors).
        """
        pass

    @abstractmethod
    def compute_control(self, 
                        control_timestep: float, 
                        cur_pos: np.ndarray, 
                        cur_quat: np.ndarray, 
                        cur_vel: np.ndarray, 
                        cur_ang_vel: np.ndarray, 
                        target_pos: np.ndarray, 
                        target_vel: np.ndarray = None, 
                        target_acc: np.ndarray = None,
                        target_yaw: float = 0.0) -> np.ndarray:
        """
        Computes the control action (RPMs) for the quadrotor.

        Args:
            control_timestep (float): Time step duration (seconds).
            cur_pos (np.ndarray): Current position [x, y, z].
            cur_quat (np.ndarray): Current attitude quaternion [x, y, z, w].
            cur_vel (np.ndarray): Current linear velocity [vx, vy, vz].
            cur_ang_vel (np.ndarray): Current angular velocity [wx, wy, wz].
            target_pos (np.ndarray): Desired position [x, y, z].
            target_vel (np.ndarray, optional): Desired velocity [vx, vy, vz]. Defaults to None.
            target_acc (np.ndarray, optional): Desired acceleration [ax, ay, az]. Defaults to None.

        Returns:
            np.ndarray: Motor RPM commands [m1, m2, m3, m4].
        """
        pass
