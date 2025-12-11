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
        This Public method runs strict input validation before calling the internal implementation.

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
        
        Raises:
            ValueError: If inputs contain NaNs, Infs, or have incorrect shapes.
        """
        # 1. Runtime Input Validation
        if control_timestep <= 0:
            raise ValueError(f"Control timestep must be positive. Got {control_timestep}")
            
        for name, val in [("cur_pos", cur_pos), ("cur_vel", cur_vel), ("cur_ang_vel", cur_ang_vel), ("target_pos", target_pos)]:
             if not isinstance(val, np.ndarray):
                 raise TypeError(f"{name} must be a numpy array.")
             if val.shape != (3,):
                 raise ValueError(f"{name} must be shape (3,), got {val.shape}")
             if not np.all(np.isfinite(val)):
                 raise ValueError(f"Input {name} contains NaN or Inf.")
                 
        if cur_quat.shape != (4,):
             raise ValueError(f"cur_quat must be shape (4,), got {cur_quat.shape}")
        if not np.all(np.isfinite(cur_quat)):
             raise ValueError("cur_quat contains NaN or Inf.")

        # Handle simplified defaults for subclasses
        if target_vel is None: target_vel = np.zeros(3)
        if target_acc is None: target_acc = np.zeros(3)

        # 2. Delegate to Implementation
        rpm = self._compute_control(
            control_timestep,
            cur_pos,
            cur_quat,
            cur_vel,
            cur_ang_vel,
            target_pos,
            target_vel,
            target_acc,
            target_yaw
        )
        
        # 3. Output Validation
        if not isinstance(rpm, np.ndarray):
             raise TypeError("Controller must return a numpy array.")
        if rpm.shape != (4,):
             raise ValueError(f"Controller output must be shape (4,), got {rpm.shape}")
        
        return rpm

    @abstractmethod
    def _compute_control(self, 
                         control_timestep: float, 
                         cur_pos: np.ndarray, 
                         cur_quat: np.ndarray, 
                         cur_vel: np.ndarray, 
                         cur_ang_vel: np.ndarray, 
                         target_pos: np.ndarray, 
                         target_vel: np.ndarray, 
                         target_acc: np.ndarray,
                         target_yaw: float) -> np.ndarray:
        """
        Internal implementation of the control law.
        Inputs are guaranteed to be valid and non-None.
        """
        pass
