from abc import ABC, abstractmethod
import numpy as np

class BaseController(ABC):
    """
    Abstract base class for all Crazyflie controllers.
    """
    
    @abstractmethod
    def compute_control(self, 
                       control_timestep, 
                       cur_pos, 
                       cur_quat, 
                       cur_vel, 
                       cur_ang_vel, 
                       target_pos, 
                       target_rpy=np.zeros(3), 
                       target_vel=np.zeros(3), 
                       target_rpy_rates=np.zeros(3),
                       target_acc=np.zeros(3)):
        """
        Compute the control action (RPMs) for the drone.
        
        Args:
            control_timestep (float): Time step for control.
            cur_pos (np.ndarray): Current position (x, y, z).
            cur_quat (np.ndarray): Current quaternion (x, y, z, w).
            cur_vel (np.ndarray): Current velocity (vx, vy, vz).
            cur_ang_vel (np.ndarray): Current angular velocity (wx, wy, wz).
            target_pos (np.ndarray): Target position (x, y, z).
            target_rpy (np.ndarray, optional): Target roll, pitch, yaw. Defaults to zeros.
            target_vel (np.ndarray, optional): Target velocity. Defaults to zeros.
            target_rpy_rates (np.ndarray, optional): Target angular rates. Defaults to zeros.
            
        Returns:
            np.ndarray: Array of 4 motor RPMs.
            np.ndarray: PWM values (optional, or same as RPMs scaled).
            np.ndarray: Debug info (optional).
        """
        pass
