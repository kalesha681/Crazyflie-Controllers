from .base_controller import BaseController
import numpy as np
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel

class PIDController(BaseController):
    def __init__(self, drone_model=DroneModel.CF2X):
        self.ctrl = DSLPIDControl(drone_model=drone_model)
        # Tuning for more aggressive tracking
        # Position gains
        self.ctrl.P_COEFF_FOR = np.array([.4, .4, 1.25])
        self.ctrl.I_COEFF_FOR = np.array([.05, .05, .05])
        self.ctrl.D_COEFF_FOR = np.array([.2, .2, .5])
        # Attitude gains
        self.ctrl.P_COEFF_TOR = np.array([70000., 70000., 60000.])
        self.ctrl.I_COEFF_TOR = np.array([.0, .0, 500.])
        self.ctrl.D_COEFF_TOR = np.array([20000., 20000., 12000.])

    def compute_control(self, 
                       control_timestep, 
                       cur_pos, 
                       cur_quat, 
                       cur_vel, 
                       cur_ang_vel, 
                       target_pos, 
                       target_rpy=None, 
                       target_vel=None, 
                       target_rpy_rates=None,
                       target_acc=None):
        
        if target_rpy is None: target_rpy = np.zeros(3)
        if target_vel is None: target_vel = np.zeros(3)
        if target_rpy_rates is None: target_rpy_rates = np.zeros(3)
        
        rpm, _, _ = self.ctrl.computeControl(
            control_timestep=control_timestep,
            cur_pos=cur_pos,
            cur_quat=cur_quat,
            cur_vel=cur_vel,
            cur_ang_vel=cur_ang_vel,
            target_pos=target_pos,
            target_rpy=target_rpy,
            target_vel=target_vel,
            target_rpy_rates=target_rpy_rates
        )
        return rpm
