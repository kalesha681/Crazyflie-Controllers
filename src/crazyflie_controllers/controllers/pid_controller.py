import numpy as np
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel
from crazyflie_controllers.controllers.base_controller import BaseController


class PIDController(BaseController):
    """
    PID Controller wrapper around DSLPIDControl.
    """

    def __init__(self, drone_model=DroneModel.CF2X):
        self.ctrl = DSLPIDControl(drone_model=drone_model)
        # Tuning for more aggressive tracking
        # Position gains
        self.ctrl.P_COEFF_FOR = np.array([0.4, 0.4, 1.25])
        self.ctrl.I_COEFF_FOR = np.array([0.05, 0.05, 0.05])
        self.ctrl.D_COEFF_FOR = np.array([0.2, 0.2, 0.5])
        # Attitude gains
        self.ctrl.P_COEFF_TOR = np.array([70000.0, 70000.0, 60000.0])
        self.ctrl.I_COEFF_TOR = np.array([0.0, 0.0, 500.0])
        self.ctrl.D_COEFF_TOR = np.array([20000.0, 20000.0, 12000.0])

    def reset(self):
        self.ctrl.reset()

    def _compute_control(
        self,
        control_timestep: float,
        cur_pos: np.ndarray,
        cur_quat: np.ndarray,
        cur_vel: np.ndarray,
        cur_ang_vel: np.ndarray,
        target_pos: np.ndarray,
        target_vel: np.ndarray,
        target_acc: np.ndarray,
        target_yaw: float,
    ) -> np.ndarray:

        # DSLPIDControl expects target_rpy
        target_rpy = np.array([0.0, 0.0, target_yaw])
        target_rpy_rates = np.zeros(3)

        rpm, _, _ = self.ctrl.computeControl(
            control_timestep=control_timestep,
            cur_pos=cur_pos,
            cur_quat=cur_quat,
            cur_vel=cur_vel,
            cur_ang_vel=cur_ang_vel,
            target_pos=target_pos,
            target_rpy=target_rpy,
            target_vel=target_vel,
            target_rpy_rates=target_rpy_rates,
        )
        return rpm
