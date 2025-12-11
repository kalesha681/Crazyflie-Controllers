import numpy as np
from gym_pybullet_drones.utils.enums import DroneModel
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from scipy.spatial.transform import Rotation
from crazyflie_controllers.controllers.base_controller import BaseController


class SMCController(BaseController):
    """
    Sliding Mode Controller (SMC) for robust trajectory tracking.
    """

    def __init__(self, drone_model=DroneModel.CF2X):
        self.drone_model = drone_model
        # Initialize DSLPID for Attitude Loop
        self.base_ctrl = DSLPIDControl(drone_model=drone_model)

        # Physical Constants
        self.m = 0.027  # kg
        self.g = 9.81  # m/s^2
        self.kf = 3.16e-10  # Propeller thrust coefficient

        # SMC Gains (Position)
        self.lambda_x = 8.0
        self.lambda_y = 8.0
        self.lambda_z = 10.0

        self.k_x = 5.0
        self.k_y = 5.0
        self.k_z = 10.0

        self.PWM2RPM_SCALE = 0.2685

    def reset(self):
        self.base_ctrl.reset()

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

        # --- 1. Position Control (SMC Outer Loop) ---
        e_p = target_pos - cur_pos
        e_v = target_vel - cur_vel

        s_x = e_v[0] + self.lambda_x * e_p[0]
        s_y = e_v[1] + self.lambda_y * e_p[1]
        s_z = e_v[2] + self.lambda_z * e_p[2]

        u_x = self.k_x * np.tanh(s_x)
        u_y = self.k_y * np.tanh(s_y)
        u_z = self.k_z * np.tanh(s_z)

        des_acc_x = target_acc[0] + self.lambda_x * e_v[0] + u_x
        des_acc_y = target_acc[1] + self.lambda_y * e_v[1] + u_y
        des_acc_z = target_acc[2] + self.lambda_z * e_v[2] + u_z + self.g

        # Compute Desired Thrust (RPM)
        total_thrust_force = self.m * np.linalg.norm([des_acc_x, des_acc_y, des_acc_z])
        target_thrust_rpm = np.sqrt(total_thrust_force / (4 * self.kf))

        # Convert RPM to PWM
        target_thrust_pwm = (
            target_thrust_rpm - self.base_ctrl.PWM2RPM_CONST
        ) / self.base_ctrl.PWM2RPM_SCALE
        target_thrust_pwm = np.clip(target_thrust_pwm, 0, 65535)

        # --- Geometric Attitude Control ---
        target_z_ax = np.array([des_acc_x, des_acc_y, des_acc_z])
        norm_z = np.linalg.norm(target_z_ax)
        if norm_z < 1e-6:
            target_z_ax = np.array([0, 0, 1])
        else:
            target_z_ax = target_z_ax / norm_z

        psi = target_yaw
        target_x_c = np.array([np.cos(psi), np.sin(psi), 0])

        target_y_ax = np.cross(target_z_ax, target_x_c)
        norm_y = np.linalg.norm(target_y_ax)
        if norm_y < 1e-6:
            target_y_ax = np.array([-np.sin(psi), np.cos(psi), 0])
        else:
            target_y_ax = target_y_ax / norm_y

        target_x_ax = np.cross(target_y_ax, target_z_ax)

        target_rotation = np.column_stack((target_x_ax, target_y_ax, target_z_ax))
        target_euler = Rotation.from_matrix(target_rotation).as_euler(
            "xyz", degrees=False
        )

        # --- 2. Attitude Control (Inner Loop) ---
        target_rpy_rates = np.zeros(3)
        rpms = self.base_ctrl._dslPIDAttitudeControl(
            control_timestep,
            target_thrust_pwm,
            cur_quat,
            target_euler,
            target_rpy_rates,
        )

        return rpms
