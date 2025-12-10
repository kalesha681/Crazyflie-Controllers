
import numpy as np
import math
from .base_controller import BaseController
from gym_pybullet_drones.utils.enums import DroneModel
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl

class SMCController(BaseController):
    def __init__(self, drone_model=DroneModel.CF2X):
        self.drone_model = drone_model
        # Initialize DSLPID for Attitude Loop
        self.base_ctrl = DSLPIDControl(drone_model=drone_model)
        
        # Physical Constants for Crazyflie 2.x
        # Approximation from bitcraze firmware / gym-pybullet-drones
        self.m = 0.027  # kg
        self.g = 9.81   # m/s^2
        self.d = 0.040  # Arm length (m) approx
        
        self.kf = 3.16e-10 # Propeller thrust coefficient
        self.km = 7.94e-12 # Propeller moment coefficient
        
        # SMC Gains (Position)
        # SMC Gains (Position)
        # Tuned for more aggressive/optimal tracking
        self.lambda_x = 8.0  # Back to 8.0
        self.lambda_y = 8.0
        self.lambda_z = 10.0 # Back to 10.0
        
        self.k_x = 5.0      # Back to 5.0
        self.k_y = 5.0
        self.k_z = 10.0     # Back to 10.0
        
        # Attitude PID Gains (Inner Loop)
        self.Kp_phi = 60000.
        self.Kd_phi = 12000.
        self.Kp_theta = 60000.
        self.Kd_theta = 12000.
        self.Kp_psi = 60000.
        self.Kd_psi = 12000.
        
        self.last_rpy_err = np.zeros(3)
        self.last_pos_err = np.zeros(3)
        
        self.PWM2RPM_SCALE = 0.2685 
        self.MIN_PWM = 20000 
        self.MAX_PWM = 65535

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
        
        if target_vel is None: target_vel = np.zeros(3)
        if target_rpy is None: target_rpy = np.zeros(3)
        if target_rpy_rates is None: target_rpy_rates = np.zeros(3)
        if target_acc is None: target_acc = np.zeros(3)
        
        # --- 1. Position Control (SMC Outer Loop) ---
        
        # Errors
        e_p = target_pos - cur_pos
        e_v = target_vel - cur_vel
        
        # Sliding Surfaces
        s_x = e_v[0] + self.lambda_x * e_p[0]
        s_y = e_v[1] + self.lambda_y * e_p[1]
        s_z = e_v[2] + self.lambda_z * e_p[2]
        
        # Virtual Control Inputs (Commanded Acceleration Adjustment)
        u_x = self.k_x * np.tanh(s_x)
        u_y = self.k_y * np.tanh(s_y)
        u_z = self.k_z * np.tanh(s_z)
        
        # Global Acceleration Desired
        # ACC_CMD = ACC_TARGET + LAMBDA * ERROR_VEL + ROBUST_TERM
        # des_acc = target_acc + lambda * e_v + u_rob
        
        des_acc_x = target_acc[0] + self.lambda_x * e_v[0] + u_x
        des_acc_y = target_acc[1] + self.lambda_y * e_v[1] + u_y
        des_acc_z = target_acc[2] + self.lambda_z * e_v[2] + u_z + self.g
        
        # Compute Desired Thrust (RPM)
        # F = m * sqrt(ax^2 + ay^2 + az^2)
        total_thrust_force = self.m * np.linalg.norm([des_acc_x, des_acc_y, des_acc_z])
        
        # Convert Force to RPM (Base Thrust)
        # F = 4 * kf * RPM^2 => RPM = sqrt(F / 4kf)
        target_thrust_rpm = np.sqrt(total_thrust_force / (4 * self.kf))
        
        # Convert RPM to PWM (Required for DSLPID inner loop)
        target_thrust_pwm = (target_thrust_rpm - self.base_ctrl.PWM2RPM_CONST) / self.base_ctrl.PWM2RPM_SCALE
        target_thrust_pwm = np.clip(target_thrust_pwm, 0, 65535)
        
        # --- Geometric Attitude Control ---
        # Robustly calculate target orientation using rotation matrices
        # instead of small angle approximations.
        
        # Desired Z-axis (Body Up) aligned with thrust vector
        target_z_ax = np.array([des_acc_x, des_acc_y, des_acc_z])
        norm_z = np.linalg.norm(target_z_ax)
        if norm_z < 1e-6:
            target_z_ax = np.array([0, 0, 1])
        else:
            target_z_ax = target_z_ax / norm_z
            
        # Target Yaw
        psi = target_rpy[2]
        
        # Desired X-axis (Body Forward) projection
        # We want the projection of X_body onto the XY plane to align with steps of Psi
        target_x_c = np.array([np.cos(psi), np.sin(psi), 0])
        
        # Calculate Y-axis (Body Right) = Z cross X_c
        target_y_ax = np.cross(target_z_ax, target_x_c)
        norm_y = np.linalg.norm(target_y_ax)
        if norm_y < 1e-6:
             # Singularity (Thrust vertical), assume standard Y
             target_y_ax = np.array([-np.sin(psi), np.cos(psi), 0])
        else:
             target_y_ax = target_y_ax / norm_y
             
        # Recalculate X-axis to ensure orthogonality
        target_x_ax = np.cross(target_y_ax, target_z_ax)
        
        # Rotation Matrix
        # [X_ax, Y_ax, Z_ax]
        target_rotation = np.column_stack((target_x_ax, target_y_ax, target_z_ax))
        
        # Convert to Euler Angles
        from scipy.spatial.transform import Rotation
        target_euler = Rotation.from_matrix(target_rotation).as_euler('xyz', degrees=False)
        
        # Note: Scipy returns [x, y, z] -> [phi, theta, psi]
        # Ensure minimal rotation path usually handled by scipy
        
        # --- 2. Attitude Control (Inner Loop via DSLPIDControl) ---
        # We leverage the robust inner loop of DSLPIDControl
        # It expects: control_timestep, thrust (PWM/RPM), cur_quat, target_euler, target_rpy_rates
        
        rpms = self.base_ctrl._dslPIDAttitudeControl(
            control_timestep,
            target_thrust_pwm,
            cur_quat,
            target_euler,
            target_rpy_rates
        )
        
        return rpms
