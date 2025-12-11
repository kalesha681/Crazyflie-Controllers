import numpy as np
import cvxpy as cp
from gym_pybullet_drones.utils.enums import DroneModel
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from scipy.spatial.transform import Rotation
from crazyflie_controllers.controllers.base_controller import BaseController

class MPCController(BaseController):
    """
    Linear Model Predictive Controller (MPC) using cvxpy/OSQP.
    """
    def __init__(self, drone_model=DroneModel.CF2X, horizon=20):
        self.drone_model = drone_model
        self.H = horizon
        self.dt = 0.05 # Planning horizon dt
        
        # Physical Constants
        self.m = 0.027  # kg
        self.g = 9.81   # m/s^2
        self.kf = 3.16e-10 
        
        self.base_ctrl = DSLPIDControl(drone_model=drone_model)
        
        # --- PRE-COMPILE MPC PROBLEM ---
        self.var_x = cp.Variable((6, self.H + 1))
        self.var_u = cp.Variable((3, self.H))
        
        self.par_x_init = cp.Parameter(6)
        self.par_ref_traj = cp.Parameter((6, self.H))
        
        Q_pos = 4000.0 * np.eye(3) 
        Q_vel = 100.0 * np.eye(3)
        R_acc = 1.0 * np.eye(3)
        
        A = np.eye(6)
        A[0, 3] = self.dt
        A[1, 4] = self.dt
        A[2, 5] = self.dt
        
        B = np.zeros((6, 3))
        B[0:3, 0:3] = 0.5 * self.dt**2 * np.eye(3)
        B[3:6, 0:3] = self.dt * np.eye(3)
        
        cost = 0
        constraints = []
        
        constraints += [self.var_x[:, 0] == self.par_x_init]
        
        for k in range(self.H):
            p_err = self.var_x[0:3, k] - self.par_ref_traj[0:3, k]
            v_err = self.var_x[3:6, k] - self.par_ref_traj[3:6, k]
            
            cost += cp.quad_form(p_err, Q_pos) + \
                    cp.quad_form(v_err, Q_vel) + \
                    cp.quad_form(self.var_u[:, k], R_acc)
            
            constraints += [self.var_x[:, k+1] == A @ self.var_x[:, k] + B @ self.var_u[:, k]]
            
            constraints += [self.var_u[0, k] <= 8.0, self.var_u[0, k] >= -8.0]
            constraints += [self.var_u[1, k] <= 8.0, self.var_u[1, k] >= -8.0]
            constraints += [self.var_u[2, k] <= 10.0, self.var_u[2, k] >= -5.0]
            
        self.prob = cp.Problem(cp.Minimize(cost), constraints)
        self.warm_start_enabled = False

    def reset(self):
        self.base_ctrl.reset()
        # Clear warm-start cache for determinism
        if hasattr(self, 'var_x'): self.var_x.value = None
        if hasattr(self, 'var_u'): self.var_u.value = None
        self.warm_start_enabled = False

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
                        
        # --- 1. PREPARE PARAMETERS ---
        x_init_val = np.concatenate([cur_pos, cur_vel])
        self.par_x_init.value = x_init_val
        
        ref_traj_val = np.zeros((6, self.H))
        ref_p = target_pos
        ref_v = target_vel
        
        for k in range(self.H):
             curr_ref_p = ref_p + ref_v * (k * self.dt)
             curr_ref_v = ref_v
             ref_traj_val[0:3, k] = curr_ref_p
             ref_traj_val[3:6, k] = curr_ref_v
             
        self.par_ref_traj.value = ref_traj_val

        # --- 2. SOLVE MPC ---
        try:
            self.prob.solve(solver=cp.OSQP, warm_start=self.warm_start_enabled)
            self.warm_start_enabled = True # Enable for subsequent steps
            if self.var_u.value is None:
                des_acc = target_acc
            else:
                des_acc = self.var_u.value[:, 0]
        except:
             des_acc = target_acc

        # --- 3. CONVERT TO ATTITUDE/THRUST ---
        des_acc_x = des_acc[0]
        des_acc_y = des_acc[1]
        des_acc_z = des_acc[2] + self.g
        
        total_thrust_force = self.m * np.linalg.norm([des_acc_x, des_acc_y, des_acc_z])
        target_thrust_rpm = np.sqrt(total_thrust_force / (4 * self.kf))
        target_thrust_pwm = (target_thrust_rpm - self.base_ctrl.PWM2RPM_CONST) / self.base_ctrl.PWM2RPM_SCALE
        target_thrust_pwm = np.clip(target_thrust_pwm, 0, 65535)
        
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
        target_euler = Rotation.from_matrix(target_rotation).as_euler('xyz', degrees=False)
        
        target_rpy_rates = np.zeros(3)
        rpms = self.base_ctrl._dslPIDAttitudeControl(
            control_timestep,
            target_thrust_pwm,
            cur_quat,
            target_euler,
            target_rpy_rates
        )
        
        return rpms
