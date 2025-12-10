import numpy as np
import cvxpy as cp
from .base_controller import BaseController
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel
from scipy.spatial.transform import Rotation

class MPCController(BaseController):
    # Multi-rate MPC: Plan with coarse dt (0.05s) to get long horizon (1s)
    # Control at high freq (240Hz) using the planned acceleration.
    def __init__(self, drone_model=DroneModel.CF2X, horizon=20):
        self.drone_model = drone_model
        self.H = horizon
        self.dt = 0.05 # 20Hz Planning Horizon -> 1.0s Total Lookahead
        
        # Physical Constants
        self.m = 0.027  # kg
        self.g = 9.81   # m/s^2
        self.kf = 3.16e-10 
        
        # Initialize Inner Loop
        self.base_ctrl = DSLPIDControl(drone_model=drone_model)
        
        # --- PRE-COMPILE MPC PROBLEM ---
        # Defining the problem once with Parameters speeds up solving by ~10-50x
        
        # 1. Variables
        self.var_x = cp.Variable((6, self.H + 1))
        self.var_u = cp.Variable((3, self.H))
        
        # 2. Parameters (Inputs that change every step)
        self.par_x_init = cp.Parameter(6)
        self.par_ref_traj = cp.Parameter((6, self.H)) # [Pos; Vel] for each step
        
        # 3. Component Matrices
        Q_pos = 4000.0 * np.eye(3) 
        Q_vel = 100.0 * np.eye(3)
        R_acc = 1.0 * np.eye(3)
        
        # Dynamics Matrix (Discrete Double Integrator with COARSE dt)
        A = np.eye(6)
        A[0, 3] = self.dt
        A[1, 4] = self.dt
        A[2, 5] = self.dt
        
        B = np.zeros((6, 3))
        B[0:3, 0:3] = 0.5 * self.dt**2 * np.eye(3)
        B[3:6, 0:3] = self.dt * np.eye(3)
        
        cost = 0
        constraints = []
        
        # Initial State Constraint
        constraints += [self.var_x[:, 0] == self.par_x_init]
        
        for k in range(self.H):
            # Cost: (x_k - ref_k)' Q (x_k - ref_k) + u_k' R u_k
            # Extract pos/vel from state and reference
            p_err = self.var_x[0:3, k] - self.par_ref_traj[0:3, k]
            v_err = self.var_x[3:6, k] - self.par_ref_traj[3:6, k]
            
            cost += cp.quad_form(p_err, Q_pos) + \
                    cp.quad_form(v_err, Q_vel) + \
                    cp.quad_form(self.var_u[:, k], R_acc)
            
            # Dynamics
            constraints += [self.var_x[:, k+1] == A @ self.var_x[:, k] + B @ self.var_u[:, k]]
            
            # Input Constraints (Acceleration)
            constraints += [self.var_u[0, k] <= 8.0, self.var_u[0, k] >= -8.0]
            constraints += [self.var_u[1, k] <= 8.0, self.var_u[1, k] >= -8.0]
            constraints += [self.var_u[2, k] <= 10.0, self.var_u[2, k] >= -5.0]
            
        # Build Problem Object
        self.prob = cp.Problem(cp.Minimize(cost), constraints)

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
        
        # --- 1. PREPARE PARAMETERS ---
        
        # Initial State
        x_init_val = np.concatenate([cur_pos, cur_vel])
        self.par_x_init.value = x_init_val
        
        # Reference Trajectory (Linear Prediction)
        # We fill the parameter matrix (6, H)
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
            # Solver is cached, only parameters update. FAST.
            self.prob.solve(solver=cp.OSQP, warm_start=True)
            
            if self.var_u.value is None:
                des_acc = target_acc
            else:
                des_acc = self.var_u.value[:, 0]
        except:
             des_acc = target_acc

        # --- 2. CONVERT ACCELERATION TO ATTITUDE/THRUST ---
        des_acc_x = des_acc[0]
        des_acc_y = des_acc[1]
        des_acc_z = des_acc[2] + self.g
        
        # Calculate Thrust (Newtons -> RPM -> PWM)
        total_thrust_force = self.m * np.linalg.norm([des_acc_x, des_acc_y, des_acc_z])
        target_thrust_rpm = np.sqrt(total_thrust_force / (4 * self.kf))
        target_thrust_pwm = (target_thrust_rpm - self.base_ctrl.PWM2RPM_CONST) / self.base_ctrl.PWM2RPM_SCALE
        target_thrust_pwm = np.clip(target_thrust_pwm, 0, 65535)
        
        # Calculate Orientation (Rotation Matrix)
        target_z_ax = np.array([des_acc_x, des_acc_y, des_acc_z])
        norm_z = np.linalg.norm(target_z_ax)
        if norm_z < 1e-6:
             target_z_ax = np.array([0, 0, 1])
        else:
             target_z_ax = target_z_ax / norm_z
             
        # Desired Yaw (from trajectory)
        psi = target_rpy[2]
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
        
        # --- 3. INNER LOOP ---
        rpms = self.base_ctrl._dslPIDAttitudeControl(
            control_timestep,
            target_thrust_pwm,
            cur_quat,
            target_euler,
            target_rpy_rates
        )
        
        return rpms
