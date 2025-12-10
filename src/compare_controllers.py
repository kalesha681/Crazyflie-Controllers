
import argparse
import numpy as np
import matplotlib.pyplot as plt
import pybullet as p
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

# Controllers
from src.Controllers.pid_controller import PIDController
from src.Controllers.smc_controller import SMCController
from src.Controllers.mpc_controller import MPCController

# Trajectories
from src.trajectories.Circle.trajectory import CircleTrajectory
from src.trajectories.Square.trajectory import SquareTrajectory
from src.trajectories.Triangle.trajectory import TriangleTrajectory
from src.trajectories.Eight.trajectory import EightTrajectory
from src.trajectories.Step.trajectory import StepTrajectory

def get_trajectory(shape):
    if shape == 'circle': return CircleTrajectory()
    if shape == 'square': return SquareTrajectory()
    if shape == 'triangle': return TriangleTrajectory()
    if shape == 'eight': return EightTrajectory()
    if shape == 'step': return StepTrajectory()
    raise ValueError(f"Unknown shape: {shape}")

def calculate_step_metrics(time_arr, actual_pos, setpoint_final, step_time_idx):
    """
    Calculates Step Response Metrics:
    - Rise Time (10% -> 90%)
    - Peak Time
    - % Overshoot
    - Settling Time (2%)
    - Steady State Error
    - Delay Time (50%)
    """
    # Assuming Step in X axis (0 -> 1)
    # Analyze only the step axis relative to start
    # actual_pos is N x 2 or N x 3. Let's assume N x 3 if we passed step traj
    
    # Extract the response AFTER the step time
    # step_time_idx is when step happens
    
    # Shift time to 0 at step
    t_response = time_arr[step_time_idx:] - time_arr[step_time_idx]
    y_response = 0.0
    
    # Detect which axis had the step
    # We know StepTrajectory moves 0->1 in X by default
    y = actual_pos[step_time_idx:, 0] 
    final_val = setpoint_final[0]
    
    if abs(final_val) < 1e-3: 
         # Try Z axis?
         y = actual_pos[step_time_idx:, 2]
         final_val = setpoint_final[2]
         
    # Metrics
    if len(y) == 0: return {}
    
    # 1. Steady State Error
    # Average of last 10% of data
    n_sample = max(1, int(len(y)*0.1))
    y_ss = np.mean(y[-n_sample:])
    ess = abs(final_val - y_ss)
    
    # 2. Peak
    peak_val = np.max(y)
    peak_idx = np.argmax(y)
    tp = t_response[peak_idx]
    
    # 3. Overshoot
    if peak_val > final_val:
        overshoot_pct = ((peak_val - final_val) / final_val) * 100.0
    else:
        overshoot_pct = 0.0
        
    # Data for Rise/Delay
    # Find indices where y crosses thresholds
    # 10%
    try:
        idx_10 = np.where(y >= 0.1 * final_val)[0][0]
        t_10 = t_response[idx_10]
    except: t_10 = None
    
    # 50% (Delay)
    try:
        idx_50 = np.where(y >= 0.5 * final_val)[0][0]
        td = t_response[idx_50]
    except: td = None
    
    # 90%
    try:
        idx_90 = np.where(y >= 0.9 * final_val)[0][0]
        t_90 = t_response[idx_90]
    except: t_90 = None
    
    # Rise Time
    if t_10 is not None and t_90 is not None:
        tr = t_90 - t_10
    else:
        tr = None
        
    # Settling Time (2% band)
    # Find last time index where error > 2%
    lower_bound = 0.98 * final_val
    upper_bound = 1.02 * final_val
    
    # Check outside band
    outside_band = np.where((y < lower_bound) | (y > upper_bound))[0]
    if len(outside_band) > 0:
        last_idx = outside_band[-1]
        ts = t_response[last_idx]
    else:
        ts = 0.0 # Settled immediately (unlikely) or never left band?
        
    return {
        "Rise Time": tr,
        "Peak Time": tp,
        "Delay Time": td,
        "Overshoot %": overshoot_pct,
        "Settling Time": ts,
        "SS Error": ess
    }

def run_simulation(controller, trajectory, duration_sec=12, gui=False):
    """
    Runs a single simulation and returns actual path and target path.
    """
    env = CtrlAviary(drone_model=DroneModel.CF2X, num_drones=1, physics=Physics.PYB, gui=gui)
    
    path_actual = []
    path_target = []
    
    action = np.zeros((1, 4))
    obs, _ = env.reset()
    
    ctrl_freq = env.CTRL_FREQ
    STEPS = int(duration_sec * ctrl_freq)
    dt = 1.0 / ctrl_freq
    
    print(f"   > Simulating...")
    
    # Logging Lists
    log_time = []
    log_target_pos = []
    log_target_vel = []
    log_actual_pos = []
    log_actual_vel = []
    
    for i in range(STEPS):
        t = i * dt
        
        state = obs[0]
        cur_pos = state[0:3]
        cur_quat = state[3:7]
        cur_vel = state[10:13]
        cur_ang_vel = state[13:16]
        
        target_pos, target_vel, target_acc = trajectory.get_target(t)
        
        rpm = controller.compute_control(
            control_timestep=dt,
            cur_pos=cur_pos,
            cur_quat=cur_quat,
            cur_vel=cur_vel,
            cur_ang_vel=cur_ang_vel,
            target_pos=target_pos,
            target_vel=target_vel,
            target_acc=target_acc
        )
        
        action[0] = rpm
        obs, _, _, _, _ = env.step(action)
        if gui: p.stepSimulation()
        
        path_actual.append(cur_pos) # Store full 3D for step analysis
        path_target.append(target_pos)
        
        # Log Data
        log_time.append(t)
        log_target_pos.append(target_pos)
        log_target_vel.append(target_vel)
        log_actual_pos.append(cur_pos)
        log_actual_vel.append(cur_vel)
        
    env.close()
    
    # Convert to arrays
    log_time = np.array(log_time)
    log_target_pos = np.array(log_target_pos)
    log_target_vel = np.array(log_target_vel)
    log_actual_pos = np.array(log_actual_pos)
    log_actual_vel = np.array(log_actual_vel)
    
    # path_actual is N x 3 list
    
    return np.array(path_actual), np.array(path_target), (log_time, log_target_pos, log_target_vel, log_actual_pos, log_actual_vel)

def calculate_rmse(actual, target):
    return np.sqrt(np.mean((actual - target)**2))

def main():
    parser = argparse.ArgumentParser(description="Compare PID vs SMC")
    parser.add_argument('--trajectory', type=str, default='eight', choices=['circle', 'square', 'triangle', 'eight', 'step'])
    parser.add_argument('--no-gui', action='store_true')
    args = parser.parse_args()
    
    trajectory_shape = args.trajectory
    print(f"=== Comparing PID vs SMC on {trajectory_shape.upper()} Trajectory ===")
    
    # 1. Run PID
    print(f"[1/2] Running PID Controller...")
    pid_ctrl = PIDController()
    traj_pid = get_trajectory(trajectory_shape)
    act_pid, ref_pid, log_pid = run_simulation(pid_ctrl, traj_pid, gui=not args.no_gui)
    
    # Save PID Data
    data_dir = f"outputs/data/{trajectory_shape}"
    import os
    os.makedirs(data_dir, exist_ok=True)
    
    header = "Time,DesX,DesY,DesZ,DesVX,DesVY,DesVZ,PosX,PosY,PosZ,VelX,VelY,VelZ"
    data_pid = np.column_stack((log_pid[0], log_pid[1], log_pid[2], log_pid[3], log_pid[4]))
    np.savetxt(f"{data_dir}/pid_data.csv", data_pid, delimiter=",", header=header, comments="")

    # 2. Run SMC
    print(f"[2/2] Running SMC Controller...")
    smc_ctrl = SMCController()
    traj_smc = get_trajectory(trajectory_shape)
    act_smc, ref_smc, log_smc = run_simulation(smc_ctrl, traj_smc, gui=not args.no_gui)
    
    # Save SMC Data
    data_smc = np.column_stack((log_smc[0], log_smc[1], log_smc[2], log_smc[3], log_smc[4]))
    np.savetxt(f"{data_dir}/smc_data.csv", data_smc, delimiter=",", header=header, comments="")
    
    # 3. Run MPC
    print(f"[3/3] Running MPC Controller...")
    mpc_ctrl = MPCController()
    traj_mpc = get_trajectory(trajectory_shape)
    act_mpc, ref_mpc, log_mpc = run_simulation(mpc_ctrl, traj_mpc, gui=not args.no_gui)
    
    # Save MPC Data
    data_mpc = np.column_stack((log_mpc[0], log_mpc[1], log_mpc[2], log_mpc[3], log_mpc[4]))
    np.savetxt(f"{data_dir}/mpc_data.csv", data_mpc, delimiter=",", header=header, comments="")

    # 4. Analysis
    # Note: act_* is now N x 3 (full pos)
    # calculate_rmse wants N x M vs N x M. Should be fine.
    
    rmse_pid = calculate_rmse(act_pid, ref_pid)
    rmse_smc = calculate_rmse(act_smc, ref_smc)
    rmse_mpc = calculate_rmse(act_mpc, ref_mpc)
    
    print(f"\n=== RESULTS ===")
    print(f"PID RMSE: {rmse_pid:.4f}")
    print(f"SMC RMSE: {rmse_smc:.4f}")
    print(f"MPC RMSE: {rmse_mpc:.4f}")
    
    import os
    plot_dir = f"outputs/plots/{trajectory_shape}"
    os.makedirs(plot_dir, exist_ok=True)
    
    # 5. Step Response Stats
    if trajectory_shape == 'step':
        # Find step index (t=1.0)
        # log_pid[0] is time
        step_idx = np.where(log_pid[0] >= 1.0)[0][0]
        final_setpoint = np.array([1.0, 0.0, 1.0]) # Matches StepTrajectory x=1
        
        metrics_pid = calculate_step_metrics(log_pid[0], act_pid, final_setpoint, step_idx)
        metrics_smc = calculate_step_metrics(log_smc[0], act_smc, final_setpoint, step_idx)
        metrics_mpc = calculate_step_metrics(log_mpc[0], act_mpc, final_setpoint, step_idx)
        
        print("\n=== STEP RESPONSE METRICS ===")
        # Header
        print(f"{'Metric':<20} | {'PID':<10} | {'MPC':<10} | {'SMC':<10}")
        print("-" * 60)
        
        keys = ["Rise Time", "Peak Time", "Delay Time", "Overshoot %", "Settling Time", "SS Error"]
        for k in keys:
             # Handle None values
             v_pid = f"{metrics_pid.get(k, 0):.4f}" if metrics_pid.get(k) is not None else "-"
             v_mpc = f"{metrics_mpc.get(k, 0):.4f}" if metrics_mpc.get(k) is not None else "-"
             v_smc = f"{metrics_smc.get(k, 0):.4f}" if metrics_smc.get(k) is not None else "-"
             print(f"{k:<20} | {v_pid:<10} | {v_mpc:<10} | {v_smc:<10}")

    # 6. Plotting (Comparison)
    plt.figure(figsize=(10,10))
    # Plot X-Y by default, or Time vs X if Step?
    
    if trajectory_shape == 'step':
        # Plot Time vs X Position
        plt.plot(log_pid[0], log_pid[1][:,0], 'k--', linewidth=2, label='Target X')
        plt.plot(log_pid[0], act_pid[:,0], 'b-', alpha=0.6, label='PID')
        plt.plot(log_smc[0], act_smc[:,0], 'r-', alpha=0.6, label='SMC')
        plt.plot(log_mpc[0], act_mpc[:,0], 'g-', alpha=0.8, label='MPC')
        
        plt.title('Step Response (X-Axis)')
        plt.xlabel('Time [s]')
        plt.ylabel('X Position [m]')
    else:
        # Plot Path (X-Y)
        plt.plot(ref_pid[:,0], ref_pid[:,1], 'k--', linewidth=2, label='Target Path')
        plt.plot(act_pid[:,0], act_pid[:,1], 'b-', alpha=0.6, label=f'PID (RMSE={rmse_pid:.3f})')
        plt.plot(act_smc[:,0], act_smc[:,1], 'r-', alpha=0.6, label=f'SMC (RMSE={rmse_smc:.3f})')
        plt.plot(act_mpc[:,0], act_mpc[:,1], 'g-', alpha=0.8, label=f'MPC (RMSE={rmse_mpc:.3f})')
        
        plt.title(f'Controller Comparison: PID vs SMC vs MPC ({trajectory_shape.capitalize()})')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.axis('equal')

    plt.legend()
    plt.grid(True)
    
    filename = f"{plot_dir}/compare_pid_smc_mpc_{trajectory_shape}.png"
    plt.savefig(filename)
    print(f"\n[SUCCESS] Comparison plot saved to {filename}")
    
    # 7. Save Individual Plots (Skipping for brevity, can enable if needed)
    # Re-using act arrays which are N x 3
    # ... (Logic identical to before, just handle 3D array slicing)
    
    plt.figure(figsize=(8,8))
    # PID
    if trajectory_shape == 'step':
         plt.plot(log_pid[0], ref_pid[:,0], 'k--'); plt.plot(log_pid[0], act_pid[:,0], 'b-')
    else:
         plt.plot(ref_pid[:,0], ref_pid[:,1], 'k--'); plt.plot(act_pid[:,0], act_pid[:,1], 'b-')
    plt.title(f'PID Tracking'); plt.grid(True); plt.savefig(f"{plot_dir}/pid_tracking.png")
    
    plt.figure(figsize=(8,8))
    # SMC
    if trajectory_shape == 'step':
         plt.plot(log_smc[0], ref_smc[:,0], 'k--'); plt.plot(log_smc[0], act_smc[:,0], 'r-')
    else:
         plt.plot(ref_smc[:,0], ref_smc[:,1], 'k--'); plt.plot(act_smc[:,0], act_smc[:,1], 'r-')
    plt.title(f'SMC Tracking'); plt.grid(True); plt.savefig(f"{plot_dir}/smc_tracking.png")
    
    plt.figure(figsize=(8,8))
    # MPC
    if trajectory_shape == 'step':
         plt.plot(log_mpc[0], ref_mpc[:,0], 'k--'); plt.plot(log_mpc[0], act_mpc[:,0], 'g-')
    else:
         plt.plot(ref_mpc[:,0], ref_mpc[:,1], 'k--'); plt.plot(act_mpc[:,0], act_mpc[:,1], 'g-')
    plt.title(f'MPC Tracking'); plt.grid(True); plt.savefig(f"{plot_dir}/mpc_tracking.png")

if __name__ == "__main__":
    main()
