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

def get_controller(algo):
    if algo == 'pid': return PIDController()
    if algo == 'smc': return SMCController()
    if algo == 'mpc': return MPCController()
    raise ValueError(f"Unknown controller: {algo}")

def get_trajectory(shape):
    if shape == 'circle': return CircleTrajectory()
    if shape == 'square': return SquareTrajectory()
    if shape == 'triangle': return TriangleTrajectory()
    if shape == 'eight': return EightTrajectory()
    raise ValueError(f"Unknown shape: {shape}")

def run(args):
    # Initialize Environment
    # GUI is enabled by default unless --no-gui is passed
    gui = not args.no_gui
    env = CtrlAviary(drone_model=DroneModel.CF2X, num_drones=1, physics=Physics.PYB, gui=gui)
    
    # Initialize Components
    controller = get_controller(args.controller)
    trajectory = get_trajectory(args.trajectory)
    
    print(f"[INFO] Running {args.controller.upper()} Controller tracking {args.trajectory.upper()} Trajectory")
    
    # Simulation Loop
    # Logging Lists
    log_time = []
    log_target_pos = []
    log_target_vel = []
    log_actual_pos = []
    log_actual_vel = []
    
    action = np.zeros((1, 4))
    obs, _ = env.reset()
    
    # Run for 12 seconds
    DURATION_SEC = 12
    ctrl_freq = env.CTRL_FREQ
    STEPS = int(DURATION_SEC * ctrl_freq)
    dt = 1.0 / ctrl_freq
    
    for i in range(STEPS):
        t = i * dt
        
        # 1. Get State
        state = obs[0]
        cur_pos = state[0:3]
        cur_quat = state[3:7]
        cur_vel = state[10:13]
        cur_ang_vel = state[13:16]
        
        # 2. Get Target
        target_pos, target_vel, target_acc = trajectory.get_target(t)
        
        # 3. Compute Control
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
        
        # 4. Step Environment
        obs, _, _, _, _ = env.step(action)
        if gui: p.stepSimulation()
        
        # 5. Log
        log_time.append(t)
        log_target_pos.append(target_pos)
        log_target_vel.append(target_vel)
        log_actual_pos.append(cur_pos)
        log_actual_vel.append(cur_vel)
    
    # Convert to arrays
    log_time = np.array(log_time)
    log_target_pos = np.array(log_target_pos)
    log_target_vel = np.array(log_target_vel)
    log_actual_pos = np.array(log_actual_pos)
    log_actual_vel = np.array(log_actual_vel)
    
    # Plotting (Update to use log arrays)
    # act = np.array(path_actual) -> log_actual_pos[:, :2]
    
    plt.figure(figsize=(6,6))
    plt.plot(log_target_pos[:,0], log_target_pos[:,1], 'k--', label=f'Target {args.trajectory.capitalize()}')
    plt.plot(log_actual_pos[:,0], log_actual_pos[:,1], 'b', label=f'Drone ({args.controller.upper()})')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title(f"{args.trajectory.capitalize()} Tracking with {args.controller.upper()}")
    
    # Create directory if it doesn't exist
    import os
    output_dir = f"outputs/plots/{args.trajectory}"
    os.makedirs(output_dir, exist_ok=True)
    
    filename = f"{output_dir}/unified_{args.controller}.png"
    plt.savefig(filename)
    print(f"[SUCCESS] Plot saved to {filename}")

    # CSV Logging
    data_dir = f"outputs/data/{args.trajectory}"
    os.makedirs(data_dir, exist_ok=True)
    csv_filename = f"{data_dir}/{args.controller}_data.csv"
    
    # Stack data: Time, DesX, DesY, DesZ, DesVX, DesVY, DesVZ, X, Y, Z, VX, VY, VZ
    header = "Time,DesX,DesY,DesZ,DesVX,DesVY,DesVZ,PosX,PosY,PosZ,VelX,VelY,VelZ"
    
    data_block = np.column_stack((
        log_time,
        log_target_pos,
        log_target_vel,
        log_actual_pos,
        log_actual_vel
    ))
    
    np.savetxt(csv_filename, data_block, delimiter=",", header=header, comments="")
    print(f"[SUCCESS] Data saved to {csv_filename}")

    try:
        env.close()
    except Exception as e:
        print(f"[WARNING] Crash during env.close(): {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Crazyflie Trajectory Tracking")
    parser.add_argument('--controller', type=str, default='pid', choices=['pid', 'smc', 'mpc'], help='Controller type')
    parser.add_argument('--trajectory', type=str, default='circle', choices=['circle', 'square', 'triangle', 'eight'], help='Trajectory shape')
    parser.add_argument('--no-gui', action='store_true', help='Disable PyBullet GUI')
    
    args = parser.parse_args()
    run(args)
