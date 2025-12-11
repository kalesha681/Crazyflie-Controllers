import argparse
import sys
import os
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from dataclasses import dataclass
from pathlib import Path
from typing import List, Type, Dict

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))

import pybullet as p
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

# Core Imports
from crazyflie_controllers.controllers.base_controller import BaseController
from crazyflie_controllers.utils.logging import Logger
from crazyflie_controllers.trajectories.base_trajectory import BaseTrajectory

# Implementations
from crazyflie_controllers.controllers.pid_controller import PIDController
from crazyflie_controllers.controllers.smc_controller import SMCController
from crazyflie_controllers.controllers.mpc_controller import MPCController

from crazyflie_controllers.trajectories.circle import CircleTrajectory
from crazyflie_controllers.trajectories.square import SquareTrajectory
from crazyflie_controllers.trajectories.triangle import TriangleTrajectory
from crazyflie_controllers.trajectories.eight import EightTrajectory
from crazyflie_controllers.trajectories.step import StepTrajectory

# --- Registries ---
CONTROLLER_REGISTRY: Dict[str, Type[BaseController]] = {
    "pid": PIDController,
    "smc": SMCController,
    "mpc": MPCController,
}

TRAJECTORY_REGISTRY: Dict[str, Type[BaseTrajectory]] = {
    "circle": CircleTrajectory,
    "square": SquareTrajectory,
    "triangle": TriangleTrajectory,
    "eight": EightTrajectory,
    "step": StepTrajectory,
}

# --- Configuration ---
@dataclass(frozen=True)
class ExperimentConfig:
    trajectory: str
    controllers: List[str]
    dt: float
    duration: float
    seed: int
    save: bool
    plot: bool
    compare: bool
    gui: bool
    output_dir: Path

# --- Pipeline ---

def run_simulation(config: ExperimentConfig, ctrl_name: str):
    """
    Runs a single simulation for a specific controller-trajectory pair.
    """
    print(f"\n[pipeline] Starting Simulation: {ctrl_name.upper()} on {config.trajectory.upper()}")
    
    # 1. Initialize Objects
    if ctrl_name not in CONTROLLER_REGISTRY:
        raise ValueError(f"Invalid controller: {ctrl_name}")
    if config.trajectory not in TRAJECTORY_REGISTRY:
        raise ValueError(f"Invalid trajectory: {config.trajectory}")
        
    ControllerClass = CONTROLLER_REGISTRY[ctrl_name]
    TrajectoryClass = TRAJECTORY_REGISTRY[config.trajectory]
    
    controller = ControllerClass()
    trajectory = TrajectoryClass()
    
    # 2. Initialize Environment
    # Use p.DIRECT for headless (safe, reproducible) unless gui requested
    # Note: Gym-Pybullet-Drones handles init. 
    # To enforce p.DIRECT properly across re-inits, we might need to close previous instances.
    try:
        p.disconnect()
    except: 
        pass
        
    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        physics=Physics.PYB,
        gui=config.gui
    )
    
    # Validate dt
    sim_dt = 1.0 / env.CTRL_FREQ
    if abs(sim_dt - config.dt) > 1e-5:
        print(f"[WARNING] Requested dt={config.dt} but sim dt={sim_dt}. Using Sim dt.")
    
    effective_dt = sim_dt
    STEPS = int(config.duration * env.CTRL_FREQ)
    
    # 3. Reset
    obs, _ = env.reset()
    controller.reset()
    action = np.zeros((1, 4))
    
    # 4. Simulation Loop
    # Initialize Logger
    log_dir = config.output_dir / "data" / config.trajectory
    logger = Logger(str(log_dir))
    
    start_wall_time = time.time()
    
    for i in range(STEPS):
        t = i * effective_dt
        
        # State
        state = obs[0]
        cur_pos = state[0:3]
        cur_quat = state[3:7]
        cur_vel = state[10:13] # linear
        cur_ang_vel = state[13:16] # angular
        
        rpy = p.getEulerFromQuaternion(cur_quat)
        
        # Target
        target_pos, target_vel, target_acc, target_yaw = trajectory.get_target(t)
        
        # Control
        rpm = controller.compute_control(
            control_timestep=effective_dt,
            cur_pos=cur_pos,
            cur_quat=cur_quat,
            cur_vel=cur_vel,
            cur_ang_vel=cur_ang_vel,
            target_pos=target_pos,
            target_vel=target_vel,
            target_acc=target_acc,
            target_yaw=target_yaw
        )
        
        # Act
        action[0] = rpm
        obs, _, _, _, _ = env.step(action)
        if config.gui:
            p.stepSimulation()
            time.sleep(effective_dt)
            
        # Log using strict Logger
        logger.log(
            t=t,
            pos=cur_pos,
            vel=cur_vel,
            rpy=np.array(rpy),
            target_pos=target_pos,
            control_rpm=rpm
        )
        
    env.close()
    
    # 5. Save Data
    if config.save:
        filename = f"{ctrl_name}_{config.trajectory}_dt{effective_dt:.4f}.csv"
        
        metadata = {
            "controller": ctrl_name,
            "trajectory": config.trajectory,
            "dt": effective_dt,
            "duration": config.duration,
            "seed": config.seed
        }
        
        logger.save_as_csv(filename, metadata=metadata)


def generate_plots(config: ExperimentConfig):
    """
    Generates plots by reading SAVED CSV files only.
    """
    print(f"\n[pipeline] Generating Plots for {config.trajectory.upper()}...")
    
    plot_dir = config.output_dir / "plots" / config.trajectory
    data_dir = config.output_dir / "data" / config.trajectory
    plot_dir.mkdir(parents=True, exist_ok=True)
    
    # Load Data
    data = {}
    for ctrl in config.controllers:
        # Match pattern
        # {ctrl}_{traj}_dt*.csv
        # We find the file that matches the controller
        candidates = list(data_dir.glob(f"{ctrl}_{config.trajectory}_dt*.csv"))
        if not candidates:
            print(f"[WARNING] No data found for {ctrl}, skipping plot.")
            continue
        
        # Take the most recent or matching one?
        # Ideally we only generated one just now.
        filepath = candidates[0] 
        df = pd.read_csv(filepath, comment='#')
        data[ctrl] = df
        
    if not data:
        return

    # 1. Trajectory Comparison (2D)
    plt.figure(figsize=(10, 8))
    
    # Reference
    first_ctrl = list(data.keys())[0]
    ref_df = data[first_ctrl]
    plt.plot(ref_df['x_ref'], ref_df['y_ref'], 'k--', label='Reference', linewidth=2)
    
    for ctrl, df in data.items():
        # Calculate RMSE
        err_sq = df['ex']**2 + df['ey']**2 + df['ez']**2
        rmse = np.sqrt(err_sq.mean())
        plt.plot(df['x'], df['y'], label=f"{ctrl.upper()} (RMSE={rmse:.3f}m)")
        
    plt.title(f"Trajectory: {config.trajectory.capitalize()}")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend()
    plt.grid()
    plt.axis('equal')
    save_path = plot_dir / f"compare_traj_{config.trajectory}.png"
    plt.savefig(save_path)
    print(f"[pipeline] Saved Plot: {save_path}")
    
    # 2. Error vs Time
    plt.figure(figsize=(10, 6))
    for ctrl, df in data.items():
        error_norm = np.sqrt(df['ex']**2 + df['ey']**2 + df['ez']**2)
        plt.plot(df['time'], error_norm, label=ctrl.upper())
        
    plt.title(f"Position Error Norm: {config.trajectory.capitalize()}")
    plt.xlabel("Time [s]")
    plt.ylabel("Error [m]")
    plt.legend()
    plt.grid()
    save_path = plot_dir / f"compare_error_{config.trajectory}.png"
    plt.savefig(save_path)
    print(f"[pipeline] Saved Plot: {save_path}")
    
    # 3. Control Effort vs Time (Total RPM sum or norm?)
    # Let's plot average RPM
    plt.figure(figsize=(10, 6))
    for ctrl, df in data.items():
        avg_rpm = (df['u1'] + df['u2'] + df['u3'] + df['u4']) / 4.0
        plt.plot(df['time'], avg_rpm, label=ctrl.upper())
        
    plt.title(f"Average Control Effort (RPM): {config.trajectory.capitalize()}")
    plt.xlabel("Time [s]")
    plt.ylabel("RPM")
    plt.legend()
    plt.grid()
    save_path = plot_dir / f"compare_effort_{config.trajectory}.png"
    plt.savefig(save_path)
    print(f"[pipeline] Saved Plot: {save_path}")


def main():
    parser = argparse.ArgumentParser(description="Crazyflie Controllers Experiment Pipeline")
    
    # Mandatory
    parser.add_argument('--trajectory', type=str, required=True, choices=TRAJECTORY_REGISTRY.keys())
    parser.add_argument('--controllers', type=str, nargs='+', required=True, choices=CONTROLLER_REGISTRY.keys())
    
    # Optional with Defaults
    parser.add_argument('--duration', type=float, default=12.0)
    parser.add_argument('--dt', type=float, default=0.005) # ~200Hz default req
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--output-dir', type=str, default='outputs')
    
    # Flags
    parser.add_argument('--gui', action='store_true', help='Enable PyBullet GUI (WARNING: May crash on headless)')
    parser.add_argument('--save', action='store_true', default=True, help='Save CSV data (Default: True)')
    parser.add_argument('--no-save', action='store_false', dest='save', help='Disable CSV saving')
    parser.add_argument('--plot', action='store_true', default=True, help='Generate Plots (Default: True)')
    parser.add_argument('--no-plot', action='store_false', dest='plot', help='Disable Plotting')
    
    args = parser.parse_args()
    
    # Setup Config
    config = ExperimentConfig(
        trajectory=args.trajectory,
        controllers=args.controllers,
        dt=args.dt,
        duration=args.duration,
        seed=args.seed,
        save=args.save,
        plot=args.plot,
        compare=len(args.controllers) > 1,
        gui=args.gui,
        output_dir=Path(args.output_dir)
    )
    
    print(f"=== Experiment Configuration ===")
    print(f"Controllers: {config.controllers}")
    print(f"Trajectory:  {config.trajectory}")
    print(f"Duration:    {config.duration}s")
    print(f"Mode:        {'GUI' if config.gui else 'HEADLESS'}")
    print("================================")
    
    # Set Seed
    np.random.seed(config.seed)
    
    # Run Experiments
    for ctrl in config.controllers:
        run_simulation(config, ctrl)
        
    # Generate Plots
    if config.plot and config.save:
        generate_plots(config)
        
    print("\n[pipeline] Experiment Complete.")

if __name__ == "__main__":
    main()
