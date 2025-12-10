import numpy as np
import matplotlib.pyplot as plt
import pybullet as p
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

from src.Controllers.mpc_controller import MPCController
from src.trajectories.Square.trajectory import SquareTrajectory

def run():
    env = CtrlAviary(drone_model=DroneModel.CF2X, num_drones=1, physics=Physics.PYB, gui=False)
    
    ctrl = MPCController(horizon=15)
    traj = SquareTrajectory()
    
    obs, _ = env.reset()
    action = np.zeros((1, 4))
    path_actual, path_target = [], []
    
    ctrl_freq = env.CTRL_FREQ
    steps = int(12 * ctrl_freq)
    dt = 1.0 / ctrl_freq
    for i in range(steps):
        t = i * dt
        state = obs[0]
        
        target_pos, target_vel, _ = traj.get_target(t)
        
        rpm = ctrl.compute_control(dt, state[0:3], state[3:7], state[10:13], state[13:16], target_pos, target_vel=target_vel)
        action[0] = rpm
        
        obs, _, _, _, _ = env.step(action)
        path_actual.append(state[0:2])
        path_target.append(target_pos[:2])
        
    env.close()
    
    # Plot
    act, ref = np.array(path_actual), np.array(path_target)
    plt.figure(figsize=(6,6))
    plt.plot(ref[:,0], ref[:,1], 'k--', label='Target Square')
    plt.plot(act[:,0], act[:,1], 'r', label='Drone (MPC)')
    plt.legend(); plt.grid(True); plt.axis('equal'); plt.title("Square Tracking (MPC)")
    filename = "outputs/plots/square/mpc.png"
    plt.savefig(filename)
    print(f"[SUCCESS] Plot saved to {filename}")

if __name__ == "__main__":
    run()
