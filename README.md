# Crazyflie Controllers Benchmark Framework

![CI](https://github.com/kalesha681/Crazyflie-Controllers/actions/workflows/ci.yml/badge.svg)
![Python](https://img.shields.io/badge/python-3.10+-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)
![Code Style](https://img.shields.io/badge/code%20style-black-000000.svg)

## Project Overview

This repository provides a **research-grade, reproducible framework** for simulating and benchmarking control algorithms on the [Bitcraze Crazyflie 2.1](https://www.bitcraze.io/products/crazyflie-2-1/) quadrotor.

It is designed for students, researchers, and hobbyists to understand how different control strategies perform on a nano-UAV. The simulation is powered by **PyBullet**, ensuring physics fidelity while remaining easy to run on any computer.

We implement and compare three fundamental control architectures:
1.  **PID (Proportional-Integral-Derivative)**: The industry standard, simple and effective.
2.  **SMC (Sliding Mode Control)**: A robust nonlinear controller that handles disturbances well.
3.  **MPC (Model Predictive Control)**: An optimal control strategy that plans future moves to minimize error.

---

## Directory Structure

Here is a guide to what you will find in this repository:

```
crazyflie_controllers/
├── scripts/
│   └── run.py                 # The main entry point. Run this to start simulations!
├── src/crazyflie_controllers/ # The core source code package
│   ├── controllers/           # Implementation of PID, SMC, and MPC algorithms
│   ├── trajectories/          # Trajectory definitions (Circle, Figure-8, etc.)
│   └── utils/                 # Helper tools for logging and configuration
├── outputs/
│   ├── data/                  # Generated CSV log files from experiments
│   └── plots/                 # Auto-generated graphs and plots
├── tests/                     # Automated tests to ensure code quality
├── docs/                      # Documentation assets (images, references)
├── .github/                   # CI/CD configuration for GitHub Actions
└── requirements.txt           # List of Python dependencies
```

---

## Getting Started

### Prerequisites
*   **Operating System**: Linux (Ubuntu 20.04/22.04 recommended), macOS, or Windows (WSL2).
*   **Python**: Version 3.10 or higher.
*   **Git**: To clone the repository.

### Installation

1.  **Clone the Repository**
    ```bash
    git clone https://github.com/kalesha681/Crazyflie-Controllers.git
    cd Crazyflie-Controllers
    ```

2.  **Create a Virtual Environment (Recommended)**
    It's best practice to keep dependencies isolated.
    ```bash
    python3 -m venv venv
    source venv/bin/activate  # On Windows: venv\Scripts\activate
    ```

3.  **Install Dependencies**
    ```bash
    pip install -r requirements.txt
    ```

4.  **Install the Package**
    Install the project in editable mode so you can modify code and see changes instantly.
    ```bash
    pip install -e .
    ```

---

## Theory & Mathematics

### System Dynamics
The Crazyflie is modeled as a 6-DOF rigid body. The state vector is $x = [p, v, q, \omega]^T$, where $p$ is position, $v$ is velocity, $q$ is attitude (quaternion), and $\omega$ is angular velocity.

### 1. PID Control (Proportional-Integral-Derivative)
The PID controller minimizes error by calculating a control approach based on:
*   **P (Proportional)**: Current error (*"Where am I?"*)
*   **I (Integral)**: Accumulated past error (*"Have I been off for a while?"*)
*   **D (Derivative)**: Rate of change of error (*"How fast am I approaching?"*)

It uses a cascaded structure:
1.  **Position Controller**: Computes desired acceleration from position error.
2.  **Attitude Controller**: Computes desired torques from angle error.

### 2. Sliding Mode Control (SMC)
SMC is a nonlinear control method designed to handle uncertainty. It forces the system state onto a defined "sliding surface" and keeps it there.
*   **Sliding Surface**: $s = \dot{e} + \lambda e$
*   **Control Law**: $u = u_{eq} - k \cdot \text{sign}(s)$
*   **Why use it?**: It is extremely robust to external wind/disturbances.

### 3. Model Predictive Control (MPC)
MPC solves an optimization problem at every time step to find the best sequence of control inputs.
*   **Model**: Linearized double integrator $p_{k+1} = p_k + v_k \Delta t$.
*   **Cost Function**: Minimize $\sum (x - x_{ref})^2 + \sum u^2$ (Minimize error and fuel usage).
*   **Why use it?**: It can predict future turns and act early ("anticipation"), providing very smooth tracking.

---

## Usage

The entire system is controlled via the `scripts/run.py` script.

### 1. Visual Simulation (GUI)
Run a simulation with the 3D visualizer to see the drone fly.
```bash
# Run SMC controller on a Figure-8 trajectory
python scripts/run.py --controller smc --trajectory eight --gui
```

### 2. Benchmarking (Headless)
Run simulations for multiple controllers without the GUI to gather data quickly.
```bash
# Compare PID, SMC, and MPC on a Circle trajectory
python scripts/run.py --controller pid smc mpc --trajectory circle --no-gui
```

### 3. Command Options
| Flag | Description | Example |
| :--- | :--- | :--- |
| `--controller` | Which controller(s) to use (`pid`, `smc`, `mpc`) | `--controller pid smc` |
| `--trajectory` | Path shape (`circle`, `eight`, `square`, `step`) | `--trajectory eight` |
| `--duration` | How long to fly (seconds) | `--duration 15.0` |
| `--gui` / `--no-gui` | Toggle 3D visualization | `--gui` |

---

## Results & Analysis

We compared controllers on a standardized Figure-8 trajectory.

### Tracking Performance (RMSE)
*Lower is better.*

| Controller | RMSE (m) | Characteristics |
| :--- | :--- | :--- |
| **SMC** | **0.0120** | **Best Accuracy**. Excellent handling of turns. |
| **MPC** | 0.0130 | Very smooth, optimal inputs. |
| **PID** | 0.0343 | Reliable, but suffers from lag in fast turns. |

### Visual Comparison
![Figure-8 Tracking](docs/assets/results_eight.png)
*Figure 1: Top-down view of trajectory tracking. SMC (Green) hugs the reference (Black) tightly.*

![Figure-8 Error](docs/assets/error_eight.png)
*Figure 2: Error over time. PID (Blue) has consistent lag error.*

---

## Contributing

Contributions are welcome! Please follow these steps:
1.  Fork the repo.
2.  Create a branch: `git checkout -b feature-new-controller`.
3.  Commit changes: `git commit -m "Add Backstepping controller"`.
4.  Push to branch: `git push origin feature-new-controller`.
5.  Submit a Pull Request.

## Future Work

*   Implementation of **Non-linear MPC (NMPC)** to handle full $SO(3)$ dynamics.
*   **Domain Randomization** for more robust Sim-to-Real transfer.
*   **Hardware-in-the-loop** verification on a physical Crazyflie fleet.
*   Integration with **ROS2** for swarm control.

## Citation

If you use this work in your research or studies, please cite:
> Shaik, Kalesha. "Crazyflie Controller Benchmark Framework". 2025.

**Author**: [Kalesha Shaik](https://www.linkedin.com/in/kalesha681)