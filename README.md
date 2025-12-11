# Crazyflie Controllers Benchmark Framework

![CI](https://github.com/kalesha681/Crazyflie-Controllers/actions/workflows/ci.yml/badge.svg)
![Python](https://img.shields.io/badge/python-3.8+-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)

A reproducible, modular research framework for benchmarking linear and non-linear control architectures on the Bitcraze Crazyflie 2.1 quadrotor.

**[📄 Read the Full Research Report](REPORT.md)**

## 🎯 Research Objective
To rigorously evaluate the performance trade-offs between:
1.  **Cascaded PID** (Baseline, Linear)
2.  **Sliding Mode Control (SMC)** (Robust, Non-linear)
3.  **Linear Model Predictive Control (MPC)** (Optimal, Predictive)

Key metrics include Root Mean Square Error (RMSE), control effort, and step response characteristics (Rise Time, Overshoot) under simulated dynamic conditions.

## 🏗️ Architecture
This repository follows a strict `src/package` layout for reproducibility and abstraction.

```
crazyflie_controllers/
├── scripts/                # Execution and Analysis scripts
│   ├── run_tracking.py     # Single simulation runner
│   └── compare_controllers.py # Batch benchmark runner
├── src/crazyflie_controllers/
│   ├── controllers/        # Control Algorithms (Strict Interface)
│   │   ├── pid_controller.py
│   │   ├── smc_controller.py
│   │   └── mpc_controller.py
│   ├── trajectories/       # Geometric Reference Generators
│   │   ├── circle.py
│   │   ├── eight.py
│   │   └── ...
│   └── utils/              # Logging and Signal Processing
│       └── logging.py
├── outputs/                # Generated Data and Plots
└── tests/                  # Unit Tests
```

### Abstraction Contracts
*   **Controllers** implement `BaseController`: `compute_control(state, ref, dt) -> rpm`.
*   **Trajectories** implement `BaseTrajectory`: `get_target(t) -> (pos, vel, acc, yaw)`.
*   **Logging** enforces a strict CSV schema ensuring `time, x, y, z, ...` column uniformity across experiments.

## 🚀 Quick Start

### Installation
```bash
git clone https://github.com/kalesha681/Crazyflie-Controllers.git
cd Crazyflie-Controllers
pip install -r requirements.txt
```

### Running Simulations

**1. Single Trajectory Tracking**
Visualize a specific controller on a specific trajectory.
```bash
# General Usage
python scripts/run.py --controller [pid|smc|mpc] --trajectory [circle|eight|...]
```

**1. Single Trajectory Tracking**
Visualize a specific controller on a specific trajectory.
```bash
python scripts/run.py --controller smc --trajectory eight
```

**2. Full Benchmark Comparison**
(Note: Benchmark script has been consolidated. Use `run.py` purely for simulation/data gen, then analyze outputs).
To generate data for all:
```bash
python scripts/run.py --controller pid --trajectory eight --no-gui
python scripts/run.py --controller smc --trajectory eight --no-gui
python scripts/run.py --controller mpc --trajectory eight --no-gui
```

## 📊 Results Summary
*Average RMSE on Figure-8 Trajectory (12s period)*

| Controller | RMSE (m) | Characteristics |
| :--- | :--- | :--- |
| **PID** | 0.0343 | Easy to tune, but lags dynamic reference. |
| **MPC** | 0.0130 | Excellent tracking, computationally expensive. |
| **SMC** | **0.0120** | **Best Performance**, robust to disturbances. |

## 🧪 Controller Mathematical Formulations

### 1. PID (Proportional-Integral-Derivative)
Standard cascaded architecture: Outer position loop outputs desired velocity/attitude, Inner attitude loop outputs motor commands.

### 2. SMC (Sliding Mode Control)
Defines a sliding surface $s = \dot{e} + \lambda e$. Control law $u = -k \cdot \text{sgn}(s)$ forces dynamics onto this surface, providing exponential convergence and robustness to model mismatches.

### 3. Linear MPC (Model Predictive Control)
Solves a constrained Quadratic Program (QP) at each time step:
$$
\min_{u} \sum_{k=0}^{N} (x_k - x_{ref})^T Q (x_k - x_{ref}) + u_k^T R u_k
$$
Subject to linear dynamics $x_{k+1} = Ax_k + Bu_k$ and actuator constraints.

## 🛠️ Reproducibility
Random seeds are fixed (`np.random.seed(42)`) in all scripts to guarantee identical results on every run.
All plots are generated strictly from the logged CSV data in `outputs/data/`, ensuring analysis integrity.

## ⚠️ Disclaimer
Simulations run in `gym-pybullet-drones` (PyBullet physics engine). While the Crazyflie model is accurate, real-world aerodynamics (ground effect, drag) may differ.

---
*Author: Kalesha Shaik*
Connect me on : [LinkedIn](www.linkedin.com/in/kalesha681)