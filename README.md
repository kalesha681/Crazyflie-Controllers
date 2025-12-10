# Comparative Analysis of Control Strategies for Quadrotors

![Python](https://img.shields.io/badge/python-3.8+-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)
![Status](https://img.shields.io/badge/status-research_active-success.svg)
![CI](https://github.com/kalesha681/Crazyflie-Controllers/actions/workflows/ci.yml/badge.svg)

**[📄 Read the Full Research Report](REPORT.md)**

## 🎯 Research Objective
This repository serves as a **control systems research testbed** to strictly evaluate and compare linear vs. non-linear control architectures for Micro Aerial Vehicles (MAVs). The project was developed to analyze the trade-offs between computational efficiency, robustness, and tracking accuracy in dynamic flight regimes.

**Key Research Questions:**
1.  Can **Sliding Mode Control (SMC)** effectively mitigate unmodeled dynamics compared to PID?
2.  Does **Linear Model Predictive Control (LMPC)** provide superior trajectory tracking through lookahead, despite linear approximations?

## 🧪 Control Architectures
Implementations verified on a **Bitcraze Crazyflie 2.1** (PyBullet Simulation):

1.  **Cascaded PID (Baseline)**: Standard nested loops for position and attitude.
2.  **Cascaded SMC (Robust)**: 6-DOF Sliding Mode Controller using geometric attitude control on $SO(3)$.
3.  **Linear MPC (Optimal)**: Recursive feasibility-constrained Quadratic Programming (QP) solver using `cvxpy/OSQP` with multi-rate planning.

## 📊 Performance Benchmarks
*Results from Figure-8 Trajectory (12s period)*

| Controller | RMSE (m) | Characteristics |
| :--- | :--- | :--- |
| **PID** | 0.0343 | High lag, poor disturbance rejection. |
| **MPC** | 0.0130 | Predictive turn-in, fast rise time ($t_r=0.35s$). |
| **SMC** | **0.0120** | **Best Performance**, zero overshoot ($M_p < 0.1\%$). |

See [REPORT.md](REPORT.md) for detailed Step Response analysis and methodology.

## 🛠️ Usage

### 1. Installation
```bash
git clone https://github.com/kalesha681/Crazyflie-Controllers.git
cd Crazyflie-Controllers
pip install -r requirements.txt
```

### 2. Run Comparative Analysis
Execute the full verification suite (PID vs SMC vs MPC):
```bash
# Verify on Eight Trajectory (Tracking Performance)
python3 -m src.compare_controllers --trajectory eight --no-gui

# Verify Step Response (Transient Dynamics)
python3 -m src.compare_controllers --trajectory step --no-gui
```
Plots are saved to `outputs/plots/`.

### 3. Individual Controller Testing
```bash
# Run MPC on Triangle
python3 -m src.run_tracking --controller mpc --trajectory triangle
```

## 📂 Repository Structure
```
crazyflie-controllers/
├── REPORT.md              # Academic Report (Abstract, Methodology, Results)
├── src/
│   ├── Controllers/       # Implementation details (SMC, MPC, PID)
│   ├── trajectories/      # Trajectory generation (Step, Eight, etc.)
│   └── compare_controllers.py  # Main comparative analysis script
├── outputs/
│   ├── plots/             # Performance graphs
│   └── data/              # CSV log data
└── README.md              # Overview
```

## 📝 License
MIT License.
