**Author**: Kalesha Shaik  
**Date**: December 2025  

---

## 1. Abstract
This project implements and evaluates three control architectures—**Proportional-Integral-Derivative (PID)**, **Sliding Mode Control (SMC)**, and **Linear Model Predictive Control (LMPC)**—for precise trajectory tracking of a Crazyflie 2.1 nano-quadrotor. The controllers were verified in a high-fidelity PyBullet simulation environment across complex 3D trajectories (Circle, Square, Triangle, Figure-8). Results demonstrate that the 6-DOF Cascaded SMC achieves sub-centimeter tracking accuracy (RMSE $\approx$ 1.2 cm), outperforming the LMPC (RMSE $\approx$ 1.3 cm) and baseline PID (RMSE $\approx$ 3.4 cm) under realized simulation conditions.

## 2. Methodology

### 2.1. System Model
The quadrotor is modeled as a 6-DOF rigid body. The state vector is $x = [p, q, v, \omega]^T$, where $p \in \mathbb{R}^3$ is position, $q \in \mathbb{H}$ is attitude (quaternion), $v \in \mathbb{R}^3$ is linear velocity, and $\omega \in \mathbb{R}^3$ is angular velocity.

### 2.2. Control Architectures

#### A. Baseline: Cascade PID
Standard cascaded architecture with an outer position loop computing desired velocities and an inner attitude loop computing motor mix.
- **Performance**: Adequate for hovering, but exhibits significant lag (RMSE > 3cm) on dynamic trajectories due to lack of feedforward and model knowledge.

#### B. Robust Control: Cascaded SMC
A nonlinear robust controller designed to handle model uncertainties.
- **Outer Loop (Position)**: Sliding surface $s = e_v + \lambda e_p$. Control law $u = -k \cdot \text{sgn}(s)$ ensures finite-time convergence to the surface.
- **Inner Loop (Attitude)**: Geometric DSLPID controller on $SO(3)$.
- **Key Feature**: Robustness to unmodeled dynamics. In this project, it achieved the **lowest RMSE (1.2 cm)** and zero overshoot in step response.

#### C. Optimal Control: Linear MPC (3D)
A predictive controller solving a constrained Quadratic Program (QP) at 240Hz.
- **Model**: Discrete-time double integrator ($p_{k+1} = p_k + v_k dt + 0.5 a_k dt^2$).
- **Cost Function**: $J = \sum_{k=0}^{N} (x_k - x_{ref})^T Q (x_k - x_{ref}) + u_k^T R u_k$
- **Optimization**: Parameterized via `cvxpy` (OSQP solver) with **multi-rate planning** ($dt=0.05s, N=20$) for a 1.0s lookahead.
- **Performance**: Fast rise time (0.35s) but aggressive tuning led to overshoot. Excellent steady-state tracking (RMSE 1.3 cm).

## 3. Results & Analysis

### 3.1. Trajectory Tracking (RMSE)
Tests were conducted on a Figure-8 trajectory ($2 \times 1$ m, 12s period).

| Controller | RMSE (m) | Performance Notes |
| :--- | :--- | :--- |
| **PID** | 0.0343 | Significant lag, corners cut. |
| **MPC** | 0.0130 | Smooth, predictive turn-in. |
| **SMC** | **0.0120** | Near-perfect tracking. |

### 3.2. Step Response (Transient Analysis)
Response to a 1.0m step input in X-position.

| Metric | PID | MPC | SMC |
| :--- | :--- | :--- | :--- |
| **Rise Time** ($t_r$) | 0.41s | **0.35s** | 1.27s |
| **Settling Time** ($t_s$) | 10.5s | 11.0s | **2.2s** |
| **Overshoot** ($M_p$) | 11.2% | 74.2% | **0.08%** |

**Interpretation**:
- **MPC** is tuned for agility (fastest rise), leading to overshoot.
- **SMC** provides a critically damped response (zero overshoot), ideal for precision tasks.

## 4. Conclusion
The implementation confirms that advanced control strategies (SMC, MPC) significantly outperform PID for agile quadrotor flight. The **SMC** proved most effective for the idealized simulation, offering near-perfect tracking. The **MPC** demonstrated predictive capability, successfully anticipating trajectory curvature.

## 5. Future Work
- Implementation of **Nonlinear MPC (NMPC)** using ACADO/CasADi.
- Hardware-in-the-loop (HIL) verification on physical Crazyflie 2.1.
