# Research Report: Comparative Analysis of Control Architectures for MAVs

## Abstract
This study evaluates the tracking performance and robustness of three distinct control architectures—Proportional-Integral-Derivative (PID), Sliding Mode Control (SMC), and Linear Model Predictive Control (MPC)—on a Crazyflie 2.1 nano-quadrotor. Simulations conducted in a high-fidelity PyBullet environment demonstrate that while PID provides a stable baseline, SMC offers superior tracking accuracy (1.2 cm RMSE) and robustness to unmodeled dynamics. MPC achieves competitive accuracy (1.3 cm RMSE) with optimal inputs but requires significantly higher computational resources.

## 1. Methodology

### 1.1 Experimental Setup
*   **Platform**: Bitcraze Crazyflie 2.1 (27g, 33mm prop radius).
*   **Simulator**: `gym-pybullet-drones` (Physics: PyBullet, 240Hz Control Loop).
*   **Trajectories**:
    *   **Square/Triangle**: Tests sharp cornering and transient response.
    *   **Figure-8**: Tests continuous dynamic tracking capacity.
    *   **Step Input**: Tests rise time, overshoot, and steady-state error.

### 1.2 Metrics
*   **Root Mean Square Error (RMSE)**: Global measure of tracking accuracy.
    $$ RMSE = \sqrt{\frac{1}{N} \sum_{i=1}^{N} ||p_{actual} - p_{ref}||^2} $$
*   **Step Response**:
    *   *Rise Time*: Time to reach 90% of setpoint.
    *   *Overshoot*: Percentage by which response exceeds setpoint.
    *   *Steady State Error*: Final discrepancy.

## 2. Controller Design

### 2.1 PID (Baseline)
A cascaded structure is employed:
1.  **Position Loop**: Errors in X/Y/Z generate desired Roll/Pitch/Thrust.
2.  **Attitude Loop**: Errors in Angles generate desired Rate/Moments.
*Tuning approach*: Ziegler-Nichols heuristic followed by manual fine-tuning for aggressive tracking.

### 2.2 Sliding Mode Control (SMC)
Developed to robustly handle non-linearities.
*   **Sliding Surface**: $s = \dot{e} + \lambda e$
*   **Control Law**: $u_{eq} + u_{discontinuous}$ where $u_{discontinuous} = k \tanh(s)$
*   *Advantage*: Guaranteed convergence if gains satisfy Lyapunov stability.

### 2.3 Model Predictive Control (MPC)
A finite-horizon optimal control problem.
*   **Model**: Linearized Double Integrator ($p_{k+1} = p_k + v_k dt$).
*   **Horizon**: 1.0s lookahead (20 steps at 20Hz planning rate).
*   **Solver**: OSQP via `cvxpy` (Warm-started for <5ms solve times).

## 3. Results

### 3.1 Trajectory Tracking (Figure-8)
| Controller | RMSE (m) | Ranking |
| :--- | :--- | :--- |
| **SMC** | **0.0120** | **1st** |
| **MPC** | 0.0130 | 2nd |
| **PID** | 0.0343 | 3rd |

*Analysis*: SMC's high-gain feedback suppresses disturbances effectively, maintaining the tightest track. PID exhibits noticeable lag ("drag error") during fast turns.

### 3.2 Step Response (X-Axis)
| Metric | PID | MPC | SMC |
| :--- | :--- | :--- | :--- |
| **Rise Time** | 0.41s | **0.35s** | 1.27s |
| **Overshoot** | 11.2% | 74.2% | **0.1%** |
| **Settling Time**| 0.8s | 1.5s | 1.8s |

*Analysis*: MPC is aggressive, prioritizing rise time, leading to significant overshoot. SMC is overdamped by design (robustness > agility). PID offers a balanced but oscillatory response.

## 4. Conclusion
For aggressive maneuvering where trajectory fidelity is paramount, **SMC** is the recommended architecture due to its superior RMSE and lack of overshoot. **MPC** is viable if lookahead helps anticipate future constraints, but requires more compute. **PID** remains a robust fallback for less demanding tasks.

## 5. Future Work
*   Implementation of Non-linear MPC (NMPC) to handle full $SO(3)$ dynamics.
*   Hardware-in-the-loop verification on physical Crazyflie fleet.
