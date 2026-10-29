# Lab 06: Jacobian Inverse Kinematics & Singularity Handling

> **Prerequisites:** [09: Manipulator Jacobian](../03-robot-kinematics/09-jacobian-kinematics.md), [10: Singularities & Manipulability](../03-robot-kinematics/10-singularities-and-joint-limits.md), Python 3.10+.  
> **Estimated Time:** 60–75 minutes  
> **Code Files:**
> - Starter / Reference: [`04-python-robotics/inverse_kinematics_6dof.py`](../04-python-robotics/inverse_kinematics_6dof.py), [`04-python-robotics/jacobian.py`](../04-python-robotics/jacobian.py)
> - Unit Test Suite: [`04-python-robotics/tests/test_ik.py`](../04-python-robotics/tests/test_ik.py), [`04-python-robotics/tests/test_jacobian.py`](../04-python-robotics/tests/test_jacobian.py)

---

## 1. Laboratory Objectives
In this laboratory, you will:
1. Implement a complete numerical inverse kinematics solver for a 6-DOF robot using the Geometric Jacobian.
2. Formulate 3D orientation error using the exact $SO(3)$ logarithmic map with near-$180^\circ$ symmetric matrix extraction.
3. Implement Levenberg–Marquardt Damped Least Squares (DLS) evaluated via Singular Value Decomposition (SVD) to prevent numerical explosions near singularities.
4. Enforce physical joint limits at every iteration step.
5. Track condition number $\kappa(\mathbf{J})$ and Yoshikawa manipulability $w$ throughout convergence.

---

## 2. Mathematical Algorithm
At each iteration $k$:
1. **Forward Kinematics:** Compute current tool pose $\mathbf{T}(\mathbf{q}_k)$ with position $\mathbf{p}_k$ and rotation $\mathbf{R}_k$.
2. **Position Error:** $\mathbf{e}_p = \mathbf{p}_d - \mathbf{p}_k$.
3. **Orientation Error ($SO(3)$ Log-Map):**
   $$\mathbf{R}_{\text{err}} = \mathbf{R}_k^T \mathbf{R}_d, \quad \theta = \arccos\left(\frac{\operatorname{trace}(\mathbf{R}_{\text{err}}) - 1}{2}\right)$$
   $$\mathbf{e}_o = \frac{\theta}{2 \sin\theta} \begin{bmatrix} R_{32} - R_{23} \\ R_{13} - R_{31} \\ R_{21} - R_{12} \end{bmatrix}$$
4. **Convergence Check:** If $\|\mathbf{e}_p\| < \epsilon_p$ and $\|\mathbf{e}_o\| < \epsilon_o$, stop (converged).
5. **Geometric Jacobian:** Compute $6 \times 6$ matrix $\mathbf{J}(\mathbf{q}_k)$.
6. **SVD-Based DLS Step:** Compute SVD $\mathbf{J} = \mathbf{U} \boldsymbol{\Sigma} \mathbf{V}^T$, then:
   $$\Delta \mathbf{q} = \mathbf{V} \operatorname{diag}\left( \frac{\sigma_i}{\sigma_i^2 + \lambda^2} \right) \mathbf{U}^T \begin{bmatrix} \mathbf{e}_p \\ \mathbf{e}_o \end{bmatrix}$$
7. **Joint Limit Clamping:** Update configuration:
   $$\mathbf{q}_{k+1} = \operatorname{clip}\left(\mathbf{q}_k + \Delta \mathbf{q}, \; \mathbf{q}_{\min}, \; \mathbf{q}_{\max}\right)$$

---

## 3. Step-by-Step Task Sequence & Run Commands

### Task 1: Run the Numerical Solver Benchmark
Execute the 6-DOF numerical IK solver in [`04-python-robotics/inverse_kinematics_6dof.py`](../04-python-robotics/inverse_kinematics_6dof.py):
```bash
python 04-python-robotics/inverse_kinematics_6dof.py
```

#### Expected Output:
```text
=== Numerical IK Solver Test ===
Ground-Truth Joint Angles (deg): [ 25.  35. -30.  15. -40.  20.]
Initial Guess (deg):             [ 33.6  26.4 -21.4  23.6 -31.4  28.6]
Convergence Status:              Success!
Iterations Required:             12
Final Position Error:            0.0004 m
Final Orientation Error:         0.0031 rad
Converged Solution (deg):        [ 25.00  35.00 -30.00  15.00 -40.00  20.00]
```

### Task 2: Singularity Stress Test (Pseudoinverse vs DLS)
Create a script testing a near-singular configuration ($\theta_5 \to 0$):
```python
import numpy as np
from jacobian import damped_least_squares_inverse

# Construct rank-deficient singular Jacobian (wrist singularity)
J = np.zeros((6, 6))
J[0, 0] = 1.0
J[1, 1] = 0.8
J[2, 2] = 0.5
J[3, 3] = 1.0
J[4, 4] = 1e-7  # Near-zero singular value!

# Test standard pseudoinverse vs DLS
J_pinv = np.linalg.pinv(J)
J_dls = damped_least_squares_inverse(J, damping=0.05)

print("Standard pinv max element:", np.max(np.abs(J_pinv)))  # Blows up to ~1e7!
print("DLS inverse max element:  ", np.max(np.abs(J_dls)))   # Well-bounded at ~10.0!
```

### Task 3: Automated Unit Testing
Run the complete automated test suite for numerical IK and Jacobians:
```bash
pytest 04-python-robotics/tests/test_ik.py -k "test_6dof_numerical" -v
pytest 04-python-robotics/tests/test_jacobian.py -v
```

#### Expected Output:
```text
test_ik.py::test_6dof_numerical_ik_convergence PASSED
test_ik.py::test_6dof_numerical_ik_unreachable_target PASSED
test_ik.py::test_6dof_joint_limits_respected PASSED
test_jacobian.py::test_jacobian_dimensions PASSED
test_jacobian.py::test_jacobian_vs_numerical_differences PASSED
test_jacobian.py::test_manipulability_properties PASSED
test_jacobian.py::test_dls_inverse_stability PASSED
test_jacobian.py::test_dls_uses_svd_robustness PASSED
```

---

## 4. Verification Checkpoints
- [ ] Solver converges within 30 iterations from initial guesses within $\pm 15^\circ$ of the true configuration.
- [ ] Final position error is strictly below $1.0\,\text{mm}$ and orientation error below $0.01\,\text{rad}$.
- [ ] Unreachable targets gracefully report failure without crashing or generating `NaN` values.
- [ ] All joint updates respect configured joint hardstops $\mathbf{q}_{\min} \le \mathbf{q} \le \mathbf{q}_{\max}$.

---

## 5. Extension Task: Manipulability Along Cartesian Trajectories
Write a script that drives the end-effector along a straight line in Cartesian space:
- At each trajectory waypoint, compute the Geometric Jacobian $\mathbf{J}$.
- Calculate the **Yoshikawa manipulability measure**: $w = \sqrt{\det(\mathbf{J}\mathbf{J}^T)} = \prod_{i=1}^6 \sigma_i$.
- Calculate the **condition number**: $\kappa(\mathbf{J}) = \frac{\sigma_{\max}}{\sigma_{\min}}$.
- Plot $w(t)$ and $\kappa(t)$ over time. Identify points along the path where manipulability dips toward zero.

---

## 6. Lab Deliverables
Submit:
1. Error convergence log showing $\|\mathbf{e}_p\|$ and $\|\mathbf{e}_o\|$ decreasing per iteration.
2. Comparative table showing condition numbers and joint step magnitudes with and without DLS damping.
3. Trajectory manipulability plot from the extension task.
