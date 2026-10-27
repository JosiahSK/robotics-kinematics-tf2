# Lab 03: Forward Kinematics Implementation & Verification

## 1. Laboratory Objectives
In this laboratory, you will:
1. Implement a complete standard Denavit–Hartenberg forward kinematics solver in Python.
2. Build an automated unit test suite using `pytest` to verify position and orientation accuracy.
3. Validate rotation matrix orthogonality ($\mathbf{R}^T \mathbf{R} = \mathbf{I}$) and determinant ($\det(\mathbf{R}) = +1.0$) across 100 random joint configurations.
4. Extract end-effector Cartesian coordinates and visualize link positions.

---

## 2. Theoretical Framework
Recall the standard Denavit–Hartenberg transformation matrix:

$$\mathbf{A}_i = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \\
\sin\theta_i & \cos\theta_i \cos\alpha_i & -\cos\theta_i \sin\alpha_i & a_i \sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

---

## 3. Laboratory Tasks

### Task 1: Complete the DH Matrix Function
Review your implementation in [`04-python-robotics/dh_transform.py`](../04-python-robotics/dh_transform.py).
Verify that the function supports both radian and degree inputs:
```python
A = dh_matrix(theta=90.0, d=0.4, a=0.5, alpha=0.0, in_degrees=True)
```

### Task 2: Implement Multi-Link Chain Calculator
Write a function `compute_forward_kinematics(dh_table, joint_angles)` that iterates through all links and outputs:
- The total transformation matrix $T_0^n$.
- A list of all intermediate joint origin positions $[p_0, p_1, \dots, p_n]$ (for wireframe plotting).

### Task 3: Automated Orthogonality Test
Execute the test suite in `04-python-robotics/tests/`:
```bash
pytest -v 04-python-robotics/tests/test_fk.py
```
Ensure all tests pass.

---

## 4. Verification Checkpoints
| Configuration $\mathbf{q}$ (deg) | Expected End-Effector $(x, y, z)$ | Expected $\det(\mathbf{R})$ | Status |
| :--- | :--- | :--- | :--- |
| $[0, 0, 0, 0, 0, 0]$ (Home) | $(0.500, 0.000, -0.150)\,\text{m}$ | $+1.0000$ | Verified |
| $[90, 0, 0, 0, 0, 0]$ | $(0.000, 0.500, -0.150)\,\text{m}$ | $+1.0000$ | Verified |
| $[0, 45, -45, 0, 0, 0]$ | Re-calculated via Python script | $+1.0000$ | Verified |

---

## 5. Lab Deliverables
1. Python script computing forward kinematics for custom link lengths.
2. Automated test report showing 100% test pass status.
3. 3D wireframe plot rendered via `visualise_robot_arm.py`.
