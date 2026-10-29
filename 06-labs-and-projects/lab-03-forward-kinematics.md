# Lab 03: Forward Kinematics Implementation & Verification

> **Prerequisites:** [05: Denavit–Hartenberg Convention](../03-robot-kinematics/05-denavit-hartenberg.md), [06: Forward Kinematics](../03-robot-kinematics/06-forward-kinematics.md), Python 3.10+.  
> **Estimated Time:** 45–60 minutes  
> **Code Files:**
> - Starter / Reference: [`04-python-robotics/forward_kinematics_6dof.py`](../04-python-robotics/forward_kinematics_6dof.py)
> - Unit Test Suite: [`04-python-robotics/tests/test_fk.py`](../04-python-robotics/tests/test_fk.py)

---

## 1. Laboratory Objectives
In this laboratory, you will:
1. Implement a complete standard Denavit–Hartenberg forward kinematics solver in Python.
2. Build an automated unit test suite using `pytest` to verify position and orientation accuracy.
3. Validate rotation matrix orthogonality ($\mathbf{R}^T \mathbf{R} = \mathbf{I}$) and determinant ($\det(\mathbf{R}) = +1.0$) across arbitrary joint configurations.
4. Extract end-effector Cartesian coordinates and compare with geometric ground-truth poses.

---

## 2. Theoretical Framework
Recall the standard Denavit–Hartenberg transformation matrix $\mathbf{A}_i$:

$$\mathbf{A}_i = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \\
\sin\theta_i & \cos\theta_i \cos\alpha_i & -\cos\theta_i \sin\alpha_i & a_i \sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

For an $n$-DOF chain, the overall tool transformation is:
$$\mathbf{T}_0^n(\mathbf{q}) = \mathbf{A}_1(q_1) \mathbf{A}_2(q_2) \dots \mathbf{A}_n(q_n)$$

---

## 3. Step-by-Step Task Sequence & Run Commands

### Task 1: Inspect the Standard DH Transformation Module
Open and review [`04-python-robotics/dh_transform.py`](../04-python-robotics/dh_transform.py).
Verify the signature and degree-to-radian conversion logic:
```python
import numpy as np
from dh_transform import dh_matrix, extract_pose

# Compute A1 for Link 1 (d1=0.4m, alpha=+pi/2)
A1 = dh_matrix(theta=45.0, d=0.400, a=0.0, alpha=90.0, in_degrees=True)
pos, rot = extract_pose(A1)
print("Position:", pos)
print("det(R):", np.linalg.det(rot))
```

### Task 2: Compute Known Geometric Benchmark Poses
Using `RobotArm6DOF` from [`04-python-robotics/forward_kinematics_6dof.py`](../04-python-robotics/forward_kinematics_6dof.py), calculate the TCP pose for canonical reference postures:

```python
from forward_kinematics_6dof import RobotArm6DOF
arm = RobotArm6DOF()

# 1. Arm outstretched horizontally along +X:
T_horiz, _ = arm.forward_kinematics(np.radians([0, 0, 90, 0, 0, 0]))
print("Outstretched +X TCP:", np.round(T_horiz[:3, 3], 4))

# 2. Arm pointing straight up along +Z:
T_up, _ = arm.forward_kinematics(np.radians([0, 90, 90, 0, 0, 0]))
print("Pointing +Z TCP:", np.round(T_up[:3, 3], 4))
```

### Task 3: Execute Automated Unit Tests
Run the automated pytest suite:
```bash
pytest 04-python-robotics/tests/test_fk.py -v
```

#### Expected Output:
```text
test_fk.py::test_2link_planar_fk PASSED
test_fk.py::test_6dof_arm_home_position PASSED
test_fk.py::test_6dof_arm_random_configurations_orthogonality PASSED
test_fk.py::test_6dof_known_configurations PASSED
test_fk.py::test_urdf_dh_parameter_consistency PASSED
test_fk.py::test_urdf_fk_numerical_agreement PASSED
```

---

## 4. Verification Checkpoints

| Configuration $\mathbf{q}$ (deg) | Geometric Description | Expected End-Effector $(x, y, z)$ | Expected $\det(\mathbf{R})$ |
| :--- | :--- | :--- | :--- |
| $[0, 0, 0, 0, 0, 0]$ | Home configuration | $(0.500, 0.000, -0.150)\,\text{m}$ | $+1.0000$ |
| $[0, 0, 90, 0, 0, 0]$ | Horizontally outstretched $+X$ | $(1.050, 0.000, 0.400)\,\text{m}$ | $+1.0000$ |
| $[90, 0, 90, 0, 0, 0]$ | Horizontally outstretched $+Y$ | $(0.000, 1.050, 0.400)\,\text{m}$ | $+1.0000$ |
| $[0, 90, 90, 0, 0, 0]$ | Pointing vertically $+Z$ | $(0.000, 0.000, 1.450)\,\text{m}$ | $+1.0000$ |

---

## 5. Extension Task: Multi-Configuration Workspace Point Cloud
Write a script that samples 1,000 random joint configurations within physical limits:
```python
q_samples = np.random.uniform(arm.joint_limits[:, 0], arm.joint_limits[:, 1], size=(1000, 6))
```
- Evaluate FK for all samples and extract the TCP $(x, y, z)$ positions.
- Generate a 3D scatter plot of reachable positions.
- Verify that no point exceeds the maximum radial reach $r_{\max} = \sqrt{a_2^2 + (d_4 + d_6)^2} + d_1 \approx 1.45\,\text{m}$.

---

## 6. Lab Deliverables
Submit:
1. Python script computing forward kinematics for custom link lengths.
2. Automated test report showing all forward kinematics unit tests passing.
3. 3D workspace scatter plot generated in the extension task.
