# Lab 05: 6-DOF Industrial Arm Kinematics & 3D Visualization

> **Prerequisites:** [08: 6-DOF Arm Kinematics Case Study](../03-robot-kinematics/08-six-dof-arm.md), [05: Denavit–Hartenberg Convention](../03-robot-kinematics/05-denavit-hartenberg.md), Python 3.10+.  
> **Estimated Time:** 60–75 minutes  
> **Code Files:**
> - Starter / Reference: [`04-python-robotics/inverse_kinematics_6dof.py`](../04-python-robotics/inverse_kinematics_6dof.py), [`04-python-robotics/forward_kinematics_6dof.py`](../04-python-robotics/forward_kinematics_6dof.py)
> - 3D Visualizer: [`04-python-robotics/visualise_robot_arm.py`](../04-python-robotics/visualise_robot_arm.py)
> - Unit Test Suite: [`04-python-robotics/tests/test_ik.py`](../04-python-robotics/tests/test_ik.py)

---

## 1. Laboratory Objectives
In this laboratory, you will:
1. Model the complete kinematics of an anthropomorphic 6-DOF robot arm (**EduArm-6DOF**).
2. Implement analytical spherical wrist decoupling (Pieper's criterion) in Python.
3. Solve for all 6 joint angles $(\theta_1, \dots, \theta_6)$ given a target Cartesian position and orientation matrix.
4. Render the articulated robot in 3D using Matplotlib (`visualise_robot_arm.py`).
5. Validate analytical IK solutions against forward kinematics re-evaluation.

---

## 2. Decoupling Geometry (Pieper's Criterion)

For a 6-DOF arm with a 3-axis intersecting spherical wrist (Joints 4, 5, 6), inverse kinematics decouples into two 3-DOF sub-problems:

```
[Target Pose (p, R)]
         │
         ▼
[Step 1: Isolate Wrist Center] ──> p_wc = p - d6 * R[:, 2]
         │
         ▼
[Step 2: Solve Position Angles] ──> theta1, theta2, theta3
         │
         ▼
[Step 3: Compute Intermediate R_0^3]
         │
         ▼
[Step 4: Solve Wrist Angles] ────> R_3^6 = (R_0^3)^T * R ──> theta4, theta5, theta6
```

1. **Wrist Center Isolation:**
   $$\mathbf{p}_{wc} = \mathbf{p} - d_6 \mathbf{R} \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$$
2. **Position Kinematics:** Joints 1, 2, and 3 place the wrist center at $\mathbf{p}_{wc} = [x_{wc}, y_{wc}, z_{wc}]^T$.
   - $\theta_1 = \operatorname{atan2}(y_{wc}, x_{wc})$
   - With link twist $\alpha_3 = +\pi/2$, the shoulder-elbow loop yields $\sin(\theta_3)$ and corresponding elbow-up / elbow-down branches.
3. **Wrist Orientation Kinematics:** Joints 4, 5, and 6 orient the tool flange:
   $$\mathbf{R}_3^6 = (\mathbf{R}_0^3)^T \mathbf{R}_d$$
   where $\theta_5 = \pm \arccos(R_{33})$ and $\theta_4, \theta_6$ are extracted from off-diagonal elements.

---

## 3. Step-by-Step Task Sequence & Run Commands

### Task 1: Execute Forward Kinematics for Benchmark Configuration
Run the 6-DOF forward kinematics module:
```bash
python 04-python-robotics/forward_kinematics_6dof.py
```

Test configuration $\mathbf{q} = [25^\circ, 35^\circ, -30^\circ, 15^\circ, -40^\circ, 20^\circ]$:
```python
import numpy as np
from forward_kinematics_6dof import RobotArm6DOF
from dh_transform import extract_pose

arm = RobotArm6DOF()
q_test = np.radians([25.0, 35.0, -30.0, 15.0, -40.0, 20.0])
T_target, _ = arm.forward_kinematics(q_test)
pos, rot = extract_pose(T_target)

print("Target Position (m):", np.round(pos, 4))
print("Target Rotation Matrix:\n", np.round(rot, 4))
```

#### Expected Output:
```text
Target Position (m): [0.5188 0.2419 0.5898]
Target Rotation Matrix:
 [[ 0.8139 -0.1105 -0.5704]
  [-0.1906 -0.9782 -0.0824]
  [-0.5488  0.1758 -0.8172]]
```

### Task 2: Solve Analytical Inverse Kinematics
Use `analytical_ik_spherical_wrist` to solve for the original configuration:
```python
from inverse_kinematics_6dof import analytical_ik_spherical_wrist

solutions = analytical_ik_spherical_wrist(arm, pos, rot)
print(f"Found {len(solutions)} valid analytical IK solutions.")
for i, sol in enumerate(solutions):
    print(f"Solution {i+1} (deg):", np.round(np.degrees(sol), 2))
```

### Task 3: Render 3D Visualization
Generate a 3D visualization showing the robot links, joint axes, and TCP:
```bash
python 04-python-robotics/visualise_robot_arm.py --angles 25 35 -30 15 -40 20 --no-show --save eduarm_3d.png
```

### Task 4: Run Automated Unit Tests
```bash
pytest 04-python-robotics/tests/test_ik.py -k "test_6dof_analytical" -v
```

#### Expected Output:
```text
test_ik.py::test_6dof_analytical_ik_known_config PASSED
test_ik.py::test_6dof_analytical_ik_fk_consistency PASSED
test_ik.py::test_6dof_analytical_ik_two_branches PASSED
test_ik.py::test_6dof_analytical_ik_unreachable PASSED
test_ik.py::test_6dof_analytical_ik_orientation_accuracy PASSED
```

---

## 4. Verification Checkpoints
- [ ] At least one analytical solution matches the ground-truth angles $[25, 35, -30, 15, -40, 20]^\circ$ within $< 0.05^\circ$.
- [ ] Re-evaluating forward kinematics on all returned solutions reproduces the target TCP position within $1.0\,\text{mm}$.
- [ ] Orientation error $\Delta \mathbf{R} = \mathbf{R}^T \mathbf{R}_d$ satisfies $\|\mathbf{e}_o\| < 0.01\,\text{rad}$.

---

## 5. Extension Task: Wrist Singularity Analysis
Configure the robot at $\theta_5 = 0^\circ$ (e.g., $\mathbf{q} = [0, 30^\circ, -30^\circ, 45^\circ, 0^\circ, -45^\circ]$):
- Why does $\sin(\theta_5) = 0$ cause rank loss in extracting $\theta_4$ and $\theta_6$?
- In this singular state, Joint 4 and Joint 6 rotate about the exact same physical axis ($Z_3 \parallel Z_5$).
- Show how the solver sets $\theta_4 = 0$ and bundles the remaining rotation into $\theta_6 = \operatorname{atan2}(R_{21}, R_{11})$.

---

## 6. Lab Deliverables
Submit:
1. Console log verifying recovery of ground-truth joint angles from Cartesian pose.
2. Saved 3D visualization `eduarm_3d.png`.
3. Short analytical write-up explaining why the wrist center isolation equation $\mathbf{p}_{wc} = \mathbf{p} - d_6 \mathbf{R} \hat{\mathbf{z}}$ fails if the wrist axes do not intersect at a common point.
