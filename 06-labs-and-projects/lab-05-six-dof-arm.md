# Lab 05: 6-DOF Industrial Arm Kinematics & 3D Visualization

## 1. Laboratory Objectives
In this laboratory, you will:
1. Model the complete kinematics of an articulated 6-DOF robot arm (**EduArm-6DOF**).
2. Implement the analytical spherical wrist decoupling method.
3. Solve for all 6 joint angles $(\theta_1, \dots, \theta_6)$ given a 3D target position and orientation.
4. Render the articulated arm in 3D using Matplotlib (`visualise_robot_arm.py`).

---

## 2. Decoupling Geometry
Recall the wrist-centre equation from [Module 03 Chapter 08](../03-robot-kinematics/08-six-dof-arm.md):
$$\mathbf{p}_{wc} = \mathbf{p}_d - d_6 \mathbf{a}_d$$
where $\mathbf{a}_d$ is the 3rd column of the target rotation matrix $\mathbf{R}_d[:, 2]$.

1. Joints 1, 2, and 3 place the wrist center at $\mathbf{p}_{wc}$.
2. Joints 4, 5, and 6 orient the tool flange via $\mathbf{R}_3^6 = (\mathbf{R}_0^3)^T \mathbf{R}_d$.

---

## 3. Laboratory Tasks

### Task 1: Forward Kinematics Verification
Run the 6-DOF forward kinematics module:
```bash
python 04-python-robotics/forward_kinematics_6dof.py
```
Record the computed end-effector position and orientation for:
- Home Pose: $\mathbf{q} = [0, 0, 0, 0, 0, 0]$
- Articulated Pose: $\mathbf{q} = [30^\circ, 45^\circ, -30^\circ, 20^\circ, -45^\circ, 15^\circ]$

### Task 2: 3D Visualization
Run the 3D visualizer to inspect the robot pose:
```bash
python 04-python-robotics/visualise_robot_arm.py --angles 30 45 -30 20 -45 15 --no-show --save eduarm_3d.png
```
Inspect `eduarm_3d.png` to confirm:
- The base pedestal origin is at $(0, 0, 0)$.
- The red, green, and blue axes denote local $X, Y, Z$ frames at each joint.
- The end-effector reaches the target coordinates.

### Task 3: Unit Testing
Run the automated test suite:
```bash
pytest -v 04-python-robotics/tests/test_fk.py -k "test_6dof"
```

---

## 4. Lab Deliverables
1. Saved 3D plot `eduarm_3d.png` displaying the robot in a custom configuration.
2. Verified table comparing analytical forward kinematics results against RViz / Matplotlib coordinates.
