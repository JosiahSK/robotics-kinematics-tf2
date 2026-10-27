# 09: Manipulator Jacobian & Velocity Kinematics

## 1. Learning Objectives
By completing this chapter, you will be able to:
- Formulate the Geometric Jacobian matrix for open-chain serial manipulators.
- Compute the linear and angular velocity contributions of each revolute joint.
- Map Cartesian task velocities $\mathbf{\xi} = [\mathbf{v}, \boldsymbol{\omega}]^T$ to joint velocities $\mathbf{\dot{q}}$.
- Apply the Principle of Virtual Work to derive the static force-torque duality $\boldsymbol{\tau} = \mathbf{J}^T \mathbf{F}$.
- Validate analytical Jacobians against numerical central finite differences.

---

## 2. Intuitive Explanation
Forward kinematics tells you where the hand is. But what if the robot is spray painting a car body or tracing a weld seam?
The robot hand must move along a path at a **precise linear speed** (e.g. $0.25\,\text{m/s}$) while maintaining a **fixed angular orientation**.

To achieve this:
- We cannot just set target waypoints and let motors jump.
- We must command the exact instantaneous **angular velocity $\dot{\theta}_i$ for every joint motor**.

The **Geometric Jacobian $\mathbf{J}(\mathbf{q})$** acts as the transmission gearbox between the robot's motors and the tool tip:

```mermaid
graph LR
    JointSpeeds["Joint Velocities dq/dt (Motor Speeds)"] -->|Jacobian J(q)| ToolSpeeds["End-Effector Velocity xi = [v, omega]^T"]
    ToolWrench["External Wrench F = [f, m]^T (Forces & Moments)"] -->|Jacobian Transpose J^T| JointTorques["Motor Torques tau = J^T * F"]
```

---

## 3. Mathematical Derivation of the Geometric Jacobian

For an $n$-joint serial manipulator, the end-effector twist $\mathbf{\xi} \in \mathbb{R}^6$ consists of linear velocity $\mathbf{v}_e \in \mathbb{R}^3$ and angular velocity $\boldsymbol{\omega}_e \in \mathbb{R}^3$:

$$\mathbf{\xi} = \begin{bmatrix} \mathbf{v}_e \\ \boldsymbol{\omega}_e \end{bmatrix} = \mathbf{J}(\mathbf{q}) \mathbf{\dot{q}} = \sum_{i=1}^{n} \mathbf{J}_i \dot{q}_i$$
where $\mathbf{J}_i$ is the $i$-th column of the $6 \times n$ Jacobian matrix.

### Revolute Joint Formulation:
If Joint $i$ is revolute:
1. **Angular Velocity:** Rotating Joint $i$ with speed $\dot{q}_i$ contributes an angular velocity directed along its rotation axis:
   $$\boldsymbol{\omega}_{e, i} = \mathbf{z}_{i-1} \dot{q}_i \implies \mathbf{J}_{\omega, i} = \mathbf{z}_{i-1}$$
2. **Linear Velocity:** A rotation about axis $\mathbf{z}_{i-1}$ passing through point $\mathbf{p}_{i-1}$ creates a linear tangential velocity at the end-effector $\mathbf{p}_e$:
   $$\mathbf{v}_{e, i} = \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1}) \dot{q}_i \implies \mathbf{J}_{v, i} = \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1})$$

Thus, for each revolute joint $i = 1, \dots, n$:
$$\mathbf{J}_i = \begin{bmatrix} \mathbf{J}_{v, i} \\ \mathbf{J}_{\omega, i} \end{bmatrix} = \begin{bmatrix} \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1}) \\ \mathbf{z}_{i-1} \end{bmatrix}$$

where:
- $\mathbf{z}_{i-1}$ is the 3rd column of the rotation matrix $T_0^{i-1}[:3, 2]$ (the joint's rotation axis in base coordinates).
- $\mathbf{p}_{i-1}$ is the translation vector $T_0^{i-1}[:3, 3]$ (the origin of frame $i-1$).
- $\mathbf{p}_e$ is the translation vector of the end-effector $T_0^n[:3, 3]$.

---

## 4. Static Force & Torque Duality (Principle of Virtual Work)

The Jacobian is not only a velocity transmission; it is also a **force transmission**!

Consider a robot maintaining static contact against an obstacle, exerting an external wrench $\mathbf{F} = [\mathbf{f}, \mathbf{m}]^T \in \mathbb{R}^6$ (force $\mathbf{f}$ and moment $\mathbf{m}$).
By the **Principle of Virtual Work**, the virtual work done in joint space equals the virtual work done in Cartesian space:

$$\delta W = \boldsymbol{\tau}^T \delta \mathbf{q} = \mathbf{F}^T \delta \mathbf{x}$$

Substituting the kinematic differential $\delta \mathbf{x} = \mathbf{J} \delta \mathbf{q}$:
$$\boldsymbol{\tau}^T \delta \mathbf{q} = \mathbf{F}^T (\mathbf{J} \delta \mathbf{q}) = (\mathbf{J}^T \mathbf{F})^T \delta \mathbf{q}$$

Since this must hold for any arbitrary virtual joint displacement $\delta \mathbf{q}$:
$$\boldsymbol{\tau} = \mathbf{J}^T(\mathbf{q}) \mathbf{F}$$

> [!TIP]
> This duality is profound:
> - **Motion:** $\mathbf{v} = \mathbf{J} \mathbf{\dot{q}}$ (uses $\mathbf{J}$)
> - **Force:** $\boldsymbol{\tau} = \mathbf{J}^T \mathbf{F}$ (uses $\mathbf{J}^T$)
> If a joint configuration provides high velocity advantage in a certain direction, it provides low mechanical force advantage in that same direction (and vice versa)!

---

## 5. Fully Worked Numerical Example: EduArm-6DOF Column 1

Consider our illustrative EduArm-6DOF at the home position $\mathbf{q} = [0, 0, 0, 0, 0, 0]^T$:
- Base frame $\{0\}$: $\mathbf{p}_0 = [0, 0, 0]^T$, $\mathbf{z}_0 = [0, 0, 1]^T$ (vertical axis).
- End-effector position at home: $\mathbf{p}_e = [0.500, \; 0.000, \; -0.150]^T\,\text{m}$.

**Compute Column 1 ($\mathbf{J}_1$):**
1. **Lever arm vector:**
   $$\mathbf{r} = \mathbf{p}_e - \mathbf{p}_0 = \begin{bmatrix} 0.500 \\ 0.000 \\ -0.150 \end{bmatrix} - \begin{bmatrix} 0.000 \\ 0.000 \\ 0.000 \end{bmatrix} = \begin{bmatrix} 0.500 \\ 0.000 \\ -0.150 \end{bmatrix}\,\text{m}$$
2. **Linear velocity contribution $\mathbf{J}_{v, 1} = \mathbf{z}_0 \times \mathbf{r}$:**
   $$\mathbf{J}_{v, 1} = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} \times \begin{bmatrix} 0.500 \\ 0.000 \\ -0.150 \end{bmatrix} = \begin{bmatrix} (0)(-0.150) - (1)(0.0) \\ (1)(0.500) - (0)(-0.150) \\ (0)(0.0) - (0)(0.500) \end{bmatrix} = \begin{bmatrix} 0.000 \\ +0.500 \\ 0.000 \end{bmatrix}\,\text{m/rad}$$
3. **Angular velocity contribution $\mathbf{J}_{\omega, 1} = \mathbf{z}_0$:**
   $$\mathbf{J}_{\omega, 1} = \begin{bmatrix} 0.000 \\ 0.000 \\ 1.000 \end{bmatrix}\,\text{rad/rad}$$
4. **Resulting Column 1:**
   $$\mathbf{J}_1 = [0.0, \; +0.500, \; 0.0, \; 0.0, \; 0.0, \; 1.0]^T$$

**Interpretation:**
If Joint 1 (the waist) rotates at $\dot{\theta}_1 = 1.0\,\text{rad/s}$ while all other joints remain locked:
- The end-effector moves sideways along $+Y$ at $0.500\,\text{m/s}$ (radius of $0.5\,\text{m} \times 1\,\text{rad/s}$).
- The end-effector rotates about $+Z$ at $1.0\,\text{rad/s}$.

---

## 6. Python Implementation

Tested using `04-python-robotics/jacobian.py`:

```python
import numpy as np
from forward_kinematics_6dof import RobotArm6DOF
from jacobian import geometric_jacobian_6dof, numerical_jacobian_6dof

arm = RobotArm6DOF()
q = np.radians([20.0, 35.0, -25.0, 15.0, -30.0, 10.0])

J_geom = geometric_jacobian_6dof(arm, q)
J_num = numerical_jacobian_6dof(arm, q, delta_q=1e-6)

print("Geometric Jacobian Shape:", J_geom.shape)
print("Top 3x3 block (Linear velocity mapping):\n", np.round(J_geom[:3, :3], 4))

max_diff = np.max(np.abs(J_geom - J_num))
print(f"Max difference with numerical finite differences: {max_diff:.2e}")
print(f"Verified accurate? {max_diff < 1e-4}")
```

---

## 7. Common Mistakes & Debugging Tips
1. **Using the Wrong Frame Index for $\mathbf{z}$ and $\mathbf{p}$:**
   Column $i$ uses axis $\mathbf{z}_{i-1}$ and origin $\mathbf{p}_{i-1}$ from the PREVIOUS frame, because Joint $i$ rotates around the axis established at the end of Link $i-1$.
2. **Forgetting Cross Product Order:**
   It is $\mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1})$, NOT $(\mathbf{p}_e - \mathbf{p}_{i-1}) \times \mathbf{z}_{i-1}$. Reversing the order inverts the sign of all velocities!
3. **Assuming $\boldsymbol{\omega}$ Equals Euler Angle Rates:**
   $\boldsymbol{\omega}$ is the instantaneous physical angular velocity vector. It does not equal $[\dot{\text{roll}}, \dot{\text{pitch}}, \dot{\text{yaw}}]^T$ directly.

---

## 8. Exercises

### Beginner
1. What are the row and column dimensions of the Jacobian matrix for a 6-DoF robot arm?
2. If joint velocity $\dot{\theta}_1 = 2\,\text{rad/s}$ and column $\mathbf{J}_1 = [0, 0.4, 0, 0, 0, 1]^T$, what is the instantaneous end-effector velocity?

### Intermediate
3. For a 2-link planar arm with $l_1 = 1.0\,\text{m}, l_2 = 1.0\,\text{m}$ at $\theta_1 = 0, \theta_2 = 0$, compute the $2 \times 2$ Jacobian. What is its determinant?
4. A 6-DOF robot exerts a downward vertical force $F_z = -100\,\text{N}$ with zero torques. If the 3rd row of its Jacobian is $[0.1, 0.4, 0.2, 0, 0, 0]$, calculate the resulting motor torques for all joints.

### Advanced
5. Derive the column equation for a **prismatic joint**: Prove that for a joint that translates along $\mathbf{z}_{i-1}$ with speed $\dot{d}_i$, its Jacobian column is:
   $$\mathbf{J}_i = \begin{bmatrix} \mathbf{z}_{i-1} \\ \mathbf{0} \end{bmatrix}$$

---

## 9. Summary & Next Steps
- The Geometric Jacobian relates joint rates to task-space velocities and tool forces to motor torques.
- Central finite differences provide a bulletproof validation for analytical Jacobians.
- Next, explore what happens when the Jacobian loses rank in [**10: Singularities, Manipulability & Joint Limits**](10-singularities-and-joint-limits.md).
