# 06: Jacobians and Gradient Descent in Robotics

## 1. Learning Objectives
By completing this chapter, you will be able to:
- Construct the analytical and geometric Jacobian matrix $\mathbf{J}(\mathbf{q})$ for multi-link robotic arms.
- Map joint velocities $\mathbf{\dot{q}}$ to operational task space linear and angular velocities $\mathbf{\xi} = [\mathbf{v}, \boldsymbol{\omega}]^T$.
- Formulate inverse kinematics as an optimization problem solved via Gradient Descent.
- Explain the role of the Jacobian Transpose method ($\mathbf{\Delta q} = \alpha \mathbf{J}^T \mathbf{e}$) in virtual work and kinematics.
- Apply Damped Least Squares (Levenberg-Marquardt) to prevent numerical instability at kinematic singularities.

---

## 2. Intuitive Explanation
In robotics, the forward kinematics function $\mathbf{x} = \mathbf{f}(\mathbf{q})$ tells you *where the end-effector is* given joint angles $\mathbf{q}$.
The **Jacobian Matrix $\mathbf{J}(\mathbf{q})$** is the multidimensional derivative of this function:
1. **At the velocity level:** It tells you how fast the end-effector moves given joint speeds:
   $$\mathbf{v} = \mathbf{J}(\mathbf{q}) \mathbf{\dot{q}}$$
2. **At the static force level (Virtual Work):** It translates end-effector contact forces $\mathbf{F}$ into required joint motor torques $\boldsymbol{\tau}$:
   $$\boldsymbol{\tau} = \mathbf{J}^T(\mathbf{q}) \mathbf{F}$$
3. **In numerical optimization:** It provides the local slope needed to guide the robot's joints toward a desired target through **Gradient Descent**.

```mermaid
graph LR
    JointSpeeds["Joint Velocities dq/dt"] -->|Jacobian J(q)| ToolSpeeds["Tool Velocities v, omega"]
    ToolForce["Contact Force F"] -->|Jacobian Transpose J^T| MotorTorque["Motor Torque tau"]
    TargetError["Position Error e"] -->|Damped Inverse J_dls| JointUpdate["IK Step delta_q"]
```

---

## 3. Mathematical Formulation

For a robotic system with $m$ operational task coordinates (e.g. $m=6$ for $[x, y, z, \text{roll}, \text{pitch}, \text{yaw}]$) and $n$ joint variables $\mathbf{q} = [q_1, q_2, \dots, q_n]^T$, the **Jacobian Matrix** is an $m \times n$ matrix:

$$\mathbf{J}(\mathbf{q}) = \begin{bmatrix}
\frac{\partial f_1}{\partial q_1} & \frac{\partial f_1}{\partial q_2} & \dots & \frac{\partial f_1}{\partial q_n} \\
\frac{\partial f_2}{\partial q_1} & \frac{\partial f_2}{\partial q_2} & \dots & \frac{\partial f_2}{\partial q_n} \\
\vdots & \vdots & \ddots & \vdots \\
\frac{\partial f_m}{\partial q_1} & \frac{\partial f_m}{\partial q_2} & \dots & \frac{\partial f_m}{\partial q_n}
\end{bmatrix}$$

### Velocity Kinematics:
$$\mathbf{\xi} = \begin{bmatrix} \mathbf{v} \\ \boldsymbol{\omega} \end{bmatrix} = \mathbf{J}(\mathbf{q}) \mathbf{\dot{q}}$$

---

## 4. Solving Inverse Kinematics via Gradient Descent

Suppose a robot's current end-effector pose is $\mathbf{x} = \mathbf{f}(\mathbf{q})$, and we want to reach target pose $\mathbf{x}_d$.
We define the quadratic scalar error cost function:

$$E(\mathbf{q}) = \frac{1}{2} \|\mathbf{x}_d - \mathbf{f}(\mathbf{q})\|^2 = \frac{1}{2} \mathbf{e}^T \mathbf{e}$$
where $\mathbf{e} = \mathbf{x}_d - \mathbf{f}(\mathbf{q})$ is the Cartesian tracking error.

### Deriving the Gradient:
Using the multivariable chain rule:
$$\nabla_{\mathbf{q}} E = \frac{\partial E}{\partial \mathbf{q}} = \left(\frac{\partial \mathbf{e}}{\partial \mathbf{q}}\right)^T \mathbf{e}$$
Since $\mathbf{e} = \mathbf{x}_d - \mathbf{f}(\mathbf{q})$, its derivative with respect to $\mathbf{q}$ is:
$$\frac{\partial \mathbf{e}}{\partial \mathbf{q}} = -\frac{\partial \mathbf{f}}{\partial \mathbf{q}} = -\mathbf{J}(\mathbf{q})$$

Therefore:
$$\nabla_{\mathbf{q}} E = -\mathbf{J}^T(\mathbf{q}) \mathbf{e}$$

### Gradient Descent Update Rule:
To minimize the error, step in the direction opposite to the gradient (downhill):
$$\mathbf{q}_{k+1} = \mathbf{q}_k - \alpha \nabla_{\mathbf{q}} E = \mathbf{q}_k + \alpha \mathbf{J}^T(\mathbf{q}_k) \mathbf{e}_k$$
where $\alpha > 0$ is the learning rate step size.

> [!NOTE]
> This is the famous **Jacobian Transpose Method**. It requires no matrix inversion at all! While slower to converge than Newton-Raphson methods, it is numerically stable and never blows up near singularities.

---

## 5. Damped Least Squares (Levenberg-Marquardt)

When faster quadratic convergence is required, Newton-based inversion uses:
$$\Delta \mathbf{q} = \mathbf{J}^{-1} \mathbf{e}$$

However, if the robot arm is stretched straight or near a singular configuration, $\det(\mathbf{J}) \approx 0$ and $\mathbf{J}^{-1}$ explodes.
To prevent infinite joint velocities, we introduce the **Damped Least Squares (DLS)** pseudo-inverse:

$$\Delta \mathbf{q} = \mathbf{J}^T \left( \mathbf{J} \mathbf{J}^T + \lambda^2 \mathbf{I} \right)^{-1} \mathbf{e}$$

where:
- $\lambda > 0$ is the damping parameter.
- When far from singularities ($\sigma_{\min} \gg \lambda$), it behaves like standard pseudoinverse $\mathbf{J}^\dagger$.
- When near singularities ($\sigma_{\min} \to 0$), $\lambda^2 \mathbf{I}$ bounds the eigenvalues, gracefully sacrificing exact tracking accuracy to preserve mechanical safety and numerical stability.

---

## 6. Fully Worked Numerical Example: 2-Link Planar Jacobian

Consider a 2-link planar arm with lengths $l_1 = 1.0\,\text{m}, l_2 = 1.0\,\text{m}$.
The forward kinematics equations are:
$$x = l_1 \cos(q_1) + l_2 \cos(q_1 + q_2)$$
$$y = l_1 \sin(q_1) + l_2 \sin(q_1 + q_2)$$

### Step 1: Form the $2 \times 2$ Jacobian:
$$\mathbf{J}(q_1, q_2) = \begin{bmatrix}
\frac{\partial x}{\partial q_1} & \frac{\partial x}{\partial q_2} \\
\frac{\partial y}{\partial q_1} & \frac{\partial y}{\partial q_2}
\end{bmatrix} = \begin{bmatrix}
-l_1 s_1 - l_2 s_{12} & -l_2 s_{12} \\
l_1 c_1 + l_2 c_{12} & l_2 c_{12}
\end{bmatrix}$$
where $s_1 = \sin(q_1), s_{12} = \sin(q_1 + q_2), c_1 = \cos(q_1), c_{12} = \cos(q_1 + q_2)$.

### Step 2: Evaluate at $q_1 = 0^\circ, q_2 = 90^\circ$ ($\pi/2\,\text{rad}$)
- $s_1 = 0, c_1 = 1.0$
- $s_{12} = \sin(90^\circ) = 1.0, c_{12} = \cos(90^\circ) = 0$
$$\mathbf{J} = \begin{bmatrix}
-(1)(0) - (1)(1) & -(1)(1) \\
(1)(1) + (1)(0) & (1)(0)
\end{bmatrix} = \begin{bmatrix}
-1.0 & -1.0 \\
+1.0 & 0.0
\end{bmatrix}$$

### Step 3: Compute Joint Velocities for Cartesian Motion
Suppose we want the end-effector to move along $+Y$ at $v_y = 0.5\,\text{m/s}$ with zero $X$ velocity ($v_x = 0$):
$$\begin{bmatrix} 0.0 \\ 0.5 \end{bmatrix} = \begin{bmatrix} -1.0 & -1.0 \\ 1.0 & 0.0 \end{bmatrix} \begin{bmatrix} \dot{q}_1 \\ \dot{q}_2 \end{bmatrix}$$

Inverting $\mathbf{J}$:
$$\det(\mathbf{J}) = (-1)(0) - (-1)(1) = 1.0$$
$$\mathbf{J}^{-1} = \frac{1}{1.0} \begin{bmatrix} 0.0 & 1.0 \\ -1.0 & -1.0 \end{bmatrix}$$
Multiply:
$$\begin{bmatrix} \dot{q}_1 \\ \dot{q}_2 \end{bmatrix} = \begin{bmatrix} 0.0 & 1.0 \\ -1.0 & -1.0 \end{bmatrix} \begin{bmatrix} 0.0 \\ 0.5 \end{bmatrix} = \begin{bmatrix} +0.50 \\ -0.50 \end{bmatrix}\,\text{rad/s}$$
**Result:** Joint 1 must rotate at $+0.50\,\text{rad/s}$ while Joint 2 rotates at $-0.50\,\text{rad/s}$.

---

## 7. Python Demonstration

Tested using `04-python-robotics/jacobian.py`:

```python
import numpy as np

def planar_jacobian(q1, q2, l1=1.0, l2=1.0):
    s1 = np.sin(q1); c1 = np.cos(q1)
    s12 = np.sin(q1 + q2); c12 = np.cos(q1 + q2)
    return np.array([
        [-l1*s1 - l2*s12, -l2*s12],
        [ l1*c1 + l2*c12,  l2*c12]
    ])

q = np.array([0.0, np.pi / 2])
v_desired = np.array([0.0, 0.5])

J = planar_jacobian(q[0], q[1])
q_dot = np.linalg.inv(J) @ v_desired

print("Jacobian Matrix J:\n", J)
print(f"Commanded Joint Velocities: q1_dot={q_dot[0]:.2f} rad/s, q2_dot={q_dot[1]:.2f} rad/s")
print(f"Verified Forward Velocity J @ q_dot: {J @ q_dot}")
```

---

## 8. Common Mistakes & Debugging Tips
1. **Inverting $\mathbf{J}$ Directly Near Singularities:**
   Never use `np.linalg.inv(J)` without protection. If the arm extends straight ($q_2 = 0$), $\det(\mathbf{J}) = l_1 l_2 \sin(q_2) = 0$, triggering a crash. Always use `damped_least_squares_inverse`.
2. **Confusing Geometric and Analytical Jacobians:**
   The Geometric Jacobian maps joint velocities directly to physical angular velocity $\boldsymbol{\omega} \in \mathbb{R}^3$. The Analytical Jacobian maps to Euler angle rates $\mathbf{\dot{\phi}} = [\dot{\text{roll}}, \dot{\text{pitch}}, \dot{\text{yaw}}]^T$. These are related by a transformation matrix $\mathbf{T}_{\text{rpy}}$: $\boldsymbol{\omega} = \mathbf{T}_{\text{rpy}} \mathbf{\dot{\phi}}$.
3. **Step Size Too Large in Gradient Descent:**
   If learning rate $\alpha$ is too large, joint angles oscillate wildly and diverge.

---

## 9. Exercises

### Beginner
1. What are the dimensions of the Jacobian matrix for a 3-DoF planar robot with a 2D end-effector position $(x, y)$?
2. If $\mathbf{J} = \begin{bmatrix} 2 & 1 \\ 0 & 3 \end{bmatrix}$ and $\mathbf{\dot{q}} = [1, 2]^T$, calculate the task velocity $\mathbf{v}$.

### Intermediate
3. For the 2-link planar arm with $l_1 = l_2 = 1.0\,\text{m}$, find all joint configurations $(q_1, q_2)$ where the robot is in a kinematic singularity ($\det(\mathbf{J}) = 0$). Explain the physical geometry of these states.
4. Calculate the Damped Least Squares update for a singular Jacobian $\mathbf{J} = \begin{bmatrix} 1 & 0 \\ 0 & 0 \end{bmatrix}$ with error vector $\mathbf{e} = [0.1, 0.2]^T$ and damping factor $\lambda = 0.1$.

### Advanced
5. Prove that the Yoshikawa Manipulability Measure $w = \sqrt{\det(\mathbf{J}\mathbf{J}^T)}$ is equal to the product of the singular values $\sigma_1 \sigma_2 \dots \sigma_m$ of the Jacobian matrix.

---

## 10. Summary & Next Steps
- The Jacobian maps joint velocities to task-space velocities and tool forces to joint torques.
- Gradient descent and damped least squares enable robust numerical inverse kinematics.
- Next, turn to probability and state estimation in [**07: Probability & Bayes' Theorem**](07-probability.md).
