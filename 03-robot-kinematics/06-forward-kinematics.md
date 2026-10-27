# 06: Forward Kinematics for Serial Manipulators

## 1. Learning Objectives
By completing this chapter, you will be able to:
- Compute forward kinematics for planar and spatial manipulators using chained DH transformation matrices.
- Compare direct trigonometric derivations with systematic DH matrix chaining.
- Extract the end-effector Cartesian position $\mathbf{p}$ and orientation matrix $\mathbf{R}$ from $T_0^n$.
- Verify rotation matrix orthogonality ($\mathbf{R}^T \mathbf{R} = \mathbf{I}$) and proper chirality ($\det(\mathbf{R}) = +1$).
- Program automated forward kinematics in Python with explicit angle unit conversions.

---

## 2. Intuitive Explanation
Forward kinematics answers the question:
*"If my motors are at angles $[\theta_1, \theta_2, \dots, \theta_n]$, where in 3D space is my tool tip, and which way is it pointing?"*

Because every link is rigid and every joint has a known angle, this is a **deterministic geometric problem**. There is always exactly **one unique answer** for any set of joint angles.

```mermaid
graph LR
    Joints["Joint Values q = [q1, q2, ..., qn]^T"] --> DH1["A1(q1)"]
    DH1 --> DH2["A2(q2)"]
    DH2 --> DHN["... An(qn)"]
    DHN --> Chained["T_0^n = A1 * A2 * ... * An"]
    Chained --> Pose["End-Effector Pose: Position p, Rotation R"]
```

---

## 3. Mathematical Formulation

Given an $n$-DoF serial manipulator with link transformation matrices $\mathbf{A}_1(q_1), \dots, \mathbf{A}_n(q_n)$:

$$T_0^n(\mathbf{q}) = \mathbf{A}_1(q_1) \mathbf{A}_2(q_2) \dots \mathbf{A}_n(q_n) = \begin{bmatrix}
\mathbf{R}_0^n(\mathbf{q}) & \mathbf{p}_0^n(\mathbf{q}) \\
\mathbf{0}_{1 \times 3} & 1
\end{bmatrix}$$

### 3.1 Position Extraction
The 3D Cartesian position vector of the end-effector in the base frame is:
$$\mathbf{p}_0^n = \begin{bmatrix} T_{14} \\ T_{24} \\ T_{34} \end{bmatrix}$$

### 3.2 Orientation Extraction
The $3 \times 3$ rotation matrix of the end-effector relative to the base is:
$$\mathbf{R}_0^n = \begin{bmatrix}
T_{11} & T_{12} & T_{13} \\
T_{21} & T_{22} & T_{23} \\
T_{31} & T_{32} & T_{33}
\end{bmatrix}$$

### 3.3 Verification Metrics
A valid forward kinematics calculation must satisfy:
1. **Orthogonality:** $\|(\mathbf{R}_0^n)^T \mathbf{R}_0^n - \mathbf{I}_3\| < 10^{-6}$
2. **Proper Rotation:** $|\det(\mathbf{R}_0^n) - 1.0| < 10^{-6}$

---

## 4. Planar 2R Arm: Trigonometry vs Standard DH

Consider a 2-link planar revolute arm ($2R$) with link lengths $l_1$ and $l_2$ moving in the $X$-$Y$ plane.

### 4.1 Direct Trigonometric Derivation:
$$\mathbf{p}_1 = \begin{bmatrix} l_1 \cos(\theta_1) \\ l_1 \sin(\theta_1) \end{bmatrix}$$
$$\mathbf{p}_2 = \mathbf{p}_1 + \begin{bmatrix} l_2 \cos(\theta_1 + \theta_2) \\ l_2 \sin(\theta_1 + \theta_2) \end{bmatrix} = \begin{bmatrix} l_1 \cos(\theta_1) + l_2 \cos(\theta_1 + \theta_2) \\ l_1 \sin(\theta_1) + l_2 \sin(\theta_1 + \theta_2) \end{bmatrix}$$
End-effector orientation (total yaw angle):
$$\phi = \theta_1 + \theta_2$$

### 4.2 Standard DH Table:
| Link $i$ | $\theta_i$ | $d_i$ | $a_i$ | $\alpha_i$ |
| :---: | :---: | :---: | :---: | :---: |
| 1 | $\theta_1$ | 0 | $l_1$ | 0 |
| 2 | $\theta_2$ | 0 | $l_2$ | 0 |

Individual DH matrices:
$$\mathbf{A}_1 = \begin{bmatrix} \cos\theta_1 & -\sin\theta_1 & 0 & l_1 \cos\theta_1 \\ \sin\theta_1 & \cos\theta_1 & 0 & l_1 \sin\theta_1 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}, \quad \mathbf{A}_2 = \begin{bmatrix} \cos\theta_2 & -\sin\theta_2 & 0 & l_2 \cos\theta_2 \\ \sin\theta_2 & \cos\theta_2 & 0 & l_2 \sin\theta_2 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

Multiplying $\mathbf{A}_1 \mathbf{A}_2$:
$$T_0^2 = \begin{bmatrix}
\cos(\theta_1 + \theta_2) & -\sin(\theta_1 + \theta_2) & 0 & l_1 \cos\theta_1 + l_2 \cos(\theta_1 + \theta_2) \\
\sin(\theta_1 + \theta_2) & \cos(\theta_1 + \theta_2) & 0 & l_1 \sin\theta_1 + l_2 \sin(\theta_1 + \theta_2) \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}$$

The DH matrix chaining produces the **exact same equations** as manual trigonometry, demonstrating that DH is simply an automated, generalizable formulation of spatial geometry.

---

## 5. Fully Worked Numerical Example

Consider the 2R planar arm with $l_1 = 1.0\,\text{m}, l_2 = 0.5\,\text{m}$ at joint angles:
- $\theta_1 = 45^\circ$ ($\pi/4\,\text{rad}$)
- $\theta_2 = 45^\circ$ ($\pi/4\,\text{rad}$)

1. **Calculate trigonometric components:**
   - $\cos(45^\circ) = \sin(45^\circ) = \frac{\sqrt{2}}{2} \approx 0.7071$
   - $\theta_1 + \theta_2 = 90^\circ \implies \cos(90^\circ) = 0, \sin(90^\circ) = 1.0$
2. **Calculate End-Effector Position:**
   $$x = 1.0 \cos(45^\circ) + 0.5 \cos(90^\circ) = 0.7071 + 0.5(0) = 0.7071\,\text{m}$$
   $$y = 1.0 \sin(45^\circ) + 0.5 \sin(90^\circ) = 0.7071 + 0.5(1.0) = 1.2071\,\text{m}$$
   $$z = 0.00\,\text{m}$$
3. **Calculate Tool Orientation:**
   $$\phi = 45^\circ + 45^\circ = 90^\circ$$
   The tool points along $+Y$.

---

## 6. Python Implementation

Tested using `04-python-robotics/forward_kinematics_2d.py`:

```python
import numpy as np
from forward_kinematics_2d import fk_2link_planar, fk_2link_dh

l1, l2 = 1.0, 0.5
th1, th2 = np.deg2rad(45.0), np.deg2rad(45.0)

# Trigonometric FK
ee_pose, joints = fk_2link_planar(l1, l2, th1, th2)
print(f"Trig FK Position: ({ee_pose[0]:.4f}, {ee_pose[1]:.4f}) m, Orientation: {np.rad2deg(ee_pose[2]):.1f} deg")

# DH Matrix FK
T_dh = fk_2link_dh(l1, l2, th1, th2)
print("DH End-Effector Position:", T_dh[:2, 3])
print("Do Trig and DH match?", np.allclose(ee_pose[:2], T_dh[:2, 3]))
```

---

## 7. Common Mistakes & Debugging Tips
1. **Mixing Radians and Degrees:**
   Python's `math.sin` and `np.cos` strictly expect **radians**. Passing degrees ($45$) instead of radians ($0.7854$) produces completely erroneous results.
2. **Intermediate Joint Frame Coordinates for 3D Plotting:**
   To draw the robot arm's links, save the intermediate matrices $T_0^1, T_0^2, \dots, T_0^n$. Their translation columns $T_0^i[:3, 3]$ give the 3D vertices of every joint elbow!

---

## 8. Exercises

### Beginner
1. For a 2-link planar arm with $l_1 = 1.0\,\text{m}, l_2 = 1.0\,\text{m}$, calculate the end-effector $(x, y)$ position when $\theta_1 = 0^\circ, \theta_2 = 0^\circ$ (fully extended).
2. What is the tool orientation $\phi$ when $\theta_1 = 30^\circ, \theta_2 = -30^\circ$?

### Intermediate
3. A 3-link planar arm ($3R$) has link lengths $l_1 = 1.0\,\text{m}, l_2 = 0.8\,\text{m}, l_3 = 0.5\,\text{m}$. Write the forward kinematics equations for $x, y,$ and $\phi$.
4. Verify whether the rotation matrix $\mathbf{R} = \begin{bmatrix} 0.866 & -0.5 & 0 \\ 0.5 & 0.866 & 0 \\ 0 & 0 & 1 \end{bmatrix}$ satisfies the two orthogonality conditions.

### Advanced
5. Derive the symbolic forward kinematics transformation matrix $T_0^3$ for a SCARA robot ($R-R-P-R$ kinematic architecture) using standard DH matrices.

---

## 9. Summary & Next Steps
- Forward kinematics computes tool pose deterministically from joint angles.
- Matrix chaining systematically replicates spatial trigonometry for arbitrary multi-link robots.
- Next, tackle the reverse challenge in [**07: Inverse Kinematics Fundamentals**](07-inverse-kinematics.md).
