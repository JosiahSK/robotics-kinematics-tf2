# 04: Homogeneous Transformations in Robotics & Manipulators

## 1. Learning Objectives
By completing this chapter, you will be able to:
- Construct $4 \times 4$ homogeneous transformation matrices $\mathbf{A}_i$ for serial links.
- Model multi-link chains through successive matrix multiplications ${}^0 \mathbf{T}_n = \mathbf{A}_1 \mathbf{A}_2 \dots \mathbf{A}_n$.
- Extract 3D position vectors $\mathbf{p}$ and $3 \times 3$ rotation matrices $\mathbf{R}$ from composite transformation matrices.
- Apply homogeneous transformation matrices to transform end-effector targets from world space into base space.

---

## 2. Intuitive Explanation
In a multi-link serial robot, each motor moves the next link.
- Joint 1 rotates Link 1 relative to the Base (Frame $\{0\} \to \{1\}$).
- Joint 2 rotates Link 2 relative to Link 1 (Frame $\{1\} \to \{2\}$).
- Joint 3 rotates Link 3 relative to Link 2 (Frame $\{2\} \to \{3\}$).

To find where Link 3 is relative to the Base, we do not need to do complex spatial trigonometry by hand.
We simply express each link transition as a $4 \times 4$ matrix $\mathbf{A}_i$, and multiply them:
$${}^0 \mathbf{T}_3 = \mathbf{A}_1 \; \mathbf{A}_2 \; \mathbf{A}_3$$

```mermaid
graph LR
    F0["Frame {0} (Base)"] -->|A1| F1["Frame {1}"]
    F1 -->|A2| F2["Frame {2}"]
    F2 -->|A3| F3["Frame {3}"]
    F3 -->|A4| F4["Frame {4}"]
    F4 -->|A5| F5["Frame {5}"]
    F5 -->|A6| F6["Frame {6} (Flange)"]
```

---

## 3. Mathematical Structure

A general homogeneous transformation matrix $\mathbf{T} \in SE(3)$ has the partitioned structure:

$$\mathbf{T} = \begin{bmatrix}
r_{11} & r_{12} & r_{13} & p_x \\
r_{21} & r_{22} & r_{23} & p_y \\
r_{31} & r_{32} & r_{33} & p_z \\
0 & 0 & 0 & 1
\end{bmatrix} = \begin{bmatrix}
\mathbf{n} & \mathbf{s} & \mathbf{a} & \mathbf{p} \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### Physical Meaning of the Columns:
- $\mathbf{n} = [r_{11}, r_{21}, r_{31}]^T$: The **Normal Vector** (unit direction of the child $X$-axis in parent coordinates).
- $\mathbf{s} = [r_{12}, r_{22}, r_{32}]^T$: The **Sliding Vector** (unit direction of the child $Y$-axis in parent coordinates).
- $\mathbf{a} = [r_{13}, r_{23}, r_{33}]^T$: The **Approach Vector** (unit direction of the child $Z$-axis, pointing along the tool approach direction).
- $\mathbf{p} = [p_x, p_y, p_z]^T$: The **Position Vector** (origin of child frame measured from parent origin).

---

## 4. Fully Worked Numerical Example: 2-Link Spatial Arm

Consider a 2-link spatial robot arm:
- **Link 1:** Base waist rotation by $\theta_1 = 90^\circ$ about $Z_0$, with a vertical height offset $d_1 = 0.50\,\text{m}$ along $Z_0$.
  $$\mathbf{A}_1 = \begin{bmatrix} \cos(90^\circ) & -\sin(90^\circ) & 0 & 0 \\ \sin(90^\circ) & \cos(90^\circ) & 0 & 0 \\ 0 & 0 & 1 & 0.50 \\ 0 & 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} 0 & -1 & 0 & 0 \\ 1 & 0 & 0 & 0 \\ 0 & 0 & 1 & 0.50 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$
- **Link 2:** Shoulder pitch rotation by $\theta_2 = 0^\circ$ with link length $a_2 = 0.80\,\text{m}$ along $X_1$:
  $$\mathbf{A}_2 = \begin{bmatrix} 1 & 0 & 0 & 0.80 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

**Compute composite transform $T_0^2 = \mathbf{A}_1 \mathbf{A}_2$:**
$$T_0^2 = \begin{bmatrix} 0 & -1 & 0 & 0 \\ 1 & 0 & 0 & 0 \\ 0 & 0 & 1 & 0.50 \\ 0 & 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} 1 & 0 & 0 & 0.80 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} 0 & -1 & 0 & 0(0.80) - 1(0) + 0(0) + 0 \\ 1 & 0 & 0 & 1(0.80) + 0(0) + 0(0) + 0 \\ 0 & 0 & 1 & 0(0.80) + 0(0) + 1(0) + 0.50 \\ 0 & 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} 0 & -1 & 0 & 0.00 \\ 1 & 0 & 0 & 0.80 \\ 0 & 0 & 1 & 0.50 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

**Interpretation:**
- The end of Link 2 is located at $x = 0.00\,\text{m}, y = 0.80\,\text{m}, z = 0.50\,\text{m}$ in the base frame.
- Its local $X$-axis points along the base $+Y$-axis ($\mathbf{n} = [0, 1, 0]^T$).
- Its local $Z$-axis points along the base $+Z$-axis ($\mathbf{a} = [0, 0, 1]^T$).

---

## 5. Python Demonstration

```python
import numpy as np
from dh_transform import create_homogeneous_matrix, extract_pose
from rotation_matrices import rot_z

# Link 1: 90 deg z-rotation, 0.5m z-height
A1 = create_homogeneous_matrix(rot_z(np.pi / 2), np.array([0.0, 0.0, 0.5]))

# Link 2: 0 deg rotation, 0.8m x-length
A2 = create_homogeneous_matrix(np.eye(3), np.array([0.8, 0.0, 0.0]))

T_0_2 = A1 @ A2
pos, rot = extract_pose(T_0_2)

print("T_0_2 Matrix:\n", np.round(T_0_2, 4))
print(f"Extracted Position: {pos}")
print(f"Is rotation valid? {np.allclose(rot @ rot.T, np.eye(3))}")
```

---

## 6. Common Mistakes & Debugging Tips
1. **Wrong Multiplication Direction:**
   Remember: $T_0^n = A_1 A_2 \dots A_n$. Pre-multiplying (e.g. $A_n \dots A_1$) rotates about the base frame instead of the local moving joint frame, completely invalidating the kinematic chain.
2. **Forgetting to Normalize Output Columns:**
   After chaining 6 or 7 matrices in floating-point math, tiny rounding errors accumulate. Always check $\det(\mathbf{R}) \approx 1.0$ and normalize columns if necessary.

---

## 7. Exercises

### Beginner
1. What physical quantity is stored in the 4th column of an $SE(3)$ homogeneous matrix?
2. Write the $4 \times 4$ transformation matrix representing a pure $90^\circ$ rotation about the $X$-axis with zero translation.

### Intermediate
3. A robot base is mounted at $(1.0, 2.0, 0.0)$ on a factory floor. The end-effector is at $(0.5, 0.0, 0.8)$ relative to the robot base. What are the coordinates of the end-effector relative to the factory floor origin?
4. Prove that the product of two upper-triangular homogeneous matrices of the form $\begin{bmatrix} \mathbf{R} & \mathbf{p} \\ \mathbf{0} & 1 \end{bmatrix}$ always has $[0, 0, 0, 1]$ as its bottom row.

### Advanced
5. Given a serial robot with $N$ links, write the mathematical formula for the position of joint origin $i$ in terms of the individual link transformation matrices $\mathbf{A}_1, \dots, \mathbf{A}_i$.

---

## 8. Summary & Next Steps
- Homogeneous transformation matrices represent spatial links in serial chains.
- Next, learn the standard convention for assigning frames systematically in [**05: Denavit–Hartenberg (DH) Convention**](05-denavit-hartenberg.md).
