# 04: Homogeneous Transformations & $SE(3)$

## 1. Learning Objectives
By completing this chapter, you will be able to:
- Explain why 3D translations cannot be represented by $3 \times 3$ matrix multiplication alone.
- Construct 4D homogeneous coordinates for points ($w=1$) and direction vectors ($w=0$).
- Assemble $4 \times 4$ homogeneous transformation matrices belonging to the Special Euclidean Group $SE(3)$.
- Chain sequential transformations via matrix multiplication.
- Apply the analytical inversion formula $\mathbf{T}^{-1}$ without performing expensive Gaussian elimination.

---

## 2. Intuitive Explanation
A rigid body moving through space experiences both **rotation** and **translation**:
$$\mathbf{p}' = \mathbf{R} \mathbf{p} + \mathbf{t}$$

This is an **affine transformation**, not a linear transformation. Why does that matter?
Because you cannot chain multiple steps using simple matrix multiplication!
If you transform from frame $\{C\}$ to $\{B\}$, and then $\{B\}$ to $\{A\}$:
$${}^A \mathbf{p} = {}^A \mathbf{R}_B ({}^B \mathbf{R}_C \; {}^C \mathbf{p} + {}^B \mathbf{t}_C) + {}^A \mathbf{t}_B = ({}^A \mathbf{R}_B \; {}^B \mathbf{R}_C) \; {}^C \mathbf{p} + ({}^A \mathbf{R}_B \; {}^B \mathbf{t}_C + {}^A \mathbf{t}_B)$$

For a 6-joint robot arm, chaining 6 separate additions and multiplications becomes a nested mathematical nightmare.

**The Solution:** Homogeneous coordinates! By adding a dummy 4th coordinate ($w=1$), translation becomes a standard matrix multiplication. All rotations, translations, and frame chains collapse into a clean, uniform matrix product:
$${}^0 \mathbf{T}_6 = \mathbf{A}_1 \mathbf{A}_2 \mathbf{A}_3 \mathbf{A}_4 \mathbf{A}_5 \mathbf{A}_6$$

```mermaid
graph LR
    Affine["Affine Math: p' = R*p + t (Messy chaining)"] -->|Add 4th Projective Coordinate w=1| Homogeneous["SE(3) Matrix: P' = T * P (Clean matrix chain!)"]
```

---

## 3. Mathematical Formulation

### 3.1 Homogeneous Coordinates
- A **3D physical point** $\mathbf{p} = [x, y, z]^T \in \mathbb{R}^3$ is represented by the 4D column vector:
  $$\mathbf{\tilde{p}} = \begin{bmatrix} x \\ y \\ z \\ 1 \end{bmatrix}$$
- A **3D direction / velocity vector** $\mathbf{v} = [v_x, v_y, v_z]^T$ is represented by:
  $$\mathbf{\tilde{v}} = \begin{bmatrix} v_x \\ v_y \\ v_z \\ 0 \end{bmatrix}$$
  *(Notice that setting $w=0$ makes vectors invariant to translation: translating an origin does not alter the direction of a velocity or force!)*

### 3.2 The $SE(3)$ Homogeneous Matrix
A rigid body transformation matrix $\mathbf{T} \in SE(3)$ has shape $4 \times 4$:

$$\mathbf{T} = \begin{bmatrix} \mathbf{R} & \mathbf{p} \\ \mathbf{0}_{1 \times 3} & 1 \end{bmatrix} = \begin{bmatrix} r_{11} & r_{12} & r_{13} & p_x \\ r_{21} & r_{22} & r_{23} & p_y \\ r_{31} & r_{32} & r_{33} & p_z \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

where:
- $\mathbf{R} \in SO(3)$ is a $3 \times 3$ proper rotation matrix ($\mathbf{R}^T \mathbf{R} = \mathbf{I}$, $\det(\mathbf{R}) = +1$).
- $\mathbf{p} \in \mathbb{R}^3$ is the $3 \times 1$ translation displacement vector.
- $\mathbf{0}_{1 \times 3} = [0, 0, 0]$ is the perspective row (zero for affine rigid transforms).
- $1$ is the homogeneous scale factor.

---

## 4. Analytical Inversion of $SE(3)$ Matrices

General $4 \times 4$ matrix inversion requires Gaussian elimination or LU decomposition ($O(n^3) \approx 64$ floating-point operations) and suffers from numerical drift.

Because the rotation block $\mathbf{R}$ is orthonormal ($\mathbf{R}^{-1} = \mathbf{R}^T$), we can derive the **exact analytical inverse** directly:

### Derivation:
Let $\mathbf{T} = \begin{bmatrix} \mathbf{R} & \mathbf{p} \\ \mathbf{0} & 1 \end{bmatrix}$. We seek $\mathbf{T}^{-1} = \begin{bmatrix} \mathbf{X} & \mathbf{y} \\ \mathbf{0} & 1 \end{bmatrix}$ such that $\mathbf{T} \mathbf{T}^{-1} = \mathbf{I}_4$:

$$\begin{bmatrix} \mathbf{R} & \mathbf{p} \\ \mathbf{0} & 1 \end{bmatrix} \begin{bmatrix} \mathbf{X} & \mathbf{y} \\ \mathbf{0} & 1 \end{bmatrix} = \begin{bmatrix} \mathbf{RX} & \mathbf{Ry} + \mathbf{p} \\ \mathbf{0} & 1 \end{bmatrix} = \begin{bmatrix} \mathbf{I}_3 & \mathbf{0} \\ \mathbf{0} & 1 \end{bmatrix}$$

From row 1:
1. $\mathbf{RX} = \mathbf{I}_3 \implies \mathbf{X} = \mathbf{R}^{-1} = \mathbf{R}^T$
2. $\mathbf{Ry} + \mathbf{p} = \mathbf{0} \implies \mathbf{Ry} = -\mathbf{p} \implies \mathbf{y} = -\mathbf{R}^T \mathbf{p}$

Thus:
$$\mathbf{T}^{-1} = \begin{bmatrix} \mathbf{R}^T & -\mathbf{R}^T \mathbf{p} \\ \mathbf{0}_{1 \times 3} & 1 \end{bmatrix}$$

> [!TIP]
> This analytical inversion requires only 1 transpose, 1 matrix-vector product, and 1 negation. It runs in $O(1)$ time and guarantees zero determinant degradation.

---

## 5. Fully Worked Numerical Example

Consider a sensor frame $\{B\}$ mounted on robot $\{A\}$:
- Rotated by $+90^\circ$ around $Z$ ($\mathbf{R}_z(90^\circ)$).
- Translated by $\mathbf{p} = [0.5, 0.2, 0.0]^T\,\text{m}$.

1. **Construct ${}^A \mathbf{T}_B$:**
   $${}^A \mathbf{T}_B = \begin{bmatrix} 0 & -1 & 0 & 0.5 \\ 1 & 0 & 0 & 0.2 \\ 0 & 0 & 1 & 0.0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$
2. **Compute the analytical inverse ${}^B \mathbf{T}_A = ({}^A \mathbf{T}_B)^{-1}$:**
   - $\mathbf{R}^T = \begin{bmatrix} 0 & 1 & 0 \\ -1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix}$
   - $-\mathbf{R}^T \mathbf{p} = -\begin{bmatrix} 0 & 1 & 0 \\ -1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} 0.5 \\ 0.2 \\ 0.0 \end{bmatrix} = -\begin{bmatrix} 0.2 \\ -0.5 \\ 0.0 \end{bmatrix} = \begin{bmatrix} -0.2 \\ 0.5 \\ 0.0 \end{bmatrix}$
   - Assemble matrix:
     $${}^B \mathbf{T}_A = \begin{bmatrix} 0 & 1 & 0 & -0.2 \\ -1 & 0 & 0 & 0.5 \\ 0 & 0 & 1 & 0.0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$
3. **Verify by matrix multiplication:**
   $${}^A \mathbf{T}_B \; {}^B \mathbf{T}_A = \begin{bmatrix} 0 & -1 & 0 & 0.5 \\ 1 & 0 & 0 & 0.2 \\ 0 & 0 & 1 & 0.0 \\ 0 & 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} 0 & 1 & 0 & -0.2 \\ -1 & 0 & 0 & 0.5 \\ 0 & 0 & 1 & 0.0 \\ 0 & 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} = \mathbf{I}_4$$

---

## 6. Python Demonstration

Tested using `04-python-robotics/dh_transform.py`:

```python
import numpy as np
from dh_transform import create_homogeneous_matrix, invert_homogeneous_matrix
from rotation_matrices import rot_z

R = rot_z(np.pi / 2) # 90 deg yaw
p = np.array([0.5, 0.2, 0.0])

T = create_homogeneous_matrix(R, p)
T_inv = invert_homogeneous_matrix(T)

print("Forward Transform T:\n", np.round(T, 4))
print("Analytical Inverse T^-1:\n", np.round(T_inv, 4))
print("T @ T^-1 is Identity?", np.allclose(T @ T_inv, np.eye(4)))
```

---

## 7. Common Mistakes & Debugging Tips
1. **Setting $w=1$ for Velocity Vectors:**
   If you transform a velocity vector $\mathbf{v}$ with $w=1$, the matrix will add translation $\mathbf{p}$, which corrupts the velocity. Always set $w=0$ when transforming pure vectors (velocities, accelerations, forces).
2. **Inverting with `np.linalg.inv` in High-Rate Control Loops:**
   Using general numerical inversion instead of analytical $\mathbf{T}^{-1} = [\mathbf{R}^T, -\mathbf{R}^T \mathbf{p}]$ wastes CPU and introduces numerical float errors into $\mathbf{R}$.
3. **Chaining Transforms in the Wrong Order:**
   Remember: ${}^A \mathbf{T}_C = {}^A \mathbf{T}_B \; {}^B \mathbf{T}_C$. The adjacent indices must match (Parent $\to$ Match $\to$ Child).

---

## 8. Exercises

### Beginner
1. Write the $4 \times 4$ homogeneous transformation matrix for a pure translation of $\mathbf{p} = [1, -2, 3]^T$ with no rotation.
2. What is the homogeneous representation of the 3D origin point $(0, 0, 0)$?

### Intermediate
3. Transform the point $\mathbf{p} = [0, 1, 0]^T$ using the matrix:
   $$\mathbf{T} = \begin{bmatrix} 0 & -1 & 0 & 3 \\ 1 & 0 & 0 & 2 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$
4. Calculate the analytical inverse of the transformation matrix given in Exercise 3.

### Advanced
5. Prove that the set of all $4 \times 4$ matrices of the form $\begin{bmatrix} \mathbf{R} & \mathbf{p} \\ \mathbf{0} & 1 \end{bmatrix}$ with $\mathbf{R} \in SO(3)$ forms a mathematical group (the Special Euclidean Group $SE(3)$) by demonstrating closure, associativity, identity, and inverse.

---

## 9. Summary & Next Steps
- Homogeneous coordinates unify rotation and translation into single $4 \times 4$ matrix operations.
- The analytical inverse $\mathbf{T}^{-1} = [\mathbf{R}^T, -\mathbf{R}^T \mathbf{p}]$ is fast and numerically exact.
- Next, master the calculus tools needed for velocities and kinematics in [**05: Derivatives & Calculus**](05-derivatives-and-calculus.md).
