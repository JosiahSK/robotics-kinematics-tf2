# 01: Vectors for Robotics & AI

## 1. Learning Objectives
By completing this chapter, you will be able to:
- Define vectors mathematically as elements of $\mathbb{R}^n$ and physically as directed displacements, velocities, and forces.
- Represent vectors in standard column-vector form.
- Compute the Euclidean $L_2$ norm and normalize vectors into unit direction vectors.
- Understand vector addition, scalar multiplication, and linear combinations.
- Apply basis vectors to decompose robot state spaces.

---

## 2. Intuitive Explanation
In robotics, numbers rarely exist in isolation:
- Saying a robot is at "$5$" is meaningless. Is it $5\,\text{m}$ north? $5\,\text{m}$ east? $5\,\text{m}$ up?
- Stating a motor exerts a force of "$10\,\text{N}$" is incomplete unless you know *which direction* it is pushing.

A **vector** packages both **magnitude** (how much) and **direction** (which way) into a single mathematical entity. In a 3D workspace, a position vector points from the chosen coordinate origin to a target location.

```mermaid
graph LR
    Origin["Origin (0, 0, 0)"] -->|"Vector p = [x, y, z]^T"| Target["Robot End-Effector / Obstacle"]
```

---

## 3. Mathematical Formulation

A vector $\mathbf{v}$ in $n$-dimensional real space $\mathbb{R}^n$ is written as a $n \times 1$ column matrix:

$$\mathbf{v} = \begin{bmatrix} v_1 \\ v_2 \\ \vdots \\ v_n \end{bmatrix} \in \mathbb{R}^n$$

In 3D robotics space ($n=3$):
$$\mathbf{p} = \begin{bmatrix} x \\ y \\ z \end{bmatrix} = x \mathbf{\hat{i}} + y \mathbf{\hat{j}} + z \mathbf{\hat{k}}$$
where $\mathbf{\hat{i}} = [1, 0, 0]^T$, $\mathbf{\hat{j}} = [0, 1, 0]^T$, and $\mathbf{\hat{k}} = [0, 0, 1]^T$ are the standard orthonormal basis vectors aligned with the $X$, $Y$, and $Z$ coordinate axes.

### 4. Vector Operations

### 4.1 Vector Addition (Triangle / Parallelogram Rule)
Given two vectors $\mathbf{u}, \mathbf{v} \in \mathbb{R}^n$:
$$\mathbf{u} + \mathbf{v} = \begin{bmatrix} u_1 + v_1 \\ u_2 + v_2 \\ \vdots \\ u_n + v_n \end{bmatrix}$$
*Robotics meaning:* If you travel displacement $\mathbf{u}$ and then displacement $\mathbf{v}$, your total displacement from the start is $\mathbf{u} + \mathbf{v}$.

### 4.2 Scalar Multiplication
For any scalar $c \in \mathbb{R}$:
$$c \mathbf{v} = \begin{bmatrix} c v_1 \\ c v_2 \\ \vdots \\ c v_n \end{bmatrix}$$
*Robotics meaning:* Scaling motor thrust or scaling velocity by time $\Delta t$ ($\Delta \mathbf{p} = \mathbf{v} \Delta t$).

### 4.3 Euclidean Norm ($L_2$ Magnitude)
The length or magnitude of vector $\mathbf{v}$ is denoted $\|\mathbf{v}\|_2$ (or simply $\|\mathbf{v}\|$):
$$\|\mathbf{v}\| = \sqrt{v_1^2 + v_2^2 + \dots + v_n^2} = \sqrt{\mathbf{v}^T \mathbf{v}}$$

### 4.4 Unit Vector (Normalisation)
A unit vector $\mathbf{\hat{v}}$ has length $\|\mathbf{\hat{v}}\| = 1.0$ and points in the same direction as $\mathbf{v}$:
$$\mathbf{\hat{v}} = \frac{\mathbf{v}}{\|\mathbf{v}\|}, \quad \text{for } \|\mathbf{v}\| \ne 0$$

---

## 5. Fully Worked Numerical Example

Suppose a mobile delivery robot starts at waypoint $A$ with coordinates $\mathbf{p}_A = [1.0, 2.0, 0.0]^T\,\text{m}$ and must reach charging dock $B$ at $\mathbf{p}_B = [4.0, 6.0, 0.0]^T\,\text{m}$.

1. **Calculate the displacement vector $\mathbf{d}_{AB}$:**
   $$\mathbf{d}_{AB} = \mathbf{p}_B - \mathbf{p}_A = \begin{bmatrix} 4.0 - 1.0 \\ 6.0 - 2.0 \\ 0.0 - 0.0 \end{bmatrix} = \begin{bmatrix} 3.0 \\ 4.0 \\ 0.0 \end{bmatrix}\,\text{m}$$
2. **Calculate the straight-line distance (Euclidean norm):**
   $$\|\mathbf{d}_{AB}\| = \sqrt{3.0^2 + 4.0^2 + 0.0^2} = \sqrt{9.0 + 16.0 + 0} = \sqrt{25.0} = 5.0\,\text{m}$$
3. **Calculate the unit heading direction vector $\mathbf{\hat{u}}$:**
   $$\mathbf{\hat{u}} = \frac{\mathbf{d}_{AB}}{\|\mathbf{d}_{AB}\|} = \frac{1}{5.0} \begin{bmatrix} 3.0 \\ 4.0 \\ 0.0 \end{bmatrix} = \begin{bmatrix} 0.60 \\ 0.80 \\ 0.00 \end{bmatrix}$$
4. **Commanding Velocity:** If the robot travels at a desired linear speed of $v = 1.5\,\text{m/s}$, its velocity vector is:
   $$\mathbf{v} = v \mathbf{\hat{u}} = 1.5 \begin{bmatrix} 0.60 \\ 0.80 \\ 0.00 \end{bmatrix} = \begin{bmatrix} 0.90 \\ 1.20 \\ 0.00 \end{bmatrix}\,\text{m/s}$$

---

## 6. Python Implementation

Tested using `04-python-robotics/vectors_matrices.py`:

```python
import numpy as np

p_A = np.array([1.0, 2.0, 0.0])
p_B = np.array([4.0, 6.0, 0.0])

# Displacement
d_AB = p_B - p_A
distance = np.linalg.norm(d_AB)
unit_dir = d_AB / distance

print(f"Displacement Vector: {d_AB}")
print(f"Distance to Goal: {distance:.2f} m")
print(f"Unit Direction Vector: {unit_dir}")

# Set velocity
speed = 1.5 # m/s
velocity = speed * unit_dir
print(f"Commanded Velocity Vector: {velocity} m/s")
```

---

## 7. Common Mistakes & Debugging Tips
1. **Confusing Row Vectors and Column Vectors in NumPy:**
   In NumPy, standard 1D arrays `np.array([1, 2, 3])` have shape `(3,)` and behave smoothly with `@`. However, when doing explicit matrix math, a column vector has shape `(3, 1)` and a row vector has shape `(1, 3)`. Mixing them can trigger unexpected broadcasting errors.
2. **Dividing by Zero during Normalization:**
   Attempting to normalize a zero vector $\mathbf{0}$ yields `NaN` or `inf`. Always check if `norm > 1e-12` before dividing.
3. **Subtracting Points vs Subtracting Vectors:**
   Point $-$ Point $=$ Displacement Vector. Point $+$ Vector $=$ New Point. Adding two spatial points (e.g., Target $+$ Robot) has no physical meaning!

---

## 8. Exercises

### Beginner
1. Compute the magnitude of the vector $\mathbf{v} = [2.0, -3.0, 6.0]^T$.
2. Normalize the vector $\mathbf{u} = [0.0, 5.0, 0.0]^T$ into a unit vector.

### Intermediate
3. A drone is hovering at position $\mathbf{p}_0 = [10, 5, 20]^T\,\text{m}$. A gust of wind exerts an acceleration $\mathbf{a} = [-1.0, 0.5, -0.2]^T\,\text{m/s}^2$ for $t = 3.0\,\text{s}$. If initial velocity is $\mathbf{v}_0 = [1, 0, 0]^T\,\text{m/s}$, calculate its final position $\mathbf{p}(t) = \mathbf{p}_0 + \mathbf{v}_0 t + \frac{1}{2}\mathbf{a}t^2$.
4. Prove that the unit vector $\mathbf{\hat{v}} = \mathbf{v}/\|\mathbf{v}\|$ always has a norm of exactly $1.0$.

### Advanced
5. In a robot joint space, the configuration is represented by a 6D vector $\mathbf{q} \in \mathbb{R}^6$. Define the weighted Euclidean distance metric $d_{\mathbf{W}}(\mathbf{q}_1, \mathbf{q}_2) = \sqrt{(\mathbf{q}_1 - \mathbf{q}_2)^T \mathbf{W} (\mathbf{q}_1 - \mathbf{q}_2)}$ where $\mathbf{W}$ is a positive-definite diagonal matrix. Why do robotics motion planners use weighted distance metrics instead of standard Euclidean distance when comparing arm configurations?

---

## 9. Summary & Next Steps
- Vectors represent directed quantities like displacements, velocities, forces, and joint states.
- Normalizing a vector isolates pure direction.
- Next, learn how matrices transform vectors in [**02: Matrices**](02-matrices.md).
