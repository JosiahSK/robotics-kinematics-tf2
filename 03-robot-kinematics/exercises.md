# Module 03: Exercises — Robot Kinematics

Test your understanding of serial manipulators, forward/inverse kinematics, DH tables, and Jacobians across three difficulty levels. Complete solutions are in [`solutions.md`](solutions.md).

---

## Beginner Level

### Exercise 3.1: DH Parameter Identification
Identify the four Denavit–Hartenberg parameters for the following link:
- Two joint axes $z_{i-1}$ and $z_i$ are parallel and separated by a perpendicular distance of $0.65\,\text{m}$.
- There is no axial displacement along $z_{i-1}$ ($d_i = 0$).
- Joint $i$ is revolute and currently rotated by $30^\circ$.
State the values of $\theta_i, d_i, a_i,$ and $\alpha_i$.

### Exercise 3.2: 2-Link Planar Forward Kinematics
A 2-link planar arm has link lengths $l_1 = 0.70\,\text{m}$ and $l_2 = 0.50\,\text{m}$.
Calculate the end-effector $(x, y)$ position when $\theta_1 = 90^\circ$ and $\theta_2 = -90^\circ$.

### Exercise 3.3: Reachability Bounds
A 2-link planar manipulator has $l_1 = 1.2\,\text{m}$ and $l_2 = 0.8\,\text{m}$.
Which of the following Cartesian targets are physically reachable?
- Point A: $(1.0, 1.0)\,\text{m}$
- Point B: $(1.8, 1.2)\,\text{m}$
- Point C: $(0.1, 0.2)\,\text{m}$

---

## Intermediate Level

### Exercise 3.4: 2-Link Planar Analytical Inverse Kinematics
For the arm with $l_1 = 1.0\,\text{m}$ and $l_2 = 0.8\,\text{m}$:
Compute both the **Elbow-Down** and **Elbow-Up** joint configurations $(\theta_1, \theta_2)$ in degrees to reach target $(x = 1.0\,\text{m}, y = 1.0\,\text{m})$.

### Exercise 3.5: Spherical Wrist Center Decoupling
An industrial 6-DOF robot with a spherical wrist has a tool length $d_6 = 0.15\,\text{m}$ along the approach axis $Z_6$.
The commanded end-effector target pose has position:
$$\mathbf{p}_d = [0.80, \; 0.30, \; 0.60]^T\,\text{m}$$
and tool approach vector:
$$\mathbf{a}_d = [0.00, \; 0.00, \; 1.00]^T$$
1. Calculate the 3D coordinates of the wrist center $\mathbf{p}_{wc}$.
2. If Joint 1 is a waist rotation about the vertical base $Z$-axis, compute the base angle $\theta_1$ in degrees.

### Exercise 3.6: Geometric Jacobian Calculation
A 2-link planar arm has $l_1 = 1.0\,\text{m}$ and $l_2 = 1.0\,\text{m}$.
At configuration $\theta_1 = 45^\circ, \theta_2 = 0^\circ$:
1. Calculate the $2 \times 2$ Jacobian matrix $\mathbf{J}$.
2. Calculate its determinant $\det(\mathbf{J})$.
3. What does this determinant value indicate about the arm's instantaneous mobility?

---

## Advanced Level

### Exercise 3.7: 6-DOF Wrist Orientation Extraction
In a decoupled 6-DOF manipulator, the first three joints have been solved, giving:
$$\mathbf{R}_0^3 = \begin{bmatrix} 0 & 0 & 1 \\ 0 & 1 & 0 \\ -1 & 0 & 0 \end{bmatrix}$$
The desired end-effector orientation is:
$$\mathbf{R}_d = \begin{bmatrix} 0 & -1 & 0 \\ 0 & 0 & 1 \\ -1 & 0 & 0 \end{bmatrix}$$
1. Compute the required wrist rotation matrix $\mathbf{R}_3^6 = (\mathbf{R}_0^3)^T \mathbf{R}_d$.
2. Extract the wrist joint angles $(\theta_4, \theta_5, \theta_6)$ assuming standard Z-Y-Z spherical wrist Euler geometry.

### Exercise 3.8: Damped Least Squares at Singularity
A 2-DoF manipulator with Jacobian $\mathbf{J} = \begin{bmatrix} 0.0 & 0.0 \\ 1.0 & 0.5 \end{bmatrix}$ is asked to track error $\mathbf{e} = [0.2, 0.0]^T$.
1. Explain why standard pseudo-inverse $\mathbf{J}^\dagger = \mathbf{J}^T (\mathbf{J}\mathbf{J}^T)^{-1}$ fails.
2. Compute the Damped Least Squares update $\Delta \mathbf{q} = \mathbf{J}^T (\mathbf{J}\mathbf{J}^T + \lambda^2 \mathbf{I})^{-1} \mathbf{e}$ using damping factor $\lambda = 0.1$.
3. Show that the resulting joint velocity vector $\Delta \mathbf{q}$ is strictly finite.
