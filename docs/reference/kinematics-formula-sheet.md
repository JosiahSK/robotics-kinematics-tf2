# Kinematics Formula Sheet

A quick-reference sheet of verified mathematical formulas for 2D and 3D manipulator kinematics, homogeneous transformations, Jacobian differential kinematics, and numerical solvers.

---

## 1. 2D Planar Kinematics (Two-Link Manipulator)

### Forward Kinematics:
$$\begin{aligned}
x &= l_1 \cos\theta_1 + l_2 \cos(\theta_1 + \theta_2) \\
y &= l_1 \sin\theta_1 + l_2 \sin(\theta_1 + \theta_2) \\
\phi &= \theta_1 + \theta_2 \quad (\text{tool orientation})
\end{aligned}$$

### Inverse Kinematics (Law of Cosines):
Radial reach distance: $r = \sqrt{x^2 + y^2}$. Reachability criterion: $|l_1 - l_2| \le r \le l_1 + l_2$.

$$\cos\theta_2 = \frac{x^2 + y^2 - l_1^2 - l_2^2}{2 l_1 l_2}, \quad \sin\theta_2 = \pm\sqrt{1 - \cos^2\theta_2}$$
- **Elbow-Down Branch:** $\theta_2 = +\arccos(\cos\theta_2)$
- **Elbow-Up Branch:** $\theta_2 = -\arccos(\cos\theta_2)$
- **Base Angle:** $\theta_1 = \text{atan2}(y, x) - \text{atan2}(l_2 \sin\theta_2, \; l_1 + l_2 \cos\theta_2)$

---

## 2. Spatial Transformations in $SE(3)$

### Standard Denavit–Hartenberg (DH) Matrix:
$$\mathbf{A}_i = \text{Rot}_z(\theta_i) \text{Trans}_z(d_i) \text{Trans}_x(a_i) \text{Rot}_x(\alpha_i) = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \\
\sin\theta_i & \cos\theta_i \cos\alpha_i & -\cos\theta_i \sin\alpha_i & a_i \sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### Fast Homogeneous Transformation Inversion:
For any $\mathbf{T} = \begin{bmatrix} \mathbf{R} & \mathbf{p} \\ \mathbf{0} & 1 \end{bmatrix} \in SE(3)$:
$$\mathbf{T}^{-1} = \begin{bmatrix} \mathbf{R}^T & -\mathbf{R}^T \mathbf{p} \\ \mathbf{0} & 1 \end{bmatrix}$$

---

## 3. 6-DOF Spherical Wrist Analytical Decoupling (EduArm-6DOF)

Given target pose $\mathbf{T}_{\text{target}} = \begin{bmatrix} \mathbf{R}_d & \mathbf{p}_d \\ \mathbf{0} & 1 \end{bmatrix}$ with approach vector $\mathbf{a}_d = \mathbf{R}_d[:, 2]$:

### Step 1: Wrist Center Decoupling
$$\mathbf{p}_{wc} = \begin{bmatrix} x_w \\ y_w \\ z_w \end{bmatrix} = \mathbf{p}_d - d_6 \mathbf{a}_d$$

### Step 2: Base Angle ($\theta_1$)
$$\theta_1 = \text{atan2}(y_w, \; x_w) \quad \text{or} \quad \theta_1' = \text{atan2}(-y_w, \; -x_w)$$

### Step 3: Elbow ($\theta_3$) and Shoulder ($\theta_2$)
Planar coordinates: $r = \sqrt{x_w^2 + y_w^2}, \quad s = z_w - d_1, \quad D^2 = r^2 + s^2$.

For the canonical DH model ($\alpha_3 = +90^\circ$):
$$\sin\theta_3 = \frac{D^2 - a_2^2 - d_4^2}{2 a_2 d_4}$$
- **Elbow-Down:** $\theta_3 = \arcsin(\sin\theta_3)$
- **Elbow-Up:** $\theta_3 = \pi - \arcsin(\sin\theta_3)$

Shoulder angle from linear decomposition ($r = A\cos\theta_2 + B\sin\theta_2, \; s = A\sin\theta_2 - B\cos\theta_2$):
$$A = a_2 + d_4 \sin\theta_3, \quad B = d_4 \cos\theta_3 \implies \theta_2 = \text{atan2}(A s + B r, \; A r - B s)$$

### Step 4: Wrist Angles ($\theta_4, \theta_5, \theta_6$)
Isolate wrist rotation: $\mathbf{R}_3^6 = (\mathbf{R}_0^3)^T \mathbf{R}_d$.
- $\cos\theta_5 = \mathbf{R}_3^6[2, 2] \implies \theta_5 = \pm\arccos(\mathbf{R}_3^6[2, 2])$
- For $\sin\theta_5 \ne 0$:
  $$\theta_4 = \text{atan2}\left(\frac{\mathbf{R}_3^6[1, 2]}{\sin\theta_5}, \; \frac{\mathbf{R}_3^6[0, 2]}{\sin\theta_5}\right), \quad \theta_6 = \text{atan2}\left(\frac{\mathbf{R}_3^6[2, 1]}{\sin\theta_5}, \; \frac{-\mathbf{R}_3^6[2, 0]}{\sin\theta_5}\right)$$
- At wrist singularity ($\theta_5 \approx 0$): set $\theta_4 = 0, \; \theta_6 = \text{atan2}(\mathbf{R}_3^6[1, 0], \; \mathbf{R}_3^6[0, 0])$.

---

## 4. Jacobian Differential Kinematics & Velocity Control

### Geometric Jacobian Column Formula:
For revolute joint $i$ with joint axis $\mathbf{z}_{i-1}$ and frame origin $\mathbf{p}_{i-1}$:
$$\mathbf{J}_i = \begin{bmatrix} \mathbf{J}_{v, i} \\ \mathbf{J}_{\omega, i} \end{bmatrix} = \begin{bmatrix} \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1}) \\ \mathbf{z}_{i-1} \end{bmatrix}$$

### Damped Least Squares (DLS / SVD Inversion):
For Jacobian $\mathbf{J} = \mathbf{U} \boldsymbol{\Sigma} \mathbf{V}^T$ and damping factor $\lambda$:
$$\mathbf{J}_{\text{dls}} = \mathbf{V} \operatorname{diag}\left(\frac{\sigma_i}{\sigma_i^2 + \lambda^2}\right) \mathbf{U}^T$$

### Exact $SO(3)$ Orientation Error Vector:
Defined in base frame: $\mathbf{R}_{\text{err}} = \mathbf{R}_d \mathbf{R}_{\text{curr}}^T$.
$$\theta = \arccos\left(\frac{\text{trace}(\mathbf{R}_{\text{err}}) - 1}{2}\right), \quad \mathbf{e}_o = \frac{\theta}{2\sin\theta} \begin{bmatrix} R_{32}-R_{23} \\ R_{13}-R_{31} \\ R_{21}-R_{12} \end{bmatrix}$$
When $\theta \approx \pi$, extract axis $\mathbf{k}$ from symmetric part: $\mathbf{k}\mathbf{k}^T = \frac{1}{2}(\mathbf{R}_{\text{err}} + \mathbf{I})$.

### Yoshikawa Manipulability Measure:
$$w(\mathbf{q}) = \sqrt{\det(\mathbf{J}\mathbf{J}^T)} = \sigma_1 \sigma_2 \cdots \sigma_m$$
$w(\mathbf{q}) = 0$ indicates a kinematic singularity.
