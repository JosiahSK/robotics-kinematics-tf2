# Module 54: Solutions — Mathematics for Robotics

Step-by-step verified solutions for [Module 54 Exercises](exercises.md).

---

## Solution 2.1: Vector Norm & Orthogonality
1. **Euclidean Norm:**
   $$\|\mathbf{u}\| = \sqrt{3.0^2 + (-4.0)^2 + 0.0^2} = \sqrt{9.0 + 16.0 + 0} = \sqrt{25.0} = 5.0$$
2. **Scalar Dot Product:**
   $$\mathbf{u} \cdot \mathbf{v} = (3.0)(4.0) + (-4.0)(3.0) + (0.0)(5.0) = 12.0 - 12.0 + 0.0 = 0.0$$
3. **Orthogonality:**
   Yes, $\mathbf{u}$ and $\mathbf{v}$ are strictly orthogonal because their scalar dot product is identically zero ($\mathbf{u} \cdot \mathbf{v} = 0$).

---

## Solution 2.2: Matrix Inverse & Determinant
1. **Determinant:**
   $$\det(\mathbf{A}) = (2.0)(4.0) - (3.0)(1.0) = 8.0 - 3.0 = 5.0$$
2. **Analytical Inverse:**
   $$\mathbf{A}^{-1} = \frac{1}{\det(\mathbf{A})} \begin{bmatrix} 4.0 & -3.0 \\ -1.0 & 2.0 \end{bmatrix} = \frac{1}{5.0} \begin{bmatrix} 4.0 & -3.0 \\ -1.0 & 2.0 \end{bmatrix} = \begin{bmatrix} 0.80 & -0.60 \\ -0.20 & 0.40 \end{bmatrix}$$
3. **Verification:**
   $$\mathbf{A} \mathbf{A}^{-1} = \begin{bmatrix} 2 & 3 \\ 1 & 4 \end{bmatrix} \begin{bmatrix} 0.8 & -0.6 \\ -0.2 & 0.4 \end{bmatrix} = \begin{bmatrix} 2(0.8) + 3(-0.2) & 2(-0.6) + 3(0.4) \\ 1(0.8) + 4(-0.2) & 1(-0.6) + 4(0.4) \end{bmatrix} = \begin{bmatrix} 1.6 - 0.6 & -1.2 + 1.2 \\ 0.8 - 0.8 & -0.6 + 1.6 \end{bmatrix} = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix} = \mathbf{I}_2$$

---

## Solution 2.3: Variance and Standard Deviation
1. **Encoder 1:**
   - Standard deviation $\sigma = \sqrt{\sigma^2} = \sqrt{0.0064} = 0.08\,\text{m} = 8.0\,\text{cm}$.
2. **Encoder 2:**
   - Variance $\sigma^2 = (0.12\,\text{m})^2 = 0.0144\,\text{m}^2$.

---

## Solution 2.4: 3D Cross Product and Torque
$\mathbf{r} = [0.5, 0.0, 0.2]^T\,\text{m}$, $\mathbf{F} = [0.0, 0.0, -50.0]^T\,\text{N}$.

1. **Determinant Method:**
   $$\boldsymbol{\tau} = \mathbf{r} \times \mathbf{F} = \det\begin{bmatrix} \mathbf{\hat{i}} & \mathbf{\hat{j}} & \mathbf{\hat{k}} \\ 0.5 & 0.0 & 0.2 \\ 0.0 & 0.0 & -50.0 \end{bmatrix}$$
   - $\tau_x = (0.0)(-50.0) - (0.2)(0.0) = 0.0\,\text{N}\cdot\text{m}$
   - $\tau_y = -[(0.5)(-50.0) - (0.2)(0.0)] = -[-25.0] = +25.0\,\text{N}\cdot\text{m}$
   - $\tau_z = (0.5)(0.0) - (0.0)(0.0) = 0.0\,\text{N}\cdot\text{m}$
   $$\boldsymbol{\tau} = [0.0, +25.0, 0.0]^T\,\text{N}\cdot\text{m}$$

2. **Skew-Symmetric Matrix Method:**
   $$[\mathbf{r}]_\times = \begin{bmatrix} 0 & -0.2 & 0.0 \\ 0.2 & 0 & -0.5 \\ -0.0 & 0.5 & 0 \end{bmatrix}$$
   $$\boldsymbol{\tau} = [\mathbf{r}]_\times \mathbf{F} = \begin{bmatrix} 0 & -0.2 & 0.0 \\ 0.2 & 0 & -0.5 \\ 0.0 & 0.5 & 0 \end{bmatrix} \begin{bmatrix} 0 \\ 0 \\ -50 \end{bmatrix} = \begin{bmatrix} 0 \\ -0.5(-50) \\ 0 \end{bmatrix} = \begin{bmatrix} 0.0 \\ +25.0 \\ 0.0 \end{bmatrix}\,\text{N}\cdot\text{m}$$
3. Both methods yield $[0.0, 25.0, 0.0]^T\,\text{N}\cdot\text{m}$.

---

## Solution 2.5: Planar 2-Link Robot Jacobian
$l_1 = 0.8\,\text{m}, l_2 = 0.6\,\text{m}$, $\theta_1 = 30^\circ, \theta_2 = 60^\circ \implies \theta_1 + \theta_2 = 90^\circ$.

1. **Forward Kinematics:**
   $$x = l_1 \cos(30^\circ) + l_2 \cos(90^\circ) = 0.8(\sqrt{3}/2) + 0.6(0) = 0.8(0.8660) = 0.6928\,\text{m}$$
   $$y = l_1 \sin(30^\circ) + l_2 \sin(90^\circ) = 0.8(0.5000) + 0.6(1.0) = 0.4000 + 0.6000 = 1.0000\,\text{m}$$
   $$\mathbf{p} = [0.6928, 1.0000]^T\,\text{m}$$

2. **Jacobian Matrix $\mathbf{J}$:**
   $$\mathbf{J} = \begin{bmatrix} -l_1 \sin(\theta_1) - l_2 \sin(\theta_1+\theta_2) & -l_2 \sin(\theta_1+\theta_2) \\ l_1 \cos(\theta_1) + l_2 \cos(\theta_1+\theta_2) & l_2 \cos(\theta_1+\theta_2) \end{bmatrix}$$
   $$\mathbf{J} = \begin{bmatrix} -0.8(0.5) - 0.6(1.0) & -0.6(1.0) \\ 0.8(0.8660) + 0.6(0) & 0.6(0) \end{bmatrix} = \begin{bmatrix} -0.40 - 0.60 & -0.60 \\ 0.6928 + 0.0 & 0.0 \end{bmatrix} = \begin{bmatrix} -1.0000 & -0.6000 \\ +0.6928 & 0.0000 \end{bmatrix}$$

3. **Linear Velocity with $\mathbf{\dot{\theta}} = [0.5, -0.5]^T\,\text{rad/s}$:**
   $$\begin{bmatrix} v_x \\ v_y \end{bmatrix} = \mathbf{J} \mathbf{\dot{\theta}} = \begin{bmatrix} -1.0000 & -0.6000 \\ 0.6928 & 0.0000 \end{bmatrix} \begin{bmatrix} 0.5 \\ -0.5 \end{bmatrix} = \begin{bmatrix} -1.0(0.5) - 0.6(-0.5) \\ 0.6928(0.5) + 0 \end{bmatrix} = \begin{bmatrix} -0.50 + 0.30 \\ 0.3464 \end{bmatrix} = \begin{bmatrix} -0.2000 \\ +0.3464 \end{bmatrix}\,\text{m/s}$$

---

## Solution 2.6: Bayes' Sensor Update
- Prior: $P(\text{Obs}) = 0.30 \implies P(\neg\text{Obs}) = 0.70$.
- Likelihoods:
  - $P(\text{Trig} \mid \text{Obs}) = 0.90$
  - $P(\text{Trig} \mid \neg\text{Obs}) = 0.05$
- Numerator for Obstacle:
  $$\text{num}(\text{Obs}) = P(\text{Trig} \mid \text{Obs}) P(\text{Obs}) = (0.90)(0.30) = 0.270$$
- Numerator for No Obstacle:
  $$\text{num}(\neg\text{Obs}) = P(\text{Trig} \mid \neg\text{Obs}) P(\neg\text{Obs}) = (0.05)(0.70) = 0.035$$
- Total Evidence $P(\text{Trig}) = 0.270 + 0.035 = 0.305$.
- Normalized Posterior:
  $$P(\text{Obs} \mid \text{Trig}) = \frac{0.270}{0.305} \approx 0.8852 = 88.52\%$$

---

## Solution 2.7: Error Ellipse Eigenvalue Derivation
$\mathbf{\Sigma} = \begin{bmatrix} 0.09 & 0.03 \\ 0.03 & 0.05 \end{bmatrix}$

1. **Trace & Determinant:**
   - $\text{tr}(\mathbf{\Sigma}) = 0.09 + 0.05 = 0.14$.
   - $\det(\mathbf{\Sigma}) = (0.09)(0.05) - (0.03)^2 = 0.0045 - 0.0009 = 0.0036$.
2. **Eigenvalues:**
   $$\lambda^2 - 0.14\lambda + 0.0036 = 0$$
   $$\lambda = \frac{0.14 \pm \sqrt{0.14^2 - 4(0.0036)}}{2} = \frac{0.14 \pm \sqrt{0.0196 - 0.0144}}{2} = \frac{0.14 \pm \sqrt{0.0052}}{2} = \frac{0.14 \pm 0.07211}{2}$$
   $$\lambda_1 = 0.10606\,\text{m}^2, \quad \lambda_2 = 0.03394\,\text{m}^2$$
3. **Eigenvectors:**
   For $\lambda_1 = 0.10606$:
   $$(0.09 - 0.10606)v_x + 0.03 v_y = 0 \implies -0.01606 v_x + 0.03 v_y = 0 \implies v_y = \frac{0.01606}{0.03} v_x = 0.5353 v_x$$
   Normalizing: $\mathbf{v}_1 = [0.8817, 0.4719]^T$.
4. **Tilt Angle:**
   $$\theta = \arctan(0.5353) \approx 28.16^\circ$$
5. **95% Semi-Axis Lengths ($s = 2.447$):**
   - Semi-major: $a = 2.447 \sqrt{0.10606} = 2.447(0.3257) = 0.797\,\text{m}$.
   - Semi-minor: $b = 2.447 \sqrt{0.03394} = 2.447(0.1842) = 0.451\,\text{m}$.

---

## Solution 2.8: Linearization of Non-Linear Kinematics
1. **Jacobian $\mathbf{H}$:**
   $$\mathbf{H} = \begin{bmatrix} \frac{\partial (r\cos\theta)}{\partial r} & \frac{\partial (r\cos\theta)}{\partial \theta} \\ \frac{\partial (r\sin\theta)}{\partial r} & \frac{\partial (r\sin\theta)}{\partial \theta} \end{bmatrix} = \begin{bmatrix} \cos(\theta) & -r\sin(\theta) \\ \sin(\theta) & r\cos(\theta) \end{bmatrix}$$
2. **Evaluation at $r = 5.0\,\text{m}, \theta = 45^\circ$ ($\cos=\sin=\frac{\sqrt{2}}{2} \approx 0.7071$):**
   $$\mathbf{H} = \begin{bmatrix} 0.7071 & -5(0.7071) \\ 0.7071 & 5(0.7071) \end{bmatrix} = \begin{bmatrix} 0.7071 & -3.5355 \\ 0.7071 & 3.5355 \end{bmatrix}$$
   Given $\mathbf{\Sigma}_{r\theta} = \begin{bmatrix} 0.01 & 0 \\ 0 & 0.0025 \end{bmatrix}$:
   $$\mathbf{\Sigma}_{xy} = \mathbf{H} \mathbf{\Sigma}_{r\theta} \mathbf{H}^T$$
   $$\mathbf{H} \mathbf{\Sigma}_{r\theta} = \begin{bmatrix} 0.7071(0.01) & -3.5355(0.0025) \\ 0.7071(0.01) & 3.5355(0.0025) \end{bmatrix} = \begin{bmatrix} 0.007071 & -0.008839 \\ 0.007071 & 0.008839 \end{bmatrix}$$
   Multiply by $\mathbf{H}^T$:
   $$\mathbf{\Sigma}_{xy} = \begin{bmatrix} 0.007071 & -0.008839 \\ 0.007071 & 0.008839 \end{bmatrix} \begin{bmatrix} 0.7071 & 0.7071 \\ -3.5355 & 3.5355 \end{bmatrix}$$
   $$\mathbf{\Sigma}_{xy} = \begin{bmatrix} (0.0050 + 0.03125) & (0.0050 - 0.03125) \\ (0.0050 - 0.03125) & (0.0050 + 0.03125) \end{bmatrix} = \begin{bmatrix} 0.03625 & -0.02625 \\ -0.02625 & 0.03625 \end{bmatrix}\,\text{m}^2$$
