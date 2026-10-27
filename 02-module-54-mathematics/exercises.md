# Module 54: Exercises — Mathematics for Robotics

Test your mathematical comprehension across linear algebra, calculus, and probability. Complete step-by-step solutions are provided in [`solutions.md`](solutions.md).

---

## Beginner Level

### Exercise 2.1: Vector Norm & Orthogonality
Given the vectors $\mathbf{u} = [3.0, -4.0, 0.0]^T$ and $\mathbf{v} = [4.0, 3.0, 5.0]^T$:
1. Calculate the Euclidean norm $\|\mathbf{u}\|$.
2. Calculate the scalar dot product $\mathbf{u} \cdot \mathbf{v}$.
3. Are the vectors $\mathbf{u}$ and $\mathbf{v}$ orthogonal? Justify your answer.

### Exercise 2.2: Matrix Inverse & Determinant
Given the matrix $\mathbf{A} = \begin{bmatrix} 2.0 & 3.0 \\ 1.0 & 4.0 \end{bmatrix}$:
1. Compute the determinant $\det(\mathbf{A})$.
2. Compute the analytical inverse $\mathbf{A}^{-1}$.
3. Verify that $\mathbf{A} \mathbf{A}^{-1} = \mathbf{I}_2$.

### Exercise 2.3: Variance and Standard Deviation
A wheel encoder measuring distance travels over 10 trials with variance $\sigma^2 = 0.0064\,\text{m}^2$.
1. What is the standard deviation $\sigma$ in metres and in centimetres?
2. If another encoder has standard deviation $\sigma = 0.12\,\text{m}$, what is its variance?

---

## Intermediate Level

### Exercise 2.4: 3D Cross Product and Torque
A robotic arm joint is located at the origin. A tool attachment extends to position $\mathbf{r} = [0.5, 0.0, 0.2]^T\,\text{m}$.
An external payload pulls downward with a force $\mathbf{F} = [0.0, 0.0, -50.0]^T\,\text{N}$.
1. Compute the torque vector $\boldsymbol{\tau} = \mathbf{r} \times \mathbf{F}$ using the determinant definition.
2. Form the $3 \times 3$ skew-symmetric matrix $[\mathbf{r}]_\times$ and compute the torque via matrix-vector multiplication $[\mathbf{r}]_\times \mathbf{F}$.
3. Verify both methods produce the exact same result.

### Exercise 2.5: Planar 2-Link Robot Jacobian
A 2-link planar manipulator has link lengths $l_1 = 0.8\,\text{m}$ and $l_2 = 0.6\,\text{m}$.
At configuration $\theta_1 = 30^\circ$ ($\pi/6\,\text{rad}$) and $\theta_2 = 60^\circ$ ($\pi/3\,\text{rad}$):
1. Compute the forward kinematics position $(x, y)$.
2. Calculate the numerical entries of the $2 \times 2$ Jacobian matrix $\mathbf{J}(\theta_1, \theta_2)$.
3. If Joint 1 rotates at $\dot{\theta}_1 = 0.5\,\text{rad/s}$ and Joint 2 rotates at $\dot{\theta}_2 = -0.5\,\text{rad/s}$, calculate the linear velocity of the end-effector $[v_x, v_y]^T$.

### Exercise 2.6: Bayes' Sensor Update
A robot is attempting to detect an obstacle in front of its gripper.
- Prior probability that an obstacle is present: $P(\text{Obstacle}) = 0.30$.
- Sensor true positive rate: $P(\text{Trigger} \mid \text{Obstacle}) = 0.90$.
- Sensor false positive rate: $P(\text{Trigger} \mid \text{No Obstacle}) = 0.05$.
If the sensor triggers, compute the posterior probability $P(\text{Obstacle} \mid \text{Trigger})$.

---

## Advanced Level

### Exercise 2.7: Error Ellipse Eigenvalue Derivation
A robot localization filter produces the 2D position covariance matrix:
$$\mathbf{\Sigma} = \begin{bmatrix} 0.09 & 0.03 \\ 0.03 & 0.05 \end{bmatrix}\,\text{m}^2$$
1. Calculate the trace and determinant of $\mathbf{\Sigma}$.
2. Compute the exact eigenvalues $\lambda_1, \lambda_2$.
3. Compute the normalized eigenvectors $\mathbf{v}_1, \mathbf{v}_2$.
4. Determine the tilt angle $\theta$ of the uncertainty ellipse in degrees.
5. Calculate the semi-major and semi-minor axis lengths for a $95\%$ confidence ellipse ($s = \sqrt{5.991} \approx 2.447$).

### Exercise 2.8: Linearization of Non-Linear Kinematics
Given the non-linear polar-to-Cartesian measurement function:
$$\mathbf{h}(r, \theta) = \begin{bmatrix} r \cos(\theta) \\ r \sin(\theta) \end{bmatrix}$$
1. Compute the $2 \times 2$ Jacobian matrix of partial derivatives $\mathbf{H} = \frac{\partial \mathbf{h}}{\partial [r, \theta]^T}$.
2. If radar measurement estimates are $r = 5.0\,\text{m}, \theta = 45^\circ$, and their covariance is $\mathbf{\Sigma}_{r\theta} = \text{diag}(0.01\,\text{m}^2, \; 0.0025\,\text{rad}^2)$, compute the Cartesian covariance matrix $\mathbf{\Sigma}_{xy} = \mathbf{H} \mathbf{\Sigma}_{r\theta} \mathbf{H}^T$.
