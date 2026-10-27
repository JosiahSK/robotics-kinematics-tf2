# Assessment: Module 54 — Mathematics for Robotics

**Course:** Robotics & AI Engineering Notes  
**Module:** 54 — Linear Algebra, Calculus, and Probability for Robotics  
**Total Marks:** 50 Points  
**Format:** Closed-book, scientific calculator permitted.

---

## Section A: Multiple Choice Questions (10 Points, 2 Points Each)

1. For any 3D unit vector $\mathbf{k}$, what is the trace of the skew-symmetric cross-product matrix $[\mathbf{k}]_\times$?
   - A) $1.0$
   - B) $-1.0$
   - C) $0.0$
   - D) $3.0$

2. If a $3 \times 3$ matrix $\mathbf{R}$ satisfies $\mathbf{R}^T \mathbf{R} = \mathbf{I}$ and $\det(\mathbf{R}) = -1.0$, what transformation does it represent?
   - A) A proper rotation in $SO(3)$
   - B) An affine shear
   - C) A reflection (improper rotation)
   - D) A singular collapse

3. What are the physical units of covariance $\text{Cov}(x, y)$ if $x$ is position in metres ($\text{m}$) and $y$ is angle in radians ($\text{rad}$)?
   - A) Dimensionless
   - B) $\text{m} \cdot \text{rad}$
   - C) $\text{m}^2$
   - D) $\text{rad}^2$

4. Which mathematical property guarantees that fusing two independent Gaussian sensor estimates always produces a fused estimate with smaller uncertainty than either sensor alone?
   - A) Orthogonality of basis vectors
   - B) Product of Gaussians variance formula: $\sigma'^2 = \frac{\sigma_1^2 \sigma_2^2}{\sigma_1^2 + \sigma_2^2} < \min(\sigma_1^2, \sigma_2^2)$
   - C) The Jacobian transpose condition
   - D) Determinant scaling

5. In the multivariable chain rule, if $f(q_1, q_2)$ is a function of joint angles that change over time, how is $\frac{df}{dt}$ calculated?
   - A) $\frac{\partial f}{\partial q_1} + \frac{\partial f}{\partial q_2}$
   - B) $\frac{\partial f}{\partial q_1}\dot{q}_1 + \frac{\partial f}{\partial q_2}\dot{q}_2$
   - C) $\frac{\partial f}{\partial q_1}\frac{\partial f}{\partial q_2} \dot{q}_1 \dot{q}_2$
   - D) $\frac{d^2f}{dt^2}$

---

## Section B: Linear Algebra & Calculus Problems (20 Points)

### Question 6 (10 Points): Skew-Symmetric Cross Product Equivalence
Given vector $\boldsymbol{\omega} = [0, 0, 3]^T\,\text{rad/s}$ and position vector $\mathbf{r} = [2, 1, 0]^T\,\text{m}$:
1. Compute the linear velocity $\mathbf{v} = \boldsymbol{\omega} \times \mathbf{r}$ using the standard 3D determinant formula.
2. Form the $3 \times 3$ skew-symmetric matrix $[\boldsymbol{\omega}]_\times$.
3. Compute $\mathbf{v} = [\boldsymbol{\omega}]_\times \mathbf{r}$ and show that both methods match exactly.
4. Prove that $\mathbf{v} \cdot \mathbf{r} = 0$, confirming perpendicularity.

### Question 7 (10 Points): Jacobian & Gradient Descent
A 1-DoF arm rotates an arm of length $L = 1.0\,\text{m}$ in a plane:
$$x(\theta) = L \cos(\theta)$$
The desired target is $x_d = 0.50\,\text{m}$. The current joint angle is $\theta_0 = 30^\circ$ ($\pi/6\,\text{rad}$).
1. Calculate the current position $x(\theta_0)$ and the tracking error $e = x_d - x(\theta_0)$.
2. Compute the 1D Jacobian $J = \frac{\partial x}{\partial \theta}$ at $\theta_0$.
3. Perform one step of Gradient Descent with learning rate $\alpha = 0.4$:
   $$\theta_1 = \theta_0 + \alpha J^T e$$
4. Check if the updated angle $\theta_1$ reduces the tracking error.

---

## Section C: Probability & Sensor Fusion (20 Points)

### Question 8 (10 Points): 1D Kalman Sensor Fusion
An automated warehouse AGV measures distance to a reflector:
- Prior belief (dead reckoning): $\mu_1 = 5.00\,\text{m}$, standard deviation $\sigma_1 = 0.30\,\text{m}$.
- LiDAR measurement: $\mu_2 = 5.30\,\text{m}$, standard deviation $\sigma_2 = 0.10\,\text{m}$.
1. Calculate the variances $\sigma_1^2$ and $\sigma_2^2$.
2. Compute the Kalman Gain $K$.
3. Calculate the fused mean position $\mu'$.
4. Calculate the fused standard deviation $\sigma'$.

### Question 9 (10 Points): Covariance Matrix & Error Ellipse
A robot state estimator outputs the 2D position covariance:
$$\mathbf{\Sigma} = \begin{bmatrix} 0.16 & 0.00 \\ 0.00 & 0.04 \end{bmatrix}\,\text{m}^2$$
1. What are the standard deviations along $X$ and $Y$?
2. What is the correlation coefficient $\rho_{xy}$?
3. Find the eigenvalues and eigenvectors of $\mathbf{\Sigma}$.
4. Sketch the orientation and semi-axis lengths of the 1-sigma uncertainty ellipse.
