# Module 03: Solutions — Robot Kinematics

Step-by-step verified solutions for [Module 03 Exercises](exercises.md).

---

## Solution 3.1: DH Parameter Identification
- Joint angle: $\theta_i = 30^\circ$ ($\pi/6\,\text{rad}$)
- Link offset: $d_i = 0.0\,\text{m}$ (no axial displacement)
- Link length: $a_i = 0.65\,\text{m}$ (perpendicular distance between axes along the common normal)
- Link twist: $\alpha_i = 0.0^\circ$ ($0\,\text{rad}$) (axes are parallel, so angle between them is zero)

---

## Solution 3.2: 2-Link Planar Forward Kinematics
$l_1 = 0.70\,\text{m}, l_2 = 0.50\,\text{m}$, $\theta_1 = 90^\circ, \theta_2 = -90^\circ \implies \theta_1 + \theta_2 = 0^\circ$.

1. **$X$-coordinate:**
   $$x = l_1 \cos(90^\circ) + l_2 \cos(0^\circ) = 0.70(0) + 0.50(1) = 0.50\,\text{m}$$
2. **$Y$-coordinate:**
   $$y = l_1 \sin(90^\circ) + l_2 \sin(0^\circ) = 0.70(1) + 0.50(0) = 0.70\,\text{m}$$
**Answer:** The end-effector is at $(x = 0.50\,\text{m}, y = 0.70\,\text{m})$.

---

## Solution 3.3: Reachability Bounds
$l_1 = 1.2\,\text{m}, l_2 = 0.8\,\text{m}$.
- Maximum outer reach: $r_{\max} = 1.2 + 0.8 = 2.0\,\text{m}$.
- Minimum inner reach: $r_{\min} = |1.2 - 0.8| = 0.4\,\text{m}$.

Evaluate target distances $r = \sqrt{x^2 + y^2}$:
- **Point A $(1.0, 1.0)$:** $r = \sqrt{1.0^2 + 1.0^2} = \sqrt{2.0} \approx 1.414\,\text{m}$.
  Since $0.4 \le 1.414 \le 2.0$, **Point A is REACHABLE**.
- **Point B $(1.8, 1.2)$:** $r = \sqrt{1.8^2 + 1.2^2} = \sqrt{3.24 + 1.44} = \sqrt{4.68} \approx 2.163\,\text{m}$.
  Since $2.163 > 2.0$, **Point B is UNREACHABLE (too far)**.
- **Point C $(0.1, 0.2)$:** $r = \sqrt{0.1^2 + 0.2^2} = \sqrt{0.01 + 0.04} = \sqrt{0.05} \approx 0.224\,\text{m}$.
  Since $0.224 < 0.4$, **Point C is UNREACHABLE (inside inner hole)**.

---

## Solution 3.4: 2-Link Planar Analytical Inverse Kinematics
$l_1 = 1.0\,\text{m}, l_2 = 0.8\,\text{m}$, target $(x = 1.0\,\text{m}, y = 1.0\,\text{m})$.

1. **Distance squared:**
   $$r^2 = 1.0^2 + 1.0^2 = 2.0\,\text{m}^2 \implies r \approx 1.4142\,\text{m}$$
2. **Solve for $\cos\theta_2$:**
   $$\cos\theta_2 = \frac{r^2 - l_1^2 - l_2^2}{2 l_1 l_2} = \frac{2.0 - 1.0^2 - 0.8^2}{2(1.0)(0.8)} = \frac{2.0 - 1.0 - 0.64}{1.60} = \frac{0.36}{1.60} = 0.2250$$
   $$\theta_2 = \pm \arccos(0.2250) \approx \pm 76.999^\circ \approx \pm 77.0^\circ$$

3. **Solve for $\theta_1$:**
   $\text{atan2}(y, x) = \text{atan2}(1.0, 1.0) = 45.0^\circ$.
   - **Case 1: Elbow-Down ($\theta_2 = +77.0^\circ$):**
     $$\sin\theta_2 = \sin(77.0^\circ) = 0.9744$$
     $$\beta = \text{atan2}(0.8 \times 0.9744, \; 1.0 + 0.8 \times 0.2250) = \text{atan2}(0.7795, 1.1800) \approx 33.45^\circ$$
     $$\theta_1 = 45.0^\circ - 33.45^\circ = 11.55^\circ$$
     **Solution 1 (Elbow-Down):** $\theta_1 = 11.55^\circ, \theta_2 = +77.00^\circ$.

   - **Case 2: Elbow-Up ($\theta_2 = -77.0^\circ$):**
     $$\beta = \text{atan2}(-0.7795, 1.1800) \approx -33.45^\circ$$
     $$\theta_1 = 45.0^\circ - (-33.45^\circ) = 78.45^\circ$$
     **Solution 2 (Elbow-Up):** $\theta_1 = 78.45^\circ, \theta_2 = -77.00^\circ$.

---

## Solution 3.5: Spherical Wrist Center Decoupling
1. **Wrist Center Coordinates:**
   $$\mathbf{p}_{wc} = \mathbf{p}_d - d_6 \mathbf{a}_d = \begin{bmatrix} 0.80 \\ 0.30 \\ 0.60 \end{bmatrix} - 0.15 \begin{bmatrix} 0.00 \\ 0.00 \\ 1.00 \end{bmatrix} = \begin{bmatrix} 0.80 \\ 0.30 \\ 0.60 - 0.15 \end{bmatrix} = \begin{bmatrix} 0.80 \\ 0.30 \\ 0.45 \end{bmatrix}\,\text{m}$$
2. **Base Angle $\theta_1$:**
   $$\theta_1 = \text{atan2}(y_w, x_w) = \text{atan2}(0.30, 0.80) \approx 0.3588\,\text{rad} \approx 20.56^\circ$$

---

## Solution 3.6: Geometric Jacobian Calculation
$l_1 = 1.0\,\text{m}, l_2 = 1.0\,\text{m}$, $\theta_1 = 45^\circ, \theta_2 = 0^\circ \implies \theta_1 + \theta_2 = 45^\circ$.
$\cos(45^\circ) = \sin(45^\circ) = \frac{\sqrt{2}}{2} \approx 0.7071$.

1. **Evaluate Jacobian Entries:**
   $$\mathbf{J} = \begin{bmatrix} -1.0(0.7071) - 1.0(0.7071) & -1.0(0.7071) \\ 1.0(0.7071) + 1.0(0.7071) & 1.0(0.7071) \end{bmatrix} = \begin{bmatrix} -1.4142 & -0.7071 \\ +1.4142 & +0.7071 \end{bmatrix}$$
2. **Determinant:**
   $$\det(\mathbf{J}) = (-1.4142)(0.7071) - (-0.7071)(1.4142) = -1.0 + 1.0 = 0.0$$
3. **Mobility Interpretation:**
   $\det(\mathbf{J}) = 0$ indicates a **boundary kinematic singularity** (arm fully extended). The robot cannot generate velocity along the radial direction $45^\circ$ pointing outward from the base.

---

## Solution 3.7: 6-DOF Wrist Orientation Extraction
1. **Compute $\mathbf{R}_3^6$:**
   $$\mathbf{R}_0^3 = \begin{bmatrix} 0 & 0 & 1 \\ 0 & 1 & 0 \\ -1 & 0 & 0 \end{bmatrix} \implies (\mathbf{R}_0^3)^T = \begin{bmatrix} 0 & 0 & -1 \\ 0 & 1 & 0 \\ 1 & 0 & 0 \end{bmatrix}$$
   $$\mathbf{R}_3^6 = (\mathbf{R}_0^3)^T \mathbf{R}_d = \begin{bmatrix} 0 & 0 & -1 \\ 0 & 1 & 0 \\ 1 & 0 & 0 \end{bmatrix} \begin{bmatrix} 0 & -1 & 0 \\ 0 & 0 & 1 \\ -1 & 0 & 0 \end{bmatrix} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 0 & 1 \\ 0 & -1 & 0 \end{bmatrix}$$
2. **Extract Euler Z-Y-Z Angles:**
   $\mathbf{R}_{33} = \cos(\theta_5) = 0 \implies \theta_5 = 90^\circ$ ($\pi/2\,\text{rad}$).
   With $\sin(\theta_5) = 1.0$:
   - $\theta_4 = \text{atan2}(r_{23}, r_{13}) = \text{atan2}(1, 0) = 90^\circ$.
   - $\theta_6 = \text{atan2}(r_{32}, -r_{31}) = \text{atan2}(-1, 0) = -90^\circ$.
   **Answer:** $(\theta_4, \theta_5, \theta_6) = (90^\circ, 90^\circ, -90^\circ)$.

---

## Solution 3.8: Damped Least Squares at Singularity
1. **Failure of Standard Inversion:**
   $\mathbf{J} \mathbf{J}^T = \begin{bmatrix} 0 & 0 \\ 0 & 1.25 \end{bmatrix}$, which has determinant $0$. Thus $(\mathbf{J}\mathbf{J}^T)^{-1}$ does not exist.
2. **Compute Damped Least Squares:**
   With $\lambda = 0.1 \implies \lambda^2 = 0.01$:
   $$\mathbf{J} \mathbf{J}^T + \lambda^2 \mathbf{I} = \begin{bmatrix} 0.01 & 0.00 \\ 0.00 & 1.26 \end{bmatrix}$$
   Inverse:
   $$(\mathbf{J} \mathbf{J}^T + \lambda^2 \mathbf{I})^{-1} = \begin{bmatrix} 100.0 & 0.0 \\ 0.0 & 0.7937 \end{bmatrix}$$
   Compute $\mathbf{J}^T (\mathbf{J} \mathbf{J}^T + \lambda^2 \mathbf{I})^{-1}$:
   $$\mathbf{J}^T = \begin{bmatrix} 0.0 & 1.0 \\ 0.0 & 0.5 \end{bmatrix} \implies \begin{bmatrix} 0.0 & 1.0 \\ 0.0 & 0.5 \end{bmatrix} \begin{bmatrix} 100.0 & 0.0 \\ 0.0 & 0.7937 \end{bmatrix} = \begin{bmatrix} 0.0 & 0.7937 \\ 0.0 & 0.3968 \end{bmatrix}$$
3. **Multiply by error vector $\mathbf{e} = [0.2, 0.0]^T$:**
   $$\Delta \mathbf{q} = \begin{bmatrix} 0.0 & 0.7937 \\ 0.0 & 0.3968 \end{bmatrix} \begin{bmatrix} 0.2 \\ 0.0 \end{bmatrix} = \begin{bmatrix} 0.0000 \\ 0.0000 \end{bmatrix}$$
   **Conclusion:** The update is strictly finite ($[0, 0]^T$). Because the error lies along the singular direction where motion is physically impossible, DLS recognizes that moving the motors cannot reduce the error and safely commands zero speed rather than exploding to infinity!
