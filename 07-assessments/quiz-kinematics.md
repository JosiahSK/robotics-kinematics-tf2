# Assessment: Robot Kinematics — Serial & 6-DOF Manipulators

**Course:** Robotics & AI Engineering Notes  
**Topic:** Robot Kinematics, Denavit–Hartenberg, Analytical/Numerical IK & Singularities  
**Total Marks:** 50 Points  
**Format:** Closed-book, scientific calculator permitted.

---

## Section A: Multiple Choice Questions (10 Points, 2 Points Each)

1. In the standard Denavit–Hartenberg convention, what does the link twist parameter $\alpha_i$ represent?
   - A) Rotation about $z_{i-1}$ from $x_{i-1}$ to $x_i$
   - B) Translation along $z_{i-1}$
   - C) Rotation about $x_i$ from $z_{i-1}$ to $z_i$
   - D) Distance along $x_i$ between joint origins

2. What geometric condition allows a 6-DOF serial robot arm to be solved in closed-form analytical inverse kinematics via Pieper's Criterion?
   - A) All links must have identical lengths
   - B) Three consecutive joint axes must intersect at a single point (spherical wrist) or be parallel
   - C) The Jacobian must have determinant $+1.0$ everywhere
   - D) Every joint must have range $[-\pi, +\pi]$

3. What geometric configuration causes a **Wrist Singularity** in a standard anthropomorphic 6-DOF robot arm?
   - A) The arm is stretched out to maximum reach
   - B) Joint 5 pitch angle is zero ($\theta_5 = 0$), causing axes 4 and 6 to become collinear
   - C) The wrist center lies on the Joint 1 vertical axis
   - D) The end-effector touches the factory floor

4. Why is the Damped Least Squares (DLS) method used instead of the standard Moore-Penrose pseudoinverse in numerical inverse kinematics?
   - A) It computes faster on GPUs
   - B) It prevents joint velocities from exploding to infinity near kinematic singularities
   - C) It eliminates the need for forward kinematics
   - D) It guarantees exact zero Cartesian tracking error everywhere

5. In modern robotics numerical IK, how is the 3D orientation error $\mathbf{e}_o$ between current rotation $\mathbf{R}$ and desired rotation $\mathbf{R}_d$ rigorously computed?
   - A) Subtracting Euler angles: $\mathbf{\phi}_d - \mathbf{\phi}$
   - B) Cross product sum: $\mathbf{e}_o = \frac{1}{2}(\mathbf{n} \times \mathbf{n}_d + \mathbf{s} \times \mathbf{s}_d + \mathbf{a} \times \mathbf{a}_d)$
   - C) Multiplying determinants: $\det(\mathbf{R}_d) - \det(\mathbf{R})$
   - D) Subtracting quaternions: $q_d - q$

---

## Section B: Analytical Kinematics & DH Problems (20 Points)

### Question 6 (10 Points): DH Parameter Table Construction
Consider an illustrative 3-DOF spatial R-R-R articulated arm:
- Joint 1: Base waist rotation about vertical axis $Z_0$. Joint 1 origin is at ground level.
- Link 1: Extends vertically by height $d_1 = 0.40\,\text{m}$ to Joint 2 (shoulder).
- Joint 2: Shoulder pitch rotation about axis $Z_1$ perpendicular to $Z_0$.
- Link 2: Upper arm extending horizontally with length $a_2 = 0.50\,\text{m}$.
- Joint 3: Elbow pitch rotation about axis $Z_2$ parallel to $Z_1$.
- Link 3: Forearm extending horizontally with length $a_3 = 0.40\,\text{m}$.
Construct the complete $3 \times 4$ Standard Denavit–Hartenberg table ($\theta_i, d_i, a_i, \alpha_i$) for this robot arm.

### Question 7 (10 Points): Spherical Wrist Decoupling Derivation
An industrial arm with a spherical wrist has tool length $d_6 = 0.12\,\text{m}$ along $Z_6$.
The target end-effector pose is specified as:
$$\mathbf{p}_d = \begin{bmatrix} 0.70 \\ 0.40 \\ 0.50 \end{bmatrix}\,\text{m}, \quad \mathbf{R}_d = \begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix}$$
1. Extract the approach vector $\mathbf{a}_d$ from $\mathbf{R}_d$.
2. Derive and compute the numerical $(x, y, z)$ position of the wrist center $\mathbf{p}_{wc}$.
3. Calculate the required base waist angle $\theta_1$ in radians and degrees.

---

## Section C: Velocity Kinematics & Singularities (20 Points)

### Question 8 (10 Points): Planar Arm Singularity Analysis
A 2-link planar arm with link lengths $l_1 = 1.0\,\text{m}$ and $l_2 = 1.0\,\text{m}$ has the Jacobian matrix:
$$\mathbf{J}(\theta_1, \theta_2) = \begin{bmatrix}
-l_1 \sin\theta_1 - l_2 \sin(\theta_1+\theta_2) & -l_2 \sin(\theta_1+\theta_2) \\
l_1 \cos\theta_1 + l_2 \cos(\theta_1+\theta_2) & l_2 \cos(\theta_1+\theta_2)
\end{bmatrix}$$
1. Derive the symbolic expression for $\det(\mathbf{J})$.
2. Identify all values of $\theta_2$ where the robot is in a kinematic singularity.
3. At $\theta_2 = 90^\circ$, compute the numerical value of Yoshikawa's Manipulability Measure $w$.

### Question 9 (10 Points): Numerical IK Step Calculation
A robot end-effector has current position $\mathbf{p} = [0.50, 0.50]^T\,\text{m}$ and target position $\mathbf{p}_d = [0.52, 0.49]^T\,\text{m}$.
The current Jacobian matrix is:
$$\mathbf{J} = \begin{bmatrix} 1.0 & 0.0 \\ 0.0 & 2.0 \end{bmatrix}$$
1. Compute the Cartesian error vector $\mathbf{e} = \mathbf{p}_d - \mathbf{p}$.
2. Invert $\mathbf{J}$ analytically.
3. Compute the commanded joint update $\Delta \mathbf{q} = \mathbf{J}^{-1} \mathbf{e}$.
4. If the initial joint angles are $\mathbf{q}_0 = [0.50, 0.30]^T\,\text{rad}$, compute the updated angles $\mathbf{q}_1$.
