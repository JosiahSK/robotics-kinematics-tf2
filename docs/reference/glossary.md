# Robotics Glossary

This glossary provides concise engineering definitions for key concepts in robotics mathematics, spatial transformations, manipulator kinematics, and ROS 2 coordinate tracking. Each term links to its in-depth derivation and implementation chapter.

---

### A
- **Analytical Inverse Kinematics:** Closed-form mathematical formulas that solve for joint angles directly from end-effector pose without iteration. Fast ($< 1\,\mu\text{s}$) and guarantees all branch solutions. See [07: Inverse Kinematics](../../03-robot-kinematics/07-inverse-kinematics.md).
- **Approach Vector ($\mathbf{a}_d$):** The unit vector specifying the pointing direction of the robot's tool flange, corresponding to the third column ($Z$-axis) of the end-effector rotation matrix in DH convention: $\mathbf{a}_d = \mathbf{R}[:, 2]$. See [08: 6-DOF Industrial Arm](../../03-robot-kinematics/08-six-dof-arm.md).
- **`atan2(y, x)`:** Four-quadrant inverse tangent returning the counter-clockwise angle in $[-\pi, +\pi]$ radians, correctly disambiguating all four Cartesian quadrants based on the individual signs of $y$ and $x$. See [00: Mathematical Prerequisites](../../00-getting-started/mathematical-prerequisites.md).

---

### C
- **Common Normal:** The unique line segment perpendicular to two non-parallel, non-intersecting joint axes in 3D space. Its directed length defines link length $a_i$. See [05: Denavit–Hartenberg](../../03-robot-kinematics/05-denavit-hartenberg.md).
- **Covariance Matrix ($\boldsymbol{\Sigma}$):** A symmetric positive semi-definite matrix whose diagonal elements represent variance (uncertainty) along individual axes and whose off-diagonals represent cross-correlations. See [09: Covariance Matrices](../../02-module-54-mathematics/09-covariance-matrices.md).

---

### D
- **Damped Least Squares (DLS):** A Levenberg–Marquardt numerical inversion technique $\mathbf{J}^* = \mathbf{J}^T (\mathbf{J}\mathbf{J}^T + \lambda^2 \mathbf{I})^{-1}$ that bounds joint velocities near kinematic singularities by trading tracking accuracy for inversion stability. See [09: Jacobian Kinematics](../../03-robot-kinematics/09-jacobian-kinematics.md).
- **Degrees of Freedom (DOF):** The minimum number of independent coordinates required to uniquely specify the configuration of a mechanical system in space. A rigid body in 3D space has 6 DOF (3 position, 3 orientation). See [00: Robotics Fundamentals](../../00-getting-started/robotics-fundamentals.md).
- **Denavit–Hartenberg (DH) Convention:** A systematic 4-parameter convention $(\theta_i, d_i, a_i, \alpha_i)$ for attaching coordinate frames to adjacent links of a serial kinematic chain. See [05: Denavit–Hartenberg](../../03-robot-kinematics/05-denavit-hartenberg.md).

---

### F
- **Forward Kinematics (FK):** The mathematical mapping from known joint coordinates $\mathbf{q} \in \mathbb{R}^n$ to the Cartesian pose (position and orientation) of the end-effector $\mathbf{T}_0^n(\mathbf{q}) \in SE(3)$. See [06: Forward Kinematics](../../03-robot-kinematics/06-forward-kinematics.md).

---

### G
- **Geometric Jacobian ($\mathbf{J}$):** A $6 \times n$ matrix mapping joint velocity vectors $\mathbf{\dot{q}}$ to the Cartesian linear and angular velocities of the end-effector: $\mathbf{v} = \mathbf{J}(\mathbf{q})\mathbf{\dot{q}}$. See [09: Jacobian Kinematics](../../03-robot-kinematics/09-jacobian-kinematics.md).
- **Grübler's Mobility Criterion:** A topological formula $M = 6(N - 1 - J) + \sum f_i$ calculating the net degrees of freedom of a spatial kinematic mechanism. See [00: Robotics Fundamentals](../../00-getting-started/robotics-fundamentals.md).

---

### H
- **Homogeneous Transformation Matrix ($\mathbf{T}$):** A $4 \times 4$ matrix combining a $3 \times 3$ rotation matrix $\mathbf{R} \in SO(3)$ and a $3 \times 1$ translation vector $\mathbf{p} \in \mathbb{R}^3$, forming an element of the Special Euclidean group $SE(3)$. See [04: Homogeneous Transformations](../../02-module-54-mathematics/04-homogeneous-transformations.md).

---

### I
- **Inverse Kinematics (IK):** The nonlinear problem of computing the joint angles $\mathbf{q}$ needed to position and orient the robot's end-effector at a desired Cartesian pose $\mathbf{T}_{\text{target}}$. See [07: Inverse Kinematics](../../03-robot-kinematics/07-inverse-kinematics.md).

---

### K
- **Kinematic Singularity:** A joint configuration $\mathbf{q}$ where the Jacobian matrix $\mathbf{J}(\mathbf{q})$ loses rank ($\det(\mathbf{J}\mathbf{J}^T) = 0$). At a singularity, the manipulator loses the ability to move in one or more Cartesian directions and requires infinite joint velocity to track arbitrary trajectories. See [10: Singularities & Joint Limits](../../03-robot-kinematics/10-singularities-and-joint-limits.md).

---

### L
- **Logarithmic Map ($\text{Log}: SO(3) \to \mathfrak{so}(3)$):** The Lie-algebra mapping from a 3D rotation matrix $\mathbf{R}$ to its corresponding axis-angle vector $\mathbf{e}_o = \theta \mathbf{k}$, providing exact orientation error vectors without Euler-angle gimbal lock. See [08: 6-DOF Industrial Arm](../../03-robot-kinematics/08-six-dof-arm.md).

---

### M
- **Manipulability Measure ($w$):** Yoshikawa's scalar metric $w = \sqrt{\det(\mathbf{J}\mathbf{J}^T)} = \prod \sigma_i$ quantifying how far an arm is from a kinematic singularity and how uniformly it can produce Cartesian velocities. See [09: Jacobian Kinematics](../../03-robot-kinematics/09-jacobian-kinematics.md).

---

### P
- **Pieper's Criterion:** The geometric condition that an analytical closed-form inverse kinematics solution exists for a 6-DOF serial arm if three consecutive joint axes intersect at a single point (spherical wrist) or are parallel to each other. See [08: 6-DOF Industrial Arm](../../03-robot-kinematics/08-six-dof-arm.md).

---

### Q
- **Quaternion (Unit Hamilton):** A 4-tuple $\mathbf{q} = [x, y, z, w]$ satisfying $x^2 + y^2 + z^2 + w^2 = 1$ that parameterizes a 3D rotation without gimbal lock or trigonometric singularities. In ROS 2, scalar $w$ is the fourth component. See [04: Robot Pose](../../01-module-53-tf2-kinematics/04-robot-pose.md).

---

### R
- **REP 103:** ROS Enhancement Proposal defining standard coordinate frame conventions: right-handed, $X$ forward (red), $Y$ left (green), $Z$ up (blue). See [01: Coordinate Frames](../../01-module-53-tf2-kinematics/01-coordinate-frames.md).
- **REP 105:** ROS Enhancement Proposal defining the mobile robot spatial coordinate hierarchy: `earth -> map -> odom -> base_link`. See [03: Map, Odom & Base Link](../../01-module-53-tf2-kinematics/03-map-odom-base-link.md).
- **Rodrigues' Rotation Formula:** Analytical formula $\mathbf{R}(\mathbf{k}, \theta) = \mathbf{I} + \sin\theta [\mathbf{k}]_\times + (1 - \cos\theta)[\mathbf{k}]_\times^2$ computing the $3 \times 3$ rotation matrix for a rotation by angle $\theta$ around unit axis $\mathbf{k}$. See [03: Rotation Representations](../../03-robot-kinematics/03-rotation-matrices.md).

---

### S
- **Special Orthogonal Group ($SO(3)$):** The mathematical Lie group of all $3 \times 3$ proper rotation matrices: $\mathbf{R}^T \mathbf{R} = \mathbf{I}$ and $\det(\mathbf{R}) = +1$. See [03: Rotation Representations](../../03-robot-kinematics/03-rotation-matrices.md).
- **Special Euclidean Group ($SE(3)$):** The Lie group of all $4 \times 4$ rigid-body spatial transformations combining 3D rotation and translation. See [04: Homogeneous Transformations](../../02-module-54-mathematics/04-homogeneous-transformations.md).
- **Spherical Wrist:** A 3-DOF wrist architecture where the axes of joints 4, 5, and 6 intersect at a single point (the wrist center), enabling kinematic decoupling. See [08: 6-DOF Industrial Arm](../../03-robot-kinematics/08-six-dof-arm.md).

---

### T
- **TF2 (Transform Library 2nd Generation):** The ROS 2 spatial transform library that maintains a distributed, time-buffered tree of coordinate frames and answers spatial queries at past, present, and extrapolated timestamps. See [01: Module 53 README](../../01-module-53-tf2-kinematics/README.md).
- **Tool Center Point (TCP):** The exact Cartesian point on an end-effector (gripper tip, welding nozzle, drill point) that performs the robot's physical work. See [05: URDF Robot Arm](../../05-ros2-practical/urdf-robot-arm.md).
- **Transient Local Durability QoS:** A ROS 2 Quality of Service policy where the DDS publisher stores the most recently published message and delivers it automatically to late-joining subscribers (replacing ROS 1's "latched" topic mechanism). Used on `/tf_static`. See [02: Static & Dynamic Transforms](../../01-module-53-tf2-kinematics/02-static-and-dynamic-transforms.md).

---

### W
- **Wrist Center ($\mathbf{p}_{wc}$):** The point of intersection of joint axes 4, 5, and 6 in an anthropomorphic robot arm. Calculated from tool pose as $\mathbf{p}_{wc} = \mathbf{p}_d - d_6 \mathbf{a}_d$. See [08: 6-DOF Industrial Arm](../../03-robot-kinematics/08-six-dof-arm.md).
