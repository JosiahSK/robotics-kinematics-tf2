# Student Progress & Mastery Checklist

Use this interactive checklist to track your learning progress across the theoretical modules, Python implementations, ROS 2 practical labs, and assessments in this repository.

---

## Stage 1: Mathematical Foundations (Module 54)

- [ ] **Vectors & Coordinate Frames**
  - [ ] Understand vector representations in $2\text{D}$ and $3\text{D}$ space
  - [ ] Compute dot product (projection) and cross product (orthogonality)
  - [ ] Apply the Right-Hand Rule for coordinate frame axes $(X = \text{Red}, Y = \text{Green}, Z = \text{Blue})$
- [ ] **Rotations & Orientations**
  - [ ] Derive 2D and 3D elementary rotation matrices: $\mathbf{R}_x(\theta)$, $\mathbf{R}_y(\theta)$, $\mathbf{R}_z(\theta)$
  - [ ] Verify properties of the Special Orthogonal group $SO(3)$: $\mathbf{R}^T \mathbf{R} = \mathbf{I}$, $\det(\mathbf{R}) = +1$
  - [ ] Understand Euler angle singularities (Gimbal Lock) and why pitch at $\pm 90^\circ$ causes rank loss
  - [ ] Understand unit quaternions $\mathbf{q} = [w, x, y, z]$ and Hamilton multiplication
- [ ] **Homogeneous Transformations**
  - [ ] Construct $4 \times 4$ homogeneous transformation matrices $\mathbf{T} \in SE(3)$
  - [ ] Perform spatial coordinate transforms on points and vectors
  - [ ] Compute the analytical inverse of a transformation matrix: $\mathbf{T}^{-1} = \begin{bmatrix} \mathbf{R}^T & -\mathbf{R}^T \mathbf{p} \\ \mathbf{0}^T & 1 \end{bmatrix}$
- [ ] **Calculus, Jacobians & Velocities**
  - [ ] Formulate the manipulator geometric Jacobian matrix $\mathbf{J}(\mathbf{q}) = \begin{bmatrix} \mathbf{J}_v \\ \mathbf{J}_\omega \end{bmatrix}$
  - [ ] Identify kinematic singularities via $\det(\mathbf{J}) = 0$ or rank deficiency
  - [ ] Compute Yoshikawa manipulability measure $w = \sqrt{\det(\mathbf{J} \mathbf{J}^T)}$
- [ ] **Probability & Covariance**
  - [ ] Formulate multi-variable Gaussian distributions $\mathcal{N}(\boldsymbol{\mu}, \boldsymbol{\Sigma})$
  - [ ] Interpret covariance ellipses and principle uncertainty axes via eigenvalues/eigenvectors
  - [ ] Apply linear uncertainty propagation: $\boldsymbol{\Sigma}_y = \mathbf{J} \boldsymbol{\Sigma}_x \mathbf{J}^T$

---

## Stage 2: ROS 2 TF2 & Coordinate Frames (Module 53)

- [ ] **TF2 Coordinate Conventions (REP 105)**
  - [ ] Understand the role of `earth` $\to$ `map` $\to$ `odom` $\to$ `base_link` $\to$ `base_footprint`
  - [ ] Distinguish global drift-free correction (`map`) from smooth local odometry (`odom`)
  - [ ] Enforce the Single-Parent Rule in the directed tree structure of TF2
- [ ] **Static vs. Dynamic Transforms**
  - [ ] Understand why dynamic transforms stream on `/tf` with Volatile QoS
  - [ ] Understand why static transforms publish on `/tf_static` with **Transient Local durability QoS**
  - [ ] Explain the difference between ROS 1 latching and ROS 2 DDS Transient Local durability
- [ ] **TF2 CLI Tools**
  - [ ] Inspect live transforms using `ros2 run tf2_ros tf2_echo <parent> <child>`
  - [ ] Generate visual frame graphs using `ros2 run tf2_tools view_frames`
  - [ ] Check transform latency and buffer frequency with `ros2 run tf2_ros tf2_monitor`
- [ ] **ROS 2 Programming**
  - [ ] Implement a dynamic transform broadcaster node using `TransformBroadcaster`
  - [ ] Implement a static transform publisher using `StaticTransformBroadcaster`
  - [ ] Implement a transform listener node with temporal buffer and lookup timeouts
  - [ ] Safely handle `LookupException`, `ConnectivityException`, and `ExtrapolationException`

---

## Stage 3: Manipulator Kinematics & 6-DOF Arms

- [ ] **Planar Manipulators**
  - [ ] Derive forward kinematics for a 2-link planar arm
  - [ ] Derive analytical inverse kinematics using the Law of Cosines
  - [ ] Implement both **elbow-down** ($\theta_2 \ge 0$) and **elbow-up** ($\theta_2 < 0$) branches
- [ ] **Denavit–Hartenberg (DH) Convention**
  - [ ] Apply the 4 DH parameters: link length $a_i$, link twist $\alpha_i$, link offset $d_i$, joint angle $\theta_i$
  - [ ] Assign coordinate frames according to standard DH rules (common normal along $X_i$, joint axis along $Z_i$)
  - [ ] Formulate standard $A_i$ transformation matrix and chained forward kinematics
- [ ] **EduArm-6DOF Anthropomorphic Arm**
  - [ ] Construct the 6-joint DH parameter table for the reference EduArm-6DOF
  - [ ] Verify forward kinematics against known configurations (all-zeros, standard test poses)
  - [ ] Map DH frame conventions to URDF joint origins and axis definitions
- [ ] **Inverse Kinematics Solvers**
  - [ ] Implement analytical wrist decoupling (Pieper's criterion) for spherical wrist arms
  - [ ] Compute wrist center $\mathbf{p}_{wc} = \mathbf{p}_{\text{tcp}} - d_6 \mathbf{R} \hat{\mathbf{z}}$
  - [ ] Implement numerical Damped Least-Squares (DLS) IK using SVD to avoid singularity explosions
  - [ ] Implement exact $SO(3)$ logarithmic map for 3D orientation error with $180^\circ$ symmetric fallback

---

## Stage 4: Python Engineering & Quality Verification

- [ ] **Environment Setup**
  - [ ] Install Python 3.10+ and dependencies (`pip install -r requirements.txt`)
  - [ ] Verify test suite execution: `pytest 04-python-robotics/tests/ -v`
  - [ ] Confirm all **39 test cases pass** with zero warnings or failures
- [ ] **Code Architecture & Modules**
  - [ ] Review `dh_transform.py`: DH parameter matrices and SE(3) pose extraction
  - [ ] Review `forward_kinematics_6dof.py`: `RobotArm6DOF` class and link poses
  - [ ] Review `inverse_kinematics_6dof.py`: `NumericalIKSolver6DOF` and analytical Pieper solver
  - [ ] Review `jacobian.py`: Geometric Jacobian and SVD-based DLS pseudoinverse
  - [ ] Review `test_urdf_consistency.py`: Multi-configuration DH vs URDF kinematic consistency checks

---

## Stage 5: Practical Labs & Assessments

- [ ] **Lab Exercises (`06-labs-and-projects/`)**
  - [ ] Lab 01: TF2 Static and Dynamic Broadcasters
  - [ ] Lab 02: 2D Robot Arm Forward Kinematics and Workspace Simulation
  - [ ] Lab 03: 2D Inverse Kinematics and Trajectory Tracking
  - [ ] Lab 04: Denavit–Hartenberg Modeling of a 6-DOF Manipulator
  - [ ] Lab 05: Numerical Inverse Kinematics and Singularity Avoidance
  - [ ] Lab 06: Full-Stack ROS 2 Robot Arm Control and RViz Visualization
- [ ] **Quizzes & Self-Assessment (`07-assessments/`)**
  - [ ] Quiz 01: ROS 2 TF2 & Coordinate Frames ([quiz-module-53.md](../../07-assessments/quiz-module-53.md))
  - [ ] Quiz 02: Mathematics for Robotics ([quiz-module-54.md](../../07-assessments/quiz-module-54.md))
  - [ ] Quiz 03: Robot Kinematics ([quiz-kinematics.md](../../07-assessments/quiz-kinematics.md))
  - [ ] Verify your answers against the [Assessments Answer Key](../../07-assessments/answer-key.md)
