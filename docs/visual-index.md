# Visual Index: Robotics Diagrams & Scientific Figures

This document provides a catalog of all 12 educational diagrams and generated plots included in the repository. Each entry details the core concept explained, the corresponding theory chapter, descriptive alt-text, the image source file, and whether the asset is a hand-crafted vector SVG or generated programmatically from mathematical equations.

---

## Catalog of Visual Assets

| # | Figure Title | Concept Explained | Corresponding Chapter | Format & Generation |
| :---: | :--- | :--- | :--- | :--- |
| **01** | **[REP 103 Coordinate Frames](../assets/diagrams/01-coordinate-frames-rep103.svg)** | Right-handed Cartesian convention: $X$ forward (red), $Y$ left (green), $Z$ up (blue); vehicle chassis orientation and full tree. | [01: Coordinate Frames](../01-module-53-tf2-kinematics/01-coordinate-frames.md) | Vector SVG (Hand-crafted) |
| **02** | **[The Map-Odom-Base Hierarchy](../assets/diagrams/02-map-odom-base-link.svg)** | REP 105 spatial hierarchy: continuous high-frequency wheel odometry (`odom -> base_link`) decoupled from discrete SLAM corrections (`map -> odom`). | [03: Map, Odom & Base Link](../01-module-53-tf2-kinematics/03-map-odom-base-link.md) | Vector SVG (Hand-crafted) |
| **03** | **[Static vs. Dynamic Transforms](../assets/diagrams/03-fixed-vs-dynamic-transforms.svg)** | `/tf_static` (Transient Local durability QoS, broadcast once) vs `/tf` (Volatile durability, 50–100 Hz streams). | [02: Static & Dynamic Transforms](../01-module-53-tf2-kinematics/02-static-and-dynamic-transforms.md) | Vector SVG (Hand-crafted) |
| **04** | **[Camera Body vs. Optical Frame](../assets/diagrams/04-camera-body-vs-optical.svg)** | Standard robot body frame ($X$ forward, $Z$ up) vs computer vision optical frame ($Z$ depth forward, $X$ right, $Y$ down). | [06: Sensor Data Transformations](../01-module-53-tf2-kinematics/06-sensor-transformations.md) | Vector SVG (Hand-crafted) |
| **05** | **[Denavit–Hartenberg Parameters](../assets/diagrams/05-dh-parameters-definition.svg)** | Geometric definition of the 4 standard DH parameters: joint angle $\theta_i$, link offset $d_i$, link length $a_i$, and link twist $\alpha_i$ along common normals. | [05: Denavit–Hartenberg](../03-robot-kinematics/05-denavit-hartenberg.md) | Vector SVG (Hand-crafted) |
| **06** | **[Planar 2R Forward Kinematics](../assets/generated/06-planar-2r-forward-kinematics.png)** | Two-link planar arm with link lengths $l_1=1.0\,\text{m}, l_2=0.5\,\text{m}$ at $\theta_1=45^\circ, \theta_2=45^\circ$, showing exact end-effector coordinates $(0.707, 1.207)\,\text{m}$. | [06: Forward Kinematics](../03-robot-kinematics/06-forward-kinematics.md) | High-Res PNG (Generated from Matplotlib equations) |
| **07** | **[Planar 2R Workspace & Dual Branches](../assets/generated/07-planar-2r-workspace-ik.png)** | Outer reachable boundary ($r_{\max}=l_1+l_2$), inner void ($r_{\min}=\|l_1-l_2\|$), and visual comparison of elbow-up vs elbow-down configurations. | [07: Inverse Kinematics](../03-robot-kinematics/07-inverse-kinematics.md) | High-Res PNG (Generated from Matplotlib equations) |
| **08** | **[EduArm-6DOF Kinematic Schematic](../assets/diagrams/08-eduarm-6dof-schematic.svg)** | 6-DOF anthropomorphic serial manipulator: base pedestal, waist column ($d_1=0.400\,\text{m}$), upper arm ($a_2=0.500\,\text{m}$), forearm ($d_4=0.450\,\text{m}$), and spherical wrist ($d_6=0.100\,\text{m}$). | [08: 6-DOF Industrial Arm](../03-robot-kinematics/08-six-dof-arm.md) | Vector SVG (Hand-crafted) |
| **09** | **[Spherical Wrist Decoupling (Pieper)](../assets/diagrams/09-wrist-center-decoupling.svg)** | Flowchart of Pieper's kinematic decoupling: isolating wrist center $\mathbf{p}_{wc} = \mathbf{p}_d - d_6 \mathbf{a}_d$ (Joints 1–3) from wrist orientation $\mathbf{R}_3^6$ (Joints 4–6). | [08: 6-DOF Industrial Arm](../03-robot-kinematics/08-six-dof-arm.md) | Vector SVG (Hand-crafted) |
| **10** | **[Manipulability Velocity Ellipsoid](../assets/generated/10-jacobian-manipulability-ellipsoid.png)** | Cartesian velocity transmission ellipsoid $\dot{\mathbf{x}}^T (\mathbf{J}\mathbf{J}^T)^{-1} \dot{\mathbf{x}} \le 1$, principal singular value axes, and Yoshikawa index comparison near singularity. | [09: Jacobian Kinematics](../03-robot-kinematics/09-jacobian-kinematics.md) | High-Res PNG (Generated from Matplotlib SVD) |
| **11** | **[Gaussian Covariance Error Ellipses](../assets/generated/11-gaussian-covariance-ellipse.png)** | 2D Gaussian probability density contour, eigenvector orientation, and $1\sigma$ ($39.3\%$), $2\sigma$ ($86.5\%$), $3\sigma$ ($98.9\%$) confidence ellipses. | [09: Covariance Matrices](../02-module-54-mathematics/09-covariance-matrices.md) | High-Res PNG (Generated from Matplotlib equations) |
| **12** | **[ROS 2 TF Computational Dataflow](../assets/diagrams/12-ros2-tf-dataflow.svg)** | Complete runtime computational graph from URDF XML and `/joint_states` through `robot_state_publisher` to TF2 buffers and RViz displays. | [02: Robot State Publisher](../05-ros2-practical/robot-state-publisher.md) | Vector SVG (Hand-crafted) |

---

## Visual Standards & Conventions

All repository diagrams strictly adhere to these robotics presentation rules:
1. **Axis Color Convention (RGB = XYZ):**
   - **X-axis:** Red (`#e74c3c` / `#c0392b`)
   - **Y-axis:** Green (`#27ae60` / `#2ecc71`)
   - **Z-axis:** Blue (`#2980b9` / `#3498db`)
2. **Canonical Model Consistency:** All robot arm illustrations display dimensions matching the verified DH model ($d_1=0.400\,\text{m}, a_2=0.500\,\text{m}, d_4=0.450\,\text{m}, d_6=0.100\,\text{m}$).
3. **Typography:** Diagrams use clean, accessible sans-serif typography (`system-ui`, `Arial`, `Helvetica`) with minimum 11pt font size for high readability on desktop and mobile screens.
4. **Reproducibility:** Plots in `assets/generated/` are rendered programmatically via NumPy and Matplotlib scripts in `scripts/generate_diagrams.py`.
