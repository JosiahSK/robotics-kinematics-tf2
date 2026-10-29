# Robotics & AI Student Notes: Kinematics, TF2 & Robotics Mathematics

[![Python 3.10+](https://img.shields.io/badge/python-3.10%20%7C%203.11%20%7C%203.12-blue.svg)](https://www.python.org/)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy%20(Primary)%20%7C%20Humble-green.svg)](https://docs.ros.org/en/jazzy/)
[![CI Workflow](https://github.com/JosiahSK/robotics-kinematics-tf2/actions/workflows/ci.yml/badge.svg)](https://github.com/JosiahSK/robotics-kinematics-tf2/actions)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

An open-source, mathematically rigorous, and practical textbook repository for engineering students and roboticists mastering coordinate transforms, Denavit–Hartenberg modeling, ROS 2 TF2, and 6-DOF manipulator kinematics.

---

## 1. Audience & Prerequisites

- **Target Audience:** Undergraduate and graduate students in Robotics, Mechatronics, and AI, as well as practicing engineers seeking a clear, verified reference for spatial transformations and manipulator kinematics.
- **Prerequisites:** High-school trigonometry ($\sin, \cos, \text{atan2}$), basic vector-matrix arithmetic, and Python 3.10+. No prior ROS experience is needed for the pure Python track.
- **Supported Environments:**
  - **Python Track:** Cross-platform (Windows, macOS, Linux). Requires Python 3.10+.
  - **ROS 2 Track:** **ROS 2 Jazzy Jalisco** (recommended on Ubuntu 24.04 / WSL2) or **Humble Hawksbill** (Ubuntu 22.04).

---

## 2. Start Here: Recommended Study Sequence

While the syllabus numbers Module 53 (TF2) before Module 54 (Maths), kinematics relies heavily on vector bases and rotation matrices. We strongly recommend following this pedagogical learning path:

```
[00: Getting Started] ──> [02: Mathematics for Robotics] ──> [01: ROS 2 TF2 & Frames]
                                      │
                                      ▼
[05: ROS 2 Practical & URDF] <── [03: Manipulator Kinematics] ──> [04: Python Kinematics]
         │                                                                   │
         └─────────────────────────┬─────────────────────────────────────────┘
                                   ▼
                       [06: Labs & Guided Projects]
                                   │
                                   ▼
                       [07: Quizzes & Assessments]
```

1. **[00: Getting Started](00-getting-started/README.md):** Notation conventions, right-hand rules, and mathematical prerequisites.
2. **[02: Mathematics for Robotics](02-module-54-mathematics/README.md):** $SO(3)$, $SE(3)$, homogeneous transforms, Jacobians, and Gaussian covariance.
3. **[01: ROS 2 TF2 & Kinematics](01-module-53-tf2-kinematics/README.md):** The TF tree, `map` $\to$ `odom` $\to$ `base_link`, static transforms with Transient Local QoS, and sensor frames.
4. **[03: Manipulator Kinematics](03-robot-kinematics/README.md):** DH parameters, planar 2R/3R arms, EduArm-6DOF spherical wrist decoupling, and singular configurations.
5. **[04: Python Implementations](04-python-robotics/README.md):** 39 passing unit tests, verified FK/IK solvers, SVD DLS pseudoinverse, and 3D Matplotlib visualizer.
6. **[05: ROS 2 Practical](05-ros2-practical/README.md):** URDF robot description package (`eduarm_description`), RViz joint sliders, and DH-to-URDF coordinate mapping.
7. **[06: Labs & Projects](06-labs-and-projects/README.md):** Six guided engineering laboratory assignments from TF2 setup to singularity handling.
8. **[07: Assessments](07-assessments/README.md):** Quizzes for each major module with a comprehensive [Master Answer Key](07-assessments/answer-key.md).
9. **[Reference Centre](docs/reference/README.md):** Fast lookups for formulas, DH parameters, TF2 CLI commands, glossary, and troubleshooting tips.

---

## 3. Directory Navigation

| Track | Module Directory | Focus & Contents |
| :--- | :--- | :--- |
| **Foundations** | [`00-getting-started/`](00-getting-started/README.md) | Conventions, Craig notation, math prerequisites, and course setup |
| **Theory** | [`02-module-54-mathematics/`](02-module-54-mathematics/README.md) | Linear algebra, calculus, $SO(3)/SE(3)$, Jacobians, covariance |
| **Theory** | [`01-module-53-tf2-kinematics/`](01-module-53-tf2-kinematics/README.md) | ROS 2 TF2 tree, frame broadcast, buffer lookup, sensor frames |
| **Theory** | [`03-robot-kinematics/`](03-robot-kinematics/README.md) | DH convention, forward/inverse kinematics, 6-DOF arms, velocity |
| **Code** | [`04-python-robotics/`](04-python-robotics/README.md) | Python reference algorithms, 39 pytest unit tests, 3D visualizer |
| **Practical** | [`05-ros2-practical/`](05-ros2-practical/README.md) | `eduarm_description` ROS 2 package, URDF specification, RViz |
| **Hands-on** | [`06-labs-and-projects/`](06-labs-and-projects/README.md) | 6 guided student lab sheets with starter tasks and verification |
| **Revision** | [`07-assessments/`](07-assessments/README.md) | Student quizzes (Modules 53, 54, Kinematics) and master answer key |
| **Reference** | [`docs/reference/`](docs/reference/README.md) | Formula sheets, DH cheat sheet, TF2 cheat sheet, glossary, FAQ |
| **Visuals** | [`docs/visual-index.md`](docs/visual-index.md) | Index of all 12 SVG vector diagrams and Matplotlib plots |

---

## 4. Visual Gallery of Core Concepts

Explore the complete catalog in the [Visual Diagram & Plot Index](docs/visual-index.md).

| 6-DOF Robot Architecture | Kinematic Wrist Decoupling |
| :---: | :---: |
| [![EduArm-6DOF Schematic](assets/diagrams/08-eduarm-6dof-schematic.svg)](docs/visual-index.md#1-vector-svg-diagrams) | [![Wrist Decoupling](assets/diagrams/09-wrist-center-decoupling.svg)](docs/visual-index.md#1-vector-svg-diagrams) |
| *EduArm-6DOF kinematic chain & DH frames* | *Pieper wrist center isolation ($\mathbf{p}_{wc} = \mathbf{p} - d_6 \mathbf{R} \hat{\mathbf{z}}$)* |
| **TF2 Frame Hierarchy (REP 105)** | **Planar 2R Workspace & Dual Branches** |
| [![TF Hierarchy](assets/diagrams/02-map-odom-base-link.svg)](docs/visual-index.md#1-vector-svg-diagrams) | [![Planar 2R Workspace](assets/generated/07-planar-2r-workspace-ik.png)](docs/visual-index.md#2-programmatic-matplotlib-plots) |
| *Standard ROS 2 coordinate tree and QoS durability* | *Reachable workspace and elbow-up / elbow-down solutions* |

---

## 5. Quickstart: Python Track (No ROS Required)

Run kinematics algorithms, 3D interactive visualizations, and automated unit tests on **Windows, macOS, or Linux**:

```bash
# 1. Clone repository & enter workspace
git clone https://github.com/JosiahSK/robotics-kinematics-tf2.git
cd robotics-kinematics-tf2

# 2. Set up virtual environment
python -m venv venv
# Linux/macOS: source venv/bin/activate
# Windows PowerShell: .\venv\Scripts\Activate.ps1

# 3. Install dependencies
pip install -r requirements.txt

# 4. Run automated test suite (39 tests)
pytest 04-python-robotics/tests/ -v

# 5. Launch interactive 3D robot arm visualizer
python 04-python-robotics/visualise_robot_arm.py --angles 25 35 -30 15 -40 20
```

---

## 6. Quickstart: ROS 2 Track (Jazzy / Humble)

To build and inspect the `eduarm_description` URDF package in RViz with joint slider controls:

```bash
# 1. Source ROS 2 environment (Ubuntu 24.04 Jazzy or 22.04 Humble)
source /opt/ros/jazzy/setup.bash

# 2. Build the package in your colcon workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/JosiahSK/robotics-kinematics-tf2.git
cd ~/ros2_ws
colcon build --packages-select eduarm_description --symlink-install
source install/setup.bash

# 3. Launch RViz with interactive joint state sliders
ros2 launch eduarm_description view_arm.launch.py
```

---

## 7. Model Transparency & Scope

1. **EduArm-6DOF Academic Model:** An anthropomorphic 6-DOF manipulator ($d_1=0.4\,\text{m}, a_2=0.5\,\text{m}, d_4=0.45\,\text{m}, d_6=0.1\,\text{m}$) crafted for closed-form analytical solvability and geometric transparency.
2. **Kinematics Focus:** Covers forward kinematics, inverse kinematics (analytical Pieper and numerical Levenberg-Marquardt DLS), and velocity Jacobians. Dynamics (inertia matrices, Coriolis forces, joint torques) are reserved for advanced courses.
3. **Environment Support:** Python reference code and tests run cross-platform on Windows, macOS, and Linux. ROS 2 practical packages target **Ubuntu 24.04 (Jazzy Jalisco)** and **Ubuntu 22.04 (Humble Hawksbill)**.

---

## 8. License

This repository is distributed under the open-source [MIT License](LICENSE).
