# 06: Practical Labs & Engineering Projects

This module contains **6 guided laboratory assignments** bridging robotics mathematics, Denavit–Hartenberg modeling, numerical Jacobians, Python engineering, and ROS 2 TF2 simulation.

Every lab follows a structured engineering workflow:
1. **Prerequisites & Time Estimates:** Required conceptual background and expected completion duration (typically 45–75 minutes).
2. **Mathematical Algorithm:** Core equations and geometric derivations.
3. **Task Sequence & Run Commands:** Concrete terminal commands and Python scripts.
4. **Expected Output & Verification Checkpoints:** Exact console outputs and numerical tolerances for self-checking.
5. **Extension Tasks:** Advanced challenges (uncertainty propagation, trajectory generation, condition number tracking).
6. **Deliverables:** Structured submission artifacts for coursework grading or personal portfolio records.

---

## Laboratory Index

| Lab | Title | Key Robotics Concepts | Starter / Reference Code | Primary Deliverable |
| :--- | :--- | :--- | :--- | :--- |
| [**Lab 01**](lab-01-tf2-visualization.md) | TF2 Tree Inspection & RViz | Dynamic & static transforms, QoS, `view_frames`, RViz frames | CLI tools / [Lab 07](../01-module-53-tf2-kinematics/07-ros2-practical-lab.md) | Live frame graph PDF & RViz screenshot |
| [**Lab 02**](lab-02-coordinate-transformations.md) | Sensor Frame Transforms | Optical frames ($Z$-forward), pitch tilt, transformation into `map` | `sensor_transformer_node.py` | Python ROS 2 transformation node |
| [**Lab 03**](lab-03-forward-kinematics.md) | Forward Kinematics Implementation | Standard DH chaining, $SO(3)$ orthogonality, ground-truth poses | [`dh_transform.py`](../04-python-robotics/dh_transform.py) | Verified FK module & pytest report |
| [**Lab 04**](lab-04-inverse-kinematics.md) | Planar Inverse Kinematics | Law of Cosines, dual branches (elbow up/down), reachability | [`inverse_kinematics_2d.py`](../04-python-robotics/inverse_kinematics_2d.py) | Batch target evaluator & configuration plot |
| [**Lab 05**](lab-05-six-dof-arm.md) | 6-DOF Industrial Arm Decoupling | Pieper wrist center isolation, spherical wrist orientation | [`inverse_kinematics_6dof.py`](../04-python-robotics/inverse_kinematics_6dof.py) | Analytical 6-DOF solver & 3D render |
| [**Lab 06**](lab-06-jacobian-ik.md) | Jacobian IK & Singularities | SVD DLS Levenberg-Marquardt, $SO(3)$ log-map, joint limits | [`jacobian.py`](../04-python-robotics/jacobian.py) | Singularity-safe numerical IK solver |

---

## How to Work Through the Labs

1. **Pure Python Labs (Labs 03, 04, 05, 06):**
   - Can be run on any platform (Windows, macOS, Linux) with Python 3.10+.
   - Validate your implementations using `pytest 04-python-robotics/tests/ -v`.
2. **ROS 2 Practical Labs (Labs 01, 02):**
   - Require an active ROS 2 environment (ROS 2 Jazzy Jalisco on Ubuntu 24.04 or Humble on Ubuntu 22.04 / WSL2).
   - Use the [ROS 2 TF2 Cheat Sheet](../docs/reference/ros2-tf2-cheat-sheet.md) for quick command lookups.
