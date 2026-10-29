# Python Robotics Mathematics & Kinematics Engine

This directory contains executable, tested, and mathematically verified Python implementations of robotics mathematics, Denavit-Hartenberg (DH) forward kinematics, analytical and numerical inverse kinematics (IK), the geometric Jacobian, and 3D visualization.

---

## Directory Structure

| File | Description | Key Functions / Classes |
| :--- | :--- | :--- |
| [`vectors_matrices.py`](vectors_matrices.py) | Vector & matrix linear algebra | `dot_product`, `cross_product`, `skew_symmetric_matrix`, `matrix_properties` |
| [`rotation_matrices.py`](rotation_matrices.py) | SO(2) and SO(3) rotations, RPY, Quaternions, Rodrigues | `rot2d`, `rpy_to_matrix`, `quaternion_to_matrix`, `matrix_to_axis_angle` |
| [`dh_transform.py`](dh_transform.py) | Standard Denavit-Hartenberg matrices | `dh_matrix`, `chain_dh_matrices`, `invert_homogeneous_matrix` |
| [`forward_kinematics_2d.py`](forward_kinematics_2d.py) | 2-Link and 3-Link planar forward kinematics | `fk_2link_planar`, `fk_2link_dh`, `fk_3link_planar` |
| [`inverse_kinematics_2d.py`](inverse_kinematics_2d.py) | Analytical planar inverse kinematics | `ik_2link_planar` (elbow-up & down), `ik_3link_planar` |
| [`forward_kinematics_6dof.py`](forward_kinematics_6dof.py) | 6-DOF articulated serial robot forward kinematics | `RobotArm6DOF`, `forward_kinematics`, `urdf_to_dh_joints`, `dh_to_urdf_joints` |
| [`inverse_kinematics_6dof.py`](inverse_kinematics_6dof.py) | Analytical (wrist decoupling) & Numerical (DLS) IK | `analytical_ik_spherical_wrist`, `NumericalIKSolver6DOF` |
| [`jacobian.py`](jacobian.py) | Geometric Jacobian, finite-difference validation, SVD | `geometric_jacobian_6dof`, `damped_least_squares_inverse`, `manipulability_index` |
| [`visualise_robot_arm.py`](visualise_robot_arm.py) | 3D Matplotlib visualizer for robot arm | `visualize_robot_arm`, `plot_coordinate_frame` |
| [`tests/`](tests/) | Complete automated test suite using `pytest` | 39 comprehensive unit tests covering all core modules |

---

## Installation & Setup

1. Create a Python 3.10+ virtual environment:
   ```bash
   python -m venv venv
   # On Linux/macOS:
   source venv/bin/activate
   # On Windows PowerShell:
   .\venv\Scripts\Activate.ps1
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

---

## Running the Automated Test Suite

All algorithms are covered by unit tests validating algebraic identities, rotation orthogonality, boundary singularities, and kinematics convergence:

```bash
pytest -v tests/
```

Expected output:
```text
============================= 39 passed in ~1.0s ==============================
```

---

## Running Individual Examples

### 1. Linear Algebra & Skew-Symmetric Cross Product
```bash
python vectors_matrices.py
```

### 2. 3D Rotation Representations (RPY, Axis-Angle, Quaternions)
```bash
python rotation_matrices.py
```

### 3. Denavit-Hartenberg Homogeneous Matrix
```bash
python dh_transform.py
```

### 4. 2-Link Planar Forward & Inverse Kinematics
```bash
python forward_kinematics_2d.py
python inverse_kinematics_2d.py
```

### 5. 6-DOF Forward & Inverse Kinematics
```bash
python forward_kinematics_6dof.py
python inverse_kinematics_6dof.py
```

### 6. Manipulator Jacobian & Singularity Analysis
```bash
python jacobian.py
```

### 7. 3D Visualization of the 6-DOF Robot Arm
View the robot in any custom configuration:
```bash
# Interactive 3D plot
python visualise_robot_arm.py --angles 30 45 -30 20 -45 15

# Headless / Save to image:
python visualise_robot_arm.py --angles 0 30 -60 0 45 0 --no-show --save robot_pose.png
```
