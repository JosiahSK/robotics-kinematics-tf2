"""
Robotic Manipulator Jacobian & Singularity Analysis
Author: Josiah & Educational Robotics Contributors
License: MIT

This module computes the geometric Jacobian matrix, numerical finite-difference
Jacobian, manipulability index, condition number, and damped least-squares pseudo-inverse
for serial robotic manipulators.
"""

from typing import Tuple
import numpy as np
from forward_kinematics_6dof import RobotArm6DOF
from rotation_matrices import rot_x, rot_y, rot_z


def geometric_jacobian_6dof(
    arm: RobotArm6DOF,
    joint_angles: np.ndarray
) -> np.ndarray:
    """
    Compute the 6x6 geometric Jacobian J(q) for the 6-DOF serial arm:
    
        v_e     [ J_v ]
    xi = [    ] = [     ] * q_dot
        omega_e [ J_w ]
        
    For revolute joint i (from 1 to 6):
        J_v,i = z_{i-1} x (p_e - p_{i-1})
        J_w,i = z_{i-1}
        
    where:
        z_{i-1} is the unit z-axis of frame i-1 expressed in base frame 0.
        p_{i-1} is the origin of frame i-1 expressed in base frame 0.
        p_e is the end-effector origin (frame 6).
    """
    q = np.asarray(joint_angles, dtype=float)
    if q.shape != (6,):
        raise ValueError(f"Expected 6 joint angles, got {q.shape}.")
        
    T_0_6, transforms = arm.forward_kinematics(q)
    p_e = T_0_6[:3, 3]
    
    J = np.zeros((6, 6), dtype=float)
    
    for i in range(6):
        T_i_minus_1 = transforms[i]
        # z-axis is the 3rd column of the rotation matrix of frame i-1
        z_prev = T_i_minus_1[:3, 2]
        p_prev = T_i_minus_1[:3, 3]
        
        # Linear velocity component: z_{i-1} x (p_e - p_{i-1})
        J_v = np.cross(z_prev, (p_e - p_prev))
        # Angular velocity component: z_{i-1}
        J_w = z_prev
        
        J[:3, i] = J_v
        J[3:, i] = J_w
        
    return J


def numerical_jacobian_6dof(
    arm: RobotArm6DOF,
    joint_angles: np.ndarray,
    delta_q: float = 1e-6
) -> np.ndarray:
    """
    Compute numerical Jacobian via central finite differences for validation.
    Translational rows computed from position difference;
    Rotational rows computed from small rotation difference.
    """
    q = np.asarray(joint_angles, dtype=float)
    J_num = np.zeros((6, 6), dtype=float)
    
    for i in range(6):
        q_plus = q.copy()
        q_minus = q.copy()
        q_plus[i] += delta_q
        q_minus[i] -= delta_q
        
        T_plus, _ = arm.forward_kinematics(q_plus)
        T_minus, _ = arm.forward_kinematics(q_minus)
        
        # Linear velocity difference
        p_diff = (T_plus[:3, 3] - T_minus[:3, 3]) / (2.0 * delta_q)
        J_num[:3, i] = p_diff
        
        # Angular velocity difference: R_diff = R_plus @ R_minus^T
        # For small delta_theta, R_diff approx I + [omega * 2 * delta_q]_x
        R_plus = T_plus[:3, :3]
        R_minus = T_minus[:3, :3]
        R_rel = R_plus @ R_minus.T
        omega = np.array([
            R_rel[2, 1] - R_rel[1, 2],
            R_rel[0, 2] - R_rel[2, 0],
            R_rel[1, 0] - R_rel[0, 1]
        ]) / (4.0 * delta_q)
        J_num[3:, i] = omega
        
    return J_num


def manipulability_index(J: np.ndarray) -> float:
    """
    Compute Yoshikawa's manipulability measure:
        w = sqrt(det(J * J^T))
    Indicates distance to singularities (w -> 0 near singularities).
    """
    JJT = J @ J.T
    det = np.linalg.det(JJT)
    return float(np.sqrt(max(0.0, det)))


def damped_least_squares_inverse(
    J: np.ndarray,
    damping: float = 0.05
) -> np.ndarray:
    """
    Compute the Levenberg-Marquardt Damped Least-Squares (DLS) pseudoinverse using SVD:
        J_dls = V * diag(sigma_i / (sigma_i^2 + lambda^2)) * U^T
        
    This is numerically more stable than the explicit formula J^T(JJ^T + lambda^2*I)^{-1}
    because it avoids forming JJ^T and avoids explicit matrix inversion.
    
    Parameters:
        J: m x n Jacobian matrix.
        damping: Damping factor lambda. Prevents infinite joint velocities near singularities.
        
    Returns:
        n x m DLS pseudoinverse.
    """
    U, sigma, Vt = np.linalg.svd(J, full_matrices=False)
    # Damped singular values: sigma_i / (sigma_i^2 + lambda^2)
    sigma_dls = sigma / (sigma ** 2 + damping ** 2)
    return Vt.T @ np.diag(sigma_dls) @ U.T


if __name__ == "__main__":
    print("=" * 60)
    print("Jacobian and Singularity Analysis Verification")
    print("=" * 60)
    
    arm = RobotArm6DOF()
    q_test = np.radians([20.0, 35.0, -25.0, 15.0, -30.0, 10.0])
    
    J_geom = geometric_jacobian_6dof(arm, q_test)
    J_num = numerical_jacobian_6dof(arm, q_test)
    
    print("Geometric Jacobian (top-left 3x3 translational block):")
    print(np.round(J_geom[:3, :3], 4))
    
    diff = np.max(np.abs(J_geom - J_num))
    print(f"\nMax difference between Geometric and Numerical Jacobian: {diff:.2e}")
    print(f"Jacobian formulations match? {diff < 1e-5}")
    
    w = manipulability_index(J_geom)
    cond = np.linalg.cond(J_geom)
    print(f"\nYoshikawa Manipulability Measure w: {w:.6f}")
    print(f"Condition Number kappa(J): {cond:.2f}")
    
    J_dls = damped_least_squares_inverse(J_geom, damping=0.05)
    print(f"DLS Inverse computed successfully. Shape: {J_dls.shape}")
    print("=" * 60)
