"""
Standard Denavit-Hartenberg (DH) Transformation Module
Author: Josiah & Educational Robotics Contributors
License: MIT

This module implements the standard Denavit-Hartenberg (DH) matrix formulation
for serial robotic manipulators, supporting chaining, extraction of position
and orientation, frame transforms, and parameter conversions.
"""

from typing import List, Tuple, Union, Optional
import numpy as np


class DHParameter:
    """
    Encapsulates standard Denavit-Hartenberg parameters for a single joint:
    
    Parameters:
        theta: Joint angle (rotation about z_{i-1}). Variable for revolute joints.
        d: Link offset (translation along z_{i-1}). Variable for prismatic joints.
        a: Link length (translation along x_i). Perpendicular distance between z_{i-1} and z_i.
        alpha: Link twist (rotation about x_i). Angle from z_{i-1} to z_i measured about x_i.
        is_variable_theta: True if revolute joint (default True).
    """
    def __init__(
        self,
        theta: float = 0.0,
        d: float = 0.0,
        a: float = 0.0,
        alpha: float = 0.0,
        is_variable_theta: bool = True
    ):
        self.theta = float(theta)
        self.d = float(d)
        self.a = float(a)
        self.alpha = float(alpha)
        self.is_variable_theta = is_variable_theta

    def __repr__(self) -> str:
        return (f"DHParameter(theta={self.theta:.4f} rad, d={self.d:.4f} m, "
                f"a={self.a:.4f} m, alpha={self.alpha:.4f} rad)")


def dh_matrix(
    theta: float,
    d: float,
    a: float,
    alpha: float,
    in_degrees: bool = False
) -> np.ndarray:
    """
    Compute 4x4 standard Denavit-Hartenberg homogeneous transformation matrix A_i:
    
    A_i = Rot(z, theta) * Trans(z, d) * Trans(x, a) * Rot(x, alpha)
    
        [ cos(theta)  -sin(theta)*cos(alpha)   sin(theta)*sin(alpha)  a*cos(theta) ]
    =   [ sin(theta)   cos(theta)*cos(alpha)  -cos(theta)*sin(alpha)  a*sin(theta) ]
        [     0               sin(alpha)              cos(alpha)           d       ]
        [     0                   0                       0                1       ]
        
    Parameters:
        theta: Rotation angle about z_{i-1}.
        d: Translation distance along z_{i-1}.
        a: Translation distance along x_i (link length).
        alpha: Rotation angle about x_i (link twist).
        in_degrees: If True, theta and alpha are converted from degrees to radians.
        
    Returns:
        np.ndarray: 4x4 homogeneous transformation matrix SE(3).
    """
    if in_degrees:
        theta = np.deg2rad(theta)
        alpha = np.deg2rad(alpha)
        
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    A = np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0]
    ], dtype=float)
    
    # Zero out tiny floating point residuals (e.g. 1e-16 from sin(pi))
    A[np.abs(A) < 1e-15] = 0.0
    return A


def extract_pose(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Extract 3D position vector and 3x3 rotation matrix from 4x4 SE(3) matrix.
    
    Parameters:
        T: 4x4 homogeneous transformation matrix.
        
    Returns:
        Tuple[position (3,), rotation_matrix (3,3)]
    """
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4):
        raise ValueError(f"Expected 4x4 matrix, got {T.shape}.")
    position = T[:3, 3].copy()
    rotation = T[:3, :3].copy()
    return position, rotation


def chain_dh_matrices(
    dh_table: List[DHParameter],
    joint_angles: Optional[List[float]] = None,
    in_degrees: bool = False
) -> Tuple[np.ndarray, List[np.ndarray]]:
    """
    Chain successive DH transformation matrices from base frame (frame 0) to end frame.
    
    Parameters:
        dh_table: List of DHParameter instances for each link i=1..N.
        joint_angles: Optional list of joint values overriding the theta/d variables.
        in_degrees: If True, joint_angles are treated as degrees.
        
    Returns:
        Tuple[T_total, cumulative_transforms]:
            - T_total: 4x4 transform T_0^N.
            - cumulative_transforms: List of intermediate transforms [T_0^0, T_0^1, ..., T_0^N].
    """
    num_joints = len(dh_table)
    if joint_angles is not None:
        if len(joint_angles) != num_joints:
            raise ValueError(f"Expected {num_joints} joint angles, got {len(joint_angles)}.")
            
    T_current = np.eye(4, dtype=float)
    cumulative_transforms = [T_current.copy()]
    
    for i, param in enumerate(dh_table):
        if joint_angles is not None:
            val = np.deg2rad(joint_angles[i]) if in_degrees else joint_angles[i]
            theta = val if param.is_variable_theta else param.theta
            d = param.d if param.is_variable_theta else val
        else:
            theta = param.theta
            d = param.d
            
        A_i = dh_matrix(theta=theta, d=d, a=param.a, alpha=param.alpha, in_degrees=False)
        T_current = T_current @ A_i
        cumulative_transforms.append(T_current.copy())
        
    return T_current, cumulative_transforms


def create_homogeneous_matrix(
    rotation: np.ndarray,
    translation: np.ndarray
) -> np.ndarray:
    """
    Construct 4x4 transformation matrix from 3x3 rotation and 3D translation vector.
    """
    R = np.asarray(rotation, dtype=float)
    p = np.asarray(translation, dtype=float).flatten()
    if R.shape != (3, 3) or p.shape != (3,):
        raise ValueError("Invalid shapes: R must be (3, 3), translation must be (3,).")
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def invert_homogeneous_matrix(T: np.ndarray) -> np.ndarray:
    """
    Analytically invert a rigid-body homogeneous transformation matrix:
        T = [ R  p ]  =>  T^-1 = [ R^T  -R^T * p ]
            [ 0  1 ]             [  0        1    ]
    Computationally faster and numerically more stable than np.linalg.inv.
    """
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4):
        raise ValueError("Matrix must be 4x4.")
    R = T[:3, :3]
    p = T[:3, 3]
    R_inv = R.T
    p_inv = -R_inv @ p
    
    T_inv = np.eye(4, dtype=float)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = p_inv
    return T_inv


if __name__ == "__main__":
    print("=" * 60)
    print("Standard DH Transformation Verification")
    print("=" * 60)
    
    # Example: Simple single link with length 0.5m rotated by 90 degrees
    A = dh_matrix(theta=90.0, d=0.2, a=0.5, alpha=0.0, in_degrees=True)
    print("Transformation A (theta=90 deg, d=0.2 m, a=0.5 m, alpha=0):")
    print(np.round(A, 4))
    
    pos, rot = extract_pose(A)
    print(f"\nExtracted Position: {pos}")
    print(f"Extracted Rotation Matrix:\n{np.round(rot, 4)}")
    print(f"det(R) = {np.linalg.det(rot):.6f}")
    
    A_inv = invert_homogeneous_matrix(A)
    print("\nInverted Transformation A^-1:")
    print(np.round(A_inv, 4))
    print(f"A * A^-1 is Identity? {np.allclose(A @ A_inv, np.eye(4))}")
    print("=" * 60)
