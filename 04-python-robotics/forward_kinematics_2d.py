"""
Planar Manipulator Forward Kinematics (2-Link and 3-Link)
Author: Josiah & Educational Robotics Contributors
License: MIT

This module computes the forward kinematics for 2-link and 3-link planar
robotic arms using both direct trigonometry and standard DH matrices.
"""

from typing import List, Tuple, Union
import numpy as np
from dh_transform import dh_matrix


def fk_2link_planar(
    l1: float,
    l2: float,
    theta1_rad: float,
    theta2_rad: float
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute forward kinematics of a planar 2R robot arm.
    
    Equations:
        x_1 = l1 * cos(theta1)
        y_1 = l1 * sin(theta1)
        
        x_2 = x_1 + l2 * cos(theta1 + theta2)
        y_2 = y_1 + l2 * sin(theta1 + theta2)
        
    Parameters:
        l1, l2: Link lengths in metres.
        theta1_rad, theta2_rad: Joint angles in radians.
        
    Returns:
        Tuple[end_effector_pose, joint_positions]:
            - end_effector_pose: np.array([x, y, phi]) where phi is end-effector orientation.
            - joint_positions: np.array of shape (3, 2) containing coordinates [[x0, y0], [x1, y1], [x2, y2]].
    """
    if l1 <= 0.0 or l2 <= 0.0:
        raise ValueError("Link lengths must be strictly positive.")
        
    x0, y0 = 0.0, 0.0
    x1 = l1 * np.cos(theta1_rad)
    y1 = l1 * np.sin(theta1_rad)
    
    phi = theta1_rad + theta2_rad
    x2 = x1 + l2 * np.cos(phi)
    y2 = y1 + l2 * np.sin(phi)
    
    ee_pose = np.array([x2, y2, phi], dtype=float)
    joint_coords = np.array([
        [x0, y0],
        [x1, y1],
        [x2, y2]
    ], dtype=float)
    
    return ee_pose, joint_coords


def fk_2link_dh(
    l1: float,
    l2: float,
    theta1_rad: float,
    theta2_rad: float
) -> np.ndarray:
    """
    Compute 2-link planar forward kinematics via standard DH matrices:
        A1: theta=theta1, d=0, a=l1, alpha=0
        A2: theta=theta2, d=0, a=l2, alpha=0
    T_0^2 = A1 @ A2
    """
    A1 = dh_matrix(theta=theta1_rad, d=0.0, a=l1, alpha=0.0)
    A2 = dh_matrix(theta=theta2_rad, d=0.0, a=l2, alpha=0.0)
    return A1 @ A2


def fk_3link_planar(
    l1: float,
    l2: float,
    l3: float,
    theta1_rad: float,
    theta2_rad: float,
    theta3_rad: float
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute forward kinematics for a 3R planar manipulator.
    
    Returns:
        Tuple[np.array([x, y, phi]), joint_positions (4, 2)]
    """
    if l1 <= 0.0 or l2 <= 0.0 or l3 <= 0.0:
        raise ValueError("Link lengths must be positive.")
        
    x0, y0 = 0.0, 0.0
    x1 = l1 * np.cos(theta1_rad)
    y1 = l1 * np.sin(theta1_rad)
    
    phi12 = theta1_rad + theta2_rad
    x2 = x1 + l2 * np.cos(phi12)
    y2 = y1 + l2 * np.sin(phi12)
    
    phi = phi12 + theta3_rad
    x3 = x2 + l3 * np.cos(phi)
    y3 = y2 + l3 * np.sin(phi)
    
    ee_pose = np.array([x3, y3, phi], dtype=float)
    joint_coords = np.array([
        [x0, y0],
        [x1, y1],
        [x2, y2],
        [x3, y3]
    ], dtype=float)
    return ee_pose, joint_coords


if __name__ == "__main__":
    print("=" * 60)
    print("2-Link Planar Forward Kinematics Verification")
    print("=" * 60)
    
    l1, l2 = 1.0, 0.8
    th1, th2 = np.deg2rad(45.0), np.deg2rad(30.0)
    
    ee_pose, joints = fk_2link_planar(l1, l2, th1, th2)
    print(f"Links: l1={l1} m, l2={l2} m")
    print(f"Joint angles: th1=45 deg, th2=30 deg")
    print(f"End-Effector Pose: x={ee_pose[0]:.4f} m, y={ee_pose[1]:.4f} m, phi={np.rad2deg(ee_pose[2]):.2f} deg")
    
    # DH Comparison
    T = fk_2link_dh(l1, l2, th1, th2)
    print(f"\nDH Transformation Matrix:\n{np.round(T, 4)}")
    print(f"DH End-Effector x, y: ({T[0, 3]:.4f}, {T[1, 3]:.4f})")
    print(f"Trigonometric matches DH? {np.allclose(ee_pose[:2], T[:2, 3])}")
    print("=" * 60)
