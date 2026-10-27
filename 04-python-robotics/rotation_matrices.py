"""
Robotics Mathematics: 2D and 3D Rotation Representations
Author: Josiah & Educational Robotics Contributors
License: MIT

This module provides verified transformations between:
1. 2D rotation matrices SO(2).
2. 3D elementary rotation matrices Rx, Ry, Rz in SO(3).
3. Roll-Pitch-Yaw (RPY / Z-Y-X Euler angles).
4. Axis-Angle representation via Rodrigues' formula.
5. Unit Quaternions (Hamilton convention: [x, y, z, w] for ROS 2 compatibility).
"""

from typing import Tuple, Union
import numpy as np


def is_rotation_matrix(R: np.ndarray, tol: float = 1e-6) -> bool:
    """
    Validate that R belongs to the Special Orthogonal group SO(3):
    1. R is 3x3 real matrix.
    2. R^T * R = I (orthonormal columns and rows).
    3. det(R) = +1 (proper rotation, preserves chirality / right-handedness).
    """
    R = np.asarray(R, dtype=float)
    if R.shape != (3, 3):
        return False
    is_orthogonal = np.allclose(R.T @ R, np.eye(3), atol=tol)
    det_is_one = np.isclose(np.linalg.det(R), 1.0, atol=tol)
    return bool(is_orthogonal and det_is_one)


def rot2d(theta_rad: float) -> np.ndarray:
    """
    Generate 2D rotation matrix SO(2) for counter-clockwise angle theta in radians.
    
    R(theta) = [ cos(theta)  -sin(theta) ]
               [ sin(theta)   cos(theta) ]
    """
    c = np.cos(theta_rad)
    s = np.sin(theta_rad)
    return np.array([
        [c, -s],
        [s,  c]
    ], dtype=float)


def rot_x(phi_rad: float) -> np.ndarray:
    """
    Elementary 3D rotation matrix about the X-axis by phi radians (Roll).
    
    R_x(phi) = [ 1     0         0     ]
               [ 0  cos(phi) -sin(phi) ]
               [ 0  sin(phi)  cos(phi) ]
    """
    c = np.cos(phi_rad)
    s = np.sin(phi_rad)
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0,   c,  -s],
        [0.0,   s,   c]
    ], dtype=float)


def rot_y(theta_rad: float) -> np.ndarray:
    """
    Elementary 3D rotation matrix about the Y-axis by theta radians (Pitch).
    
    R_y(theta) = [  cos(theta)  0  sin(theta) ]
                 [       0      1       0      ]
                 [ -sin(theta)  0  cos(theta) ]
    """
    c = np.cos(theta_rad)
    s = np.sin(theta_rad)
    return np.array([
        [ c, 0.0,  s],
        [0.0, 1.0, 0.0],
        [-s, 0.0,  c]
    ], dtype=float)


def rot_z(psi_rad: float) -> np.ndarray:
    """
    Elementary 3D rotation matrix about the Z-axis by psi radians (Yaw).
    
    R_z(psi) = [ cos(psi) -sin(psi)  0 ]
               [ sin(psi)  cos(psi)  0 ]
               [    0         0      1 ]
    """
    c = np.cos(psi_rad)
    s = np.sin(psi_rad)
    return np.array([
        [  c,  -s, 0.0],
        [  s,   c, 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=float)


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert Roll-Pitch-Yaw (extrinsic X-Y-Z or intrinsic Z-Y-X Euler angles)
    to a 3x3 rotation matrix:
        R = R_z(yaw) @ R_y(pitch) @ R_x(roll)
        
    Parameters:
        roll: rotation about X-axis in radians.
        pitch: rotation about Y-axis in radians.
        yaw: rotation about Z-axis in radians.
    """
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def matrix_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    """
    Extract Roll, Pitch, Yaw from a 3x3 rotation matrix R:
        R = R_z(yaw) @ R_y(pitch) @ R_x(roll)
    
    Handles the pitch gimbal lock singularity at pitch = +/- pi/2.
    
    Returns:
        Tuple[float, float, float]: (roll, pitch, yaw) in radians.
    """
    if not is_rotation_matrix(R):
        raise ValueError("Input matrix is not a valid SO(3) rotation matrix.")
    
    # R[2, 0] = -sin(pitch)
    pitch = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    
    # Check for gimbal lock (pitch near +/- 90 degrees)
    if np.isclose(np.cos(pitch), 0.0, atol=1e-6):
        # Gimbal lock: roll and yaw are coupled. Choose roll = 0
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
    else:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
        
    return float(roll), float(pitch), float(yaw)


def axis_angle_to_matrix(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    """
    Compute rotation matrix using Rodrigues' rotation formula:
        R = I + sin(theta) * [k]_x + (1 - cos(theta)) * [k]_x^2
    where k is a 3D unit axis vector.
    """
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        return np.eye(3)
    k = axis / norm
    
    K = np.array([
        [0.0, -k[2], k[1]],
        [k[2], 0.0, -k[0]],
        [-k[1], k[0], 0.0]
    ])
    
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.eye(3) + s * K + (1.0 - c) * (K @ K)


def matrix_to_axis_angle(R: np.ndarray) -> Tuple[np.ndarray, float]:
    """
    Extract unit rotation axis and angle (radians in [0, pi]) from SO(3) matrix.
    """
    if not is_rotation_matrix(R):
        raise ValueError("Input matrix must be a valid SO(3) rotation matrix.")
        
    trace = np.trace(R)
    cos_theta = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
    theta = float(np.arccos(cos_theta))
    
    if np.isclose(theta, 0.0, atol=1e-7):
        return np.array([1.0, 0.0, 0.0]), 0.0
    elif np.isclose(theta, np.pi, atol=1e-5):
        # 180 degree rotation: R = 2 * k * k^T - I
        diag = np.diagonal(R)
        idx = int(np.argmax(diag))
        if idx == 0:
            kx = np.sqrt(max(0.0, (R[0, 0] + 1) / 2))
            ky = (R[0, 1]) / (2 * kx) if kx > 1e-6 else 0.0
            kz = (R[0, 2]) / (2 * kx) if kx > 1e-6 else 0.0
        elif idx == 1:
            ky = np.sqrt(max(0.0, (R[1, 1] + 1) / 2))
            kx = (R[0, 1]) / (2 * ky) if ky > 1e-6 else 0.0
            kz = (R[1, 2]) / (2 * ky) if ky > 1e-6 else 0.0
        else:
            kz = np.sqrt(max(0.0, (R[2, 2] + 1) / 2))
            kx = (R[0, 2]) / (2 * kz) if kz > 1e-6 else 0.0
            ky = (R[1, 2]) / (2 * kz) if kz > 1e-6 else 0.0
        axis = np.array([kx, ky, kz])
        return axis / np.linalg.norm(axis), float(np.pi)
    else:
        axis = np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1]
        ]) / (2.0 * np.sin(theta))
        return axis / np.linalg.norm(axis), theta


def quaternion_to_matrix(q: np.ndarray) -> np.ndarray:
    """
    Convert unit quaternion q = [x, y, z, w] (ROS 2 convention) to 3x3 rotation matrix.
    """
    q = np.asarray(q, dtype=float)
    if q.shape != (4,):
        raise ValueError("Quaternion must have 4 components [x, y, z, w].")
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError("Quaternion cannot have zero norm.")
    x, y, z, w = q / norm
    
    return np.array([
        [1.0 - 2.0 * (y**2 + z**2), 2.0 * (x*y - z*w),       2.0 * (x*z + y*w)],
        [2.0 * (x*y + z*w),       1.0 - 2.0 * (x**2 + z**2), 2.0 * (y*z - x*w)],
        [2.0 * (x*z - y*w),       2.0 * (y*z + x*w),       1.0 - 2.0 * (x**2 + y**2)]
    ], dtype=float)


def matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """
    Convert 3x3 rotation matrix to unit quaternion q = [x, y, z, w] (ROS 2 convention).
    Uses Shepperd's algorithm for numerical stability across all angles.
    """
    if not is_rotation_matrix(R):
        raise ValueError("Input must be a valid SO(3) rotation matrix.")
        
    trace = np.trace(R)
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
        
    q = np.array([x, y, z, w], dtype=float)
    return q / np.linalg.norm(q)


if __name__ == "__main__":
    print("=" * 60)
    print("3D Rotation Matrices & Parameterizations Demonstration")
    print("=" * 60)
    
    roll, pitch, yaw = np.radians([30.0, 45.0, 60.0])
    R = rpy_to_matrix(roll, pitch, yaw)
    print(f"Original RPY (deg): [30, 45, 60]")
    print(f"Generated Rotation Matrix:\n{np.round(R, 4)}")
    print(f"Is valid SO(3)? {is_rotation_matrix(R)}")
    
    recovered_rpy = np.degrees(matrix_to_rpy(R))
    print(f"Recovered RPY (deg): {np.round(recovered_rpy, 2)}")
    
    q = matrix_to_quaternion(R)
    print(f"Equivalent ROS 2 Quaternion [x, y, z, w]: {np.round(q, 4)}")
    R_from_q = quaternion_to_matrix(q)
    print(f"Quaternion recovery matches matrix? {np.allclose(R, R_from_q)}")
    
    axis, angle = matrix_to_axis_angle(R)
    print(f"Axis-Angle representation: Axis = {np.round(axis, 4)}, Angle = {np.degrees(angle):.2f} deg")
    print("=" * 60)
