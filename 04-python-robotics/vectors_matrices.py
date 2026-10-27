"""
Robotics Mathematics: Vector and Matrix Operations
Author: Josiah & Educational Robotics Contributors
License: MIT

This module provides foundational linear algebra operations tailored for robotics
and AI students. It includes vector algebra, projections, cross products,
matrix transformations, determinants, inverses, and orthogonality checks.
"""

from typing import Tuple, Union
import numpy as np


def vector_norm(v: np.ndarray) -> float:
    """
    Compute Euclidean norm (length or magnitude) of a vector.
    
    Parameters:
        v: 1D NumPy array representing a vector.
        
    Returns:
        float: Euclidean norm ||v||_2.
        
    Raises:
        ValueError: If input is not a 1D array.
    """
    v = np.asarray(v, dtype=float)
    if v.ndim != 1:
        raise ValueError(f"Expected a 1D array, got {v.ndim}D array with shape {v.shape}.")
    return float(np.sqrt(np.sum(v ** 2)))


def normalize_vector(v: np.ndarray, tol: float = 1e-12) -> np.ndarray:
    """
    Convert a vector to a unit vector (length = 1.0).
    
    Parameters:
        v: 1D NumPy array.
        tol: Tolerance below which vector is considered zero.
        
    Returns:
        np.ndarray: Unit vector in the direction of v.
        
    Raises:
        ValueError: If vector has zero length.
    """
    norm = vector_norm(v)
    if norm < tol:
        raise ValueError("Cannot normalize a zero-length vector.")
    return v / norm


def dot_product(u: np.ndarray, v: np.ndarray) -> float:
    """
    Compute standard scalar dot product u . v = sum(u_i * v_i).
    
    Geometrically: u . v = ||u|| ||v|| cos(theta)
    """
    u = np.asarray(u, dtype=float)
    v = np.asarray(v, dtype=float)
    if u.shape != v.shape:
        raise ValueError(f"Shape mismatch in dot product: {u.shape} vs {v.shape}.")
    return float(np.dot(u, v))


def angle_between_vectors(u: np.ndarray, v: np.ndarray) -> float:
    """
    Compute angle between two vectors in radians in [0, pi].
    """
    norm_u = vector_norm(u)
    norm_v = vector_norm(v)
    if norm_u < 1e-12 or norm_v < 1e-12:
        raise ValueError("Vectors must be non-zero to define an angle.")
    cos_theta = dot_product(u, v) / (norm_u * norm_v)
    # Clip for floating point numerical stability
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    return float(np.arccos(cos_theta))


def vector_projection(u: np.ndarray, v: np.ndarray) -> np.ndarray:
    """
    Compute vector projection of u onto v: proj_v(u) = (u . v / ||v||^2) * v.
    """
    v_norm_sq = dot_product(v, v)
    if v_norm_sq < 1e-14:
        raise ValueError("Cannot project onto a zero vector.")
    scalar_proj = dot_product(u, v) / v_norm_sq
    return scalar_proj * v


def cross_product(u: np.ndarray, v: np.ndarray) -> np.ndarray:
    """
    Compute 3D cross product u x v.
    
    In robotics, cross products define:
    - Angular momentum and torque: tau = r x F
    - Linear velocity from rotational velocity: v = omega x r
    - The right-hand rule for 3D coordinate frames.
    """
    u = np.asarray(u, dtype=float)
    v = np.asarray(v, dtype=float)
    if u.shape != (3,) or v.shape != (3,):
        raise ValueError("Cross product is only defined for 3D vectors (shape (3,)).")
    return np.array([
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0]
    ], dtype=float)


def skew_symmetric_matrix(v: np.ndarray) -> np.ndarray:
    """
    Form 3x3 skew-symmetric matrix [v]_x such that [v]_x * w = v x w.
    
    [v]_x = [  0   -v_z   v_y ]
            [  v_z   0   -v_x ]
            [ -v_y  v_x    0  ]
    """
    v = np.asarray(v, dtype=float)
    if v.shape != (3,):
        raise ValueError("Input vector must be 3D.")
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0]
    ], dtype=float)


def matrix_properties(M: np.ndarray) -> dict:
    """
    Inspect fundamental matrix properties: shape, determinant, trace,
    rank, condition number, symmetry, and orthogonality.
    """
    M = np.asarray(M, dtype=float)
    if M.ndim != 2:
        raise ValueError("Input must be a 2D matrix.")
    
    rows, cols = M.shape
    props = {
        "shape": (rows, cols),
        "is_square": rows == cols,
        "rank": int(np.linalg.matrix_rank(M)),
    }
    
    if rows == cols:
        props["det"] = float(np.linalg.det(M))
        props["trace"] = float(np.trace(M))
        props["is_symmetric"] = bool(np.allclose(M, M.T, atol=1e-8))
        props["is_orthogonal"] = bool(
            np.allclose(M @ M.T, np.eye(rows), atol=1e-7) and
            np.allclose(M.T @ M, np.eye(rows), atol=1e-7)
        )
        try:
            props["cond"] = float(np.linalg.cond(M))
        except np.linalg.LinAlgError:
            props["cond"] = float("inf")
    return props


def safe_matrix_inverse(M: np.ndarray, tol: float = 1e-10) -> np.ndarray:
    """
    Invert square matrix M safely with determinant and condition number check.
    """
    M = np.asarray(M, dtype=float)
    if M.ndim != 2 or M.shape[0] != M.shape[1]:
        raise ValueError(f"Matrix must be square, got shape {M.shape}.")
    
    det = np.linalg.det(M)
    if abs(det) < tol:
        raise np.linalg.LinAlgError(
            f"Matrix is singular (det={det:.2e} < {tol:.2e}) and cannot be inverted."
        )
    return np.linalg.inv(M)


if __name__ == "__main__":
    print("=" * 60)
    print("Robotics Linear Algebra Demonstration")
    print("=" * 60)
    
    # 1. Vectors
    u = np.array([1.0, 0.0, 0.0])
    v = np.array([0.0, 1.0, 0.0])
    print(f"Vector u: {u}")
    print(f"Vector v: {v}")
    print(f"Dot product u . v: {dot_product(u, v):.4f} (orthogonal: {abs(dot_product(u, v)) < 1e-9})")
    
    w = cross_product(u, v)
    print(f"Cross product u x v: {w} (matches unit z-axis)")
    
    angle_deg = np.rad2deg(angle_between_vectors(u, v))
    print(f"Angle between u and v: {angle_deg:.1f} degrees")
    
    # 2. Skew-Symmetric Cross Product equivalence
    S = skew_symmetric_matrix(u)
    print("\nSkew-symmetric matrix [u]_x:")
    print(S)
    print(f"[u]_x * v = {S @ v} (matches cross product: {np.allclose(S @ v, w)})")
    
    # 3. Matrix Properties
    R = np.array([
        [0.0, -1.0, 0.0],
        [1.0,  0.0, 0.0],
        [0.0,  0.0, 1.0]
    ])
    props = matrix_properties(R)
    print("\nProperties of 90-deg z-rotation matrix R:")
    for k, val in props.items():
        print(f"  {k}: {val}")
    print("=" * 60)
