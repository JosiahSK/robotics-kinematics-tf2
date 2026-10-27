import sys
import os
import pytest
import numpy as np

# Ensure 04-python-robotics is in sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from rotation_matrices import (
    is_rotation_matrix,
    rot2d,
    rot_x,
    rot_y,
    rot_z,
    rpy_to_matrix,
    matrix_to_rpy,
    axis_angle_to_matrix,
    matrix_to_axis_angle,
    quaternion_to_matrix,
    matrix_to_quaternion
)


def test_elementary_rotations_properties():
    angles = [0.0, np.pi / 6, np.pi / 4, np.pi / 2, np.pi, -np.pi / 3]
    for theta in angles:
        for rot_fn in [rot_x, rot_y, rot_z]:
            R = rot_fn(theta)
            assert is_rotation_matrix(R), f"Matrix from {rot_fn.__name__} failed SO(3) check."
            assert np.isclose(np.linalg.det(R), 1.0, atol=1e-7)
            assert np.allclose(R @ R.T, np.eye(3), atol=1e-7)


def test_rpy_conversion_roundtrip():
    test_cases = [
        (0.0, 0.0, 0.0),
        (0.2, -0.3, 0.5),
        (np.pi / 3, np.pi / 6, -np.pi / 4),
        (-0.5, 0.4, 1.2)
    ]
    for roll, pitch, yaw in test_cases:
        R = rpy_to_matrix(roll, pitch, yaw)
        rec_r, rec_p, rec_y = matrix_to_rpy(R)
        assert np.isclose(roll, rec_r, atol=1e-5)
        assert np.isclose(pitch, rec_p, atol=1e-5)
        assert np.isclose(yaw, rec_y, atol=1e-5)


def test_quaternion_roundtrip():
    # Random rotations
    np.random.seed(42)
    for _ in range(10):
        rpy = np.random.uniform(-np.pi + 0.1, np.pi - 0.1, 3)
        R_orig = rpy_to_matrix(*rpy)
        q = matrix_to_quaternion(R_orig)
        
        # Must be unit norm
        assert np.isclose(np.linalg.norm(q), 1.0, atol=1e-7)
        
        R_rec = quaternion_to_matrix(q)
        assert np.allclose(R_orig, R_rec, atol=1e-5)


def test_axis_angle_rodrigues():
    axis = np.array([1.0, 1.0, 1.0]) / np.sqrt(3.0)
    angle = np.pi / 3.0
    R = axis_angle_to_matrix(axis, angle)
    assert is_rotation_matrix(R)
    
    rec_axis, rec_angle = matrix_to_axis_angle(R)
    assert np.isclose(angle, rec_angle, atol=1e-5)
    assert np.allclose(axis, rec_axis, atol=1e-5)


def test_invalid_rotation_matrix_detection():
    # Non-orthogonal matrix
    bad_M = np.array([
        [1.0, 0.5, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
    ])
    assert not is_rotation_matrix(bad_M)
    
    # Reflection matrix (det = -1)
    reflection = np.diag([1.0, 1.0, -1.0])
    assert not is_rotation_matrix(reflection)
